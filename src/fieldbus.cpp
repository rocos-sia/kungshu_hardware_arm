//
// Created by think on 8/15/25.
//

#include "kungshu_hardware_arm/fieldbus.h"

#include <spdlog/spdlog.h>

#include <cinttypes>

using namespace KSH;

bool Fieldbus::Start() {
  is_initialized_ = false;

  ecx_contextt *context;
  ec_groupt *grp;
  ec_slavet *slave;

  spdlog::info("Starting Fieldbus on interface: {}", ifname_);

  spdlog::info("Initializing fieldbus...");
  if (!ecx_init(&ctx_, ifname_.c_str()) > 0) {
    spdlog::error("ecx_init failed. no socket connection on interface: {}",
                  ifname_);
    return false;
  }
  spdlog::info("Done.");

  if (!ecx_config_init(&ctx_)) {
    spdlog::error("ecx_config_init failed.");
    return false;
  }

  spdlog::info("{} slaves found and configured.", ctx_.slavecount);

  for (int i = 1; i <= ctx_.slavecount; i++) {
    ec_slavet *slave = &ctx_.slavelist[i];
    slave->PO2SOconfig = slave_setup;
  }

  spdlog::info("Sequential mapping of I/O... ");
  ecx_config_map_group(&ctx_, IOmap_, group_);
  spdlog::info("mapped {}O+{}I bytes from {} segments",
               ctx_.grouplist[group_].Obytes, ctx_.grouplist[group_].Ibytes,
               ctx_.grouplist[group_].nsegments);
  if (ctx_.grouplist[group_].nsegments > 1) {
    /* Show how slaves are distributed */
    for (int i = 0; i < ctx_.grouplist[group_].nsegments; ++i) {
      spdlog::info("{}{}", i == 0 ? " (" : "+",
                   ctx_.grouplist[group_].IOsegment[i]);
    }
    spdlog::info(" slaves)");
  }

  spdlog::info("Configuring distributed clock... ");
  ecx_configdc(&ctx_);
  spdlog::info("done");

  spdlog::info("Waiting for all slaves in safe operational... ");
  ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  spdlog::info("done");

  spdlog::info("Send a roundtrip to make outputs in slaves happy... ");
  // fieldbus_roundtrip(fieldbus);
  spdlog::info("done\n");

  PrintSlaveInfo(true);

  spdlog::info("Setting operational state..");
  /* Act on slave 0 (a virtual slave used for broadcasting) */
  slave = ctx_.slavelist;
  slave->state = EC_STATE_OPERATIONAL;
  ecx_writestate(&ctx_, 0);
  /* Poll the result ten times before giving up */
  for (int i = 0; i < 10; ++i) {
    spdlog::info(".");
    // fieldbus_roundtrip(fieldbus);
    ecx_statecheck(&ctx_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE / 10);
    if (slave->state == EC_STATE_OPERATIONAL) {
      spdlog::info(" all slaves are now operational\n");

      fieldbus_thread_ = std::thread([this]() {
        while (true) {
          Roundtrip();
          std::this_thread::sleep_for(std::chrono::microseconds(4000));
        }
      });

      return true;
    }
  }

  spdlog::info(" failed,");
  ecx_readstate(&ctx_);
  for (int i = 1; i <= ctx_.slavecount; ++i) {
    slave = ctx_.slavelist + i;
    if (slave->state != EC_STATE_OPERATIONAL) {
      printf(" slave %d is 0x%04X (AL-status=0x%04X %s)", i, slave->state,
             slave->ALstatuscode, ec_ALstatuscode2string(slave->ALstatuscode));
    }
  }

  return false;
}

bool Fieldbus::Close() { return true; }

int Fieldbus::Roundtrip() {
  ec_timet start, end, diff;
  int wkc;

  start = osal_current_time();
  ecx_send_processdata(&ctx_);
  wkc = ecx_receive_processdata(&ctx_, EC_TIMEOUTRET);
  end = osal_current_time();
  osal_time_diff(&start, &end, &diff);
  roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

  if (roundtrip_time_ < min_time_)
    min_time_ = roundtrip_time_;
  else if (roundtrip_time_ > max_time_)
    max_time_ = roundtrip_time_;

  memcpy(inputs_, ctx_.slavelist[0].inputs, ctx_.slavelist[0].Ibytes);
  memcpy(outputs_, ctx_.slavelist[0].outputs, ctx_.slavelist[0].Obytes);

  return wkc;
}

void Fieldbus::PrintSlaveInfo(bool printMAP, bool printSDO) {
  int cnt, i, j, nSM;
  uint16 ssigen;
  int expectedWKC;

  printf("Starting slaveinfo\n");

  ec_groupt *group = &ctx_.grouplist[0];

  while (ctx_.ecaterror) printf("%s", ecx_elist2string(&ctx_));
  printf("%d slaves found and configured.\n", ctx_.slavecount);
  expectedWKC = (group->outputsWKC * 2) + group->inputsWKC;
  printf("Calculated workcounter %d\n", expectedWKC);
  /* wait for all slaves to reach SAFE_OP state */
  ecx_statecheck(&ctx_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
  if (ctx_.slavelist[0].state != EC_STATE_SAFE_OP) {
    printf("Not all slaves reached safe operational state.\n");
    ecx_readstate(&ctx_);
    for (i = 1; i <= ctx_.slavecount; i++) {
      if (ctx_.slavelist[i].state != EC_STATE_SAFE_OP) {
        printf("Slave %d State=%2x StatusCode=%4x : %s\n", i,
               ctx_.slavelist[i].state, ctx_.slavelist[i].ALstatuscode,
               ec_ALstatuscode2string(ctx_.slavelist[i].ALstatuscode));
      }
    }
  }

  ecx_readstate(&ctx_);
  for (cnt = 1; cnt <= ctx_.slavecount; cnt++) {
    printf(
        "\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n "
        "State: %d\n Delay: %d[ns]\n Has DC: %d\n",
        cnt, ctx_.slavelist[cnt].name, ctx_.slavelist[cnt].Obits,
        ctx_.slavelist[cnt].Ibits, ctx_.slavelist[cnt].state,
        ctx_.slavelist[cnt].pdelay, ctx_.slavelist[cnt].hasdc);
    if (ctx_.slavelist[cnt].hasdc)
      printf(" DCParentport:%d\n", ctx_.slavelist[cnt].parentport);
    printf(" Activeports:%d.%d.%d.%d\n",
           (ctx_.slavelist[cnt].activeports & 0x01) > 0,
           (ctx_.slavelist[cnt].activeports & 0x02) > 0,
           (ctx_.slavelist[cnt].activeports & 0x04) > 0,
           (ctx_.slavelist[cnt].activeports & 0x08) > 0);
    printf(" Configured address: %4.4x\n", ctx_.slavelist[cnt].configadr);
    printf(" Man: %8.8x ID: %8.8x Rev: %8.8x\n",
           (int)ctx_.slavelist[cnt].eep_man, (int)ctx_.slavelist[cnt].eep_id,
           (int)ctx_.slavelist[cnt].eep_rev);
    for (nSM = 0; nSM < EC_MAXSM; nSM++) {
      if (ctx_.slavelist[cnt].SM[nSM].StartAddr > 0)
        printf(" SM%1d A:%4.4x L:%4d F:%8.8x Type:%d\n", nSM,
               etohs(ctx_.slavelist[cnt].SM[nSM].StartAddr),
               etohs(ctx_.slavelist[cnt].SM[nSM].SMlength),
               etohl(ctx_.slavelist[cnt].SM[nSM].SMflags),
               ctx_.slavelist[cnt].SMtype[nSM]);
    }
    for (j = 0; j < ctx_.slavelist[cnt].FMMUunused; j++) {
      printf(
          " FMMU%1d Ls:%8.8x Ll:%4d Lsb:%d Leb:%d Ps:%4.4x Psb:%d Ty:%2.2x "
          "Act:%2.2x\n",
          j, etohl(ctx_.slavelist[cnt].FMMU[j].LogStart),
          etohs(ctx_.slavelist[cnt].FMMU[j].LogLength),
          ctx_.slavelist[cnt].FMMU[j].LogStartbit,
          ctx_.slavelist[cnt].FMMU[j].LogEndbit,
          etohs(ctx_.slavelist[cnt].FMMU[j].PhysStart),
          ctx_.slavelist[cnt].FMMU[j].PhysStartBit,
          ctx_.slavelist[cnt].FMMU[j].FMMUtype,
          ctx_.slavelist[cnt].FMMU[j].FMMUactive);
    }
    printf(" FMMUfunc 0:%d 1:%d 2:%d 3:%d\n", ctx_.slavelist[cnt].FMMU0func,
           ctx_.slavelist[cnt].FMMU1func, ctx_.slavelist[cnt].FMMU2func,
           ctx_.slavelist[cnt].FMMU3func);
    printf(" MBX length wr: %d rd: %d MBX protocols : %2.2x\n",
           ctx_.slavelist[cnt].mbx_l, ctx_.slavelist[cnt].mbx_rl,
           ctx_.slavelist[cnt].mbx_proto);
    ssigen = ecx_siifind(&ctx_, cnt, ECT_SII_GENERAL);
    /* SII general section */
    if (ssigen) {
      ctx_.slavelist[cnt].CoEdetails =
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x07);
      ctx_.slavelist[cnt].FoEdetails =
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x08);
      ctx_.slavelist[cnt].EoEdetails =
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x09);
      ctx_.slavelist[cnt].SoEdetails =
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x0a);
      if ((ecx_siigetbyte(&ctx_, cnt, ssigen + 0x0d) & 0x02) > 0) {
        ctx_.slavelist[cnt].blockLRW = 1;
        ctx_.slavelist[0].blockLRW++;
      }
      ctx_.slavelist[cnt].Ebuscurrent =
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x0e);
      ctx_.slavelist[cnt].Ebuscurrent +=
          ecx_siigetbyte(&ctx_, cnt, ssigen + 0x0f) << 8;
      ctx_.slavelist[0].Ebuscurrent += ctx_.slavelist[cnt].Ebuscurrent;
    }
    printf(
        " CoE details: %2.2x FoE details: %2.2x EoE details: %2.2x SoE "
        "details: %2.2x\n",
        ctx_.slavelist[cnt].CoEdetails, ctx_.slavelist[cnt].FoEdetails,
        ctx_.slavelist[cnt].EoEdetails, ctx_.slavelist[cnt].SoEdetails);
    printf(" Ebus current: %d[mA]\n only LRD/LWR:%d\n",
           ctx_.slavelist[cnt].Ebuscurrent, ctx_.slavelist[cnt].blockLRW);
    if ((ctx_.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE) && printSDO)
      si_sdo(cnt);
    if (printMAP) {
      if (ctx_.slavelist[cnt].mbx_proto & ECT_MBXPROT_COE)
        si_map_sdo(cnt);
      else
        si_map_sii(cnt);
    }
  }
}

void Fieldbus::PrintAvaliableAdapters() {
  ec_adaptert *adapter = NULL;
  ec_adaptert *head = NULL;

  fmt::print("Adapters:\n");
}
uint16 Fieldbus::GetStatusWord(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }

  return inputs_[id].status_word;
}

int32 Fieldbus::GetPosition(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }

  return inputs_[id].position_actual_value;
}

int32 Fieldbus::GetVelocity(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }

  return inputs_[id].velocity_actual_value;
}

int16 Fieldbus::GetTorque(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }
}

int32 Fieldbus::GetAuxiliaryPosition(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }

  return inputs_[id].auxiliary_position_actual_value;
}

int16 Fieldbus::GetAnalogInput(int id) const {
  if (id < 0 || id >= 14) {
    // throw std::out_of_range("ID must be between 0 and 13");
    spdlog::warn("ID must be between 0 and 13, returning 0");
    return 0;
  }

  return inputs_[id].analog_input;
}

std::string Fieldbus::dtype2string(uint16 data_type, uint16 bit_length) {
  char str[32] = {0};

  switch (data_type) {
    case ECT_BOOLEAN:
      sprintf(str, "BOOLEAN");
      break;
    case ECT_INTEGER8:
      sprintf(str, "INTEGER8");
      break;
    case ECT_INTEGER16:
      sprintf(str, "INTEGER16");
      break;
    case ECT_INTEGER32:
      sprintf(str, "INTEGER32");
      break;
    case ECT_INTEGER24:
      sprintf(str, "INTEGER24");
      break;
    case ECT_INTEGER64:
      sprintf(str, "INTEGER64");
      break;
    case ECT_UNSIGNED8:
      sprintf(str, "UNSIGNED8");
      break;
    case ECT_UNSIGNED16:
      sprintf(str, "UNSIGNED16");
      break;
    case ECT_UNSIGNED32:
      sprintf(str, "UNSIGNED32");
      break;
    case ECT_UNSIGNED24:
      sprintf(str, "UNSIGNED24");
      break;
    case ECT_UNSIGNED64:
      sprintf(str, "UNSIGNED64");
      break;
    case ECT_REAL32:
      sprintf(str, "REAL32");
      break;
    case ECT_REAL64:
      sprintf(str, "REAL64");
      break;
    case ECT_BIT1:
      sprintf(str, "BIT1");
      break;
    case ECT_BIT2:
      sprintf(str, "BIT2");
      break;
    case ECT_BIT3:
      sprintf(str, "BIT3");
      break;
    case ECT_BIT4:
      sprintf(str, "BIT4");
      break;
    case ECT_BIT5:
      sprintf(str, "BIT5");
      break;
    case ECT_BIT6:
      sprintf(str, "BIT6");
      break;
    case ECT_BIT7:
      sprintf(str, "BIT7");
      break;
    case ECT_BIT8:
      sprintf(str, "BIT8");
      break;
    case ECT_VISIBLE_STRING:
      sprintf(str, "VISIBLE_STR(%d)", bit_length);
      break;
    case ECT_OCTET_STRING:
      sprintf(str, "OCTET_STR(%d)", bit_length);
      break;
    default:
      sprintf(str, "dt:0x%4.4X (%d)", data_type, bit_length);
  }
  return str;
}

std::string Fieldbus::otype2string(uint16 obj_type) {
  char str[32] = {0};

  switch (obj_type) {
    case OTYPE_VAR:
      sprintf(str, "VAR");
      break;
    case OTYPE_ARRAY:
      sprintf(str, "ARRAY");
      break;
    case OTYPE_RECORD:
      sprintf(str, "RECORD");
      break;
    default:
      sprintf(str, "ot:0x%4.4X", obj_type);
  }
  return str;
}

std::string Fieldbus::access2string(uint16 access) {
  char str[32] = {0};

  sprintf(str, "%s%s%s%s%s%s", ((access & ATYPE_Rpre) != 0 ? "R" : "_"),
          ((access & ATYPE_Wpre) != 0 ? "W" : "_"),
          ((access & ATYPE_Rsafe) != 0 ? "R" : "_"),
          ((access & ATYPE_Wsafe) != 0 ? "W" : "_"),
          ((access & ATYPE_Rop) != 0 ? "R" : "_"),
          ((access & ATYPE_Wop) != 0 ? "W" : "_"));
  return str;
}

std::string Fieldbus::SDO2string(uint16 slave, uint16 index, uint8 subidx,
                                 uint16 dtype) {
  char usdo[128];
  int l = sizeof(usdo) - 1, i;
  uint8 *u8;
  int8 *i8;
  uint16 *u16;
  int16 *i16;
  uint32 *u32;
  int32 *i32;
  uint64 *u64;
  int64 *i64;
  float *sr;
  double *dr;
  char *p;
  size_t size;

  memset(&usdo, 0, sizeof(usdo));
  ecx_SDOread(&ctx_, slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);
  if (ctx_.ecaterror) {
    return ecx_elist2string(&ctx_);
  } else {
    char str[sizeof(usdo) + 3] = {0};
    switch (dtype) {
      case ECT_BOOLEAN:
        u8 = (uint8 *)&usdo[0];
        if (*u8)
          snprintf(str, sizeof(str), "TRUE");
        else
          snprintf(str, sizeof(str), "FALSE");
        break;
      case ECT_INTEGER8:
        i8 = (int8 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%2.2x / %d", *i8, *i8);
        break;
      case ECT_INTEGER16:
        i16 = (int16 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%4.4x / %d", *i16, *i16);
        break;
      case ECT_INTEGER32:
      case ECT_INTEGER24:
        i32 = (int32 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%8.8x / %d", *i32, *i32);
        break;
      case ECT_INTEGER64:
        i64 = (int64 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%16.16" PRIx64 " / %" PRId64, *i64, *i64);
        break;
      case ECT_UNSIGNED8:
        u8 = (uint8 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%2.2x / %u", *u8, *u8);
        break;
      case ECT_UNSIGNED16:
        u16 = (uint16 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%4.4x / %u", *u16, *u16);
        break;
      case ECT_UNSIGNED32:
      case ECT_UNSIGNED24:
        u32 = (uint32 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%8.8x / %u", *u32, *u32);
        break;
      case ECT_UNSIGNED64:
        u64 = (uint64 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%16.16" PRIx64 " / %" PRIu64, *u64, *u64);
        break;
      case ECT_REAL32:
        sr = (float *)&usdo[0];
        snprintf(str, sizeof(str), "%f", *sr);
        break;
      case ECT_REAL64:
        dr = (double *)&usdo[0];
        snprintf(str, sizeof(str), "%f", *dr);
        break;
      case ECT_BIT1:
      case ECT_BIT2:
      case ECT_BIT3:
      case ECT_BIT4:
      case ECT_BIT5:
      case ECT_BIT6:
      case ECT_BIT7:
      case ECT_BIT8:
        u8 = (uint8 *)&usdo[0];
        snprintf(str, sizeof(str), "0x%x / %u", *u8, *u8);
        break;
      case ECT_VISIBLE_STRING:
        snprintf(str, sizeof(str), "\"%s\"", usdo);
        strcat(str, "\"");
        break;
      case ECT_OCTET_STRING:
        p = str;
        size = sizeof(str);
        for (i = 0; i < l; i++) {
          int n = snprintf(p, size, "0x%2.2x ", usdo[i]);
          if (n > (int)size) break;
          p += n;
          size -= n;
        }
        break;
      default:
        snprintf(str, sizeof(str), "Unknown type");
    }
    str[sizeof(str) - 1] = 0;
    return str;
  }
}
int Fieldbus::si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset,
                           int bitoffset) {
  uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
  uint8 subcnt;
  int wkc, bsize = 0, rdl;
  int32 rdat2;
  uint8 bitlen, obj_subidx;
  uint16 obj_idx;
  int abs_offset, abs_bit;

  rdl = sizeof(rdat);
  rdat = 0;
  /* read PDO assign subindex 0 ( = number of PDO's) */
  wkc = ecx_SDOread(&ctx_, slave, PDOassign, 0x00, FALSE, &rdl, &rdat,
                    EC_TIMEOUTRXM);
  rdat = etohs(rdat);
  /* positive result from slave ? */
  if ((wkc > 0) && (rdat > 0)) {
    /* number of available sub indexes */
    nidx = rdat;
    bsize = 0;
    /* read all PDO's */
    for (idxloop = 1; idxloop <= nidx; idxloop++) {
      rdl = sizeof(rdat);
      rdat = 0;
      /* read PDO assign */
      wkc = ecx_SDOread(&ctx_, slave, PDOassign, (uint8)idxloop, FALSE, &rdl,
                        &rdat, EC_TIMEOUTRXM);
      /* result is index of PDO */
      idx = etohs(rdat);
      if (idx > 0) {
        rdl = sizeof(subcnt);
        subcnt = 0;
        /* read number of subindexes of PDO */
        wkc = ecx_SDOread(&ctx_, slave, idx, 0x00, FALSE, &rdl, &subcnt,
                          EC_TIMEOUTRXM);
        subidx = subcnt;
        /* for each subindex */
        for (subidxloop = 1; subidxloop <= subidx; subidxloop++) {
          rdl = sizeof(rdat2);
          rdat2 = 0;
          /* read SDO that is mapped in PDO */
          wkc = ecx_SDOread(&ctx_, slave, idx, (uint8)subidxloop, FALSE, &rdl,
                            &rdat2, EC_TIMEOUTRXM);
          rdat2 = etohl(rdat2);
          /* extract bitlength of SDO */
          bitlen = LO_BYTE(rdat2);
          bsize += bitlen;
          obj_idx = (uint16)(rdat2 >> 16);
          obj_subidx = (uint8)((rdat2 >> 8) & 0x000000ff);
          abs_offset = mapoffset + (bitoffset / 8);
          abs_bit = bitoffset % 8;
          ODlist_.Slave = slave;
          ODlist_.Index[0] = obj_idx;
          OElist_.Entries = 0;
          wkc = 0;
          /* read object entry from dictionary if not a filler (0x0000:0x00) */
          if (obj_idx || obj_subidx)
            wkc = ecx_readOEsingle(&ctx_, 0, obj_subidx, &ODlist_, &OElist_);
          printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset, abs_bit,
                 obj_idx, obj_subidx, bitlen);
          if ((wkc > 0) && OElist_.Entries) {
            printf(" %-12s %s\n",
                   dtype2string(OElist_.DataType[obj_subidx], bitlen).c_str(),
                   OElist_.Name[obj_subidx]);
          } else
            printf("\n");
          bitoffset += bitlen;
        };
      };
    };
  };
  /* return total found bitlength (PDO) */
  return bsize;
}
int Fieldbus::si_map_sdo(int slave) {
  int wkc, rdl;
  int retVal = 0;
  uint8 nSM, iSM, tSM;
  int Tsize, outputs_bo, inputs_bo;
  uint8 SMt_bug_add;

  printf("PDO mapping according to CoE :\n");
  SMt_bug_add = 0;
  outputs_bo = 0;
  inputs_bo = 0;
  rdl = sizeof(nSM);
  nSM = 0;
  /* read SyncManager Communication Type object count */
  wkc = ecx_SDOread(&ctx_, slave, ECT_SDO_SMCOMMTYPE, 0x00, FALSE, &rdl, &nSM,
                    EC_TIMEOUTRXM);
  /* positive result from slave ? */
  if ((wkc > 0) && (nSM > 2)) {
    /* make nSM equal to number of defined SM */
    nSM--;
    /* limit to maximum number of SM defined, if true the slave can't be
     * configured */
    if (nSM > EC_MAXSM) nSM = EC_MAXSM;
    /* iterate for every SM type defined */
    for (iSM = 2; iSM <= nSM; iSM++) {
      rdl = sizeof(tSM);
      tSM = 0;
      /* read SyncManager Communication Type */
      wkc = ecx_SDOread(&ctx_, slave, ECT_SDO_SMCOMMTYPE, iSM + 1, FALSE, &rdl,
                        &tSM, EC_TIMEOUTRXM);
      if (wkc > 0) {
        if ((iSM == 2) &&
            (tSM ==
             2))  // SM2 has type 2 == mailbox out, this is a bug in the slave!
        {
          SMt_bug_add = 1;  // try to correct, this works if the types are 0 1 2
                            // 3 and should be 1 2 3 4
          printf("Activated SM type workaround, possible incorrect mapping.\n");
        }
        if (tSM) tSM += SMt_bug_add;  // only add if SMt > 0

        if (tSM == 3)  // outputs
        {
          /* read the assign RXPDO */
          printf(
              "  SM%1d outputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);
          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(ctx_.slavelist[slave].outputs - IOmap_),
                               outputs_bo);
          outputs_bo += Tsize;
        }
        if (tSM == 4)  // inputs
        {
          /* read the assign TXPDO */
          printf(
              "  SM%1d inputs\n     addr b   index: sub bitl data_type    "
              "name\n",
              iSM);

          Tsize = si_PDOassign(slave, ECT_SDO_PDOASSIGN + iSM,
                               (int)(ctx_.slavelist[slave].inputs - IOmap_),
                               inputs_bo);
          inputs_bo += Tsize;
        }
      }
    }
  }

  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0)) retVal = 1;
  return retVal;
}
int Fieldbus::si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset) {
  uint16 a, w, c, e, er;
  uint8 eectl;
  uint16 obj_idx;
  uint8 obj_subidx;
  uint8 obj_name;
  uint8 obj_datatype;
  uint8 bitlen;
  int totalsize;
  ec_eepromPDOt eepPDO;
  ec_eepromPDOt *PDO;
  int abs_offset, abs_bit;
  char str_name[EC_MAXNAME + 1];

  eectl = ctx_.slavelist[slave].eep_pdi;

  totalsize = 0;
  PDO = &eepPDO;
  PDO->nPDO = 0;
  PDO->Length = 0;
  PDO->Index[1] = 0;
  for (c = 0; c < EC_MAXSM; c++) PDO->SMbitsize[c] = 0;
  if (t > 1) t = 1;
  PDO->Startpos = ecx_siifind(&ctx_, slave, ECT_SII_PDO + t);
  if (PDO->Startpos > 0) {
    a = PDO->Startpos;
    w = ecx_siigetbyte(&ctx_, slave, a++);
    w += (ecx_siigetbyte(&ctx_, slave, a++) << 8);
    PDO->Length = w;
    c = 1;
    /* traverse through all PDOs */
    do {
      PDO->nPDO++;
      PDO->Index[PDO->nPDO] = ecx_siigetbyte(&ctx_, slave, a++);
      PDO->Index[PDO->nPDO] += (ecx_siigetbyte(&ctx_, slave, a++) << 8);
      PDO->BitSize[PDO->nPDO] = 0;
      c++;
      /* number of entries in PDO */
      e = ecx_siigetbyte(&ctx_, slave, a++);
      PDO->SyncM[PDO->nPDO] = ecx_siigetbyte(&ctx_, slave, a++);
      a++;
      obj_name = ecx_siigetbyte(&ctx_, slave, a++);
      a += 2;
      c += 2;
      if (PDO->SyncM[PDO->nPDO] < EC_MAXSM) /* active and in range SM? */
      {
        str_name[0] = 0;
        if (obj_name) ecx_siistring(&ctx_, str_name, slave, obj_name);
        if (t)
          printf("  SM%1d RXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO],
                 PDO->Index[PDO->nPDO], str_name);
        else
          printf("  SM%1d TXPDO 0x%4.4X %s\n", PDO->SyncM[PDO->nPDO],
                 PDO->Index[PDO->nPDO], str_name);
        printf("     addr b   index: sub bitl data_type    name\n");
        /* read all entries defined in PDO */
        for (er = 1; er <= e; er++) {
          c += 4;
          obj_idx = ecx_siigetbyte(&ctx_, slave, a++);
          obj_idx += (ecx_siigetbyte(&ctx_, slave, a++) << 8);
          obj_subidx = ecx_siigetbyte(&ctx_, slave, a++);
          obj_name = ecx_siigetbyte(&ctx_, slave, a++);
          obj_datatype = ecx_siigetbyte(&ctx_, slave, a++);
          bitlen = ecx_siigetbyte(&ctx_, slave, a++);
          abs_offset = mapoffset + (bitoffset / 8);
          abs_bit = bitoffset % 8;

          PDO->BitSize[PDO->nPDO] += bitlen;
          a += 2;

          /* skip entry if filler (0x0000:0x00) */
          if (obj_idx || obj_subidx) {
            str_name[0] = 0;
            if (obj_name) ecx_siistring(&ctx_, str_name, slave, obj_name);

            printf("  [0x%4.4X.%1d] 0x%4.4X:0x%2.2X 0x%2.2X", abs_offset,
                   abs_bit, obj_idx, obj_subidx, bitlen);
            printf(" %-12s %s\n", dtype2string(obj_datatype, bitlen).c_str(),
                   str_name);
          }
          bitoffset += bitlen;
          totalsize += bitlen;
        }
        PDO->SMbitsize[PDO->SyncM[PDO->nPDO]] += PDO->BitSize[PDO->nPDO];
        c++;
      } else /* PDO deactivated because SM is 0xff or > EC_MAXSM */
      {
        c += 4 * e;
        a += 8 * e;
        c++;
      }
      if (PDO->nPDO >= (EC_MAXEEPDO - 1))
        c = PDO->Length; /* limit number of PDO entries in buffer */
    } while (c < PDO->Length);
  }
  if (eectl)
    ecx_eeprom2pdi(
        &ctx_, slave); /* if eeprom control was previously pdi then restore */
  return totalsize;
}
int Fieldbus::si_map_sii(int slave) {
  int retVal = 0;
  int Tsize, outputs_bo, inputs_bo;

  printf("PDO mapping according to SII :\n");

  outputs_bo = 0;
  inputs_bo = 0;
  /* read the assign RXPDOs */
  Tsize = si_siiPDO(slave, 1, (int)(ctx_.slavelist[slave].outputs - IOmap_),
                    outputs_bo);
  outputs_bo += Tsize;
  /* read the assign TXPDOs */
  Tsize = si_siiPDO(slave, 0, (int)(ctx_.slavelist[slave].inputs - IOmap_),
                    inputs_bo);
  inputs_bo += Tsize;
  /* found some I/O bits ? */
  if ((outputs_bo > 0) || (inputs_bo > 0)) retVal = 1;
  return retVal;
}
void Fieldbus::si_sdo(int cnt) {
  int i, j;

  ODlist_.Entries = 0;
  memset(&ODlist_, 0, sizeof(ODlist_));
  if (ecx_readODlist(&ctx_, cnt, &ODlist_)) {
    printf(" CoE Object Description found, %d entries.\n", ODlist_.Entries);
    for (i = 0; i < ODlist_.Entries; i++) {
      uint16_t max_sub;
      char name[128] = {0};

      ecx_readODdescription(&ctx_, i, &ODlist_);
      while (ctx_.ecaterror) printf(" - %s\n", ecx_elist2string(&ctx_));
      snprintf(name, sizeof(name) - 1, "\"%s\"", ODlist_.Name[i]);
      if (ODlist_.ObjectCode[i] == OTYPE_VAR) {
        printf("0x%04x      %-40s      [%s]\n", ODlist_.Index[i], name,
               otype2string(ODlist_.ObjectCode[i]));
      } else {
        printf("0x%04x      %-40s      [%s  maxsub(0x%02x / %d)]\n",
               ODlist_.Index[i], name,
               otype2string(ODlist_.ObjectCode[i]).c_str(), ODlist_.MaxSub[i],
               ODlist_.MaxSub[i]);
      }
      memset(&OElist_, 0, sizeof(OElist_));
      ecx_readOE(&ctx_, i, &ODlist_, &OElist_);
      while (ctx_.ecaterror) printf("- %s\n", ecx_elist2string(&ctx_));

      if (ODlist_.ObjectCode[i] != OTYPE_VAR) {
        int l = sizeof(max_sub);
        ecx_SDOread(&ctx_, cnt, ODlist_.Index[i], 0, FALSE, &l, &max_sub,
                    EC_TIMEOUTRXM);
      } else {
        max_sub = ODlist_.MaxSub[i];
      }

      for (j = 0; j < max_sub + 1; j++) {
        if ((OElist_.DataType[j] > 0) && (OElist_.BitLength[j] > 0)) {
          snprintf(name, sizeof(name) - 1, "\"%s\"", OElist_.Name[j]);
          printf(
              "    0x%02x      %-40s      [%-16s %6s]      ", j, name,
              dtype2string(OElist_.DataType[j], OElist_.BitLength[j]).c_str(),
              access2string(OElist_.ObjAccess[j]).c_str());
          if ((OElist_.ObjAccess[j] & 0x0007)) {
            printf("%s",
                   SDO2string(cnt, ODlist_.Index[i], j, OElist_.DataType[j])
                       .c_str());
          }
          printf("\n");
        }
      }
    }
  } else {
    while (ctx_.ecaterror) printf("%s", ecx_elist2string(&ctx_));
  }
}

int Fieldbus::slave_setup(ecx_contextt *ctx, uint16 slave) {
  int retval;
  uint8 u8val;
  uint16 u16val;
  uint32 u32val;

  retval = 0;

  uint16 map_1607[11] = {
      0x0005, 0x0010, 0x6040,  // Control Word
      0x0008, 0x6060,          // Modes of Operation
      0x0020, 0x607a,          // Target Position
      0x0020, 0x60ff,          // Target Velocity
      0x0010, 0x6071,          // Target Torque
  };  // Disable PDOs
  retval += ecx_SDOwrite(ctx, slave, 0x1607, 0x00, TRUE, sizeof(map_1607),
                         &map_1607, EC_TIMEOUTSAFE);

  /* Map velocity PDO assignment via Complete Access */
  uint16 map_1c12[2] = {0x0001, 0x1607};

  retval += ecx_SDOwrite(ctx, slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12),
                         &map_1c12, EC_TIMEOUTSAFE);

  uint16 map_1a07[13] = {
      0x0006, 0x0010, 0x6041,  // Status Word
      0x0020, 0x6064,          // Position Actual Value
      0x0020, 0x606C,          // Velocity Actual Value
      0x0010, 0x6077,          // Torque Actual Value
      0x0020, 0x20A0,          // Auxiliary position actual value
      0x0110, 0x2205,          // Analog input
  };  // Disable PDOs
  retval += ecx_SDOwrite(ctx, slave, 0x1a07, 0x00, TRUE, sizeof(map_1a07),
                         &map_1a07, EC_TIMEOUTSAFE);

  /* Map velocity PDO assignment via Complete Access */
  uint16 map_1c13[2] = {0x0001, 0x1a07};

  retval += ecx_SDOwrite(ctx, slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13),
                         &map_1c13, EC_TIMEOUTSAFE);

  return retval;
}
