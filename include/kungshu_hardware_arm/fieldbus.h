// Copyright 2025, Yang Luo
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn

#ifndef KSH_FIELDBUS_H
#define KSH_FIELDBUS_H

#include <kungshu_hardware_arm/drive.h>

#include <fmt/core.h>
#include <soem/soem.h>
#include <spdlog/spdlog.h>

#include <string>
#include <thread>

#include <semaphore.h>

#include <mutex>
#include <condition_variable>

namespace KSH {
class Fieldbus {
 public:

  explicit Fieldbus(const std::string &ifname) : ifname_(ifname) {}
  ~Fieldbus() {
    if (is_initialized_) {
      Close();
    }

    sem_destroy(&loop_sem_);
  }

  enum ObjType { OTYPE_VAR = 0x7, OTYPE_ARRAY = 0x8, OTYPE_RECORD = 0x9 };

  enum AccessType {
    ATYPE_Rpre = 0x01,
    ATYPE_Rsafe = 0x02,
    ATYPE_Rop = 0x04,
    ATYPE_Wpre = 0x08,
    ATYPE_Wsafe = 0x10,
    ATYPE_Wop = 0x20,
  };

 public:
  bool Start();  // Initialize EtherCAT Bus

  bool Close();

  int Roundtrip();

  /// @brief Print information of all slaves
  /// @param printMAP if true, print the mapping of PDOs
  /// @param printSDO  if true, print the SDOs of slaves
  void PrintSlaveInfo(
      bool printMAP = false,
      bool printSDO = false);  // Print information of all slaves

  /// @brief Print all available adapters
  void PrintAvaliableAdapters();  // Print all available adapters

  uint16 GetErrorCode(int id = 0) const;

  /// @brief Get Statusword of a slave
  /// @param id Slave ID (0-13)
  /// @return Status word of the slave
  uint16 GetStatusWordRaw(int id = 0) const;

  /// @brief Get Position of a slave
  /// @param id Slave ID (0-13)
  /// @return Position of the slave
  int32 GetPositionRaw(int id = 0) const;

  /// @brief Get Velocity of a slave
  /// @param id Slave ID (0-13)
  /// @return Velocity of the slave
  int32 GetVelocityRaw(int id = 0) const;

  /// @brief Get Torque of a slave
  /// @param id Slave ID (0-13)
  /// @return Torque of the slave
  int16 GetTorqueRaw(int id = 0) const;

  /// @brief Get Auxiliary Position of a slave
  /// @param id Slave ID (0-13)
  /// @return Auxiliary Position of the slave
  int32 GetAuxiliaryPositionRaw(int id = 0) const;

  /// @brief Get Analog Input of a slave (torque sensor)
  /// @param id Slave ID (0-13)
  /// @return Analog Input of the slave
  int16 GetAnalogInputRaw(int id = 0) const;

  /// @brief Set Control Word of a slave
  /// @param id Slave ID (0-13)
  /// @param value Control word value to set
  void SetControlWordRaw(int id, uint16 value);

  /// @brief Set Command of a slave
  /// @param id Slave ID (0-13)
  /// @param value Command value to set. 0->disabled, 1->enabled, 2->reset,
  void SetCommand(int id, int8 value);

  /// @brief Set Target Position of a slave
  /// @param id Slave ID (0-13)
  /// @param value Target position value to set
  void SetTargetPositionRaw(int id, int32 value);

  /// @brief Set Target Velocity of a slave
  /// @param id Slave ID (0-13)
  /// @param value Target velocity value to set
  void SetTargetVelocityRaw(int id, int32 value);

  /// @brief Set Target Torque of a slave
  /// @param id Slave ID (0-13)
  /// @param value Target torque value to set
  void SetTargetTorqueRaw(int id, int16 value);

  /// @brief Set Mode of Operation of a slave
  /// @param id Slave ID (0-13)
  /// @param value Mode of operation value to set
  void SetModeOfOperation(int id, int8 value);

  Inputs* GetInputsPointer() { return inputs_;}
  Outputs* GetOutputsPointer() { return outputs_;}

  static void LoopOnce();

 private:

  /// @brief Convert data type to string
  /// @param data_type data type
  /// @param bit_length bit length
  /// @return string representation of data type
  std::string dtype2string(uint16 data_type, uint16 bit_length);

  std::string otype2string(uint16 obj_type);

  std::string access2string(uint16 access);

  std::string SDO2string(uint16 slave, uint16 index, uint8 subidx,
                         uint16 dtype);

  int si_PDOassign(uint16 slave, uint16 PDOassign, int mapoffset,
                   int bitoffset);

  int si_map_sdo(int slave);

  int si_siiPDO(uint16 slave, uint8 t, int mapoffset, int bitoffset);

  int si_map_sii(int slave);

  void si_sdo(int cnt);

  static int slave_setup(ecx_contextt * ctx, uint16 slave);

 private:
  std::string ifname_{"enp1s0"};  // Net interface name

  ecx_contextt ctx_{};   // EtherCAT context
  uint8 group_{0};       // Group number for EtherCAT communication
  uint8 IOmap_[4096]{};  // I/O map buffer

  int roundtrip_time_ {0};

  int min_time_ {0};
  int max_time_ {std::numeric_limits<int>::max()};

  bool is_initialized_{false};  // Flag to check if the fieldbus is initialized

  ec_ODlistt ODlist_{};
  ec_OElistt OElist_{};

  Inputs inputs_[8] {};
  Outputs outputs_[8] {}; //! TODO: This need to be longer than the real pdo length. Don't know why.

  std::thread fieldbus_thread_;

  // Used for synchronization two fieldbus
  // static std::mutex mtx_;
  // static std::condition_variable cv_;
  // static bool is_loop_time_up;

public:
  static sem_t loop_sem_;

};
}  // namespace KSH

#endif  // KSH_FIELDBUS_H
