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

#include <fmt/core.h>
#include <soem/soem.h>

#include <string>

namespace KSH {
class Fieldbus {
 public:
  explicit Fieldbus(const std::string &ifname) : ifname_(ifname) {}

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

  void PrintSlaveInfo(
      bool printMAP = false,
      bool printSDO = false);  // Print information of all slaves

  void PrintAvaliableAdapters();  // Print all available adapters

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

 private:
  std::string ifname_{"enp1s0"};  // Net interface name

  ecx_contextt ctx_{};   // EtherCAT context
  uint8 group_{0};       // Group number for EtherCAT communication
  uint8 IOmap_[4096]{};  // I/O map buffer

  bool is_initialized_{false};  // Flag to check if the fieldbus is initialized

  ec_ODlistt ODlist_{};
  ec_OElistt OElist_{};
};
}  // namespace KSH

#endif  // KSH_FIELDBUS_H
