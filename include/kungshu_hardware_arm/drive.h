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

#ifndef KSH_DRIVE_H
#define KSH_DRIVE_H

#include <kungshu_hardware_arm/ethercat.h>
#include <atomic>
#include <chrono>
#include <thread>
#include <memory>
#include <spdlog/spdlog.h>

namespace KSH {

struct __attribute__((__packed__)) Outputs {
  uint16_t control_word;  // Control word
  int8_t   mode_of_operation;  // Mode of operation
  int32_t  target_position;  // Target Position
  int32_t  target_velocity;  // Target Velocity
  int16_t  target_torque;  // Target Torque
};

struct __attribute__((__packed__)) Inputs {
  uint16_t status_word;  // Status word
  int32_t  position_actual_value;  // Position Actual Value
  int32_t  velocity_actual_value;  // Velocity Actual Value
  int16_t  torque_actual_value;  // Torque Actual Value
  int32_t  auxiliary_position_actual_value;  // Auxiliary position actual value
  int16_t  analog_input;  // Analog input
};

class Drive {
public:
  explicit Drive(int id, Inputs* inputs, Outputs* outputs);

  inline uint16_t GetStatusWordRaw() { return inputs_->status_word; }

  inline int32_t GetPositionRaw() { return inputs_->position_actual_value; }

  inline int32_t GetVelocityRaw()  { return inputs_->velocity_actual_value; }

  inline int16_t GetTorqueRaw() { return inputs_->torque_actual_value; }

  inline int32_t GetAuxiliaryPositionRaw() { return inputs_->auxiliary_position_actual_value; }

  inline int16_t GetAnalogInputRaw() { return inputs_->analog_input; }

  inline void SetControlWordRaw(uint16_t value) { outputs_->control_word = value; }

  inline void SetModeOfOperationRaw(int8_t value) { outputs_->mode_of_operation = value; }

  inline void SetTargetPositionRaw(int32_t value) { outputs_->target_position = value; }

  inline void SetTargetVelocityRaw(int32_t value) { outputs_->target_velocity = value; }

  inline void SetTargetTorqueRaw(int16_t value) { outputs_->target_torque = value; }


  inline double GetPosition() {return inputs_->position_actual_value * cnt2rad_;}

  inline double GetVelocity() {return inputs_->velocity_actual_value * cnt2rad_;}

  inline double GetTorque() {return inputs_->torque_actual_value * cnt2nm_;} // mNm to Nm

  inline double GetLoadTorque() {return (inputs_->auxiliary_position_actual_value / 1000.0 - t_0_ ) * t_k_;}

  inline void SetTargetPosition(double value) {outputs_->target_position = static_cast<int32_t>(value / cnt2rad_);}

  inline void SetTargetVelocity(double value) {outputs_->target_velocity = static_cast<int32_t>(value / cnt2rad_);}

  inline void SetTargetTorque(double value) {outputs_->target_torque = static_cast<int16_t>(value / cnt2nm_);}



  int8_t GetStatus();

  /// @brief Set Command of a slave
  /// @param value Command value to set. 0->disabled, 1->enabled, 2->reset,
  inline void SetCommand(int8_t value) {
    switch (value) {
      case 0:  // disable
        setDriverState(DriveState::SwitchOnDisabled);
        break;
      case 1:  // enable
        setDriverState(DriveState::OperationEnabled);
        break;
      case 2:  // reset
        setDriverState(DriveState::SwitchOnDisabled);
        break;
      default:
        spdlog::warn("Invalid command value: {}, must be 0, 1, or 2", value);
    }
  }

  ////////////State Machine Functions//////////////
  DriveState getDriverState(int id);

  /// \brief 获取下一个状态控制字
  /// \param requestedDriveState 请求的驱动器状态
  /// \param currentDriveState  当前的驱动器状态
  /// \return 返回控制字
  Controlword getNextStateTransitionControlword(const DriveState &requestedDriveState,
                                                       const DriveState &currentDriveState);

  void engageStateMachine();

  bool setDriverState(const DriveState &driveState, bool waitForState = true);


  void SetDriverParam(double cnt_per_round, double ratio, double rate_torque, double t_0, double t_k) {
    cnt_per_round_ = cnt_per_round;
    cnt2rad_ = 2 * M_PI / cnt_per_round_;
    ratio_ = ratio;
    rate_torque_ = rate_torque;
    cnt2nm_ = rate_torque_ / 1000.0 * ratio_; // mNm to Nm
    t_0_ = t_0;
    t_k_ = t_k;
  }

private:

  int id_ {0}; // 关节名称
  Statusword statusword_{}; // 状态字
  Controlword controlword_{}; // 控制字

  ModeOfOperation mode_{ModeOfOperation::CyclicSynchronousPositionMode};

  DriveState current_drive_state_ {DriveState::NA};
  DriveState target_drive_state_ {DriveState::NA};

  std::atomic<bool> conduct_state_change_{false}; //是否启动状态机 默认不启动
  std::atomic<bool> state_change_successful_{false}; //当前状态切换是否成功

  std::chrono::system_clock::time_point drive_state_change_time_point_;

  uint16_t num_of_successful_target_state_readings_{0};

  Inputs* inputs_{};
  Outputs* outputs_{};

  static std::thread guard_thread_; // 保护线程
  static std::atomic<bool> is_guard_thread_running_; // 线程运行标志

  static std::vector<Drive*> driver_list_; // 所有驱动器列表

  static void AddToDriveGuard(Drive* drive);

  double cnt_per_round_ { 131072.0 }; // 编码器每转的计数
  double cnt2rad_ { 2 * M_PI / cnt_per_round_ }; // 编码器计数转弧度

  double ratio_ {100.0};

  double rate_torque_ { 0.7 }; // unit: Nm
  double cnt2nm_ { rate_torque_ / 1000.0 * ratio_ }; // 编码器计数转力矩

  double t_0_ { 2.50 }; // Torque sensor zero offset in V
  double t_k_ { 48.0 }; // Torque sensor sensitivity in Nm / V


};

}  // namespace KSH

#endif  // KSH_DRIVE_H
