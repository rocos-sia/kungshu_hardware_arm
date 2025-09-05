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

#ifndef KSH_ARM_NODE_H
#define KSH_ARM_NODE_H

#include <rclcpp/rclcpp.hpp>

#include <kungshu_hardware_arm/fieldbus.h>

#include <kungshu_msgs/msg/arm_servo_command.hpp>
#include <kungshu_msgs/msg/arm_state.hpp>

#include <kungshu_msgs/srv/set_enable.hpp>
#include <kungshu_msgs/srv/set_mode_of_operation.hpp>

#include  <ruckig/ruckig.hpp>



namespace KSH {

class ArmNode : public rclcpp::Node {
 public:
  ArmNode();

private:
  void command_callback(const kungshu_msgs::msg::ArmServoCommand& msg);


 private:
  std::shared_ptr<Fieldbus> left_bus_;    // left arm
  std::shared_ptr<Fieldbus> right_bus_;  // right arm

  rclcpp::Publisher<kungshu_msgs::msg::ArmState>::SharedPtr state_publisher_;
  rclcpp::Subscription<kungshu_msgs::msg::ArmServoCommand>::SharedPtr command_subscriber_;

  rclcpp::Service<kungshu_msgs::srv::SetEnable>::SharedPtr enable_srv_;
  rclcpp::Service<kungshu_msgs::srv::SetModeOfOperation>::SharedPtr mode_srv_;

  std::vector<Drive*> drivers_ {};

  std::thread time_sync_thread_;  // Thread for time synchronization

  ruckig::Ruckig<14> otg_{0.004}; // 4ms control period
  ruckig::InputParameter<14> input_;
  ruckig::OutputParameter<14> output_;




};

}  // namespace KSH

#endif  // KSH_ARM_NODE_H
