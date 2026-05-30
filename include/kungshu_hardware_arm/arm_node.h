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
#include "kungshu_msgs/srv/set_torque_enable.hpp"
// #include <kungshu_msgs/srv/move_j.hpp>
#include "kungshu_msgs/msg/move_j_command.hpp"   
#include "kungshu_msgs/msg/arm_torque_command.hpp"  
#include  <ruckig/ruckig.hpp>



namespace KSH {

class ArmNode : public rclcpp::Node {
 public:
  ArmNode();

  void MoveJ(const std::vector<double>& target_pos, const std::vector<double>& max_vel, const std::vector<double>& max_acc);


private:
  void command_callback(const kungshu_msgs::msg::ArmServoCommand& msg);


 private:
  // Configuration
  int num_arms_;           // 1 or 2
  int joints_per_arm_;     // 7
  int total_joints_;       // num_arms * joints_per_arm

  std::vector<std::shared_ptr<Fieldbus>> buses_;  // Dynamic bus list

  rclcpp::Publisher<kungshu_msgs::msg::ArmState>::SharedPtr state_publisher_;
  rclcpp::Subscription<kungshu_msgs::msg::ArmServoCommand>::SharedPtr command_subscriber_;


  rclcpp::Service<kungshu_msgs::srv::SetEnable>::SharedPtr enable_srv_;
  rclcpp::Service<kungshu_msgs::srv::SetModeOfOperation>::SharedPtr mode_srv_;
  rclcpp::Service<kungshu_msgs::srv::SetTorqueEnable>::SharedPtr torque_enable_srv_;

  // rclcpp::Service<kungshu_msgs::srv::MoveJ>::SharedPtr move_j_srv_;
  rclcpp::Subscription<kungshu_msgs::msg::MoveJCommand>::SharedPtr move_j_sub_;
  rclcpp::Subscription<kungshu_msgs::msg::ArmTorqueCommand>::SharedPtr tau_sub_;
  std::vector<Drive*> drivers_ {};

  std::thread time_sync_thread_;  // Thread for time synchronization
  std::thread publish_thread_;
  // Ruckig per arm (7 joints each)
  std::vector<ruckig::Ruckig<7>> otg_;
  std::vector<ruckig::InputParameter<7>> input_;
  std::vector<ruckig::OutputParameter<7>> output_;


  std::atomic<bool> is_running_ = false;

  std::mutex loop_mutex_;

};

}  // namespace KSH

#endif  // KSH_ARM_NODE_H
