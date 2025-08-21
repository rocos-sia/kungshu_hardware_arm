/*
 * This software is dual-licensed under GPLv3 and a commercial
 * license. See the file LICENSE.md distributed with this software for
 * full license information.
 */
#include <rclcpp/rclcpp.hpp>

#include "kungshu_hardware_arm/arm_node.h"

#include <spdlog/spdlog.h>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<KSH::ArmNode>();

  spdlog::info("Hello!! Kungshu Arm!");

  rclcpp::spin(node);

  rclcpp::shutdown();

  // while (true) {
  //
  //   for (int i = 0; i < 3; i++) {
  //     spdlog::info("status: {}; pos: {}; vel: {}; torque: {}; auxpos: {};
  //     analog: {}",
  //          fieldbus.GetStatusWord(i),
  //          fieldbus.GetPosition(i),
  //          fieldbus.GetVelocity(i),
  //          fieldbus.GetTorque(i),
  //          fieldbus.GetAuxiliaryPosition(i),
  //          fieldbus.GetAnalogInput(i));
  //   }
  //
  //   osal_usleep(1000000);
  // }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ByeBye, Kungshu Arm!");

  return (0);
}
