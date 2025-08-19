//
// Created by think on 8/19/25.
//

#include "kungshu_hardware_arm/arm_node.h"

namespace KSH {

ArmNode::ArmNode() : Node("arm_node") {
  this->declare_parameter<std::string>("left", "enp1s0");
  this->declare_parameter<std::string>("right", "enp2s0");

  //Get parameters
  std::string left = this->get_parameter("left").as_string();
  std::string right = this->get_parameter("right").as_string();

  RCLCPP_INFO(this->get_logger(), "Left arm bus: %s", left.c_str());
  RCLCPP_INFO(this->get_logger(), "Right arm bus: %s", right.c_str());

  Fieldbus left_bus(left); // left arm
  Fieldbus right_bus(right); // right arm

  left_bus.Start();
  right_bus.Start();

}

}  // namespace KSH