//
// Created by think on 8/19/25.
//

#include "kungshu_hardware_arm/arm_node.h"

namespace KSH {

ArmNode::ArmNode() : Node("arm_node") {
  this->declare_parameter<std::string>("left", "enp1s0");
  this->declare_parameter<std::string>("right", "enp2s0");

  // Get parameters
  std::string left = this->get_parameter("left").as_string();
  std::string right = this->get_parameter("right").as_string();

  RCLCPP_INFO(this->get_logger(), "Left arm bus: %s", left.c_str());
  RCLCPP_INFO(this->get_logger(), "Right arm bus: %s", right.c_str());

  left_bus_ = std::make_shared<Fieldbus>(left);
  right_bus_ = std::make_shared<Fieldbus>(right);

  left_bus_->Start();
  right_bus_->Start();

  state_publisher_ = this->create_publisher<kungshu_msgs::msg::ArmState>(
      "arm_state", rclcpp::SensorDataQoS());

  command_subscriber_ =
      this->create_subscription<kungshu_msgs::msg::ArmCommand>(
          "arm_command", rclcpp::SensorDataQoS(),
          std::bind(&ArmNode::command_callback, this, std::placeholders::_1));


}

void ArmNode::command_callback(const kungshu_msgs::msg::ArmCommand& msg)  {
  // Process the command message and send it to the appropriate bus
  // This is where you would implement the logic to handle arm commands
  for (int i = 0; i < 7; i++) {




    left_bus_->SetModeOfOperation(i, msg.mode[7+i]);
    left_bus_->SetTargetPosition(i, msg.q_d[7+i]);
    left_bus_->SetTargetVelocity(i, msg.dq_d[7+i]);
    left_bus_->SetTargetTorque(i, msg.tau_d[7+i]);



  }


}

}  // namespace KSH