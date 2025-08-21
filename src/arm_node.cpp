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

  // Add left arm driver first.
  for (int i = 0; i < 7; i++) {
    auto driver = new Drive(i,
                            &(left_bus_->GetInputsPointer()[i]),
                            &(left_bus_->GetOutputsPointer()[i]));
    drivers_.push_back(driver);
  }
  // Add right arm driver second.
  for (int i = 0; i < 7; i++) {
    auto driver = new Drive(7+i,
                            &(right_bus_->GetInputsPointer()[i]),
                            &(right_bus_->GetOutputsPointer()[i]));
    drivers_.push_back(driver);
  }

  state_publisher_ = this->create_publisher<kungshu_msgs::msg::ArmState>(
      "arm_state", 10);

  command_subscriber_ =
      this->create_subscription<kungshu_msgs::msg::ArmCommand>(
          "arm_command", 10,
          std::bind(&ArmNode::command_callback, this, std::placeholders::_1));


  time_sync_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {

      kungshu_msgs::msg::ArmState state;

      for (int i = 0; i < 14; i++) {
        state.header.stamp = this->now();
        state.q[i] = drivers_[i]->GetPosition();
        state.dq[i] = drivers_[i]->GetVelocity();
        state.tau[i] = drivers_[i]->GetTorque();
        state.status[i] = drivers_[i]->GetStatus();
        state.temperature[i] = drivers_[i]->GetStatusWord();
      }

      state_publisher_->publish(state);


      Fieldbus::LoopOnce();

      // Here you can implement time synchronization logic if needed
      // For example, you can read the current time and publish it
      rclcpp::sleep_for(std::chrono::microseconds(4000));
    }
  });



}

void ArmNode::command_callback(const kungshu_msgs::msg::ArmCommand& msg) {
  // Process the command message and send it to the appropriate bus
  // This is where you would implement the logic to handle arm commands
  for (int i = 0; i < 14; i++) {
    drivers_[i]->SetModeOfOperation(msg.mode[i]);
    drivers_[i]->SetTargetPosition(msg.q_d[i]);
    drivers_[i]->SetTargetVelocity(msg.dq_d[i]);
    drivers_[i]->SetTargetTorque(msg.tau_d[i]);
    drivers_[i]->SetCommand(msg.command[i]);
  }
}

}  // namespace KSH