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

  //driver param set
  {
    // Left 1, 2
    drivers_[0]->SetDriverParam(131072.0, 120.0, 0.7, 2.4827, 48.4514); // ST5-1-TK-110-25172157
    drivers_[1]->SetDriverParam(131072.0, 120.0, 0.7, 2.4846, 46.9179); // ST5-1-TK-110-25172163
    // Left 3, 4
    drivers_[2]->SetDriverParam(131072.0, 120.0, 0.5, 2.4864, 25.4319); // ST5-1-TK-60-25172175
    drivers_[3]->SetDriverParam(131072.0, 120.0, 0.5, 2.4922, 25.7646); // ST5-1-TK-60-25172171
    // Left 5, 6, 7
    drivers_[4]->SetDriverParam(131072.0, 100.0, 0.3, 2.5182, 7.5164); // ST5-1-TK-17-25193939
    drivers_[5]->SetDriverParam(131072.0, 100.0, 0.3, 2.4929, 7.4946); // ST5-1-TK-17-25183334
    drivers_[6]->SetDriverParam(131072.0, 100.0, 0.3, 2.5059, 7.6234); // ST5-1-TK-17-25183333

    // Right 1, 2
    drivers_[7]->SetDriverParam(131072.0, 120.0, 0.7, 2.5172, 46.8725); // ST5-1-TK-110-25172164
    drivers_[8]->SetDriverParam(131072.0, 120.0, 0.7, 2.4832, 48.1380); // ST5-1-TK-110-25195617
    // Right 3, 4
    drivers_[9]->SetDriverParam(131072.0, 120.0, 0.5, 2.4841, 25.2899); // ST5-1-TK-60-25172170
    drivers_[10]->SetDriverParam(131072.0, 120.0, 0.5, 2.4818, 25.6261); // ST5-1-TK-60-25172174
    // Right 5, 6, 7
    drivers_[11]->SetDriverParam(131072.0, 100.0, 0.3, 2.5362, 7.3970); // ST5-1-TK-17-25183328
    drivers_[12]->SetDriverParam(131072.0, 100.0, 0.3, 2.4765, 7.5133); // ST5-1-TK-17-25193936
    drivers_[13]->SetDriverParam(131072.0, 100.0, 0.3, 2.4585, 7.3354); // ST5-1-TK-17-25193942

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
        state.load[i] = drivers_[i]->GetLoadTorque();
        state.status[i] = drivers_[i]->GetStatus();
        state.temperature[i] = drivers_[i]->GetStatusWordRaw();
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
    drivers_[i]->SetModeOfOperationRaw(msg.mode[i]);
    drivers_[i]->SetTargetPosition(msg.q_d[i]);
    drivers_[i]->SetTargetVelocity(msg.dq_d[i]);
    drivers_[i]->SetTargetTorque(msg.tau_d[i]);
    drivers_[i]->SetCommand(msg.command[i]);
  }
}

}  // namespace KSH