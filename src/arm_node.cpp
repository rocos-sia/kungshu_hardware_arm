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


  rclcpp::QoS qos_best_effort(10);
  qos_best_effort.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  // （可选）设置Durability为VOLATILE（不缓存消息，仅发给在线订阅者）
  qos_best_effort.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  state_publisher_ = this->create_publisher<kungshu_msgs::msg::ArmState>(
      "states", qos_best_effort);

  // command_subscriber_ =
  //     this->create_subscription<kungshu_msgs::msg::ArmServoCommand>(
  //         "commands", qos_best_effort,
  //         std::bind(&ArmNode::command_callback, this, std::placeholders::_1));

  enable_srv_ = this->create_service<kungshu_msgs::srv::SetEnable>("set_enble_serivce",
    [this](const std::shared_ptr<kungshu_msgs::srv::SetEnable::Request> request,
           std::shared_ptr<kungshu_msgs::srv::SetEnable::Response> response) {

      spdlog::info("Set Enable Service: {}", request->enable);

      bool success = true;

      if (request->enable) {
        for (int i = 0; i < 14; i++) {
          drivers_[i]->SetTargetPosition(drivers_[i]->GetPosition());
          drivers_[i]->SetTargetVelocity(0);
          drivers_[i]->SetTargetTorque(0);
          drivers_[i]->setDriverState(DriveState::OperationEnabled , false);
        }
      }
      else {
        for (int i = 0; i < 14; i++) {
          drivers_[i]->setDriverState(DriveState::SwitchOnDisabled , false);
        }
      }

      response->success = success;
    });


  mode_srv_ = this->create_service<kungshu_msgs::srv::SetModeOfOperation>("set_mode_service",
    [this](const std::shared_ptr<kungshu_msgs::srv::SetModeOfOperation::Request> request,
           std::shared_ptr<kungshu_msgs::srv::SetModeOfOperation::Response> response) {
      bool success = true;
      for (int i = 0; i < 14; i++) {
        drivers_[i]->SetModeOfOperationRaw(request->mode);
      }
      response->success = success;
    });

  move_j_srv_ = this->create_service<kungshu_msgs::srv::MoveJ>(
    "movej_service",
        [this](const std::shared_ptr<kungshu_msgs::srv::MoveJ::Request> request,
                   std::shared_ptr<kungshu_msgs::srv::MoveJ::Response> response) {

          for (int i = 0; i < 14; i++) {
            spdlog::info("Received move j service request");

              std::array<double, 14> target_pos{}, max_vel{}, max_acc{};

              for (int i = 0; i < 14; i++) {
                target_pos[i] = request->pos[i];
                max_vel[i] = request->vel[i];
                max_acc[i] = request->acc[i];
              }

              MoveJ(target_pos, max_vel, max_acc);
          }

        }
  );



  time_sync_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {

      auto time_start = std::chrono::high_resolution_clock::now();

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

      loop_mutex_.lock();
      if (is_running_) {
        if (otg_.update(input_, output_) == ruckig::Result::Working) {

          for (int i = 0; i < 14; i++) {
              drivers_[i]->SetTargetPosition(output_.new_position[i]);
           }

            output_.pass_to_input(input_);
        }
        else {
          is_running_ = false;
        }
      }
      loop_mutex_.unlock();

      Fieldbus::LoopOnce();



      auto time_end = std::chrono::high_resolution_clock::now();

      int elasped_us = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

      // Here you can implement time synchronization logic if needed
      // For example, you can read the current time and publish it
      if (elasped_us < 4000-10) {
        std::this_thread::sleep_for(std::chrono::microseconds(4000 - 10 - elasped_us));
      }
      else {
        spdlog::warn("ArmNode loop is too slow: {} us", elasped_us);
      }

    }
  });

  // Read some pdo
  for (int i = 0; i < 3; i++) {
    Fieldbus::LoopOnce();
    std::this_thread::sleep_for(std::chrono::microseconds(4000));
  }

  for (int i = 0; i < 14; i++) {
    drivers_[i]->SetModeOfOperationRaw(8); // Cyclic Synchronous Position Mode
    drivers_[i]->SetTargetPosition(drivers_[i]->GetPosition());
    drivers_[i]->SetTargetVelocity(0);
    drivers_[i]->SetTargetTorque(0);

    input_.current_position[i] = drivers_[i]->GetPosition();
    input_.current_velocity[i] = 0.0;
    input_.current_acceleration[i] = 0.0;
  }

  input_.synchronization = ruckig::Synchronization::Phase;


}

void ArmNode::MoveJ(const std::array<double, 14>& target_pos,
                             const std::array<double, 14>& max_vel,
                             const std::array<double, 14>& max_acc) {
  loop_mutex_.lock();
  for (int i = 0; i < 14; i++) {
    // spdlog::info("Joint {}: pos->{}, target_pos->{}, max_vel->{}, max_acc->{}",i, drivers_[i]->GetPosition(), target_pos[i], max_vel[i], max_acc[i]);
    // input_.current_position[i] = drivers_[i]->GetPosition();
    // input_.current_velocity[i] = 0.0;
    // input_.current_acceleration[i] = 0.0;

    input_.target_position[i] = target_pos[i];
    input_.target_velocity[i] = 0.0;
    input_.target_acceleration[i] = 0.0;
    input_.max_velocity[i] = max_vel[i];
    input_.max_acceleration[i] = max_acc[i];
    input_.max_jerk[i] = max_acc[i] * 10;
  }

  loop_mutex_.unlock();
  is_running_ = true;
}

void ArmNode::command_callback(const kungshu_msgs::msg::ArmServoCommand& msg) {
  // Process the command message and send it to the appropriate bus
  // This is where you would implement the logic to handle arm commands
  for (int i = 0; i < 14; i++) {
    switch (drivers_[i]->GetModeOfOperationRaw()) {
      case 8:
        drivers_[i]->SetTargetPosition(msg.servo_cmd[i]);
        break;
      case 9:
        drivers_[i]->SetTargetVelocity(msg.servo_cmd[i]);
        break;
      case 10:
        drivers_[i]->SetTargetTorque(msg.servo_cmd[i]);
        break;
    }
  }
}

}  // namespace KSH