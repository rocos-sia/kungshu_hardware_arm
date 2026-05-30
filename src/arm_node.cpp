//
// Created by think on 8/19/25.
//

#include "kungshu_hardware_arm/arm_node.h"

namespace KSH {

ArmNode::ArmNode() : Node("arm_node") {
  // Declare parameters
  this->declare_parameter<int>("num_arms", 2);
  this->declare_parameter<int>("joints_per_arm", 7);
  this->declare_parameter<std::string>("left", "");
  this->declare_parameter<std::string>("right", "");

  // Get parameters
  num_arms_ = this->get_parameter("num_arms").as_int();
  joints_per_arm_ = this->get_parameter("joints_per_arm").as_int();
  total_joints_ = num_arms_ * joints_per_arm_;

  RCLCPP_INFO(this->get_logger(), "Arms: %d, Joints per arm: %d, Total: %d",
               num_arms_, joints_per_arm_, total_joints_);

  // Validate
  if (num_arms_ < 1 || num_arms_ > 2) {
    RCLCPP_ERROR(this->get_logger(), "num_arms must be 1 or 2, got %d", num_arms_);
    throw std::runtime_error("Invalid num_arms parameter");
  }

  // Create buses dynamically
  std::string left_port = this->get_parameter("left").as_string();
  std::string right_port = this->get_parameter("right").as_string();

  if (num_arms_ >= 1 && !left_port.empty()) {
    RCLCPP_INFO(this->get_logger(), "Left arm bus: %s", left_port.c_str());
    auto left_bus = std::make_shared<Fieldbus>(left_port);
    buses_.push_back(left_bus);
  }
  if (num_arms_ >= 2 && !right_port.empty()) {
    RCLCPP_INFO(this->get_logger(), "Right arm bus: %s", right_port.c_str());
    auto right_bus = std::make_shared<Fieldbus>(right_port);
    buses_.push_back(right_bus);
  }

  // Start all buses
  for (auto& bus : buses_) {
    bus->Start();
  }

  // Create drivers dynamically
  int id = 0;
  for (auto& bus : buses_) {
    for (int i = 0; i < joints_per_arm_; i++) {
      auto driver = new Drive(id,
                              bus->GetInputsPointer() + i,
                              bus->GetOutputsPointer() + i);
      drivers_.push_back(driver);
      id++;
    }
  }

  // Driver parameter set
  // TODO: Load from config file instead of hardcoding
  if (total_joints_ == 14) {
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
  else if (total_joints_ == 7) {
    // Single arm - Left arm default params
    // TODO: Make configurable per arm
    drivers_[0]->SetDriverParam(131072.0, 120.0, 0.7, 2.4827, 48.4514);
    drivers_[1]->SetDriverParam(131072.0, 120.0, 0.7, 2.4846, 46.9179);
    drivers_[2]->SetDriverParam(131072.0, 120.0, 0.5, 2.4864, 25.4319);
    drivers_[3]->SetDriverParam(131072.0, 120.0, 0.5, 2.4922, 25.7646);
    drivers_[4]->SetDriverParam(131072.0, 100.0, 0.3, 2.5182, 7.5164);
    drivers_[5]->SetDriverParam(131072.0, 100.0, 0.3, 2.4929, 7.4946);
    drivers_[6]->SetDriverParam(131072.0, 100.0, 0.3, 2.5059, 7.6234);
  }


  rclcpp::QoS qos_best_effort(10);
  qos_best_effort.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_best_effort.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


  state_publisher_ = this->create_publisher<kungshu_msgs::msg::ArmState>(
      "states", qos_best_effort);

  enable_srv_ = this->create_service<kungshu_msgs::srv::SetEnable>("set_enable_service",
    [this](const std::shared_ptr<kungshu_msgs::srv::SetEnable::Request> request,
           std::shared_ptr<kungshu_msgs::srv::SetEnable::Response> response) {

      spdlog::info("Set Enable Service: {}", request->enable);

      bool success = true;

      if (request->enable) {
        for (size_t i = 0; i < drivers_.size(); i++) {
          drivers_[i]->SetTargetPosition(drivers_[i]->GetPosition());
          drivers_[i]->SetTargetVelocity(0);
          drivers_[i]->SetTargetTorque(0);
          drivers_[i]->setDriverState(DriveState::OperationEnabled , false);
        }
      }
      else {
        for (size_t i = 0; i < drivers_.size(); i++) {
          drivers_[i]->setDriverState(DriveState::SwitchOnDisabled , false);
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100000));
      for (size_t i = 0; i < drivers_.size(); i++) {
          if(drivers_[i]->getDriverState(0) != DriveState::OperationEnabled)
          response->success = false;
      }
      response->success = success;
    });


  mode_srv_ = this->create_service<kungshu_msgs::srv::SetModeOfOperation>("set_mode_service",
    [this](const std::shared_ptr<kungshu_msgs::srv::SetModeOfOperation::Request> request,
           std::shared_ptr<kungshu_msgs::srv::SetModeOfOperation::Response> response) {
      bool success = true;
      for (size_t i = 0; i < drivers_.size(); i++) {
        drivers_[i]->SetModeOfOperationRaw(request->mode);
      }
      response->success = success;
    });

  torque_enable_srv_ = this->create_service<kungshu_msgs::srv::SetTorqueEnable>(
  "set_torque_enable",
  [this](const std::shared_ptr<kungshu_msgs::srv::SetTorqueEnable::Request> request,
         std::shared_ptr<kungshu_msgs::srv::SetTorqueEnable::Response> response)
  {
    std::lock_guard<std::mutex> lock(loop_mutex_);

    for (size_t i = 0; i < drivers_.size(); ++i) {
      if (drivers_[i]->getDriverState(0) != DriveState::OperationEnabled) {
        response->success = false;
        response->message = "Joint " + std::to_string(i) + " is not in OperationEnabled";
        return;
      }
    }

    uint8_t new_mode = request->torque_enable ? 10 : 8;  // 10=CST, 8=CSP
    for (size_t i = 0; i < drivers_.size(); ++i) {
      drivers_[i]->SetModeOfOperationRaw(new_mode);

      if (!request->torque_enable) {
        drivers_[i]->SetTargetPosition(drivers_[i]->GetPosition());
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bool all_ok = true;
    for (size_t i = 0; i < drivers_.size(); ++i) {
      if (drivers_[i]->GetModeOfOperationRaw() != new_mode) {
        all_ok = false;
        break;
      }
    }

    response->success = all_ok;
    response->message = all_ok ? "OK" : "Mode switch timeout";
  });

  move_j_sub_ = this->create_subscription<kungshu_msgs::msg::MoveJCommand>(
  "move_j_command",
  qos_best_effort,
  [this](const kungshu_msgs::msg::MoveJCommand::SharedPtr msg) {

    std::vector<double> target_pos(total_joints_), max_vel(total_joints_), max_acc(total_joints_);
    for (int i = 0; i < total_joints_; ++i) {
      target_pos[i] = msg->pos[i];
      max_vel[i]    = msg->vel[i];
      max_acc[i]    = msg->acc[i];
    }

    MoveJ(target_pos, max_vel, max_acc);
  });

  tau_sub_ = this->create_subscription<kungshu_msgs::msg::ArmTorqueCommand>(
  "tau_command",
  qos_best_effort,
  [this](const kungshu_msgs::msg::ArmTorqueCommand::SharedPtr msg)
  {
    /* 1. Status check */
    for (size_t i = 0; i < drivers_.size(); ++i) {
      if (drivers_[i]->getDriverState(0) != DriveState::OperationEnabled) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Joint %zu is not in OperationEnabled — torque command ignored", i);
        return;
      }
      if (drivers_[i]->GetModeOfOperationRaw() != 10) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Joint %zu is not in CST (10) — torque command ignored", i);
        return;
      }
    }

    /* 2. Write torque */
    for (size_t i = 0; i < drivers_.size(); ++i) {
      drivers_[i]->SetTargetTorque(msg->tau[i]);
    }
  });

  publish_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      kungshu_msgs::msg::ArmState state;

      for (size_t i = 0; i < drivers_.size(); i++) {
        state.header.stamp = this->now();
        state.q[i] = drivers_[i]->GetPosition();
        state.dq[i] = drivers_[i]->GetVelocity();
        state.tau[i] = drivers_[i]->GetTorque();
        state.load[i] = drivers_[i]->GetLoadTorque();
        state.status[i] = drivers_[i]->GetStatus();
        state.temperature[i] = drivers_[i]->GetStatusWordRaw();
      }

      state_publisher_->publish(state);
      std::this_thread::sleep_for(std::chrono::microseconds(4000));
    }
  });

  time_sync_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {

      auto time_start = std::chrono::high_resolution_clock::now();

      loop_mutex_.lock();
      if (is_running_) {
        bool all_finished = true;
        for (size_t arm = 0; arm < otg_.size(); arm++) {
          auto res = otg_[arm].update(input_[arm], output_[arm]);
          if (res == ruckig::Working) {
            all_finished = false;
            for (int i = 0; i < joints_per_arm_; i++) {
              int driver_idx = arm * joints_per_arm_ + i;
              drivers_[driver_idx]->SetTargetPosition(output_[arm].new_position[i]);
            }
            output_[arm].pass_to_input(input_[arm]);
          }
          else if (res == ruckig::Error) {
            RCLCPP_ERROR(this->get_logger(), "Ruckig error on arm %zu", arm);
          }
        }
        if (all_finished) {
          is_running_ = false;
        }
      }
      loop_mutex_.unlock();

      Fieldbus::LoopOnce();

      auto time_end = std::chrono::high_resolution_clock::now();

      int elasped_us = std::chrono::duration_cast<std::chrono::microseconds>(time_end - time_start).count();

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

  // Initialize Ruckig instances (one per arm)
  for (int arm = 0; arm < num_arms_; arm++) {
    otg_.emplace_back(0.004);
    input_.emplace_back();
    output_.emplace_back();
  }

  // Initialize drivers
  for (size_t i = 0; i < drivers_.size(); i++) {
    drivers_[i]->SetModeOfOperationRaw(8); // Cyclic Synchronous Position Mode
    drivers_[i]->SetTargetPosition(drivers_[i]->GetPosition());
    drivers_[i]->SetTargetVelocity(0);
    drivers_[i]->SetTargetTorque(0);
  }

  // Initialize Ruckig input
  for (int arm = 0; arm < num_arms_; arm++) {
    for (int i = 0; i < joints_per_arm_; i++) {
      int driver_idx = arm * joints_per_arm_ + i;
      input_[arm].current_position[i] = drivers_[driver_idx]->GetPosition();
      input_[arm].current_velocity[i] = 0.0;
      input_[arm].current_acceleration[i] = 0.0;
    }
    input_[arm].synchronization = ruckig::Synchronization::Phase;
  }
}

void ArmNode::MoveJ(const std::vector<double>& target_pos,
                             const std::vector<double>& max_vel,
                             const std::vector<double>& max_acc) {
  loop_mutex_.lock();
  for (int arm = 0; arm < num_arms_; arm++) {
    for (int i = 0; i < joints_per_arm_; i++) {
      int driver_idx = arm * joints_per_arm_ + i;
      input_[arm].target_position[i] = target_pos[driver_idx];
      input_[arm].target_velocity[i] = 0.0;
      input_[arm].target_acceleration[i] = 0.0;
      input_[arm].max_velocity[i] = max_vel[driver_idx];
      input_[arm].max_acceleration[i] = max_acc[driver_idx];
      input_[arm].max_jerk[i] = max_acc[driver_idx] * 10;
    }
  }

  loop_mutex_.unlock();
  is_running_ = true;
}

void ArmNode::command_callback(const kungshu_msgs::msg::ArmServoCommand& msg) {
  for (size_t i = 0; i < drivers_.size(); i++) {
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
