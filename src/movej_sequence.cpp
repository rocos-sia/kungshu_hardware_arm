//
// MoveJ Sequence Example
// Demonstrates sending multiple MoveJ waypoints in sequence
//

#include <rclcpp/rclcpp.hpp>
#include <kungshu_msgs/msg/move_j_command.hpp>
#include <kungshu_msgs/srv/set_enable.hpp>
#include <chrono>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class MoveJSequence : public rclcpp::Node {
public:
  MoveJSequence() : Node("movej_sequence") {
    // Publisher
    move_j_pub_ = this->create_publisher<kungshu_msgs::msg::MoveJCommand>(
        "move_j_command", rclcpp::QoS(10).best_effort());

    // Service client for enabling
    enable_client_ = this->create_client<kungshu_msgs::srv::SetEnable>("set_enable_service");

    // Wait for service
    RCLCPP_INFO(this->get_logger(), "Waiting for set_enable_service...");
    enable_client_->wait_for_service(5s);
  }

  void enable() {
    auto request = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
    request->enable = true;

    auto future = enable_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);

    if (future.get()->success) {
      RCLCPP_INFO(this->get_logger(), "Robot enabled successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to enable robot");
    }
  }

  void disable() {
    auto request = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
    request->enable = false;

    auto future = enable_client_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s);
    RCLCPP_INFO(this->get_logger(), "Robot disabled");
  }

  void moveJ(const std::vector<double>& pos, double vel = 0.3, double acc = 1.0) {
    kungshu_msgs::msg::MoveJCommand cmd;

    // Set position, velocity, acceleration for all joints
    for (size_t i = 0; i < 14; i++) {
      cmd.pos[i] = (i < pos.size()) ? pos[i] : 0.0;
      cmd.vel[i] = vel;
      cmd.acc[i] = acc;
    }

    move_j_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "MoveJ command sent");
  }

  void moveJBothArms(const std::vector<double>& left_pos,
                     const std::vector<double>& right_pos,
                     double vel = 0.3, double acc = 1.0) {
    std::vector<double> pos;
    pos.insert(pos.end(), left_pos.begin(), left_pos.end());
    pos.insert(pos.end(), right_pos.begin(), right_pos.end());
    moveJ(pos, vel, acc);
  }

  void waitSeconds(double seconds) {
    RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds...", seconds);
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(seconds * 1000)));
  }

private:
  rclcpp::Publisher<kungshu_msgs::msg::MoveJCommand>::SharedPtr move_j_pub_;
  rclcpp::Client<kungshu_msgs::srv::SetEnable>::SharedPtr enable_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveJSequence>();

  // Enable robot
  node->enable();
  node->waitSeconds(1.0);

  // ============================================
  // Example 1: Single arm (left arm only, 7 joints)
  // ============================================
  // Uncomment below for single arm mode:
  //
  // // Waypoint 1: Home position
  // node->moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  // node->waitSeconds(3.0);
  //
  // // Waypoint 2: Ready position
  // node->moveJ({0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0});
  // node->waitSeconds(3.0);
  //
  // // Waypoint 3: Extended position
  // node->moveJ({0.3, -0.3, 0.0, -0.8, 0.0, 0.3, 0.0});
  // node->waitSeconds(3.0);
  //
  // // Back to home
  // node->moveJ({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  // node->waitSeconds(3.0);

  // ============================================
  // Example 2: Dual arm (14 joints)
  // ============================================
  RCLCPP_INFO(node->get_logger(), "=== Starting MoveJ Sequence Demo ===");

  // Waypoint 1: Both arms home
  node->moveJBothArms(
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},   // Left arm
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}    // Right arm
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 1: Home");
  node->waitSeconds(3.0);

  // Waypoint 2: Both arms ready
  node->moveJBothArms(
    {0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0},  // Left arm
    {0.0,  0.5, 0.0,  1.0, 0.0, -0.5, 0.0}  // Right arm (mirrored)
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 2: Ready");
  node->waitSeconds(3.0);

  // Waypoint 3: Left arm wave
  node->moveJBothArms(
    {0.3, -0.3, 0.0, -0.8, 0.5, 0.3, 0.0},  // Left arm extended
    {0.0,  0.0, 0.0,  0.0, 0.0, 0.0, 0.0}   // Right arm home
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 3: Left Extended");
  node->waitSeconds(3.0);

  // Waypoint 4: Right arm wave
  node->moveJBothArms(
    {0.0,  0.0, 0.0,  0.0, 0.0, 0.0, 0.0},   // Left arm home
    {-0.3, 0.3, 0.0,  0.8, -0.5, -0.3, 0.0}  // Right arm extended
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 4: Right Extended");
  node->waitSeconds(3.0);

  // Waypoint 5: Both arms ready
  node->moveJBothArms(
    {0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0},   // Left arm
    {0.0,  0.5, 0.0,  1.0, 0.0, -0.5, 0.0}   // Right arm
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 5: Ready");
  node->waitSeconds(3.0);

  // Back to home
  node->moveJBothArms(
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
  );
  RCLCPP_INFO(node->get_logger(), "Waypoint 6: Home");
  node->waitSeconds(3.0);

  RCLCPP_INFO(node->get_logger(), "=== Sequence Complete ===");

  // Disable robot
  node->disable();

  rclcpp::shutdown();
  return 0;
}
