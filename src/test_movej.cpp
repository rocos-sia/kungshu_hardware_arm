#include <rclcpp/rclcpp.hpp>
#include <kungshu_msgs/srv/set_enable.hpp>
#include <kungshu_msgs/srv/move_j.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_movej_client");

  auto enable_cli = node->create_client<kungshu_msgs::srv::SetEnable>("set_enable_service");
  auto movej_cli  = node->create_client<kungshu_msgs::srv::MoveJ>("movej_service");

  while (!enable_cli->wait_for_service(1s) || !movej_cli->wait_for_service(1s)) {
    RCLCPP_INFO(node->get_logger(), "waiting for services...");
  }

  RCLCPP_INFO(node->get_logger(), "=== 上使能 ===");
  auto enable_req = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
  enable_req->enable = true;
  
  auto enable_future = enable_cli->async_send_request(enable_req);
  if (rclcpp::spin_until_future_complete(node, enable_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "上使能失败！");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "上使能成功，等待 1 s 让驱动器稳定...");
  rclcpp::sleep_for(1s);


  auto movej_req = std::make_shared<kungshu_msgs::srv::MoveJ::Request>();

  movej_req->pos = {
     0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,
    -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7
  };
  std::fill(movej_req->vel.begin(), movej_req->vel.end(), 0.3);
  std::fill(movej_req->acc.begin(), movej_req->acc.end(), 1.0);


  RCLCPP_INFO(node->get_logger(), "=== 发送 MoveJ ===");
  auto movej_future = movej_cli->async_send_request(movej_req);
  if (rclcpp::spin_until_future_complete(node, movej_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "MoveJ 调用成功！");
  } else {
    RCLCPP_ERROR(node->get_logger(), "MoveJ 调用失败！");
  }

  rclcpp::shutdown();
  return 0;
}