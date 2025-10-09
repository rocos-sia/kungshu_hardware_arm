// #include <rclcpp/rclcpp.hpp>
// #include <kungshu_msgs/srv/set_enable.hpp>
// #include <kungshu_msgs/srv/move_j.hpp>

// using namespace std::chrono_literals;

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("test_movej_client");

//   auto enable_cli = node->create_client<kungshu_msgs::srv::SetEnable>("set_enable_service");
//   auto movej_cli  = node->create_client<kungshu_msgs::srv::MoveJ>("movej_service");

//   while (!enable_cli->wait_for_service(1s) || !movej_cli->wait_for_service(1s)) {
//     RCLCPP_INFO(node->get_logger(), "waiting for services...");
//   }

//   RCLCPP_INFO(node->get_logger(), "=== 上使能 ===");
//   auto enable_req = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
//   enable_req->enable = true;
  
//   auto enable_future = enable_cli->async_send_request(enable_req);
//   if (rclcpp::spin_until_future_complete(node, enable_future) !=
//       rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_ERROR(node->get_logger(), "上使能失败！");
//     return 1;
//   }
//   RCLCPP_INFO(node->get_logger(), "上使能成功，等待 1 s 让驱动器稳定...");
//   rclcpp::sleep_for(1s);


//   auto movej_req = std::make_shared<kungshu_msgs::srv::MoveJ::Request>();

//   movej_req->pos = {
//      0.1,  0.2,  0.3,  0.4,  0.5,  0.6,  0.7,
//     -0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7
//   };
//   std::fill(movej_req->vel.begin(), movej_req->vel.end(), 0.3);
//   std::fill(movej_req->acc.begin(), movej_req->acc.end(), 1.0);


//   RCLCPP_INFO(node->get_logger(), "=== 发送 MoveJ ===");
//   auto movej_future = movej_cli->async_send_request(movej_req);
//   if (rclcpp::spin_until_future_complete(node, movej_future) ==
//       rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_INFO(node->get_logger(), "MoveJ 调用成功！");
//   } else {
//     RCLCPP_ERROR(node->get_logger(), "MoveJ 调用失败！");
//   }

//   rclcpp::shutdown();
//   return 0;
// }
// #include <rclcpp/rclcpp.hpp>
// #include <kungshu_msgs/srv/set_enable.hpp>   
// #include <kungshu_msgs/msg/move_j_command.hpp> 
// #include <array>

// using namespace std::chrono_literals;

// int main(int argc, char ** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rclcpp::Node>("test_movej_topic");

//   auto enable_cli = node->create_client<kungshu_msgs::srv::SetEnable>("set_enable_service");
//   while (!enable_cli->wait_for_service(1s)) {
//     RCLCPP_INFO(node->get_logger(), "waiting for /set_enable_service ...");
//   }

//   RCLCPP_INFO(node->get_logger(), "=== set_enable ===");
//   auto enable_req = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
//   enable_req->enable = true;
//   auto enable_future = enable_cli->async_send_request(enable_req);
//   if (rclcpp::spin_until_future_complete(node, enable_future) !=
//       rclcpp::FutureReturnCode::SUCCESS)
//   {
//     RCLCPP_ERROR(node->get_logger(), "enabled failed!");
//     return 1;
//   }
//   RCLCPP_INFO(node->get_logger(), "enabled success, wait 1 s ...");
//   rclcpp::sleep_for(1s);

//   auto pub = node->create_publisher<kungshu_msgs::msg::MoveJCommand>(
//     "move_j_command", 10); 

//   kungshu_msgs::msg::MoveJCommand cmd;
//   kungshu_msgs::msg::MoveJCommand cmd1;
//   cmd.pos = {
//      -0.4,  0.5,  0.3,  2.0,  0.0,  0.0,  0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

//   };
//    cmd1.pos = {
//      -0.4,  0.2,  0.4,  2.5,  0.0,  0.0,  0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

//   };
//   std::fill(cmd.vel.begin(), cmd.vel.end(), 0.3);
//   std::fill(cmd.acc.begin(), cmd.acc.end(), 1.0);
//   std::fill(cmd1.vel.begin(), cmd1.vel.end(), 0.3);
//   std::fill(cmd1.acc.begin(), cmd1.acc.end(), 1.0);
//   RCLCPP_INFO(node->get_logger(), "=== publish move_j_command topic ===");
//   for(int i = 1; i < 4;i++)
//   {
//     pub->publish(cmd);
//     rclcpp::sleep_for(2s);
//     pub->publish(cmd1);
//     rclcpp::sleep_for(2s);
//   }
//   // pub->publish(cmd);
  
//   rclcpp::sleep_for(2s);

//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <kungshu_msgs/srv/set_enable.hpp>
#include <kungshu_msgs/msg/move_j_command.hpp>
#include <array>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <atomic>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <cstdint>
using namespace std::chrono_literals;

enum class Action : int8_t
{
  IDLE = 0,
  WAVE  = 1,
  POSE2 = 2,
  POSE3 = 3,
  STOP  = 99
};
int  can_init();                     
bool send_positions(int sock,
                     const std::vector<uint16_t>& pos);

static const std::vector<uint32_t> FRAME_IDS = {
    0x05738001, 0x05740001, 0x05748001,
    0x05750001, 0x05758001, 0x05760001
};
int can_init()
{
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) { std::perror("socket"); return -1; }

    struct ifreq ifr{};
    std::strcpy(ifr.ifr_name, "can0");
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) { std::perror("ioctl"); close(sock); return -1; }

    struct sockaddr_can addr{};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        std::perror("bind"); close(sock); return -1;
    }
    return sock;   
}

bool send_positions(int sock, const std::vector<uint16_t>& pos)
{
    if (pos.size() != 6) { std::fprintf(stderr, "pos.size() != 6\n"); return false; }

    for (size_t i = 0; i < 6; ++i) {
        struct can_frame frame{};
        frame.can_id  = (FRAME_IDS[i] & CAN_EFF_MASK) | CAN_EFF_FLAG;
        frame.can_dlc = 2;
        frame.data[0] =  pos[i] & 0xFF;
        frame.data[1] = (pos[i] >> 8) & 0xFF;

        if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
            std::perror("write"); return false;
        }
        std::printf("Sent 0x%08X  [%02X %02X]\n",
                    frame.can_id, frame.data[0], frame.data[1]);
    }
    return true;
}
std::atomic<Action> g_cmd(Action::IDLE);   // 全局键盘命令

/* -------------- 键盘监听线程 -------------- */
void kb_hit_thread()
{
  struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  char c;
  while (rclcpp::ok())
  {
    if (read(STDIN_FILENO, &c, 1) == 1)
    {
      switch (c)
      {
        case '1': g_cmd = Action::WAVE;  break;
        case '2': g_cmd = Action::POSE2; break;
        case '3': g_cmd = Action::POSE3; break;
        case 'q':
        case 'Q': g_cmd = Action::STOP;  break;
      }
    }
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

/* -------------- 主节点 -------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  int sock = can_init();
    if (sock < 0) return 1;
  auto node = std::make_shared<rclcpp::Node>("test_movej_topic");

  /* 1. 等待并使能 */
  auto enable_cli = node->create_client<kungshu_msgs::srv::SetEnable>("set_enable_service");
  while (!enable_cli->wait_for_service(1s) && rclcpp::ok())
    RCLCPP_INFO(node->get_logger(), "waiting for /set_enable_service ...");
  auto enable_req = std::make_shared<kungshu_msgs::srv::SetEnable::Request>();
  enable_req->enable = true;
  auto enable_future = enable_cli->async_send_request(enable_req);
  if (rclcpp::spin_until_future_complete(node, enable_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "enable failed!");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "enabled success, wait 1 s ...");
  rclcpp::sleep_for(1s);

  /* 2. 发布者 */
  auto pub = node->create_publisher<kungshu_msgs::msg::MoveJCommand>("move_j_command", 10);

  /* 3. 预定义三条轨迹 */
  kungshu_msgs::msg::MoveJCommand cmd_wave,cmd_wave1, cmd2, cmd3;
  cmd_wave.pos = {
     -0.4,  0.5,  0.3,  2.0,  0.0,  0.0,  0.0,
    1.0, -1.0, 0.0, 2.0, 0.0, 0.0, 0.0

  };
  cmd_wave1.pos = {
     -0.4,  0.2,  0.4,  2.5,  0.0,  0.0,  0.0,
    1.0, -1.0, 0.0, 2.0, 0.0, 0.0, 0.0};
  cmd2.pos = { -0.4,  0.5,  0.3,  2.0,  0.0,  0.0,  0.0,
               2.4,  0.5,  0.3,  2.0,  0.0,  0.0,  0.0};
  cmd3.pos = {0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1.5};
  std::vector<uint16_t> TARGET_POS = {100, 100, 100, 100, 100,1000};
  // for (auto &c : {cmd_wave,cmd_wave1, cmd2, cmd3})
  // {
  //   std::fill(c.vel.begin(), c.vel.end(), 0.3);
  //   std::fill(c.acc.begin(), c.acc.end(), 1.0);
  // }
  auto fill_array = [](auto& arr, double v){
    std::fill(arr.begin(), arr.end(), v);   // 现在 arr 是普通成员，可写
  };

  fill_array(cmd_wave.vel, 0.3);
  fill_array(cmd_wave.acc, 1.0);
  fill_array(cmd_wave1.vel, 0.3);
  fill_array(cmd_wave1.acc, 1.0);
  fill_array(cmd2.vel, 0.3);
  fill_array(cmd2.acc, 1.0);
  fill_array(cmd3.vel, 0.3);
  fill_array(cmd3.acc, 1.0);

  /* 4. 启动键盘线程 */
  std::thread kb_thread(kb_hit_thread);

  /* 5. 主循环 */
  rclcpp::Rate loop(100);          // 10 ms 周期
  Action  last_cmd = Action::IDLE;
  bool    side = true;            // 挥手 flip-flop 标志
  while (rclcpp::ok())
  {
    Action cur = g_cmd.load();
    if (cur == Action::STOP) break;

    if (cur != last_cmd)          // 状态切换时打印一次
    {
      RCLCPP_INFO(node->get_logger(), "Switch to action %d", static_cast<int>(cur));
      last_cmd = cur;
    }

    switch (cur)
    {
      case Action::WAVE:
        send_positions(sock, TARGET_POS);
        
        pub->publish(side ? cmd_wave : cmd_wave1);
        rclcpp::sleep_for(2s);
        side = !side;
        break;
      case Action::POSE2:
        pub->publish(cmd2);
        break;
      case Action::POSE3:
        send_positions(sock, TARGET_POS);
        pub->publish(cmd3);
        break;
      default:
        break;                    // IDLE，什么都不发
    }

    rclcpp::spin_some(node);
    loop.sleep();
  }

  /* 6. 退出：急停可选 */
  RCLCPP_WARN(node->get_logger(), "User pressed 'q', disable robot ...");
  enable_req->enable = false;
  enable_cli->async_send_request(enable_req);

  kb_thread.join();
  rclcpp::shutdown();
  return 0;
}
