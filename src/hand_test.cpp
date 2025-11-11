// #include <rclcpp/rclcpp.hpp>
// #include <example_interfaces/msg/int32_multi_array.hpp>

// #include <iostream>
// #include <thread>
// #include <cstring>
// #include <unistd.h>
// #include <linux/can.h>
// #include <linux/can/raw.h>
// #include <net/if.h>
// #include <sys/ioctl.h>
// #include <sys/socket.h>
// #include <iomanip>
// #include <vector>

// class CanController : public rclcpp::Node
// {
// public:
//     CanController()
//     : Node("can_multi_node")
//     {
//         // 初始化 CAN 套接字
//         initCanSocket("can0");

//         // 接收线程
//         recv_thread_ = std::thread(&CanController::receiveThread, this);
//         recv_thread_.detach();

//         // ROS2 订阅者
//         subscription_ = this->create_subscription<example_interfaces::msg::Int32MultiArray>(
//             "/multi_int_topic", 10,
//             std::bind(&CanController::multiArrayCallback, this, std::placeholders::_1)
//         );

//         frame_ids_ = {0x05738001, 0x05740001, 0x05748001, 0x05750001, 0x05758001, 0x05760001};
//     }

//     ~CanController()
//     {
//         if (can_socket_ > 0) {
//             close(can_socket_);
//         }
//     }

// private:
//     int can_socket_;
//     std::thread recv_thread_;
//     rclcpp::Subscription<example_interfaces::msg::Int32MultiArray>::SharedPtr subscription_;
//     std::vector<uint32_t> frame_ids_;

//     void initCanSocket(const char* ifname)
//     {
//         can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
//         if (can_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "CAN socket creation failed");
//             throw std::runtime_error("CAN socket creation failed");
//         }

//         struct ifreq ifr;
//         strcpy(ifr.ifr_name, ifname);
//         if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "ioctl failed");
//             throw std::runtime_error("ioctl failed");
//         }

//         struct sockaddr_can addr;
//         addr.can_family = AF_CAN;
//         addr.can_ifindex = ifr.ifr_ifindex;

//         if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
//             RCLCPP_ERROR(this->get_logger(), "bind failed");
//             throw std::runtime_error("bind failed");
//         }

//         // 设置过滤器：接收所有扩展帧
//         struct can_filter rfilter;
//         rfilter.can_id = 0;
//         rfilter.can_mask = 0;
//         setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
//     }

//     void receiveThread()
//     {
//         struct can_frame frame;
//         int nbytes;

//         while (rclcpp::ok()) {
//             nbytes = read(can_socket_, &frame, sizeof(frame));
//             if (nbytes < 0) {
//                 RCLCPP_ERROR(this->get_logger(), "CAN read error");
//                 continue;
//             }

//             // 打印 CAN 报文
//             std::cout << "[DEBUG] Received CAN ID: 0x" 
//                       << std::hex << std::setw(8) << std::setfill('0') 
//                       << frame.can_id << std::dec << " Data:";
//             for (int i = 0; i < frame.can_dlc; ++i) {
//                 std::cout << " 0x" << std::hex << std::setw(2) << std::setfill('0') 
//                           << static_cast<int>(frame.data[i]);
//             }
//             std::cout << std::dec << std::endl;
//         }
//     }

//     void multiArrayCallback(const example_interfaces::msg::Int32MultiArray::SharedPtr msg)
//     {
//         if (msg->data.size() < frame_ids_.size()) {
//             RCLCPP_WARN(this->get_logger(), "Received data size < %zu, ignore", frame_ids_.size());
//             return;
//         }

//         struct can_frame frame;
//         memset(&frame, 0, sizeof(frame));
//         frame.can_dlc = 2; // 每个手指用两个字节发送

//         // 遍历数组 + 帧 ID
//         for (size_t i = 0; i < frame_ids_.size(); ++i) {
//             frame.can_id = (frame_ids_[i] & CAN_EFF_MASK) | CAN_EFF_FLAG;
//             uint16_t v = static_cast<uint16_t>(msg->data[i] & 0xFFFF);
//             frame.data[0] = v & 0xFF;          // 低字节
//             frame.data[1] = (v >> 8) & 0xFF;   // 高字节

//             int nbytes = write(can_socket_, &frame, sizeof(frame));
//             if (nbytes != sizeof(frame)) {
//                 RCLCPP_ERROR(this->get_logger(), "CAN write error for frame %zu", i);
//             } else {
//                 RCLCPP_INFO(this->get_logger(),
//                     "Sent CAN ID: 0x%08X Data: 0x%02X 0x%02X",
//                     frame.can_id, frame.data[0], frame.data[1]);
//             }
//         }
//     }
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CanController>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <cstdint>


int  can_init();                     
bool send_positions(int sock,
                     const std::vector<uint16_t>& pos);

static const std::vector<uint32_t> FRAME_IDS = {
    0x05738001, 0x05740001, 0x05748001,
    0x05750001, 0x05758001, 0x05760001
};
static const std::vector<uint32_t> FRAME_IDS2 = {
    0x05738002, 0x05740002, 0x05748002,
    0x05750002, 0x05758002, 0x05760002
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
bool send_positions2(int sock, const std::vector<uint16_t>& pos)
{
    if (pos.size() != 6) { std::fprintf(stderr, "pos.size() != 6\n"); return false; }

    for (size_t i = 0; i < 6; ++i) {
        struct can_frame frame{};
        frame.can_id  = (FRAME_IDS2[i] & CAN_EFF_MASK) | CAN_EFF_FLAG;
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

int main()
{
    const std::vector<uint16_t> TARGET_POS = {100, 100, 100, 100, 1000, 1000};
    const std::vector<uint16_t> OPEN_POS = {1000, 1000, 1000, 1000, 1000, 0};

    int sock = can_init();
    if (sock < 0) return 1;
    send_positions(sock, TARGET_POS);
    usleep(1000000);
    send_positions(sock, TARGET_POS);
    // while(1){
    //     bool ok = send_positions2(sock, TARGET_POS);
    //     if(ok) std::printf("send_positions success\n");
    //     usleep(500000);
    // }
    bool ok = send_positions2(sock, TARGET_POS);
    if(ok) std::printf("send_positions success\n");
    close(sock);
    return ok ? 0 : 1;
}