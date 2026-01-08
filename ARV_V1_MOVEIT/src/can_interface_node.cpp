// ============================================================
// SocketCAN 接口节点 (SocketCAN Interface Node)
//
// 职责:
//   - 订阅 /effort_controller/commands 获取力矩指令
//   - 通过 SocketCAN 发送 MIT 格式控制帧到 6 个电机
//   - 接收电机反馈帧并发布到 /joint_states
//
// Stage: SIM2REAL - SocketCAN 通信
// Author: ARV V1 Team
// Date: 2025-01-08
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>

#include "mit_protocol.hpp"

class CanInterfaceNode : public rclcpp::Node
{
public:
    CanInterfaceNode() : Node("can_interface_node"),
                         socket_fd_(-1),
                         running_(false)
    {
        // 1. 声明参数
        this->declare_parameter("can_interface", "can0");
        this->declare_parameter("publish_rate", 1000.0);  // 1kHz 目标

        // 2. 获取参数
        can_interface_ = this->get_parameter("can_interface").as_string();
        double rate = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "[INIT] CAN interface: %s", can_interface_.c_str());

        // 3. 初始化电机配置
        motor_config_ = MitProtocol::getDefaultMotorConfig();

        // 4. 初始化 SocketCAN
        if (!initSocketCAN()) {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to initialize SocketCAN");
            return;
        }

        // 5. 初始化 ROS2 通信
        initROS2Communication();

        // 6. 启动接收线程
        running_ = true;
        receive_thread_ = std::thread(&CanInterfaceNode::receiveLoop, this);

        RCLCPP_INFO(this->get_logger(), "[OK] CAN interface node started");
    }

    ~CanInterfaceNode()
    {
        running_ = false;

        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }

        if (socket_fd_ >= 0) {
            close(socket_fd_);
        }

        RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] CAN interface closed");
    }

private:
    // ========== 成员变量 ==========
    std::string can_interface_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::thread receive_thread_;

    std::array<MitProtocol::MotorConfig, MitProtocol::NUM_JOINTS> motor_config_;

    // ROS2 通信
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // 数据缓存 (预分配，避免实时分配)
    std::mutex data_mutex_;
    std::array<float, 6> current_positions_ = {0};
    std::array<float, 6> current_velocities_ = {0};
    std::array<float, 6> current_torques_ = {0};

    // ========== 初始化函数 ==========

    bool initSocketCAN()
    {
        // 创建 CAN 套接字
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to create CAN socket: %s",
                         strerror(errno));
            return false;
        }

        // 获取接口索引
        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to get interface index: %s",
                         strerror(errno));
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // 绑定到 CAN 接口
        struct sockaddr_can addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to bind CAN socket: %s",
                         strerror(errno));
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "[OK] SocketCAN initialized on %s",
                    can_interface_.c_str());
        return true;
    }

    void initROS2Communication()
    {
        // 订阅力矩命令
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", 10,
            std::bind(&CanInterfaceNode::torqueCallback, this, std::placeholders::_1));

        // 发布关节状态
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 10);

        RCLCPP_INFO(this->get_logger(), "[OK] ROS2 communication initialized");
    }

    // ========== 回调函数 ==========

    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != MitProtocol::NUM_JOINTS) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[WARN] Invalid torque size: %zu", msg->data.size());
            return;
        }

        // 发送力矩到每个电机
        for (size_t i = 0; i < MitProtocol::NUM_JOINTS; ++i) {
            float torque = static_cast<float>(msg->data[i]);
            sendTorqueCommand(i, torque);
        }
    }

    // ========== CAN 收发函数 ==========

    void sendTorqueCommand(size_t joint_idx, float torque)
    {
        if (socket_fd_ < 0 || joint_idx >= MitProtocol::NUM_JOINTS) return;

        // 编码纯力矩控制帧
        struct can_frame frame = MitProtocol::encodeTorqueOnlyFrame(
            motor_config_[joint_idx].ctrl_id, torque);

        // 发送
        ssize_t nbytes = write(socket_fd_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[WARN] CAN write failed for joint %zu", joint_idx);
        }
    }

    void receiveLoop()
    {
        struct can_frame frame;

        while (running_) {
            ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
            if (nbytes < 0) {
                if (errno == EINTR) continue;
                RCLCPP_ERROR(this->get_logger(), "[ERROR] CAN read error: %s",
                             strerror(errno));
                break;
            }

            if (nbytes == sizeof(frame)) {
                processReceivedFrame(frame);
            }
        }
    }

    void processReceivedFrame(const struct can_frame& frame)
    {
        // 查找对应的关节索引
        int joint_idx = -1;
        for (size_t i = 0; i < MitProtocol::NUM_JOINTS; ++i) {
            if (frame.can_id == motor_config_[i].feedback_id) {
                joint_idx = static_cast<int>(i);
                break;
            }
        }

        if (joint_idx < 0) return;  // 未知 CAN ID

        // 解码反馈帧
        MitProtocol::MotorFeedback fb = MitProtocol::decodeFeedbackFrame(frame);

        // 更新缓存
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_positions_[joint_idx] = fb.position;
            current_velocities_[joint_idx] = fb.velocity;
            current_torques_[joint_idx] = fb.torque;
        }

        // 每收到 Joint 6 反馈后发布完整状态
        if (joint_idx == 5) {
            publishJointStates();
        }
    }

    void publishJointStates()
    {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        msg.position.resize(6);
        msg.velocity.resize(6);
        msg.effort.resize(6);

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < 6; ++i) {
                msg.position[i] = current_positions_[i];
                msg.velocity[i] = current_velocities_[i];
                msg.effort[i] = current_torques_[i];
            }
        }

        joint_state_pub_->publish(msg);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
