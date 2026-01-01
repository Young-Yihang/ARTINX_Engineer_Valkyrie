// ============================================================
// 硬件接口节点 (Hardware Interface Node)
//
// 职责:
//   - 订阅 /effort_controller/commands 获取力矩指令
//   - 通过串口发送力矩到下位机
//   - 从串口接收关节状态 (位置、速度)
//   - 发布到 /hardware_joint_states 供闭环控制
//
// Stage: SIM2REAL
// Author: ARV V1 Team
// Date: 2025-11-04
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <atomic>
#include "serial_protocol.hpp"

class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode() : Node("hardware_interface_node"),
                              num_joints_(6),
                              running_(false)
    {
        // 1. 声明参数
        this->declare_parameter("serial_port", "/dev/ttyS4");
        this->declare_parameter("baud_rate", 921600);
        this->declare_parameter("publish_rate", 100.0); // 100Hz 发布频率

        // 2. 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        double rate = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "[INIT] Serial port: %s, Baud: %d",
                    port.c_str(), baud);

        // 3. 初始化串口
        if (!initSerial(port, baud))
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to open serial port");
            return;
        }

        // 4. 初始化 ROS2 通信
        initROS2Communication();

        // 5. 启动收发线程
        running_ = true;
        receive_thread_ = std::thread(&HardwareInterfaceNode::receiveLoop, this);

        RCLCPP_INFO(this->get_logger(), "[OK] Hardware interface node started");
    }

    ~HardwareInterfaceNode()
    {
        running_ = false;
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }
        if (serial_ && serial_->isOpen())
        {
            serial_->close();
        }
        RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Hardware interface closed");
    }

private:
    // ========== 成员变量 ==========
    int num_joints_;
    std::atomic<bool> running_;

    // 串口
    std::unique_ptr<serial::Serial> serial_;
    std::thread receive_thread_;

    // ROS2 通信
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // 数据缓存
    std::mutex data_mutex_;
    float current_torques_[6] = {0};
    float current_positions_[6] = {0};
    float current_velocities_[6] = {0};

    // ========== 初始化函数 ==========

    bool initSerial(const std::string &port, int baud)
    {
        try
        {
            serial_ = std::make_unique<serial::Serial>(
                port,
                baud,
                serial::Timeout::simpleTimeout(100) // 100ms 超时
            );

            if (!serial_->isOpen())
            {
                return false;
            }

            // 清空缓冲区
            serial_->flush();

            return true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial init: %s", e.what());
            return false;
        }
    }

    void initROS2Communication()
    {
        // 订阅力矩命令
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands",
            10,
            std::bind(&HardwareInterfaceNode::torqueCallback, this, std::placeholders::_1));

        // 发布关节状态
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/hardware_joint_states",
            10);
    }

    // ========== 回调函数 ==========

    void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != static_cast<size_t>(num_joints_))
        {
            RCLCPP_WARN(this->get_logger(), "[WARN] Invalid torque size: %zu", msg->data.size());
            return;
        }

        // 1. 缓存力矩数据
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < num_joints_; ++i)
            {
                current_torques_[i] = static_cast<float>(msg->data[i]);
            }
        }

        // 2. 打包并发送
        sendTorqueCommand();
    }

    // ========== 串口收发函数 ==========

    void sendTorqueCommand()
    {
        if (!serial_ || !serial_->isOpen())
        {
            return;
        }

        SerialProtocol::TorqueCommand cmd;
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 6; ++i)
                cmd.torques[i] = current_torques_[i];
        }

        std::vector<uint8_t> packet = SerialProtocol::buildTorquePacket(cmd);

        try
        {
            serial_->write(packet);
            // RCLCPP_DEBUG(this->get_logger(), "[TX] Sent %zu bytes", packet.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Send failed: %s", e.what());
        }
    }

    void receiveLoop()
    {
        // State Machine for SEASKY Protocol
        // [SOF(1)] [Len(2)] [CRC8(1)] [Cmd(2)] [Flags(2)] [Payload(N)] [CRC16(2)]

        enum State
        {
            WAIT_SOF,
            READ_LEN,
            READ_HEADER_CRC,
            READ_BODY
        };
        State state = WAIT_SOF;

        std::vector<uint8_t> buffer;
        uint16_t data_len = 0;

        while (running_)
        {
            if (!serial_ || !serial_->isOpen())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            try
            {
                uint8_t byte;
                switch (state)
                {
                case WAIT_SOF:
                    if (serial_->read(&byte, 1) == 1)
                    {
                        if (byte == SerialProtocol::SOF)
                        {
                            buffer.clear();
                            buffer.push_back(byte);
                            state = READ_LEN;
                        }
                    }
                    break;

                case READ_LEN:
                    if (serial_->available() >= 2)
                    {
                        uint8_t len_bytes[2];
                        serial_->read(len_bytes, 2);
                        buffer.push_back(len_bytes[0]);
                        buffer.push_back(len_bytes[1]);

                        data_len = len_bytes[0] | (len_bytes[1] << 8);
                        state = READ_HEADER_CRC;
                    }
                    break;

                case READ_HEADER_CRC:
                    if (serial_->read(&byte, 1) == 1)
                    {
                        buffer.push_back(byte);

                        // Validate Header CRC8 (SOF + Len_L + Len_H)
                        uint8_t expected_crc = SerialProtocol::Get_CRC8_Check_Sum(buffer.data(), 3, 0xFF);
                        if (byte == expected_crc)
                        {
                            state = READ_BODY;
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "[RX] Header CRC Fail");
                            state = WAIT_SOF;
                        }
                    }
                    break;

                case READ_BODY:
                    // Body size = DataLen + 2 (CRC16)
                    // Note: DataLen includes CmdID(2) + Flags(2) + Payload(N)
                    size_t body_size = data_len + 2;

                    if (serial_->available() >= body_size)
                    {
                        std::vector<uint8_t> body(body_size);
                        serial_->read(body.data(), body_size);
                        buffer.insert(buffer.end(), body.begin(), body.end());

                        // Validate Whole Packet CRC16
                        uint16_t received_crc = body[body_size - 2] | (body[body_size - 1] << 8);
                        uint16_t calculated_crc = SerialProtocol::Get_CRC16_Check_Sum(buffer.data(), buffer.size() - 2, 0xFFFF);

                        if (received_crc == calculated_crc)
                        {
                            processPacket(buffer);
                        }
                        else
                        {
                            RCLCPP_WARN(this->get_logger(), "[RX] Body CRC Fail");
                        }
                        state = WAIT_SOF;
                    }
                    break;
                }
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "[ERROR] Receive: %s", e.what());
                state = WAIT_SOF;
            }
        }
    }

    void processPacket(const std::vector<uint8_t> &packet)
    {
        // Packet structure: [SOF(1)][Len(2)][CRC8(1)] [Cmd(2)][Flags(2)][Payload...][CRC16(2)]
        // Offset to CmdID is 4
        size_t offset = 4;
        uint16_t cmd_id = SerialProtocol::read_uint16(packet.data(), offset);
        uint16_t flags = SerialProtocol::read_uint16(packet.data(), offset); // offset becomes 8

        if (cmd_id == SerialProtocol::CMD_JOINT_FEEDBACK)
        {
            float positions[6];
            float velocities[6];

            for (int i = 0; i < 6; ++i)
                positions[i] = SerialProtocol::read_float(packet.data(), offset);
            for (int i = 0; i < 6; ++i)
                velocities[i] = SerialProtocol::read_float(packet.data(), offset);

            updateAndPublishJointStates(positions, velocities);
        }
    }

    void updateAndPublishJointStates(const float positions[6], const float velocities[6])
    {
        // 1. 更新缓存
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            std::memcpy(current_positions_, positions, sizeof(float) * 6);
            std::memcpy(current_velocities_, velocities, sizeof(float) * 6);
        }

        // 2. 发布 ROS2 消息
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->now();
        msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        msg.position.resize(6);
        msg.velocity.resize(6);
        for (int i = 0; i < 6; ++i)
        {
            msg.position[i] = positions[i];
            msg.velocity[i] = velocities[i];
        }

        joint_state_pub_->publish(msg);

        // RCLCPP_DEBUG(this->get_logger(), "[RX] Published joint states");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareInterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
