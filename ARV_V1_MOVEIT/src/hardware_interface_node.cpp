// 硬件接口节点: 串口发送力矩指令, 接收关节状态发布至/joint_states
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <io_context/io_context.hpp>
#include <serial_driver/serial_driver.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include "serial_protocol.hpp"

class HardwareInterfaceNode : public rclcpp::Node
{
public:
    HardwareInterfaceNode() : Node("hardware_interface_node"),
                              num_joints_(6),
                              running_(false),
                              simulation_mode_(false)
    {
        // 1. 声明参数
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 921600);
        this->declare_parameter("publish_rate", 200.0);    // 100Hz 发布频率
        this->declare_parameter("simulation_mode", false); // 新增：仿真模式参数

        // 2. 获取参数
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();
        double rate = this->get_parameter("publish_rate").as_double();
        simulation_mode_ = this->get_parameter("simulation_mode").as_bool();

        RCLCPP_INFO(this->get_logger(), "[INIT] Serial port: %s, Baud: %d",
                    port.c_str(), baud);

        if (simulation_mode_)
        {
            RCLCPP_INFO(this->get_logger(), "[SIMULATION MODE] Serial TX only, no RX feedback");
        }

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

        if (simulation_mode_)
        {
            // 仿真模式：不启动串口接收线程，等待MuJoCo反馈
            RCLCPP_INFO(this->get_logger(), "[OK] Simulation mode - Serial TX only, waiting for MuJoCo feedback");
        }
        else
        {
            // 真机模式：启动串口接收线程
            receive_thread_ = std::thread(&HardwareInterfaceNode::receiveLoop, this);
            RCLCPP_INFO(this->get_logger(), "[OK] Hardware mode - Serial RX/TX enabled");
        }

        RCLCPP_INFO(this->get_logger(), "[OK] Hardware interface node started");
    }

    ~HardwareInterfaceNode()
    {
        running_ = false;

        // 先关闭串口，解除阻塞读，确保线程可退出
        try
        {
            if (serial_port_ && serial_port_->is_open())
            {
                serial_port_->close();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "[WARN] Serial close: %s", e.what());
        }

        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Hardware interface closed");
    }

private:
    // ========== 成员变量 ==========
    int num_joints_;
    std::atomic<bool> running_;
    bool simulation_mode_; // 新增：是否为仿真模式（不从串口读取）

    // 串口
    std::string device_name_;
    uint32_t baud_rate_{0};
    std::unique_ptr<drivers::common::IoContext> io_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
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
            device_name_ = port;
            baud_rate_ = static_cast<uint32_t>(baud);

            // IoContext 内部会启动 worker 线程
            io_ctx_ = std::make_unique<drivers::common::IoContext>(1);
            serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_ctx_);

            drivers::serial_driver::SerialPortConfig config(
                baud_rate_,
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE);

            serial_driver_->init_port(device_name_, config);
            serial_port_ = serial_driver_->port();
            serial_port_->open();

            return serial_port_ && serial_port_->is_open();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial init: %s", e.what());
            return false;
        }
    }

    bool ensureSerialOpen(std::chrono::milliseconds backoff)
    {
        if (serial_port_ && serial_port_->is_open())
        {
            return true;
        }

        try
        {
            if (!serial_port_)
            {
                // 极端情况下（initSerial 未完成）尝试重新 init
                return initSerial(device_name_.empty() ? std::string("/dev/ttyS4") : device_name_,
                                  static_cast<int>(baud_rate_ == 0 ? 921600 : baud_rate_));
            }
            serial_port_->open();
            return serial_port_->is_open();
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "[WARN] Serial reopen failed: %s", e.what());
            std::this_thread::sleep_for(backoff);
            return false;
        }
    }

    bool readExact(uint8_t *dst, size_t len)
    {
        size_t received = 0;
        std::vector<uint8_t> tmp;
        tmp.reserve(256);

        while (running_ && received < len)
        {
            if (!serial_port_ || !serial_port_->is_open())
            {
                return false;
            }

            const size_t need = len - received;
            tmp.assign(need, 0);

            size_t n = 0;
            try
            {
                n = serial_port_->receive(tmp);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial receive: %s", e.what());
                try
                {
                    serial_port_->close();
                }
                catch (...)
                {
                }
                return false;
            }

            if (n == 0)
            {
                continue;
            }

            if (n > need)
            {
                n = need;
            }

            std::memcpy(dst + received, tmp.data(), n);
            received += n;
        }

        return received == len;
    }

    void initROS2Communication()
    {
        // 订阅力矩命令
        torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands",
            10,
            std::bind(&HardwareInterfaceNode::torqueCallback, this, std::placeholders::_1));

        // 发布关节状态到标准话题 (MoveIt/RViz/数字孪生都订阅此话题)
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            10);
        RCLCPP_INFO(this->get_logger(), "[INIT] Publishing to /joint_states");
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
        if (!serial_port_ || !serial_port_->is_open())
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
            const size_t sent = serial_port_->send(packet);
            if (sent != packet.size())
            {
                RCLCPP_WARN(this->get_logger(), "[WARN] Partial send: %zu/%zu", sent, packet.size());
            }
            // RCLCPP_DEBUG(this->get_logger(), "[TX] Sent %zu bytes", packet.size());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Send failed: %s", e.what());
            try
            {
                serial_port_->close();
            }
            catch (...)
            {
            }
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
            if (!ensureSerialOpen(std::chrono::milliseconds(200)))
            {
                continue;
            }

            try
            {
                switch (state)
                {
                case WAIT_SOF:
                {
                    uint8_t byte = 0;
                    if (!readExact(&byte, 1))
                    {
                        state = WAIT_SOF;
                        break;
                    }

                    if (byte == SerialProtocol::SOF)
                    {
                        buffer.clear();
                        buffer.push_back(byte);
                        state = READ_LEN;
                    }
                }
                break;

                case READ_LEN:
                {
                    uint8_t len_bytes[2] = {0, 0};
                    if (!readExact(len_bytes, 2))
                    {
                        state = WAIT_SOF;
                        break;
                    }
                    buffer.push_back(len_bytes[0]);
                    buffer.push_back(len_bytes[1]);

                    data_len = len_bytes[0] | (len_bytes[1] << 8);
                    state = READ_HEADER_CRC;
                }
                break;

                case READ_HEADER_CRC:
                {
                    uint8_t byte = 0;
                    if (!readExact(&byte, 1))
                    {
                        state = WAIT_SOF;
                        break;
                    }
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

                    {
                        std::vector<uint8_t> body(body_size);
                        if (!readExact(body.data(), body_size))
                        {
                            state = WAIT_SOF;
                            break;
                        }
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
                try
                {
                    if (serial_port_ && serial_port_->is_open())
                    {
                        serial_port_->close();
                    }
                }
                catch (...)
                {
                }
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
