// 硬件接口节点: 串口发送力矩指令, 接收关节状态发布至/joint_states
#include <atomic>
#include <chrono>
#include <io_context/io_context.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <thread>

#include "serial_protocol.hpp"

class HardwareInterfaceNode : public rclcpp::Node {
public:
  HardwareInterfaceNode()
      : Node("hardware_interface_node"), num_joints_(7), running_(false), simulation_mode_(false) {
    // 1. 声明参数
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 921600);
    this->declare_parameter("publish_rate", 200.0);
    this->declare_parameter("simulation_mode", false);  // 新增：仿真模式参数
    this->declare_parameter("force_zero_torque",
                            false);  // 新增：强制零力矩开关（默认false允许正常控制）

    // 2. 获取参数
    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    double rate = this->get_parameter("publish_rate").as_double();
    simulation_mode_ = this->get_parameter("simulation_mode").as_bool();

    RCLCPP_INFO(this->get_logger(), "[INIT] Serial port: %s, Baud: %d", port.c_str(), baud);

    if (simulation_mode_) {
      RCLCPP_INFO(this->get_logger(), "[SIMULATION MODE] Serial TX only, no RX feedback");
    }

    // 3. 初始化串口（失败不退出，依赖自动重连）
    if (!initSerial(port, baud)) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Initial serial open failed: %s", port.c_str());
      RCLCPP_INFO(this->get_logger(),
                  "[INFO] Node will continue running and auto-reconnect when device available");
    } else {
      RCLCPP_INFO(this->get_logger(), "[OK] Serial port opened: %s @ %d baud", port.c_str(), baud);
    }

    // 4. 初始化 ROS2 通信
    initROS2Communication();

    // 5. 启动收发线程（无论串口是否打开）
    running_ = true;

    if (simulation_mode_) {
      // 仿真模式：不启动串口接收线程，等待MuJoCo反馈
      RCLCPP_INFO(this->get_logger(),
                  "[OK] Simulation mode - Serial TX only, waiting for MuJoCo feedback");
    } else {
      // 真机模式：启动串口接收线程（会自动重连）
      receive_thread_ = std::thread(&HardwareInterfaceNode::receiveLoop, this);
      RCLCPP_INFO(this->get_logger(),
                  "[OK] Hardware mode - Serial RX/TX enabled (auto-reconnect every 200ms)");
    }

    // 6. 启动200Hz发送定时器（解耦架构：独立于解算层）
    auto send_period = std::chrono::microseconds(5000);  // 5ms = 200Hz
    send_timer_ =
        this->create_wall_timer(send_period, std::bind(&HardwareInterfaceNode::sendLoop, this));

    last_torque_update_ = this->now();  // 初始化时间戳
    RCLCPP_INFO(this->get_logger(), "[OK] Send timer started at 200Hz (decoupled from controller)");

    // 7. 启动健康监控定时器 (5Hz)
    last_rx_activity_.store(std::chrono::steady_clock::now());
    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                            std::bind(&HardwareInterfaceNode::healthCheck, this));

    RCLCPP_INFO(this->get_logger(), "[OK] Hardware interface node started");
  }

  ~HardwareInterfaceNode() {
    running_ = false;

    // 先关闭串口，解除阻塞读，确保线程可退出
    try {
      if (serial_port_ && serial_port_->is_open()) {
        serial_port_->close();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Serial close: %s", e.what());
    }

    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Hardware interface closed");
  }

private:
  // ========== 成员变量 ==========
  int num_joints_;
  std::atomic<bool> running_;
  bool simulation_mode_;  // 新增：是否为仿真模式（不从串口读取）

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
  rclcpp::TimerBase::SharedPtr send_timer_;  // 新增：200Hz发送定时器

  // 数据缓存
  std::mutex data_mutex_;
  float current_positions_[7] = {0};
  float current_velocities_[7] = {0};

  // 力矩缓存（解耦架构）
  std::mutex torque_cache_mutex_;
  float cached_torques_[7] = {0};
  rclcpp::Time last_torque_update_;
  bool torque_data_valid_ = false;

  // Health monitoring
  std::atomic<std::chrono::steady_clock::time_point> last_rx_activity_;
  std::atomic<uint64_t> rx_packet_count_{0};
  std::atomic<uint64_t> tx_packet_count_{0};
  std::atomic<uint64_t> rx_crc_errors_{0};
  rclcpp::TimerBase::SharedPtr health_timer_;

  // ========== 初始化函数 ==========

  bool initSerial(const std::string &port, int baud) {
    try {
      device_name_ = port;
      baud_rate_ = static_cast<uint32_t>(baud);

      // IoContext 内部会启动 worker 线程
      io_ctx_ = std::make_unique<drivers::common::IoContext>(1);
      serial_driver_ = std::make_unique<drivers::serial_driver::SerialDriver>(*io_ctx_);

      drivers::serial_driver::SerialPortConfig config(
          baud_rate_, drivers::serial_driver::FlowControl::NONE,
          drivers::serial_driver::Parity::NONE, drivers::serial_driver::StopBits::ONE);

      serial_driver_->init_port(device_name_, config);
      serial_port_ = serial_driver_->port();
      serial_port_->open();

      return serial_port_ && serial_port_->is_open();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial init: %s", e.what());
      return false;
    }
  }

  bool ensureSerialOpen(std::chrono::milliseconds backoff) {
    if (serial_port_ && serial_port_->is_open()) {
      return true;
    }

    try {
      if (!serial_port_) {
        // 极端情况下（initSerial 未完成）尝试重新 init
        return initSerial(device_name_.empty() ? std::string("/dev/ttyS4") : device_name_,
                          static_cast<int>(baud_rate_ == 0 ? 921600 : baud_rate_));
      }
      serial_port_->open();
      return serial_port_->is_open();
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Serial reopen failed: %s", e.what());
      std::this_thread::sleep_for(backoff);
      return false;
    }
  }

  bool readExact(uint8_t *dst, size_t len) {
    // Safety check: reject unreasonable lengths
    if (len == 0 || len > 4096) {
      RCLCPP_ERROR(this->get_logger(), "[SAFETY] Invalid read length: %zu", len);
      return false;
    }

    size_t received = 0;
    std::vector<uint8_t> tmp;
    tmp.reserve(256);

    // Timeout protection
    auto start_time = std::chrono::steady_clock::now();
    auto last_activity = start_time;
    constexpr auto TOTAL_TIMEOUT = std::chrono::milliseconds(500);
    constexpr auto BYTE_TIMEOUT = std::chrono::milliseconds(200);

    while (running_ && received < len) {
      // Check total timeout
      auto now = std::chrono::steady_clock::now();
      if (now - start_time > TOTAL_TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(), "[TIMEOUT] Read total timeout (%zu/%zu bytes)", received,
                     len);
        return false;
      }

      // Check stall timeout (no data received)
      if (now - last_activity > BYTE_TIMEOUT) {
        RCLCPP_ERROR(this->get_logger(), "[TIMEOUT] Read stall timeout (%zu/%zu bytes)", received,
                     len);
        return false;
      }

      if (!serial_port_ || !serial_port_->is_open()) {
        return false;
      }

      const size_t need = len - received;
      tmp.assign(need, 0);

      size_t n = 0;
      try {
        n = serial_port_->receive(tmp);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial receive: %s", e.what());
        try {
          serial_port_->close();
        } catch (...) {
        }
        return false;
      }

      if (n == 0) {
        continue;
      }

      // Update activity timestamp when data received
      last_activity = std::chrono::steady_clock::now();

      if (n > need) {
        n = need;
      }

      std::memcpy(dst + received, tmp.data(), n);
      received += n;
    }

    return received == len;
  }

  void initROS2Communication() {
    // 订阅力矩命令
    torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10,
        std::bind(&HardwareInterfaceNode::torqueCallback, this, std::placeholders::_1));

    // 发布关节状态到标准话题 (MoveIt/RViz/数字孪生都订阅此话题)
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    RCLCPP_INFO(this->get_logger(), "[INIT] Publishing to /joint_states");
  }

  // ========== 回调函数 ==========

  void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != static_cast<size_t>(num_joints_)) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Invalid torque size: %zu", msg->data.size());
      return;
    }

    // 解耦架构：仅缓存数据，不立即发送（由sendLoop定时器负责发送）
    std::lock_guard<std::mutex> lock(torque_cache_mutex_);
    for (int i = 0; i < num_joints_; ++i) {
      cached_torques_[i] = static_cast<float>(msg->data[i]);
    }
    last_torque_update_ = this->now();
    torque_data_valid_ = true;

    // 不再调用 sendTorqueCommand()！发送由定时器驱动
  }

  // ========== 串口收发函数 ==========

  // 新增：200Hz定时发送循环（解耦架构核心）
  void sendLoop() {
    if (!serial_port_ || !serial_port_->is_open()) {
      return;  // 串口未打开，等待自动重连
    }

    float torques_to_send[7];
    bool data_fresh = false;

    // 1. 读取缓存的力矩数据
    {
      std::lock_guard<std::mutex> lock(torque_cache_mutex_);

      // 检查数据新鲜度（10ms超时阈值）
      if (torque_data_valid_) {
        double age = (this->now() - last_torque_update_).seconds();
        if (age > 0.01) {
          RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 2000,
              "[WARN] Torque data stale (%.1f ms old), continuing with cached values", age * 1000);
        }
        data_fresh = true;
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[WARN] No torque data received yet, sending zeros");
      }

      // 应用 force_zero_torque 安全开关
      if (this->get_parameter("force_zero_torque").as_bool()) {
        std::fill(torques_to_send, torques_to_send + num_joints_, 0.0f);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[SAFETY] Force zero torque mode enabled - sending all zeros");
      } else if (data_fresh) {
        std::copy(cached_torques_, cached_torques_ + num_joints_, torques_to_send);
      } else {
        std::fill(torques_to_send, torques_to_send + num_joints_, 0.0f);
      }
    }

    // 2. 构建并发送SEASKY数据包
    SerialProtocol::TorqueCommand cmd;
    for (int i = 0; i < num_joints_; ++i) {
      cmd.torques[i] = torques_to_send[i];
    }

    std::vector<uint8_t> packet = SerialProtocol::buildTorquePacket(cmd);

    try {
      const size_t sent = serial_port_->send(packet);
      if (sent != packet.size()) {
        RCLCPP_WARN(this->get_logger(), "[WARN] Partial send: %zu/%zu", sent, packet.size());
      } else {
        tx_packet_count_++;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[ERROR] Send failed: %s",
                            e.what());
      try {
        serial_port_->close();
      } catch (...) {
      }
    }
  }

  // 保留旧函数供兼容（但不再使用）
  void sendTorqueCommand() {
    if (!serial_port_ || !serial_port_->is_open()) {
      return;
    }

    SerialProtocol::TorqueCommand cmd;
    {
      std::lock_guard<std::mutex> lock(torque_cache_mutex_);
      for (int i = 0; i < num_joints_; ++i) cmd.torques[i] = cached_torques_[i];
    }

    std::vector<uint8_t> packet = SerialProtocol::buildTorquePacket(cmd);

    try {
      const size_t sent = serial_port_->send(packet);
      if (sent != packet.size()) {
        RCLCPP_WARN(this->get_logger(), "[WARN] Partial send: %zu/%zu", sent, packet.size());
      }
      // RCLCPP_DEBUG(this->get_logger(), "[TX] Sent %zu bytes", packet.size());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Send failed: %s", e.what());
      try {
        serial_port_->close();
      } catch (...) {
      }
    }
  }

  void receiveLoop() {
    // State Machine for SEASKY Protocol
    // [SOF(1)] [Len(2)] [CRC8(1)] [Cmd(2)] [Flags(2)] [Payload(N)] [CRC16(2)]

    enum State { WAIT_SOF, READ_LEN, READ_HEADER_CRC, READ_BODY };
    State state = WAIT_SOF;

    std::vector<uint8_t> buffer;
    uint16_t data_len = 0;

    while (running_) {
      if (!ensureSerialOpen(std::chrono::milliseconds(200))) {
        continue;
      }

      try {
        switch (state) {
          case WAIT_SOF: {
            uint8_t byte = 0;
            if (!readExact(&byte, 1)) {
              state = WAIT_SOF;
              break;
            }

            // Update RX activity on every byte read
            last_rx_activity_.store(std::chrono::steady_clock::now());

            if (byte == SerialProtocol::SOF) {
              buffer.clear();
              buffer.push_back(byte);
              state = READ_LEN;
            }
          } break;

          case READ_LEN: {
            uint8_t len_bytes[2] = {0, 0};
            if (!readExact(len_bytes, 2)) {
              state = WAIT_SOF;
              break;
            }
            buffer.push_back(len_bytes[0]);
            buffer.push_back(len_bytes[1]);

            data_len = len_bytes[0] | (len_bytes[1] << 8);

            // Safety check: validate packet length
            if (data_len > 512) {
              RCLCPP_ERROR(this->get_logger(), "[SAFETY] Invalid packet length: %u, resync",
                           data_len);
              state = WAIT_SOF;
              break;
            }

            state = READ_HEADER_CRC;
          } break;

          case READ_HEADER_CRC: {
            uint8_t byte = 0;
            if (!readExact(&byte, 1)) {
              state = WAIT_SOF;
              break;
            }
            buffer.push_back(byte);

            // Validate Header CRC8 (SOF + Len_L + Len_H)
            uint8_t expected_crc = SerialProtocol::Get_CRC8_Check_Sum(buffer.data(), 3, 0xFF);
            if (byte == expected_crc) {
              state = READ_BODY;
            } else {
              RCLCPP_WARN(this->get_logger(), "[RX] Header CRC Fail");
              rx_crc_errors_++;
              state = WAIT_SOF;
            }
          } break;

          case READ_BODY:
            // Body size = CmdID(2) + DataLen + CRC16(2)
            size_t body_size = 2 + data_len + 2;  // CmdID(2) + data_len + CRC16(2)

            {
              std::vector<uint8_t> body(body_size);
              if (!readExact(body.data(), body_size)) {
                state = WAIT_SOF;
                break;
              }
              buffer.insert(buffer.end(), body.begin(), body.end());

              // Validate Whole Packet CRC16
              uint16_t received_crc = body[body_size - 2] | (body[body_size - 1] << 8);
              uint16_t calculated_crc =
                  SerialProtocol::Get_CRC16_Check_Sum(buffer.data(), buffer.size() - 2, 0xFFFF);

              if (received_crc == calculated_crc) {
                processPacket(buffer);
                rx_packet_count_++;
              } else {
                RCLCPP_WARN(this->get_logger(), "[RX] Body CRC Fail");
                rx_crc_errors_++;
              }
              state = WAIT_SOF;
            }
            break;
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Receive: %s", e.what());
        try {
          if (serial_port_ && serial_port_->is_open()) {
            serial_port_->close();
          }
        } catch (...) {
        }
        state = WAIT_SOF;
      }
    }
  }

  void processPacket(const std::vector<uint8_t> &packet) {
    // Packet structure: [SOF(1)][Len(2)][CRC8(1)] [Cmd(2)][Flags(2)][Payload...][CRC16(2)]
    // Offset to CmdID is 4
    size_t offset = 4;
    uint16_t cmd_id = SerialProtocol::read_uint16(packet.data(), offset);
    uint16_t flags = SerialProtocol::read_uint16(packet.data(), offset);  // offset becomes 8

    if (cmd_id == SerialProtocol::CMD_JOINT_FEEDBACK) {
      float positions[7];
      float velocities[7];
      uint32_t islive[7];  // 存活状态（暂不使用）

      // 协议格式：每关节交替读取 Position, Speed, IsLive
      // Payload: [Pos0, Spd0, IsLive0, ..., Pos6(Gripper), Spd6, IsLive6]
      for (int i = 0; i < num_joints_; ++i) {
        positions[i] = SerialProtocol::read_float(packet.data(), offset);
        velocities[i] = SerialProtocol::read_float(packet.data(), offset);
        islive[i] = SerialProtocol::read_uint32(packet.data(), offset);  // 读取但暂不使用
      }

      updateAndPublishJointStates(positions, velocities);
    }
  }

  void updateAndPublishJointStates(const float positions[7], const float velocities[7]) {
    // 1. 更新缓存
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::memcpy(current_positions_, positions, sizeof(float) * num_joints_);
      std::memcpy(current_velocities_, velocities, sizeof(float) * num_joints_);
    }

    // 2. 发布 ROS2 消息
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_gripper1"};

    msg.position.resize(num_joints_);
    msg.velocity.resize(num_joints_);
    for (int i = 0; i < num_joints_; ++i) {
      msg.position[i] = positions[i];
      msg.velocity[i] = velocities[i];
    }

    joint_state_pub_->publish(msg);
  }

  void healthCheck() {
    static uint64_t last_rx_count = 0;
    static uint64_t last_tx_count = 0;
    static uint64_t last_crc_errors = 0;

    uint64_t current_rx = rx_packet_count_.load();
    uint64_t current_tx = tx_packet_count_.load();
    uint64_t current_crc = rx_crc_errors_.load();

    double rx_rate = (current_rx - last_rx_count) / 5.0;  // 5 sec interval
    double tx_rate = (current_tx - last_tx_count) / 5.0;
    uint64_t new_crc_errors = current_crc - last_crc_errors;

    // Check RX thread health
    auto now = std::chrono::steady_clock::now();
    auto last_activity = last_rx_activity_.load();
    auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_activity).count();

    bool serial_ok = serial_port_ && serial_port_->is_open();

    if (simulation_mode_) {
      RCLCPP_INFO(this->get_logger(), "[HEALTH] TX: %.1f Hz | Serial: SIMULATION MODE", tx_rate);
    } else {
      if (elapsed_ms > 1000 && serial_ok) {
        RCLCPP_WARN(this->get_logger(),
                    "[HEALTH] RX thread inactive for %ld ms! Serial OK but no data", elapsed_ms);
      }

      std::string status = serial_ok ? "OK" : "CLOSED";
      if (new_crc_errors > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[HEALTH] TX: %.1f Hz | RX: %.1f Hz | Serial: %s | CRC errors: +%lu", tx_rate,
                    rx_rate, status.c_str(), new_crc_errors);
      } else {
        RCLCPP_INFO(this->get_logger(), "[HEALTH] TX: %.1f Hz | RX: %.1f Hz | Serial: %s", tx_rate,
                    rx_rate, status.c_str());
      }
    }

    last_rx_count = current_rx;
    last_tx_count = current_tx;
    last_crc_errors = current_crc;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HardwareInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
