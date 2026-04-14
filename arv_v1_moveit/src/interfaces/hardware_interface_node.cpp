/// @file hardware_interface_node.cpp
/// @brief Hardware USB CDC interface — TX torques/gripper, RX joint states via Seasky protocol.
#include <atomic>
#include <chrono>
#include <filesystem>
#include <io_context/io_context.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>

#include "serial_protocol.hpp"

class HardwareInterfaceNode : public rclcpp::Node {
public:
  HardwareInterfaceNode()
      : Node("hardware_interface_node"), running_(false), simulation_mode_(false) {
    this->declare_parameter("serial_port", "/dev/ttyACM0");
    this->declare_parameter("baud_rate", 921600);  // USB CDC 忽略此值, serial_driver API 强制要求
    this->declare_parameter("send_rate_hz", 1000);
    this->declare_parameter("gripper_rate_hz", 50);
    this->declare_parameter("simulation_mode", false);
    this->declare_parameter("force_zero_torque", false);
    this->declare_parameter("link_diag_enabled", true);
    this->declare_parameter("link_diag_period_ms", 2000);

    std::string port = this->get_parameter("serial_port").as_string();
    int baud = this->get_parameter("baud_rate").as_int();
    simulation_mode_ = this->get_parameter("simulation_mode").as_bool();

    RCLCPP_INFO(this->get_logger(), "[INIT] USB CDC device: %s (baud=%d ignored by CDC)",
                port.c_str(), baud);

    if (simulation_mode_) {
      RCLCPP_INFO(this->get_logger(), "[SIMULATION MODE] Serial TX only, no RX feedback");
    }

    if (!initSerial(port, baud)) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Initial serial open failed: %s", port.c_str());
      RCLCPP_INFO(this->get_logger(),
                  "[INFO] Node will continue running and auto-reconnect when device available");
    } else {
      RCLCPP_INFO(this->get_logger(), "[OK] USB CDC device opened: %s", port.c_str());
    }

    initROS2Communication();
    running_ = true;

    if (simulation_mode_) {
      RCLCPP_INFO(this->get_logger(),
                  "[OK] Simulation mode - Serial TX only, waiting for MuJoCo feedback");
    } else {
      receive_thread_ = std::thread(&HardwareInterfaceNode::receiveLoop, this);
      RCLCPP_INFO(this->get_logger(),
                  "[OK] Hardware mode - USB CDC RX/TX enabled (auto-reconnect every 200ms)");

      const bool diag_enabled = this->get_parameter("link_diag_enabled").as_bool();
      if (diag_enabled) {
        link_diag_thread_ = std::thread(&HardwareInterfaceNode::linkDiagLoop, this);
        RCLCPP_INFO(this->get_logger(), "[OK] Link diag thread enabled");
      }
    }

    const int send_hz = this->get_parameter("send_rate_hz").as_int();
    const int grip_hz = this->get_parameter("gripper_rate_hz").as_int();
    gripper_divider_ = (grip_hz > 0 && send_hz >= grip_hz) ? (send_hz / grip_hz) : 4;
    auto send_period = std::chrono::microseconds(1000000 / send_hz);
    send_timer_ =
        this->create_wall_timer(send_period, std::bind(&HardwareInterfaceNode::sendLoop, this));

    last_torque_update_ = this->now();
    RCLCPP_INFO(this->get_logger(), "[OK] Send timer started at %dHz, gripper at %dHz (1:%d)",
                send_hz, grip_hz, gripper_divider_);

    last_rx_activity_.store(std::chrono::steady_clock::now());
    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                            std::bind(&HardwareInterfaceNode::healthCheck, this));

    RCLCPP_INFO(this->get_logger(), "[OK] Hardware interface node started");
  }

  ~HardwareInterfaceNode() {
    running_ = false;

    // [FIX] 锁内 swap-out 端口，锁外 close，避免持锁阻塞导致 receiveLoop 死锁
    decltype(serial_port_) port_to_close;
    {
      std::lock_guard<std::mutex> slock(serial_mutex_);
      port_to_close = std::exchange(serial_port_, nullptr);
    }
    // serial_mutex_ 已释放，receiveLoop 可检测 serial_port_==null 后退出
    try {
      if (port_to_close && port_to_close->is_open()) {
        port_to_close->close();
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Serial close: %s", e.what());
    }

    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }

    if (link_diag_thread_.joinable()) {
      link_diag_thread_.join();
    }

    RCLCPP_INFO(this->get_logger(), "[SHUTDOWN] Hardware interface closed");
  }

private:
  std::atomic<bool> running_;
  bool simulation_mode_;

  // --- USB CDC (via serial_driver API) ---
  std::string device_name_;
  uint32_t baud_rate_{0};  // CDC 忽略, serial_driver API 要求
  std::unique_ptr<drivers::common::IoContext> io_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::shared_ptr<drivers::serial_driver::SerialPort> serial_port_;
  std::mutex serial_mutex_;  // [FIX] 保护 serial_port_ 跨线程操作 (RX/TX/析构)
  std::thread receive_thread_;
  std::thread link_diag_thread_;

  // --- ROS2 ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arm_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_command_pub_;
  rclcpp::TimerBase::SharedPtr send_timer_;

  // --- 数据缓存 ---
  std::mutex data_mutex_;
  float current_positions_[SerialProtocol::NUM_ALL_JOINTS] = {0};
  float current_velocities_[SerialProtocol::NUM_ALL_JOINTS] = {0};

  // 力矩缓存: index 0-5=臂, 6=夹爪
  std::mutex torque_cache_mutex_;
  float cached_torques_[SerialProtocol::NUM_ALL_JOINTS] = {0};
  rclcpp::Time last_torque_update_;
  bool torque_data_valid_ = false;

  int gripper_divider_ = 4;
  int gripper_counter_ = 0;

  // 0x0006 ARM_STATUS TX: 10Hz (1kHz / 100)
  uint8_t cached_arm_state_ = 0x05;  // 默认 ArmState::RELAX
  std::mutex arm_state_mutex_;
  int status_divider_ = 100;
  int status_counter_ = 0;

  // Health monitoring
  std::atomic<std::chrono::steady_clock::time_point> last_rx_activity_;
  std::atomic<uint64_t> rx_packet_count_{0};
  std::atomic<uint64_t> tx_packet_count_{0};
  std::atomic<uint64_t> tx_attempt_count_{0};
  std::atomic<uint64_t> rx_crc_errors_{0};
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::atomic<std::chrono::steady_clock::time_point> last_tx_attempt_activity_;
  std::atomic<std::chrono::steady_clock::time_point> last_tx_success_activity_;

  void linkDiagLoop() {
    uint64_t last_rx = 0;
    uint64_t last_tx_ok = 0;
    uint64_t last_tx_try = 0;

    int period_ms = this->get_parameter("link_diag_period_ms").as_int();
    if (period_ms < 200) period_ms = 200;

    while (running_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
      if (!running_) break;

      const uint64_t rx_now = rx_packet_count_.load();
      const uint64_t tx_ok_now = tx_packet_count_.load();
      const uint64_t tx_try_now = tx_attempt_count_.load();
      const uint64_t crc_now = rx_crc_errors_.load();

      const auto now = std::chrono::steady_clock::now();
      const auto rx_last = last_rx_activity_.load();
      const auto tx_try_last = last_tx_attempt_activity_.load();
      const auto tx_ok_last = last_tx_success_activity_.load();

      const auto rx_idle_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - rx_last).count();
      const auto tx_try_idle_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - tx_try_last).count();
      const auto tx_ok_idle_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - tx_ok_last).count();

      const uint64_t d_rx = rx_now - last_rx;
      const uint64_t d_tx_ok = tx_ok_now - last_tx_ok;
      const uint64_t d_tx_try = tx_try_now - last_tx_try;

      RCLCPP_INFO(this->get_logger(),
                  "[LINK] dTX_try=%lu dTX_ok=%lu dRX=%lu | idle(ms): tx_try=%ld tx_ok=%ld rx=%ld "
                  "| crc=%lu",
                  d_tx_try, d_tx_ok, d_rx, tx_try_idle_ms, tx_ok_idle_ms, rx_idle_ms, crc_now);

      last_rx = rx_now;
      last_tx_ok = tx_ok_now;
      last_tx_try = tx_try_now;
    }
  }

  bool initSerial(const std::string &port, int baud) {
    try {
      device_name_ = port;
      baud_rate_ = static_cast<uint32_t>(baud);

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
    // [FIX] 锁内仅做状态检查和重连尝试，sleep 移到锁外避免阻塞 sendLoop/receiveLoop
    {
      std::lock_guard<std::mutex> slock(serial_mutex_);
      if (serial_port_ && serial_port_->is_open()) {
        return true;
      }

      const std::string dev = device_name_.empty() ? std::string("/dev/ttyACM0") : device_name_;
      if (std::filesystem::exists(dev)) {
        const int baud =
            static_cast<int>(baud_rate_ == 0 ? 115200 : baud_rate_);  // CDC 忽略, 占位值
        try {
          if (initSerial(dev, baud)) {
            RCLCPP_WARN(this->get_logger(), "[RECONNECT] USB CDC reinitialised: %s", dev.c_str());
            return true;
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "[WARN] Serial reinit failed: %s", e.what());
        }
      }
    }
    // serial_mutex_ 已释放，安全 sleep
    std::this_thread::sleep_for(backoff);
    return false;
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

      const size_t need = len - received;
      tmp.assign(need, 0);

      // [FIX] 只持锁复制 shared_ptr，锁外执行 receive()
      // 避免阻塞 IO 长期占用 serial_mutex_ 导致 ROS2 executor（sendLoop）永久冻结
      std::shared_ptr<drivers::serial_driver::SerialPort> port_snap;
      {
        std::lock_guard<std::mutex> slock(serial_mutex_);
        if (!serial_port_ || !serial_port_->is_open()) {
          return false;
        }
        port_snap = serial_port_;  // 引用计数保活，即使析构也不会野指针
      }

      size_t n = 0;
      try {
        n = port_snap->receive(tmp);  // 阻塞在锁外，serial_mutex_ 已释放
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Serial receive: %s", e.what());
        try {
          std::lock_guard<std::mutex> slock(serial_mutex_);
          if (serial_port_ && serial_port_->is_open()) {
            serial_port_->close();
          }
        } catch (...) {
          RCLCPP_DEBUG(this->get_logger(), "Serial port close failed during cleanup");
        }
        return false;
      }

      if (n == 0) {
        // [FIX] 避免忙循环耗尽 CPU
        std::this_thread::sleep_for(std::chrono::microseconds(200));
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
    // 7轴命令: [0-5]=臂力矩, [6]=夹爪力(N), sendLoop 转 flag
    torque_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", 10,
        std::bind(&HardwareInterfaceNode::torqueCallback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    task_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/task_command", 10);
    arm_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/arm_state", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) {
            std::lock_guard<std::mutex> lk(arm_state_mutex_);
            cached_arm_state_ = msg->data;
        });
  }

  // --- 回调函数 ---

  void torqueCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < SerialProtocol::NUM_ARM_JOINTS) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Torque msg size %zu < %zu", msg->data.size(),
                  SerialProtocol::NUM_ARM_JOINTS);
      return;
    }
    std::lock_guard<std::mutex> lock(torque_cache_mutex_);
    const size_t n = std::min(msg->data.size(), SerialProtocol::NUM_ALL_JOINTS);
    for (size_t i = 0; i < n; ++i) {
      cached_torques_[i] = static_cast<float>(msg->data[i]);
    }
    if (msg->data.size() < SerialProtocol::NUM_ALL_JOINTS) {
      cached_torques_[SerialProtocol::NUM_ARM_JOINTS] = 0.0f;
    }
    last_torque_update_ = this->now();
    torque_data_valid_ = true;
  }

  // --- 串口收发 ---

  void sendLoop() {
    last_tx_attempt_activity_.store(std::chrono::steady_clock::now());
    {
      std::lock_guard<std::mutex> slock(serial_mutex_);
      if (!serial_port_ || !serial_port_->is_open()) {
        return;
      }
    }

    const bool force_zero = this->get_parameter("force_zero_torque").as_bool();

    // ── 读取缓存力矩 ──
    float torques_to_send[SerialProtocol::NUM_ARM_JOINTS];
    bool data_fresh = false;
    {
      std::lock_guard<std::mutex> lock(torque_cache_mutex_);
      if (torque_data_valid_) {
        double age = (this->now() - last_torque_update_).seconds();
        if (age > 0.1) {
          torque_data_valid_ = false;
          std::fill(cached_torques_, cached_torques_ + SerialProtocol::NUM_ALL_JOINTS, 0.0f);
          RCLCPP_ERROR(this->get_logger(),
                       "[SAFETY] Torque data stale (%.0f ms), invalidated → sending zeros",
                       age * 1000);
        } else if (age > 0.01) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "[WARN] Torque data slightly stale (%.1f ms old)", age * 1000);
        }
        data_fresh = torque_data_valid_;  // 可能刚被清除
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[WARN] No torque data received yet, sending zeros");
      }

      if (force_zero) {
        std::fill(torques_to_send, torques_to_send + SerialProtocol::NUM_ARM_JOINTS, 0.0f);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[SAFETY] Force zero torque mode enabled - sending all zeros");
      } else if (data_fresh) {
        std::copy(cached_torques_, cached_torques_ + SerialProtocol::NUM_ARM_JOINTS,
                  torques_to_send);
      } else {
        std::fill(torques_to_send, torques_to_send + SerialProtocol::NUM_ARM_JOINTS, 0.0f);
      }
    }

    // ── 发送力矩包 ──
    SerialProtocol::TorqueCommand cmd;
    for (size_t i = 0; i < SerialProtocol::NUM_ARM_JOINTS; ++i) {
      cmd.torques[i] = torques_to_send[i];
    }

    auto sendRaw = [&](const std::vector<uint8_t> &pkt) {
      tx_attempt_count_++;
      last_tx_attempt_activity_.store(std::chrono::steady_clock::now());
      std::lock_guard<std::mutex> slock(serial_mutex_);
      try {
        if (!serial_port_ || !serial_port_->is_open()) return;
        const size_t sent = serial_port_->send(pkt);
        if (sent != pkt.size()) {
          RCLCPP_WARN(this->get_logger(), "[WARN] Partial send: %zu/%zu", sent, pkt.size());
        } else {
          tx_packet_count_++;
          last_tx_success_activity_.store(std::chrono::steady_clock::now());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "[ERROR] Send failed: %s", e.what());
        try {
          serial_port_->close();
        } catch (...) {
          RCLCPP_DEBUG(this->get_logger(), "Serial port close failed during cleanup");
        }
      }
    };

    sendRaw(SerialProtocol::buildTorquePacket(cmd));

    // ── 分频发送夹爪包: [6] > +0.1 → GRIP, < -0.1 → RELEASE, else → STOP ──
    gripper_counter_ = (gripper_counter_ + 1) % gripper_divider_;
    if (gripper_counter_ == 0) {
      SerialProtocol::GripperAction action;
      if (force_zero) {
        action = SerialProtocol::GripperAction::STOP;
      } else {
        std::lock_guard<std::mutex> glock(torque_cache_mutex_);
        const float g = cached_torques_[SerialProtocol::NUM_ARM_JOINTS];  // index 6
        if (g > 0.1f)
          action = SerialProtocol::GripperAction::GRIP;
        else if (g < -0.1f)
          action = SerialProtocol::GripperAction::RELEASE;
        else
          action = SerialProtocol::GripperAction::STOP;
      }
      sendRaw(SerialProtocol::buildGripperPacket(action));
    }

    // ── 分频发送臂状态包 0x0006: 10Hz ──
    status_counter_ = (status_counter_ + 1) % status_divider_;
    if (status_counter_ == 0) {
      SerialProtocol::ArmStatusPacket status;
      {
        std::lock_guard<std::mutex> lk(arm_state_mutex_);
        status.arm_state = static_cast<SerialProtocol::ArmState>(cached_arm_state_);
      }
      status.task_progress = 0;
      status.error_code    = SerialProtocol::ArmError::NO_ERROR;
      status.gripper_state = SerialProtocol::GripperState::OPEN;
      sendRaw(SerialProtocol::buildArmStatusPacket(status));
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
            if (data_len > 256) {
              RCLCPP_ERROR(this->get_logger(),
                           "[SAFETY] data_len=%u too large in READ_BODY, resync", data_len);
              state = WAIT_SOF;
              break;
            }
            // Body size = CmdID(2) + DataLen + CRC16(2)
            {
              size_t body_size = 2 + data_len + 2;  // CmdID(2) + data_len + CRC16(2)
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
          std::lock_guard<std::mutex> slock(serial_mutex_);
          if (serial_port_ && serial_port_->is_open()) {
            serial_port_->close();
          }
        } catch (...) {
          RCLCPP_DEBUG(this->get_logger(), "Serial port close failed during cleanup");
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
    (void)flags;

    if (cmd_id == SerialProtocol::CMD_JOINT_FEEDBACK) {
      constexpr size_t expected_payload = SerialProtocol::NUM_ALL_JOINTS * (4 + 4 + 4);  // 84B
      if (offset + expected_payload > packet.size()) {
        RCLCPP_ERROR(this->get_logger(),
                     "[RX] Joint feedback packet too short: %zu bytes, need %zu", packet.size(),
                     offset + expected_payload);
        return;
      }

      float positions[SerialProtocol::NUM_ALL_JOINTS];
      float velocities[SerialProtocol::NUM_ALL_JOINTS];
      uint32_t islive[SerialProtocol::NUM_ALL_JOINTS];

      for (size_t i = 0; i < SerialProtocol::NUM_ALL_JOINTS; ++i) {
        positions[i] = SerialProtocol::read_float(packet.data(), offset);
        velocities[i] = SerialProtocol::read_float(packet.data(), offset);
        islive[i] = SerialProtocol::read_uint32(packet.data(), offset);
      }
      updateAndPublishJointStates(positions, velocities);

    } else if (cmd_id == SerialProtocol::CMD_TASK_COMMAND) {
      // 3B → Int32: (cmd<<16)|(param<<8)|seq
      if (offset + 3 <= packet.size()) {
        const uint8_t task_cmd = packet[offset];
        const uint8_t param = packet[offset + 1];
        const uint8_t seq = packet[offset + 2];

        auto msg = std_msgs::msg::Int32();
        msg.data = (static_cast<int32_t>(task_cmd) << 16) | (static_cast<int32_t>(param) << 8) |
                   static_cast<int32_t>(seq);
        task_command_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "[RX] TaskCmd 0x%02X param=%u seq=%u", task_cmd, param,
                    seq);
      }
    }
  }

  void updateAndPublishJointStates(const float positions[SerialProtocol::NUM_ALL_JOINTS],
                                   const float velocities[SerialProtocol::NUM_ALL_JOINTS]) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      std::memcpy(current_positions_, positions, sizeof(float) * SerialProtocol::NUM_ALL_JOINTS);
      std::memcpy(current_velocities_, velocities, sizeof(float) * SerialProtocol::NUM_ALL_JOINTS);
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_gripper1"};

    msg.position.resize(SerialProtocol::NUM_ALL_JOINTS);
    msg.velocity.resize(SerialProtocol::NUM_ALL_JOINTS);
    for (size_t i = 0; i < SerialProtocol::NUM_ALL_JOINTS; ++i) {
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
  try {
    auto node = std::make_shared<HardwareInterfaceNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("hardware_interface"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
