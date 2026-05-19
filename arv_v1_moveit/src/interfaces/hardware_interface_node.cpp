/// @file hardware_interface_node.cpp
/// @brief Hardware USB CDC interface — TX cmd (position+gripper)/RX joint states via Seasky
/// protocol.
#include <atomic>
#include <chrono>
#include <climits>
#include <filesystem>
#include <io_context/io_context.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <thread>

#include "serial_protocol.hpp"

namespace JointLimits {
// URDF joint limits for feedback clamp (joint_6 continuous, gripper excluded)
static constexpr size_t kNumClampedJoints = 5;
static constexpr size_t kClampIdx[kNumClampedJoints] = {0, 1, 2, 3, 4};
static constexpr double kLower[kNumClampedJoints] = {-1.2217, 0.49, -0.90, -2.975, -1.5708};
static constexpr double kUpper[kNumClampedJoints] = {1.2217, 3.14, 0.70, 3.14, 1.5708};
static constexpr const char *kName[kNumClampedJoints] = {"J1", "J2", "J3", "J4(Roll1)", "J5"};
}  // namespace JointLimits

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

    last_cmd_update_ = this->now();

    // ── 1kHz TX 独立 steady_clock 线程 (替换 ROS wall_timer) ──
    // 原因: wall_timer 受 ROS executor 调度, 观测到 gap max≈49ms;
    //       sleep_until 直接绑定时钟, 典型 jitter < 1ms.
    send_thread_ = std::thread([this, send_hz]() {
      using Clock = std::chrono::steady_clock;
      const auto period = std::chrono::microseconds(1000000 / send_hz);
      auto next_tick = Clock::now() + period;
      while (running_) {
        sendLoop();
        next_tick += period;
        // ── 追赶跳过 (catch-up skip) ──
        // 若 sendLoop() 因 serial write 阻塞导致 next_tick 已落后,
        // 不做连续无 sleep 追赶 (会 flood 串口); 直接对齐到 now+period.
        const auto now = Clock::now();
        if (next_tick < now) {
          next_tick = now + period;
        }
        std::this_thread::sleep_until(next_tick);
      }
    });
    RCLCPP_INFO(this->get_logger(), "[OK] Send thread started at %dHz, gripper at %dHz (1:%d)",
                send_hz, grip_hz, gripper_divider_);

    last_rx_activity_.store(std::chrono::steady_clock::now());
    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                            std::bind(&HardwareInterfaceNode::healthCheck, this));

    RCLCPP_INFO(this->get_logger(), "[OK] Hardware interface node started");
  }

  ~HardwareInterfaceNode() {
    running_ = false;

    // [FIX] swap-out port under lock, close outside lock — holding the lock during
    //       close would deadlock receiveLoop.
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

    // send_thread_ 先 join: running_=false 后循环退出, sleep_until 会在下一 tick 醒来
    if (send_thread_.joinable()) {
      send_thread_.join();
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
  std::mutex serial_mutex_;  // [FIX] guards serial_port_ across RX / TX / dtor threads.
  std::thread receive_thread_;
  std::thread link_diag_thread_;

  // --- ROS2 ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr arm_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr task_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr link_diag_pub_;
  std::thread send_thread_;  // 1kHz TX 独立线程 (替换 ROS wall_timer, 消除 49ms 调度空洞)

  // --- 数据缓存 ---
  std::mutex data_mutex_;
  float current_positions_[SerialProtocol::NUM_ALL_JOINTS] = {0};
  float current_velocities_[SerialProtocol::NUM_ALL_JOINTS] = {0};

  // 力矩缓存: index 0-5=臂, 6=夹爪
  std::mutex cmd_cache_mutex_;
  float cached_cmd_[SerialProtocol::NUM_ALL_JOINTS] = {0};
  rclcpp::Time last_cmd_update_;
  std::chrono::steady_clock::time_point last_cmd_steady_{};  // 锁内用 steady_clock 计算 age
  bool cmd_data_valid_ = false;

  int gripper_divider_ = 4;
  int gripper_counter_ = 0;

  // 0x0006 ARM_STATUS TX: 10Hz (1kHz / 100)
  // 0x0006 = mission_executor 心跳: 若 /arm_state 停发 >500ms (即 mission_executor 挂了),
  // 这里也不再向 MCU 发 0x0006, 让 MCU watchdog 检测到"上位机死亡".
  uint8_t cached_arm_state_ = 0x05;  // 默认 ArmState::RELAX
  std::mutex arm_state_mutex_;
  std::chrono::steady_clock::time_point arm_state_last_rx_{};
  bool arm_state_ever_received_ = false;
  int status_divider_ = 100;
  int status_counter_ = 0;

  // 夹爪状态: 基于最近一次发送的 GripperAction 推导 (无 MCU 反馈)
  // GRIP → CLOSED, RELEASE → OPEN, STOP → 保持
  std::atomic<SerialProtocol::GripperState> last_gripper_state_{SerialProtocol::GripperState::OPEN};

  // Health monitoring
  std::atomic<std::chrono::steady_clock::time_point> last_rx_activity_;
  std::atomic<uint64_t> rx_packet_count_{0};
  std::atomic<uint64_t> tx_packet_count_{0};
  std::atomic<uint64_t> tx_attempt_count_{0};
  std::atomic<uint64_t> rx_crc_errors_{0};
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::atomic<std::chrono::steady_clock::time_point> last_tx_attempt_activity_;
  std::atomic<std::chrono::steady_clock::time_point> last_tx_success_activity_;

  // sendLoop timing (lightweight, single-threaded — no atomics needed)
  struct SendStats {
    int64_t min_us{INT64_MAX}, max_us{0}, sum_us{0};
    int64_t lock_sum_us{0};
    int64_t gap_max_us{0}, param_max_us{0}, cache_max_us{0};
    int64_t build_cmd_max_us{0}, send_cmd_max_us{0};
    int64_t build_gripper_max_us{0}, send_gripper_max_us{0};
    int64_t build_status_max_us{0}, send_status_max_us{0};
    int64_t gap_sum_us{0}, param_sum_us{0}, cache_sum_us{0};
    int64_t build_cmd_sum_us{0}, send_cmd_sum_us{0};
    int64_t build_gripper_sum_us{0}, send_gripper_sum_us{0};
    int64_t build_status_sum_us{0}, send_status_sum_us{0};
    uint64_t count{0}, overruns{0};
    void record(int64_t total, int64_t lock, int64_t gap, int64_t param, int64_t cache,
                int64_t build_cmd, int64_t send_cmd, int64_t build_gripper, int64_t send_gripper,
                int64_t build_status, int64_t send_status) {
      if (total < min_us) min_us = total;
      if (total > max_us) max_us = total;
      if (gap > gap_max_us) gap_max_us = gap;
      if (param > param_max_us) param_max_us = param;
      if (cache > cache_max_us) cache_max_us = cache;
      if (build_cmd > build_cmd_max_us) build_cmd_max_us = build_cmd;
      if (send_cmd > send_cmd_max_us) send_cmd_max_us = send_cmd;
      if (build_gripper > build_gripper_max_us) build_gripper_max_us = build_gripper;
      if (send_gripper > send_gripper_max_us) send_gripper_max_us = send_gripper;
      if (build_status > build_status_max_us) build_status_max_us = build_status;
      if (send_status > send_status_max_us) send_status_max_us = send_status;
      sum_us += total;
      lock_sum_us += lock;
      gap_sum_us += gap;
      param_sum_us += param;
      cache_sum_us += cache;
      build_cmd_sum_us += build_cmd;
      send_cmd_sum_us += send_cmd;
      build_gripper_sum_us += build_gripper;
      send_gripper_sum_us += send_gripper;
      build_status_sum_us += build_status;
      send_status_sum_us += send_status;
      ++count;
      if (total > 1500) ++overruns;
    }
    int64_t avg_us() const { return count ? sum_us / (int64_t)count : 0; }
    int64_t lock_avg_us() const { return count ? lock_sum_us / (int64_t)count : 0; }
    int64_t avg(int64_t sum) const { return count ? sum / (int64_t)count : 0; }
    void reset() {
      min_us = INT64_MAX;
      max_us = 0;
      sum_us = 0;
      lock_sum_us = 0;
      gap_max_us = param_max_us = cache_max_us = 0;
      build_cmd_max_us = send_cmd_max_us = 0;
      build_gripper_max_us = send_gripper_max_us = 0;
      build_status_max_us = send_status_max_us = 0;
      gap_sum_us = param_sum_us = cache_sum_us = 0;
      build_cmd_sum_us = send_cmd_sum_us = 0;
      build_gripper_sum_us = send_gripper_sum_us = 0;
      build_status_sum_us = send_status_sum_us = 0;
      count = 0;
      overruns = 0;
    }
  } send_stats_;
  std::chrono::steady_clock::time_point last_send_entry_{};

  // ── RX gap 分布统计 ──
  // receiveLoop 单线程写 live_, healthCheck 在 rx_gap_mutex_ 下 swap→snap 读
  struct RxGapStats {
    int64_t min_us{INT64_MAX}, max_us{0}, sum_us{0};
    uint64_t count{0};
    // 分桶: <2ms / 2-5ms / 5-20ms / >20ms
    uint64_t lt2ms{0}, b2_5ms{0}, b5_20ms{0}, gt20ms{0};
    void record(int64_t us) {
      if (us < min_us) min_us = us;
      if (us > max_us) max_us = us;
      sum_us += us;
      ++count;
      if (us < 2000)
        ++lt2ms;
      else if (us < 5000)
        ++b2_5ms;
      else if (us < 20000)
        ++b5_20ms;
      else
        ++gt20ms;
    }
    int64_t avg_us() const { return count ? sum_us / (int64_t)count : 0; }
    void reset() { *this = RxGapStats{}; }
  };
  // receiveLoop 线程私有 (无锁)
  RxGapStats sof_gap_live_;
  RxGapStats jfb_gap_live_;
  std::chrono::steady_clock::time_point last_sof_tp_{};
  std::chrono::steady_clock::time_point last_jfb_tp_{};
  bool first_sof_{true}, first_jfb_{true};
  // healthCheck 读快照 (rx_gap_mutex_ 保护)
  mutable std::mutex rx_gap_mutex_;
  RxGapStats sof_gap_snap_;
  RxGapStats jfb_gap_snap_;
  // torque RX 计数 & seq 追踪
  std::atomic<uint64_t> cmd_rx_count_{0};
  uint64_t last_cmd_rx_count_{0};
  std::atomic<uint64_t> cmd_seq_dup_{0};
  std::atomic<uint64_t> cmd_seq_skip_{0};
  uint8_t last_cmd_seq_{0xFF};
  // ── torque cache 健康计数 (替换锁内日志, 防止 send_thread_ 卡顿) ──
  std::atomic<uint64_t> cache_stale_warn_{0};         // age > 10ms 秒
  std::atomic<uint64_t> cache_stale_invalidated_{0};  // age > 100ms, 已开始发魔
  std::atomic<uint64_t> cache_no_cmd_{0};             // cmd_data_valid_=false
  std::atomic<uint64_t> cache_force_zero_{0};         // force_zero_torque 模式

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
    // [FIX] only state-check + reconnect under lock; sleep stays outside lock so
    //       sendLoop / receiveLoop are not stalled.
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

      // [FIX] copy shared_ptr under lock, call receive() outside.
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
        // [FIX] back off to avoid busy-loop CPU burn.
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
    // 7轴命令: [0..5]=臂关节目标位置(rad, route_mode), [6]=夹爪 tristate signal
    //   >+0.1=GRIP, <-0.1=RELEASE, else=STOP. sendLoop 转 0x0004 离散 flag.
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_position_target_to_mcu", rclcpp::SensorDataQoS(),
        std::bind(&HardwareInterfaceNode::cmdCallback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS());
    task_command_pub_ = this->create_publisher<std_msgs::msg::Int32>("/task_command", 10);
    link_diag_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/hardware_link_diag", 10);
    arm_state_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/arm_state", 10, [this](const std_msgs::msg::UInt8::SharedPtr msg) {
          // ArmState 合法值 0x00-0x06 (见 serial_protocol.hpp), 越界丢弃防止污染 MCU 状态包
          if (msg->data > 0x06) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[WARN] Invalid arm_state 0x%02X, ignored", msg->data);
            return;
          }
          std::lock_guard<std::mutex> lk(arm_state_mutex_);
          cached_arm_state_ = msg->data;
          arm_state_last_rx_ = std::chrono::steady_clock::now();
          arm_state_ever_received_ = true;
        });
  }

  // --- 回调函数 ---

  void cmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() < SerialProtocol::NUM_ARM_JOINTS) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Torque msg size %zu < %zu", msg->data.size(),
                  SerialProtocol::NUM_ARM_JOINTS);
      return;
    }
    cmd_rx_count_++;

    // ── Priority3: seq id 追踪 (torque msg 第8元素为 seq, 可选) ──
    // seq 字段由 torque_controller_node 在 data[7] 写入 (uint8 转 double)
    if (msg->data.size() >= 8) {
      const uint8_t seq = static_cast<uint8_t>(msg->data[7]);
      if (last_cmd_seq_ != 0xFF) {
        const uint8_t expected = static_cast<uint8_t>(last_cmd_seq_ + 1);
        if (seq == last_cmd_seq_) {
          cmd_seq_dup_++;
        } else if (seq != expected) {
          cmd_seq_skip_++;
        }
      }
      last_cmd_seq_ = seq;
    }

    std::lock_guard<std::mutex> lock(cmd_cache_mutex_);
    // data[0..5]=臂力矩, data[6]=夹爪, data[7]=seq(忽略缓存)
    const size_t n = std::min(msg->data.size(), SerialProtocol::NUM_ALL_JOINTS);
    for (size_t i = 0; i < n; ++i) {
      cached_cmd_[i] = static_cast<float>(msg->data[i]);
    }
    if (msg->data.size() < SerialProtocol::NUM_ALL_JOINTS) {
      cached_cmd_[SerialProtocol::NUM_ARM_JOINTS] = 0.0f;
    }
    last_cmd_update_ = this->now();
    last_cmd_steady_ = std::chrono::steady_clock::now();  // sendLoop 锁内 age 用
    cmd_data_valid_ = true;
  }

  // --- 串口收发 ---

  void sendLoop() {
    using Clock = std::chrono::steady_clock;
    using us = std::chrono::microseconds;
    const auto t_entry = Clock::now();
    int64_t serial_lock_us = 0;
    const int64_t timer_gap_us =
        (last_send_entry_ == Clock::time_point{})
            ? 0
            : std::chrono::duration_cast<us>(t_entry - last_send_entry_).count();
    last_send_entry_ = t_entry;

    // tx_attempt 时间戳仅在 sendRaw 真正尝试发包时更新, 避免串口未开时诊断撒谎
    {
      std::lock_guard<std::mutex> slock(serial_mutex_);
      if (!serial_port_ || !serial_port_->is_open()) {
        return;
      }
    }

    const auto t_param0 = Clock::now();
    const bool force_zero = this->get_parameter("force_zero_torque").as_bool();
    const int64_t param_us = std::chrono::duration_cast<us>(Clock::now() - t_param0).count();

    // ── 读取缓存力矩 (锁内仅做数据操作, 无任何日志/系统调用) ──
    float cmd_to_send[SerialProtocol::NUM_ARM_JOINTS];
    bool data_fresh = false;
    const auto t_cache0 = Clock::now();
    {
      std::lock_guard<std::mutex> lock(cmd_cache_mutex_);
      if (cmd_data_valid_) {
        // age 用 steady_clock 计算, 避免在锁内调用 this->now() (ROS 时钟可能慢)
        const double age_s =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_steady_)
                .count();
        if (age_s > 0.1) {
          cmd_data_valid_ = false;
          std::fill(cached_cmd_, cached_cmd_ + SerialProtocol::NUM_ALL_JOINTS, 0.0f);
          cache_stale_invalidated_++;  // 健康汇总: [HEALTH/TORQUE_CACHE] 输出
        } else if (age_s > 0.01) {
          cache_stale_warn_++;
        }
        data_fresh = cmd_data_valid_;
      } else {
        cache_no_cmd_++;
      }

      if (force_zero) {
        std::fill(cmd_to_send, cmd_to_send + SerialProtocol::NUM_ARM_JOINTS, 0.0f);
        cache_force_zero_++;
      } else if (data_fresh) {
        std::copy(cached_cmd_, cached_cmd_ + SerialProtocol::NUM_ARM_JOINTS, cmd_to_send);
      } else {
        std::fill(cmd_to_send, cmd_to_send + SerialProtocol::NUM_ARM_JOINTS, 0.0f);
      }
    }
    const int64_t cache_us = std::chrono::duration_cast<us>(Clock::now() - t_cache0).count();

    // ── 发送 0x0002 包 (载荷 = 6×q_target rad, byte 布局历史叫 torque) ──
    SerialProtocol::JointPositionTarget cmd;
    for (size_t i = 0; i < SerialProtocol::NUM_ARM_JOINTS; ++i) {
      cmd.positions[i] = cmd_to_send[i];
    }

    auto sendRaw = [&](const std::vector<uint8_t> &pkt) -> int64_t {
      tx_attempt_count_++;
      last_tx_attempt_activity_.store(std::chrono::steady_clock::now());
      const auto t_lock0 = Clock::now();
      std::lock_guard<std::mutex> slock(serial_mutex_);
      serial_lock_us += std::chrono::duration_cast<us>(Clock::now() - t_lock0).count();
      int64_t send_us = 0;
      try {
        if (!serial_port_ || !serial_port_->is_open()) return send_us;
        const auto t_send0 = Clock::now();
        const size_t sent = serial_port_->send(pkt);
        send_us = std::chrono::duration_cast<us>(Clock::now() - t_send0).count();
        if (sent != pkt.size()) {
          RCLCPP_WARN(this->get_logger(), "[WARN] Partial send: %zu/%zu", sent, pkt.size());
        } else {
          tx_packet_count_++;
          last_tx_success_activity_.store(std::chrono::steady_clock::now());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "[ERROR] Send failed: %s", e.what());
        serial_port_->close();
      }
      return send_us;
    };

    const auto t_build_cmd0 = Clock::now();
    const auto cmd_packet = SerialProtocol::buildPositionTargetPacket(cmd);
    const int64_t build_cmd_us =
        std::chrono::duration_cast<us>(Clock::now() - t_build_cmd0).count();
    const int64_t send_cmd_us = sendRaw(cmd_packet);

    // ── 分频发送夹爪包: [6] > +0.1 → GRIP, < -0.1 → RELEASE, else → STOP ──
    int64_t build_gripper_us = 0;
    int64_t send_gripper_us = 0;
    gripper_counter_ = (gripper_counter_ + 1) % gripper_divider_;
    if (gripper_counter_ == 0) {
      SerialProtocol::GripperAction action;
      if (force_zero) {
        action = SerialProtocol::GripperAction::STOP;
      } else {
        std::lock_guard<std::mutex> glock(cmd_cache_mutex_);
        const float g = cached_cmd_[SerialProtocol::NUM_ARM_JOINTS];  // index 6
        if (g > 0.1f)
          action = SerialProtocol::GripperAction::GRIP;
        else if (g < -0.1f)
          action = SerialProtocol::GripperAction::RELEASE;
        else
          action = SerialProtocol::GripperAction::STOP;
      }
      // 推导状态: GRIP→CLOSED, RELEASE→OPEN, STOP→保持
      if (action == SerialProtocol::GripperAction::GRIP)
        last_gripper_state_.store(SerialProtocol::GripperState::CLOSED);
      else if (action == SerialProtocol::GripperAction::RELEASE)
        last_gripper_state_.store(SerialProtocol::GripperState::OPEN);
      const auto t_build_gripper0 = Clock::now();
      const auto gripper_packet = SerialProtocol::buildGripperPacket(action);
      build_gripper_us = std::chrono::duration_cast<us>(Clock::now() - t_build_gripper0).count();
      send_gripper_us = sendRaw(gripper_packet);
    }

    // ── 分频发送臂状态包 0x0006: 10Hz, 同时承载 mission_executor 心跳 ──
    // 若 /arm_state 停发 >500ms (mission_executor 挂了), 这里跳过发送让 MCU watchdog
    // 检测到上位机死亡. 仅 hardware_interface 本身存活不算"上位机活着".
    int64_t build_status_us = 0;
    int64_t send_status_us = 0;
    status_counter_ = (status_counter_ + 1) % status_divider_;
    if (status_counter_ == 0) {
      bool fresh = false;
      SerialProtocol::ArmStatusPacket status;
      {
        std::lock_guard<std::mutex> lk(arm_state_mutex_);
        if (arm_state_ever_received_) {
          auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - arm_state_last_rx_)
                         .count();
          fresh = age < 500;
        }
        status.arm_state = static_cast<SerialProtocol::ArmState>(cached_arm_state_);
      }
      if (fresh) {
        status.task_progress = 0;
        status.error_code = SerialProtocol::ArmError::NO_ERROR;
        status.gripper_state = last_gripper_state_.load();
        const auto t_build_status0 = Clock::now();
        const auto status_packet = SerialProtocol::buildArmStatusPacket(status);
        build_status_us = std::chrono::duration_cast<us>(Clock::now() - t_build_status0).count();
        send_status_us = sendRaw(status_packet);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[0x0006] /arm_state stale, skip TX (mission_executor 挂了?)");
      }
    }

    // ── sendLoop 时序统计 (每1000次打印一次, 约1s) ──
    {
      const int64_t total_us = std::chrono::duration_cast<us>(Clock::now() - t_entry).count();
      send_stats_.record(total_us, serial_lock_us, timer_gap_us, param_us, cache_us, build_cmd_us,
                         send_cmd_us, build_gripper_us, send_gripper_us, build_status_us,
                         send_status_us);

      if (total_us > 1500) {
        RCLCPP_WARN(this->get_logger(),
                    "[SEND/TIMING] OVERRUN total=%ldus gap=%ldus param=%ldus cache=%ldus "
                    "build_cmd=%ldus send_cmd=%ldus build_gripper=%ldus "
                    "send_gripper=%ldus build_status=%ldus send_status=%ldus serial_lock=%ldus",
                    total_us, timer_gap_us, param_us, cache_us, build_cmd_us, send_cmd_us,
                    build_gripper_us, send_gripper_us, build_status_us, send_status_us,
                    serial_lock_us);
      }

      if (send_stats_.count % 1000 == 0) {
        RCLCPP_INFO(
            this->get_logger(),
            "[SEND/TIMING/1k] total: min=%ldus avg=%ldus max=%ldus | "
            "gap: avg=%ldus max=%ldus | param: avg=%ldus max=%ldus | "
            "cache: avg=%ldus max=%ldus | build_cmd: avg=%ldus max=%ldus | "
            "send_cmd: avg=%ldus max=%ldus | build_gripper: avg=%ldus max=%ldus | "
            "send_gripper: avg=%ldus max=%ldus | build_status: avg=%ldus max=%ldus | "
            "send_status: avg=%ldus max=%ldus | serial_lock: avg=%ldus | overruns=%lu",
            send_stats_.min_us, send_stats_.avg_us(), send_stats_.max_us,
            send_stats_.avg(send_stats_.gap_sum_us), send_stats_.gap_max_us,
            send_stats_.avg(send_stats_.param_sum_us), send_stats_.param_max_us,
            send_stats_.avg(send_stats_.cache_sum_us), send_stats_.cache_max_us,
            send_stats_.avg(send_stats_.build_cmd_sum_us), send_stats_.build_cmd_max_us,
            send_stats_.avg(send_stats_.send_cmd_sum_us), send_stats_.send_cmd_max_us,
            send_stats_.avg(send_stats_.build_gripper_sum_us), send_stats_.build_gripper_max_us,
            send_stats_.avg(send_stats_.send_gripper_sum_us), send_stats_.send_gripper_max_us,
            send_stats_.avg(send_stats_.build_status_sum_us), send_stats_.build_status_max_us,
            send_stats_.avg(send_stats_.send_status_sum_us), send_stats_.send_status_max_us,
            send_stats_.lock_avg_us(), send_stats_.overruns);
        send_stats_.reset();
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
              // ── SOF gap 记录 (仅 receiveLoop 线程写, 无锁) ──
              const auto now_tp = std::chrono::steady_clock::now();
              if (!first_sof_) {
                const int64_t gap_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(now_tp - last_sof_tp_)
                        .count();
                sof_gap_live_.record(gap_us);
              } else {
                first_sof_ = false;
              }
              last_sof_tp_ = now_tp;

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
      // ── JFB gap 记录 (仅 receiveLoop 线程写, 无锁) ──
      {
        const auto now_tp = std::chrono::steady_clock::now();
        if (!first_jfb_) {
          const int64_t gap_us =
              std::chrono::duration_cast<std::chrono::microseconds>(now_tp - last_jfb_tp_).count();
          jfb_gap_live_.record(gap_us);
        } else {
          first_jfb_ = false;
        }
        last_jfb_tp_ = now_tp;
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
    // 安全检查：丢弃含 NaN/Inf 的帧，防止 MoveIt FK 产生 NaN 变换导致 FCL 树崩溃
    for (size_t i = 0; i < SerialProtocol::NUM_ALL_JOINTS; ++i) {
      if (!std::isfinite(positions[i]) || !std::isfinite(velocities[i])) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[SAFETY] joint_states frame dropped: joint[%zu] pos=%.3f vel=%.3f "
                             "(NaN/Inf from serial)",
                             i, positions[i], velocities[i]);
        return;  // 整帧丢弃，保留上一帧有效数据
      }
    }

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

    // Clamp joint feedback to URDF limits so planner accepts current state (J6 continuous excluded)
    for (size_t k = 0; k < JointLimits::kNumClampedJoints; ++k) {
      const size_t idx = JointLimits::kClampIdx[k];
      if (msg.position[idx] > JointLimits::kUpper[k]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "%s feedback %.3f > %.3f, clamped", JointLimits::kName[k],
                             msg.position[idx], JointLimits::kUpper[k]);
        msg.position[idx] = JointLimits::kUpper[k];
      } else if (msg.position[idx] < JointLimits::kLower[k]) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "%s feedback %.3f < %.3f, clamped", JointLimits::kName[k],
                             msg.position[idx], JointLimits::kLower[k]);
        msg.position[idx] = JointLimits::kLower[k];
      }
    }

    joint_state_pub_->publish(msg);
  }

  void healthCheck() {
    static uint64_t last_rx_count = 0;
    static uint64_t last_tx_count = 0;
    static uint64_t last_crc_errors = 0;
    constexpr double kInterval = 5.0;

    uint64_t current_rx = rx_packet_count_.load();
    uint64_t current_tx = tx_packet_count_.load();
    uint64_t current_crc = rx_crc_errors_.load();
    uint64_t current_trx = cmd_rx_count_.load();

    double rx_rate = (current_rx - last_rx_count) / kInterval;
    double tx_rate = (current_tx - last_tx_count) / kInterval;
    double torque_rx_hz = (current_trx - last_cmd_rx_count_) / kInterval;
    uint64_t new_crc = current_crc - last_crc_errors;

    auto now = std::chrono::steady_clock::now();
    auto last_act = last_rx_activity_.load();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_act).count();
    bool serial_ok = serial_port_ && serial_port_->is_open();

    // ── swap RX gap live → snap ──
    RxGapStats sof_snap, jfb_snap;
    {
      std::lock_guard<std::mutex> lk(rx_gap_mutex_);
      sof_snap = sof_gap_live_;
      sof_gap_live_.reset();
      jfb_snap = jfb_gap_live_;
      jfb_gap_live_.reset();
    }

    uint64_t seq_dup = cmd_seq_dup_.exchange(0);
    uint64_t seq_skip = cmd_seq_skip_.exchange(0);

    if (simulation_mode_) {
      RCLCPP_INFO(this->get_logger(), "[HEALTH] TX: %.1f Hz | Serial: SIMULATION MODE", tx_rate);
    } else {
      if (elapsed_ms > 1000 && serial_ok) {
        RCLCPP_WARN(this->get_logger(),
                    "[HEALTH] RX thread inactive for %ld ms! Serial OK but no data", elapsed_ms);
      }

      const std::string status = serial_ok ? "OK" : "CLOSED";
      if (new_crc > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[HEALTH] TX: %.1f Hz | RX_OK: %.1f Hz | torque_rx: %.1f Hz | Serial: %s"
                    " | CRC: +%lu",
                    tx_rate, rx_rate, torque_rx_hz, status.c_str(), new_crc);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[HEALTH] TX: %.1f Hz | RX_OK: %.1f Hz | torque_rx: %.1f Hz | Serial: %s",
                    tx_rate, rx_rate, torque_rx_hz, status.c_str());
      }

      // ── [HEALTH/RX_PIPE] SOF & JFB gap 分布 ──
      if (sof_snap.count > 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[HEALTH/RX_PIPE] SOF: n=%lu avg=%ldus max=%ldus"
                    " | gap_bucket: <2ms=%lu 2-5ms=%lu 5-20ms=%lu >20ms=%lu"
                    " | JFB: n=%lu avg=%ldus max=%ldus gt20ms=%lu",
                    sof_snap.count, sof_snap.avg_us(), sof_snap.max_us, sof_snap.lt2ms,
                    sof_snap.b2_5ms, sof_snap.b5_20ms, sof_snap.gt20ms, jfb_snap.count,
                    jfb_snap.avg_us(), jfb_snap.max_us, jfb_snap.gt20ms);
      } else {
        RCLCPP_INFO(this->get_logger(), "[HEALTH/RX_PIPE] no SOF in this window");
      }

      // ── [HEALTH/TORQUE_CACHE] 力矩缓存健康 (替换锁内日志) ──
      const uint64_t stale_w = cache_stale_warn_.exchange(0);
      const uint64_t stale_i = cache_stale_invalidated_.exchange(0);
      const uint64_t no_tq = cache_no_cmd_.exchange(0);
      const uint64_t fzero = cache_force_zero_.exchange(0);
      if (stale_i > 0) {
        RCLCPP_ERROR(this->get_logger(),
                     "[HEALTH/TORQUE_CACHE] INVALIDATED=%lu (age>100ms) stale_warn=%lu "
                     "no_torque=%lu force_zero=%lu (in %.0fs)",
                     stale_i, stale_w, no_tq, fzero, kInterval);
      } else if (stale_w > 0 || no_tq > 0) {
        RCLCPP_WARN(this->get_logger(),
                    "[HEALTH/TORQUE_CACHE] stale_warn=%lu no_torque=%lu force_zero=%lu (in %.0fs)",
                    stale_w, no_tq, fzero, kInterval);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[HEALTH/TORQUE_CACHE] ok stale_warn=0 no_torque=0 force_zero=%lu", fzero);
      }
      if (seq_dup > 0 || seq_skip > 0) {
        RCLCPP_WARN(this->get_logger(), "[HEALTH/TORQUE_SEQ] dup=%lu skip=%lu (in %.0fs)", seq_dup,
                    seq_skip, kInterval);
      } else {
        RCLCPP_INFO(this->get_logger(), "[HEALTH/TORQUE_SEQ] dup=0 skip=0");
      }
    }

    last_rx_count = current_rx;
    last_tx_count = current_tx;
    last_crc_errors = current_crc;
    last_cmd_rx_count_ = current_trx;

    std_msgs::msg::Float64MultiArray diag_msg;
    diag_msg.data = {tx_rate,
                     rx_rate,
                     static_cast<double>(new_crc),
                     serial_ok ? 1.0 : 0.0,
                     static_cast<double>(elapsed_ms),
                     torque_rx_hz};
    link_diag_pub_->publish(diag_msg);
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
