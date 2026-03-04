/**
 * @file mission_executor_node.cpp
 * @brief Application Layer - ncurses TUI for Mission Control (v4.0)
 *
 * Provides a non-blocking, hotkey-driven interface using ncurses.
 * Features strict "Takeover Mode" isolation to prevent hotkey conflicts.
 */

#include <ncurses.h>
#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <clocale>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <thread>
#include <vector>

#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/move_to_cartesian_rpy.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"
#include "arv_v1_interfaces/srv/stop_cartesian_motion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"   // /task_command from hardware_interface
#include "std_msgs/msg/u_int8.hpp"  // /control_mode

// 控制模式常量 (与 serial_protocol.hpp ControlMode 保持同步)
namespace ControlMode {
constexpr uint8_t RELAX = 0;      // 全零力矩
constexpr uint8_t FREEDRIVE = 1;  // 仅重力补偿
constexpr uint8_t HOLD = 2;       // 重力补偿+PD
constexpr uint8_t EXECUTE = 3;    // 轨迹执行
}  // namespace ControlMode

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using ListTrajectories = arv_v1_interfaces::srv::ListTrajectories;
using SaveLastTrajectory = arv_v1_interfaces::srv::SaveLastTrajectory;
using MoveToCartesianRPY = arv_v1_interfaces::srv::MoveToCartesianRPY;
using StopCartesianMotion = arv_v1_interfaces::srv::StopCartesianMotion;
using GripperControl = arv_v1_interfaces::srv::GripperControl;
using namespace std::chrono_literals;

// --- 数据结构 ---

struct MissionState {
  std::string id;
  std::string trajectory;
  std::string description;
};

struct TrajectoryEntry {
  std::string name;
  std::string description;
};

// 日志环形缓冲区
class LogBuffer {
public:
  void add(const std::string& msg, int color_pair) {
    std::lock_guard<std::mutex> lock(mu_);
    if (logs_.size() >= max_size_) logs_.pop_front();
    logs_.push_back({msg, color_pair});
  }
  std::vector<std::pair<std::string, int>> get() {
    std::lock_guard<std::mutex> lock(mu_);
    return {logs_.begin(), logs_.end()};
  }

private:
  std::deque<std::pair<std::string, int>> logs_;
  const size_t max_size_ = 5;  // 底部显示5行日志
  std::mutex mu_;
};

// --- 颜色定义 ---
#define COLOR_PAIR_DEFAULT 1
#define COLOR_PAIR_HEADER 2
#define COLOR_PAIR_SUCCESS 3
#define COLOR_PAIR_ERROR 4
#define COLOR_PAIR_WARNING 5
#define COLOR_PAIR_HIGHLIGHT 6

// --- 节点 ---

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    initNcurses();

    // [FIX] getenv("HOME") 可返回 nullptr，构造 std::string 是 UB
    const char* home_env = getenv("HOME");
    if (!home_env) {
      RCLCPP_FATAL(get_logger(), "HOME environment variable not set");
      throw std::runtime_error("HOME env not set");
    }
    std::string home(home_env);
    trajectory_dir_ = home + "/ros2_ws/src/arv_v1_moveit/config/trajectories";
    mission_yaml_path_ = home + "/ros2_ws/src/arv_v1_moveit/config/mission_sequence.yaml";

    load_client_ = create_client<LoadTrajectory>("/load_trajectory");
    save_client_ = create_client<SaveLastTrajectory>("/save_last_trajectory");
    cartesian_client_ = create_client<MoveToCartesianRPY>("/move_to_cartesian_rpy");
    stop_cartesian_client_ = create_client<StopCartesianMotion>("/stop_cartesian_motion");
    gripper_client_ = create_client<GripperControl>("/gripper_control");

    // 订阅末端位姿 (30Hz from cartesian_controller_node)
    ee_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cartesian_controller/current_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(pose_mu_);
          current_ee_pose_ = *msg;
          has_ee_pose_ = true;
        });

    // 订阅下位机任务指令 (Int32: cmd<<16 | param<<8 | seq)
    task_command_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/task_command", 10,
        std::bind(&MissionExecutorNode::onTaskCommand, this, std::placeholders::_1));

    // 控制模式发布者
    control_mode_pub_ = create_publisher<std_msgs::msg::UInt8>("/control_mode", 10);

    // [FIX] 立即发布 HOLD，防止 torque_controller 默认 RELAX 导致上电掉落。
    // torque_controller 在 sleep 2 前已启动，此时订阅已建立。
    publishControlMode(ControlMode::HOLD);

    // 非阻塞服务检测 (避免 TUI 启动黑屏)
    log("Checking services (non-blocking)...", COLOR_PAIR_DEFAULT);
    if (!load_client_->wait_for_service(2s)) {
      logWarn("load_trajectory service not ready, will retry on use");
    }
    // 其他服务不阻塞等待，首次调用时自然检测
    save_client_->wait_for_service(500ms);
    cartesian_client_->wait_for_service(500ms);

    loadMissionSequence();
    fetchTrajectories();

    // 再次发布，保证 wait_for_service 期间重启的节点也能接收到
    publishControlMode(ControlMode::HOLD);
    log("Mission Executor v4.0 ready", COLOR_PAIR_SUCCESS);
  }

  ~MissionExecutorNode() {
    // async_ destroyed automatically (RAII) before shutdownNcurses
    shutdownNcurses();
  }

  void run() {
    int ch;
    int mode_broadcast_counter_ = 0;
    while (rclcpp::ok() && running_) {
      drawUI();
      ch = getch();  // 非阻塞, timeout=100ms
      if (ch != ERR) {
        handleInput(ch);
      }
      rclcpp::spin_some(get_node_base_interface());
      // 每 10 次循环 (~1s) 重发一次当前控制模式，防止节点重启后模式丢失
      if (++mode_broadcast_counter_ >= 10) {
        mode_broadcast_counter_ = 0;
        auto msg = std_msgs::msg::UInt8();
        msg.data = control_mode_;
        control_mode_pub_->publish(msg);
      }
    }
  }

private:
  // --- Async Task Runner ---
  class AsyncTaskRunner {
    using Task = std::packaged_task<void()>;
    std::thread worker_;
    std::queue<Task> tasks_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};

  public:
    AsyncTaskRunner() {
      worker_ = std::thread([this] {
        while (true) {
          Task task;
          {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [this] { return stop_ || !tasks_.empty(); });
            if (stop_ && tasks_.empty()) return;
            task = std::move(tasks_.front());
            tasks_.pop();
          }
          task();
        }
      });
    }
    ~AsyncTaskRunner() {
      {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
      }
      cv_.notify_one();
      if (worker_.joinable()) worker_.join();
    }
    template <typename F>
    void post(F&& fn) {
      {
        std::lock_guard<std::mutex> lk(mu_);
        tasks_.push(Task(std::forward<F>(fn)));
      }
      cv_.notify_one();
    }
  };

  AsyncTaskRunner async_;
  bool running_ = true;
  LogBuffer log_buffer_;

  // --- UI/UX 核心状态 ---
  enum class View { STATE_MACHINE, TRAJECTORY, CARTESIAN, GRIPPER, HELP };
  View view_ = View::STATE_MACHINE;

  bool takeover_mode_ = false;  // "接管模式"，隔离全局按键

  // 弹窗输入状态
  enum class InputMode { NONE, SAVE_NAME, SAVE_DESC, DELETE_CONFIRM, OVERWRITE_CONFIRM };
  InputMode input_mode_ = InputMode::NONE;
  std::string input_buffer_;
  std::string pending_name_;

  // --- 状态机 ---
  std::vector<MissionState> states_;
  size_t current_idx_ = 0;
  std::string mission_name_;
  std::string mission_desc_;
  std::string reset_trajectory_;
  std::string mission_yaml_path_;
  std::atomic<bool> executing_{false};

  // --- 轨迹管理 ---
  std::vector<TrajectoryEntry> trajectories_;
  std::string trajectory_dir_;
  static constexpr size_t PER_PAGE = 7;
  size_t traj_page_ = 0;

  // --- 夹爪状态 ---
  double gripper_torque_cmd_ = 0.0;  // 本地预设想发的力 (N, prismatic joint)
  double gripper_last_sent_ = 0.0;   // 实际最后发送的力 (N)

  // --- 笛卡尔状态 ---
  double cartesian_step_ = 0.05;  // 5cm
  geometry_msgs::msg::PoseStamped current_ee_pose_;
  std::mutex pose_mu_;
  bool has_ee_pose_ = false;

  // --- 控制模式 ---
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub_;
  uint8_t control_mode_ = ControlMode::HOLD;

  void publishControlMode(uint8_t mode) {
    control_mode_ = mode;
    auto msg = std_msgs::msg::UInt8();
    msg.data = mode;
    control_mode_pub_->publish(msg);
    static const char* names[] = {"RELAX", "FREEDRIVE", "HOLD", "EXECUTE"};
    log(std::string("[MODE] -> ") + (mode <= 3 ? names[mode] : "?"),
        mode == 0 ? COLOR_PAIR_WARNING : COLOR_PAIR_SUCCESS);
  }

  // --- ROS2 客户端 ---
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<SaveLastTrajectory>::SharedPtr save_client_;
  rclcpp::Client<MoveToCartesianRPY>::SharedPtr cartesian_client_;
  rclcpp::Client<StopCartesianMotion>::SharedPtr stop_cartesian_client_;
  rclcpp::Client<GripperControl>::SharedPtr gripper_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_command_sub_;
  uint8_t last_task_seq_ = 0xFF;  // 去重: 上一次处理的 seq

  // --- Ncurses 初始化 ---

  void initNcurses() {
    initscr();             // 初始化
    cbreak();              // 禁用行缓冲，直接读字符
    noecho();              // 不回显
    keypad(stdscr, TRUE);  // 捕获特殊按键(方向键、F1等)
    timeout(100);          // getch() 阻塞 100ms，保证 10Hz 顺滑刷新

    if (has_colors()) {
      start_color();
      init_pair(COLOR_PAIR_DEFAULT, COLOR_WHITE, COLOR_BLACK);
      init_pair(COLOR_PAIR_HEADER, COLOR_CYAN, COLOR_BLACK);
      init_pair(COLOR_PAIR_SUCCESS, COLOR_GREEN, COLOR_BLACK);
      init_pair(COLOR_PAIR_ERROR, COLOR_RED, COLOR_BLACK);
      init_pair(COLOR_PAIR_WARNING, COLOR_YELLOW, COLOR_BLACK);
      init_pair(COLOR_PAIR_HIGHLIGHT, COLOR_BLACK, COLOR_WHITE);  // 反色
    }
  }

  void shutdownNcurses() { endwin(); }

  void log(const std::string& msg, int color = COLOR_PAIR_DEFAULT) {
    log_buffer_.add(msg, color);
    // 移除 RCLCPP_INFO，防止底层的标准输出与 ncurses 争夺屏幕控制权导致 UI 撕裂/上移
  }

  void logErr(const std::string& msg) { log(msg, COLOR_PAIR_ERROR); }
  void logOk(const std::string& msg) { log(msg, COLOR_PAIR_SUCCESS); }
  void logWarn(const std::string& msg) { log(msg, COLOR_PAIR_WARNING); }

  // --- 后台加载 ---

  void loadMissionSequence() {
    std::lock_guard<std::mutex> lk(status_mu_);
    states_.clear();
    if (!std::filesystem::exists(mission_yaml_path_)) {
      logWarn("Mission YAML not found, using IDLE.");
      states_.push_back({"IDLE", "", "等待指令"});
      return;
    }
    try {
      auto root = YAML::LoadFile(mission_yaml_path_);
      auto mission = root["mission"];
      mission_name_ = mission["name"].as<std::string>("unnamed");
      mission_desc_ = mission["description"].as<std::string>("");
      reset_trajectory_ = mission["reset_trajectory"].as<std::string>("");
      for (const auto& s : mission["states"]) {
        MissionState ms;
        ms.id = s["id"].as<std::string>("?");
        ms.trajectory = s["trajectory"].as<std::string>("");
        ms.description = s["description"].as<std::string>("");
        states_.push_back(ms);
      }
    } catch (const std::exception& e) {
      logErr("Parse mission YAML failed.");
      states_.push_back({"IDLE", "", "等待指令(配置错误)"});
    }
    current_idx_ = 0;
  }

  void fetchTrajectories() {
    if (!load_client_->service_is_ready()) return;
    auto client = create_client<ListTrajectories>("/list_trajectories");
    if (!client->wait_for_service(1s)) return;
    auto req = std::make_shared<ListTrajectories::Request>();
    auto fut = client->async_send_request(req);
    // [FIX] 必须捕获 client，否则函数返回后 client 析构 → Broken promise
    async_.post([this, client, fut = std::move(fut)]() mutable {
      try {
        if (fut.wait_for(2s) == std::future_status::ready) {
          auto res = fut.get();
          std::lock_guard<std::mutex> lk(status_mu_);
          trajectories_.clear();
          for (size_t i = 0; i < res->names.size(); i++) {
            TrajectoryEntry e;
            e.name = res->names[i];
            e.description = i < res->descriptions.size() ? res->descriptions[i] : "";
            trajectories_.push_back(e);
          }
          traj_page_ = 0;
        }
      } catch (const std::exception& e) {
        logErr(std::string("Exception: ") + e.what());
      } catch (...) {
        logErr("Unknown exception");
      }
    });
  }

  std::mutex status_mu_;  // 保护后台数据刷新

  // --- 输入分发 (核心防冲突逻辑) ---

  void handleInput(int ch) {
    if (input_mode_ != InputMode::NONE) {
      handleStringInput(ch);
      return;
    }

    // [Z] 全局夹爪切换：任何视图/接管模式下均有效
    if (ch == 'z' || ch == 'Z') {
      toggleGripper();
      return;
    }

    if (takeover_mode_) {
      handleTakeoverInput(ch);
      return;
    }

    // 全局防冲突: 此时不在接管模式，也不在输入框，允许 M/T/C/G 切换
    handleGlobalInput(ch);
  }

  void handleGlobalInput(int ch) {
    // 退出
    if (ch == 'q' || ch == 'Q') {
      running_ = false;
      return;
    }

    // 视图切换
    if (ch == 'm' || ch == 'M') {
      view_ = View::STATE_MACHINE;
      return;
    }
    if (ch == 't' || ch == 'T') {
      view_ = View::TRAJECTORY;
      fetchTrajectories();
      return;
    }
    if (ch == 'h' || ch == 'H') {
      view_ = View::HELP;
      return;
    }

    // 需要接管的视图
    if (ch == 'c' || ch == 'C') {
      view_ = View::CARTESIAN;
      return;
    }
    if (ch == 'g' || ch == 'G') {
      view_ = View::GRIPPER;
      return;
    }

    // 视图内操作
    if (view_ == View::STATE_MACHINE)
      handleSMCommand(ch);
    else if (view_ == View::TRAJECTORY)
      handleTrajCommand(ch);
    else if (view_ == View::CARTESIAN) {
      // Cartesian 视图：Enter 触发接管模式
      if (ch == '\n' || ch == '\r') {
        takeover_mode_ = true;
        logWarn("Entered Takeover mode. Press 'Esc' to release.");
      }
    } else if (view_ == View::GRIPPER) {
      // 夹爪视图：Z 已全局生效，此处仅处理 Space 停止
      handleGripperInput(ch);
    }
  }

  void handleTakeoverInput(int ch) {
    if (ch == 27) {  // Esc 退出接管
      takeover_mode_ = false;
      if (view_ == View::CARTESIAN && stop_cartesian_client_->service_is_ready()) {
        auto req = std::make_shared<StopCartesianMotion::Request>();
        stop_cartesian_client_->async_send_request(req);
        log("Cartesian motion stopped.", COLOR_PAIR_DEFAULT);
      } else {
        log("Exited Takeover mode.", COLOR_PAIR_DEFAULT);
      }
      return;
    }

    if (view_ == View::CARTESIAN) handleCartesianTakeover(ch);
  }

  void handleStringInput(int ch) {
    // 字符串弹窗统一处理
    if (ch == 27) {  // Esc cancel
      input_mode_ = InputMode::NONE;
      log("Input canceled.");
      return;
    }

    if (input_mode_ == InputMode::DELETE_CONFIRM || input_mode_ == InputMode::OVERWRITE_CONFIRM) {
      // 单字符确认
      if (input_mode_ == InputMode::DELETE_CONFIRM)
        handleDeleteConfirm(ch);
      else if (input_mode_ == InputMode::OVERWRITE_CONFIRM)
        handleOverwrite(ch);
      return;
    }

    // 否则是字符累加
    if (ch == '\n' || ch == '\r') {
      std::string val = input_buffer_;
      input_buffer_.clear();
      if (input_mode_ == InputMode::SAVE_NAME)
        handleSaveName(val);
      else if (input_mode_ == InputMode::SAVE_DESC)
        handleSaveDesc(val);
    } else if (ch == KEY_BACKSPACE || ch == 127 || ch == '\b') {
      if (!input_buffer_.empty()) input_buffer_.pop_back();
    } else if (isprint(ch)) {
      // 限制名字不能有空格
      if (input_mode_ == InputMode::SAVE_NAME && ch == ' ') return;
      input_buffer_.push_back(ch);
    }
  }

  // 下位机任务指令回调 (Int32: cmd<<16 | param<<8 | seq)
  void onTaskCommand(const std_msgs::msg::Int32::SharedPtr msg) {
    const uint8_t cmd = static_cast<uint8_t>((msg->data >> 16) & 0xFF);
    const uint8_t param = static_cast<uint8_t>((msg->data >> 8) & 0xFF);
    const uint8_t seq = static_cast<uint8_t>(msg->data & 0xFF);

    // 去重: 同一 seq 不重复执行
    if (seq == last_task_seq_) return;
    last_task_seq_ = seq;

    char log_buf[64];
    switch (cmd) {
      case 0x01:  // EMERGENCY_STOP
        logWarn("[HW] EMERGENCY_STOP");
        resetToIdle();
        break;
      case 0x02:  // RESET_HOME
        logOk("[HW] RESET_HOME");
        resetToIdle();
        break;
      case 0x10:  // PICK_ORE
        snprintf(log_buf, sizeof(log_buf), "[HW] PICK_ORE ore_id=%u", param);
        logOk(log_buf);
        executeTrajectoryByKey("pick_ore_" + std::to_string(param));
        break;
      case 0x11:  // STOW_ORE
        snprintf(log_buf, sizeof(log_buf), "[HW] STOW_ORE slot_id=%u", param);
        logOk(log_buf);
        executeTrajectoryByKey("stow_slot_" + std::to_string(param));
        break;
      case 0x20:  // MOVE_TO_EXCHANGE
        logOk("[HW] MOVE_TO_EXCHANGE");
        executeTrajectoryByKey("move_to_exchange");
        break;
      case 0x21:  // EXECUTE_EXCHANGE
        snprintf(log_buf, sizeof(log_buf), "[HW] EXECUTE_EXCHANGE slot_id=%u", param);
        logOk(log_buf);
        executeTrajectoryByKey("exchange_slot_" + std::to_string(param));
        break;
      case 0x30:  // NEXT_STEP
        logOk("[HW] NEXT_STEP");
        {
          std::lock_guard<std::mutex> lk(status_mu_);
          if (current_idx_ + 1 < states_.size()) {
            current_idx_++;
            log("State -> " + states_[current_idx_].id, COLOR_PAIR_WARNING);
          }
        }
        break;
      case 0x31:  // ABORT_TASK
        logWarn("[HW] ABORT_TASK");
        executing_ = false;
        break;
      case 0x40:  // GRIPPER_CMD
        snprintf(log_buf, sizeof(log_buf), "[HW] GRIPPER_CMD param=%u", param);
        logOk(log_buf);
        sendGripper(param == 1 ? 5.0 : (param == 0 ? -5.0 : 0.0));
        break;
      case 0x50:  // SET_CONTROL_MODE
        snprintf(log_buf, sizeof(log_buf), "[HW] SET_CONTROL_MODE param=%u", param);
        logOk(log_buf);
        if (param <= ControlMode::EXECUTE) {
          publishControlMode(param);
        } else {
          logWarn("[HW] Invalid control mode value");
        }
        break;
      default:
        snprintf(log_buf, sizeof(log_buf), "[HW] Unknown TaskCmd 0x%02X param=%u", cmd, param);
        logWarn(log_buf);
        break;
    }
  }

  // 按轨迹名执行 (供 TaskCmd 和 TUI 共用)
  void executeTrajectoryByKey(const std::string& name) {
    if (control_mode_ != ControlMode::HOLD) {
      logWarn("须先切到 HOLD 模式才能执行: " + name);
      return;
    }
    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      logWarn("Busy, ignoring: " + name);
      return;
    }
    log("Exec traj: " + name + "...", COLOR_PAIR_HEADER);
    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = name;
    req->execute = true;
    // [FIX] try-catch 防 async_send_request 异常导致 executing_ 卡死
    try {
      auto fut = load_client_->async_send_request(req);
      async_.post([this, fut = std::move(fut), name]() mutable {
        try {
          // [FIX] 加超时，防服务节点崩溃时永久阻塞
          if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto res = fut.get();
            if (res->success)
              logOk("Done: " + name);
            else
              logErr("Fail: " + name + " - " + res->message);
          } else {
            logErr("Timeout: " + name);
          }
        } catch (const std::exception& e) {
          logErr("Error: " + std::string(e.what()));
        }
        executing_ = false;
      });
    } catch (const std::exception& e) {
      logErr("Request failed: " + std::string(e.what()));
      executing_ = false;
    }
  }

  // --- 状态机操作 ---

  void handleSMCommand(int k) {
    if (k == 'e' || k == 'E')
      executeCurrentState();
    else if (k == 'x' || k == 'X')
      resetToIdle();
    else if (k == 'r' || k == 'R') {
      loadMissionSequence();
      logOk("Sequence reloaded from yaml.");
    }
    // 控制模式切换: [0] RELAX  [1] FREEDRIVE  [2] HOLD
    else if (k == '0')
      publishControlMode(ControlMode::RELAX);
    else if (k == '1')
      publishControlMode(ControlMode::FREEDRIVE);
    else if (k == '2')
      publishControlMode(ControlMode::HOLD);
  }

  void executeCurrentState() {
    if (control_mode_ != ControlMode::HOLD) {
      logWarn("须先切到 HOLD 模式 [2] 才能执行轨迹");
      return;
    }
    if (current_idx_ >= states_.size()) {
      log("All states finished.");
      return;
    }

    auto& st = states_[current_idx_];
    if (st.trajectory.empty()) {
      if (current_idx_ + 1 < states_.size()) {
        current_idx_++;
        log("[" + st.id + "] Skipped (no traj) -> " + states_[current_idx_].id);
      }
      return;
    }

    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      logWarn("Busy executing...");
      return;
    }
    log("Exec [" + st.id + "] : " + st.trajectory + "...", COLOR_PAIR_HEADER);

    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = st.trajectory;
    req->execute = true;
    try {
      auto fut = load_client_->async_send_request(req);
      async_.post([this, fut = std::move(fut), id = st.id]() mutable {
        try {
          if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto res = fut.get();
            if (res->success) {
              logOk("Success: " + id + " (" + std::to_string(res->duration) + "s)");
              std::lock_guard<std::mutex> lk(status_mu_);
              if (current_idx_ + 1 < states_.size()) current_idx_++;
            } else {
              logErr("Fail: " + id + " - " + res->message);
            }
          } else {
            logErr("Timeout: " + id);
          }
        } catch (const std::exception& e) {
          logErr("Error: " + std::string(e.what()));
        }
        executing_ = false;
      });
    } catch (const std::exception& e) {
      logErr("Request failed: " + std::string(e.what()));
      executing_ = false;
    }
  }

  void resetToIdle() {
    if (!reset_trajectory_.empty()) {
      bool expected = false;
      if (!executing_.compare_exchange_strong(expected, true)) return;
      log("Resetting via: " + reset_trajectory_ + "...");
      auto req = std::make_shared<LoadTrajectory::Request>();
      req->name = reset_trajectory_;
      req->execute = true;
      try {
        auto fut = load_client_->async_send_request(req);
        async_.post([this, fut = std::move(fut)]() mutable {
          try {
            if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
              auto res = fut.get();
              if (res->success) {
                logOk("Reset OK.");
                std::lock_guard<std::mutex> lk(status_mu_);
                current_idx_ = 0;
              } else {
                logErr("Reset Fail: " + res->message);
              }
            } else {
              logErr("Timeout: reset trajectory");
            }
          } catch (const std::exception& e) {
            logErr(std::string("Exception: ") + e.what());
          } catch (...) {
            logErr("Unknown exception");
          }
          executing_ = false;
        });
      } catch (const std::exception& e) {
        logErr("Request failed: " + std::string(e.what()));
        executing_ = false;
      }
    } else {
      current_idx_ = 0;
      logOk("Reset to IDLE.");
    }
  }

  // --- 轨迹管理操作 ---

  void handleTrajCommand(int k) {
    if (k == 'r' || k == 'R') {
      fetchTrajectories();
      log("List refreshed.");
    } else if (k == 's' || k == 'S') {
      input_mode_ = InputMode::SAVE_NAME;
      input_buffer_.clear();
    } else if (k == 'n' || k == 'N') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + 1) % pages;
    } else if (k == 'p' || k == 'P') {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      if (pages > 1) traj_page_ = (traj_page_ + pages - 1) % pages;
    } else if (k == 'd' || k == 'D') {
      if (trajectories_.empty()) return;
      input_mode_ = InputMode::DELETE_CONFIRM;
    } else if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) executeTraj(trajectories_[idx].name);
    }
  }

  void executeTraj(const std::string& name) {
    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      logWarn("Busy...");
      return;
    }
    log("Executing: " + name + "...");
    auto req = std::make_shared<LoadTrajectory::Request>();
    req->name = name;
    req->execute = true;
    try {
      auto fut = load_client_->async_send_request(req);
      async_.post([this, fut = std::move(fut), name]() mutable {
        try {
          if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
            auto res = fut.get();
            if (res->success)
              logOk("Done: " + name);
            else
              logErr("Fail: " + res->message);
          } else {
            logErr("Timeout: " + name);
          }
        } catch (const std::exception& e) {
          logErr(std::string("Exception: ") + e.what());
        } catch (...) {
          logErr("Unknown exception");
        }
        executing_ = false;
      });
    } catch (const std::exception& e) {
      logErr("Request failed: " + std::string(e.what()));
      executing_ = false;
    }
  }

  void handleSaveName(const std::string& name) {
    if (name.empty()) {
      input_mode_ = InputMode::NONE;
      logWarn("Save canceled.");
      return;
    }
    bool exists = false;
    for (const auto& t : trajectories_) {
      if (t.name == name) {
        exists = true;
        break;
      }
    }
    pending_name_ = name;
    input_mode_ = exists ? InputMode::OVERWRITE_CONFIRM : InputMode::SAVE_DESC;
  }

  void handleSaveDesc(const std::string& desc) {
    doSave(pending_name_, desc);
    input_mode_ = InputMode::NONE;
  }

  void handleOverwrite(int k) {
    if (k == 'y' || k == 'Y') {
      input_mode_ = InputMode::SAVE_DESC;
    } else {
      input_mode_ = InputMode::NONE;
      logWarn("Overwrite canceled.");
    }
  }

  void doSave(const std::string& name, const std::string& desc) {
    log("Saving " + name + "...");
    auto req = std::make_shared<SaveLastTrajectory::Request>();
    req->name = name;
    req->description = desc;
    try {
      auto fut = save_client_->async_send_request(req);
      async_.post([this, fut = std::move(fut), name]() mutable {
        try {
          if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
            auto res = fut.get();
            if (res->success)
              logOk("Saved: " + name);
            else
              logErr("Save Fail: " + res->message);
          } else {
            logErr("Save timeout: " + name);
          }
        } catch (const std::exception& e) {
          logErr(std::string("Exception: ") + e.what());
        } catch (...) {
          logErr("Unknown exception");
        }
        fetchTrajectories();
      });
    } catch (const std::exception& e) {
      logErr("Save request failed: " + std::string(e.what()));
    }
  }

  void handleDeleteConfirm(int k) {
    if (k >= '1' && k <= '9') {
      size_t idx = (k - '1') + traj_page_ * PER_PAGE;
      if (idx < trajectories_.size()) {
        std::string path = trajectory_dir_ + "/" + trajectories_[idx].name + ".yaml";
        try {
          if (std::filesystem::exists(path)) {
            std::filesystem::remove(path);
            logOk("Deleted: " + trajectories_[idx].name);
            fetchTrajectories();
          }
        } catch (const std::exception& e) {
          logErr(std::string("Exception: ") + e.what());
        } catch (...) {
          logErr("Unknown exception");
        }
      }
    }
    input_mode_ = InputMode::NONE;
  }

  // --- 夹爪全局切换 ---

  void toggleGripper() {
    // 当前为正（夹紧）→ 松开（-70 N）；否则 → 夹紧（70 N）
    gripper_torque_cmd_ = (gripper_torque_cmd_ > 0.0) ? -70.0 : 70.0;
    sendGripper(gripper_torque_cmd_);
  }

  // --- 夹爪直接控制：GRIPPER 视图内 Space 停止 ---

  void handleGripperInput(int ch) {
    if (ch == ' ') {
      gripper_torque_cmd_ = 0.0;
      sendGripper(0.0);
    }
  }

  void sendGripper(double torque) {
    gripper_last_sent_ = torque;
    auto req = std::make_shared<GripperControl::Request>();
    req->torque = torque;
    try {
      auto fut = gripper_client_->async_send_request(req);
      async_.post([this, fut = std::move(fut), torque]() mutable {
        try {
          if (fut.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            auto res = fut.get();
            if (res->success) {
              logOk("Gripper cmd sent: " + std::to_string(torque));
            } else {
              logErr("Gripper fail: " + res->message);
            }
          } else {
            logErr("Gripper timeout");
          }
        } catch (const std::exception& e) {
          logErr("Gripper exception: " + std::string(e.what()));
        }
      });
    } catch (const std::exception& e) {
      logErr("Gripper request failed: " + std::string(e.what()));
    }
  }

  // --- 接管操作：笛卡尔 (Jogging) ---

  void handleCartesianTakeover(int ch) {
    // 改变步长
    if (ch == '+' || ch == '=') {
      cartesian_step_ = std::min(1.0, cartesian_step_ + 0.01);
      return;
    }
    if (ch == '-') {
      cartesian_step_ = std::max(0.01, cartesian_step_ - 0.01);
      return;
    }

    double dx = 0, dy = 0, dz = 0, droll = 0, dpitch = 0, dyaw = 0;

    // 平移 W/S=X, A/D=Y, R/F=Z
    if (ch == 'w' || ch == 'W')
      dx = cartesian_step_;
    else if (ch == 's' || ch == 'S')
      dx = -cartesian_step_;
    else if (ch == 'a' || ch == 'A')
      dy = cartesian_step_;
    else if (ch == 'd' || ch == 'D')
      dy = -cartesian_step_;
    else if (ch == 'r' || ch == 'R')
      dz = cartesian_step_;
    else if (ch == 'f' || ch == 'F')
      dz = -cartesian_step_;

    // 姿态 方向键=Pitch/Yaw  Q/E=Roll
    else if (ch == KEY_UP)
      dpitch = cartesian_step_;
    else if (ch == KEY_DOWN)
      dpitch = -cartesian_step_;
    else if (ch == KEY_LEFT)
      dyaw = cartesian_step_;
    else if (ch == KEY_RIGHT)
      dyaw = -cartesian_step_;
    else if (ch == 'q')
      droll = -cartesian_step_;
    else if (ch == 'e')
      droll = cartesian_step_;
    else
      return;

    sendCartesianRelative(dx, dy, dz, droll, dpitch, dyaw);
  }

  // 从 quaternion 提取 RPY (ZYX convention)
  static void quaternionToRPY(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch,
                              double& yaw) {
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void sendCartesianRelative(double dx, double dy, double dz, double droll, double dpitch,
                             double dyaw) {
    geometry_msgs::msg::Pose cur_pose;
    {
      std::lock_guard<std::mutex> lk(pose_mu_);
      if (!has_ee_pose_) {
        logErr("No EE pose yet. Is cartesian_controller running?");
        return;
      }
      cur_pose = current_ee_pose_.pose;
    }

    double cur_roll, cur_pitch, cur_yaw;
    quaternionToRPY(cur_pose.orientation, cur_roll, cur_pitch, cur_yaw);

    auto req = std::make_shared<MoveToCartesianRPY::Request>();
    req->x = cur_pose.position.x + dx;
    req->y = cur_pose.position.y + dy;
    req->z = cur_pose.position.z + dz;
    req->roll = cur_roll + droll;
    req->pitch = cur_pitch + dpitch;
    req->yaw = cur_yaw + dyaw;
    req->velocity_scaling = 0.0;
    req->acceleration_scaling = 0.0;
    req->async_execution = true;

    char buf[128];
    snprintf(buf, sizeof(buf), "Jog -> (%.3f, %.3f, %.3f) RPY(%.2f, %.2f, %.2f)", req->x, req->y,
             req->z, req->roll, req->pitch, req->yaw);
    log(buf, COLOR_PAIR_DEFAULT);

    cartesian_client_->async_send_request(
        req, [this](rclcpp::Client<MoveToCartesianRPY>::SharedFuture fut) {
          try {
            auto res = fut.get();
            if (res->success)
              logOk("Jog OK (plan: " + std::to_string(res->planning_time).substr(0, 5) + "s)");
            else
              logErr("Jog fail: " + res->message);
          } catch (const std::exception& e) {
            logErr("Jog exception: " + std::string(e.what()));
          }
        });
  }

  // --- UTF-8 工具 ---

  /**
   * @brief 将 UTF-8 字符串截断至不超过 max_cols 个终端列宽，并以空格补齐。
   * ASCII = 1字节/1列；CJK (U+1100..U+FFFF 3字节) = 2列；其余多字节 = 1列。
   */
  static std::string utf8Pad(const std::string& s, int field_cols) {
    int cols = 0;
    size_t pos = 0;
    while (pos < s.size()) {
      unsigned char c = s[pos];
      int char_bytes, char_cols;
      if (c < 0x80) {
        char_bytes = 1;
        char_cols = 1;
      } else if (c < 0xE0) {
        char_bytes = 2;
        char_cols = 1;
      } else if (c < 0xF0) {
        char_bytes = 3;
        char_cols = 2;  // CJK 全角
      } else {
        char_bytes = 4;
        char_cols = 2;
      }
      if (cols + char_cols > field_cols) break;
      cols += char_cols;
      pos += char_bytes;
    }
    std::string result = s.substr(0, pos);
    result.append(field_cols - cols, ' ');
    return result;
  }

  // --- NCURSES UI 重绘 (全屏 10Hz) ---

  void drawUI() {
    erase();  // 清空缓存区
    int max_y, max_x;
    getmaxyx(stdscr, max_y, max_x);

    // 1. 顶部栏 F1-F4
    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(0, 0, "===================== ARV_V1 Mission Executor v4.0 =====================");
    mvprintw(1, 0, "|");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    auto drawTab = [&](const std::string& name, View v) {
      if (view_ == v) attron(A_BOLD | COLOR_PAIR(COLOR_PAIR_HIGHLIGHT));
      printw(" %s ", name.c_str());
      if (view_ == v) attroff(A_BOLD | COLOR_PAIR(COLOR_PAIR_HIGHLIGHT));
      printw("  ");
    };
    mvprintw(1, 2, " ");
    drawTab("[M] StateMachine", View::STATE_MACHINE);
    drawTab("[T] Trajectory", View::TRAJECTORY);
    drawTab("[C] Cartesian", View::CARTESIAN);
    drawTab("[G] Gripper", View::GRIPPER);

    // 控制模式指示器
    {
      static const char* mode_labels[] = {"RELAX", "FREEDRIVE", "HOLD", "EXECUTE"};
      int mode_color = (control_mode_ == ControlMode::RELAX)     ? COLOR_PAIR_ERROR
                     : (control_mode_ == ControlMode::FREEDRIVE) ? COLOR_PAIR_WARNING
                                                                 : COLOR_PAIR_SUCCESS;
      attron(A_BOLD | COLOR_PAIR(mode_color));
      printw(" [%s] ", control_mode_ <= 3 ? mode_labels[control_mode_] : "?");
      attroff(A_BOLD | COLOR_PAIR(mode_color));
    }

    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(1, 71, "|");
    mvprintw(2, 0, "------------------------------------------------------------------------");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    // 2. 接管模式全局提醒
    if (takeover_mode_) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR) | A_BOLD);
      mvprintw(3, 2, ">>> TAKEOVER MODE ACTIVE <<< [Esc] to exit. Global hotkeys M/T/C/G blocked.");
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR) | A_BOLD);
    } else {
      if (view_ == View::CARTESIAN) {
        attron(COLOR_PAIR(COLOR_PAIR_WARNING));
        mvprintw(3, 2, "Press [Enter] to takeover and control.");
        attroff(COLOR_PAIR(COLOR_PAIR_WARNING));
      } else {
        mvprintw(3, 2, "Global hotkeys active.");
      }
    }

    // 3. 核心视图区
    int cur_line = 5;
    std::lock_guard<std::mutex> lk(status_mu_);  // 防止访问后台数据时越界

    if (view_ == View::STATE_MACHINE) {
      mvprintw(cur_line++, 2, "Mission: %s - %s", mission_name_.c_str(),
               utf8Pad(mission_desc_, max_x - 12 - (int)mission_name_.size()).c_str());
      cur_line++;
      // 列布局: col4=[x] col8=ID(16) col25=Traj(16) col42=Desc(剩余) col(max_x-8)<-NOW
      for (size_t i = 0; i < states_.size(); i++) {
        if (i == current_idx_)
          attron(A_BOLD | COLOR_PAIR(COLOR_PAIR_WARNING));
        else if (i < current_idx_)
          attron(COLOR_PAIR(COLOR_PAIR_SUCCESS));
        else
          attron(A_DIM);

        const char* marker = (i < current_idx_ ? "*" : (i == current_idx_ ? ">" : " "));
        const std::string& traj_str =
            states_[i].trajectory.empty() ? "(No Traj)" : states_[i].trajectory;
        // 描述最多用至 (max_x - 42 - 8) 列，至少留 8 列给 " <- NOW"
        int desc_cols = std::max(1, max_x - 42 - 8);
        std::string desc_field = utf8Pad(states_[i].description, desc_cols);

        mvprintw(cur_line, 4, "[%s] ", marker);
        mvprintw(cur_line, 8, "%s", utf8Pad(states_[i].id, 16).c_str());
        mvprintw(cur_line, 25, "%s", utf8Pad(traj_str, 16).c_str());
        mvprintw(cur_line, 42, "%s", desc_field.c_str());
        if (i == current_idx_) mvprintw(cur_line, max_x - 8, "<- NOW");
        cur_line++;

        attroff(A_BOLD | COLOR_PAIR(COLOR_PAIR_WARNING));
        attroff(COLOR_PAIR(COLOR_PAIR_SUCCESS));
        attroff(A_DIM);
      }
      cur_line++;
      mvprintw(cur_line++, 2,
               "Hotkeys: [E] Execute   [X] Reset   [R] Reload   [0]Relax [1]Free [2]Hold");
    } else if (view_ == View::TRAJECTORY) {
      size_t pages = (trajectories_.size() + PER_PAGE - 1) / PER_PAGE;
      mvprintw(cur_line++, 2, "Database: %zu found (Page %zu/%zu)", trajectories_.size(),
               traj_page_ + 1, std::max((size_t)1, pages));
      cur_line++;
      size_t s = traj_page_ * PER_PAGE;
      size_t e = std::min(s + PER_PAGE, trajectories_.size());
      for (size_t i = s; i < e; i++) {
        mvprintw(cur_line++, 4, "[%zu] %-20s %s", (i - s + 1), trajectories_[i].name.c_str(),
                 trajectories_[i].description.c_str());
      }
      cur_line = 14;
      mvprintw(cur_line++, 2,
               "Hotkeys: [1-9] Execute   [S] Save Last   [D] Delete   [N/P] Next/Prev Page");
    } else if (view_ == View::CARTESIAN) {
      mvprintw(cur_line++, 2, "Cartesian Jogging Panel");
      cur_line++;
      {
        std::lock_guard<std::mutex> plk(pose_mu_);
        if (has_ee_pose_) {
          auto& p = current_ee_pose_.pose;
          double r, pi, y;
          quaternionToRPY(p.orientation, r, pi, y);
          attron(COLOR_PAIR(COLOR_PAIR_SUCCESS));
          mvprintw(cur_line++, 4, "EE Pos : X=%.3f  Y=%.3f  Z=%.3f", p.position.x, p.position.y,
                   p.position.z);
          mvprintw(cur_line++, 4, "EE RPY : R=%.2f  P=%.2f  Y=%.2f", r, pi, y);
          attroff(COLOR_PAIR(COLOR_PAIR_SUCCESS));
        } else {
          attron(COLOR_PAIR(COLOR_PAIR_ERROR));
          mvprintw(cur_line++, 4, "EE Pose: waiting for /cartesian_controller/current_pose ...");
          attroff(COLOR_PAIR(COLOR_PAIR_ERROR));
        }
      }
      cur_line++;
      attron(A_BOLD);
      mvprintw(cur_line++, 4, "Step Size: %.3f m / rad", cartesian_step_);
      attroff(A_BOLD);
      cur_line++;
      mvprintw(cur_line++, 4, "Translation: [W/S] X axis  [A/D] Y axis  [R/F] Z axis");
      mvprintw(cur_line++, 4, "Orientation: [Up/Down] Pitch  [Left/Right] Yaw  [Q/E] Roll");
      mvprintw(cur_line++, 4, "Config     : [+/-] Change Step Size");
    } else if (view_ == View::GRIPPER) {
      mvprintw(cur_line++, 2, "Gripper Status");
      cur_line++;

      // 当前状态
      const char* state_str = (gripper_last_sent_ > 0.0) ? "GRIP"
                            : (gripper_last_sent_ < 0.0) ? "RELEASE"
                                                         : "STOP";
      int state_color = (gripper_last_sent_ > 0.0) ? COLOR_PAIR_SUCCESS
                      : (gripper_last_sent_ < 0.0) ? COLOR_PAIR_WARNING
                                                   : COLOR_PAIR_DEFAULT;
      mvprintw(cur_line++, 4, "State : ");
      attron(COLOR_PAIR(state_color) | A_BOLD);
      printw("%s  (%.0f N)", state_str, gripper_last_sent_);
      attroff(COLOR_PAIR(state_color) | A_BOLD);

      cur_line++;
      mvprintw(cur_line++, 4, "Global hotkey (works everywhere):");
      mvprintw(cur_line++, 6, "[ Z ]     Toggle grip/release  (+70 N / -70 N)");
      mvprintw(cur_line++, 6, "[ SPACE ] Stop (0 N)  -- only in this view");
    }

    // 4. 输入区
    cur_line = max_y - 7;
    attron(COLOR_PAIR(COLOR_PAIR_HEADER));
    mvprintw(cur_line++, 0,
             "------------------------------------------------------------------------");
    attroff(COLOR_PAIR(COLOR_PAIR_HEADER));

    if (input_mode_ == InputMode::SAVE_NAME) {
      mvprintw(cur_line, 2, "Enter Trajectory Name (no spaces): %s", input_buffer_.c_str());
    } else if (input_mode_ == InputMode::SAVE_DESC) {
      mvprintw(cur_line, 2, "Enter Description: %s", input_buffer_.c_str());
    } else if (input_mode_ == InputMode::DELETE_CONFIRM) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR));
      mvprintw(cur_line, 2, "Delete? Press [1-9] matching index, or [Esc] to cancel.");
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR));
    } else if (input_mode_ == InputMode::OVERWRITE_CONFIRM) {
      attron(COLOR_PAIR(COLOR_PAIR_ERROR));
      mvprintw(cur_line, 2, "Name '%s' exists! Overwrite? [Y/N]", pending_name_.c_str());
      attroff(COLOR_PAIR(COLOR_PAIR_ERROR));
    } else {
      if (!takeover_mode_) mvprintw(cur_line, 2, "Press [H] for Help | [Q] to Quit");
    }

    // 5. 日志区
    cur_line++;
    auto logs = log_buffer_.get();
    for (const auto& l : logs) {
      attron(COLOR_PAIR(l.second));
      mvprintw(cur_line++, 0, "  %s", l.first.c_str());
      attroff(COLOR_PAIR(l.second));
    }

    refresh();  // 应用屏幕缓存
  }
};

int main(int argc, char** argv) {
  setlocale(LC_ALL, "");  // 使 ncurses 支持当前终端的字符集 (UTF-8)
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MissionExecutorNode>();
    node->run();
  } catch (const std::exception& e) {
    std::cerr << "Fatal: " << e.what() << std::endl;
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
