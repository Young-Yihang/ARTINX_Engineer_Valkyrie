/// @file mission_executor_node.cpp
/// @brief Headless mission state machine — MCU key events → trajectory sequencing + interrupt
/// allowlist + B fast-forward.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <thread>
#include <unordered_map>
#include <vector>

#include "arv_v1_interfaces/srv/cancel_current_trajectory.hpp"
#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/plan_to_preset.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace ControlMode {
constexpr uint8_t RELAX = 0;
constexpr uint8_t FREEDRIVE = 1;
constexpr uint8_t ARMED = 2;
}  // namespace ControlMode

using LoadTrajectory = arv_v1_interfaces::srv::LoadTrajectory;
using GripperControl = arv_v1_interfaces::srv::GripperControl;
using PlanToPreset = arv_v1_interfaces::srv::PlanToPreset;
using CancelCurrentTrajectory = arv_v1_interfaces::srv::CancelCurrentTrajectory;
using namespace std::chrono_literals;

// --- Binding 数据结构 ---

struct BindingStep {
  std::string trajectory;      // trajectory 名 (可含 {param} 模板, loader 已展开)
  bool wait_for_next = false;  // step 跑完是否停下等 B
};

struct Binding {
  std::vector<BindingStep> steps;
  std::set<uint8_t> interruptible_by;  // 序列执行中接受的 cmd 白名单
};

struct ActiveTask {
  std::string binding_key;
  size_t step_idx = 0;
  bool awaiting_next = false;  // 当前 step 已跑完, 阻塞在 wait_for_next
};

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    trajectory_timeout_s_ = declare_parameter<double>("trajectory_execution_timeout", 120.0);
    // 默认值用 ament_index 找 install share, 跨用户名 portable
    std::string pkg_share;
    try {
      pkg_share = ament_index_cpp::get_package_share_directory("arv_v1_moveit");
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "[Bootstrap] get_package_share_directory failed: %s", e.what());
      pkg_share = "/tmp";
    }
    key_bindings_path_ = declare_parameter<std::string>("key_bindings_path",
                                                        pkg_share + "/config/key_bindings.yaml");
    trajectory_dir_ =
        declare_parameter<std::string>("trajectory_dir", pkg_share + "/config/trajectories");

    // Callback groups: subscription / service client 各一. Reentrant 让 service
    // 客户端 wait_for 在不同线程跑, 不阻塞 onTaskCommand 派发.
    sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    load_client_ =
        create_client<LoadTrajectory>("/load_trajectory", rclcpp::ServicesQoS(), client_cb_group_);
    gripper_client_ =
        create_client<GripperControl>("/gripper_control", rclcpp::ServicesQoS(), client_cb_group_);
    plan_preset_client_ =
        create_client<PlanToPreset>("/plan_to_preset", rclcpp::ServicesQoS(), client_cb_group_);
    cancel_client_ = create_client<CancelCurrentTrajectory>(
        "/cancel_current_trajectory", rclcpp::ServicesQoS(), client_cb_group_);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = sub_cb_group_;

    task_command_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/task_command", 10,
        std::bind(&MissionExecutorNode::onTaskCommand, this, std::placeholders::_1), sub_opts);

    control_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/control_mode", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { control_mode_ = msg->data; }, sub_opts);

    control_mode_pub_ = create_publisher<std_msgs::msg::UInt8>("/control_mode", 10);
    arm_state_pub_ = create_publisher<std_msgs::msg::UInt8>("/arm_state", 10);

    arm_status_timer_ = create_wall_timer(100ms, [this]() {
      auto msg = std_msgs::msg::UInt8();
      msg.data = deriveArmState();
      arm_state_pub_->publish(msg);
    });

    mode_broadcast_timer_ = create_wall_timer(1s, [this]() {
      auto msg = std_msgs::msg::UInt8();
      msg.data = control_mode_;
      control_mode_pub_->publish(msg);
    });

    publishControlMode(ControlMode::RELAX);

    if (!load_client_->wait_for_service(2s)) {
      RCLCPP_WARN(get_logger(), "load_trajectory service not ready, will retry on use");
    }

    loadKeyBindings();
    publishControlMode(ControlMode::RELAX);
    RCLCPP_INFO(get_logger(), "Mission Executor ready (headless, %zu bindings)", bindings_.size());
  }

private:
  // --- State ---
  std::atomic<bool> executing_{false};
  double trajectory_timeout_s_ = 120.0;
  std::string key_bindings_path_;
  std::string trajectory_dir_;
  uint8_t control_mode_ = ControlMode::RELAX;
  uint8_t last_task_seq_ = 0xFF;

  std::unordered_map<std::string, Binding>
      bindings_;  // key="pick_obj_0".."pick_obj_5"/"stow_left"/...

  std::optional<ActiveTask> active_task_;
  std::mutex active_mu_;
  std::condition_variable task_cv_;
  std::atomic<bool> step_skip_requested_{false};   // B 砍 current step
  std::atomic<bool> task_abort_requested_{false};  // X/ESTOP/F 砍整个序列
  // 上一个 binding 正常跑完 (非 abort), 当前空闲但保持 hold 位. 操作手凭此区分
  // "READY = 刚启动空闲" vs "HOLDING = 序列做完等待下一指令".
  std::atomic<bool> last_sequence_completed_{false};

  // --- ROS2 ---
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<GripperControl>::SharedPtr gripper_client_;
  rclcpp::Client<PlanToPreset>::SharedPtr plan_preset_client_;
  rclcpp::Client<CancelCurrentTrajectory>::SharedPtr cancel_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_command_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_state_pub_;
  rclcpp::TimerBase::SharedPtr arm_status_timer_;
  rclcpp::TimerBase::SharedPtr mode_broadcast_timer_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;

  uint8_t deriveArmState() const {
    if (control_mode_ == ControlMode::RELAX) return 0x05;
    if (control_mode_ == ControlMode::FREEDRIVE) return 0x06;
    if (executing_) return 0x01;
    if (last_sequence_completed_) return 0x02;  // HOLDING: 序列做完, 在 hold 位待命
    return 0x00;                                // READY: 空闲, 上次 abort/未跑过
  }

  void publishControlMode(uint8_t mode) {
    control_mode_ = mode;
    auto msg = std_msgs::msg::UInt8();
    msg.data = mode;
    control_mode_pub_->publish(msg);
    static const char* names[] = {"RELAX", "FREEDRIVE", "ARMED"};
    RCLCPP_INFO(get_logger(), "[MODE] -> %s", mode <= 2 ? names[mode] : "?");
  }

  // --- key_bindings.yaml 加载 ---

  void loadKeyBindings() {
    bindings_.clear();
    if (!std::filesystem::exists(key_bindings_path_)) {
      RCLCPP_ERROR(get_logger(), "key_bindings.yaml not found: %s", key_bindings_path_.c_str());
      return;
    }
    try {
      auto root = YAML::LoadFile(key_bindings_path_);
      auto bs = root["bindings"];
      for (auto it = bs.begin(); it != bs.end(); ++it) {
        std::string key = it->first.as<std::string>();
        auto node = it->second;

        Binding b;
        if (node["interruptible_by"]) {
          for (const auto& c : node["interruptible_by"]) b.interruptible_by.insert(c.as<uint8_t>());
        }
        for (const auto& s : node["steps"]) {
          BindingStep step;
          step.trajectory = s["trajectory"].as<std::string>("");
          step.wait_for_next = s["wait_for_next"].as<bool>(false);
          b.steps.push_back(step);
        }

        // pick_obj 模板展开成 pick_obj_0..pick_obj_5
        if (key == "pick_obj") {
          for (int i = 0; i < 6; ++i) {
            Binding expanded = b;
            for (auto& st : expanded.steps) {
              std::string s = st.trajectory;
              auto pos = s.find("{param}");
              while (pos != std::string::npos) {
                s.replace(pos, 7, std::to_string(i));
                pos = s.find("{param}");
              }
              st.trajectory = s;
            }
            bindings_["pick_obj_" + std::to_string(i)] = expanded;
          }
        } else {
          bindings_[key] = b;
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Parse key_bindings.yaml failed: %s", e.what());
      return;
    }

    // 检查所有引用的 trajectory 文件是否存在, 缺的列出 WARN (不阻止启动)
    std::vector<std::string> missing;
    for (const auto& [key, b] : bindings_) {
      for (const auto& s : b.steps) {
        std::string path = trajectory_dir_ + "/" + s.trajectory + ".yaml";
        if (!std::filesystem::exists(path)) {
          missing.push_back(s.trajectory + " (referenced by " + key + ")");
        }
      }
    }
    if (!missing.empty()) {
      RCLCPP_WARN(get_logger(), "[KeyBindings] %zu trajectory files missing:", missing.size());
      for (const auto& m : missing) RCLCPP_WARN(get_logger(), "  - %s.yaml", m.c_str());
    }
  }

  // --- MCU Command Dispatch ---

  void onTaskCommand(const std_msgs::msg::Int32::SharedPtr msg) {
    const uint8_t cmd = static_cast<uint8_t>((msg->data >> 16) & 0xFF);
    const uint8_t param = static_cast<uint8_t>((msg->data >> 8) & 0xFF);
    const uint8_t seq = static_cast<uint8_t>(msg->data & 0xFF);

    if (seq == last_task_seq_) return;
    last_task_seq_ = seq;

    // 不受 binding 状态约束的全局 cmd
    switch (cmd) {
      case 0x01:
        RCLCPP_WARN(get_logger(), "[HW] EMERGENCY_STOP");
        emergencyStop();
        return;
      case 0x02:
        RCLCPP_INFO(get_logger(), "[HW] RESET_HOME");
        resetToIdle();
        return;
      case 0x31:
        RCLCPP_WARN(get_logger(), "[HW] ABORT_TASK");
        triggerTaskAbortAndWait();
        return;
      case 0x40:
        RCLCPP_INFO(get_logger(), "[HW] GRIPPER_CMD param=%u", param);
        sendGripperAsync(param == 1 ? 70.0 : (param == 0 ? -70.0 : 0.0));
        return;
      case 0x50:
        RCLCPP_INFO(get_logger(), "[HW] SET_CONTROL_MODE param=%u", param);
        if (param <= ControlMode::ARMED) publishControlMode(param);
        return;
    }

    // 启动类 cmd: 决定 binding key
    std::optional<std::string> start_key;
    switch (cmd) {
      case 0x10:
        if (param < 6)
          start_key = "pick_obj_" + std::to_string(param);
        else
          RCLCPP_WARN(get_logger(), "[HW] PICK_OBJ param=%u out of range, ignored", param);
        break;
      case 0x24:
        start_key = "stow_left";
        break;
      case 0x25:
        start_key = "stow_right";
        break;
      case 0x26:
        start_key = "retrieve_left";
        break;
      case 0x27:
        start_key = "retrieve_right";
        break;
      case 0x30:
        // B 推进/fast-forward
        handleNextStep();
        return;
      default:
        RCLCPP_WARN(get_logger(), "[HW] Unknown TaskCmd 0x%02X param=%u", cmd, param);
        return;
    }

    if (start_key) handleStartBinding(*start_key, cmd);
  }

  // --- 启动 binding / 处理打断白名单 ---

  void handleStartBinding(const std::string& key, uint8_t cmd) {
    auto it = bindings_.find(key);
    if (it == bindings_.end()) {
      RCLCPP_WARN(get_logger(), "[Binding] key '%s' not found", key.c_str());
      return;
    }

    // 检查是否在执行中
    bool has_active;
    {
      std::lock_guard<std::mutex> lk(active_mu_);
      has_active = active_task_.has_value();
    }

    if (has_active) {
      // 查 current binding 的白名单看是否允许 cmd 打断
      bool allowed;
      {
        std::lock_guard<std::mutex> lk(active_mu_);
        auto cur_it = bindings_.find(active_task_->binding_key);
        allowed = (cur_it != bindings_.end() && cur_it->second.interruptible_by.count(cmd) > 0);
      }
      if (!allowed) {
        RCLCPP_WARN(get_logger(), "[Binding] '%s' rejected (current task not interruptible)",
                    key.c_str());
        return;
      }
      // 允许打断 → abort 当前 → 启动新
      RCLCPP_INFO(get_logger(), "[Binding] interrupt current, switch to '%s'", key.c_str());
      triggerTaskAbort();
      // 等当前 detach 线程退出 (CV 醒 + load_trajectory cancel 返回)
      // 简单做法: 自旋等 active_task_ 清空, 最多 5s
      for (int i = 0; i < 100; ++i) {
        std::this_thread::sleep_for(50ms);
        std::lock_guard<std::mutex> lk(active_mu_);
        if (!active_task_) break;
      }
      task_abort_requested_ = false;  // 复位让新任务能跑
    }

    startBinding(key);
  }

  void startBinding(const std::string& key) {
    if (control_mode_ != ControlMode::ARMED) {
      RCLCPP_WARN(get_logger(), "[Binding] '%s' rejected: not ARMED", key.c_str());
      return;
    }
    // 防御: 残留 abort flag 静默吞掉新 binding 是历史 bug, 留个 warning 指纹
    if (task_abort_requested_.exchange(false)) {
      RCLCPP_WARN(get_logger(), "[Binding] stale abort flag cleared at start of '%s'", key.c_str());
    }
    {
      std::lock_guard<std::mutex> lk(active_mu_);
      active_task_ = ActiveTask{key, 0, false};
    }
    executing_ = true;
    last_sequence_completed_ = false;  // 新序列启动, 清 HOLDING 标记
    RCLCPP_INFO(get_logger(), "[Binding] start '%s'", key.c_str());

    std::thread([this, key]() { executeBinding(key); }).detach();
  }

  void executeBinding(const std::string& key) {
    auto it = bindings_.find(key);
    if (it == bindings_.end()) {
      RCLCPP_ERROR(get_logger(), "[Binding] executeBinding: '%s' missing", key.c_str());
      finishBinding();
      return;
    }
    const auto& binding = it->second;

    for (size_t i = 0; i < binding.steps.size(); ++i) {
      if (task_abort_requested_) break;
      if (control_mode_ != ControlMode::ARMED) {
        RCLCPP_WARN(get_logger(), "[Binding] mode not ARMED mid-sequence, aborting");
        task_abort_requested_ = true;
        break;
      }
      {
        std::lock_guard<std::mutex> lk(active_mu_);
        if (active_task_) active_task_->step_idx = i;
      }

      const auto& step = binding.steps[i];
      RCLCPP_INFO(get_logger(), "[Binding] '%s' step %zu/%zu: %s", key.c_str(), i + 1,
                  binding.steps.size(), step.trajectory.c_str());

      // ---- 调 LoadTrajectory 同步等 action result ----
      auto req = std::make_shared<LoadTrajectory::Request>();
      req->name = step.trajectory;
      req->execute = true;
      auto fut = load_client_->async_send_request(req);
      auto status = fut.wait_for(std::chrono::duration<double>(trajectory_timeout_s_));
      bool was_skip = step_skip_requested_.exchange(false);

      if (status != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "[Binding] step %zu LoadTrajectory timeout", i);
        break;
      }
      auto res = fut.get();
      if (!res->success) {
        // 自然失败 → abort sequence
        if (task_abort_requested_) break;
        RCLCPP_ERROR(get_logger(), "[Binding] step %zu fail: %s", i, res->message.c_str());
        break;
      }

      // B fast-forward 砍的轨迹返回 success+message="canceled", 直接进下一步.
      // 注: gripper schedule 由 trajectory_manager 自己 fork side-thread 触发 (Commit 7),
      // 这里不再 scheduleGripperActions, 避免重复 + 时序错位.
      if (was_skip || res->message == "canceled") {
        RCLCPP_INFO(get_logger(), "[Binding] step %zu canceled (fast-forward), next", i);
        continue;
      }

      if (step.wait_for_next) {
        std::unique_lock<std::mutex> lk(active_mu_);
        if (active_task_) active_task_->awaiting_next = true;
        RCLCPP_INFO(get_logger(), "[Binding] step %zu done, waiting for B", i);
        task_cv_.wait(lk, [&] { return step_skip_requested_ || task_abort_requested_; });
        if (active_task_) active_task_->awaiting_next = false;
        if (task_abort_requested_) break;
        step_skip_requested_ = false;
      }
    }

    finishBinding();
  }

  void finishBinding() {
    bool aborted = task_abort_requested_.load();
    {
      std::lock_guard<std::mutex> lk(active_mu_);
      if (active_task_) {
        RCLCPP_INFO(get_logger(), "[Binding] '%s' %s", active_task_->binding_key.c_str(),
                    aborted ? "aborted" : "finished");
      }
      active_task_.reset();
    }
    step_skip_requested_ = false;
    task_abort_requested_ = false;
    executing_ = false;
    // 正常完成 → HOLDING; abort/X/ESTOP → 不 hold (resetToIdle 会清, ESTOP 切 RELAX)
    last_sequence_completed_ = !aborted;
  }

  // --- B 推进 ---

  void handleNextStep() {
    bool has_active;
    bool awaiting;
    {
      std::lock_guard<std::mutex> lk(active_mu_);
      has_active = active_task_.has_value();
      awaiting = has_active && active_task_->awaiting_next;
    }
    if (!has_active) {
      RCLCPP_DEBUG(get_logger(), "[B] no active task, ignored");
      return;
    }
    step_skip_requested_ = true;
    if (awaiting) {
      task_cv_.notify_all();
    } else {
      // 跑轨迹中 → 调 /cancel_current_trajectory 砍 FJT, LoadTrajectory 会返回 canceled
      RCLCPP_INFO(get_logger(), "[B] fast-forward: canceling current trajectory");
      auto req = std::make_shared<CancelCurrentTrajectory::Request>();
      cancel_client_->async_send_request(req);
    }
  }

  // --- abort 路径 ---

  void triggerTaskAbort() {
    task_abort_requested_ = true;
    // 同时砍掉跑中轨迹
    if (cancel_client_->service_is_ready()) {
      auto req = std::make_shared<CancelCurrentTrajectory::Request>();
      cancel_client_->async_send_request(req);
    }
    task_cv_.notify_all();
  }

  // set abort flag, 等 binding 线程退出后清 flag.
  // 用于无 follow-up reset 路径的 abort 入口 (ABORT_TASK, ESTOP), 防止 flag 残留污染下一个 binding.
  void triggerTaskAbortAndWait() {
    triggerTaskAbort();
    {
      std::lock_guard<std::mutex> lk(active_mu_);
      if (!active_task_) {
        task_abort_requested_ = false;
        return;
      }
    }
    bool cleared = false;
    for (int i = 0; i < 100; ++i) {
      std::this_thread::sleep_for(50ms);
      std::lock_guard<std::mutex> lk(active_mu_);
      if (!active_task_) {
        cleared = true;
        break;
      }
    }
    if (!cleared) {
      // 5s 自旋超时: binding 线程仍未退出 (可能卡在 LoadTrajectory action). 强清 flag 但告警,
      // 否则真机上静默清 flag 会让人误以为 abort 干净完成, 排查无从下手.
      RCLCPP_ERROR(get_logger(),
                   "[Abort] active_task_ 5s 内未清空, 强制清 task_abort_requested_; "
                   "binding 线程可能卡在 action, 下一个 binding 行为不可保证");
    }
    task_abort_requested_ = false;
  }

  void emergencyStop() {
    triggerTaskAbortAndWait();
    executing_ = false;
    last_sequence_completed_ = false;
    publishControlMode(ControlMode::RELAX);
    RCLCPP_WARN(get_logger(), "[ESTOP] Forced RELAX, abort signaled.");
  }

  void resetToIdle() {
    if (control_mode_ != ControlMode::ARMED) {
      RCLCPP_WARN(get_logger(), "[RESET_HOME] Not ARMED, ignoring.");
      return;
    }

    triggerTaskAbort();

    // 让正在跑的 binding 退出
    for (int i = 0; i < 100; ++i) {
      std::this_thread::sleep_for(50ms);
      std::lock_guard<std::mutex> lk(active_mu_);
      if (!active_task_) break;
    }
    task_abort_requested_ = false;

    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "[RESET_HOME] Busy, ignoring.");
      return;
    }

    std::thread([this]() {
      auto req = std::make_shared<PlanToPreset::Request>();
      req->preset_name = "Escape";
      req->planning_timeout = 5.0;
      req->speed_factor = 1.0;
      auto fut = plan_preset_client_->async_send_request(req);
      if (fut.wait_for(std::chrono::seconds(30)) == std::future_status::ready) {
        auto res = fut.get();
        if (res->success) {
          RCLCPP_INFO(get_logger(), "[RESET_HOME] Escaped (%.1fs)", res->duration);
        } else {
          RCLCPP_ERROR(get_logger(), "[RESET_HOME] Failed: %s", res->message.c_str());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "[RESET_HOME] Service timeout");
      }
      executing_ = false;
      last_sequence_completed_ = false;  // 回零后是 READY, 不是 HOLDING
    }).detach();
  }

  // --- Gripper ---

  void sendGripperAsync(double force) {
    auto req = std::make_shared<GripperControl::Request>();
    req->force = force;
    gripper_client_->async_send_request(
        req, [this, force](rclcpp::Client<GripperControl>::SharedFuture fut) {
          try {
            auto res = fut.get();
            if (res->success)
              RCLCPP_INFO(get_logger(), "Gripper: %.0f N", force);
            else
              RCLCPP_ERROR(get_logger(), "Gripper fail: %s", res->message.c_str());
          } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Gripper error: %s", e.what());
          }
        });
  }

  bool sendGripperSync(double force) {
    auto req = std::make_shared<GripperControl::Request>();
    req->force = force;
    auto fut = gripper_client_->async_send_request(req);
    if (fut.wait_for(5s) == std::future_status::ready) {
      auto res = fut.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "Gripper: %.0f N", force);
        return true;
      }
      RCLCPP_ERROR(get_logger(), "Gripper fail: %s", res->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Gripper sync timeout");
    }
    return false;
  }

  static std::optional<double> parseGripperAction(const std::string& action) {
    if (action == "open") return -70.0;
    if (action == "close") return 70.0;
    if (action == "stop") return 0.0;
    return std::nullopt;
  }

  // 注: wall-clock 调度. 当前轨迹末点 v=0 稳定段触发, 误差不致命.
  // 联机如观察到 gripper 触发时 actual q 与目标差大, 改用 action feedback 触发.
  void scheduleGripperActions(const std::vector<double>& times,
                              const std::vector<std::string>& commands) {
    if (times.empty()) return;
    if (times.size() != commands.size()) {
      RCLCPP_ERROR(get_logger(), "gripper_action arrays size mismatch");
      return;
    }
    auto start = std::chrono::steady_clock::now();
    for (size_t i = 0; i < times.size(); i++) {
      auto target = start + std::chrono::duration<double>(times[i]);
      std::this_thread::sleep_until(target);
      auto force = parseGripperAction(commands[i]);
      if (force) sendGripperSync(*force);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutorNode>();
  // MultiThreadedExecutor + Reentrant client_cb_group_ 让 service 客户端 wait_for
  // 在 detach 线程里阻塞时, action 响应能在另一线程跑, 不死锁.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
