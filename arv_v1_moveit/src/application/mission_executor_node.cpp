/// @file mission_executor_node.cpp
/// @brief Headless mission state machine — MCU command dispatch + trajectory execution.

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

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
using namespace std::chrono_literals;

struct MissionState {
  std::string id;
  std::string trajectory;
  std::string description;
};

class MissionExecutorNode : public rclcpp::Node {
public:
  MissionExecutorNode() : Node("mission_executor") {
    trajectory_timeout_s_ = declare_parameter<double>("trajectory_execution_timeout", 120.0);

    const char* home_env = getenv("HOME");
    if (!home_env) {
      RCLCPP_FATAL(get_logger(), "HOME environment variable not set");
      throw std::runtime_error("HOME env not set");
    }
    std::string home(home_env);
    mission_yaml_path_ = home + "/ros2_ws/src/arv_v1_moveit/config/mission_sequence.yaml";

    load_client_ = create_client<LoadTrajectory>("/load_trajectory");
    gripper_client_ = create_client<GripperControl>("/gripper_control");
    plan_preset_client_ = create_client<PlanToPreset>("/plan_to_preset");

    task_command_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/task_command", 10,
        std::bind(&MissionExecutorNode::onTaskCommand, this, std::placeholders::_1));

    control_mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "/control_mode", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { control_mode_ = msg->data; });

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

    loadMissionSequence();
    publishControlMode(ControlMode::RELAX);
    RCLCPP_INFO(get_logger(), "Mission Executor ready (headless)");
  }

private:
  // --- State ---
  std::vector<MissionState> states_;
  std::atomic<size_t> current_idx_{0};
  std::string mission_name_;
  std::string reset_trajectory_;
  std::string mission_yaml_path_;
  std::atomic<bool> executing_{false};
  double trajectory_timeout_s_ = 120.0;
  uint8_t control_mode_ = ControlMode::RELAX;
  std::mutex status_mu_;
  uint8_t last_task_seq_ = 0xFF;

  // --- ROS2 ---
  rclcpp::Client<LoadTrajectory>::SharedPtr load_client_;
  rclcpp::Client<GripperControl>::SharedPtr gripper_client_;
  rclcpp::Client<PlanToPreset>::SharedPtr plan_preset_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr task_command_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr arm_state_pub_;
  rclcpp::TimerBase::SharedPtr arm_status_timer_;
  rclcpp::TimerBase::SharedPtr mode_broadcast_timer_;

  uint8_t deriveArmState() const {
    if (control_mode_ == ControlMode::RELAX) return 0x05;
    if (control_mode_ == ControlMode::FREEDRIVE) return 0x06;
    if (executing_) return 0x01;
    if (current_idx_ > 0) return 0x02;
    return 0x00;
  }

  void publishControlMode(uint8_t mode) {
    control_mode_ = mode;
    auto msg = std_msgs::msg::UInt8();
    msg.data = mode;
    control_mode_pub_->publish(msg);
    static const char* names[] = {"RELAX", "FREEDRIVE", "ARMED"};
    RCLCPP_INFO(get_logger(), "[MODE] -> %s", mode <= 2 ? names[mode] : "?");
  }

  // --- Mission YAML ---

  void loadMissionSequence() {
    std::lock_guard<std::mutex> lk(status_mu_);
    states_.clear();
    if (!std::filesystem::exists(mission_yaml_path_)) {
      RCLCPP_WARN(get_logger(), "Mission YAML not found, using IDLE.");
      states_.push_back({"IDLE", "", "等待指令"});
      return;
    }
    try {
      auto root = YAML::LoadFile(mission_yaml_path_);
      auto mission = root["mission"];
      mission_name_ = mission["name"].as<std::string>("unnamed");
      reset_trajectory_ = mission["reset_trajectory"].as<std::string>("");
      for (const auto& s : mission["states"]) {
        MissionState ms;
        ms.id = s["id"].as<std::string>("?");
        ms.trajectory = s["trajectory"].as<std::string>("");
        ms.description = s["description"].as<std::string>("");
        states_.push_back(ms);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Parse mission YAML failed: %s", e.what());
      states_.push_back({"IDLE", "", "等待指令(配置错误)"});
    }
    current_idx_ = 0;
  }

  // --- MCU Command Dispatch ---

  void onTaskCommand(const std_msgs::msg::Int32::SharedPtr msg) {
    const uint8_t cmd = static_cast<uint8_t>((msg->data >> 16) & 0xFF);
    const uint8_t param = static_cast<uint8_t>((msg->data >> 8) & 0xFF);
    const uint8_t seq = static_cast<uint8_t>(msg->data & 0xFF);

    if (seq == last_task_seq_) return;
    last_task_seq_ = seq;

    switch (cmd) {
      case 0x01:
        RCLCPP_WARN(get_logger(), "[HW] EMERGENCY_STOP");
        emergencyStop();
        break;
      case 0x02:
        RCLCPP_INFO(get_logger(), "[HW] RESET_HOME");
        resetToIdle();
        break;
      case 0x10:
        RCLCPP_INFO(get_logger(), "[HW] PICK_OBJ obj_id=%u", param);
        executeTrajectoryByKey("pick_obj_" + std::to_string(param));
        break;
      case 0x11:
        RCLCPP_INFO(get_logger(), "[HW] STOW_OBJ slot_id=%u", param);
        executeTrajectoryByKey("stow_slot_" + std::to_string(param));
        break;
      case 0x20:
        RCLCPP_INFO(get_logger(), "[HW] MOVE_TO_DEPOSIT");
        executeTrajectoryByKey("move_to_deposit");
        break;
      case 0x21:
        RCLCPP_INFO(get_logger(), "[HW] EXECUTE_DEPOSIT slot_id=%u", param);
        executeTrajectoryByKey("deposit_slot_" + std::to_string(param));
        break;
      case 0x30:
        RCLCPP_INFO(get_logger(), "[HW] NEXT_STEP");
        {
          std::lock_guard<std::mutex> lk(status_mu_);
          if (current_idx_ + 1 < states_.size()) {
            current_idx_++;
            RCLCPP_INFO(get_logger(), "State -> %s", states_[current_idx_].id.c_str());
          }
        }
        break;
      case 0x31:
        RCLCPP_WARN(get_logger(), "[HW] ABORT_TASK");
        executing_ = false;
        break;
      case 0x40:
        RCLCPP_INFO(get_logger(), "[HW] GRIPPER_CMD param=%u", param);
        sendGripperAsync(param == 1 ? 5.0 : (param == 0 ? -5.0 : 0.0));
        break;
      case 0x50:
        RCLCPP_INFO(get_logger(), "[HW] SET_CONTROL_MODE param=%u", param);
        if (param <= ControlMode::ARMED) publishControlMode(param);
        break;
      default:
        RCLCPP_WARN(get_logger(), "[HW] Unknown TaskCmd 0x%02X param=%u", cmd, param);
        break;
    }
  }

  // --- Trajectory Execution ---

  void executeTrajectoryByKey(const std::string& name) {
    if (control_mode_ != ControlMode::ARMED) {
      RCLCPP_WARN(get_logger(), "须先切到 ARMED 模式才能执行: %s", name.c_str());
      return;
    }
    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "Busy, ignoring: %s", name.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Exec traj: %s", name.c_str());

    std::thread([this, name]() {
      try {
        auto req = std::make_shared<LoadTrajectory::Request>();
        req->name = name;
        req->execute = true;
        auto fut = load_client_->async_send_request(req);
        if (fut.wait_for(std::chrono::duration<double>(trajectory_timeout_s_)) ==
            std::future_status::ready) {
          auto res = fut.get();
          if (res->success) {
            scheduleGripperActions(res->gripper_action_times, res->gripper_action_commands);
            RCLCPP_INFO(get_logger(), "Done: %s", name.c_str());
          } else {
            RCLCPP_ERROR(get_logger(), "Fail: %s — %s", name.c_str(), res->message.c_str());
          }
        } else {
          RCLCPP_ERROR(get_logger(), "Timeout: %s", name.c_str());
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
      }
      executing_ = false;
    }).detach();
  }

  void emergencyStop() {
    executing_ = false;
    publishControlMode(ControlMode::RELAX);
    RCLCPP_WARN(get_logger(), "[ESTOP] Forced RELAX, executing_ cleared.");
  }

  void resetToIdle() {
    if (control_mode_ != ControlMode::ARMED) {
      RCLCPP_WARN(get_logger(), "[RESET_HOME] Not ARMED, ignoring.");
      return;
    }
    bool expected = false;
    if (!executing_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(get_logger(), "[RESET_HOME] Busy, ignoring.");
      return;
    }

    std::thread([this]() {
      auto req = std::make_shared<PlanToPreset::Request>();
      req->preset_name = "Escape";
      req->planning_timeout = 5.0;
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
      current_idx_ = 0;
      executing_ = false;
    }).detach();
  }

  // --- Gripper ---

  void sendGripperAsync(double torque) {
    auto req = std::make_shared<GripperControl::Request>();
    req->torque = torque;
    gripper_client_->async_send_request(
        req, [this, torque](rclcpp::Client<GripperControl>::SharedFuture fut) {
          try {
            auto res = fut.get();
            if (res->success)
              RCLCPP_INFO(get_logger(), "Gripper: %.0f N", torque);
            else
              RCLCPP_ERROR(get_logger(), "Gripper fail: %s", res->message.c_str());
          } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Gripper error: %s", e.what());
          }
        });
  }

  bool sendGripperSync(double torque) {
    auto req = std::make_shared<GripperControl::Request>();
    req->torque = torque;
    auto fut = gripper_client_->async_send_request(req);
    if (fut.wait_for(5s) == std::future_status::ready) {
      auto res = fut.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "Gripper: %.0f N", torque);
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
      auto torque = parseGripperAction(commands[i]);
      if (torque) sendGripperSync(*torque);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionExecutorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
