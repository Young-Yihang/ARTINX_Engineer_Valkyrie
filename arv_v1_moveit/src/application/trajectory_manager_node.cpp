/// @file trajectory_manager_node.cpp
/// @brief Trajectory CRUD: save, load, list, execute via action client.

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <std_msgs/msg/u_int16.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arv_v1_interfaces/srv/cancel_current_trajectory.hpp"
#include "arv_v1_interfaces/srv/compose_trajectory.hpp"
#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "arv_v1_interfaces/srv/list_trajectories.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/plan_to_preset.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"
#include "arv_v1_interfaces/srv/save_trajectory.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using GripperControl = arv_v1_interfaces::srv::GripperControl;

class TrajectoryManagerNode : public rclcpp::Node {
public:
  TrajectoryManagerNode() : Node("trajectory_manager_node") {
    this->declare_parameter<std::string>("trajectory_dir", "");
    std::string traj_dir = this->get_parameter("trajectory_dir").as_string();
    if (traj_dir.empty()) {
      try {
        traj_dir =
            ament_index_cpp::get_package_share_directory("arv_v1_moveit") + "/config/trajectories";
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "[Bootstrap] get_package_share_directory failed (%s), fallback /tmp", e.what());
        traj_dir = "/tmp/trajectories";
      }
    }
    trajectory_dir_ = traj_dir;

    if (!std::filesystem::exists(trajectory_dir_)) {
      std::filesystem::create_directories(trajectory_dir_);
      RCLCPP_INFO(this->get_logger(), "Created trajectory directory: %s", trajectory_dir_.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "Trajectory directory: %s", trajectory_dir_.c_str());

    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    // Callback groups: service 串行 (MutuallyExclusive), action client 并发 (Reentrant)
    // 让 service callback 同步 wait_for action result 时, action 响应回调能在另一线程跑, 不死锁.
    service_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&TrajectoryManagerNode::jointStateCallback, this, std::placeholders::_1));

    trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/ARM_controller/joint_trajectory", 10,
        std::bind(&TrajectoryManagerNode::trajectoryCallback, this, std::placeholders::_1));

    rcl_action_client_options_t action_opts = rcl_action_client_get_default_options();
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/ARM_controller/follow_joint_trajectory", action_cb_group_, action_opts);

    rclcpp::QoS svc_qos = rclcpp::ServicesQoS();

    save_srv_ = this->create_service<arv_v1_interfaces::srv::SaveTrajectory>(
        "/save_trajectory",
        std::bind(&TrajectoryManagerNode::saveTrajectoryCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    save_last_srv_ = this->create_service<arv_v1_interfaces::srv::SaveLastTrajectory>(
        "/save_last_trajectory",
        std::bind(&TrajectoryManagerNode::saveLastTrajectoryCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    load_srv_ = this->create_service<arv_v1_interfaces::srv::LoadTrajectory>(
        "/load_trajectory",
        std::bind(&TrajectoryManagerNode::loadTrajectoryCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    list_srv_ = this->create_service<arv_v1_interfaces::srv::ListTrajectories>(
        "/list_trajectories",
        std::bind(&TrajectoryManagerNode::listTrajectoriesCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    plan_preset_srv_ = this->create_service<arv_v1_interfaces::srv::PlanToPreset>(
        "/plan_to_preset",
        std::bind(&TrajectoryManagerNode::planToPresetCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    compose_srv_ = this->create_service<arv_v1_interfaces::srv::ComposeTrajectory>(
        "/compose_trajectory",
        std::bind(&TrajectoryManagerNode::composeTrajectoryCallback, this, std::placeholders::_1,
                  std::placeholders::_2),
        svc_qos, service_cb_group_);

    cancel_srv_ = this->create_service<arv_v1_interfaces::srv::CancelCurrentTrajectory>(
        "/cancel_current_trajectory",
        std::bind(&TrajectoryManagerNode::cancelCurrentTrajectoryCallback, this,
                  std::placeholders::_1, std::placeholders::_2),
        svc_qos, service_cb_group_);

    // gripper_client_ 走 Reentrant action_cb_group_, 让 side-thread 的 async_send_request
    // future.wait_for 在 service_cb 同步阻塞期间能拿到响应.
    gripper_client_ = this->create_client<GripperControl>("/gripper_control", rclcpp::ServicesQoS(),
                                                          action_cb_group_);

    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), "     Trajectory Manager Node Started");
    RCLCPP_INFO(this->get_logger(), "==============================================");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Services:");
    RCLCPP_INFO(this->get_logger(), "  /save_last_trajectory - Save last executed trajectory");
    RCLCPP_INFO(this->get_logger(), "  /save_trajectory      - Save specified trajectory");
    RCLCPP_INFO(this->get_logger(), "  /load_trajectory      - Load and execute trajectory");
    RCLCPP_INFO(this->get_logger(), "  /list_trajectories    - List saved trajectories");
    RCLCPP_INFO(this->get_logger(),
                "  /plan_to_preset       - Plan & execute to SRDF named target");
    RCLCPP_INFO(this->get_logger(),
                "  /compose_trajectory   - Plan & save multi-waypoint trajectory");
    RCLCPP_INFO(this->get_logger(),
                "  /cancel_current_trajectory - Cancel active FJT goal (B fast-forward)");
    RCLCPP_INFO(this->get_logger(), " ");
    RCLCPP_INFO(this->get_logger(), "Usage:");
    RCLCPP_INFO(this->get_logger(), "  1. Plan and execute in RViz");
    RCLCPP_INFO(this->get_logger(), "  2. Save: ros2 service call /save_last_trajectory \\");
    RCLCPP_INFO(this->get_logger(), "           arv_v1_interfaces/srv/SaveLastTrajectory \\");
    RCLCPP_INFO(this->get_logger(), "           \"{name: 'my_traj', description: 'desc'}\"");
    RCLCPP_INFO(this->get_logger(), " ");
  }

private:
  std::string trajectory_dir_;
  std::vector<std::string> joint_names_;
  std::array<double, 6> current_position_{};
  bool joint_state_received_ = false;
  std::mutex state_mutex_;

  trajectory_msgs::msg::JointTrajectory last_trajectory_;
  std::mutex trajectory_mutex_;
  bool has_last_trajectory_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

  rclcpp::Service<arv_v1_interfaces::srv::SaveTrajectory>::SharedPtr save_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::SaveLastTrajectory>::SharedPtr save_last_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::LoadTrajectory>::SharedPtr load_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::ListTrajectories>::SharedPtr list_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::PlanToPreset>::SharedPtr plan_preset_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::ComposeTrajectory>::SharedPtr compose_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::CancelCurrentTrajectory>::SharedPtr cancel_srv_;

  // 夹爪并入轨迹管理职责: action 起飞那刻起 wall-clock side-thread 触发 gripper_actions,
  // 跟 action result 等待并行. 避免同步等 LoadTrajectory 完成后再调度导致的时序错位.
  rclcpp::Client<GripperControl>::SharedPtr gripper_client_;

  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;

  // 当前 active FJT goal handle, /cancel_current_trajectory 用其调 async_cancel_goal.
  // 由 loadTrajectoryCallback 的 send_goal 线程 set/clear, cancel callback 读
  GoalHandleFJT::SharedPtr current_goal_handle_;
  std::mutex goal_handle_mu_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool move_group_ready_ = false;

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t j = 0; j < joint_names_.size(); ++j) {
        if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
          current_position_[j] = msg->position[i];
        }
      }
    }

    if (!joint_state_received_) {
      joint_state_received_ = true;
      RCLCPP_INFO(this->get_logger(), "First joint state received");
    }
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    last_trajectory_ = *msg;
    has_last_trajectory_ = true;

    double duration = 0.0;
    if (!msg->points.empty()) {
      const auto& last_point = msg->points.back();
      duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    }

    RCLCPP_INFO(this->get_logger(), "[Captured] Trajectory with %zu points, duration: %.2fs",
                msg->points.size(), duration);
  }

  bool saveTrajectoryToFile(const std::string& name, const std::string& description,
                            const trajectory_msgs::msg::JointTrajectory& trajectory,
                            const std::vector<double>& gripper_times,
                            const std::vector<std::string>& gripper_cmds, std::string& saved_path,
                            std::string& error_message,
                            const std::vector<std::vector<double>>& waypoints_meta = {}) {
    std::string filename = trajectory_dir_ + "/" + name + ".yaml";

    try {
      YAML::Emitter out;
      out << YAML::BeginMap;

      out << YAML::Key << "meta" << YAML::Value << YAML::BeginMap;
      out << YAML::Key << "name" << YAML::Value << name;
      out << YAML::Key << "description" << YAML::Value << description;

      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
      out << YAML::Key << "saved_at" << YAML::Value << ss.str();

      double duration = 0.0;
      if (!trajectory.points.empty()) {
        const auto& last_point = trajectory.points.back();
        duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
      }
      out << YAML::Key << "duration_sec" << YAML::Value << duration;

      // gripper_actions: 跟 LoadTrajectory 期望的格式对齐 (meta.gripper_actions)
      if (!gripper_times.empty() && gripper_times.size() == gripper_cmds.size()) {
        out << YAML::Key << "gripper_actions" << YAML::Value << YAML::BeginSeq;
        for (size_t i = 0; i < gripper_times.size(); ++i) {
          out << YAML::BeginMap;
          out << YAML::Key << "time" << YAML::Value << gripper_times[i];
          out << YAML::Key << "action" << YAML::Value << gripper_cmds[i];
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;
      }

      // waypoints 溯源: ComposeTrajectory 写入原始关节角, 便于将来重规划
      if (!waypoints_meta.empty()) {
        out << YAML::Key << "waypoints" << YAML::Value << YAML::BeginSeq;
        for (const auto& wp : waypoints_meta) {
          out << YAML::Flow << YAML::BeginSeq;
          for (const auto& v : wp) out << v;
          out << YAML::EndSeq;
        }
        out << YAML::EndSeq;
      }
      out << YAML::EndMap;  // meta

      out << YAML::Key << "joint_names" << YAML::Value << YAML::BeginSeq;
      if (!trajectory.joint_names.empty()) {
        for (const auto& jname : trajectory.joint_names) {
          out << jname;
        }
      } else {
        for (const auto& jname : joint_names_) {
          out << jname;
        }
      }
      out << YAML::EndSeq;

      if (!trajectory.points.empty()) {
        out << YAML::Key << "start_position" << YAML::Value << YAML::Flow;
        out << YAML::BeginSeq;
        for (const auto& pos : trajectory.points[0].positions) {
          out << pos;
        }
        out << YAML::EndSeq;
      }

      out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
      for (const auto& point : trajectory.points) {
        out << YAML::BeginMap;

        double time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        out << YAML::Key << "time" << YAML::Value << time;

        out << YAML::Key << "positions" << YAML::Value << YAML::Flow;
        out << YAML::BeginSeq;
        for (const auto& pos : point.positions) {
          out << pos;
        }
        out << YAML::EndSeq;

        if (!point.velocities.empty()) {
          out << YAML::Key << "velocities" << YAML::Value << YAML::Flow;
          out << YAML::BeginSeq;
          for (const auto& vel : point.velocities) {
            out << vel;
          }
          out << YAML::EndSeq;
        }

        if (!point.accelerations.empty()) {
          out << YAML::Key << "accelerations" << YAML::Value << YAML::Flow;
          out << YAML::BeginSeq;
          for (const auto& acc : point.accelerations) {
            out << acc;
          }
          out << YAML::EndSeq;
        }

        out << YAML::EndMap;
      }
      out << YAML::EndSeq;  // points

      out << YAML::EndMap;  // root

      std::ofstream fout(filename);
      if (!fout.is_open()) {
        error_message = "Failed to open file for writing: " + filename;
        return false;
      }
      fout << out.c_str();
      fout.close();

      saved_path = filename;

      RCLCPP_INFO(this->get_logger(), "[Save] Saved to: %s (%zu points, %.2fs)", filename.c_str(),
                  trajectory.points.size(), duration);

      return true;

    } catch (const std::exception& e) {
      error_message = std::string("Exception: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "[Save] Error: %s", e.what());
      return false;
    }
  }

  void saveTrajectoryCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::SaveTrajectory::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::SaveTrajectory::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[SaveTrajectory] Saving: %s", request->name.c_str());

    if (request->name.empty()) {
      response->success = false;
      response->message = "Trajectory name cannot be empty";
      return;
    }
    if (request->trajectory.points.empty()) {
      response->success = false;
      response->message = "Trajectory has no points";
      return;
    }

    std::string saved_path, error_message;
    std::vector<std::string> g_cmds(request->gripper_action_commands.begin(),
                                    request->gripper_action_commands.end());
    if (saveTrajectoryToFile(request->name, request->description, request->trajectory,
                             request->gripper_action_times, g_cmds, saved_path, error_message)) {
      response->success = true;
      response->message = "Trajectory saved successfully";
      response->saved_path = saved_path;
    } else {
      response->success = false;
      response->message = error_message;
    }
  }

  void saveLastTrajectoryCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::SaveLastTrajectory::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::SaveLastTrajectory::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[SaveLastTrajectory] Saving: %s", request->name.c_str());

    if (request->name.empty()) {
      response->success = false;
      response->message = "Trajectory name cannot be empty";
      return;
    }

    trajectory_msgs::msg::JointTrajectory trajectory_to_save;
    {
      std::lock_guard<std::mutex> lock(trajectory_mutex_);
      if (!has_last_trajectory_ || last_trajectory_.points.empty()) {
        response->success = false;
        response->message = "No trajectory captured. Please execute a trajectory in RViz first.";
        return;
      }
      trajectory_to_save = last_trajectory_;
    }

    std::string saved_path, error_message;
    std::vector<double> empty_times;
    std::vector<std::string> empty_cmds;
    if (saveTrajectoryToFile(request->name, request->description, trajectory_to_save, empty_times,
                             empty_cmds, saved_path, error_message)) {
      response->success = true;
      response->message = "Last trajectory saved successfully";
      response->saved_path = saved_path;
    } else {
      response->success = false;
      response->message = error_message;
    }
  }

  // 旁线程: action 起跑那刻锚 wall-clock, 按 gripper_action_times 顺序触发夹爪.
  // detach 跑完即退, 不阻塞 service callback. 跟 action result 等待并行.
  // 注: 若 action 中途被 cancel, 此线程仍按原计划触发剩余动作 (TODO: 加 abort flag).
  void fireGripperSchedule(std::vector<double> times, std::vector<std::string> commands,
                           std::chrono::steady_clock::time_point start) {
    for (size_t i = 0; i < times.size(); ++i) {
      auto target = start + std::chrono::duration<double>(times[i]);
      std::this_thread::sleep_until(target);
      double force;
      if (commands[i] == "open")
        force = -70.0;
      else if (commands[i] == "close")
        force = 70.0;
      else if (commands[i] == "stop")
        force = 0.0;
      else
        continue;

      auto req = std::make_shared<GripperControl::Request>();
      req->force = force;
      auto fut = gripper_client_->async_send_request(req);
      if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
        auto res = fut.get();
        RCLCPP_INFO(this->get_logger(), "[Gripper] t=%.2fs %s (%.0f N)", times[i],
                    commands[i].c_str(), force);
        (void)res;
      } else {
        RCLCPP_ERROR(this->get_logger(), "[Gripper] timeout at t=%.2fs", times[i]);
      }
    }
  }

  void loadTrajectoryCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::LoadTrajectory::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::LoadTrajectory::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Loading: %s, execute: %s",
                request->name.c_str(), request->execute ? "true" : "false");

    std::string filename = trajectory_dir_ + "/" + request->name + ".yaml";

    if (!std::filesystem::exists(filename)) {
      response->success = false;
      response->message = "Trajectory file not found: " + filename;
      return;
    }

    try {
      YAML::Node config = YAML::LoadFile(filename);

      trajectory_msgs::msg::JointTrajectory trajectory;

      if (config["joint_names"]) {
        for (const auto& name : config["joint_names"]) {
          trajectory.joint_names.push_back(name.as<std::string>());
        }
      } else {
        trajectory.joint_names = joint_names_;
      }

      double duration = 0.0;
      if (config["points"]) {
        for (const auto& pt : config["points"]) {
          trajectory_msgs::msg::JointTrajectoryPoint point;

          double time = pt["time"].as<double>();
          point.time_from_start = rclcpp::Duration::from_seconds(time);
          duration = std::max(duration, time);

          for (const auto& pos : pt["positions"]) {
            point.positions.push_back(pos.as<double>());
          }

          if (pt["velocities"]) {
            for (const auto& vel : pt["velocities"]) {
              point.velocities.push_back(vel.as<double>());
            }
          }

          if (pt["accelerations"]) {
            for (const auto& acc : pt["accelerations"]) {
              point.accelerations.push_back(acc.as<double>());
            }
          }

          trajectory.points.push_back(point);
        }
      }

      response->duration = duration;

      // 解析夹爪动作序列 (可选字段, 向后兼容)
      if (config["meta"] && config["meta"]["gripper_actions"]) {
        double prev_time = -1.0;
        for (const auto& ga : config["meta"]["gripper_actions"]) {
          if (!ga["time"] || !ga["action"]) {
            RCLCPP_WARN(this->get_logger(),
                        "[LoadTrajectory] Skipping gripper action with missing time/action field");
            continue;
          }
          double t = ga["time"].as<double>();
          std::string cmd = ga["action"].as<std::string>();
          if (t < 0.0) {
            RCLCPP_WARN(this->get_logger(), "[LoadTrajectory] Skipping negative time: %.3f", t);
            continue;
          }
          if (t < prev_time) {
            // 非单调会导致 sleep_until 立即触发, 跳过而不仅是 warn
            RCLCPP_WARN(this->get_logger(),
                        "[LoadTrajectory] Skipping non-monotonic time: %.3f < %.3f", t, prev_time);
            continue;
          }
          if (cmd != "open" && cmd != "close" && cmd != "stop") {
            RCLCPP_WARN(this->get_logger(), "[LoadTrajectory] Unknown gripper action: '%s'",
                        cmd.c_str());
            continue;
          }
          response->gripper_action_times.push_back(t);
          response->gripper_action_commands.push_back(cmd);
          prev_time = t;
        }
        if (!response->gripper_action_times.empty()) {
          RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Found %zu gripper actions",
                      response->gripper_action_times.size());
        }
      }

      RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Loaded %zu points, duration: %.2fs",
                  trajectory.points.size(), duration);

      if (request->execute) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
          response->success = false;
          response->message = "Action server not available";
          return;
        }

        auto goal = FollowJointTrajectory::Goal();
        goal.trajectory = trajectory;

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        auto goal_fut = action_client_->async_send_goal(goal, send_goal_options);

        // 等 goal_handle (action server accept)
        if (goal_fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
          response->success = false;
          response->message = "Goal send timeout";
          return;
        }
        auto goal_handle = goal_fut.get();
        if (!goal_handle) {
          response->success = false;
          response->message = "Goal rejected by action server";
          return;
        }

        // 记入 current_goal_handle_, 让 /cancel_current_trajectory 能砍
        {
          std::lock_guard<std::mutex> lk(goal_handle_mu_);
          current_goal_handle_ = goal_handle;
        }

        // ━━━ Fork gripper schedule 旁线程 (action 起跑那刻锚 wall-clock) ━━━
        // 跟下面 async_get_result 等待并行, 在轨迹跑的过程中按时触发夹爪.
        if (!response->gripper_action_times.empty() &&
            response->gripper_action_times.size() == response->gripper_action_commands.size()) {
          auto schedule_start = std::chrono::steady_clock::now();
          std::vector<double> g_times(response->gripper_action_times.begin(),
                                      response->gripper_action_times.end());
          std::vector<std::string> g_cmds(response->gripper_action_commands.begin(),
                                          response->gripper_action_commands.end());
          std::thread(&TrajectoryManagerNode::fireGripperSchedule, this, std::move(g_times),
                      std::move(g_cmds), schedule_start)
              .detach();
        }

        // 同步等 action result (action_cb_group_ Reentrant 保证 result 回调在另一线程跑, 不死锁)
        auto result_fut = action_client_->async_get_result(goal_handle);
        auto wait_dur = std::chrono::duration<double>(duration + 5.0);
        auto result_status = result_fut.wait_for(wait_dur);

        // 清 current_goal_handle_ (无论结果)
        {
          std::lock_guard<std::mutex> lk(goal_handle_mu_);
          current_goal_handle_.reset();
        }

        if (result_status != std::future_status::ready) {
          response->success = false;
          response->message = "Execution timeout";
          return;
        }

        auto wrapped = result_fut.get();
        switch (wrapped.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            response->success = true;
            response->message = "";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            // 区分: B fast-forward 砍的轨迹也算 "成功" 但消息标 canceled
            response->success = true;
            response->message = "canceled";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            response->success = false;
            response->message = "aborted by controller";
            break;
          default:
            response->success = false;
            response->message = "unknown result code";
            break;
        }
      } else {
        response->success = true;
        response->message = "Trajectory loaded (not executed)";
      }

    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Exception: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "[LoadTrajectory] Error: %s", e.what());
    }
  }

  bool ensureMoveGroup() {
    if (move_group_ready_) return true;
    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "ARM");
      move_group_ready_ = true;
      RCLCPP_INFO(this->get_logger(), "[PlanToPreset] MoveGroupInterface initialized (group=ARM)");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[PlanToPreset] MoveGroupInterface init failed: %s",
                   e.what());
    }
    return move_group_ready_;
  }

  void planToPresetCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::PlanToPreset::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::PlanToPreset::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[PlanToPreset] Request: preset='%s', timeout=%.1fs",
                request->preset_name.c_str(), request->planning_timeout);

    if (!ensureMoveGroup()) {
      response->success = false;
      response->message = "MoveGroupInterface not available (move_group node running?)";
      return;
    }

    double timeout = request->planning_timeout > 0.0 ? request->planning_timeout : 5.0;
    double speed =
        (request->speed_factor > 0.0 && request->speed_factor <= 1.0) ? request->speed_factor : 0.3;
    move_group_->setPlanningTime(timeout);
    move_group_->setMaxVelocityScalingFactor(speed);
    move_group_->setMaxAccelerationScalingFactor(speed);
    // Pilz PTP 关节空间解析解, 跟 compose 一致, 避免 OMPL 卡死
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("PTP");

    // 显式设 start state 从自己的 /joint_states sub (current_position_), 绕开 MoveIt
    // current_state_monitor 的 QoS 不匹配坑 — 不然 plan 用空 start state 生成无效轨迹.
    auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      response->success = false;
      response->message = "Robot model not available";
      return;
    }
    moveit::core::RobotState start_state(robot_model);
    start_state.setToDefaultValues();
    std::vector<double> cur(6);
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (!joint_state_received_) {
        response->success = false;
        response->message = "No /joint_states received yet";
        return;
      }
      cur.assign(current_position_.begin(), current_position_.end());
    }
    start_state.setJointGroupPositions("ARM", cur);
    start_state.update();
    move_group_->setStartState(start_state);

    if (!move_group_->setNamedTarget(request->preset_name)) {
      response->success = false;
      response->message = "Unknown preset: " + request->preset_name;
      return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      response->success = false;
      response->message = "Planning failed (code=" + std::to_string(result.val) + ")";
      RCLCPP_WARN(this->get_logger(), "[PlanToPreset] %s", response->message.c_str());
      return;
    }

    auto& traj = plan.trajectory.joint_trajectory;
    double duration = 0.0;
    if (!traj.points.empty()) {
      auto& last = traj.points.back().time_from_start;
      duration = last.sec + last.nanosec * 1e-9;
    }
    RCLCPP_INFO(this->get_logger(), "[PlanToPreset] Planned %zu points, duration=%.2fs",
                traj.points.size(), duration);

    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      response->success = false;
      response->message = "Action server not available";
      return;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory = traj;

    auto future = action_client_->async_send_goal(goal);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      response->success = false;
      response->message = "Goal send timeout";
      return;
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
      response->success = false;
      response->message = "Goal rejected by action server";
      return;
    }

    auto result_future = action_client_->async_get_result(goal_handle);
    auto wait_duration = std::chrono::duration<double>(duration + 5.0);
    if (result_future.wait_for(wait_duration) != std::future_status::ready) {
      response->success = false;
      response->message = "Execution timeout";
      return;
    }

    auto action_result = result_future.get();
    if (action_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      response->success = true;
      response->message = "Executed to preset: " + request->preset_name;
      response->duration = duration;
    } else {
      response->success = false;
      response->message =
          "Execution failed (result_code=" + std::to_string(static_cast<int>(action_result.code)) +
          ")";
    }
  }

  void composeTrajectoryCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::ComposeTrajectory::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::ComposeTrajectory::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[Compose] name='%s', waypoints=%zu, speed=%.2f",
                request->name.c_str(), request->waypoints.size(), request->speed_factor);

    if (request->name.empty()) {
      response->success = false;
      response->message = "Trajectory name cannot be empty";
      return;
    }
    if (request->waypoints.size() < 2) {
      response->success = false;
      response->message = "Need at least 2 waypoints";
      return;
    }
    // 校验每个 waypoint 有 ≥6 个 joint 位置 (前 6 个用作 ARM 关节)
    std::vector<std::vector<double>> wp_joints;
    wp_joints.reserve(request->waypoints.size());
    for (size_t i = 0; i < request->waypoints.size(); ++i) {
      const auto& js = request->waypoints[i];
      if (js.position.size() < 6) {
        response->success = false;
        response->message = "Waypoint " + std::to_string(i) + " has <6 joint positions";
        return;
      }
      wp_joints.emplace_back(js.position.begin(), js.position.begin() + 6);
    }

    if (!ensureMoveGroup()) {
      response->success = false;
      response->message = "MoveGroupInterface not available (move_group node running?)";
      return;
    }

    double speed =
        (request->speed_factor > 0.0 && request->speed_factor <= 1.0) ? request->speed_factor : 0.3;
    move_group_->setMaxVelocityScalingFactor(speed);
    move_group_->setMaxAccelerationScalingFactor(speed);
    move_group_->setPlanningTime(5.0);
    // Pilz PTP: 关节空间点到点, 解析解, 成功率高且毫秒级返回 (避免 OMPL 采样不收敛卡死)
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("PTP");

    trajectory_msgs::msg::JointTrajectory combined;
    combined.joint_names = joint_names_;
    double t_offset = 0.0;

    auto robot_model = move_group_->getRobotModel();
    if (!robot_model) {
      response->success = false;
      response->message = "Robot model not available from MoveGroupInterface";
      return;
    }

    for (size_t seg = 0; seg + 1 < wp_joints.size(); ++seg) {
      // 直接从 robot model 构造 RobotState, 不依赖 current_state_monitor (QoS 不匹配会卡)
      moveit::core::RobotState start_state(robot_model);
      start_state.setToDefaultValues();
      start_state.setJointGroupPositions("ARM", wp_joints[seg]);
      start_state.update();
      move_group_->setStartState(start_state);

      move_group_->setJointValueTarget(wp_joints[seg + 1]);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto code = move_group_->plan(plan);
      if (code != moveit::core::MoveItErrorCode::SUCCESS) {
        response->success = false;
        response->message = "Planning failed at segment " + std::to_string(seg) +
                            " (code=" + std::to_string(code.val) + ")";
        RCLCPP_ERROR(this->get_logger(), "[Compose] %s", response->message.c_str());
        return;
      }

      const auto& seg_traj = plan.trajectory.joint_trajectory;
      // 段间衔接: 第一段全保留, 后续段跳过首点 (避免与上一段末点重复)
      size_t start_idx = (seg == 0) ? 0 : 1;
      for (size_t i = start_idx; i < seg_traj.points.size(); ++i) {
        auto pt = seg_traj.points[i];
        double t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
        pt.time_from_start = rclcpp::Duration::from_seconds(t + t_offset);
        combined.points.push_back(pt);
      }
      if (!seg_traj.points.empty()) {
        const auto& last_t = seg_traj.points.back().time_from_start;
        t_offset += last_t.sec + last_t.nanosec * 1e-9;
      }
    }

    if (combined.points.empty()) {
      response->success = false;
      response->message = "No points produced by planner";
      return;
    }

    std::vector<std::string> g_cmds(request->gripper_action_commands.begin(),
                                    request->gripper_action_commands.end());
    std::vector<double> g_times(request->gripper_action_times.begin(),
                                request->gripper_action_times.end());

    std::string saved_path, err;
    if (saveTrajectoryToFile(request->name, request->description, combined, g_times, g_cmds,
                             saved_path, err, wp_joints)) {
      response->success = true;
      response->message = "Composed and saved (" + std::to_string(wp_joints.size()) +
                          " waypoints, " + std::to_string(combined.points.size()) + " points)";
      response->saved_path = saved_path;
      response->duration = t_offset;
      response->point_count = combined.points.size();
    } else {
      response->success = false;
      response->message = err;
    }
  }

  void cancelCurrentTrajectoryCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::CancelCurrentTrajectory::Request> /*req*/,
      std::shared_ptr<arv_v1_interfaces::srv::CancelCurrentTrajectory::Response> res) {
    GoalHandleFJT::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lk(goal_handle_mu_);
      handle = current_goal_handle_;
    }
    if (!handle) {
      // 幂等: 无 active goal 也 success
      res->success = true;
      res->message = "no active goal";
      return;
    }
    RCLCPP_WARN(this->get_logger(), "[CancelCurrent] Canceling active FJT goal");
    action_client_->async_cancel_goal(handle);
    res->success = true;
    res->message = "cancel requested";
  }

  void listTrajectoriesCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::ListTrajectories::Request> /*request*/,
      std::shared_ptr<arv_v1_interfaces::srv::ListTrajectories::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[ListTrajectories] Scanning: %s", trajectory_dir_.c_str());

    try {
      for (const auto& entry : std::filesystem::directory_iterator(trajectory_dir_)) {
        if (entry.path().extension() == ".yaml") {
          std::string name = entry.path().stem().string();
          std::string description = "";

          try {
            YAML::Node config = YAML::LoadFile(entry.path().string());
            if (config["meta"] && config["meta"]["description"]) {
              description = config["meta"]["description"].as<std::string>();
            }
          } catch (...) {
            RCLCPP_DEBUG(this->get_logger(), "Cannot read trajectory description");
            description = "(unable to read description)";
          }

          response->names.push_back(name);
          response->descriptions.push_back(description);
        }
      }

      RCLCPP_INFO(this->get_logger(), "[ListTrajectories] Found %zu trajectories",
                  response->names.size());

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[ListTrajectories] Error: %s", e.what());
      response->names.clear();
      response->descriptions.clear();
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TrajectoryManagerNode>();
    // MultiThreadedExecutor + Reentrant action_cb_group_ 让 service callback 同步等 action
    // result 时, action 的 goal_response / result / cancel 回调能在其他线程跑, 避免死锁.
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("trajectory_manager"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
