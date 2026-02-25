/**
 * @file torque_controller_node.cpp
 * @brief Torque controller: full dynamics feedforward + cascade PID feedback
 *
 * Control law: τ = M(q)q̈ + C(q,q̇) + G(q) + CascadePID(e_pos, e_vel)
 */

#include <urdf/model.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // 力矩数组消息
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "cascade_pid.hpp"
#include "dynamics_computer.hpp"
#include "kalman_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class TorqueControllerActionServer : public rclcpp::Node  // 节点类生成
{
  static constexpr int kArmJoints = 6;  // 6-DOF arm
  static constexpr int kAllJoints = 7;  // 6-DOF arm + 1 gripper
public:                                 // 构造函数log
  TorqueControllerActionServer()
      : Node("torque_controller_action_server"),
        is_executing_(false),
        q_actual_(6),
        q_dot_actual_(6),
        q_target_(6),
        state_received_(false),
        has_target_(false),
        control_frequency_(200.0),
        joint_filters_{
            KalmanFilter1D(1.0 / 200.0),  // Joint 1
            KalmanFilter1D(1.0 / 200.0),  // Joint 2
            KalmanFilter1D(1.0 / 200.0),  // Joint 3
            KalmanFilter1D(1.0 / 200.0),  // Joint 4
            KalmanFilter1D(1.0 / 200.0),  // Joint 5
            KalmanFilter1D(1.0 / 200.0)   // Joint 6
        },
        q_dot_filtered_(6),
        kalman_filter_enabled_(false)

  {
    RCLCPP_INFO(this->get_logger(),
                "[START] Torque controller node starting (Cascade P+PI Control)");

    // 卡尔曼滤波器参数
    this->declare_parameter("kalman.enabled", false);  // 默认禁用
    kalman_filter_enabled_ = this->get_parameter("kalman.enabled").as_bool();

    RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter state: %s",
                kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");

    // 声明卡尔曼滤波器参数
    this->declare_parameter("kalman.Q_pos", 1e-5);    // 过程噪声：位置
    this->declare_parameter("kalman.Q_vel", 1e-4);    // 过程噪声：速度
    this->declare_parameter("kalman.R_pos", 1e-3);    // 测量噪声：位置
    this->declare_parameter("kalman.R_vel", 2.5e-2);  // 测量噪声：速度

    // 读取参数并设置滤波器
    double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
    double Q_vel = this->get_parameter("kalman.Q_vel").as_double();
    double R_pos = this->get_parameter("kalman.R_pos").as_double();
    double R_vel = this->get_parameter("kalman.R_vel").as_double();

    for (auto &filter : joint_filters_) {
      filter.setProcessNoise(Q_pos, Q_vel);
      filter.setMeasurementNoise(R_pos, R_vel);
    }

    RCLCPP_INFO(this->get_logger(), "[OK] Kalman filter initialized:");
    RCLCPP_INFO(this->get_logger(), "   Q_pos=%.1e, Q_vel=%.1e", Q_pos, Q_vel);
    RCLCPP_INFO(this->get_logger(), "   R_pos=%.1e, R_vel=%.1e", R_pos, R_vel);

    // Kalman print removed for cleaner output

    // --- Load Safety Parameters (BEFORE initializeDynamics) ---
    this->declare_parameter("safety.max_torque_default", 20.0);
    this->declare_parameter("safety.joint_state_timeout_ms", 100);
    this->declare_parameter("safety.max_control_period_ms", 10);
    this->declare_parameter("safety.max_velocity_sanity", 20.0);
    this->declare_parameter("safety.max_position_error", 0.8);

    max_torque_default_ = this->get_parameter("safety.max_torque_default").as_double();
    joint_state_timeout_sec_ =
        this->get_parameter("safety.joint_state_timeout_ms").as_int() / 1000.0;
    max_control_period_sec_ = this->get_parameter("safety.max_control_period_ms").as_int() / 1000.0;
    max_velocity_sanity_ = this->get_parameter("safety.max_velocity_sanity").as_double();
    max_position_error_ = this->get_parameter("safety.max_position_error").as_double();

    // Load per-joint torque limits (prefer config, fallback to URDF, then default)
    max_torque_per_joint_.resize(6, max_torque_default_);  // Initialize with default

    // Then override with config if specified
    for (int i = 1; i <= 6; i++) {
      std::string param_name = "safety.max_torque_per_joint.joint_" + std::to_string(i);
      this->declare_parameter(param_name, -1.0);  // -1 means "use URDF value"
      double config_limit = this->get_parameter(param_name).as_double();
      if (config_limit > 0)  // If explicitly set in config
      {
        max_torque_per_joint_[i - 1] = config_limit;
      }
    }

    // Load gripper force limit from config (prismatic joint, unit: N)
    this->declare_parameter("safety.max_torque_per_joint.joint_gripper1", -1.0);
    double gripper_limit =
        this->get_parameter("safety.max_torque_per_joint.joint_gripper1").as_double();
    if (gripper_limit > 0) {
      gripper_max_torque_ = gripper_limit;
    }

    RCLCPP_INFO(this->get_logger(),
                "[SAFETY] Timeout: %.0f ms, Velocity sanity: %.1f rad/s, Gripper limit: %.1f N",
                joint_state_timeout_sec_ * 1000.0, max_velocity_sanity_, gripper_max_torque_);

    // --- 初始化动力学求解器 ---
    RCLCPP_INFO(this->get_logger(), "[INFO] Starting dynamics solver initialization...");
    if (!initializeDynamics()) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to initialize dynamics solver");
      throw std::runtime_error("Failed to initialize dynamics");
    }

    // --- 级联 P+PI 初始化（完全替代 PD）---
    RCLCPP_INFO(this->get_logger(), "[PID] Initializing Cascade P+PI controller...");

    cascade_pid_ = std::make_unique<MultiJointCascadePid>(6);

    // 级联 P+PI 参数: 外环位置P, 内环速度PI
    for (int i = 1; i <= 6; i++) {
      std::string prefix = "cascade_pid.joint_" + std::to_string(i);

      // 位置环参数（只用 P）
      this->declare_parameter(prefix + ".pos_Kp", 10.0);
      this->declare_parameter(prefix + ".pos_Ki", 0.0);  // 外环不用积分
      this->declare_parameter(prefix + ".pos_Kd", 0.0);  // 外环不用微分

      // 速度环参数（使用 PI）
      this->declare_parameter(prefix + ".vel_Kp", 50.0);
      this->declare_parameter(prefix + ".vel_Ki", 5.0);  // 内环积分消除静差
      this->declare_parameter(prefix + ".vel_Kd", 0.0);  // 内环不用微分

      // 速度限制
      this->declare_parameter(prefix + ".vel_limit", 2.0);
    }

    // 读取参数并设置控制器
    for (int i = 0; i < 6; i++) {
      std::string prefix = "cascade_pid.joint_" + std::to_string(i + 1);

      PidGains pos_gains(this->get_parameter(prefix + ".pos_Kp").as_double(),
                         this->get_parameter(prefix + ".pos_Ki").as_double(),
                         this->get_parameter(prefix + ".pos_Kd").as_double());

      PidGains vel_gains(this->get_parameter(prefix + ".vel_Kp").as_double(),
                         this->get_parameter(prefix + ".vel_Ki").as_double(),
                         this->get_parameter(prefix + ".vel_Kd").as_double());

      double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();

      cascade_pid_->setJointParams(i, pos_gains, vel_gains, vel_limit);
    }

    RCLCPP_INFO(this->get_logger(),
                "[OK] Cascade P+PI initialized (Outer: Position-P, Inner: Velocity-PI)");

    // --- 注册参数变化回调（必须在所有参数声明之后）---
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TorqueControllerActionServer::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[CONFIG] Dynamic parameter tuning enabled (use 'ros2 param set' to modify)");

    // --- 订阅关节状态 ---
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",                                              // 话题名称
        10,                                                           // 队列大小
        std::bind(&TorqueControllerActionServer::jointStateCallback,  // 把成员回调函数绑定
                  this, std::placeholders::_1));

    // --- 创建 Action Server ---
    // MoveIt 会发送到: /ARM_controller/follow_joint_trajectory
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "ARM_controller/follow_joint_trajectory",  // 完整的 action 名称
        std::bind(&TorqueControllerActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&TorqueControllerActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&TorqueControllerActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[OK] Action server created: /ARM_controller/follow_joint_trajectory");

    // --- 创建力矩/力发布者 ---
    // 7-element array: [0-5] arm joint torques (Nm), [6] gripper force (N, prismatic)
    torque_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(),
                "[OK] Effort publisher created: /effort_controller/commands (6×Nm + 1×N)");

    // --- 创建夹爪控制服务 ---
    gripper_service_ = this->create_service<arv_v1_interfaces::srv::GripperControl>(
        "/gripper_control", std::bind(&TorqueControllerActionServer::gripperControlCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "[OK] Gripper control service created: /gripper_control");

    // --- 创建轨迹转发发布者 ---
    // 用于将接收到的轨迹转发给 trajectory_manager_node 进行保存
    trajectory_forward_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/ARM_controller/joint_trajectory", 10);
    RCLCPP_INFO(this->get_logger(),
                "[OK] Trajectory forward publisher created: /ARM_controller/joint_trajectory");

    RCLCPP_INFO(this->get_logger(), "[INFO] Control frequency: %.1f Hz", control_frequency_);

    auto period = std::chrono::duration<double, std::milli>(1000.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        period, std::bind(&TorqueControllerActionServer::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "[INFO] Control loop timer started (%.1f Hz)",
                control_frequency_);

    RCLCPP_INFO(this->get_logger(), "[OK] Torque controller fully initialized");
  }

  ~TorqueControllerActionServer() {
    RCLCPP_INFO(this->get_logger(),
                "[INFO] Dynamics torque calculation destructing, program ending");
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  trajectory_msgs::msg::JointTrajectory current_trajectory_;
  rclcpp::Time trajectory_start_time_;
  std::shared_ptr<GoalHandleFJT> current_goal_handle_;
  std::atomic<bool> is_executing_;  // 原子操作，避免竞态

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  KDL::JntArray q_actual_, q_dot_actual_, q_target_;
  std::mutex state_mutex_;   // 保护状态变量
  std::mutex action_mutex_;  // 保护执行标志和目标句柄
  std::mutex filter_mutex_;  // 保护滤波器
  bool state_received_, has_target_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<DynamicsComputer> dynamic_computer_;
  std::unique_ptr<MultiJointCascadePid> cascade_pid_;

  std::array<KalmanFilter1D, 6> joint_filters_;
  KDL::JntArray q_dot_filtered_;
  bool kalman_filter_enabled_;

  size_t control_loop_count_ = 0;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      trajectory_forward_pub_;  // 转发轨迹供trajectory_manager捕获
  double control_frequency_;

  // 安全参数
  rclcpp::Time last_joint_state_time_, last_control_loop_time_;
  double joint_state_timeout_sec_ = 0.1;  // 100ms
  double max_control_period_sec_ = 0.01;  // 10ms
  std::vector<double> max_torque_per_joint_;
  double max_torque_default_ = 20.0;
  double max_velocity_sanity_ = 20.0;
  double max_position_error_ = 0.8;

  // 夹爪控制 (prismatic joint, 单位: N)
  std::mutex gripper_mutex_;
  double gripper_torque_cmd_ = 0.0;   // 当前夹爪力指令 (N)
  double gripper_max_torque_ = 70.0;  // 夹爪力限幅 (N), 从yaml覆盖
  rclcpp::Service<arv_v1_interfaces::srv::GripperControl>::SharedPtr gripper_service_;

  // Action 回调函数
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);  // 关节状态回调

  // 安全函数
  void emergencyStop(const std::string &reason) {
    RCLCPP_ERROR(this->get_logger(), "[EMERGENCY STOP] %s", reason.c_str());

    // 1. Send zero torque immediately
    std_msgs::msg::Float64MultiArray safe_msg;
    safe_msg.data.resize(kAllJoints, 0.0);
    torque_pub_->publish(safe_msg);

    // 2. Execute complete recovery ceremony
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      if (is_executing_.load(std::memory_order_acquire) && current_goal_handle_) {
        executionRecoveryCeremony(reason);
      }
    }
  }

  // 执行失败后完整清理: 清空轨迹、中止目标、复位积分器和滤波器、发送重力补偿
  // 注意: 必须在持有 action_mutex_ 时调用
  void executionRecoveryCeremony(const std::string &reason) {
    RCLCPP_ERROR(this->get_logger(), "[RECOVERY] %s", reason.c_str());

    current_trajectory_ = trajectory_msgs::msg::JointTrajectory();
    trajectory_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    if (current_goal_handle_) {
      if (current_goal_handle_->is_active()) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "Recovery: " + reason;
        current_goal_handle_->abort(result);
      }
      current_goal_handle_.reset();
    }

    is_executing_.store(false, std::memory_order_release);

    if (cascade_pid_) cascade_pid_->resetAll();

    if (kalman_filter_enabled_) {
      std::lock_guard<std::mutex> filter_lock(filter_mutex_);
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      for (size_t i = 0; i < 6; i++) {
        joint_filters_[i].initialize(q_actual_(i), q_dot_actual_(i));
      }
    }

    // 发送重力补偿保持位置
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (state_received_) {
        KDL::JntArray tau_gravity(6);
        dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);

        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data.resize(kAllJoints);
        for (int i = 0; i < kArmJoints; i++) {
          double limit =
              (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
          torque_msg.data[i] = std::clamp(tau_gravity(i), -limit, limit);
        }
        // 夹爪: 紧急停止时设为0
        torque_msg.data[kArmJoints] = 0.0;
        torque_pub_->publish(torque_msg);
      }
    }

    RCLCPP_INFO(this->get_logger(), "[RECOVERY] Complete - Ready for new goals");
  }

  bool initializeDynamics();  // 动力学初始化

  bool interpolateTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, double t_now,
                             KDL::JntArray &q_d, KDL::JntArray &qd_d, KDL::JntArray &qdd_d);

  void computeFeedbackTorque(const KDL::JntArray &q_d,        // 期望位置
                             const KDL::JntArray &qd_d,       // 期望速度
                             const KDL::JntArray &q_actual,   // 实际位置
                             const KDL::JntArray &qd_actual,  // 实际速度
                             KDL::JntArray &tau_fb);          // 输出：反馈力矩

  void controlLoop();  // 核心控制循环

  // --- 新增：参数动态调节 ---
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 参数变化回调函数
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  // 夹爪控制服务回调
  void gripperControlCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::GripperControl::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::GripperControl::Response> response) {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    double clamped = std::clamp(request->torque, -gripper_max_torque_, gripper_max_torque_);
    gripper_torque_cmd_ = clamped;
    response->success = true;
    response->message = "Gripper torque set to " + std::to_string(clamped) + " Nm";
    RCLCPP_INFO(this->get_logger(), "[GRIPPER] Torque command: %.3f Nm", clamped);
  }
};

rclcpp_action::GoalResponse TorqueControllerActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "[INFO] New trajectory received (%zu points)",
              goal->trajectory.points.size());

  bool currently_executing;
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    currently_executing = is_executing_.load(std::memory_order_acquire);
  }

  if (currently_executing) {
    RCLCPP_WARN(this->get_logger(),
                "[WARN] Detected new trajectory, will preempt current execution");
    // 不再 REJECT，而是继续接受
  }

  if (goal->trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory is empty, goal rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "[OK] New goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TorqueControllerActionServer::handleCancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "[INFO] Trajectory manually cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TorqueControllerActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "[INFO] Goal accepted, ready to execute");

  // 检查是否收到关节状态（不需要锁，只读取）
  bool has_state;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    has_state = state_received_;
  }

  if (!has_state) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] No joint state data received, execution refused");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
    goal_handle->abort(result);
    return;
  }

  // 避免嵌套锁: 先拷贝状态数据，再持有action锁
  KDL::JntArray q_current(kArmJoints);
  has_state = false;  // Reset and reuse the has_state variable from above
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (state_received_) {
      q_current = q_actual_;
      has_state = true;
    }
  }
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);

    if (is_executing_.load(std::memory_order_acquire) && current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "[WARN] Cancelling old trajectory, switching to new one");

      // 通知旧轨迹被抢占
      auto old_result = std::make_shared<FollowJointTrajectory::Result>();
      old_result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
      current_goal_handle_->abort(old_result);
    }

    // 缓存轨迹
    const auto goal = goal_handle->get_goal();
    current_trajectory_ = goal->trajectory;
    current_goal_handle_ = goal_handle;
    trajectory_start_time_ = this->now();
    is_executing_.store(true, std::memory_order_release);

    // 转发轨迹到话题，供 trajectory_manager_node 捕获保存
    trajectory_forward_pub_->publish(current_trajectory_);
    RCLCPP_DEBUG(this->get_logger(),
                 "[FORWARD] Trajectory published to /ARM_controller/joint_trajectory");

    // 打印轨迹信息
    const auto &first_point = current_trajectory_.points[0];
    const auto &last_point = current_trajectory_.points.back();
    double total_duration =
        last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

    // --- 智能积分器清零策略 ---
    // 检查位置连续性：比较轨迹起点和当前位置 (使用局部拷贝q_current)
    bool position_continuous = true;
    double max_pos_jump = 0.0;

    if (first_point.positions.size() >= kArmJoints && has_state) {
      // 使用局部拷贝 q_current 而非 q_actual_, 避免再次加锁
      for (size_t i = 0; i < kArmJoints; i++) {
        double pos_error = std::abs(first_point.positions[i] - q_current(i));
        max_pos_jump = std::max(max_pos_jump, pos_error);
        if (pos_error > 0.05)  // 阈值：5度（0.087 rad）或更保守的0.05 rad
        {
          position_continuous = false;
        }
      }
    }

    if (!position_continuous) {
      // 位置不连续：必须完全清零积分器，避免冲击
      cascade_pid_->resetAll();
      RCLCPP_WARN(this->get_logger(),
                  "[PID] Position discontinuity detected (max_jump=%.3f rad), integrators cleared",
                  max_pos_jump);
    } else {
      // 清零积分器，避免速度指令跳变影响
      cascade_pid_->resetAll();
      RCLCPP_INFO(this->get_logger(),
                  "[PID] Position continuous (max_jump=%.4f rad), but velocity command changes, "
                  "integrators cleared",
                  max_pos_jump);
    }

    // --- 新增：保存规划终点位置 ---
    if (last_point.positions.size() >= kArmJoints) {
      for (size_t i = 0; i < kArmJoints; i++) {
        q_target_(i) = last_point.positions[i];
      }
      has_target_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "[INFO] Target end-point: q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  q_target_(0), q_target_(1), q_target_(2), q_target_(3), q_target_(4),
                  q_target_(5));
    }

    RCLCPP_INFO(this->get_logger(), "[INFO] Trajectory cached (%zu points, %.3fs)",
                current_trajectory_.points.size(), total_duration);
  }
}

void TorqueControllerActionServer::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // --- 修复死锁 #1: 使用unique_lock手动控制锁生命周期 ---
  std::unique_lock<std::mutex> state_lock(state_mutex_);

  // 验证数据
  if (msg->position.size() < kAllJoints || msg->velocity.size() < kAllJoints) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[ERROR] Invalid joint state size! Expected at least %d", kAllJoints);
    return;
  }

  // --- SAFETY: Validate sensor data for NaN/Inf and sanity ---
  bool has_velocity_spike = false;
  for (size_t i = 0; i < kArmJoints; i++) {
    // Check for NaN or Inf (这种情况必须完全拒绝)
    if (!std::isfinite(msg->position[i]) || !std::isfinite(msg->velocity[i])) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[SAFETY] Non-finite sensor data on joint %zu (pos=%.2f, vel=%.2f), rejecting!", i,
          msg->position[i], msg->velocity[i]);
      return;  // Reject entire message - 完全拒绝，不更新时间戳
    }

    // Sanity check: position within expanded limits (warn but accept)
    if (std::abs(msg->position[i]) > 2 * M_PI) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[SAFETY] Joint %zu position out of range: %.2f rad", i,
                           msg->position[i]);
    }

    // Sanity check: velocity spike detection (限幅而不是拒绝，避免timeout)
    if (std::abs(msg->velocity[i]) > max_velocity_sanity_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu velocity spike: %.2f rad/s (limit: %.1f), clamping!",
                           i, msg->velocity[i], max_velocity_sanity_);
      has_velocity_spike = true;
      // 不再return，而是继续处理，但会限幅速度
    }
  }

  // 提取位置和速度（velocity spike时进行限幅）
  for (size_t i = 0; i < kArmJoints; i++) {
    q_actual_(i) = msg->position[i];

    // 速度限幅：如果检测到spike，限制在安全范围内
    if (std::abs(msg->velocity[i]) > max_velocity_sanity_) {
      q_dot_actual_(i) = (msg->velocity[i] > 0) ? max_velocity_sanity_ : -max_velocity_sanity_;
    } else {
      q_dot_actual_(i) = msg->velocity[i];
    }
  }

  // --- 关键：即使有velocity spike也更新时间戳，避免误判timeout ---
  last_joint_state_time_ = this->now();

  // --- 修复数据竞争 #2: Kalman滤波处理 (添加filter_mutex_保护) ---
  // 注意：卡尔曼滤波器只用于速度滤波，位置保持原始测量值（编码器精度高）
  if (kalman_filter_enabled_) {
    // 获取 filter_mutex_ 保护 joint_filters_ 的访问
    std::lock_guard<std::mutex> filter_lock(filter_mutex_);

    // 启用滤波：使用卡尔曼滤波器过滤速度
    for (size_t i = 0; i < kArmJoints; i++) {
      // 首次接收：初始化滤波器
      if (!state_received_) {
        joint_filters_[i].initialize(msg->position[i], msg->velocity[i]);
      } else {
        // 后续：预测-更新循环
        joint_filters_[i].predict();
        joint_filters_[i].update(msg->position[i], msg->velocity[i]);
      }

      // [NOTE] Key modification: only use filtered velocity, position remains original
      // q_actual_(i) 已经在上面设置为 msg->position[i]，保持不变
      q_dot_filtered_(i) = joint_filters_[i].getVelocity();
    }
  }  // filter_mutex_ 自动释放
  else {
    // 禁用滤波：直接使用原始测量值
    for (size_t i = 0; i < kArmJoints; i++) {
      q_dot_filtered_(i) = q_dot_actual_(i);  // 直接使用原始速度
                                              // q_actual_ 已在上面赋值为原始位置，保持不变
    }
  }

  // --- 修复死锁 #1: 首次接收数据时避免嵌套锁 ---
  // 策略: 先在 state_mutex_ 下拷贝数据和标记, 然后释放锁, 再获取 action_mutex_
  bool is_first_state = !state_received_;
  KDL::JntArray q_startup(kArmJoints);

  if (is_first_state) {
    RCLCPP_INFO(this->get_logger(), "[OK] First joint state data received");

    // 拷贝启动位置 (仍在state_mutex_保护下)
    q_startup = q_actual_;

    // --- 关键：在所有初始化完成后才标记state_received_ ---
    // 确保状态机闭环：只有当数据完整处理后才设置标志
    state_received_ = true;

    RCLCPP_INFO(this->get_logger(), "[STATE] state_received=true, ready for control");
  }

  // 打印首次Kalman增益 (在所有锁释放后,避免嵌套锁)
  if (is_first_state && kalman_filter_enabled_) {
    // 需要 filter_mutex_ 来安全访问 joint_filters_[0]
    std::lock_guard<std::mutex> filter_lock(filter_mutex_);
    RCLCPP_INFO(this->get_logger(), "📝  First Kalman gain (Joint 1):");
    auto K = joint_filters_[0].getKalmanGain();
    RCLCPP_INFO(this->get_logger(), "   K = [%.4f, %.4f]", K(0, 0), K(0, 1));
    RCLCPP_INFO(this->get_logger(), "       [%.4f, %.4f]", K(1, 0), K(1, 1));
  }

  // 释放 state_mutex_ (通过手动unlock)
  state_lock.unlock();

  // 现在可以安全地获取 action_mutex_ 而不会死锁
  if (is_first_state) {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    if (!has_target_) {
      q_target_ = q_startup;
      has_target_ = true;

      RCLCPP_INFO(this->get_logger(), "[INFO] Saving startup pose as initial target:");
      RCLCPP_INFO(this->get_logger(), "   q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  q_target_(0), q_target_(1), q_target_(2), q_target_(3), q_target_(4),
                  q_target_(5));
    }
  }
}

bool TorqueControllerActionServer::initializeDynamics() {
  RCLCPP_INFO(this->get_logger(), "[INFO] Starting dynamics solver initialization...");

  // 1. 读取 URDF 文件（使用 ament_index 动态查找）
  std::string urdf_path;
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("arv_v1_model");
    urdf_path = pkg_path + "/urdf/arv_v1.urdf";
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to find arv_v1_model package: %s", e.what());
    return false;
  }

  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot open URDF file: %s", urdf_path.c_str());
    return false;
  }

  std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
  urdf_file.close();

  // 2. 解析 URDF
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_string)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] URDF parsing failed");
    return false;
  }

  // --- SAFETY: Extract joint effort limits from URDF ---
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3",
                                          "joint_4", "joint_5", "joint_6"};
  for (size_t i = 0; i < kArmJoints; i++) {
    auto joint = urdf_model.getJoint(joint_names[i]);
    if (joint && joint->limits) {
      double urdf_limit = joint->limits->effort;
      // Only use URDF limit if not overridden in config (config value <= 0)
      if (max_torque_per_joint_[i] <= 0) {
        max_torque_per_joint_[i] = urdf_limit;
        RCLCPP_INFO(this->get_logger(), "[SAFETY] Joint %s: Using URDF effort limit: %.1f Nm",
                    joint_names[i].c_str(), urdf_limit);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "[SAFETY] Joint %s: Using config override: %.1f Nm (URDF: %.1f Nm)",
                    joint_names[i].c_str(), max_torque_per_joint_[i], urdf_limit);
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "[SAFETY] Joint %s: No URDF limit found, using default: %.1f Nm",
                  joint_names[i].c_str(), max_torque_per_joint_[i]);
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "[SAFETY] Final torque limits (Nm): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
              max_torque_per_joint_[0], max_torque_per_joint_[1], max_torque_per_joint_[2],
              max_torque_per_joint_[3], max_torque_per_joint_[4], max_torque_per_joint_[5]);

  // 3. 提取 KDL 树
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to build KDL tree from URDF");
    return false;
  }

  // 4. 获取运动链（从 base_link 到 link6）
  if (!kdl_tree.getChain("base_link", "link6", kdl_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to extract kinematic chain");
    return false;
  }

  // 5. 创建动力学计算工具
  KDL::Vector gravity(0.0, 0.0, -9.81);  // 重力向量
  dynamic_computer_ = std::make_unique<DynamicsComputer>(kdl_chain_, gravity);

  // 设置错误日志回调，将 DynamicsComputer 的错误转发到 ROS2 日志系统
  // 捕获 this 指针以安全访问节点的 logger
  dynamic_computer_->setErrorLogger(
      [this](const std::string &msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); });

  RCLCPP_INFO(this->get_logger(), "[OK] Dynamics solver initialized");
  RCLCPP_INFO(this->get_logger(), "   - Gravity: [%.2f, %.2f, %.2f] m/s²", gravity.x(), gravity.y(),
              gravity.z());

  return true;
}

bool TorqueControllerActionServer::interpolateTrajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory, double t_now, KDL::JntArray &q_d,
    KDL::JntArray &qd_d, KDL::JntArray &qdd_d) {
  // 1. 检查轨迹是否存在
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory is empty, cannot interpolate");
    return false;
  }

  // 2. 如果时间在第一个点之前，返回第一个点
  const auto &first_point = trajectory.points[0];
  double t_first = first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9;

  if (t_now <= t_first) {
    // 返回第一个点的值
    for (size_t i = 0; i < kArmJoints; i++) {
      if (i < first_point.positions.size()) {
        q_d(i) = first_point.positions[i];
      } else {
        q_d(i) = 0.0;  // Fail-safe
      }
      qd_d(i) = (i < first_point.velocities.size()) ? first_point.velocities[i] : 0.0;
      qdd_d(i) = (i < first_point.accelerations.size()) ? first_point.accelerations[i] : 0.0;
    }
    return true;
  }

  // 3. 如果时间在最后一个点之后，返回最后一个点
  const auto &last_point = trajectory.points.back();
  double t_last = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

  if (t_now >= t_last) {
    // 返回最后一个点的值（速度和加速度应该为 0）
    for (size_t i = 0; i < kArmJoints; i++) {
      if (i < last_point.positions.size()) {
        q_d(i) = last_point.positions[i];
      } else {
        q_d(i) = 0.0;
      }
      qd_d(i) = 0.0;   // 停止时速度为 0
      qdd_d(i) = 0.0;  // 停止时加速度为 0
    }
    return true;
  }

  // 4. 在轨迹中查找包围当前时间的两个点
  size_t idx_before = 0;
  size_t idx_after = 0;

  for (size_t i = 0; i < trajectory.points.size() - 1; i++) {
    double t_i = trajectory.points[i].time_from_start.sec +
                 trajectory.points[i].time_from_start.nanosec * 1e-9;
    double t_i_next = trajectory.points[i + 1].time_from_start.sec +
                      trajectory.points[i + 1].time_from_start.nanosec * 1e-9;

    if (t_now >= t_i && t_now <= t_i_next) {
      idx_before = i;
      idx_after = i + 1;
      break;
    }
  }

  // 5. 获取两个点
  const auto &point_before = trajectory.points[idx_before];
  const auto &point_after = trajectory.points[idx_after];

  double t_before = point_before.time_from_start.sec + point_before.time_from_start.nanosec * 1e-9;
  double t_after = point_after.time_from_start.sec + point_after.time_from_start.nanosec * 1e-9;

  // 6. 计算插值比例 α
  double alpha = (t_now - t_before) / (t_after - t_before);

  // 防止除零
  if (t_after - t_before < 1e-9) {
    alpha = 0.0;
  }

  // 7. 线性插值
  for (size_t i = 0; i < kArmJoints; i++) {
    // 位置插值
    double pos_before = (i < point_before.positions.size()) ? point_before.positions[i] : 0.0;
    double pos_after = (i < point_after.positions.size()) ? point_after.positions[i] : 0.0;
    q_d(i) = pos_before + alpha * (pos_after - pos_before);

    // 速度插值
    if (i < point_before.velocities.size() && i < point_after.velocities.size()) {
      qd_d(i) = point_before.velocities[i] +
                alpha * (point_after.velocities[i] - point_before.velocities[i]);
    } else {
      qd_d(i) = 0.0;
    }

    // 加速度插值
    if (i < point_before.accelerations.size() && i < point_after.accelerations.size()) {
      qdd_d(i) = point_before.accelerations[i] +
                 alpha * (point_after.accelerations[i] - point_before.accelerations[i]);
    } else {
      qdd_d(i) = 0.0;
    }
  }

  return true;
}

// --- PD 反馈控制 ---
void TorqueControllerActionServer::computeFeedbackTorque(const KDL::JntArray &q_d,
                                                         const KDL::JntArray &qd_d,
                                                         const KDL::JntArray &q_actual,
                                                         const KDL::JntArray &qd_actual,
                                                         KDL::JntArray &tau_fb) {
  // 级联 P+PI: 外环P生成速度指令, 内环PI输出力矩
  // qd_actual 应传入卡尔曼滤波后的速度以减少噪声

  if (!cascade_pid_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[ERROR] Cascade P+PI not initialized!");
    for (size_t i = 0; i < kArmJoints; i++) {
      tau_fb(i) = 0.0;
    }
    return;
  }

  std::vector<double> pos_ref(kArmJoints), pos_fdb(kArmJoints), vel_fdb(kArmJoints),
      torque_out(kArmJoints);

  // 准备输入数据
  for (size_t i = 0; i < kArmJoints; i++) {
    pos_ref[i] = q_d(i);        // 期望位置
    pos_fdb[i] = q_actual(i);   // 实际位置（编码器，高精度）
    vel_fdb[i] = qd_actual(i);  // 实际速度（卡尔曼滤波后，低噪声）
  }

  double dt = 1.0 / control_frequency_;  // 200Hz -> 0.005s
  cascade_pid_->compute(pos_ref, pos_fdb, vel_fdb, dt, torque_out);

  // 输出力矩并进行安全限幅
  for (size_t i = 0; i < kArmJoints; i++) {
    tau_fb(i) = torque_out[i];

    // --- SAFETY: Torque saturation (hardware protection) ---
    double joint_limit =
        (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
    if (tau_fb(i) > joint_limit) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu feedback torque saturated: %.2f -> %.2f Nm", i,
                           tau_fb(i), joint_limit);
      tau_fb(i) = joint_limit;
    } else if (tau_fb(i) < -joint_limit) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu feedback torque saturated: %.2f -> -%.2f Nm", i,
                           tau_fb(i), joint_limit);
      tau_fb(i) = -joint_limit;
    }
  }
}

void TorqueControllerActionServer::controlLoop() {
  // --- SAFETY: Monitor control loop timing ---
  rclcpp::Time now = this->now();
  if (control_loop_count_ > 0)  // Skip first iteration
  {
    double period = (now - last_control_loop_time_).seconds();
    if (period > max_control_period_sec_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[SAFETY] Control loop slow: %.1f ms (expected 5ms @ 200Hz)",
                           period * 1000.0);
    }
  }
  last_control_loop_time_ = now;

  // --- Health monitoring ---
  control_loop_count_++;

  // Print health status every 5 seconds (1000 loops at 200Hz)
  if (control_loop_count_ % 1000 == 0) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    double time_since_state =
        state_received_ ? (this->now() - last_joint_state_time_).seconds() : -1.0;

    RCLCPP_INFO(
        this->get_logger(), "[HEALTH] Loop #%zu | Mode: %s | State age: %.1f ms | Kalman: %s",
        control_loop_count_, is_executing_.load(std::memory_order_acquire) ? "EXECUTING" : "HOLD",
        time_since_state * 1000.0, kalman_filter_enabled_ ? "ON" : "OFF");
  }

  // 检查是否正在执行轨迹（原子读取，无需锁）
  bool executing = is_executing_.load(std::memory_order_acquire);

  if (!executing) {
    // 没有活动轨迹时，发送重力补偿力矩保持位置
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    if (!state_received_) {
      return;  // 还没收到状态，无法计算
    }

    // --- SAFETY: Check joint state timeout ---
    double time_since_last_state = (this->now() - last_joint_state_time_).seconds();
    if (time_since_last_state > joint_state_timeout_sec_) {
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[SAFETY] Joint state timeout: %.3fs since last update (limit: %.0f ms)",
          time_since_last_state, joint_state_timeout_sec_ * 1000.0);

      // Timeout: 使用重力补偿维持位置，不直接return

      KDL::JntArray tau_gravity(kArmJoints);
      bool can_compute_gravity = false;

      // 尝试用最后的位置数据计算重力补偿
      try {
        dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);
        can_compute_gravity = true;
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "[SAFETY] Cannot compute gravity, sending zero torque");
      }

      std_msgs::msg::Float64MultiArray safe_msg;
      safe_msg.data.resize(kAllJoints);
      if (can_compute_gravity) {
        // 使用重力补偿维持位置
        for (int i = 0; i < kArmJoints; i++) {
          safe_msg.data[i] = tau_gravity(i);
          // 安全限幅
          double joint_limit =
              (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
          safe_msg.data[i] = std::max(-joint_limit, std::min(joint_limit, safe_msg.data[i]));
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[SAFETY] Timeout: Publishing gravity compensation only");
      } else {
        // 无法计算，发送零力矩
        for (int i = 0; i < kAllJoints; i++) {
          safe_msg.data[i] = 0.0;
        }
      }
      // 夹爪: timeout时保持当前力矩指令
      {
        std::lock_guard<std::mutex> glock(gripper_mutex_);
        safe_msg.data[kArmJoints] = gripper_torque_cmd_;
      }
      torque_pub_->publish(safe_msg);

      // Abort any active trajectory (need to check again with action_mutex)
      std::shared_ptr<GoalHandleFJT> goal_to_abort;
      {
        std::lock_guard<std::mutex> action_lock(action_mutex_);
        if (current_goal_handle_) {
          goal_to_abort = current_goal_handle_;
        }
      }
      if (goal_to_abort) {
        emergencyStop("Joint state timeout");
      }

      // --- 不再return，继续等待下次循环 ---
      // 这样timeout后能自动恢复，不会卡在这里
      return;
    }

    // 计算重力补偿
    KDL::JntArray tau_gravity(kArmJoints);
    dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);

    // --- 修改：PD 控制目标位置 ---
    KDL::JntArray tau_pd(kArmJoints);
    if (has_target_) {
      computeFeedbackTorque(q_target_, KDL::JntArray(kArmJoints), q_actual_, q_dot_filtered_,
                            tau_pd);
    } else {
      computeFeedbackTorque(q_actual_, KDL::JntArray(kArmJoints), q_actual_, q_dot_filtered_,
                            tau_pd);
    }

    // 发布力矩：重力补偿 + PD 控制
    std_msgs::msg::Float64MultiArray torque_msg;
    torque_msg.data.resize(kAllJoints);
    for (int i = 0; i < kArmJoints; i++) {
      torque_msg.data[i] = tau_gravity(i) + tau_pd(i);

      // --- SAFETY: NaN/Inf check before publishing ---
      if (!std::isfinite(torque_msg.data[i])) {
        RCLCPP_ERROR(
            this->get_logger(),
            "[SAFETY] Non-finite torque detected on joint %d in idle mode (gravity=%.2f, pd=%.2f)",
            i, tau_gravity(i), tau_pd(i));
        emergencyStop("Non-finite torque in idle mode");
        return;
      }

      // --- SAFETY: Final torque saturation ---
      double joint_limit =
          (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
      if (torque_msg.data[i] > joint_limit) {
        torque_msg.data[i] = joint_limit;
      } else if (torque_msg.data[i] < -joint_limit) {
        torque_msg.data[i] = -joint_limit;
      }
    }
    // 夹爪: 透传当前力矩指令
    {
      std::lock_guard<std::mutex> glock(gripper_mutex_);
      torque_msg.data[kArmJoints] = gripper_torque_cmd_;
    }
    torque_pub_->publish(torque_msg);

    return;
  }

  // --- 执行轨迹阶段 ---
  // 读取 action 状态和轨迹信息（线程安全）
  trajectory_msgs::msg::JointTrajectory current_traj_copy;
  rclcpp::Time traj_start_copy;
  std::shared_ptr<GoalHandleFJT> goal_handle_copy;
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    current_traj_copy = current_trajectory_;
    traj_start_copy = trajectory_start_time_;
    goal_handle_copy = current_goal_handle_;
  }

  // 现在时间获取
  double t_now = (this->now() - traj_start_copy).seconds();

  // 获取轨迹TIME
  const auto &last_point = current_traj_copy.points.back();
  double total_duration =
      last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

  if (t_now >= total_duration)  // 检查是否完成
  {
    RCLCPP_INFO(this->get_logger(), "[OK] Trajectory execution completed!");
    KDL::JntArray q_actual_copy(kArmJoints), qd_filtered_copy(kArmJoints);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      q_actual_copy = q_actual_;
      qd_filtered_copy = q_dot_filtered_;  // 从成员变量拷贝滤波后的速度
    }

    // 计算重力补偿
    KDL::JntArray tau_gravity(kArmJoints);
    dynamic_computer_->computeGravityTorque(q_actual_copy, tau_gravity);

    KDL::JntArray tau_pd(kArmJoints);
    if (has_target_) {
      // 期望位置 = 规划终点，期望速度 = 0
      computeFeedbackTorque(q_target_, KDL::JntArray(kArmJoints), q_actual_copy, qd_filtered_copy,
                            tau_pd);
    } else {
      // 如果没有目标，PD 输出为 0
      for (int i = 0; i < kArmJoints; i++) {
        tau_pd(i) = 0.0;
      }
    }

    // 发布力矩：重力补偿 + PD 控制
    std_msgs::msg::Float64MultiArray hold_torque;
    hold_torque.data.resize(kAllJoints);
    for (int i = 0; i < kArmJoints; i++) {
      hold_torque.data[i] = tau_gravity(i) + tau_pd(i);

      // --- SAFETY: NaN/Inf check and saturation ---
      if (!std::isfinite(hold_torque.data[i])) {
        RCLCPP_ERROR(this->get_logger(),
                     "[SAFETY] Non-finite torque at trajectory completion on joint %d", i);
        emergencyStop("Non-finite torque at trajectory completion");
        return;
      }
      double joint_limit =
          (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
      hold_torque.data[i] = std::max(-joint_limit, std::min(joint_limit, hold_torque.data[i]));
    }
    // 夹爪: 透传当前力矩指令
    {
      std::lock_guard<std::mutex> glock(gripper_mutex_);
      hold_torque.data[kArmJoints] = gripper_torque_cmd_;
    }
    torque_pub_->publish(hold_torque);

    RCLCPP_INFO(this->get_logger(),
                "[OK] Trajectory execution completed, switching to hold mode (target: q_target)");
    RCLCPP_INFO(this->get_logger(), "   τ_total=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                hold_torque.data[0], hold_torque.data[1], hold_torque.data[2], hold_torque.data[3],
                hold_torque.data[4], hold_torque.data[5]);

    // 返回成功结果并清理状态（线程安全）
    if (goal_handle_copy) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle_copy->succeed(result);
    }

    // 清理状态（但保留 q_target_ 和 has_target_）
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      is_executing_.store(false, std::memory_order_release);
      current_goal_handle_.reset();
    }
    return;
  }

  // 插值（时间不连续）
  KDL::JntArray q_d(kArmJoints), qd_d(kArmJoints), qdd_d(kArmJoints);
  if (!interpolateTrajectory(current_traj_copy, t_now, q_d, qd_d, qdd_d)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory interpolation failed at t=%.3fs", t_now);

    // ✅ Fix: Trigger recovery ceremony to clean up state
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    if (current_goal_handle_ && is_executing_.load(std::memory_order_acquire)) {
      executionRecoveryCeremony("Interpolation failure");
    }
    return;
  }

  // 获取实际状态（线程安全）
  KDL::JntArray q_actual(kArmJoints), qd_filtered(kArmJoints);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    q_actual = q_actual_;
    qd_filtered = q_dot_filtered_;  // 从成员变量拷贝滤波后的速度
  }

  KDL::JntArray tau_ff(kArmJoints);  // PD的话需要输入期望和实际，前馈只需要期望
  dynamic_computer_->computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff);
  KDL::JntArray tau_fb(kArmJoints);
  computeFeedbackTorque(q_d, qd_d, q_actual, qd_filtered,
                        tau_fb);  // 计算PD反馈力矩，使用滤波后的速度

  KDL::JntArray tau_total(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    tau_total(i) = tau_ff(i) + tau_fb(i);
  }  // PD+前馈力矩

  // 发送力矩到 ros2_control effort_controller
  std_msgs::msg::Float64MultiArray torque_msg;
  torque_msg.data.resize(kAllJoints);
  for (int i = 0; i < kArmJoints; i++) {
    // --- SAFETY: NaN/Inf check before publishing ---
    if (!std::isfinite(tau_total(i))) {
      RCLCPP_ERROR(
          this->get_logger(),
          "[SAFETY] Non-finite torque detected on joint %d during trajectory (ff=%.2f, fb=%.2f)", i,
          tau_ff(i), tau_fb(i));
      emergencyStop("Non-finite torque during trajectory execution");
      return;
    }

    // --- SAFETY: Final torque saturation ---
    double joint_limit =
        (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
    torque_msg.data[i] = std::max(-joint_limit, std::min(joint_limit, tau_total(i)));

    if (std::abs(tau_total(i)) > joint_limit) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %d total torque saturated: %.2f -> %.2f Nm", i,
                           tau_total(i), torque_msg.data[i]);
    }
  }
  // 夹爪: 透传当前力矩指令
  {
    std::lock_guard<std::mutex> glock(gripper_mutex_);
    torque_msg.data[kArmJoints] = gripper_torque_cmd_;
  }
  torque_pub_->publish(torque_msg);

  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = this->now();  // 获取反馈指针

  feedback->actual.positions.resize(kArmJoints);
  feedback->actual.velocities.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    feedback->actual.positions[i] = q_actual(i);
    feedback->actual.velocities[i] = qd_filtered(i);  // 反馈使用滤波后的速度
  }

  feedback->desired.positions.resize(kArmJoints);
  feedback->desired.velocities.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    feedback->desired.positions[i] = q_d(i);
    feedback->desired.velocities[i] = qd_d(i);  // 更新q-期望
  }

  feedback->error.positions.resize(kArmJoints);
  feedback->error.velocities.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    feedback->error.positions[i] = q_d(i) - q_actual(i);
    feedback->error.velocities[i] = qd_d(i) - qd_filtered(i);  // 速度误差基于滤波后的速度
  }

  current_goal_handle_->publish_feedback(feedback);
}

// --- 参数变化回调函数实现 ---
rcl_interfaces::msg::SetParametersResult TorqueControllerActionServer::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "参数更新成功";

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    // --- 卡尔曼滤波器参数动态更新 ---
    if (name == "kalman.enabled") {
      // 更新卡尔曼滤波开关
      kalman_filter_enabled_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter %s",
                  kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");
    } else if (name == "kalman.Q_pos" || name == "kalman.Q_vel") {
      // 安全检查：确保所有相关参数都已声明
      if (this->has_parameter("kalman.Q_pos") && this->has_parameter("kalman.Q_vel")) {
        double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
        double Q_vel = this->get_parameter("kalman.Q_vel").as_double();

        // --- 修复数据竞争 #2: 修改 joint_filters_ 前获取 filter_mutex_ ---
        {
          std::lock_guard<std::mutex> filter_lock(filter_mutex_);
          for (auto &filter : joint_filters_) {
            filter.setProcessNoise(Q_pos, Q_vel);
          }
        }  // filter_mutex_ 自动释放

        RCLCPP_INFO(this->get_logger(), "[CONFIG] Process noise updated: Q_pos=%.1e, Q_vel=%.1e",
                    Q_pos, Q_vel);
      }
    } else if (name == "kalman.R_pos" || name == "kalman.R_vel") {
      // 安全检查：确保所有相关参数都已声明
      if (this->has_parameter("kalman.R_pos") && this->has_parameter("kalman.R_vel")) {
        double R_pos = this->get_parameter("kalman.R_pos").as_double();
        double R_vel = this->get_parameter("kalman.R_vel").as_double();

        // --- 修复数据竞争 #2: 修改 joint_filters_ 前获取 filter_mutex_ ---
        {
          std::lock_guard<std::mutex> filter_lock(filter_mutex_);
          for (auto &filter : joint_filters_) {
            filter.setMeasurementNoise(R_pos, R_vel);
          }
        }  // filter_mutex_ 自动释放

        RCLCPP_INFO(this->get_logger(),
                    "[CONFIG] Measurement noise updated: R_pos=%.1e, R_vel=%.1e", R_pos, R_vel);
      }
    }
    // --- 级联 P+PI 参数动态更新 ---
    else if (name.find("cascade_pid.joint_") == 0) {
      // 解析参数名: cascade_pid.joint_1.pos_Kp
      size_t first_dot = name.find('.', 13);  // 找到 "joint_X" 后的点
      if (first_dot == std::string::npos) continue;

      size_t second_dot = name.find('.', first_dot + 1);
      if (second_dot == std::string::npos) continue;

      std::string joint_num_str = name.substr(13, first_dot - 13);  // 提取 "1"
      std::string loop_type = name.substr(first_dot + 1, 3);        // 提取 "pos" 或 "vel"
      std::string gain_type = name.substr(second_dot + 1);  // 提取 "Kp"/"Ki"/"Kd" 或 "limit"

      int joint_idx = std::stoi(joint_num_str) - 1;  // 0-based索引

      if (joint_idx >= 0 && joint_idx < 6) {
        double new_value = param.as_double();

        // 读取该关节的所有参数
        std::string prefix = "cascade_pid.joint_" + std::to_string(joint_idx + 1);

        PidGains pos_gains(this->get_parameter(prefix + ".pos_Kp").as_double(),
                           this->get_parameter(prefix + ".pos_Ki").as_double(),
                           this->get_parameter(prefix + ".pos_Kd").as_double());

        PidGains vel_gains(this->get_parameter(prefix + ".vel_Kp").as_double(),
                           this->get_parameter(prefix + ".vel_Ki").as_double(),
                           this->get_parameter(prefix + ".vel_Kd").as_double());

        double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();

        // 更新级联PID
        cascade_pid_->setJointParams(joint_idx, pos_gains, vel_gains, vel_limit);

        RCLCPP_INFO(this->get_logger(), "[CONFIG] Cascade PID Joint %d updated: %s.%s = %.2f",
                    joint_idx + 1, loop_type.c_str(), gain_type.c_str(), new_value);
      }
    }
    // kalman.print_interval parameter removed
  }

  return result;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TorqueControllerActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}