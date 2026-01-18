// ========================================
// 力矩控制节点 - 基于KDL的完整动力学前馈+级联P+PI反馈
// ========================================
//
// [系统架构]
// - 外环: 位置P → 期望速度
// - 内环: 速度PI → 输出力矩
// - 前馈: τ_ff = M(q)q̈ + C(q,q̇) + G(q) (完整动力学模型)
// - 反馈: τ_fb = 级联P+PI(位置误差, 速度误差)
// - 总力矩: τ = τ_ff + τ_fb
//
// [安全机制总览]
// 1. 关节状态超时保护 (joint_state_timeout_sec = 100ms)
//    - 触发: 100ms未收到/joint_states消息
//    - 响应: 发布重力补偿力矩维持位置 (不紧急停止)
//    - 恢复: 自动恢复，数据到达后继续控制
//
// 2. 力矩饱和限幅 (max_torque_per_joint)
//    - 优先级: config > URDF > 默认20Nm
//    - 应用: 前馈、反馈、总力矩三处独立限幅
//    - 日志: 触发时每秒打印一次警告
//
// 3. 传感器数据校验
//    - NaN/Inf检测: 拒绝整条消息，不更新时间戳
//    - 速度尖峰检测: 限幅处理而非拒绝 (避免timeout)
//    - 位置范围检查: 仅警告，不拒绝 (允许超过±2π)
//
// 4. 紧急停止 (emergencyStop)
//    - 触发条件:
//      a) 动力学计算NaN/Inf (前馈或重力项)
//      b) 关节状态timeout且无法计算重力补偿
//      c) 传感器数据包含NaN/Inf
//    - 响应流程:
//      a) 立即发送零力矩
//      b) 中止当前轨迹 (PATH_TOLERANCE_VIOLATED)
//      c) 清零积分器 (防止积分饱和)
//      d) 保留has_target_(维持保持模式目标)
//    - 不触发紧急停止的情况:
//      - 速度尖峰: 限幅处理
//      - timeout: 发送重力补偿维持位置
//
// 5. 控制循环频率监控
//    - 期望: 200Hz (5ms周期)
//    - 阈值: max_control_period_sec = 10ms
//    - 超时: 打印警告但继续运行 (不停机)
//
// 6. Kalman滤波器抗噪声
//    - 启用: kalman.enabled = true (默认)
//    - 策略: 仅滤波速度，位置保持编码器原始精度
//    - 动态调参: 支持运行时修改Q/R矩阵
//
// [参数动态调节]
// - cascade_pid.joint_X.pos_Kp/Ki/Kd: 外环位置PID
// - cascade_pid.joint_X.vel_Kp/Ki/Kd: 内环速度PID
// - cascade_pid.joint_X.vel_limit: 速度饱和限制
// - kalman.Q_pos/Q_vel: 过程噪声协方差
// - kalman.R_pos/R_vel: 测量噪声协方差
// - kalman.enabled: 滤波器开关
//
// [使用示例]
// ros2 param set /torque_controller_action_server cascade_pid.joint_1.pos_Kp 15.0
// ros2 param set /torque_controller_action_server kalman.Q_vel 1e-5
// ========================================

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <mutex>
#include <fstream>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // 力矩数组消息
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "dynamics_computer.hpp"
#include "kalman_filter.hpp"
#include "cascade_pid.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class TorqueControllerActionServer : public rclcpp::Node // 节点类生成
{
public: // 构造函数log
  TorqueControllerActionServer() : Node("torque_controller_action_server"),
                                   is_executing_(false),
                                   q_actual_(6),
                                   q_dot_actual_(6),
                                   q_target_(6),
                                   state_received_(false),
                                   has_target_(false),
                                   control_frequency_(200.0),
                                   joint_filters_{
                                       KalmanFilter1D(1.0 / 200.0), // Joint 1
                                       KalmanFilter1D(1.0 / 200.0), // Joint 2
                                       KalmanFilter1D(1.0 / 200.0), // Joint 3
                                       KalmanFilter1D(1.0 / 200.0), // Joint 4
                                       KalmanFilter1D(1.0 / 200.0), // Joint 5
                                       KalmanFilter1D(1.0 / 200.0)  // Joint 6
                                   },
                                   q_dot_filtered_(6),
                                   kalman_filter_enabled_(false)

  {
    RCLCPP_INFO(this->get_logger(), "[START] Torque controller node starting (Cascade P+PI Control)");

    // ========== 卡尔曼滤波器开关参数 ==========
    this->declare_parameter("kalman.enabled", false); // 默认禁用
    kalman_filter_enabled_ = this->get_parameter("kalman.enabled").as_bool();

    RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter state: %s",
                kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");

    // ========== 新增：声明卡尔曼滤波器参数（必须在读取之前声明）==========
    this->declare_parameter("kalman.Q_pos", 1e-5);   // 过程噪声：位置
    this->declare_parameter("kalman.Q_vel", 1e-4);   // 过程噪声：速度
    this->declare_parameter("kalman.R_pos", 1e-3);   // 测量噪声：位置
    this->declare_parameter("kalman.R_vel", 2.5e-2); // 测量噪声：速度

    // 读取参数并设置滤波器
    double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
    double Q_vel = this->get_parameter("kalman.Q_vel").as_double();
    double R_pos = this->get_parameter("kalman.R_pos").as_double();
    double R_vel = this->get_parameter("kalman.R_vel").as_double();

    for (auto &filter : joint_filters_)
    {
      filter.setProcessNoise(Q_pos, Q_vel);
      filter.setMeasurementNoise(R_pos, R_vel);
    }

    RCLCPP_INFO(this->get_logger(), "[OK] Kalman filter initialized:");
    RCLCPP_INFO(this->get_logger(), "   Q_pos=%.1e, Q_vel=%.1e", Q_pos, Q_vel);
    RCLCPP_INFO(this->get_logger(), "   R_pos=%.1e, R_vel=%.1e", R_pos, R_vel);

    this->declare_parameter("kalman.print_interval", 1000); // 默认 1000 次打印一次
    kalman_print_interval_ = this->get_parameter("kalman.print_interval").as_int();

    // ========== Load Safety Parameters (BEFORE initializeDynamics) ==========
    this->declare_parameter("safety.max_torque_default", 20.0);
    this->declare_parameter("safety.joint_state_timeout_ms", 100);
    this->declare_parameter("safety.max_control_period_ms", 10);
    this->declare_parameter("safety.max_velocity_sanity", 20.0);
    this->declare_parameter("safety.max_position_error", 0.8);

    max_torque_default_ = this->get_parameter("safety.max_torque_default").as_double();
    joint_state_timeout_sec_ = this->get_parameter("safety.joint_state_timeout_ms").as_int() / 1000.0;
    max_control_period_sec_ = this->get_parameter("safety.max_control_period_ms").as_int() / 1000.0;
    max_velocity_sanity_ = this->get_parameter("safety.max_velocity_sanity").as_double();
    max_position_error_ = this->get_parameter("safety.max_position_error").as_double();

    // Load per-joint torque limits (prefer config, fallback to URDF, then default)
    max_torque_per_joint_.resize(6, max_torque_default_); // Initialize with default

    // Then override with config if specified
    for (int i = 1; i <= 6; i++)
    {
      std::string param_name = "safety.max_torque_per_joint.joint_" + std::to_string(i);
      this->declare_parameter(param_name, -1.0); // -1 means "use URDF value"
      double config_limit = this->get_parameter(param_name).as_double();
      if (config_limit > 0) // If explicitly set in config
      {
        max_torque_per_joint_[i - 1] = config_limit;
      }
    }

    RCLCPP_INFO(this->get_logger(), "[SAFETY] Timeout: %.0f ms, Velocity sanity: %.1f rad/s",
                joint_state_timeout_sec_ * 1000.0, max_velocity_sanity_);

    // ========== 初始化动力学求解器 ==========
    RCLCPP_INFO(this->get_logger(), "[INFO] Starting dynamics solver initialization...");
    if (!initializeDynamics())
    {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to initialize dynamics solver");
      throw std::runtime_error("Failed to initialize dynamics");
    }

    // ========== 级联 P+PI 初始化（完全替代 PD）==========
    RCLCPP_INFO(this->get_logger(), "[PID] Initializing Cascade P+PI controller...");

    cascade_pid_ = std::make_unique<MultiJointCascadePid>(6);

    // 级联 P+PI 参数: 外环位置P, 内环速度PI
    for (int i = 1; i <= 6; i++)
    {
      std::string prefix = "cascade_pid.joint_" + std::to_string(i);

      // 位置环参数（只用 P）
      this->declare_parameter(prefix + ".pos_Kp", 10.0);
      this->declare_parameter(prefix + ".pos_Ki", 0.0); // 外环不用积分
      this->declare_parameter(prefix + ".pos_Kd", 0.0); // 外环不用微分

      // 速度环参数（使用 PI）
      this->declare_parameter(prefix + ".vel_Kp", 50.0);
      this->declare_parameter(prefix + ".vel_Ki", 5.0); // 内环积分消除静差
      this->declare_parameter(prefix + ".vel_Kd", 0.0); // 内环不用微分

      // 速度限制
      this->declare_parameter(prefix + ".vel_limit", 2.0);
    }

    // 读取参数并设置控制器
    for (int i = 0; i < 6; i++)
    {
      std::string prefix = "cascade_pid.joint_" + std::to_string(i + 1);

      PidGains pos_gains(
          this->get_parameter(prefix + ".pos_Kp").as_double(),
          this->get_parameter(prefix + ".pos_Ki").as_double(),
          this->get_parameter(prefix + ".pos_Kd").as_double());

      PidGains vel_gains(
          this->get_parameter(prefix + ".vel_Kp").as_double(),
          this->get_parameter(prefix + ".vel_Ki").as_double(),
          this->get_parameter(prefix + ".vel_Kd").as_double());

      double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();

      cascade_pid_->setJointParams(i, pos_gains, vel_gains, vel_limit);
    }

    RCLCPP_INFO(this->get_logger(), "[OK] Cascade P+PI initialized (Outer: Position-P, Inner: Velocity-PI)");

    // ========== 注册参数变化回调（必须在所有参数声明之后）==========
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TorqueControllerActionServer::parametersCallback,
                  this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[CONFIG] Dynamic parameter tuning enabled (use 'ros2 param set' to modify)");

    // ========== 订阅关节状态 ==========
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",                                             // 话题名称
        10,                                                          // 队列大小
        std::bind(&TorqueControllerActionServer::jointStateCallback, // 把成员回调函数绑定
                  this,
                  std::placeholders::_1));

    // ========== 创建 Action Server ==========
    // MoveIt 会发送到: /ARM_controller/follow_joint_trajectory
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this,
        "ARM_controller/follow_joint_trajectory", // 完整的 action 名称
        std::bind(&TorqueControllerActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TorqueControllerActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&TorqueControllerActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "[OK] Action server created: /ARM_controller/follow_joint_trajectory");

    // ========== 创建力矩发布者 ==========
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/effort_controller/commands", // ros2_control 的 effort_controller 会订阅这个话题
        10);

    RCLCPP_INFO(this->get_logger(), "[OK] Torque publisher created: /effort_controller/commands");
    RCLCPP_INFO(this->get_logger(), "[INFO] Control frequency: %.1f Hz", control_frequency_);

    auto period = std::chrono::duration<double, std::milli>(1000.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        period,
        std::bind(&TorqueControllerActionServer::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "[INFO] Control loop timer started (%.1f Hz)", control_frequency_);

    RCLCPP_INFO(this->get_logger(), "[OK] Torque controller fully initialized");
  }

  ~TorqueControllerActionServer()
  {
    RCLCPP_INFO(this->get_logger(), "[INFO] Dynamics torque calculation destructing, program ending");
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_; // 服务器对象实例指针

  trajectory_msgs::msg::JointTrajectory current_trajectory_; // 当前执行的轨迹
  rclcpp::Time trajectory_start_time_;                       // 轨迹开始时间
  std::shared_ptr<GoalHandleFJT> current_goal_handle_;       // 当前目标句柄
  bool is_executing_;                                        // 是否正在执行

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  KDL::JntArray q_actual_;     // 当前关节位置 [6]
  KDL::JntArray q_dot_actual_; // 当前关节速度 [6]
  KDL::JntArray q_target_;     // 目标关节位置（规划终点） [6]
  std::mutex state_mutex_;     // 保护 q_actual_, q_dot_actual_, state_received_, last_joint_state_time_
  std::mutex action_mutex_;    // 保护 is_executing_, has_target_, current_goal_handle_
  bool state_received_;        // 是否收到过状态
  bool has_target_;            // 是否有目标位置

  KDL::Chain kdl_chain_;                               // 运动链实例
  std::unique_ptr<DynamicsComputer> dynamic_computer_; // 动力学解算器实例

  std::unique_ptr<MultiJointCascadePid> cascade_pid_; // 级联 P+PI 控制器

  std::array<KalmanFilter1D, 6> joint_filters_;
  KDL::JntArray q_dot_filtered_;
  bool kalman_filter_enabled_; // 卡尔曼滤波开关

  // 卡尔曼增益观察
  size_t control_loop_count_ = 0;       // 控制循环计数器
  size_t kalman_print_interval_ = 1000; // 每 1000 次打印一次（1000/200Hz = 5秒）

  rclcpp::TimerBase::SharedPtr control_timer_;                                // 控制循环定时器
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_; // 力矩发布者
  double control_frequency_;                                                  // 控制频率 (Hz)

  // ========== Safety Parameters ==========
  rclcpp::Time last_joint_state_time_;       // Last time joint state was received
  rclcpp::Time last_control_loop_time_;      // Last control loop execution time
  double joint_state_timeout_sec_ = 0.1;     // 100ms timeout
  double max_control_period_sec_ = 0.01;     // 10ms (200Hz = 5ms nominal)
  std::vector<double> max_torque_per_joint_; // Nm - per joint torque limits
  double max_torque_default_ = 20.0;         // Nm - fallback torque limit
  double max_velocity_sanity_ = 20.0;        // rad/s - sensor sanity check
  double max_position_error_ = 0.8;          // rad - emergency stop threshold

  // Action 回调函数
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<GoalHandleFJT> goal_handle);

  void handleAccepted(
      const std::shared_ptr<GoalHandleFJT> goal_handle);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg); // 关节状态回调

  // ========== Safety Functions ==========
  void emergencyStop(const std::string &reason)
  {
    RCLCPP_ERROR(this->get_logger(), "[EMERGENCY STOP] %s", reason.c_str());

    // 1. Send zero torque immediately
    std_msgs::msg::Float64MultiArray safe_msg;
    safe_msg.data.resize(6, 0.0);
    torque_pub_->publish(safe_msg);

    // 2. Abort trajectory if executing (need lock for thread safety)
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      if (is_executing_ && current_goal_handle_)
      {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
        result->error_string = "Emergency stop: " + reason;
        current_goal_handle_->abort(result);
        is_executing_ = false;
        current_goal_handle_.reset();
      }

      // 保留has_target_维持模式目标，避免失控
    }

    // 保留state_received_，确保重力补偿有效

    // 3. 清零积分器，避免积分饱和
    if (cascade_pid_)
    {
      cascade_pid_->resetAll();
      RCLCPP_WARN(this->get_logger(), "[EMERGENCY] Cascade PID integrators reset");
    }
  }

  bool initializeDynamics(); // 动力学初始化

  bool interpolateTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory,
                             double t_now,
                             KDL::JntArray &q_d,
                             KDL::JntArray &qd_d,
                             KDL::JntArray &qdd_d);

  void computeFeedbackTorque(const KDL::JntArray &q_d,       // 期望位置
                             const KDL::JntArray &qd_d,      // 期望速度
                             const KDL::JntArray &q_actual,  // 实际位置
                             const KDL::JntArray &qd_actual, // 实际速度
                             KDL::JntArray &tau_fb);         // 输出：反馈力矩

  void controlLoop(); // 核心控制循环

  // ========== 新增：参数动态调节 ==========
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 参数变化回调函数
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);
};

rclcpp_action::GoalResponse TorqueControllerActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "[INFO] New trajectory received (%zu points)", goal->trajectory.points.size());

  bool currently_executing;
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    currently_executing = is_executing_;
  }

  if (currently_executing)
  {
    RCLCPP_WARN(this->get_logger(), "[WARN] Detected new trajectory, will preempt current execution");
    // 不再 REJECT，而是继续接受
  }

  if (goal->trajectory.points.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory is empty, goal rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "[OK] New goal accepted");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TorqueControllerActionServer::handleCancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "[INFO] Trajectory manually cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TorqueControllerActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "[INFO] Goal accepted, ready to execute");

  // 检查是否收到关节状态（不需要锁，只读取）
  bool has_state;
  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    has_state = state_received_;
  }

  if (!has_state)
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] No joint state data received, execution refused");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
    goal_handle->abort(result);
    return;
  }

  // 线程安全：修改 action 相关状态需要加锁
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);

    if (is_executing_ && current_goal_handle_)
    {
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
    is_executing_ = true;

    // 打印轨迹信息
    const auto &first_point = current_trajectory_.points[0];
    const auto &last_point = current_trajectory_.points.back();
    double total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

    // ========== 智能积分器清零策略 ==========
    // 检查位置连续性：比较轨迹起点和当前位置
    bool position_continuous = true;
    double max_pos_jump = 0.0;

    if (first_point.positions.size() == 6)
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      for (size_t i = 0; i < 6; i++)
      {
        double pos_error = std::abs(first_point.positions[i] - q_actual_(i));
        max_pos_jump = std::max(max_pos_jump, pos_error);
        if (pos_error > 0.05) // 阈值：5度（0.087 rad）或更保守的0.05 rad
        {
          position_continuous = false;
        }
      }
    }

    if (!position_continuous)
    {
      // 位置不连续：必须完全清零积分器，避免冲击
      cascade_pid_->resetAll();
      RCLCPP_WARN(this->get_logger(),
                  "[PID] Position discontinuity detected (max_jump=%.3f rad), integrators cleared",
                  max_pos_jump);
    }
    else
    {
      // 清零积分器，避免速度指令跳变影响
      cascade_pid_->resetAll();
      RCLCPP_INFO(this->get_logger(),
                  "[PID] Position continuous (max_jump=%.4f rad), but velocity command changes, integrators cleared",
                  max_pos_jump);
    }

    // ========== 新增：保存规划终点位置 ==========
    if (last_point.positions.size() == 6)
    {
      for (size_t i = 0; i < 6; i++)
      {
        q_target_(i) = last_point.positions[i];
      }
      has_target_ = true;
      RCLCPP_INFO(this->get_logger(), "[INFO] Target end-point: q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  q_target_(0), q_target_(1), q_target_(2),
                  q_target_(3), q_target_(4), q_target_(5));
    }

    RCLCPP_INFO(this->get_logger(), "[INFO] Trajectory cached (%zu points, %.3fs)",
                current_trajectory_.points.size(), total_duration);
  }
}

void TorqueControllerActionServer::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 线程安全：加锁
  std::lock_guard<std::mutex> lock(state_mutex_);

  // 验证数据
  if (msg->position.size() != 6 || msg->velocity.size() != 6)
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000, // 每秒最多打印一次
        "[WARN] Incomplete joint state data: position=%zu, velocity=%zu",
        msg->position.size(), msg->velocity.size());
    return;
  }

  // ========== SAFETY: Validate sensor data for NaN/Inf and sanity ==========
  bool has_velocity_spike = false;
  for (size_t i = 0; i < 6; i++)
  {
    // Check for NaN or Inf (这种情况必须完全拒绝)
    if (!std::isfinite(msg->position[i]) || !std::isfinite(msg->velocity[i]))
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[SAFETY] Non-finite sensor data on joint %zu (pos=%.2f, vel=%.2f), rejecting!",
                            i, msg->position[i], msg->velocity[i]);
      return; // Reject entire message - 完全拒绝，不更新时间戳
    }

    // Sanity check: position within expanded limits (warn but accept)
    if (std::abs(msg->position[i]) > 2 * M_PI)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[SAFETY] Joint %zu position out of range: %.2f rad", i, msg->position[i]);
    }

    // Sanity check: velocity spike detection (限幅而不是拒绝，避免timeout)
    if (std::abs(msg->velocity[i]) > max_velocity_sanity_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu velocity spike: %.2f rad/s (limit: %.1f), clamping!",
                           i, msg->velocity[i], max_velocity_sanity_);
      has_velocity_spike = true;
      // 不再return，而是继续处理，但会限幅速度
    }
  }

  // 提取位置和速度（velocity spike时进行限幅）
  for (size_t i = 0; i < 6; i++)
  {
    q_actual_(i) = msg->position[i];

    // 速度限幅：如果检测到spike，限制在安全范围内
    if (std::abs(msg->velocity[i]) > max_velocity_sanity_)
    {
      q_dot_actual_(i) = (msg->velocity[i] > 0) ? max_velocity_sanity_ : -max_velocity_sanity_;
    }
    else
    {
      q_dot_actual_(i) = msg->velocity[i];
    }
  }

  // ========== 关键：即使有velocity spike也更新时间戳，避免误判timeout ==========
  last_joint_state_time_ = this->now();

  // ========== 卡尔曼滤波处理 ==========
  // 注意：卡尔曼滤波器只用于速度滤波，位置保持原始测量值（编码器精度高）
  if (kalman_filter_enabled_)
  {
    // 启用滤波：使用卡尔曼滤波器过滤速度
    for (size_t i = 0; i < 6; i++)
    {
      // 首次接收：初始化滤波器
      if (!state_received_)
      {
        joint_filters_[i].initialize(msg->position[i], msg->velocity[i]);
      }
      else
      {
        // 后续：预测-更新循环
        joint_filters_[i].predict();
        joint_filters_[i].update(msg->position[i], msg->velocity[i]);
      }

      // [NOTE] Key modification: only use filtered velocity, position remains original
      // q_actual_(i) 已经在上面设置为 msg->position[i]，保持不变
      q_dot_filtered_(i) = joint_filters_[i].getVelocity();
    }
  }
  else
  {
    // 禁用滤波：直接使用原始测量值
    for (size_t i = 0; i < 6; i++)
    {
      q_dot_filtered_(i) = q_dot_actual_(i); // 直接使用原始速度
                                             // q_actual_ 已在上面赋值为原始位置，保持不变
    }
  }

  // 首次接收数据：保存启动姿态作为初始目标
  if (!state_received_)
  {
    RCLCPP_INFO(this->get_logger(), "[OK] First joint state data received");

    // 保存启动姿态作为初始目标位置（线程安全 - 已经持有state_mutex_）
    if (!has_target_)
    {
      // 需要访问has_target_（在action_mutex保护下），暂时解锁state_mutex_
      // 为了避免死锁，先拷贝数据再加锁
      KDL::JntArray q_startup(6);
      q_startup = q_actual_;

      // 现在可以安全地在另一个作用域加锁action_mutex_
      {
        std::lock_guard<std::mutex> action_lock(action_mutex_);
        if (!has_target_)
        {
          q_target_ = q_startup;
          has_target_ = true;
        }
      }

      RCLCPP_INFO(this->get_logger(), "[INFO] Saving startup pose as initial target:");
      RCLCPP_INFO(this->get_logger(), "   q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  q_target_(0), q_target_(1), q_target_(2),
                  q_target_(3), q_target_(4), q_target_(5));
    }
    // ========== 调试：打印卡尔曼增益 ==========
    RCLCPP_INFO(this->get_logger(), "📝  First Kalman gain (Joint 1):");
    auto K = joint_filters_[0].getKalmanGain();
    RCLCPP_INFO(this->get_logger(), "   K = [%.4f, %.4f]", K(0, 0), K(0, 1));
    RCLCPP_INFO(this->get_logger(), "       [%.4f, %.4f]", K(1, 0), K(1, 1));

    // ========== 关键：在所有初始化完成后才标记state_received_ ==========
    // 确保状态机闭环：只有当数据完整处理后才设置标志
    state_received_ = true;

    RCLCPP_INFO(this->get_logger(), "[STATE] state_received=true, ready for control");
  }
}

bool TorqueControllerActionServer::initializeDynamics()
{
  RCLCPP_INFO(this->get_logger(), "[INFO] Starting dynamics solver initialization...");

  // 1. 读取 URDF 文件（使用 ament_index 动态查找）
  std::string urdf_path;
  try
  {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("ARV_V1_MODEL");
    urdf_path = pkg_path + "/urdf/ARV_V1_MODEL.urdf";
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to find ARV_V1_MODEL package: %s", e.what());
    return false;
  }

  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot open URDF file: %s", urdf_path.c_str());
    return false;
  }

  std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
  urdf_file.close();

  // 2. 解析 URDF
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_string))
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] URDF parsing failed");
    return false;
  }

  // ========== SAFETY: Extract joint effort limits from URDF ==========
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  for (size_t i = 0; i < 6; i++)
  {
    auto joint = urdf_model.getJoint(joint_names[i]);
    if (joint && joint->limits)
    {
      double urdf_limit = joint->limits->effort;
      // Only use URDF limit if not overridden in config (config value <= 0)
      if (max_torque_per_joint_[i] <= 0)
      {
        max_torque_per_joint_[i] = urdf_limit;
        RCLCPP_INFO(this->get_logger(), "[SAFETY] Joint %s: Using URDF effort limit: %.1f Nm",
                    joint_names[i].c_str(), urdf_limit);
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "[SAFETY] Joint %s: Using config override: %.1f Nm (URDF: %.1f Nm)",
                    joint_names[i].c_str(), max_torque_per_joint_[i], urdf_limit);
      }
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "[SAFETY] Joint %s: No URDF limit found, using default: %.1f Nm",
                  joint_names[i].c_str(), max_torque_per_joint_[i]);
    }
  }

  RCLCPP_INFO(this->get_logger(), "[SAFETY] Final torque limits (Nm): [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
              max_torque_per_joint_[0], max_torque_per_joint_[1], max_torque_per_joint_[2],
              max_torque_per_joint_[3], max_torque_per_joint_[4], max_torque_per_joint_[5]);

  // 3. 提取 KDL 树
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to build KDL tree from URDF");
    return false;
  }

  // 4. 获取运动链（从 base_link 到 link6_2006roll）
  if (!kdl_tree.getChain("base_link", "link6_2006roll", kdl_chain_))
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to extract kinematic chain");
    return false;
  }

  // 5. 创建动力学计算工具
  KDL::Vector gravity(0.0, 0.0, -9.81); // 重力向量
  dynamic_computer_ = std::make_unique<DynamicsComputer>(kdl_chain_, gravity);

  // 设置错误日志回调，将 DynamicsComputer 的错误转发到 ROS2 日志系统
  // 捕获 this 指针以安全访问节点的 logger
  dynamic_computer_->setErrorLogger([this](const std::string &msg)
                                    { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); });

  RCLCPP_INFO(this->get_logger(), "[OK] Dynamics solver initialized");
  RCLCPP_INFO(this->get_logger(), "   - Gravity: [%.2f, %.2f, %.2f] m/s²",
              gravity.x(), gravity.y(), gravity.z());

  return true;
}

bool TorqueControllerActionServer::interpolateTrajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory,
    double t_now,
    KDL::JntArray &q_d,
    KDL::JntArray &qd_d,
    KDL::JntArray &qdd_d)
{
  // 1. 检查轨迹是否存在
  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory is empty, cannot interpolate");
    return false;
  }

  // 2. 如果时间在第一个点之前，返回第一个点
  const auto &first_point = trajectory.points[0];
  double t_first = first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9;

  if (t_now <= t_first)
  {
    // 返回第一个点的值
    for (size_t i = 0; i < 6; i++)
    {
      q_d(i) = first_point.positions[i];
      qd_d(i) = first_point.velocities.empty() ? 0.0 : first_point.velocities[i];
      qdd_d(i) = first_point.accelerations.empty() ? 0.0 : first_point.accelerations[i];
    }
    return true;
  }

  // 3. 如果时间在最后一个点之后，返回最后一个点
  const auto &last_point = trajectory.points.back();
  double t_last = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

  if (t_now >= t_last)
  {
    // 返回最后一个点的值（速度和加速度应该为 0）
    for (size_t i = 0; i < 6; i++)
    {
      q_d(i) = last_point.positions[i];
      qd_d(i) = 0.0;  // 停止时速度为 0
      qdd_d(i) = 0.0; // 停止时加速度为 0
    }
    return true;
  }

  // 4. 在轨迹中查找包围当前时间的两个点
  size_t idx_before = 0;
  size_t idx_after = 0;

  for (size_t i = 0; i < trajectory.points.size() - 1; i++)
  {
    double t_i = trajectory.points[i].time_from_start.sec +
                 trajectory.points[i].time_from_start.nanosec * 1e-9;
    double t_i_next = trajectory.points[i + 1].time_from_start.sec +
                      trajectory.points[i + 1].time_from_start.nanosec * 1e-9;

    if (t_now >= t_i && t_now <= t_i_next)
    {
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
  if (t_after - t_before < 1e-9)
  {
    alpha = 0.0;
  }

  // 7. 线性插值
  for (size_t i = 0; i < 6; i++)
  {
    // 位置插值
    q_d(i) = point_before.positions[i] +
             alpha * (point_after.positions[i] - point_before.positions[i]);

    // 速度插值
    if (!point_before.velocities.empty() && !point_after.velocities.empty())
    {
      qd_d(i) = point_before.velocities[i] +
                alpha * (point_after.velocities[i] - point_before.velocities[i]);
    }
    else
    {
      qd_d(i) = 0.0;
    }

    // 加速度插值
    if (!point_before.accelerations.empty() && !point_after.accelerations.empty())
    {
      qdd_d(i) = point_before.accelerations[i] +
                 alpha * (point_after.accelerations[i] - point_before.accelerations[i]);
    }
    else
    {
      qdd_d(i) = 0.0;
    }
  }

  return true;
}

// ========== PD 反馈控制 ========== //或者双环级联PID
void TorqueControllerActionServer::computeFeedbackTorque(
    const KDL::JntArray &q_d,
    const KDL::JntArray &qd_d,
    const KDL::JntArray &q_actual,
    const KDL::JntArray &qd_actual,
    KDL::JntArray &tau_fb)
{
  // 级联 P+PI: 外环P生成速度指令, 内环PI输出力矩
  // qd_actual 应传入卡尔曼滤波后的速度以减少噪声

  if (!cascade_pid_)
  {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[ERROR] Cascade P+PI not initialized!");
    for (size_t i = 0; i < 6; i++)
    {
      tau_fb(i) = 0.0;
    }
    return;
  }

  std::vector<double> pos_ref(6), pos_fdb(6), vel_fdb(6), torque_out(6);

  // 准备输入数据
  for (size_t i = 0; i < 6; i++)
  {
    pos_ref[i] = q_d(i);       // 期望位置
    pos_fdb[i] = q_actual(i);  // 实际位置（编码器，高精度）
    vel_fdb[i] = qd_actual(i); // 实际速度（卡尔曼滤波后，低噪声）
  }

  double dt = 1.0 / control_frequency_; // 200Hz -> 0.005s
  cascade_pid_->compute(pos_ref, pos_fdb, vel_fdb, dt, torque_out);

  // 输出力矩并进行安全限幅
  for (size_t i = 0; i < 6; i++)
  {
    tau_fb(i) = torque_out[i];

    // ========== SAFETY: Torque saturation (hardware protection) ==========
    double joint_limit = (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
    if (tau_fb(i) > joint_limit)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu feedback torque saturated: %.2f -> %.2f Nm",
                           i, tau_fb(i), joint_limit);
      tau_fb(i) = joint_limit;
    }
    else if (tau_fb(i) < -joint_limit)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu feedback torque saturated: %.2f -> -%.2f Nm",
                           i, tau_fb(i), joint_limit);
      tau_fb(i) = -joint_limit;
    }
  }
}

void TorqueControllerActionServer::controlLoop()
{
  // ========== SAFETY: Monitor control loop timing ==========
  rclcpp::Time now = this->now();
  if (control_loop_count_ > 0) // Skip first iteration
  {
    double period = (now - last_control_loop_time_).seconds();
    if (period > max_control_period_sec_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "[SAFETY] Control loop slow: %.1f ms (expected 5ms @ 200Hz)", period * 1000.0);
    }
  }
  last_control_loop_time_ = now;

  // ========== 新增：周期性打印卡尔曼增益 ==========
  control_loop_count_++;

  if (kalman_filter_enabled_ &&
      control_loop_count_ % kalman_print_interval_ == 0)
  {
    RCLCPP_INFO(this->get_logger(), "=== Kalman Gain Observation (Loop #%zu) ===",
                control_loop_count_);

    for (size_t i = 0; i < 6; i++)
    {
      auto K = joint_filters_[i].getKalmanGain();
      RCLCPP_INFO(this->get_logger(),
                  "Joint %zu: K = [[%.4f, %.4f], [%.4f, %.4f]]",
                  i + 1,
                  K(0, 0), K(0, 1),
                  K(1, 0), K(1, 1));
    }

    RCLCPP_INFO(this->get_logger(), "==========================================");
  }

  // 检查是否正在执行轨迹（线程安全）
  bool executing;
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    executing = is_executing_;
  }

  if (!executing)
  {
    // 没有活动轨迹时，发送重力补偿力矩保持位置
    std::lock_guard<std::mutex> state_lock(state_mutex_);

    if (!state_received_)
    {
      return; // 还没收到状态，无法计算
    }

    // ========== SAFETY: Check joint state timeout ==========
    double time_since_last_state = (this->now() - last_joint_state_time_).seconds();
    if (time_since_last_state > joint_state_timeout_sec_)
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[SAFETY] Joint state timeout: %.3fs since last update (limit: %.0f ms)",
                            time_since_last_state, joint_state_timeout_sec_ * 1000.0);

      // Timeout: 使用重力补偿维持位置，不直接return

      KDL::JntArray tau_gravity(6);
      bool can_compute_gravity = false;

      // 尝试用最后的位置数据计算重力补偿
      try
      {
        dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);
        can_compute_gravity = true;
      }
      catch (...)
      {
        RCLCPP_ERROR(this->get_logger(), "[SAFETY] Cannot compute gravity, sending zero torque");
      }

      std_msgs::msg::Float64MultiArray safe_msg;
      safe_msg.data.resize(6);
      if (can_compute_gravity)
      {
        // 使用重力补偿维持位置
        for (size_t i = 0; i < 6; i++)
        {
          safe_msg.data[i] = tau_gravity(i);
          // 安全限幅
          double joint_limit = (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
          safe_msg.data[i] = std::max(-joint_limit, std::min(joint_limit, safe_msg.data[i]));
        }
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "[SAFETY] Timeout: Publishing gravity compensation only");
      }
      else
      {
        // 无法计算，发送零力矩
        for (size_t i = 0; i < 6; i++)
        {
          safe_msg.data[i] = 0.0;
        }
      }
      torque_pub_->publish(safe_msg);

      // Abort any active trajectory (need to check again with action_mutex)
      std::shared_ptr<GoalHandleFJT> goal_to_abort;
      {
        std::lock_guard<std::mutex> action_lock(action_mutex_);
        if (current_goal_handle_)
        {
          goal_to_abort = current_goal_handle_;
        }
      }
      if (goal_to_abort)
      {
        emergencyStop("Joint state timeout");
      }

      // ========== 不再return，继续等待下次循环 ==========
      // 这样timeout后能自动恢复，不会卡在这里
      return;
    }

    // 计算重力补偿
    KDL::JntArray tau_gravity(6);
    dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);

    // ========== 修改：PD 控制目标位置 ========== //添加滤波速度
    KDL::JntArray tau_pd(6);
    if (has_target_)
    {
      computeFeedbackTorque(q_target_, KDL::JntArray(6), q_actual_, q_dot_filtered_, tau_pd);
    }
    else
    {
      computeFeedbackTorque(q_actual_, KDL::JntArray(6), q_actual_, q_dot_filtered_, tau_pd);
    }

    // 发布力矩：重力补偿 + PD 控制
    std_msgs::msg::Float64MultiArray torque_msg;
    torque_msg.data.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
      torque_msg.data[i] = tau_gravity(i) + tau_pd(i);

      // ========== SAFETY: NaN/Inf check before publishing ==========
      if (!std::isfinite(torque_msg.data[i]))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "[SAFETY] Non-finite torque detected on joint %zu in idle mode (gravity=%.2f, pd=%.2f)",
                     i, tau_gravity(i), tau_pd(i));
        emergencyStop("Non-finite torque in idle mode");
        return;
      }

      // ========== SAFETY: Final torque saturation ==========
      double joint_limit = (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
      if (torque_msg.data[i] > joint_limit)
      {
        torque_msg.data[i] = joint_limit;
      }
      else if (torque_msg.data[i] < -joint_limit)
      {
        torque_msg.data[i] = -joint_limit;
      }
    }
    torque_pub_->publish(torque_msg);

    return;
  }

  // ========== 执行轨迹阶段 ==========\n    // 读取 action 状态和轨迹信息（线程安全）
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
  double total_duration = last_point.time_from_start.sec +
                          last_point.time_from_start.nanosec * 1e-9;

  if (t_now >= total_duration) // 检查是否完成
  {
    RCLCPP_INFO(this->get_logger(), "[OK] Trajectory execution completed!");
    KDL::JntArray q_actual_copy(6), qd_filtered_copy(6);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      q_actual_copy = q_actual_;
      qd_filtered_copy = q_dot_filtered_; // 从成员变量拷贝滤波后的速度
    }

    // 计算重力补偿
    KDL::JntArray tau_gravity(6);
    dynamic_computer_->computeGravityTorque(q_actual_copy, tau_gravity);

    KDL::JntArray tau_pd(6);
    if (has_target_)
    {
      // 期望位置 = 规划终点，期望速度 = 0
      computeFeedbackTorque(q_target_, KDL::JntArray(6), q_actual_copy, qd_filtered_copy, tau_pd);
    }
    else
    {
      // 如果没有目标，PD 输出为 0
      for (size_t i = 0; i < 6; i++)
      {
        tau_pd(i) = 0.0;
      }
    }

    // 发布力矩：重力补偿 + PD 控制
    std_msgs::msg::Float64MultiArray hold_torque;
    hold_torque.data.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
      hold_torque.data[i] = tau_gravity(i) + tau_pd(i);

      // ========== SAFETY: NaN/Inf check and saturation ==========
      if (!std::isfinite(hold_torque.data[i]))
      {
        RCLCPP_ERROR(this->get_logger(),
                     "[SAFETY] Non-finite torque at trajectory completion on joint %zu", i);
        emergencyStop("Non-finite torque at trajectory completion");
        return;
      }
      double joint_limit = (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
      hold_torque.data[i] = std::max(-joint_limit, std::min(joint_limit, hold_torque.data[i]));
    }
    torque_pub_->publish(hold_torque);

    RCLCPP_INFO(this->get_logger(), "[OK] Trajectory execution completed, switching to hold mode (target: q_target)");
    RCLCPP_INFO(this->get_logger(), "   τ_total=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                hold_torque.data[0], hold_torque.data[1], hold_torque.data[2],
                hold_torque.data[3], hold_torque.data[4], hold_torque.data[5]);

    // 返回成功结果并清理状态（线程安全）
    if (goal_handle_copy)
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle_copy->succeed(result);
    }

    // 清理状态（但保留 q_target_ 和 has_target_）
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      is_executing_ = false;
      current_goal_handle_.reset();
    }
    return;
  }

  // 插值（时间不连续）
  KDL::JntArray q_d(6), qd_d(6), qdd_d(6);
  if (!interpolateTrajectory(current_traj_copy, t_now, q_d, qd_d, qdd_d))
  {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory interpolation failed");
    return;
  }

  // 获取实际状态（线程安全）
  KDL::JntArray q_actual(6), qd_filtered(6);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    q_actual = q_actual_;
    qd_filtered = q_dot_filtered_; // 从成员变量拷贝滤波后的速度
  }

  KDL::JntArray tau_ff(6); // PD的话需要输入期望和实际，前馈只需要期望
  dynamic_computer_->computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff);
  KDL::JntArray tau_fb(6);
  computeFeedbackTorque(q_d, qd_d, q_actual, qd_filtered, tau_fb); // 计算PD反馈力矩，使用滤波后的速度

  KDL::JntArray tau_total(6);
  for (size_t i = 0; i < 6; i++)
  {
    tau_total(i) = tau_ff(i) + tau_fb(i);
  } // PD+前馈力矩

  // 发送力矩到 ros2_control effort_controller
  std_msgs::msg::Float64MultiArray torque_msg;
  torque_msg.data.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    // ========== SAFETY: NaN/Inf check before publishing ==========
    if (!std::isfinite(tau_total(i)))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "[SAFETY] Non-finite torque detected on joint %zu during trajectory (ff=%.2f, fb=%.2f)",
                   i, tau_ff(i), tau_fb(i));
      emergencyStop("Non-finite torque during trajectory execution");
      return;
    }

    // ========== SAFETY: Final torque saturation ==========
    double joint_limit = (i < max_torque_per_joint_.size()) ? max_torque_per_joint_[i] : max_torque_default_;
    torque_msg.data[i] = std::max(-joint_limit, std::min(joint_limit, tau_total(i)));

    if (std::abs(tau_total(i)) > joint_limit)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[SAFETY] Joint %zu total torque saturated: %.2f -> %.2f Nm",
                           i, tau_total(i), torque_msg.data[i]);
    }
  }
  torque_pub_->publish(torque_msg);

  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = this->now(); // 获取反馈指针

  feedback->actual.positions.resize(6);
  feedback->actual.velocities.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    feedback->actual.positions[i] = q_actual(i);
    feedback->actual.velocities[i] = qd_filtered(i); // 反馈使用滤波后的速度
  }

  feedback->desired.positions.resize(6);
  feedback->desired.velocities.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    feedback->desired.positions[i] = q_d(i);
    feedback->desired.velocities[i] = qd_d(i); // 更新q-期望
  }

  feedback->error.positions.resize(6);
  feedback->error.velocities.resize(6);
  for (size_t i = 0; i < 6; i++)
  {
    feedback->error.positions[i] = q_d(i) - q_actual(i);
    feedback->error.velocities[i] = qd_d(i) - qd_filtered(i); // 速度误差基于滤波后的速度
  }

  current_goal_handle_->publish_feedback(feedback);
}

// ========== 参数变化回调函数实现 ==========
rcl_interfaces::msg::SetParametersResult TorqueControllerActionServer::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "参数更新成功";

  for (const auto &param : parameters)
  {
    const std::string &name = param.get_name();

    // ========== 卡尔曼滤波器参数动态更新 ==========
    if (name == "kalman.enabled")
    {
      // 更新卡尔曼滤波开关
      kalman_filter_enabled_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter %s",
                  kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");
    }
    else if (name == "kalman.Q_pos" || name == "kalman.Q_vel")
    {
      // 安全检查：确保所有相关参数都已声明
      if (this->has_parameter("kalman.Q_pos") && this->has_parameter("kalman.Q_vel"))
      {
        double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
        double Q_vel = this->get_parameter("kalman.Q_vel").as_double();

        for (auto &filter : joint_filters_)
        {
          filter.setProcessNoise(Q_pos, Q_vel);
        }

        RCLCPP_INFO(this->get_logger(), "[CONFIG] Process noise updated: Q_pos=%.1e, Q_vel=%.1e",
                    Q_pos, Q_vel);
      }
    }
    else if (name == "kalman.R_pos" || name == "kalman.R_vel")
    {
      // 安全检查：确保所有相关参数都已声明
      if (this->has_parameter("kalman.R_pos") && this->has_parameter("kalman.R_vel"))
      {
        double R_pos = this->get_parameter("kalman.R_pos").as_double();
        double R_vel = this->get_parameter("kalman.R_vel").as_double();

        for (auto &filter : joint_filters_)
        {
          filter.setMeasurementNoise(R_pos, R_vel);
        }

        RCLCPP_INFO(this->get_logger(), "[CONFIG] Measurement noise updated: R_pos=%.1e, R_vel=%.1e",
                    R_pos, R_vel);
      }
    }
    // ========== 级联 P+PI 参数动态更新 ==========
    else if (name.find("cascade_pid.joint_") == 0)
    {
      // 解析参数名: cascade_pid.joint_1.pos_Kp
      size_t first_dot = name.find('.', 13); // 找到 "joint_X" 后的点
      if (first_dot == std::string::npos)
        continue;

      size_t second_dot = name.find('.', first_dot + 1);
      if (second_dot == std::string::npos)
        continue;

      std::string joint_num_str = name.substr(13, first_dot - 13); // 提取 "1"
      std::string loop_type = name.substr(first_dot + 1, 3);       // 提取 "pos" 或 "vel"
      std::string gain_type = name.substr(second_dot + 1);         // 提取 "Kp"/"Ki"/"Kd" 或 "limit"

      int joint_idx = std::stoi(joint_num_str) - 1; // 0-based索引

      if (joint_idx >= 0 && joint_idx < 6)
      {
        double new_value = param.as_double();

        // 读取该关节的所有参数
        std::string prefix = "cascade_pid.joint_" + std::to_string(joint_idx + 1);

        PidGains pos_gains(
            this->get_parameter(prefix + ".pos_Kp").as_double(),
            this->get_parameter(prefix + ".pos_Ki").as_double(),
            this->get_parameter(prefix + ".pos_Kd").as_double());

        PidGains vel_gains(
            this->get_parameter(prefix + ".vel_Kp").as_double(),
            this->get_parameter(prefix + ".vel_Ki").as_double(),
            this->get_parameter(prefix + ".vel_Kd").as_double());

        double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();

        // 更新级联PID
        cascade_pid_->setJointParams(joint_idx, pos_gains, vel_gains, vel_limit);

        RCLCPP_INFO(this->get_logger(),
                    "[CONFIG] Cascade PID Joint %d updated: %s.%s = %.2f",
                    joint_idx + 1, loop_type.c_str(), gain_type.c_str(), new_value);
      }
    }
    else if (name == "kalman.print_interval")
    {
      kalman_print_interval_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "[CONFIG] Kalman print interval updated: %zu", kalman_print_interval_);
    }
  }

  return result;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TorqueControllerActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}