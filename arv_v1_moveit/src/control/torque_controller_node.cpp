/// @file torque_controller_node.cpp
/// @brief Torque controller: τ = M(q)q̈ + C(q,q̇) + G(q) + CascadePID

#include <urdf/model.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <climits>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <fstream>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <realtime_tools/realtime_publisher.hpp>  // RT non-blocking publish
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// --- Lightweight per-section loop profiler (no heap, no locks) ---
// Usage: record(us) each iteration, dump_and_reset() every N loops.
struct SectionStats {
  int64_t min_us{INT64_MAX}, max_us{0}, sum_us{0};
  uint64_t count{0};
  void record(int64_t us) {
    if (us < min_us) min_us = us;
    if (us > max_us) max_us = us;
    sum_us += us;
    ++count;
  }
  int64_t avg_us() const { return count ? sum_us / static_cast<int64_t>(count) : 0; }
  void reset() {
    min_us = INT64_MAX;
    max_us = 0;
    sum_us = 0;
    count = 0;
  }
};

struct LoopProfiler {
  SectionStats total;       // full controlLoop() wall time
  SectionStats gap;         // interval between controlLoop() entries
  SectionStats joint_age;   // latest /joint_states age when available
  SectionStats state_lock;  // time spent waiting on state_mutex_
  SectionStats dynamics;    // KDL dynamics computation
  SectionStats publish;     // safeTorquePublish
  uint64_t overruns{0};     // loops where total > overrun_threshold_us
  int64_t overrun_threshold_us{1500};

  // Call at end of each loop iteration.
  // Returns true when it is time to print statistics (every print_interval calls).
  bool record_and_check(int64_t t_us, int64_t gap_us, int64_t joint_age_us, int64_t lock_us,
                        int64_t dyn_us, int64_t pub_us, uint64_t print_interval = 1000) {
    total.record(t_us);
    gap.record(gap_us);
    if (joint_age_us >= 0) joint_age.record(joint_age_us);
    state_lock.record(lock_us);
    dynamics.record(dyn_us);
    publish.record(pub_us);
    if (t_us > overrun_threshold_us) ++overruns;
    return (total.count % print_interval) == 0;
  }

  void reset_all() {
    total.reset();
    gap.reset();
    joint_age.reset();
    state_lock.reset();
    dynamics.reset();
    publish.reset();
  }
};

#include "arv_v1_interfaces/srv/gripper_control.hpp"
#include "cascade_pid.hpp"
#include "dynamics_computer.hpp"
#include "impedance_controller.hpp"
#include "kalman_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// 控制模式常量 (与 serial_protocol.hpp ControlMode 保持同步)
namespace ControlMode {
constexpr uint8_t RELAX = 0;      // 全零力矩
constexpr uint8_t FREEDRIVE = 1;  // 仅重力补偿
constexpr uint8_t ARMED = 2;      // 就绪: G(q)+PD 保持, 可接受轨迹执行
}  // namespace ControlMode

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class TorqueControllerActionServer : public rclcpp::Node {
  static constexpr int kArmJoints = 6;  // 6-DOF arm
  static constexpr int kAllJoints = 7;  // 6-DOF arm + 1 gripper
public:
  TorqueControllerActionServer()
      : Node("torque_controller_action_server"),
        is_executing_(false),
        q_actual_(6),
        q_dot_actual_(6),
        q_target_(6),
        state_received_(false),
        has_target_(false),
        control_frequency_(1000.0),
        joint_filters_{
            KalmanFilter1D(1.0 / 1000.0),  // Joint 1
            KalmanFilter1D(1.0 / 1000.0),  // Joint 2
            KalmanFilter1D(1.0 / 1000.0),  // Joint 3
            KalmanFilter1D(1.0 / 1000.0),  // Joint 4
            KalmanFilter1D(1.0 / 1000.0),  // Joint 5
            KalmanFilter1D(1.0 / 1000.0)   // Joint 6
        },
        kalman_filter_enabled_(false),
        q_dot_filtered_(6) {
    RCLCPP_INFO(this->get_logger(),
                "[START] Torque controller node starting (Cascade P+PI Control)");

    // --- route_mode: 控制律 bypass, 仅路由 q_target ---
    this->declare_parameter("route_mode", false);
    route_mode_ = this->get_parameter("route_mode").as_bool();
    RCLCPP_INFO(this->get_logger(), "[ROUTE_MODE] %s — control law %s, cmd_topic data[0..5] = %s",
                route_mode_.load() ? "ENABLED" : "DISABLED",
                route_mode_.load() ? "BYPASSED" : "ACTIVE",
                route_mode_.load() ? "q_target(rad)" : "tau(Nm)");

    this->declare_parameter("kalman.enabled", false);

    kalman_filter_enabled_ = this->get_parameter("kalman.enabled").as_bool();

    RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter state: %s",
                kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");

    this->declare_parameter("kalman.Q_pos", 1e-5);
    this->declare_parameter("kalman.Q_vel", 1e-4);
    this->declare_parameter("kalman.R_pos", 1e-3);
    this->declare_parameter("kalman.R_vel", 2.5e-2);

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
    max_torque_per_joint_.resize(6, max_torque_default_);
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
      gripper_max_force_ = gripper_limit;
    }

    RCLCPP_INFO(this->get_logger(),
                "[SAFETY] Timeout: %.0f ms, Velocity sanity: %.1f rad/s, Gripper limit: %.1f N",
                joint_state_timeout_sec_ * 1000.0, max_velocity_sanity_, gripper_max_force_);

    this->declare_parameter("payload.enabled", false);
    this->declare_parameter("payload.mass", 0.447);
    this->declare_parameter("payload.gripper_force_threshold", 0.5);
    this->declare_parameter("payload.gripper_pos_threshold", 0.01);
    payload_compensation_enabled_ = this->get_parameter("payload.enabled").as_bool();
    payload_mass_ = this->get_parameter("payload.mass").as_double();
    payload_force_threshold_ = this->get_parameter("payload.gripper_force_threshold").as_double();
    payload_pos_threshold_ = this->get_parameter("payload.gripper_pos_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "[PAYLOAD] %s (mass=%.3f kg, force_thr=%.1f N, pos_thr=%.3f m)",
                payload_compensation_enabled_ ? "ENABLED" : "DISABLED", payload_mass_,
                payload_force_threshold_, payload_pos_threshold_);

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

      this->declare_parameter(prefix + ".pos_Kp", 10.0);
      this->declare_parameter(prefix + ".pos_Ki", 0.0);
      this->declare_parameter(prefix + ".pos_Kd", 0.0);
      this->declare_parameter(prefix + ".max_integral_pos", 2.0);

      this->declare_parameter(prefix + ".vel_Kp", 50.0);
      this->declare_parameter(prefix + ".vel_Ki", 5.0);
      this->declare_parameter(prefix + ".vel_Kd", 0.0);

      this->declare_parameter(prefix + ".vel_limit", 2.0);
    }

    for (int i = 0; i < 6; i++) {
      std::string prefix = "cascade_pid.joint_" + std::to_string(i + 1);

      PidGains pos_gains(this->get_parameter(prefix + ".pos_Kp").as_double(),
                         this->get_parameter(prefix + ".pos_Ki").as_double(),
                         this->get_parameter(prefix + ".pos_Kd").as_double());

      PidGains vel_gains(this->get_parameter(prefix + ".vel_Kp").as_double(),
                         this->get_parameter(prefix + ".vel_Ki").as_double(),
                         this->get_parameter(prefix + ".vel_Kd").as_double());

      double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();
      double max_integral_pos = this->get_parameter(prefix + ".max_integral_pos").as_double();

      cascade_pid_->setJointParams(i, pos_gains, vel_gains, vel_limit, max_integral_pos);
    }

    cascade_pid_->setJointContinuous(5, true);
    cascade_pid_->getJointController(1).setDerivativeMode(2, true);  // J2: 2ms采样+/dt, 同MCU
    cascade_pid_->getJointController(3).setDerivativeMode(2, true);  // J4: 2ms采样+/dt, 同MCU

    RCLCPP_INFO(
        this->get_logger(),
        "[OK] Cascade PI+PI initialized (Outer: Position-PI w/ anti-windup, Inner: Velocity-PI)");

    // --- 阻抗控制初始化 (J1/J2/J5) ---
    impedance_controller_ = std::make_unique<MultiJointImpedance>(kArmJoints);
    {
      auto load_imp = [&](size_t idx, const std::string &joint_name) {
        std::string prefix = "impedance." + joint_name;
        this->declare_parameter(prefix + ".K", 0.0);
        this->declare_parameter(prefix + ".D", 0.0);
        this->declare_parameter(prefix + ".tau_limit", 0.0);
        ImpedanceGains g;
        g.K = this->get_parameter(prefix + ".K").as_double();
        g.D = this->get_parameter(prefix + ".D").as_double();
        g.tau_limit = this->get_parameter(prefix + ".tau_limit").as_double();
        impedance_controller_->setJointGains(idx, g);
        RCLCPP_INFO(this->get_logger(), "[IMP] %s: K=%.1f D=%.2f limit=%.1f", joint_name.c_str(),
                    g.K, g.D, g.tau_limit);
      };
      load_imp(0, "joint_1");
      load_imp(1, "joint_2");
      load_imp(4, "joint_5");
    }
    RCLCPP_INFO(this->get_logger(), "[OK] Impedance controller initialized (J1/J2/J5)");

    // --- 摩擦前馈参数 ---
    this->declare_parameter("friction.coulomb", std::vector<double>(kArmJoints, 0.0));
    this->declare_parameter("friction.viscous", std::vector<double>(kArmJoints, 0.0));
    this->declare_parameter("friction.fal_alpha", 0.5);
    this->declare_parameter("friction.fal_delta", 0.05);
    friction_coulomb_ = this->get_parameter("friction.coulomb").as_double_array();
    friction_viscous_ = this->get_parameter("friction.viscous").as_double_array();
    fal_alpha_ = this->get_parameter("friction.fal_alpha").as_double();
    fal_delta_ = this->get_parameter("friction.fal_delta").as_double();
    RCLCPP_INFO(this->get_logger(),
                "[OK] Friction feedforward: fc=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] fal(α=%.2f,δ=%.3f)",
                friction_coulomb_[0], friction_coulomb_[1], friction_coulomb_[2],
                friction_coulomb_[3], friction_coulomb_[4], friction_coulomb_[5], fal_alpha_,
                fal_delta_);

    // --- 注册参数变化回调（必须在所有参数声明之后）---
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TorqueControllerActionServer::parametersCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[CONFIG] Dynamic parameter tuning enabled (use 'ros2 param set' to modify)");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&TorqueControllerActionServer::jointStateCallback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
        this, "ARM_controller/follow_joint_trajectory",
        std::bind(&TorqueControllerActionServer::handleGoal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&TorqueControllerActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&TorqueControllerActionServer::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "[OK] Action server created: /ARM_controller/follow_joint_trajectory");

    cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/joint_position_target_to_mcu", rclcpp::SensorDataQoS());
    rt_cmd_pub_ =
        std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
            cmd_pub_);

    RCLCPP_INFO(this->get_logger(),
                "[OK] Command publisher created: /joint_position_target_to_mcu "
                "(RT, BestEffort; data[0..5]=q_target rad, data[6]=gripper signal)");

    gripper_service_ = this->create_service<arv_v1_interfaces::srv::GripperControl>(
        "/gripper_control", std::bind(&TorqueControllerActionServer::gripperControlCallback, this,
                                      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "[OK] Gripper control service created: /gripper_control");

    trajectory_forward_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/ARM_controller/joint_trajectory", 10);
    RCLCPP_INFO(this->get_logger(),
                "[OK] Trajectory forward publisher created: /ARM_controller/joint_trajectory");

    control_mode_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/control_mode", 10,
        std::bind(&TorqueControllerActionServer::controlModeCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(),
                "[OK] Control mode subscriber created: /control_mode (default: RELAX)");

    joint_target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/joint_position_target", rclcpp::SensorDataQoS(),
        std::bind(&TorqueControllerActionServer::jointTargetCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "[OK] Joint target subscriber created: /joint_position_target");

    RCLCPP_INFO(this->get_logger(), "[INFO] Control frequency: %.1f Hz", control_frequency_);

    auto period = std::chrono::duration<double, std::milli>(1000.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        period, std::bind(&TorqueControllerActionServer::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "[INFO] Control loop timer started (%.1f Hz)",
                control_frequency_);

    // 预热消息内存，避免首次 publish 触发 malloc
    torque_msg_preallocated_.data.resize(kAllJoints + 1, 0.0);  // +1 for seq field
    zero_msg_preallocated_.data.resize(kAllJoints + 1, 0.0);

    RCLCPP_INFO(this->get_logger(), "[OK] Torque controller fully initialized (mode: RELAX)");
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
  std::atomic<bool> is_executing_;  // 避免竞态 (controlLoop读, handleAccepted写)

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  KDL::JntArray q_actual_, q_dot_actual_, q_target_;
  std::mutex state_mutex_;   // 保护状态变量
  std::mutex action_mutex_;  // 保护执行标志和目标句柄
  std::mutex filter_mutex_;  // 保护滤波器
  bool state_received_, has_target_;

  KDL::Chain kdl_chain_;
  std::unique_ptr<DynamicsComputer> dynamic_computer_;
  std::unique_ptr<MultiJointCascadePid> cascade_pid_;
  std::unique_ptr<MultiJointImpedance> impedance_controller_;
  static constexpr bool kUseImpedance_[kArmJoints] = {true, false, false, false, true, false};

  std::array<KalmanFilter1D, 6> joint_filters_;
  KDL::JntArray q_dot_filtered_;
  bool kalman_filter_enabled_;

  // route_mode: 控制律 bypass, q_target 直接转发到 effort topic (语义改成 q_target rad)
  // MCU 端: 收到 0x0002 后用 data[0..5] 作为 q_target 跑本地 PID+阻抗
  std::atomic<bool> route_mode_{false};

  size_t control_loop_count_ = 0;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> rt_cmd_pub_;
  LoopProfiler loop_prof_;  // timing instrumentation
  std::chrono::steady_clock::time_point last_control_loop_entry_;
  std::atomic<uint64_t> torque_publish_count_{0};
  uint64_t last_timing_publish_count_{0};
  std::atomic<uint64_t> control_slow_count_{0};
  std::atomic<uint64_t> control_overrun_count_{0};
  std::atomic<uint64_t> control_overrun_max_us_{0};
  std::atomic<uint64_t> control_overrun_max_gap_us_{0};
  std::atomic<uint64_t> control_overrun_max_js_age_us_{0};
  std::atomic<uint64_t> joint_state_msg_count_{0};
  std::atomic<uint64_t> joint_state_invalid_size_count_{0};
  std::atomic<uint64_t> joint_state_nonfinite_count_{0};
  std::atomic<uint64_t> joint_state_pos_range_count_{0};
  std::atomic<uint64_t> joint_state_velocity_spike_count_{0};
  std::atomic<uint64_t> joint_state_timeout_count_{0};
  std::atomic<uint64_t> torque_saturation_count_{0};
  std::atomic<uint64_t> torque_saturation_max_milli_{0};
  std::atomic<uint64_t> torque_nonfinite_count_{0};
  // Priority3: torque 序列号 (每次 safeTorquePublish 递增, uint8 回卷)
  // hardware 侧用来判断 torque_rx 的重复 callback vs 真实发布
  std::atomic<uint8_t> torque_seq_{0};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_forward_pub_;
  double control_frequency_;

  // 安全参数
  rclcpp::Time last_joint_state_time_, last_control_loop_time_;
  double joint_state_timeout_sec_ = 0.1;  // 100ms
  double max_control_period_sec_ = 0.01;  // 10ms
  std::vector<double> max_torque_per_joint_;
  double max_torque_default_ = 20.0;
  double max_velocity_sanity_ = 20.0;
  double max_position_error_ = 0.8;

  // 降级: 复用上次有效力矩，避免自由落体
  std::array<double, 7> last_valid_torque_{};

  // 预分配发布消息，避免热路径 malloc
  // data[0..5]=臂力矩 data[6]=夹爪 data[7]=seq(uint8转db)
  std_msgs::msg::Float64MultiArray torque_msg_preallocated_;
  std_msgs::msg::Float64MultiArray zero_msg_preallocated_;

  // 控制模式 (由 mission_executor 通过 /control_mode topic 管理)
  std::atomic<uint8_t> control_mode_{ControlMode::RELAX};  // 默认 RELAX: 上电安全
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr control_mode_sub_;

  // 关节位置目标 (由 cartesian_controller IK 伺服输出, ARMED 模式非执行时更新 q_target_)
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_sub_;

  // 夹爪控制 (prismatic joint, 单位: N)
  std::mutex gripper_mutex_;
  double gripper_force_cmd_ = 0.0;   // 当前夹爪力指令 (N)
  double gripper_max_force_ = 70.0;  // 夹爪力限幅 (N), 从yaml覆盖
  rclcpp::Service<arv_v1_interfaces::srv::GripperControl>::SharedPtr gripper_service_;

  // --- 载荷重力补偿: τ_payload = J(q)^T × [0,0,-mg,0,0,0] ---
  KDL::Chain kdl_chain_tcp_;  // base_link→tcp (含 joint_tcp 偏移 0.179m)
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;  // 雅可比求解器 (预分配)
  KDL::Jacobian jacobian_{kArmJoints};                    // 预分配 6×6 雅可比
  Eigen::Matrix<double, 6, 1> payload_wrench_;            // 预分配力螺旋
  KDL::JntArray tau_payload_{kArmJoints};                 // 预分配载荷力矩输出
  double payload_mass_ = 0.0;
  double payload_force_threshold_ = 0.5;  // N, 夹力阈值
  double payload_pos_threshold_ = 0.01;   // m, 夹爪位置阈值 (10mm)
  bool payload_compensation_enabled_ = false;
  double gripper_position_ =
      0.0;  // 夹爪位置 (state_mutex_ 保护, jointStateCallback写/controlLoop读)

  void computePayloadCompensation(const KDL::JntArray &q, double gripper_pos, double gripper_force,
                                  KDL::JntArray &tau_out);

  // --- 摩擦前馈: τ_friction = fc·fal(q̇,α,δ) + fv·q̇ ---
  // fal 平滑库仑摩擦 sign() 跳变，避免零速极限环
  std::vector<double> friction_coulomb_{std::vector<double>(kArmJoints, 0.0)};
  std::vector<double> friction_viscous_{std::vector<double>(kArmJoints, 0.0)};
  double fal_alpha_ = 0.5;   // 非线性指数 (0<α<1)
  double fal_delta_ = 0.05;  // 线性区宽度 (rad/s)

  static double fal(double e, double alpha, double delta) {
    if (delta < 1e-10) delta = 1e-10;
    if (std::abs(e) <= delta)
      return e / std::pow(delta, 1.0 - alpha);
    else
      return std::pow(std::abs(e), alpha) * std::copysign(1.0, e);
  }

  void updateMaxAtomic(std::atomic<uint64_t> &target, uint64_t value) {
    uint64_t old = target.load(std::memory_order_relaxed);
    while (old < value && !target.compare_exchange_weak(old, value, std::memory_order_relaxed)) {
    }
  }

  double computeFrictionTorque(int joint_idx, double velocity) {
    return friction_coulomb_[joint_idx] * fal(velocity, fal_alpha_, fal_delta_) +
           friction_viscous_[joint_idx] * velocity;
  }

  /// NaN check + per-joint saturation + publish + fallback backup.
  /// Returns false if NaN detected (emergencyStop triggered).
  bool safeTorquePublish(const KDL::JntArray &tau_arm, double gripper_force);

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid,
                                         std::shared_ptr<const FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  void emergencyStop(const std::string &reason) {
    RCLCPP_ERROR(this->get_logger(), "[EMERGENCY STOP] %s", reason.c_str());

    std_msgs::msg::Float64MultiArray safe_msg;
    safe_msg.data.resize(kAllJoints, 0.0);
    cmd_pub_->publish(safe_msg);
    torque_publish_count_++;

    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      if (is_executing_.load(std::memory_order_acquire) && current_goal_handle_) {
        executionRecoveryCeremony(reason);
      }
    }
  }

  // Full recovery: clear trajectory, abort goal, reset PID+Kalman, send G(q)
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
    if (impedance_controller_) impedance_controller_->resetAll();

    // 锁顺序: state_mutex_ → filter_mutex_ (全局统一，避免死锁)
    if (kalman_filter_enabled_) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      std::lock_guard<std::mutex> filter_lock(filter_mutex_);
      for (size_t i = 0; i < 6; i++) {
        joint_filters_[i].initialize(q_actual_(i), q_dot_actual_(i));
      }
    }

    // [FIX] recovery → snap q_target_ to actual, publish G(q) once
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (state_received_) {
        q_target_ = q_actual_;
        has_target_ = true;
        KDL::JntArray tau_gravity(kArmJoints);
        dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);
        safeTorquePublish(tau_gravity, 0.0);
      }
    }

    RCLCPP_INFO(this->get_logger(), "[RECOVERY] Complete - Ready for new goals");
  }

  bool initializeDynamics();

  bool interpolateTrajectory(const trajectory_msgs::msg::JointTrajectory &trajectory, double t_now,
                             KDL::JntArray &q_d, KDL::JntArray &qd_d, KDL::JntArray &qdd_d);

  void computeFeedbackTorque(const KDL::JntArray &q_d, const KDL::JntArray &qd_d,
                             const KDL::JntArray &q_actual, const KDL::JntArray &qd_actual,
                             KDL::JntArray &tau_fb);

  void controlLoop();

  // route_mode 主路径: 仅路由 q_target 到 hardware (复用 effort topic, 语义变 q_target rad)
  // 来源优先级: trajectory action (TODO Phase 2) → /joint_position_target → q_actual fallback
  void routeTargetToHardware();

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter> &parameters);

  void gripperControlCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::GripperControl::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::GripperControl::Response> response) {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    double clamped = std::clamp(request->force, -gripper_max_force_, gripper_max_force_);
    gripper_force_cmd_ = clamped;
    response->success = true;
    response->message = "Gripper force set to " + std::to_string(clamped) + " N";
    RCLCPP_INFO(this->get_logger(), "[GRIPPER] Force command: %.3f N", clamped);
  }

  void controlModeCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
    uint8_t new_mode = msg->data;
    uint8_t old_mode = control_mode_.load(std::memory_order_acquire);
    if (new_mode == old_mode) return;
    if (new_mode > ControlMode::ARMED) {
      RCLCPP_ERROR(get_logger(), "[MODE] Invalid mode value: %u, ignoring", new_mode);
      return;
    }

    // --- Lock ordering: action_mutex_ → state_mutex_ → filter_mutex_  (DO NOT REORDER) ---
    // 各域独立获取, 不嵌套, 消除死锁可能. PID/impedance 仅本回调和 controlLoop timer
    // 交替访问, resetAll 自身线程安全, 无需上锁.
    if (is_executing_.load(std::memory_order_acquire) && new_mode != ControlMode::ARMED) {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      if (is_executing_.load(std::memory_order_acquire)) {  // double-check under lock
        executionRecoveryCeremony("Control mode changed during execution");
      }
    }

    if (new_mode == ControlMode::ARMED) {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      if (state_received_) {
        q_target_ = q_actual_;
        has_target_ = true;
      }
    }

    if (cascade_pid_) cascade_pid_->resetAll();
    if (impedance_controller_) impedance_controller_->resetAll();

    control_mode_.store(new_mode, std::memory_order_release);

    static const char *mode_names[] = {"RELAX", "FREEDRIVE", "ARMED"};
    RCLCPP_WARN(get_logger(), "[MODE] %s -> %s", mode_names[old_mode], mode_names[new_mode]);
  }

  void jointTargetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (static_cast<int>(msg->data.size()) < kArmJoints) return;

    // 只在 ARMED 模式 + 非轨迹执行时接受
    if (control_mode_.load(std::memory_order_acquire) != ControlMode::ARMED) return;
    if (is_executing_.load(std::memory_order_acquire)) return;

    // NaN/Inf 全量校验
    for (int i = 0; i < kArmJoints; i++) {
      if (!std::isfinite(msg->data[i])) return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < kArmJoints; i++) {
      q_target_(i) = msg->data[i];
    }
    has_target_ = true;
  }
};

rclcpp_action::GoalResponse TorqueControllerActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal) {
  (void)uuid;

  RCLCPP_INFO(this->get_logger(), "[INFO] New trajectory received (%zu points)",
              goal->trajectory.points.size());

  // 模式检查: 只有 ARMED 模式下才允许执行轨迹
  uint8_t mode = control_mode_.load(std::memory_order_acquire);
  if (mode != ControlMode::ARMED) {
    static const char *mode_names[] = {"RELAX", "FREEDRIVE", "ARMED"};
    RCLCPP_WARN(this->get_logger(),
                "[REJECT] Cannot execute trajectory in %s mode, switch to ARMED first",
                mode <= ControlMode::ARMED ? mode_names[mode] : "UNKNOWN");
    return rclcpp_action::GoalResponse::REJECT;
  }

  bool currently_executing;
  {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    currently_executing = is_executing_.load(std::memory_order_acquire);
  }

  if (currently_executing) {
    RCLCPP_WARN(this->get_logger(),
                "[WARN] Detected new trajectory, will preempt current execution");
    // 不再 REJECT，而是继续接受，轨迹应当接受抢占式
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

    const auto goal = goal_handle->get_goal();
    current_trajectory_ = goal->trajectory;
    current_goal_handle_ = goal_handle;
    trajectory_start_time_ = this->now();
    is_executing_.store(true, std::memory_order_release);

    // 转发轨迹到话题，供 trajectory_manager_node 捕获保存
    trajectory_forward_pub_->publish(current_trajectory_);
    RCLCPP_DEBUG(this->get_logger(),
                 "[FORWARD] Trajectory published to /ARM_controller/joint_trajectory");

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
      if (impedance_controller_) impedance_controller_->resetAll();
      RCLCPP_WARN(this->get_logger(),
                  "[PID] Position discontinuity detected (max_jump=%.3f rad), integrators cleared",
                  max_pos_jump);
    } else {
      // 清零积分器，避免速度指令跳变影响
      cascade_pid_->resetAll();
      if (impedance_controller_) impedance_controller_->resetAll();
      RCLCPP_INFO(this->get_logger(),
                  "[PID] Position continuous (max_jump=%.4f rad), but velocity command changes, "
                  "integrators cleared",
                  max_pos_jump);
    }

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
  joint_state_msg_count_++;

  if (msg->position.size() < kAllJoints || msg->velocity.size() < kAllJoints) {
    joint_state_invalid_size_count_++;
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[ERROR] Invalid joint state size! Expected at least %d", kAllJoints);
    return;
  }

  // Validate before taking state_mutex_ so diagnostics cannot hold the state lock.
  bool has_velocity_spike = false;
  for (size_t i = 0; i < kArmJoints; i++) {
    if (!std::isfinite(msg->position[i]) || !std::isfinite(msg->velocity[i])) {
      joint_state_nonfinite_count_++;
      RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "[SAFETY] Non-finite sensor data on joint %zu (pos=%.2f, vel=%.2f), rejecting!", i + 1,
          msg->position[i], msg->velocity[i]);
      return;  // Reject entire message - 完全拒绝，不更新时间戳
    }

    if (std::abs(msg->position[i]) > 2 * M_PI) {
      joint_state_pos_range_count_++;
    }

    if (std::abs(msg->velocity[i]) > max_velocity_sanity_) {
      joint_state_velocity_spike_count_++;
      has_velocity_spike = true;
    }
  }

  // --- 修复死锁 #1: 使用unique_lock手动控制锁生命周期 ---
  std::unique_lock<std::mutex> state_lock(state_mutex_);

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
  gripper_position_ = msg->position[kArmJoints];  // 夹爪位置 (index 6, m)

  // --- 关键：即使有velocity spike也更新时间戳，避免误判timeout ---
  last_joint_state_time_ = this->now();

  // --- 修复数据竞争 #2: Kalman滤波处理 (添加filter_mutex_保护) ---
  // 注意：卡尔曼滤波器只用于速度滤波，位置保持原始测量值（编码器精度高）
  if (kalman_filter_enabled_) {
    // 获取 filter_mutex_ 保护 joint_filters_ 的访问
    std::lock_guard<std::mutex> filter_lock(filter_mutex_);

    // 启用滤波：使用卡尔曼滤波器过滤速度
    for (size_t i = 0; i < kArmJoints; i++) {
      // --- Filter enabled: first sample initializes, subsequent run predict+update ---
      // 仅取 filtered 的 velocity, position 保留原始 (q_actual_ 已在 state_mutex_ 下赋值).
      if (!state_received_) {
        joint_filters_[i].initialize(msg->position[i], msg->velocity[i]);
      } else {
        joint_filters_[i].predict();
        joint_filters_[i].update(msg->position[i], msg->velocity[i]);
      }
      q_dot_filtered_(i) = joint_filters_[i].getVelocity();
    }
  }  // filter_mutex_ 自动释放
  else {
    // --- Filter disabled: pass-through raw velocity ---
    for (size_t i = 0; i < kArmJoints; i++) {
      q_dot_filtered_(i) = q_dot_actual_(i);
    }
  }

  // --- First-sample bootstrap: copy q_startup under state_mutex_, then promote state_received_ ---
  // state_received_ 必须在所有初始化都跑完后才置 true, 确保 controlLoop 看到 true 时数据完整可用.
  // 锁顺序: state_mutex_ 仍持有, 后续 filter_mutex_ 独立获取 (避免嵌套).
  bool is_first_state = !state_received_;
  KDL::JntArray q_startup(kArmJoints);

  if (is_first_state) {
    RCLCPP_INFO(this->get_logger(), "[OK] First joint state data received");
    q_startup = q_actual_;
    state_received_ = true;
    RCLCPP_INFO(this->get_logger(), "[STATE] state_received=true, ready for control");
  }

  if (is_first_state && kalman_filter_enabled_) {
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

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to build KDL tree from URDF");
    return false;
  }

  if (!kdl_tree.getChain("base_link", "link6", kdl_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to extract kinematic chain");
    return false;
  }

  KDL::Vector gravity(0.0, 0.0, -9.81);
  dynamic_computer_ = std::make_unique<DynamicsComputer>(kdl_chain_, gravity);

  dynamic_computer_->setErrorLogger(
      [this](const std::string &msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); });

  RCLCPP_INFO(this->get_logger(), "[OK] Dynamics solver initialized");
  RCLCPP_INFO(this->get_logger(), "   - Gravity: [%.2f, %.2f, %.2f] m/s²", gravity.x(), gravity.y(),
              gravity.z());

  // 载荷补偿: 提取 TCP 链 (base_link→tcp, 含 fixed joint_tcp 偏移)
  if (payload_compensation_enabled_) {
    if (!kdl_tree.getChain("base_link", "tcp", kdl_chain_tcp_)) {
      RCLCPP_WARN(this->get_logger(),
                  "[PAYLOAD] Failed to extract TCP chain, payload compensation disabled");
      payload_compensation_enabled_ = false;
    } else {
      jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_tcp_);
      payload_wrench_.setZero();
      RCLCPP_INFO(this->get_logger(),
                  "[PAYLOAD] Jacobian solver initialized (%d segments, %d joints)",
                  kdl_chain_tcp_.getNrOfSegments(), kdl_chain_tcp_.getNrOfJoints());
    }
  }

  return true;
}

bool TorqueControllerActionServer::interpolateTrajectory(
    const trajectory_msgs::msg::JointTrajectory &trajectory, double t_now, KDL::JntArray &q_d,
    KDL::JntArray &qd_d, KDL::JntArray &qdd_d) {
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Trajectory is empty, cannot interpolate");
    return false;
  }

  const auto &first_point = trajectory.points[0];
  double t_first = first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9;

  if (t_now <= t_first) {
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

  const auto &last_point = trajectory.points.back();
  double t_last = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

  if (t_now >= t_last) {
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

  const auto &point_before = trajectory.points[idx_before];
  const auto &point_after = trajectory.points[idx_after];

  double t_before = point_before.time_from_start.sec + point_before.time_from_start.nanosec * 1e-9;
  double t_after = point_after.time_from_start.sec + point_after.time_from_start.nanosec * 1e-9;

  // [FIX] divide-by-zero guard before the division.
  double dt_seg = t_after - t_before;
  double alpha = (dt_seg < 1e-9) ? 0.0 : (t_now - t_before) / dt_seg;

  for (size_t i = 0; i < kArmJoints; i++) {
    double pos_before = (i < point_before.positions.size()) ? point_before.positions[i] : 0.0;
    double pos_after = (i < point_after.positions.size()) ? point_after.positions[i] : 0.0;
    q_d(i) = pos_before + alpha * (pos_after - pos_before);

    if (i < point_before.velocities.size() && i < point_after.velocities.size()) {
      qd_d(i) = point_before.velocities[i] +
                alpha * (point_after.velocities[i] - point_before.velocities[i]);
    } else {
      qd_d(i) = 0.0;
    }

    if (i < point_before.accelerations.size() && i < point_after.accelerations.size()) {
      qdd_d(i) = point_before.accelerations[i] +
                 alpha * (point_after.accelerations[i] - point_before.accelerations[i]);
    } else {
      qdd_d(i) = 0.0;
    }
  }

  return true;
}

// --- 反馈控制: 阻抗(J1/J2/J5) + 级联PID(J3/J4/J6) 混合调度 ---
void TorqueControllerActionServer::computeFeedbackTorque(const KDL::JntArray &q_d,
                                                         const KDL::JntArray &qd_d,
                                                         const KDL::JntArray &q_actual,
                                                         const KDL::JntArray &qd_actual,
                                                         KDL::JntArray &tau_fb) {
  if (!cascade_pid_ || !impedance_controller_) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "[ERROR] Feedback controllers not initialized!");
    for (size_t i = 0; i < kArmJoints; i++) tau_fb(i) = 0.0;
    return;
  }

  std::vector<double> pos_ref(kArmJoints), pos_fdb(kArmJoints), vel_fdb(kArmJoints),
      torque_out(kArmJoints);

  for (size_t i = 0; i < kArmJoints; i++) {
    pos_ref[i] = q_d(i);
    pos_fdb[i] = q_actual(i);
    vel_fdb[i] = qd_actual(i);
  }

  double dt = 1.0 / control_frequency_;
  // 级联PID 仍对全部关节计算（非阻抗关节使用其输出）
  cascade_pid_->compute(pos_ref, pos_fdb, vel_fdb, dt, torque_out);

  for (size_t i = 0; i < kArmJoints; i++) {
    if (kUseImpedance_[i]) {
      tau_fb(i) =
          impedance_controller_->getJointController(i).compute(pos_ref[i], pos_fdb[i], vel_fdb[i]);
    } else {
      tau_fb(i) = torque_out[i];
    }
  }
}

// --- 载荷重力补偿: τ_payload = J(q)^T × [0,0,-mg,0,0,0] ---
void TorqueControllerActionServer::computePayloadCompensation(const KDL::JntArray &q,
                                                              double gripper_pos,
                                                              double gripper_force,
                                                              KDL::JntArray &tau_out) {
  for (int i = 0; i < kArmJoints; i++) tau_out(i) = 0.0;

  if (!payload_compensation_enabled_ || !jac_solver_) return;

  // 抓取检测: 有夹力 + 夹爪未完全闭合 (被物体阻挡)
  if (gripper_force <= payload_force_threshold_ || gripper_pos <= payload_pos_threshold_) return;

  int ret = jac_solver_->JntToJac(q, jacobian_);
  if (ret < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "[PAYLOAD] Jacobian failed (ret=%d)", ret);
    return;
  }

  payload_wrench_.setZero();
  payload_wrench_(2) = -payload_mass_ * 9.81;  // Fz = -mg

  Eigen::Matrix<double, Eigen::Dynamic, 1> tau_eigen = jacobian_.data.transpose() * payload_wrench_;

  for (int i = 0; i < kArmJoints; i++) {
    tau_out(i) = std::isfinite(tau_eigen(i)) ? tau_eigen(i) : 0.0;
  }
}

bool TorqueControllerActionServer::safeTorquePublish(const KDL::JntArray &tau_arm,
                                                     double gripper_force) {
  // 复用预分配消息，避免热路径 malloc
  auto &msg = torque_msg_preallocated_;
  for (int i = 0; i < kArmJoints; i++) {
    if (!std::isfinite(tau_arm(i))) {
      torque_nonfinite_count_++;
      RCLCPP_ERROR(this->get_logger(), "[SAFETY] Non-finite torque on joint %d: %.2f", i + 1,
                   tau_arm(i));
      emergencyStop("Non-finite torque detected");
      return false;
    }
    double limit = (i < static_cast<int>(max_torque_per_joint_.size())) ? max_torque_per_joint_[i]
                                                                        : max_torque_default_;
    msg.data[i] = std::clamp(tau_arm(i), -limit, limit);
    if (std::abs(tau_arm(i)) > limit) {
      torque_saturation_count_++;
      const double exceed = std::abs(tau_arm(i)) - limit;
      updateMaxAtomic(torque_saturation_max_milli_, static_cast<uint64_t>(exceed * 1000.0));
    }
  }
  msg.data[kArmJoints] = std::clamp(gripper_force, -gripper_max_force_, gripper_max_force_);
  // data[7] = seq id (单调递增 uint8 回卷, hardware 侧用来检测重复/跳序 callback)
  msg.data[kAllJoints] = static_cast<double>(torque_seq_++ & 0xFF);
  rt_cmd_pub_->try_publish(msg);
  torque_publish_count_++;
  for (int i = 0; i < kAllJoints; i++) {
    last_valid_torque_[i] = msg.data[i];
  }
  return true;
}

// route_mode 路径: 不算 PID/阻抗/G, 仅采当前 q_target 转发到 cmd topic
// pos_target_msg.data[0..5] 语义 = q_target(rad), data[6] = gripper, data[7] = seq
// MCU 端配合: 收到 0x0002 后解析 data[0..5] 为 q_target, 用本地 K/D 跑闭环
// 锁顺序遵守: action_mutex_ → state_mutex_ → gripper_mutex_
void TorqueControllerActionServer::routeTargetToHardware() {
  const bool executing = is_executing_.load(std::memory_order_acquire);
  const uint8_t mode = control_mode_.load(std::memory_order_acquire);

  KDL::JntArray q_target_out(kArmJoints);
  bool out_valid = false;        // q_target_out 已由轨迹路径填好
  bool trajectory_done = false;  // 本帧轨迹结束 (用 last_point + succeed action)
  std::shared_ptr<GoalHandleFJT> completed_handle;  // 待 succeed 的 goal

  // ---- Phase 2: 轨迹执行中, 实时插值 q_d 作为 q_target ----
  if (executing && mode == ControlMode::ARMED) {
    trajectory_msgs::msg::JointTrajectory traj_copy;
    rclcpp::Time traj_start_copy;
    std::shared_ptr<GoalHandleFJT> goal_handle_copy;
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      traj_copy = current_trajectory_;
      traj_start_copy = trajectory_start_time_;
      goal_handle_copy = current_goal_handle_;
    }
    if (!traj_copy.points.empty()) {
      const double t_now = (this->now() - traj_start_copy).seconds();
      const auto &last_pt = traj_copy.points.back();
      const double total_dur = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9;
      if (t_now >= total_dur) {
        // 轨迹时间到, 收尾用 last_point, 标记 succeed
        for (int i = 0; i < kArmJoints; i++) {
          q_target_out(i) = (i < (int)last_pt.positions.size()) ? last_pt.positions[i] : 0.0;
        }
        out_valid = true;
        trajectory_done = true;
        completed_handle = goal_handle_copy;
      } else {
        // 中间帧: 插值 q_d
        KDL::JntArray q_d(kArmJoints), qd_d(kArmJoints), qdd_d(kArmJoints);
        if (interpolateTrajectory(traj_copy, t_now, q_d, qd_d, qdd_d)) {
          q_target_out = q_d;
          out_valid = true;
        }
        // else: 插值失败 → 走下方 Hold fallback
      }
    }
  }

  // ---- state snapshot + 轨迹完成时同步写回 q_target_ (供下帧 Hold 接住) ----
  KDL::JntArray q_copy(kArmJoints), q_target_copy(kArmJoints);
  bool has_target_copy = false;
  bool state_ok = false;
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    state_ok = state_received_;
    if (state_ok) {
      q_copy = q_actual_;
      has_target_copy = has_target_;
      if (has_target_) q_target_copy = q_target_;
    }
    if (trajectory_done && out_valid) {
      // 把 last_point 写回 q_target_ 让下一帧 Hold path 接住
      for (int i = 0; i < kArmJoints; i++) q_target_(i) = q_target_out(i);
      has_target_ = true;
    }
  }
  if (!state_ok) return;  // 无 joint_states, MCU IsTimeout 自身 Relax

  // ---- 非轨迹分支决定 q_target_out ----
  if (!out_valid) {
    if (mode == ControlMode::RELAX || mode == ControlMode::FREEDRIVE) {
      q_target_out = q_copy;
    } else if (has_target_copy) {
      q_target_out = q_target_copy;
    } else {
      q_target_out = q_copy;
    }
  }

  // ---- 轨迹完成 succeed (在 state lock 释放后取 action lock) ----
  if (trajectory_done && completed_handle) {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    completed_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[ROUTE] Trajectory completed");
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      is_executing_.store(false, std::memory_order_release);
      current_goal_handle_.reset();
    }
  }

  // ---- gripper + publish ----
  double gripper_force_copy;
  {
    std::lock_guard<std::mutex> glock(gripper_mutex_);
    gripper_force_copy = gripper_force_cmd_;
  }

  auto &msg = torque_msg_preallocated_;
  for (int i = 0; i < kArmJoints; i++) {
    msg.data[i] = std::isfinite(q_target_out(i)) ? q_target_out(i) : q_copy(i);
  }
  msg.data[kArmJoints] = std::clamp(gripper_force_copy, -gripper_max_force_, gripper_max_force_);
  msg.data[kAllJoints] = static_cast<double>(torque_seq_++ & 0xFF);
  rt_cmd_pub_->try_publish(msg);
  torque_publish_count_++;
}

void TorqueControllerActionServer::controlLoop() {
  using Clock = std::chrono::steady_clock;
  using us = std::chrono::microseconds;

  const auto t_entry = Clock::now();
  const int64_t control_gap_us =
      (last_control_loop_entry_ == Clock::time_point{})
          ? 0
          : std::chrono::duration_cast<us>(t_entry - last_control_loop_entry_).count();
  last_control_loop_entry_ = t_entry;

  // --- SAFETY: Monitor control loop timing ---
  rclcpp::Time now = this->now();
  if (control_loop_count_ > 0) {
    double period = (now - last_control_loop_time_).seconds();
    if (period > max_control_period_sec_) {
      control_slow_count_++;
      updateMaxAtomic(control_overrun_max_gap_us_, static_cast<uint64_t>(period * 1000000.0));
    }
  }
  last_control_loop_time_ = now;

  // --- Health monitoring ---
  control_loop_count_++;

  // Timing section accumulators (filled by each path before recording)
  int64_t lock_us = 0, dyn_us = 0, pub_us = 0;
  int64_t joint_state_age_us = -1;
  const char *path_name = "RELAX";  // updated per code path

  // -----------------------------------------------------------------------
  // Helper lambdas to avoid duplicating timing boilerplate at every return.
  // Call commit() once right before each early return.
  // -----------------------------------------------------------------------
  auto commit = [&]() {
    int64_t total_us = std::chrono::duration_cast<us>(Clock::now() - t_entry).count();

    bool print_now = loop_prof_.record_and_check(total_us, control_gap_us, joint_state_age_us,
                                                 lock_us, dyn_us, pub_us, 1000);

    if (total_us > loop_prof_.overrun_threshold_us) {
      control_overrun_count_++;
      updateMaxAtomic(control_overrun_max_us_, static_cast<uint64_t>(total_us));
      updateMaxAtomic(control_overrun_max_gap_us_, static_cast<uint64_t>(control_gap_us));
      if (joint_state_age_us > 0) {
        updateMaxAtomic(control_overrun_max_js_age_us_, static_cast<uint64_t>(joint_state_age_us));
      }
    }

    if (print_now) {
      const uint64_t publish_now = torque_publish_count_.load(std::memory_order_relaxed);
      const uint64_t publish_delta = publish_now - last_timing_publish_count_;
      last_timing_publish_count_ = publish_now;
      RCLCPP_INFO(this->get_logger(),
                  "[TIMING/1k] total: min=%ldus avg=%ldus max=%ldus | "
                  "gap: avg=%ldus max=%ldus | js_age: avg=%ldus max=%ldus | "
                  "lock: avg=%ldus max=%ldus | dyn: avg=%ldus max=%ldus | "
                  "pub: avg=%ldus max=%ldus count=%lu | overruns=%lu",
                  loop_prof_.total.min_us, loop_prof_.total.avg_us(), loop_prof_.total.max_us,
                  loop_prof_.gap.avg_us(), loop_prof_.gap.max_us, loop_prof_.joint_age.avg_us(),
                  loop_prof_.joint_age.max_us, loop_prof_.state_lock.avg_us(),
                  loop_prof_.state_lock.max_us, loop_prof_.dynamics.avg_us(),
                  loop_prof_.dynamics.max_us, loop_prof_.publish.avg_us(),
                  loop_prof_.publish.max_us, publish_delta, loop_prof_.overruns);
      loop_prof_.reset_all();
    }
  };

  // Print health status every 5 seconds (5000 loops at 1kHz)
  if (control_loop_count_ % 5000 == 0) {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    double time_since_state =
        state_received_ ? (this->now() - last_joint_state_time_).seconds() : -1.0;
    static uint64_t last_joint_state_msg_count = 0;
    const uint64_t joint_state_msg_now = joint_state_msg_count_.load(std::memory_order_relaxed);
    const double joint_state_rx_rate =
        static_cast<double>(joint_state_msg_now - last_joint_state_msg_count) / 5.0;
    last_joint_state_msg_count = joint_state_msg_now;
    const uint64_t slow_count = control_slow_count_.exchange(0);
    const uint64_t overrun_count = control_overrun_count_.exchange(0);
    const uint64_t overrun_max_us = control_overrun_max_us_.exchange(0);
    const uint64_t overrun_max_gap_us = control_overrun_max_gap_us_.exchange(0);
    const uint64_t overrun_max_js_age_us = control_overrun_max_js_age_us_.exchange(0);
    const uint64_t invalid_size = joint_state_invalid_size_count_.exchange(0);
    const uint64_t nonfinite = joint_state_nonfinite_count_.exchange(0);
    const uint64_t pos_range = joint_state_pos_range_count_.exchange(0);
    const uint64_t vel_spike = joint_state_velocity_spike_count_.exchange(0);
    const uint64_t timeouts = joint_state_timeout_count_.exchange(0);
    const uint64_t saturations = torque_saturation_count_.exchange(0);
    const uint64_t saturation_max_milli = torque_saturation_max_milli_.exchange(0);
    const uint64_t nonfinite_torque = torque_nonfinite_count_.exchange(0);

    static const char *mode_names[] = {"RELAX", "FREEDRIVE", "ARMED"};
    uint8_t m = control_mode_.load(std::memory_order_acquire);
    const char *mode_str = (m <= ControlMode::ARMED) ? mode_names[m] : "UNKNOWN";
    RCLCPP_INFO(
        this->get_logger(),
        "[HEALTH] Loop #%zu | Mode: %s%s | State age: %.1f ms | js_rx: %.1f Hz | Kalman: %s",
        control_loop_count_, mode_str, is_executing_.load(std::memory_order_acquire) ? "+EXEC" : "",
        time_since_state * 1000.0, joint_state_rx_rate, kalman_filter_enabled_ ? "ON" : "OFF");
    RCLCPP_INFO(
        this->get_logger(),
        "[HEALTH/CTRL_DIAG] slow=%lu overruns=%lu max_total=%luus max_gap=%luus max_js_age=%luus "
        "| js_invalid=%lu js_nonfinite=%lu js_pos_range=%lu js_vel_spike=%lu js_timeout=%lu "
        "| torque_sat=%lu sat_max=%.3fNm torque_nonfinite=%lu",
        slow_count, overrun_count, overrun_max_us, overrun_max_gap_us, overrun_max_js_age_us,
        invalid_size, nonfinite, pos_range, vel_spike, timeouts, saturations,
        static_cast<double>(saturation_max_milli) / 1000.0, nonfinite_torque);
  }

  // === ROUTE MODE: bypass 控制律, 仅转发 q_target → MCU 跑本地 PID/阻抗 ===
  // 决策: 上位机闭环延迟 3-5ms 使下位机鲁棒参数 K=10 D=0.63 在上位机失稳 (J5 现象)
  // 比赛优先: 把控制律全压回 MCU 1ms 同 tick 闭环, 上位机只做规划层
  if (route_mode_.load(std::memory_order_acquire)) {
    const auto t_pub_start = Clock::now();
    routeTargetToHardware();
    pub_us = std::chrono::duration_cast<us>(Clock::now() - t_pub_start).count();
    path_name = "ROUTE";
    commit();
    return;
  }

  bool executing = is_executing_.load(std::memory_order_acquire);

  if (!executing) {
    uint8_t mode = control_mode_.load(std::memory_order_acquire);

    // RELAX 模式: 发全零力矩，不需要 joint_states
    if (mode == ControlMode::RELAX) {
      path_name = "RELAX";
      {
        const auto t0 = Clock::now();
        rt_cmd_pub_->try_publish(zero_msg_preallocated_);
        torque_publish_count_++;
        pub_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
      }
      commit();
      return;
    }

    // 以下模式需要 joint_states — 先在锁内拷贝状态，释放后再计算
    KDL::JntArray q_copy(kArmJoints), qd_copy(kArmJoints);
    KDL::JntArray q_target_copy(kArmJoints);
    rclcpp::Time last_state_time_copy;
    bool has_target_copy = false;
    double gripper_pos_copy = 0.0;
    {
      const auto t0 = Clock::now();
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      lock_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();

      if (!state_received_) {
        commit();
        return;
      }
      q_copy = q_actual_;
      qd_copy = q_dot_filtered_;
      last_state_time_copy = last_joint_state_time_;
      has_target_copy = has_target_;
      if (has_target_) q_target_copy = q_target_;
      gripper_pos_copy = gripper_position_;
    }

    // 读取夹爪力指令
    double gripper_force_copy;
    {
      std::lock_guard<std::mutex> glock(gripper_mutex_);
      gripper_force_copy = gripper_force_cmd_;
    }

    // --- SAFETY: Check joint state timeout ---
    double time_since_last_state = (this->now() - last_state_time_copy).seconds();
    joint_state_age_us = static_cast<int64_t>(time_since_last_state * 1000000.0);
    if (time_since_last_state > joint_state_timeout_sec_) {
      path_name = "TIMEOUT";
      joint_state_timeout_count_++;
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "[SAFETY] Joint state timeout: %.3fs (limit: %.0f ms)",
                            time_since_last_state, joint_state_timeout_sec_ * 1000.0);
      KDL::JntArray tau_timeout(kArmJoints);
      {
        const auto t0 = Clock::now();
        KDL::JntArray tau_g_full(kArmJoints);
        dynamic_computer_->computeGravityTorque(q_copy, tau_g_full);
        computePayloadCompensation(q_copy, gripper_pos_copy, gripper_force_copy, tau_payload_);
        dyn_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
        // 对齐 ARMED_HOLD: 仅 J2/J3 用 G，避免 js timeout/恢复时 J1/J4/J5/J6 力矩跳变
        static constexpr bool kUseGravityTO[kArmJoints] = {false, true, true, false, false, false};
        for (int i = 0; i < kArmJoints; i++) {
          tau_timeout(i) = (kUseGravityTO[i] ? tau_g_full(i) : 0.0) + tau_payload_(i);
        }
      }
      {
        const auto t0 = Clock::now();
        safeTorquePublish(tau_timeout, gripper_force_copy);
        pub_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
      }
      if (cascade_pid_) cascade_pid_->resetAll();
      if (impedance_controller_) impedance_controller_->resetAll();
      commit();
      return;
    }

    // --- 控制律计算: G(q) + friction + payload [+ PD] ---
    KDL::JntArray tau_arm(kArmJoints);
    {
      const auto t0 = Clock::now();
      KDL::JntArray tau_gravity(kArmJoints);
      dynamic_computer_->computeGravityTorque(q_copy, tau_gravity);
      computePayloadCompensation(q_copy, gripper_pos_copy, gripper_force_copy, tau_payload_);

      // 对齐下位机 ff={0,g1,g2,0,0,0}: J1/J4/J5/J6 KDL G 与实物偏差大，反而激励振荡
      static constexpr bool kUseGravity[kArmJoints] = {false, true, true, false, false, false};
      for (int i = 0; i < kArmJoints; i++) {
        double g = kUseGravity[i] ? tau_gravity(i) : 0.0;
        tau_arm(i) = g + computeFrictionTorque(i, qd_copy(i)) + tau_payload_(i);
      }

      // FREEDRIVE: G + friction + payload，无 PD
      if (mode == ControlMode::FREEDRIVE) {
        path_name = "FREEDRIVE";
        dyn_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
        const auto tp = Clock::now();
        safeTorquePublish(tau_arm, gripper_force_copy);
        pub_us = std::chrono::duration_cast<us>(Clock::now() - tp).count();
        commit();
        return;
      }

      // ARMED (hold): 追加 PD
      path_name = "ARMED_HOLD";
      KDL::JntArray tau_pd(kArmJoints);
      KDL::JntArray qd_zero(kArmJoints);
      if (has_target_copy) {
        computeFeedbackTorque(q_target_copy, qd_zero, q_copy, qd_copy, tau_pd);
      } else {
        computeFeedbackTorque(q_copy, qd_zero, q_copy, qd_copy, tau_pd);
      }
      for (int i = 0; i < kArmJoints; i++) tau_arm(i) += tau_pd(i);
      dyn_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
    }

    {
      const auto tp = Clock::now();
      safeTorquePublish(tau_arm, gripper_force_copy);
      pub_us = std::chrono::duration_cast<us>(Clock::now() - tp).count();
    }
    commit();
    return;
  }

  // --- ARMED + executing: 轨迹跟踪 ---
  path_name = "ARMED_EXEC";
  trajectory_msgs::msg::JointTrajectory traj_copy;
  rclcpp::Time traj_start_copy;
  std::shared_ptr<GoalHandleFJT> goal_handle_copy;
  {
    const auto t0 = Clock::now();
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    lock_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();
    traj_copy = current_trajectory_;
    traj_start_copy = trajectory_start_time_;
    goal_handle_copy = current_goal_handle_;
  }

  if (traj_copy.points.empty()) {
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    if (is_executing_.load(std::memory_order_acquire)) {
      executionRecoveryCeremony("Empty trajectory detected");
    }
    commit();
    return;
  }

  double t_now = (this->now() - traj_start_copy).seconds();
  const auto &last_pt = traj_copy.points.back();
  double total_dur = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec * 1e-9;

  // --- 轨迹完成: succeed + flip ---
  if (t_now >= total_dur) {
    RCLCPP_INFO(this->get_logger(), "[OK] Trajectory completed (%.3fs)", total_dur);
    if (goal_handle_copy) {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle_copy->succeed(result);
    }
    {
      std::lock_guard<std::mutex> action_lock(action_mutex_);
      is_executing_.store(false, std::memory_order_release);
      current_goal_handle_.reset();
    }
    commit();
    return;
  }

  // --- 轨迹插值 ---
  KDL::JntArray q_d(kArmJoints), qd_d(kArmJoints), qdd_d(kArmJoints);
  if (!interpolateTrajectory(traj_copy, t_now, q_d, qd_d, qdd_d)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Interpolation failed at t=%.3fs", t_now);
    std::lock_guard<std::mutex> action_lock(action_mutex_);
    if (current_goal_handle_ && is_executing_.load(std::memory_order_acquire)) {
      executionRecoveryCeremony("Interpolation failure");
    }
    commit();
    return;
  }

  // --- 状态拷贝 ---
  KDL::JntArray q_act(kArmJoints), qd_filt(kArmJoints);
  rclcpp::Time last_state_time_exec;
  double grip_pos = 0.0;
  {
    const auto t0 = Clock::now();
    std::lock_guard<std::mutex> lock(state_mutex_);
    lock_us += std::chrono::duration_cast<us>(Clock::now() - t0).count();  // accumulate both locks
    q_act = q_actual_;
    qd_filt = q_dot_filtered_;
    last_state_time_exec = last_joint_state_time_;
    grip_pos = gripper_position_;
  }
  joint_state_age_us =
      static_cast<int64_t>((this->now() - last_state_time_exec).seconds() * 1000000.0);
  double grip_force;
  {
    std::lock_guard<std::mutex> glock(gripper_mutex_);
    grip_force = gripper_force_cmd_;
  }

  // --- TRACK: M(q_d)*q̈_d + C(q_d,q̇_d) + G(q) + friction + PD + payload ---
  KDL::JntArray tau_arm(kArmJoints);
  {
    const auto t0 = Clock::now();
    KDL::JntSpaceInertiaMatrix M(kArmJoints);
    KDL::JntArray C(kArmJoints), G(kArmJoints), tau_fb(kArmJoints);
    dynamic_computer_->getMassMatrix(q_d, M);
    dynamic_computer_->getCoriolisForces(q_d, qd_d, C);
    dynamic_computer_->getGravityForces(q_act, G);
    computeFeedbackTorque(q_d, qd_d, q_act, qd_filt, tau_fb);
    computePayloadCompensation(q_act, grip_pos, grip_force, tau_payload_);
    dyn_us = std::chrono::duration_cast<us>(Clock::now() - t0).count();

    for (int i = 0; i < kArmJoints; i++) {
      double tau_inertia = 0.0;
      for (int j = 0; j < kArmJoints; j++) tau_inertia += M(i, j) * qdd_d(j);
      tau_arm(i) = tau_inertia + C(i) + G(i) + computeFrictionTorque(i, qd_filt(i)) + tau_fb(i) +
                   tau_payload_(i);
    }
  }

  {
    const auto tp = Clock::now();
    if (!safeTorquePublish(tau_arm, grip_force)) {
      commit();
      return;
    }
    pub_us = std::chrono::duration_cast<us>(Clock::now() - tp).count();
  }

  // --- Action feedback ---
  auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
  feedback->header.stamp = this->now();
  feedback->actual.positions.resize(kArmJoints);
  feedback->actual.velocities.resize(kArmJoints);
  feedback->desired.positions.resize(kArmJoints);
  feedback->desired.velocities.resize(kArmJoints);
  feedback->error.positions.resize(kArmJoints);
  feedback->error.velocities.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    feedback->actual.positions[i] = q_act(i);
    feedback->actual.velocities[i] = qd_filt(i);
    feedback->desired.positions[i] = q_d(i);
    feedback->desired.velocities[i] = qd_d(i);
    feedback->error.positions[i] = q_d(i) - q_act(i);
    feedback->error.velocities[i] = qd_d(i) - qd_filt(i);
  }
  if (goal_handle_copy) goal_handle_copy->publish_feedback(feedback);
  commit();
}

rcl_interfaces::msg::SetParametersResult TorqueControllerActionServer::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "参数更新成功";

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    if (name == "route_mode") {
      route_mode_ = param.as_bool();
      RCLCPP_WARN(this->get_logger(),
                  "[ROUTE_MODE] dynamically set to %s — cmd_topic data[0..5] = %s",
                  route_mode_.load() ? "ENABLED" : "DISABLED",
                  route_mode_.load() ? "q_target(rad)" : "tau(Nm)");
    } else if (name == "kalman.enabled") {
      kalman_filter_enabled_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[INFO] Kalman filter %s",
                  kalman_filter_enabled_ ? "[OK] Enabled" : "[DISABLED]");
    } else if (name == "kalman.Q_pos" || name == "kalman.Q_vel") {
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
    } else if (name.find("cascade_pid.joint_") == 0) {
      size_t first_dot = name.find('.', 13);
      if (first_dot == std::string::npos) continue;

      size_t second_dot = name.find('.', first_dot + 1);
      if (second_dot == std::string::npos) continue;

      std::string joint_num_str = name.substr(13, first_dot - 13);
      std::string loop_type = name.substr(first_dot + 1, 3);
      std::string gain_type = name.substr(second_dot + 1);

      int joint_idx = std::stoi(joint_num_str) - 1;

      if (joint_idx >= 0 && joint_idx < 6) {
        double new_value = param.as_double();

        std::string prefix = "cascade_pid.joint_" + std::to_string(joint_idx + 1);

        PidGains pos_gains(this->get_parameter(prefix + ".pos_Kp").as_double(),
                           this->get_parameter(prefix + ".pos_Ki").as_double(),
                           this->get_parameter(prefix + ".pos_Kd").as_double());

        PidGains vel_gains(this->get_parameter(prefix + ".vel_Kp").as_double(),
                           this->get_parameter(prefix + ".vel_Ki").as_double(),
                           this->get_parameter(prefix + ".vel_Kd").as_double());

        double vel_limit = this->get_parameter(prefix + ".vel_limit").as_double();
        double max_integral_pos = this->get_parameter(prefix + ".max_integral_pos").as_double();

        cascade_pid_->setJointParams(joint_idx, pos_gains, vel_gains, vel_limit, max_integral_pos);

        RCLCPP_INFO(this->get_logger(), "[CONFIG] Cascade PID Joint %d updated: %s.%s = %.2f",
                    joint_idx + 1, loop_type.c_str(), gain_type.c_str(), new_value);
      }
    } else if (name.find("impedance.joint_") == 0) {
      // impedance.joint_X.{K,D,tau_limit}
      size_t dot1 = name.find('.', 10);  // after "impedance."
      if (dot1 == std::string::npos) continue;
      std::string joint_name = name.substr(10, dot1 - 10);  // "joint_1" etc.
      int joint_idx = std::stoi(joint_name.substr(6)) - 1;  // "1"→0
      if (joint_idx >= 0 && joint_idx < kArmJoints && kUseImpedance_[joint_idx]) {
        std::string prefix = "impedance." + joint_name;
        ImpedanceGains g;
        g.K = this->get_parameter(prefix + ".K").as_double();
        g.D = this->get_parameter(prefix + ".D").as_double();
        g.tau_limit = this->get_parameter(prefix + ".tau_limit").as_double();
        impedance_controller_->setJointGains(joint_idx, g);
        RCLCPP_INFO(this->get_logger(), "[CONFIG] Impedance %s updated: K=%.1f D=%.2f limit=%.1f",
                    joint_name.c_str(), g.K, g.D, g.tau_limit);
      }
    } else if (name == "payload.enabled") {
      payload_compensation_enabled_ = param.as_bool();
      RCLCPP_INFO(this->get_logger(), "[PAYLOAD] %s",
                  payload_compensation_enabled_ ? "ENABLED" : "DISABLED");
    } else if (name == "payload.mass") {
      payload_mass_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PAYLOAD] Mass: %.3f kg", payload_mass_);
    } else if (name == "payload.gripper_force_threshold") {
      payload_force_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PAYLOAD] Force threshold: %.2f N",
                  payload_force_threshold_);
    } else if (name == "payload.gripper_pos_threshold") {
      payload_pos_threshold_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[PAYLOAD] Pos threshold: %.3f m", payload_pos_threshold_);
    } else if (name == "friction.coulomb") {
      friction_coulomb_ = param.as_double_array();
      RCLCPP_INFO(this->get_logger(), "[FRICTION] Coulomb updated: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                  friction_coulomb_[0], friction_coulomb_[1], friction_coulomb_[2],
                  friction_coulomb_[3], friction_coulomb_[4], friction_coulomb_[5]);
    } else if (name == "friction.viscous") {
      friction_viscous_ = param.as_double_array();
      RCLCPP_INFO(this->get_logger(), "[FRICTION] Viscous updated: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                  friction_viscous_[0], friction_viscous_[1], friction_viscous_[2],
                  friction_viscous_[3], friction_viscous_[4], friction_viscous_[5]);
    } else if (name == "friction.fal_alpha") {
      fal_alpha_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[FRICTION] fal alpha: %.2f", fal_alpha_);
    } else if (name == "friction.fal_delta") {
      fal_delta_ = param.as_double();
      RCLCPP_INFO(this->get_logger(), "[FRICTION] fal delta: %.3f", fal_delta_);
    }
  }

  return result;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TorqueControllerActionServer>();
    rclcpp::executors::StaticSingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("torque_controller"), "Fatal: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
