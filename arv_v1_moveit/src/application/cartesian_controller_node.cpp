#include <tf2/LinearMath/Quaternion.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

#include "arv_v1_interfaces/srv/move_to_cartesian_rpy.hpp"
#include "arv_v1_interfaces/srv/stop_cartesian_motion.hpp"

class CartesianControllerNode : public rclcpp::Node {
public:
  CartesianControllerNode();
  ~CartesianControllerNode();

private:
  // ========== 状态枚举 ==========
  enum class State { IDLE, PLANNING, EXECUTING, ERROR };

  // ========== 成员变量 ==========
  // MoveIt 接口 (延迟初始化)
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // 状态管理
  std::atomic<State> state_{State::IDLE};
  std::mutex execution_mutex_;

  // 规划工作线程 (话题路径专用，避免阻塞 ROS2 回调线程)
  std::thread planning_thread_;
  std::condition_variable plan_cv_;
  std::mutex plan_request_mutex_;
  std::optional<geometry_msgs::msg::Pose> pending_pose_;
  std::atomic<bool> shutdown_{false};

  // 配置参数
  std::string planning_group_;
  std::string end_effector_link_;
  std::string reference_frame_;
  double default_velocity_scaling_;
  double default_acceleration_scaling_;
  double pose_topic_min_interval_ms_;

  // 工作空间边界
  double ws_min_radius_;
  double ws_max_radius_;

  // 时间记录 (用于限速)
  rclcpp::Time last_pose_command_time_;

  // ========== ROS2 接口 ==========
  // 话题
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // 服务
  rclcpp::Service<arv_v1_interfaces::srv::MoveToCartesianRPY>::SharedPtr move_rpy_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::StopCartesianMotion>::SharedPtr stop_srv_;

  // 定时器
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr pose_publish_timer_;

  // ========== 方法声明 ==========
  void declareParameters();
  void loadParameters();
  void initializeMoveGroup();

  // 坐标转换
  geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw);

  // 验证
  bool validateTargetPose(double x, double y, double z, std::string& error_msg);

  // 话题路径工作线程
  void planningWorker();

  // 规划与执行 (带锁 — 服务路径调用)
  bool planAndExecute(const geometry_msgs::msg::Pose& target_pose, double velocity_scaling,
                      double acceleration_scaling, bool async, double& planning_time,
                      double& trajectory_duration, std::string& error_message);

  // 规划与执行 (无锁 — 话题路径已在外部持锁)
  bool planAndExecuteUnlocked(const geometry_msgs::msg::Pose& target_pose, double velocity_scaling,
                              double acceleration_scaling, bool async, double& planning_time,
                              double& trajectory_duration, std::string& error_message);

  // 异步执行完成监控
  void monitorAsyncExecution();

  // 回调函数
  void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void moveToCartesianRPYCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Response> response);
  void stopMotionCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Response> response);

  // 状态发布
  void publishCurrentPose();
  void publishStatus(const std::string& status);
};

CartesianControllerNode::CartesianControllerNode() : Node("cartesian_controller_node") {
  declareParameters();
  loadParameters();

  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_controller/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/cartesian_controller/status", 10);

  move_rpy_srv_ = this->create_service<arv_v1_interfaces::srv::MoveToCartesianRPY>(
      "/move_to_cartesian_rpy", std::bind(&CartesianControllerNode::moveToCartesianRPYCallback,
                                          this, std::placeholders::_1, std::placeholders::_2));

  stop_srv_ = this->create_service<arv_v1_interfaces::srv::StopCartesianMotion>(
      "/stop_cartesian_motion", std::bind(&CartesianControllerNode::stopMotionCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_target_pose", 10,
      std::bind(&CartesianControllerNode::poseTargetCallback, this, std::placeholders::_1));

  init_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&CartesianControllerNode::initializeMoveGroup, this));

  pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&CartesianControllerNode::publishCurrentPose, this));

  last_pose_command_time_ = this->now();

  // 启动规划工作线程（仅服务话题路径，与服务路径互不干扰）
  planning_thread_ = std::thread(&CartesianControllerNode::planningWorker, this);

  RCLCPP_INFO(this->get_logger(), "Cartesian Controller Node starting...");
  publishStatus("STARTING");
}

CartesianControllerNode::~CartesianControllerNode() {
  shutdown_ = true;
  plan_cv_.notify_all();
  if (planning_thread_.joinable()) {
    planning_thread_.join();
  }
}

void CartesianControllerNode::declareParameters() {
  this->declare_parameter("planning_group", "ARM");
  this->declare_parameter("end_effector_link", "link6_2006roll");
  this->declare_parameter("reference_frame", "base_link");
  this->declare_parameter("default_velocity_scaling", 1.0);
  this->declare_parameter("default_acceleration_scaling", 1.0);
  this->declare_parameter("pose_topic_min_interval_ms", 50.0);

  this->declare_parameter("ws_min_radius_", 0.01);
  this->declare_parameter("ws_max_radius_", 0.7);  // 单位m
}

void CartesianControllerNode::loadParameters() {
  planning_group_ = this->get_parameter("planning_group").as_string();
  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  default_velocity_scaling_ = this->get_parameter("default_velocity_scaling").as_double();
  default_acceleration_scaling_ = this->get_parameter("default_acceleration_scaling").as_double();
  pose_topic_min_interval_ms_ = this->get_parameter("pose_topic_min_interval_ms").as_double();

  ws_min_radius_ = this->get_parameter("ws_min_radius_").as_double();
  ws_max_radius_ = this->get_parameter("ws_max_radius_").as_double();
}

void CartesianControllerNode::planningWorker() {
  while (!shutdown_) {
    geometry_msgs::msg::Pose target;
    {
      std::unique_lock<std::mutex> lock(plan_request_mutex_);
      // 阻塞等待新目标或关闭信号
      plan_cv_.wait(lock, [this] { return pending_pose_.has_value() || shutdown_.load(); });
      if (shutdown_) break;
      target = pending_pose_.value();
      pending_pose_.reset();  // 取走目标，丢弃等待期间累积的旧值
    }

    // 尝试获取执行锁 — 如果服务路径正在执行，跳过本次话题目标
    std::unique_lock<std::mutex> exec_lock(execution_mutex_, std::try_to_lock);
    if (!exec_lock.owns_lock()) {
      RCLCPP_DEBUG(this->get_logger(), "[topic path] skipped: service path is executing");
      continue;
    }

    // 停止当前运动：无运动时为空操作；有运动时实现话题抢占
    if (move_group_) {
      move_group_->stop();
    }

    double pt = 0.0, td = 0.0;
    std::string err;
    // 话题路径已持有 execution_mutex_，planAndExecute 内不再重复加锁
    planAndExecuteUnlocked(target, default_velocity_scaling_, default_acceleration_scaling_,
                           /*async=*/true, pt, td, err);
    if (!err.empty()) {
      RCLCPP_WARN(this->get_logger(), "[topic path] planning failed: %s", err.c_str());
    }
  }
  RCLCPP_INFO(this->get_logger(), "Planning worker thread exiting");
}

bool CartesianControllerNode::validateTargetPose(double x, double y, double z,
                                                 std::string& error_msg) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    RCLCPP_ERROR(this->get_logger(), "Input axis is not finite, invalid!");
    error_msg = "Non-finite values: x: " + std::to_string(x) + " y: " + std::to_string(y) +
                " z: " + std::to_string(z);
    return false;
  }
  double r2 = x * x + y * y + z * z;
  if (r2 > ws_max_radius_ * ws_max_radius_) {
    RCLCPP_ERROR(this->get_logger(), "Input axis is out of bounds, invalid!");
    error_msg = "Out of bounds: x: " + std::to_string(x) + " y: " + std::to_string(y) +
                " z: " + std::to_string(z);
    return false;
  }
  return true;
}

geometry_msgs::msg::Quaternion CartesianControllerNode::rpyToQuaternion(double roll, double pitch,
                                                                        double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  geometry_msgs::msg::Quaternion quat_msg;
  quat_msg.x = q.x();
  quat_msg.y = q.y();
  quat_msg.z = q.z();
  quat_msg.w = q.w();
  return quat_msg;
}

void CartesianControllerNode::initializeMoveGroup() {
  if (move_group_) {
    return;  // 已初始化，取消定时器
  }
  try {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(1.0);
    move_group_->setMaxVelocityScalingFactor(default_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(default_acceleration_scaling_);
    init_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized for group '%s'",
                planning_group_.c_str());
    publishStatus("IDLE");
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "MoveGroup init failed (will retry): %s", e.what());
  }
}

bool CartesianControllerNode::planAndExecute(const geometry_msgs::msg::Pose& target_pose,
                                             double velocity_scaling, double acceleration_scaling,
                                             bool async, double& planning_time,
                                             double& trajectory_duration,
                                             std::string& error_message) {
  // 服务路径: 加锁后委托给无锁版本
  std::lock_guard<std::mutex> lock(execution_mutex_);
  return planAndExecuteUnlocked(target_pose, velocity_scaling, acceleration_scaling, async,
                                planning_time, trajectory_duration, error_message);
}

bool CartesianControllerNode::planAndExecuteUnlocked(const geometry_msgs::msg::Pose& target_pose,
                                                     double velocity_scaling,
                                                     double acceleration_scaling, bool async,
                                                     double& planning_time,
                                                     double& trajectory_duration,
                                                     std::string& error_message) {
  if (!move_group_) {
    error_message = "MoveGroupInterface not initialized yet";
    return false;
  }

  state_ = State::PLANNING;
  publishStatus("PLANNING");

  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 首选 LIN，失败后回退 PTP
  move_group_->setPlannerId("LIN");
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "LIN planning failed, trying PTP fallback");
    move_group_->setPlannerId("PTP");
    result = move_group_->plan(plan);
  }

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    state_ = State::ERROR;
    error_message = "Planning failed (both LIN and PTP)";
    publishStatus("ERROR");
    return false;
  }

  planning_time = plan.planning_time;
  trajectory_duration =
      plan.trajectory.joint_trajectory.points.empty()
          ? 0.0
          : plan.trajectory.joint_trajectory.points.back().time_from_start.sec +
                plan.trajectory.joint_trajectory.points.back().time_from_start.nanosec * 1e-9;

  state_ = State::EXECUTING;
  publishStatus("EXECUTING");

  if (async) {
    move_group_->asyncExecute(plan);
    // 不立即设 IDLE — 启动监控线程等待执行完成
    std::thread(&CartesianControllerNode::monitorAsyncExecution, this).detach();
    return true;
  } else {
    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
      state_ = State::ERROR;
      error_message = "Execution failed";
      publishStatus("ERROR");
      return false;
    }
    state_ = State::IDLE;
    publishStatus("IDLE");
    return true;
  }
}

void CartesianControllerNode::monitorAsyncExecution() {
  // 轮询 MoveGroupInterface 的运动状态，直到不再 EXECUTING
  const auto timeout = std::chrono::seconds(30);
  const auto poll_interval = std::chrono::milliseconds(50);
  auto start = std::chrono::steady_clock::now();

  while (state_ == State::EXECUTING && !shutdown_) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      RCLCPP_WARN(this->get_logger(), "Async execution monitor timed out (30s)");
      state_ = State::IDLE;
      publishStatus("IDLE");
      return;
    }
    std::this_thread::sleep_for(poll_interval);
  }

  // 如果状态仍是 EXECUTING (被 stop 中断等)，恢复 IDLE
  State expected = State::EXECUTING;
  if (state_.compare_exchange_strong(expected, State::IDLE)) {
    publishStatus("IDLE");
  }
}

void CartesianControllerNode::poseTargetCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // ========== 此函数在 ROS2 spin 线程执行，必须快速返回 ==========
  auto elapsed_ms = (this->now() - last_pose_command_time_).nanoseconds() / 1e6;
  if (elapsed_ms < pose_topic_min_interval_ms_) {
    return;
  }
  last_pose_command_time_ = this->now();

  std::string err;
  if (!validateTargetPose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, err)) {
    RCLCPP_WARN(this->get_logger(), "Pose validation failed: %s", err.c_str());
    return;
  }

  // 存入最新目标并通知工作线程（覆盖未被处理的旧目标，始终保留最新值）
  {
    std::lock_guard<std::mutex> lock(plan_request_mutex_);
    pending_pose_ = msg->pose;
  }
  plan_cv_.notify_one();  // 非阻塞，立即返回
}

void CartesianControllerNode::moveToCartesianRPYCallback(
    const std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Request> request,
    std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Response> response) {
  std::string err;
  if (!validateTargetPose(request->x, request->y, request->z, err)) {
    response->success = false;
    response->message = err;
    return;
  }

  geometry_msgs::msg::Pose target;
  target.position.x = request->x;
  target.position.y = request->y;
  target.position.z = request->z;
  target.orientation = rpyToQuaternion(request->roll, request->pitch, request->yaw);

  double vel =
      request->velocity_scaling > 0.0 ? request->velocity_scaling : default_velocity_scaling_;
  double acc = request->acceleration_scaling > 0.0 ? request->acceleration_scaling
                                                   : default_acceleration_scaling_;

  response->success =
      planAndExecute(target, vel, acc, request->async_execution, response->planning_time,
                     response->trajectory_duration, response->message);
}

void CartesianControllerNode::stopMotionCallback(
    const std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Request> /*request*/,
    std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Response> response) {
  if (move_group_) {
    move_group_->stop();
    state_ = State::IDLE;
    publishStatus("IDLE");
    response->success = true;
    response->message = "Motion stopped";
    RCLCPP_INFO(this->get_logger(), "Motion stopped by service call");
  } else {
    response->success = false;
    response->message = "MoveGroup not initialized";
  }
}

void CartesianControllerNode::publishCurrentPose() {
  if (!move_group_) {
    return;
  }
  try {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = reference_frame_;
    pose_msg.pose = move_group_->getCurrentPose(end_effector_link_).pose;
    current_pose_pub_->publish(pose_msg);
  } catch (const std::exception& e) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Failed to get current pose: %s", e.what());
  }
}

void CartesianControllerNode::publishStatus(const std::string& status) {
  std_msgs::msg::String msg;
  msg.data = status;
  status_pub_->publish(msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}