/**
 * @file cartesian_controller_node.cpp
 * @brief Cartesian IK controller using Pilz LIN/PTP planner
 *
 * 通过 MoveGroupInterface 调用 move_group 节点的 Pilz 规划器，
 * 将笛卡尔空间目标位姿转换为关节轨迹并执行。
 *
 * [NOTE] MoveGroupInterface 构造时自动启动 CurrentStateMonitor（订阅 /joint_states）。
 * 该监控器在后台运行无害，但禁止在 executor 回调内调用 getCurrentState/getCurrentPose —
 * 这些方法内部 waitForCompleteState() 阻塞等待 /joint_states 回调被调度，
 * 而该回调需要同一个 executor 来处理 → 回调线程自锁。
 * 位姿查询统一使用 TF lookupTransform（读缓存，不依赖 executor 调度）。
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "arv_v1_interfaces/srv/move_to_cartesian_rpy.hpp"
#include "arv_v1_interfaces/srv/stop_cartesian_motion.hpp"

class CartesianControllerNode : public rclcpp::Node {
public:
  CartesianControllerNode();
  ~CartesianControllerNode() = default;

private:
  // --- 成员变量 ---
  // MoveIt 接口 (延迟初始化，仅用于 plan/execute/stop)
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // TF 位姿查询（替代 MoveGroupInterface::getCurrentState()，避免回调自锁）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 配置参数
  std::string planning_group_;
  std::string end_effector_link_;
  std::string reference_frame_;
  double default_velocity_scaling_;
  double default_acceleration_scaling_;

  // 工作空间边界
  double ws_min_radius_;
  double ws_max_radius_;

  // --- ROS2 接口 ---
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Service<arv_v1_interfaces::srv::MoveToCartesianRPY>::SharedPtr move_rpy_srv_;
  rclcpp::Service<arv_v1_interfaces::srv::StopCartesianMotion>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr pose_publish_timer_;

  // --- 方法声明 ---
  void declareParameters();
  void loadParameters();
  void initializeMoveGroup();

  geometry_msgs::msg::Quaternion rpyToQuaternion(double roll, double pitch, double yaw);
  bool validateTargetPose(double x, double y, double z, std::string& error_msg);

  bool planAndExecute(const geometry_msgs::msg::Pose& target_pose, double velocity_scaling,
                      double acceleration_scaling, double& planning_time,
                      double& trajectory_duration, std::string& error_message);

  void moveToCartesianRPYCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::MoveToCartesianRPY::Response> response);
  void stopMotionCallback(
      const std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Request> request,
      std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Response> response);

  void publishCurrentPose();
  void publishStatus(const std::string& status);
};

// --- 实现 ---

CartesianControllerNode::CartesianControllerNode() : Node("cartesian_controller_node") {
  declareParameters();
  loadParameters();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 发布者
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_controller/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/cartesian_controller/status", 10);

  // 服务
  move_rpy_srv_ = this->create_service<arv_v1_interfaces::srv::MoveToCartesianRPY>(
      "/move_to_cartesian_rpy", std::bind(&CartesianControllerNode::moveToCartesianRPYCallback,
                                          this, std::placeholders::_1, std::placeholders::_2));
  stop_srv_ = this->create_service<arv_v1_interfaces::srv::StopCartesianMotion>(
      "/stop_cartesian_motion", std::bind(&CartesianControllerNode::stopMotionCallback, this,
                                          std::placeholders::_1, std::placeholders::_2));

  // 定时器
  init_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(500),
                              std::bind(&CartesianControllerNode::initializeMoveGroup, this));
  pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&CartesianControllerNode::publishCurrentPose, this));

  RCLCPP_INFO(this->get_logger(), "Cartesian Controller Node starting...");
  publishStatus("STARTING");
}

void CartesianControllerNode::declareParameters() {
  this->declare_parameter("planning_group", "ARM");
  this->declare_parameter("end_effector_link", "tcp");
  this->declare_parameter("reference_frame", "base_link");
  this->declare_parameter("default_velocity_scaling", 1.0);
  this->declare_parameter("default_acceleration_scaling", 1.0);
  this->declare_parameter("ws_min_radius", 0.01);
  this->declare_parameter("ws_max_radius", 0.7);
}

void CartesianControllerNode::loadParameters() {
  planning_group_ = this->get_parameter("planning_group").as_string();
  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  default_velocity_scaling_ = this->get_parameter("default_velocity_scaling").as_double();
  default_acceleration_scaling_ = this->get_parameter("default_acceleration_scaling").as_double();
  ws_min_radius_ = this->get_parameter("ws_min_radius").as_double();
  ws_max_radius_ = this->get_parameter("ws_max_radius").as_double();
}

void CartesianControllerNode::initializeMoveGroup() {
  if (move_group_) {
    return;
  }
  try {
    // [NOTE] 构造 MoveGroupInterface 会自动启动 CurrentStateMonitor（后台订阅 /joint_states）。
    // 该监控器无害，但禁止在 executor 回调内调用 getCurrentState/getCurrentPose（回调自锁）。
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setPoseReferenceFrame(reference_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(2.0);
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

bool CartesianControllerNode::validateTargetPose(double x, double y, double z,
                                                 std::string& error_msg) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    RCLCPP_ERROR(this->get_logger(), "Input axis is not finite, invalid!");
    error_msg = "Non-finite values: x: " + std::to_string(x) + " y: " + std::to_string(y) +
                " z: " + std::to_string(z);
    return false;
  }
  double r2 = x * x + y * y + z * z;
  if (r2 < ws_min_radius_ * ws_min_radius_) {
    RCLCPP_ERROR(this->get_logger(), "Target too close to base: r=%.4f < %.4f", std::sqrt(r2),
                 ws_min_radius_);
    error_msg = "Too close to base: r=" + std::to_string(std::sqrt(r2));
    return false;
  }
  if (r2 > ws_max_radius_ * ws_max_radius_) {
    RCLCPP_ERROR(this->get_logger(), "Target out of workspace: r=%.4f > %.4f", std::sqrt(r2),
                 ws_max_radius_);
    error_msg = "Out of workspace: r=" + std::to_string(std::sqrt(r2));
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

bool CartesianControllerNode::planAndExecute(const geometry_msgs::msg::Pose& target_pose,
                                             double velocity_scaling, double acceleration_scaling,
                                             double& planning_time, double& trajectory_duration,
                                             std::string& error_message) {
  if (!move_group_) {
    error_message = "MoveGroupInterface not initialized yet";
    return false;
  }

  publishStatus("PLANNING");

  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  // 首选 LIN (笛卡尔直线)，失败后回退 PTP (关节空间)
  move_group_->setPlannerId("LIN");
  auto result = move_group_->plan(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "LIN planning failed, trying PTP fallback");
    move_group_->setPlannerId("PTP");
    result = move_group_->plan(plan);
    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "PTP fallback planning succeeded");
    }
    move_group_->setPlannerId("LIN");  // 恢复默认
  }

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
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

  publishStatus("EXECUTING");

  auto exec_result = move_group_->execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    error_message = "Execution failed";
    publishStatus("ERROR");
    return false;
  }

  publishStatus("IDLE");
  return true;
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
      planAndExecute(target, vel, acc, response->planning_time,
                     response->trajectory_duration, response->message);
}

void CartesianControllerNode::stopMotionCallback(
    const std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Request> /*request*/,
    std::shared_ptr<arv_v1_interfaces::srv::StopCartesianMotion::Response> response) {
  if (move_group_) {
    move_group_->stop();
    response->success = true;
    response->message = "Motion stopped";
    publishStatus("IDLE");
    RCLCPP_INFO(this->get_logger(), "Motion stopped by service call");
  } else {
    response->success = false;
    response->message = "MoveGroup not initialized";
  }
}

void CartesianControllerNode::publishCurrentPose() {
  // [FIX] 使用 TF 替代 MoveGroupInterface::getCurrentState()
  // getCurrentState() 在 executor 回调内调用会自锁（等待 /joint_states
  // 回调被调度，但 executor 被当前回调占用），导致 CPU 67% + 节点卡死。
  // lookupTransform 读 TF 缓存，无需 executor 调度新回调。
  try {
    auto tf = tf_buffer_->lookupTransform(
        reference_frame_, end_effector_link_, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = reference_frame_;
    pose_msg.pose.position.x = tf.transform.translation.x;
    pose_msg.pose.position.y = tf.transform.translation.y;
    pose_msg.pose.position.z = tf.transform.translation.z;
    pose_msg.pose.orientation = tf.transform.rotation;
    current_pose_pub_->publish(pose_msg);
  } catch (const tf2::TransformException& e) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "TF lookup failed: %s", e.what());
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
  // [NOTE] SingleThreadedExecutor 即可：所有阻塞 API（getCurrentState）已移除。
  // plan()/execute() 在服务回调中同步执行，期间定时器暂停但不影响功能。
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
