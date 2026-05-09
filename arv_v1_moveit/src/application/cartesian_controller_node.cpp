/// @file cartesian_controller_node.cpp
/// @brief Cartesian IK servo: pose → analytical IK → joint target. Zero-iteration closed-form.

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "analytical_ik.hpp"

static constexpr int kArmJoints = 6;

class CartesianControllerNode : public rclcpp::Node {
public:
  CartesianControllerNode();
  ~CartesianControllerNode() = default;

private:
  AnalyticalIK ik_;

  std::array<double, kArmJoints> q_current_{};
  std::array<double, kArmJoints> q_last_target_{};
  bool has_last_target_ = false;
  std::mutex js_mutex_;
  bool has_joint_state_ = false;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string end_effector_link_;
  std::string reference_frame_;
  double ws_min_radius_;
  double ws_max_radius_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr pose_publish_timer_;

  bool validateTargetPose(double x, double y, double z, std::string& error_msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishCurrentPose();
  void publishStatus(const std::string& status);
};

CartesianControllerNode::CartesianControllerNode() : Node("cartesian_controller_node") {
  this->declare_parameter("end_effector_link", "tcp");
  this->declare_parameter("reference_frame", "base_link");
  this->declare_parameter("ws_min_radius", 0.01);
  this->declare_parameter("ws_max_radius", 0.8);

  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  ws_min_radius_ = this->get_parameter("ws_min_radius").as_double();
  ws_max_radius_ = this->get_parameter("ws_max_radius").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_target_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_position_target", 10);
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_controller/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>("/cartesian_controller/status", 10);

  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&CartesianControllerNode::jointStateCallback, this, std::placeholders::_1));

  pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_target_pose", rclcpp::SensorDataQoS(),
      std::bind(&CartesianControllerNode::poseTargetCallback, this, std::placeholders::_1));

  pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33), std::bind(&CartesianControllerNode::publishCurrentPose, this));

  RCLCPP_INFO(this->get_logger(),
              "Cartesian analytical IK servo initialized (L2=%.3f L3=%.3f tcp=%.3f)",
              AnalyticalIK::kL2, AnalyticalIK::kL3, AnalyticalIK::kDtcp);
  publishStatus("IDLE");
}

bool CartesianControllerNode::validateTargetPose(double x, double y, double z,
                                                 std::string& error_msg) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    error_msg = "Non-finite position values";
    return false;
  }
  double r2 = x * x + y * y + z * z;
  if (r2 < ws_min_radius_ * ws_min_radius_) {
    error_msg = "Too close to base: r=" + std::to_string(std::sqrt(r2));
    return false;
  }
  if (r2 > ws_max_radius_ * ws_max_radius_) {
    error_msg = "Out of workspace: r=" + std::to_string(std::sqrt(r2));
    return false;
  }
  return true;
}

void CartesianControllerNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (static_cast<int>(msg->position.size()) < kArmJoints) return;
  std::lock_guard<std::mutex> lock(js_mutex_);
  for (int i = 0; i < kArmJoints; i++) {
    q_current_[i] = msg->position[i];
  }
  has_joint_state_ = true;
}

void CartesianControllerNode::poseTargetCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!has_joint_state_) return;

  std::string err;
  if (!validateTargetPose(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, err)) {
    RCLCPP_WARN(this->get_logger(), "Target rejected: %s", err.c_str());
    return;
  }

  // Extract rotation matrix (row-major) from quaternion
  tf2::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                       msg->pose.orientation.w);
  quat.normalize();
  tf2::Matrix3x3 rot_mat(quat);
  double R[9];
  for (int i = 0; i < 3; i++) {
    tf2::Vector3 row = rot_mat.getRow(i);
    R[i * 3 + 0] = row.x();
    R[i * 3 + 1] = row.y();
    R[i * 3 + 2] = row.z();
  }

  double p[3] = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};

  // Seed: prefer last IK output for continuity, clamp feedback to limits
  double q_seed[6];
  {
    std::lock_guard<std::mutex> lock(js_mutex_);
    for (int i = 0; i < kArmJoints; i++) {
      double raw = has_last_target_ ? q_last_target_[i] : q_current_[i];
      q_seed[i] = std::clamp(raw, AnalyticalIK::kJointLower[i], AnalyticalIK::kJointUpper[i]);
    }
  }

  IKResult result = ik_.solve(R, p, q_seed);
  if (!result.valid) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Analytical IK: no valid solution, holding position");
    return;
  }

  // Rate-limit and clamp (same logic as before)
  static constexpr double max_joint_step[] = {0.1, 0.1, 0.1, 0.2, 0.1, 0.2};
  std_msgs::msg::Float64MultiArray target_msg;
  target_msg.data.resize(kArmJoints);

  for (int i = 0; i < kArmJoints; i++) {
    double val = result.q[i];
    double seed = q_seed[i];

    if (i == 3) {
      // J4: wrap handling for ±π ambiguity
      double ik_wrapped = std::remainder(val, 2.0 * M_PI);
      double delta = std::remainder(ik_wrapped - seed, 2.0 * M_PI);
      double candidate = seed + delta;

      if ((candidate > AnalyticalIK::kJointUpper[i] || candidate < AnalyticalIK::kJointLower[i]) &&
          (ik_wrapped >= AnalyticalIK::kJointLower[i] &&
           ik_wrapped <= AnalyticalIK::kJointUpper[i])) {
        delta += (delta > 0) ? -2.0 * M_PI : 2.0 * M_PI;
      }

      if (std::abs(delta) > max_joint_step[i]) {
        delta = std::copysign(max_joint_step[i], delta);
      }
      val = std::clamp(seed + delta, AnalyticalIK::kJointLower[i], AnalyticalIK::kJointUpper[i]);
    } else if (i == 5) {
      // J6: continuous, no clamp
      double delta = val - seed;
      if (std::abs(delta) > max_joint_step[i]) {
        val = seed + std::copysign(max_joint_step[i], delta);
      }
      val = std::remainder(val, 2.0 * M_PI);
    } else {
      double delta = val - seed;
      if (std::abs(delta) > max_joint_step[i]) {
        val = seed + std::copysign(max_joint_step[i], delta);
      }
      val = std::clamp(val, AnalyticalIK::kJointLower[i], AnalyticalIK::kJointUpper[i]);
    }

    target_msg.data[i] = val;
    q_last_target_[i] = val;
  }
  has_last_target_ = true;
  joint_target_pub_->publish(target_msg);
}

void CartesianControllerNode::publishCurrentPose() {
  try {
    auto tf = tf_buffer_->lookupTransform(reference_frame_, end_effector_link_, tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = reference_frame_;
    pose_msg.pose.position.x = tf.transform.translation.x;
    pose_msg.pose.position.y = tf.transform.translation.y;
    pose_msg.pose.position.z = tf.transform.translation.z;
    pose_msg.pose.orientation = tf.transform.rotation;
    current_pose_pub_->publish(pose_msg);
  } catch (const tf2::TransformException& e) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF lookup failed: %s",
                          e.what());
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
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
