/**
 * @file cartesian_controller_node.cpp
 * @brief Cartesian IK servo: 笛卡尔位姿 → KDL IK → 关节位置目标
 *
 * 统一接口：视觉伺服、TUI jogging 等所有笛卡尔输入通过
 * /cartesian_target_pose 话题 → KDL IK 解算 → /joint_position_target 发给
 * torque_controller 的 HOLD 模式 PD 跟踪。
 *
 * 全程无雅可比速度映射，无奇异爆炸风险。IK 无解时保持上一帧位置。
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

static constexpr int kArmJoints = 6;

class CartesianControllerNode : public rclcpp::Node {
public:
  CartesianControllerNode();
  ~CartesianControllerNode() = default;

private:
  // --- KDL IK ---
  KDL::Chain kdl_chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;

  // 关节状态缓存（IK 种子）
  KDL::JntArray q_current_{kArmJoints};
  std::mutex js_mutex_;
  bool has_joint_state_ = false;

  // TF 位姿查询
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 配置参数
  std::string end_effector_link_;
  std::string reference_frame_;
  double ws_min_radius_;
  double ws_max_radius_;

  // --- ROS2 接口 ---
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_target_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_target_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr pose_publish_timer_;

  // --- 方法 ---
  bool initializeIK();
  bool validateTargetPose(double x, double y, double z, std::string& error_msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void publishCurrentPose();
  void publishStatus(const std::string& status);
};

// --- 实现 ---

CartesianControllerNode::CartesianControllerNode() : Node("cartesian_controller_node") {
  // 参数
  this->declare_parameter("end_effector_link", "tcp");
  this->declare_parameter("reference_frame", "base_link");
  this->declare_parameter("ws_min_radius", 0.01);
  this->declare_parameter("ws_max_radius", 0.8);

  end_effector_link_ = this->get_parameter("end_effector_link").as_string();
  reference_frame_ = this->get_parameter("reference_frame").as_string();
  ws_min_radius_ = this->get_parameter("ws_min_radius").as_double();
  ws_max_radius_ = this->get_parameter("ws_max_radius").as_double();

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 发布者
  joint_target_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/joint_position_target", 10);
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_controller/current_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cartesian_controller/status", 10);

  // 订阅者
  js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CartesianControllerNode::jointStateCallback, this, std::placeholders::_1));

  pose_target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_target_pose", rclcpp::SensorDataQoS(),
      std::bind(&CartesianControllerNode::poseTargetCallback, this, std::placeholders::_1));

  // 定时器
  pose_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CartesianControllerNode::publishCurrentPose, this));

  // IK 初始化
  if (initializeIK()) {
    RCLCPP_INFO(this->get_logger(), "Cartesian IK servo node initialized (chain: %s → %s, %d joints)",
                reference_frame_.c_str(), end_effector_link_.c_str(),
                static_cast<int>(kdl_chain_.getNrOfJoints()));
    publishStatus("IDLE");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize IK solver");
    publishStatus("ERROR");
  }
}

bool CartesianControllerNode::initializeIK() {
  // 从 URDF 文件构建 KDL chain（与 torque_controller 相同方式）
  std::string urdf_path;
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("arv_v1_model");
    urdf_path = pkg_path + "/urdf/arv_v1.urdf";
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to find arv_v1_model package: %s", e.what());
    return false;
  }

  std::ifstream urdf_file(urdf_path);
  if (!urdf_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open URDF: %s", urdf_path.c_str());
    return false;
  }
  std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                           std::istreambuf_iterator<char>());

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(urdf_string, kdl_tree)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to build KDL tree from URDF");
    return false;
  }

  if (!kdl_tree.getChain(reference_frame_, end_effector_link_, kdl_chain_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to extract chain: %s → %s",
                 reference_frame_.c_str(), end_effector_link_.c_str());
    return false;
  }

  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
  ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);

  RCLCPP_INFO(this->get_logger(), "KDL chain: %d segments, %d joints",
              static_cast<int>(kdl_chain_.getNrOfSegments()),
              static_cast<int>(kdl_chain_.getNrOfJoints()));
  return true;
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
    q_current_(i) = msg->position[i];
  }
  has_joint_state_ = true;
}

void CartesianControllerNode::poseTargetCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!ik_solver_ || !has_joint_state_) return;

  // 验证目标位置
  std::string err;
  if (!validateTargetPose(msg->pose.position.x, msg->pose.position.y,
                          msg->pose.position.z, err)) {
    RCLCPP_WARN(this->get_logger(), "Target rejected: %s", err.c_str());
    return;
  }

  // PoseStamped → KDL::Frame
  KDL::Frame target_frame(
      KDL::Rotation::Quaternion(
          msg->pose.orientation.x, msg->pose.orientation.y,
          msg->pose.orientation.z, msg->pose.orientation.w),
      KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));

  // IK 求解（当前关节位置作种子，保证解的连续性）
  KDL::JntArray q_seed(kArmJoints), q_result(kArmJoints);
  {
    std::lock_guard<std::mutex> lock(js_mutex_);
    q_seed = q_current_;
  }

  int ret = ik_solver_->CartToJnt(q_seed, target_frame, q_result);
  if (ret < 0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "IK failed (ret=%d), holding position", ret);
    return;  // IK 无解，不发布，torque_controller 保持上一帧
  }

  // 关节速度限幅: 防止 IK 多解跳变导致翻转
  // 同一末端位姿可能对应多组关节解，数值 IK 可能跨分支收敛。
  // 限幅保证每帧关节变化不超过 max_joint_step，将跳变平滑为渐变。
  // 推导: 步长 0.005m / 臂展 0.5m ≈ 0.01 rad 正常量，×10 裕量 = 0.1 rad
  //        J4 腕部 roll 力臂短，同笛卡尔步长对应更大关节变化，放宽到 0.2
  static constexpr double max_joint_step[] = {0.1, 0.1, 0.1, 0.2, 0.1, 0.2};  // rad/帧 @10Hz
  std_msgs::msg::Float64MultiArray target_msg;
  target_msg.data.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; i++) {
    double delta = q_result(i) - q_seed(i);
    if (std::abs(delta) > max_joint_step[i]) {
      target_msg.data[i] = q_seed(i) + std::copysign(max_joint_step[i], delta);
    } else {
      target_msg.data[i] = q_result(i);
    }
  }
  joint_target_pub_->publish(target_msg);
}

void CartesianControllerNode::publishCurrentPose() {
  // TF lookup: 无阻塞，读缓存
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
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
