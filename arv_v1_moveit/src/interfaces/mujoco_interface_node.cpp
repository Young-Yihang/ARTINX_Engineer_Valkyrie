/// @file mujoco_interface_node.cpp
/// @brief MuJoCo interface — simulation mode (torque→state) / digital-twin mode (state→visual).

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

class MuJoCoInterfaceNode : public rclcpp::Node {
  static constexpr int kArmJoints = 6;  // 6-DOF arm
  static constexpr int kAllJoints = 7;  // 6-DOF arm + 1 gripper
  static constexpr int kSimJoints = 8;  // 6-DOF arm + 1 gripper with 1 mimic joint

public:
  MuJoCoInterfaceNode()
      : Node("mujoco_interface"),
        model_(nullptr),
        data_(nullptr),
        sim_frequency_(200.0),
        received_first_command_(false),
        visualization_only_(false) {
    RCLCPP_INFO(this->get_logger(), "[START] MuJoCo interface node starting");

    this->declare_parameter("visualization_only", false);
    visualization_only_ = this->get_parameter("visualization_only").as_bool();

    if (visualization_only_) {
      RCLCPP_INFO(this->get_logger(),
                  "[MODE] Digital Twin - visualization only (subscribing /joint_states)");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[MODE] Physics Simulation (subscribing /effort_controller/commands)");
    }

    try {
      pkg_share_dir_ = ament_index_cpp::get_package_share_directory("arv_v1_model");
      urdf_path_ = pkg_share_dir_ + "/urdf/arv_v1.urdf";
      mesh_dir_ = pkg_share_dir_ + "/meshes";

      RCLCPP_INFO(this->get_logger(), "[OK] Package path: %s", pkg_share_dir_.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to find arv_v1_model package: %s", e.what());
      throw;
    }

    temp_dir_ = std::filesystem::temp_directory_path() / "arv_v1_mujoco";
    std::filesystem::create_directories(temp_dir_);
    RCLCPP_INFO(this->get_logger(), "[OK] Temp directory: %s", temp_dir_.c_str());

    this->declare_parameter("scene_config_file", std::string(""));
    scene_config_path_ = this->get_parameter("scene_config_file").as_string();
    if (scene_config_path_.empty()) {
      // 默认: arv_v1_moveit/config/scene_obstacles.yaml
      try {
        auto moveit_share = ament_index_cpp::get_package_share_directory("arv_v1_moveit");
        scene_config_path_ = moveit_share + "/config/scene_obstacles.yaml";
      } catch (...) {
        RCLCPP_WARN(this->get_logger(),
                    "[WARN] Cannot find arv_v1_moveit package for scene config");
      }
    }

    if (!loadMuJoCoModel()) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] MuJoCo model loading failed");
      throw std::runtime_error("Failed to load MuJoCo model");
    }
    RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo model loaded successfully");
    RCLCPP_INFO(this->get_logger(), "   Number of joints: %d", model_->nq);

    setInitialPose();

    if (visualization_only_) {
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
          "/joint_states", 10,
          std::bind(&MuJoCoInterfaceNode::jointStateCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "[OK] Subscribing: /joint_states (digital twin)");
    } else {
      effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/effort_controller/commands", 10,
          std::bind(&MuJoCoInterfaceNode::effortCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(), "[OK] Subscribing: /effort_controller/commands");

      joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
      RCLCPP_INFO(this->get_logger(), "[OK] Publishing: /joint_states");

      auto period = std::chrono::duration<double, std::milli>(1000.0 / sim_frequency_);
      sim_timer_ =
          this->create_wall_timer(period, std::bind(&MuJoCoInterfaceNode::simulationStep, this));
      RCLCPP_INFO(this->get_logger(), "[INFO] Simulation frequency: %.1f Hz", sim_frequency_);
    }
    RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo interface node initialization completed");

    health_timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                            std::bind(&MuJoCoInterfaceNode::healthCheck, this));

    render_running_ = false;
    window_ = nullptr;

    if (initializeVisualization()) {
      RCLCPP_INFO(this->get_logger(), "[OK] Visualization initialized successfully");
      render_running_ = true;
      render_thread_ = std::thread(&MuJoCoInterfaceNode::renderLoop, this);
      RCLCPP_INFO(this->get_logger(), "[INFO] Rendering thread started");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "[WARN] Visualization initialization failed, simulation only");
    }
  }

  ~MuJoCoInterfaceNode() {
    RCLCPP_INFO(this->get_logger(), "[INFO] Cleaning up MuJoCo resources...");

    render_running_ = false;
    if (render_thread_.joinable()) {
      render_thread_.join();
    }

    if (window_) {
      mjr_freeContext(&con_);
      mjv_freeScene(&scene_);
      glfwDestroyWindow(window_);
      glfwTerminate();
    }

    if (data_) {
      mj_deleteData(data_);
    }

    if (model_) {
      mj_deleteModel(model_);
    }

    RCLCPP_INFO(this->get_logger(), "[OK] Resource cleanup completed");
  }

private:
  // ========== MuJoCo ==========
  mjModel *model_;
  mjData *data_;

  // ========== ROS2 ==========
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;  // digital-twin
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr sim_timer_;

  // ========== 运行模式 ==========
  bool visualization_only_;  // true=digital-twin, false=physics-sim

  // ========== 配置 ==========
  std::string pkg_share_dir_;
  std::string urdf_path_;
  std::string mesh_dir_;
  std::filesystem::path temp_dir_;
  double sim_frequency_;
  std::string scene_config_path_;  // scene_obstacles.yaml path

  // ========== 可视化 ==========
  std::thread render_thread_;
  std::atomic<bool> render_running_;

  GLFWwindow *window_;
  mjvScene scene_;
  mjvCamera cam_;
  mjvOption opt_;
  mjrContext con_;
  std::mutex sim_mutex_;

  // ========== 启动安全 ==========
  std::atomic<bool> received_first_command_;

  // ========== Health monitoring ==========
  std::atomic<uint64_t> sim_step_count_{0};
  std::atomic<uint64_t> command_rx_count_{0};
  rclcpp::TimerBase::SharedPtr health_timer_;

  // ========== 关节 ==========
  const std::vector<std::string> joint_names_ = {"joint_1",        "joint_2",       "joint_3",
                                                 "joint_4",        "joint_5",       "joint_6",
                                                 "joint_gripper1", "joint_gripper2"};

  // ========== 磁力吸引 ==========
  // 恒力模型: 矿核指向六边形中心, 被棍子碰撞挡住 → 无穿越振荡
  struct MagnetAnchor {
    int body_id;
    double anchor[3];    // 锚点世界坐标
    double force_mag;    // N
    double detach_dist;  // 脱离距离 (m)
    bool detached;
  };
  std::vector<MagnetAnchor> magnet_anchors_;
  double magnet_force_default_ = 14.4;  // mg + 10N
  double magnet_detach_dist_ = 0.25;    // core~0.11m离中心, 留余量

  // ========== 交互状态 ==========
  bool button_left_ = false;
  bool button_middle_ = false;
  bool button_right_ = false;
  double lastx_ = 0.0;
  double lasty_ = 0.0;

  std::atomic<bool> paused_{false};
  bool show_ui_ = true;
  bool show_contacts_ = false;
  bool show_forces_ = false;

  static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods);
  static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
  static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);
  static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

  bool loadMuJoCoModel();
  void setInitialPose();
  std::string buildObstacleMJCF();
  std::string loadObstacleURDF(const std::string &id, const std::string &urdf_uri);
  void effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void simulationStep();
  void publishJointStates();
  bool initializeVisualization();
  void renderLoop();
  void healthCheck();
  void applyMagnetForces();
  void collectMagnetAnchors();
};

bool MuJoCoInterfaceNode::loadMuJoCoModel() {
  RCLCPP_INFO(this->get_logger(), "[INFO] Loading URDF: %s", urdf_path_.c_str());

  std::ifstream urdf_file(urdf_path_);
  if (!urdf_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot open URDF file: %s", urdf_path_.c_str());
    return false;
  }

  std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                          std::istreambuf_iterator<char>());
  urdf_file.close();

  RCLCPP_INFO(this->get_logger(), "[OK] URDF file read successfully");

  // 插入 <mujoco> 编译器设置 (meshdir 指向动态包路径)
  std::string mujoco_compiler =
      "\n  <mujoco>\n"
      "    <compiler meshdir=\"" +
      mesh_dir_ +
      "\" strippath=\"false\"/>\n"
      "    <option timestep=\"0.005\"/>\n"
      "    <size nconmax=\"0\" njmax=\"0\"/>\n"
      "  </mujoco>\n";

  size_t robot_pos = urdf_string.find("<robot");
  if (robot_pos == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot find <robot> tag");
    return false;
  }

  size_t bracket_pos = urdf_string.find(">", robot_pos);
  urdf_string.insert(bracket_pos + 1, mujoco_compiler);

  // strip package:// prefix (meshdir already set in compiler tag)
  std::string find_str = "package://arv_v1_model/meshes/";
  std::string replace_str = "";
  size_t pos = 0;
  while ((pos = urdf_string.find(find_str, pos)) != std::string::npos) {
    urdf_string.replace(pos, find_str.length(), replace_str);
    pos += replace_str.length();
  }

  std::string temp_urdf_path = temp_dir_ / ".mujoco_temp.urdf";
  std::ofstream temp_file(temp_urdf_path);
  temp_file << urdf_string;
  temp_file.close();

  char error[1000] = "Could not load XML model";
  mjModel *temp_model = mj_loadXML(temp_urdf_path.c_str(), nullptr, error, 1000);
  if (!temp_model) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] MuJoCo failed to load URDF: %s", error);
    return false;
  }

  std::string mjcf_path = temp_dir_ / ".mujoco_converted.xml";
  mj_saveLastXML(mjcf_path.c_str(), temp_model, error, 1000);
  mj_deleteModel(temp_model);

  std::ifstream mjcf_file(mjcf_path);
  std::string mjcf_string((std::istreambuf_iterator<char>(mjcf_file)),
                          std::istreambuf_iterator<char>());
  mjcf_file.close();

  std::string actuator_mjcf =
      "\n  <actuator>\n"
      "    <motor name=\"actuator_1\" joint=\"joint_1\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_2\" joint=\"joint_2\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_3\" joint=\"joint_3\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_4\" joint=\"joint_4\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_5\" joint=\"joint_5\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_6\" joint=\"joint_6\" gear=\"1\" ctrllimited=\"true\" "
      "ctrlrange=\"-20 20\"/>\n"
      "    <motor name=\"actuator_gripper\" joint=\"joint_gripper1\" gear=\"1\" "
      "ctrllimited=\"true\" "
      "ctrlrange=\"-5 5\"/>\n"
      "    <motor name=\"actuator_gripper2\" joint=\"joint_gripper2\" gear=\"1\" "
      "ctrllimited=\"true\" "
      "ctrlrange=\"-5 5\"/>\n"
      "  </actuator>\n";

  size_t mujoco_end = mjcf_string.find("</mujoco>");
  mjcf_string.insert(mujoco_end, actuator_mjcf);

  // 注入障碍物 (碰撞由下方 contype/conaffinity 统一设置)
  std::string obstacle_mjcf = buildObstacleMJCF();
  if (!obstacle_mjcf.empty()) {
    mujoco_end = mjcf_string.find("</mujoco>");
    mjcf_string.insert(mujoco_end, obstacle_mjcf);
    RCLCPP_INFO(this->get_logger(), "[OK] Scene obstacles injected into MJCF");
  }

  // 提升 ambient 光照 (MuJoCo 默认 ambient=0 导致背光面全黑)
  std::string visual_mjcf =
      "\n  <visual>\n"
      "    <headlight ambient=\".4 .4 .4\" diffuse=\".8 .8 .8\" specular=\".1 .1 .1\"/>\n"
      "  </visual>\n";
  mujoco_end = mjcf_string.find("</mujoco>");
  mjcf_string.insert(mujoco_end, visual_mjcf);

  // 碰撞组设置 (MuJoCo规则: A与B碰撞 ⟺ (A.contype & B.conaffinity) != 0 OR (B.contype &
  // A.conaffinity) != 0):
  //
  //   机械臂 geom             contype=1(0001)  conaffinity=4(0100)
  //   URDF 视觉 mesh (有vcol) contype=0        conaffinity=0        ← 纯视觉
  //   静态障碍物 vcol         contype=2(0010)  conaffinity=1(0001)  ← 碰机械臂
  //   可抓取物体 vcol         contype=8(1000)  conaffinity=3(0011)  ← 碰机械臂+矿框
  //   地面 ground_plane       contype=4(0100)  conaffinity=9(1001)  ← 碰机械臂+矿核
  //   其他 fallback           contype=4(0100)  conaffinity=1(0001)  ← 碰机械臂
  //
  //   碰撞矩阵 (位运算验证):
  //     pair                contype&conaffinity    结果
  //     机械臂↔机械臂      1&4=0, 1&4=0           ✗ (无自碰)
  //     机械臂↔静态vcol    1&1=1                  ✓
  //     机械臂↔可抓取      1&3=1                  ✓
  //     机械臂↔地面        1&9=1                  ✓
  //     静态vcol↔可抓取    2&3=2                  ✓ (矿核坐在矿框上)
  //     地面↔可抓取        4&3=0, 8&9=8           ✓ (矿核掉落有地面兜底)
  //     可抓取↔可抓取      8&3=0, 8&3=0           ✗ (矿核间不碰撞!)
  //     静态vcol↔静态vcol  2&1=0                  ✗
  //     地面↔静态vcol      4&1=0, 2&9=0           ✗
  {
    size_t obs_start = mjcf_string.find("<body name=\"obstacle_");
    if (obs_start == std::string::npos) obs_start = mjcf_string.size();

    // 分段: 避免 insert 后偏移失效
    std::string robot_part = mjcf_string.substr(0, obs_start);
    std::string obs_part = mjcf_string.substr(obs_start);

    auto inject_contype = [](std::string &s, const std::string &attr) {
      size_t p = 0;
      while ((p = s.find("<geom", p)) != std::string::npos) {
        size_t end = s.find("/>", p);
        if (end == std::string::npos) end = s.find(">", p);
        if (s.substr(p, end - p).find("contype") == std::string::npos) {
          s.insert(end, attr);
          p = end + attr.size() + 1;
        } else {
          p = end + 1;
        }
      }
    };

    // 非弹性碰撞: solref 5ms+过阻尼, solimp 99%能量吸收
    inject_contype(robot_part,
                   " contype=\"1\" conaffinity=\"4\" friction=\"5.0 0.1 0.01\""
                   " solref=\"0.005 2\" solimp=\"0.99 0.99 0.0001 0.5 2\"");
    inject_contype(obs_part,
                   " contype=\"4\" conaffinity=\"1\" friction=\"0.001 0.001 0.0001\""
                   " solref=\"0.005 2\" solimp=\"0.99 0.99 0.0001 0.5 2\"");
    mjcf_string = robot_part + obs_part;
  }

  std::string final_mjcf_path = temp_dir_ / ".mujoco_final.xml";
  std::ofstream final_mjcf_file(final_mjcf_path);
  final_mjcf_file << mjcf_string;
  final_mjcf_file.close();

  model_ = mj_loadXML(final_mjcf_path.c_str(), nullptr, error, 1000);
  if (!model_) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to load final MJCF: %s", error);
    return false;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to create MuJoCo data structure");
    mj_deleteModel(model_);
    model_ = nullptr;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo model loaded successfully");

  // mj_forward 使 data_->xpos 有效 (collectMagnetAnchors 需要)
  mj_forward(model_, data_);
  collectMagnetAnchors();

  return true;
}

void MuJoCoInterfaceNode::collectMagnetAnchors() {
  magnet_anchors_.clear();

  if (scene_config_path_.empty() || !std::filesystem::exists(scene_config_path_)) return;

  YAML::Node config;
  try {
    config = YAML::LoadFile(scene_config_path_);
  } catch (...) {
    return;
  }

  auto params = config["scene_manager"]["ros__parameters"];
  if (!params) return;

  if (params["magnet_force_N"]) magnet_force_default_ = params["magnet_force_N"].as<double>(14.4);
  if (params["magnet_detach_dist"])
    magnet_detach_dist_ = params["magnet_detach_dist"].as<double>(0.15);

  auto obstacle_ids = params["obstacle_ids"];
  if (!obstacle_ids) return;

  for (const auto &id_node : obstacle_ids) {
    std::string id = id_node.as<std::string>();
    auto obs = params["obstacles"][id];
    if (!obs || !obs["graspable"] || !obs["graspable"].as<bool>(false)) continue;

    std::string body_name = "obstacle_" + id;
    int bid = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
    if (bid < 0) {
      RCLCPP_WARN(get_logger(), "[MAGNET] Body '%s' not found, skipping", body_name.c_str());
      continue;
    }

    // 锚点 = 六边形中心, 力指向中心 → 被棍子挡住无穿越
    MagnetAnchor anchor;
    anchor.body_id = bid;
    anchor.anchor[0] = obs["position"][0].as<double>(0);
    anchor.anchor[1] = 0.0;  // 六边形中心Y
    anchor.anchor[2] = 0.7;  // 六边形中心Z (6棒平均高度)
    anchor.force_mag = magnet_force_default_;
    anchor.detach_dist = magnet_detach_dist_;
    anchor.detached = false;

    magnet_anchors_.push_back(anchor);
    RCLCPP_INFO(get_logger(), "[MAGNET] Anchor: %s (body=%d) pos=[%.3f,%.3f,%.3f] F=%.1fN",
                id.c_str(), bid, anchor.anchor[0], anchor.anchor[1], anchor.anchor[2],
                anchor.force_mag);
  }

  RCLCPP_INFO(get_logger(), "[MAGNET] Total %zu magnet anchors registered", magnet_anchors_.size());
}

void MuJoCoInterfaceNode::applyMagnetForces() {
  for (auto &ma : magnet_anchors_) {
    if (ma.detached) continue;

    const double *xpos = data_->xpos + 3 * ma.body_id;
    double dx = ma.anchor[0] - xpos[0];
    double dy = ma.anchor[1] - xpos[1];
    double dz = ma.anchor[2] - xpos[2];
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[MAGNET-DBG] body=%d anchor=[%.4f,%.4f,%.4f] xpos=[%.4f,%.4f,%.4f] dist=%.4f F=%.1fN",
        ma.body_id, ma.anchor[0], ma.anchor[1], ma.anchor[2], xpos[0], xpos[1], xpos[2], dist,
        ma.force_mag);

    // [SAFETY] 前200步不检测脱离, 防止初始碰撞弹力误触发
    if (dist > ma.detach_dist && sim_step_count_ > 200) {
      ma.detached = true;
      data_->xfrc_applied[6 * ma.body_id + 0] = 0;
      data_->xfrc_applied[6 * ma.body_id + 1] = 0;
      data_->xfrc_applied[6 * ma.body_id + 2] = 0;
      RCLCPP_INFO(get_logger(), "[MAGNET] Body %d detached (dist=%.3fm)", ma.body_id, dist);
      continue;
    }

    if (dist > 1e-6) {
      double scale = ma.force_mag / dist;
      double fx = dx * scale, fy = dy * scale, fz = dz * scale;
      data_->xfrc_applied[6 * ma.body_id + 0] = fx;
      data_->xfrc_applied[6 * ma.body_id + 1] = fy;
      data_->xfrc_applied[6 * ma.body_id + 2] = fz;
      if (sim_step_count_ < 10) {
        RCLCPP_WARN(get_logger(),
                    "[MAGNET-INIT] body=%d F=[%.2f,%.2f,%.2f] dist=%.4f dx=[%.4f,%.4f,%.4f]",
                    ma.body_id, fx, fy, fz, dist, dx, dy, dz);
      }
    } else {
      data_->xfrc_applied[6 * ma.body_id + 0] = 0;
      data_->xfrc_applied[6 * ma.body_id + 1] = 0;
      data_->xfrc_applied[6 * ma.body_id + 2] = 0;
    }
  }
}

void MuJoCoInterfaceNode::setInitialPose() {
  // SRDF "Start" pose
  double initial_q[kSimJoints] = {0.0, 2.6343, 1.0785, 0.0, 0.0, 0.0, 0.0, 0.0};  // arm + gripper
  for (int i = 0; i < kSimJoints; i++) {
    data_->qpos[i] = initial_q[i];
  }
  mj_forward(model_, data_);
  RCLCPP_INFO(this->get_logger(), "[OK] Initial pose set (%d joints including gripper)",
              kSimJoints);
}

std::string MuJoCoInterfaceNode::loadObstacleURDF(const std::string &id,
                                                  const std::string &urdf_uri) {
  std::string disk_path = urdf_uri;
  const std::string pkg_prefix = "package://";
  if (urdf_uri.find(pkg_prefix) == 0) {
    size_t slash = urdf_uri.find('/', pkg_prefix.size());
    if (slash == std::string::npos) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Invalid package URI: %s", urdf_uri.c_str());
      return "";
    }
    std::string pkg_name = urdf_uri.substr(pkg_prefix.size(), slash - pkg_prefix.size());
    try {
      std::string pkg_dir = ament_index_cpp::get_package_share_directory(pkg_name);
      disk_path = pkg_dir + urdf_uri.substr(slash);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] Package '%s' not found: %s", pkg_name.c_str(),
                   e.what());
      return "";
    }
  }

  if (!std::filesystem::exists(disk_path)) {
    RCLCPP_WARN(this->get_logger(), "[WARN] Obstacle URDF not found: %s (skipping)",
                disk_path.c_str());
    return "";
  }

  std::ifstream f(disk_path);
  std::string urdf_str((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  f.close();

  // 上溯两级: urdf/obstacles/ → share/arv_v1_model/ → meshes/
  std::string obs_dir = std::filesystem::path(disk_path).parent_path().string();
  std::string obs_mesh_dir =
      (std::filesystem::path(obs_dir).parent_path().parent_path() / "meshes").string();

  std::string compiler_tag = "\n  <mujoco>\n    <compiler meshdir=\"" + obs_mesh_dir +
                             "\" strippath=\"false\"/>\n  </mujoco>\n";
  size_t robot_pos = urdf_str.find("<robot");
  if (robot_pos == std::string::npos) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] No <robot> tag in obstacle URDF: %s", id.c_str());
    return "";
  }
  size_t bracket = urdf_str.find(">", robot_pos);
  urdf_str.insert(bracket + 1, compiler_tag);

  std::string find_str = "package://arv_v1_model/meshes/";
  size_t pos = 0;
  while ((pos = urdf_str.find(find_str, pos)) != std::string::npos) {
    urdf_str.replace(pos, find_str.length(), "");
  }

  std::string temp_path = (temp_dir_ / (".obs_" + id + ".urdf")).string();
  std::ofstream out(temp_path);
  out << urdf_str;
  out.close();

  char error[1000] = "";
  mjModel *tmp = mj_loadXML(temp_path.c_str(), nullptr, error, 1000);
  if (!tmp) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] MuJoCo failed to load obstacle '%s': %s", id.c_str(),
                 error);
    return "";
  }

  std::string mjcf_path = (temp_dir_ / (".obs_" + id + ".xml")).string();
  mj_saveLastXML(mjcf_path.c_str(), tmp, error, 1000);
  mj_deleteModel(tmp);

  std::ifstream mf(mjcf_path);
  std::string mjcf_str((std::istreambuf_iterator<char>(mf)), std::istreambuf_iterator<char>());
  mf.close();

  RCLCPP_INFO(this->get_logger(), "[OK] Obstacle '%s' URDF→MJCF converted", id.c_str());
  return mjcf_str;
}

std::string MuJoCoInterfaceNode::buildObstacleMJCF() {
  if (scene_config_path_.empty() || !std::filesystem::exists(scene_config_path_)) {
    RCLCPP_INFO(this->get_logger(), "[INFO] No scene config file, skipping obstacle injection");
    return "";
  }

  RCLCPP_INFO(this->get_logger(), "[INFO] Loading scene obstacles from: %s",
              scene_config_path_.c_str());

  YAML::Node config;
  try {
    config = YAML::LoadFile(scene_config_path_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to parse scene YAML: %s", e.what());
    return "";
  }

  auto params = config["scene_manager"]["ros__parameters"];
  if (!params) return "";

  auto obstacle_ids = params["obstacle_ids"];
  if (!obstacle_ids || !obstacle_ids.IsSequence()) return "";

  std::ostringstream asset_ss;
  std::ostringstream body_ss;
  bool has_asset = false;
  int count = 0;
  std::map<std::string, std::string> urdf_mjcf_cache;
  std::set<std::string> loaded_assets;

  for (const auto &id_node : obstacle_ids) {
    std::string id = id_node.as<std::string>();
    auto obs = params["obstacles"][id];
    if (!obs) continue;

    std::string type = obs["type"].as<std::string>("box");
    auto pos = obs["position"];
    double px = pos[0].as<double>(0), py = pos[1].as<double>(0), pz = pos[2].as<double>(0);
    bool graspable = obs["graspable"].as<bool>(false);

    // RPY 弧度 (mj_saveLastXML 导出 angle="radian")
    std::string euler_attr;
    if (obs["orientation_rpy"]) {
      double r = obs["orientation_rpy"][0].as<double>(0);
      double p = obs["orientation_rpy"][1].as<double>(0);
      double y = obs["orientation_rpy"][2].as<double>(0);
      if (std::abs(r) > 1e-6 || std::abs(p) > 1e-6 || std::abs(y) > 1e-6) {
        std::ostringstream e;
        e << " euler=\"" << r << " " << p << " " << y << "\"";
        euler_attr = e.str();
      }
    }

    if (type == "urdf") {
      std::string urdf_uri = obs["urdf_path"].as<std::string>("");
      if (urdf_uri.empty()) continue;

      std::string child_mjcf;
      if (urdf_mjcf_cache.count(urdf_uri)) {
        child_mjcf = urdf_mjcf_cache[urdf_uri];
      } else {
        child_mjcf = loadObstacleURDF(id, urdf_uri);
        urdf_mjcf_cache[urdf_uri] = child_mjcf;
      }
      if (child_mjcf.empty()) continue;

      auto extract = [](const std::string &src, const std::string &tag) -> std::string {
        std::string open = "<" + tag + ">";
        std::string close = "</" + tag + ">";
        size_t s = src.find(open);
        size_t e = src.find(close);
        if (s == std::string::npos || e == std::string::npos) return "";
        return src.substr(s + open.size(), e - s - open.size());
      };

      std::string child_asset = extract(child_mjcf, "asset");
      std::string child_body = extract(child_mjcf, "worldbody");

      // 虚拟碰撞体 (直接定义或模板引用)
      YAML::Node vcol_shapes;
      if (obs["collision_shapes"] && obs["collision_shapes"].IsSequence()) {
        vcol_shapes = obs["collision_shapes"];
      } else if (obs["collision_shape_template"]) {
        std::string tpl_name = obs["collision_shape_template"].as<std::string>("");
        auto templates = params["collision_shape_templates"];
        if (templates && templates[tpl_name] && templates[tpl_name].IsSequence()) {
          vcol_shapes = templates[tpl_name];
        } else {
          RCLCPP_WARN(get_logger(), "[WARN] collision_shape_template '%s' not found for '%s'",
                      tpl_name.c_str(), id.c_str());
        }
      }
      bool use_vcol = vcol_shapes && vcol_shapes.IsSequence() && vcol_shapes.size() > 0;
      if (use_vcol) {
        size_t gp = 0;
        while ((gp = child_body.find("<geom", gp)) != std::string::npos) {
          size_t ge = child_body.find("/>", gp);
          if (ge == std::string::npos) break;
          child_body.insert(ge, " contype=\"0\" conaffinity=\"0\"");
          gp = ge + 35;
        }
        RCLCPP_INFO(this->get_logger(),
                    "[OK] '%s': STL mesh set visual-only, using virtual collision", id.c_str());
      }

      if (!child_asset.empty() && loaded_assets.find(urdf_uri) == loaded_assets.end()) {
        asset_ss << child_asset;
        has_asset = true;
        loaded_assets.insert(urdf_uri);
      }

      body_ss << "    <body name=\"obstacle_" << id << "\" pos=\"" << px << " " << py << " " << pz
              << "\"" << euler_attr << ">\n";
      if (graspable) {
        body_ss << "      <freejoint name=\"fj_" << id << "\"/>\n";
      }
      body_ss << child_body;

      if (use_vcol) {
        int si = 0;
        for (const auto &shape : vcol_shapes) {
          std::string st = shape["type"].as<std::string>("box");
          double sx = shape["position"][0].as<double>(0);
          double sy = shape["position"][1].as<double>(0);
          double sz = shape["position"][2].as<double>(0);
          std::string rgba = "1 0 0 0.15";
          if (shape["rgba"] && shape["rgba"].IsSequence()) {
            std::ostringstream rs;
            rs << shape["rgba"][0].as<double>(1) << " " << shape["rgba"][1].as<double>(0) << " "
               << shape["rgba"][2].as<double>(0) << " " << shape["rgba"][3].as<double>(0.15);
            rgba = rs.str();
          }
          std::ostringstream gs;
          gs << "      <geom name=\"vcol_" << id << "_" << si << "\" pos=\"" << sx << " " << sy
             << " " << sz << "\"";
          if (shape["orientation_rpy"] && shape["orientation_rpy"].IsSequence()) {
            double vr = shape["orientation_rpy"][0].as<double>(0);
            double vp = shape["orientation_rpy"][1].as<double>(0);
            double vy = shape["orientation_rpy"][2].as<double>(0);
            if (std::abs(vr) > 1e-6 || std::abs(vp) > 1e-6 || std::abs(vy) > 1e-6)
              gs << " euler=\"" << vr << " " << vp << " " << vy << "\"";
          }
          if (st == "box") {
            auto d = shape["dimensions"];
            gs << " type=\"box\" size=\"" << d[0].as<double>(0.1) / 2 << " "
               << d[1].as<double>(0.1) / 2 << " " << d[2].as<double>(0.1) / 2 << "\"";
          } else if (st == "cylinder") {
            auto d = shape["dimensions"];
            gs << " type=\"cylinder\" size=\"" << d[1].as<double>(0.02) << " "
               << d[0].as<double>(0.1) / 2 << "\"";
          } else if (st == "sphere") {
            gs << " type=\"sphere\" size=\"" << shape["dimensions"][0].as<double>(0.05) << "\"";
          }
          // graspable: ct=8(1000) ca=3(0011) → 碰臂+矿框, 矿核间不碰
          // static:    ct=2       ca=1       → 碰臂
          // 非弹性碰撞: solref 5ms+过阻尼, solimp 99%吸收
          if (graspable) {
            gs << " mass=\"0\" rgba=\"" << rgba
               << "\" contype=\"8\" conaffinity=\"3\" friction=\"0.01 0.005 0.0001\""
               << " solref=\"0.005 2\" solimp=\"0.99 0.99 0.0001 0.5 2\"/>\n";
          } else {
            gs << " mass=\"0\" rgba=\"" << rgba
               << "\" contype=\"2\" conaffinity=\"1\" friction=\"0.001 0.001 0.0001\""
               << " solref=\"0.005 2\" solimp=\"0.99 0.99 0.0001 0.5 2\"/>\n";
          }
          body_ss << gs.str();
          si++;
        }
      }

      body_ss << "    </body>\n";
    } else {
      // box / cylinder / sphere primitives
      std::string geom_attrs;
      if (type == "box") {
        auto d = obs["dimensions"];
        double hx = d[0].as<double>(1) / 2, hy = d[1].as<double>(1) / 2,
               hz = d[2].as<double>(1) / 2;
        std::ostringstream g;
        g << "type=\"box\" size=\"" << hx << " " << hy << " " << hz << "\"";
        geom_attrs = g.str();
      } else if (type == "cylinder") {
        auto d = obs["dimensions"];
        std::ostringstream g;
        g << "type=\"cylinder\" size=\"" << d[1].as<double>(0.1) << " " << (d[0].as<double>(1) / 2)
          << "\"";
        geom_attrs = g.str();
      } else if (type == "sphere") {
        std::ostringstream g;
        g << "type=\"sphere\" size=\"" << obs["dimensions"][0].as<double>(0.1) << "\"";
        geom_attrs = g.str();
      } else {
        continue;
      }

      body_ss << "    <body name=\"obstacle_" << id << "\" pos=\"" << px << " " << py << " " << pz
              << "\"" << euler_attr << ">\n";
      if (graspable) {
        body_ss << "      <freejoint name=\"fj_" << id << "\"/>\n";
      }
      // ground: ct=4 ca=9(1001) → 碰臂+矿核, 非弹性
      if (id == "ground_plane") {
        body_ss << "      <geom " << geom_attrs << " rgba=\"0.5 0.5 0.5 0.4\""
                << " contype=\"4\" conaffinity=\"9\""
                << " solref=\"0.005 2\" solimp=\"0.99 0.99 0.0001 0.5 2\"/>\n"
                << "    </body>\n";
      } else {
        body_ss << "      <geom " << geom_attrs << " rgba=\"0.5 0.5 0.5 0.4\"/>\n"
                << "    </body>\n";
      }
    }
    count++;
  }

  if (count == 0) return "";

  std::ostringstream mjcf;
  if (has_asset) {
    mjcf << "\n  <asset>\n" << asset_ss.str() << "  </asset>\n";
  }
  mjcf << "\n  <worldbody>\n" << body_ss.str() << "  </worldbody>\n";

  RCLCPP_INFO(this->get_logger(), "[OK] Built MJCF for %d obstacles", count);
  return mjcf.str();
}

void MuJoCoInterfaceNode::effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() != static_cast<size_t>(kAllJoints)) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Torque array size mismatch! Expected %d, got %zu",
                 kAllJoints, msg->data.size());
    return;
  }

  if (!received_first_command_) {
    received_first_command_ = true;
    RCLCPP_INFO(this->get_logger(),
                "[OK] First torque command received, MuJoCo simulation started");
  }

  command_rx_count_++;

  for (int i = 0; i < kAllJoints; i++) {
    data_->ctrl[i] = msg->data[i];
  }

  data_->ctrl[kSimJoints - 1] = -data_->ctrl[kAllJoints - 1];  // mimic gripper
}

void MuJoCoInterfaceNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() < static_cast<size_t>(kAllJoints)) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[WARN] JointState size < %d, ignoring", kAllJoints);
    return;
  }

  std::lock_guard<std::mutex> lock(sim_mutex_);
  for (int i = 0; i < kAllJoints; ++i) {
    data_->qpos[i] = msg->position[i];
  }
  data_->qpos[kSimJoints - 1] = -data_->qpos[kAllJoints - 1];  // mimic gripper
  // FK only (no physics step in digital-twin mode)
  mj_forward(model_, data_);
}

void MuJoCoInterfaceNode::simulationStep() {
  if (!received_first_command_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "[WARN] Waiting for torque command from controller...");
    publishJointStates();
    return;
  }

  if (paused_) {
    publishJointStates();
    return;
  }

  std::lock_guard<std::mutex> lock(sim_mutex_);
  applyMagnetForces();
  mj_step(model_, data_);

  // [SAFETY] NaN检测 (仅日志, TODO: 调试后恢复重置)
  if (data_->warning[mjWARN_BADQACC].number > 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                          "[SAFETY] QACC NaN detected (count=%d) — reset DISABLED for debug",
                          data_->warning[mjWARN_BADQACC].number);
    // TODO: 调试完毕后恢复以下重置逻辑
    // mj_resetData(model_, data_);
    // setInitialPose();
    // std::fill(data_->xfrc_applied, data_->xfrc_applied + 6 * model_->nbody, 0.0);
    // std::fill(data_->ctrl, data_->ctrl + model_->nu, 0.0);
    // collectMagnetAnchors();
    // received_first_command_ = false;
  }

  sim_step_count_++;
  publishJointStates();
}

void MuJoCoInterfaceNode::publishJointStates() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = this->now();

  // 7 joints (6 arm + 1 gripper), 不含 mimic
  for (int i = 0; i < kAllJoints; i++) {
    msg.name.push_back(joint_names_[i]);
    msg.position.push_back(data_->qpos[i]);
    msg.velocity.push_back(data_->qvel[i]);
  }

  joint_state_pub_->publish(msg);
}

bool MuJoCoInterfaceNode::initializeVisualization() {
  RCLCPP_INFO(this->get_logger(), "[INFO] Initializing MuJoCo visualization...");

  if (!glfwInit()) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] GLFW initialization failed");
    return false;
  }

  window_ = glfwCreateWindow(1200, 900, "MuJoCo - ARV Robot", nullptr, nullptr);
  if (!window_) {
    RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to create GLFW window");
    glfwTerminate();
    return false;
  }

  glfwSetWindowUserPointer(window_, this);
  glfwSetMouseButtonCallback(window_, mouseButtonCallback);
  glfwSetCursorPosCallback(window_, mouseMoveCallback);
  glfwSetScrollCallback(window_, scrollCallback);
  glfwSetKeyCallback(window_, keyCallback);

  mjv_defaultCamera(&cam_);
  mjv_defaultOption(&opt_);
  mjv_defaultScene(&scene_);
  mjr_defaultContext(&con_);

  opt_.flags[mjVIS_JOINT] = 0;
  opt_.flags[mjVIS_ACTUATOR] = 0;

  mjv_makeScene(model_, &scene_, 1000);

  cam_.azimuth = 90.0;
  cam_.elevation = -20.0;
  cam_.distance = 3.0;
  cam_.lookat[0] = 0.0;
  cam_.lookat[1] = 0.0;
  cam_.lookat[2] = 0.5;

  RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo visualization initialized");
  RCLCPP_INFO(this->get_logger(),
              "[INFO] Keys: Space-Pause | H-Hide UI | R-Reset Camera | ESC-Exit");

  return true;
}
void MuJoCoInterfaceNode::renderLoop() {
  RCLCPP_INFO(this->get_logger(), "[INFO] Render loop started");

  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  mjr_makeContext(model_, &con_, mjFONTSCALE_150);

  while (render_running_ && !glfwWindowShouldClose(window_)) {
    {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
    }

    mjrRect viewport;
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
    viewport.left = 0;
    viewport.bottom = 0;

    mjr_render(viewport, &scene_, &con_);

    if (show_ui_) {
      // 持锁快照, 避免 OpenGL 调用阻塞 sim_mutex_
      double snap_time, snap_qpos[6], snap_qvel[6], snap_ctrl[6];
      bool snap_paused;
      {
        std::lock_guard<std::mutex> lock(sim_mutex_);
        snap_time = data_->time;
        snap_paused = paused_.load();
        for (int i = 0; i < 6; i++) {
          snap_qpos[i] = data_->qpos[i];
          snap_qvel[i] = data_->qvel[i];
          snap_ctrl[i] = data_->ctrl[i];
        }
      }

      char status[256];
      const float margin_x = 10.0f / viewport.width;
      const float margin_y = 30.0f / viewport.height;
      const float line_step = 20.0f / viewport.height;
      int ln = 0;

      snprintf(status, sizeof(status), "Time: %.2f s | %s", snap_time,
               snap_paused ? "PAUSED" : "RUNNING");
      mjr_text(mjFONT_NORMAL, status, &con_, margin_x, 1.0f - margin_y - (ln++) * line_step, 1.0f,
               1.0f, 1.0f);

      snprintf(status, sizeof(status), "Pos: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad",
               snap_qpos[0], snap_qpos[1], snap_qpos[2], snap_qpos[3], snap_qpos[4], snap_qpos[5]);
      mjr_text(mjFONT_NORMAL, status, &con_, margin_x, 1.0f - margin_y - (ln++) * line_step, 1.0f,
               1.0f, 0.0f);

      snprintf(status, sizeof(status), "Vel: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s",
               snap_qvel[0], snap_qvel[1], snap_qvel[2], snap_qvel[3], snap_qvel[4], snap_qvel[5]);
      mjr_text(mjFONT_NORMAL, status, &con_, margin_x, 1.0f - margin_y - (ln++) * line_step, 0.0f,
               1.0f, 1.0f);

      if (!visualization_only_) {
        snprintf(status, sizeof(status), "Torque: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] Nm",
                 snap_ctrl[0], snap_ctrl[1], snap_ctrl[2], snap_ctrl[3], snap_ctrl[4],
                 snap_ctrl[5]);
        mjr_text(mjFONT_NORMAL, status, &con_, margin_x, 1.0f - margin_y - (ln++) * line_step, 1.0f,
                 0.5f, 0.0f);
      }

      mjr_text(mjFONT_NORMAL, "[Space]Pause [H]HideUI [R]ResetCam [C]Contacts [F]Forces [ESC]Exit",
               &con_, margin_x, 10.0f / viewport.height, 0.7f, 0.7f, 0.7f);
    }

    glfwSwapBuffers(window_);
    glfwPollEvents();
  }

  RCLCPP_INFO(this->get_logger(), "[INFO] Render loop ended");
}

// ========== GLFW callbacks ==========

void MuJoCoInterfaceNode::mouseButtonCallback(GLFWwindow *window, int button, int action,
                                              int mods) {
  auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
  if (!node) return;

  if (action == GLFW_PRESS) {
    if (button == GLFW_MOUSE_BUTTON_LEFT)
      node->button_left_ = true;
    else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
      node->button_middle_ = true;
    else if (button == GLFW_MOUSE_BUTTON_RIGHT)
      node->button_right_ = true;
    glfwGetCursorPos(window, &node->lastx_, &node->lasty_);
  } else if (action == GLFW_RELEASE) {
    node->button_left_ = false;
    node->button_middle_ = false;
    node->button_right_ = false;
  }
}

void MuJoCoInterfaceNode::mouseMoveCallback(GLFWwindow *window, double xpos, double ypos) {
  auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
  if (!node) return;

  if (!node->button_left_ && !node->button_middle_ && !node->button_right_) return;

  double dx = xpos - node->lastx_;
  double dy = ypos - node->lasty_;
  node->lastx_ = xpos;
  node->lasty_ = ypos;

  if (node->button_left_) {
    node->cam_.azimuth += dx * 0.3;
    node->cam_.elevation += dy * 0.3;
  } else if (node->button_right_) {
    double moveScale = 0.001 * node->cam_.distance;
    double azimuth_rad = node->cam_.azimuth * M_PI / 180.0;
    double right_x = -std::sin(azimuth_rad);
    double right_y = std::cos(azimuth_rad);

    node->cam_.lookat[0] -= moveScale * (dx * right_x);
    node->cam_.lookat[1] -= moveScale * (dx * right_y);
    node->cam_.lookat[2] += moveScale * dy;
  } else if (node->button_middle_) {
    node->cam_.distance *= (1.0 - dy * 0.01);
    if (node->cam_.distance < 0.1) node->cam_.distance = 0.1;
    if (node->cam_.distance > 50.0) node->cam_.distance = 50.0;
  }
}

void MuJoCoInterfaceNode::scrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
  auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
  if (!node) return;

  node->cam_.distance *= (1.0 - yoffset * 0.05);
  if (node->cam_.distance < 0.1) node->cam_.distance = 0.1;
  if (node->cam_.distance > 50.0) node->cam_.distance = 50.0;
}

void MuJoCoInterfaceNode::keyCallback(GLFWwindow *window, int key, int scancode, int action,
                                      int mods) {
  auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
  if (!node) return;

  if (action != GLFW_PRESS) return;

  switch (key) {
    case GLFW_KEY_SPACE:
      node->paused_ = !node->paused_;
      RCLCPP_INFO(node->get_logger(), node->paused_ ? "[INFO] Paused" : "[INFO] Resumed");
      break;

    case GLFW_KEY_H:
      node->show_ui_ = !node->show_ui_;
      break;

    case GLFW_KEY_C:
      node->show_contacts_ = !node->show_contacts_;
      node->opt_.flags[mjVIS_CONTACTPOINT] = node->show_contacts_;
      break;

    case GLFW_KEY_F:
      node->show_forces_ = !node->show_forces_;
      node->opt_.flags[mjVIS_CONTACTFORCE] = node->show_forces_;
      break;

    case GLFW_KEY_R: {
      std::lock_guard<std::mutex> lock(node->sim_mutex_);
      mj_resetData(node->model_, node->data_);
      node->setInitialPose();
      std::fill(node->data_->xfrc_applied, node->data_->xfrc_applied + 6 * node->model_->nbody,
                0.0);
      std::fill(node->data_->ctrl, node->data_->ctrl + node->model_->nu, 0.0);
      node->collectMagnetAnchors();
      node->sim_step_count_ = 0;
      node->received_first_command_ = false;
      RCLCPP_INFO(node->get_logger(), "[RESET] Simulation reset to initial pose (press R)");
      break;
    }

    case GLFW_KEY_V:
      node->cam_.azimuth = 90.0;
      node->cam_.elevation = -20.0;
      node->cam_.distance = 3.0;
      node->cam_.lookat[0] = 0.0;
      node->cam_.lookat[1] = 0.0;
      node->cam_.lookat[2] = 0.5;
      RCLCPP_INFO(node->get_logger(), "[INFO] Camera reset");
      break;

    case GLFW_KEY_ESCAPE:
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      break;
  }
}

void MuJoCoInterfaceNode::healthCheck() {
  static uint64_t last_step_count = 0;
  static uint64_t last_cmd_count = 0;

  uint64_t current_steps = sim_step_count_.load();
  uint64_t current_cmds = command_rx_count_.load();

  double step_rate = (current_steps - last_step_count) / 5.0;  // 5 sec interval
  double cmd_rate = (current_cmds - last_cmd_count) / 5.0;

  if (visualization_only_) {
    RCLCPP_INFO(this->get_logger(), "[HEALTH] Mode: DIGITAL TWIN | Render: %s",
                render_running_.load() ? "OK" : "STOPPED");
  } else {
    if (!received_first_command_.load()) {
      RCLCPP_WARN(this->get_logger(), "[HEALTH] Mode: SIMULATION | Waiting for first command...");
    } else if (paused_) {
      RCLCPP_INFO(this->get_logger(), "[HEALTH] Mode: SIMULATION | Status: PAUSED | Render: %s",
                  render_running_.load() ? "OK" : "STOPPED");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "[HEALTH] Mode: SIMULATION | Sim: %.1f Hz | Cmd RX: %.1f Hz | Render: %s",
                  step_rate, cmd_rate, render_running_.load() ? "OK" : "STOPPED");
    }
  }

  last_step_count = current_steps;
  last_cmd_count = current_cmds;
}

// ========== main ==========

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MuJoCoInterfaceNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("mujoco_interface"), "Node startup failed: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}