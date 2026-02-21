/**
 * @file scene_manager_node.cpp
 * @brief 赛场障碍物场景管理节点 (Application Layer)
 *
 * 功能：
 *   - 启动后从 YAML 参数加载障碍物到 MoveIt2 PlanningScene
 *   - 支持 box / cylinder / sphere / urdf 四种类型
 *   - urdf 类型复用 SW-URDF 导出的模型 (与 MuJoCo 侧共用)
 *   - 提供 service 接口：reload / clear 场景
 *   - graspable 物体支持 attach/detach 到末端执行器
 *
 * 依赖：moveit_ros_planning_interface, geometric_shapes, shape_msgs, tf2, urdf
 */

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

class SceneManagerNode : public rclcpp::Node {
public:
  SceneManagerNode() : Node("scene_manager") {
    declareParameters();

    // PlanningSceneInterface 通过 topic 与 move_group 通信
    planning_scene_interface_ =
        std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    // 加载 YAML 配置 (collision_shapes 需要 yaml-cpp, ROS2 params 不支持嵌套列表)
    loadSceneYAML();

    // 服务：重新加载 / 清空场景
    reload_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/reload_scene", std::bind(&SceneManagerNode::reloadCallback, this, std::placeholders::_1,
                                    std::placeholders::_2));

    clear_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_scene", std::bind(&SceneManagerNode::clearCallback, this, std::placeholders::_1,
                                   std::placeholders::_2));

    // 延迟加载，等 move_group 起来
    bool auto_load = this->get_parameter("auto_load").as_bool();
    if (auto_load) {
      init_timer_ = this->create_wall_timer(std::chrono::seconds(2), [this]() {
        init_timer_->cancel();
        loadAllObstacles();
      });
    }

    RCLCPP_INFO(get_logger(), "SceneManager initialized (auto_load=%s)",
                auto_load ? "true" : "false");
  }

private:
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  YAML::Node scene_yaml_;  // collision_shapes 直接从 YAML 读取

  // 加载场景 YAML (用于 collision_shapes)
  void loadSceneYAML() {
    // 尝试从 arv_v1_moveit 包路径查找默认配置
    try {
      auto moveit_share = ament_index_cpp::get_package_share_directory("arv_v1_moveit");
      std::string yaml_path = moveit_share + "/config/scene_obstacles.yaml";
      if (std::filesystem::exists(yaml_path)) {
        scene_yaml_ = YAML::LoadFile(yaml_path);
        RCLCPP_INFO(get_logger(), "Loaded scene YAML: %s", yaml_path.c_str());
        return;
      }
    } catch (...) {
    }
    // 尝试源码目录
    try {
      std::string src_path = std::string(std::getenv("HOME") ? std::getenv("HOME") : "") +
                             "/ros2_ws/src/arv_v1_moveit/config/scene_obstacles.yaml";
      if (std::filesystem::exists(src_path)) {
        scene_yaml_ = YAML::LoadFile(src_path);
        RCLCPP_INFO(get_logger(), "Loaded scene YAML (src): %s", src_path.c_str());
      }
    } catch (...) {
    }
  }

  void declareParameters() {
    this->declare_parameter("reference_frame", "world");
    this->declare_parameter("auto_load", true);
    this->declare_parameter("obstacle_ids", std::vector<std::string>{});
  }

  // ========== 加载全部障碍物 (从 yaml-cpp 读取, 不依赖 ROS2 params) ==========
  void loadAllObstacles() {
    if (scene_yaml_.IsNull()) {
      RCLCPP_WARN(get_logger(), "No scene YAML loaded, skipping obstacle injection");
      return;
    }

    auto params = scene_yaml_["scene_manager"]["ros__parameters"];
    if (!params) {
      RCLCPP_WARN(get_logger(), "No ros__parameters in scene YAML");
      return;
    }

    std::string ref_frame = params["reference_frame"].as<std::string>("world");
    auto obstacle_ids = params["obstacle_ids"];
    if (!obstacle_ids || !obstacle_ids.IsSequence() || obstacle_ids.size() == 0) {
      RCLCPP_WARN(get_logger(), "No obstacle_ids in scene YAML");
      return;
    }

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    for (const auto& id_node : obstacle_ids) {
      std::string id = id_node.as<std::string>();
      auto obs = params["obstacles"][id];
      if (!obs) continue;
      try {
        auto obj = loadSingleObstacleFromYAML(id, obs, ref_frame);
        objects.push_back(obj);
        RCLCPP_INFO(get_logger(), "Loaded obstacle: %s", id.c_str());
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to load '%s': %s", id.c_str(), e.what());
      }
    }

    if (!objects.empty()) {
      planning_scene_interface_->addCollisionObjects(objects);
      RCLCPP_INFO(get_logger(), "Added %zu obstacles to planning scene", objects.size());
    }
  }

  // ========== 从 YAML 加载单个障碍物 ==========
  moveit_msgs::msg::CollisionObject loadSingleObstacleFromYAML(const std::string& id,
                                                               const YAML::Node& obs,
                                                               const std::string& ref_frame) {
    std::string type = obs["type"].as<std::string>("box");
    double px = obs["position"][0].as<double>(0);
    double py = obs["position"][1].as<double>(0);
    double pz = obs["position"][2].as<double>(0);
    double rr = 0, rp = 0, ry = 0;
    if (obs["orientation_rpy"] && obs["orientation_rpy"].IsSequence()) {
      rr = obs["orientation_rpy"][0].as<double>(0);
      rp = obs["orientation_rpy"][1].as<double>(0);
      ry = obs["orientation_rpy"][2].as<double>(0);
    }

    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = ref_frame;
    obj.id = id;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    geometry_msgs::msg::Pose pose;
    pose.position.x = px;
    pose.position.y = py;
    pose.position.z = pz;
    tf2::Quaternion q;
    q.setRPY(rr, rp, ry);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    if (type == "urdf") {
      std::string urdf_uri = obs["urdf_path"].as<std::string>("");
      if (!urdf_uri.empty()) {
        loadURDFObstacleFromPath(obj, pose, id, urdf_uri);
      }
    } else if (type == "box" || type == "cylinder" || type == "sphere") {
      loadPrimitiveFromYAML(obj, pose, type, id, obs);
    }

    // 追加 YAML collision_shapes 虚拟碰撞体 (若有)
    addCollisionShapes(obj, pose, id);

    return obj;
  }

  // ========== URDF 障碍物 (SW-URDF导出, 与MuJoCo共用) ==========
  void loadURDFObstacleFromPath(moveit_msgs::msg::CollisionObject& obj,
                                const geometry_msgs::msg::Pose& world_pose, const std::string& id,
                                const std::string& urdf_uri) {
    // 解析 URDF，提取 collision mesh
    auto model = urdf::parseURDFFile(resolvePackageURI(urdf_uri));
    if (!model) {
      throw std::runtime_error("Failed to parse URDF: " + urdf_uri);
    }

    // 遍历所有 link 的 collision 几何体
    for (const auto& [link_name, link] : model->links_) {
      for (const auto& collision : link->collision_array) {
        if (!collision || !collision->geometry) continue;

        auto geom = collision->geometry;
        geometry_msgs::msg::Pose link_pose = world_pose;

        // 叠加 collision origin 偏移
        if (collision->origin.position.x != 0 || collision->origin.position.y != 0 ||
            collision->origin.position.z != 0) {
          link_pose.position.x += collision->origin.position.x;
          link_pose.position.y += collision->origin.position.y;
          link_pose.position.z += collision->origin.position.z;
        }

        if (geom->type == urdf::Geometry::MESH) {
          auto mesh_geom = std::dynamic_pointer_cast<urdf::Mesh>(geom);
          if (!mesh_geom) continue;

          Eigen::Vector3d scale(mesh_geom->scale.x, mesh_geom->scale.y, mesh_geom->scale.z);
          std::string mesh_path = resolvePackageURI(mesh_geom->filename);
          shapes::Mesh* mesh = shapes::createMeshFromResource("file://" + mesh_path, scale);
          if (!mesh) {
            RCLCPP_WARN(get_logger(), "Failed to load mesh: %s", mesh_path.c_str());
            continue;
          }

          shapes::ShapeMsg shape_msg;
          shapes::constructMsgFromShape(mesh, shape_msg);
          obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
          obj.mesh_poses.push_back(link_pose);
          delete mesh;
        } else if (geom->type == urdf::Geometry::BOX) {
          auto box = std::dynamic_pointer_cast<urdf::Box>(geom);
          shape_msgs::msg::SolidPrimitive prim;
          prim.type = shape_msgs::msg::SolidPrimitive::BOX;
          prim.dimensions = {box->dim.x, box->dim.y, box->dim.z};
          obj.primitives.push_back(prim);
          obj.primitive_poses.push_back(link_pose);
        } else if (geom->type == urdf::Geometry::CYLINDER) {
          auto cyl = std::dynamic_pointer_cast<urdf::Cylinder>(geom);
          shape_msgs::msg::SolidPrimitive prim;
          prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          prim.dimensions = {cyl->length, cyl->radius};
          obj.primitives.push_back(prim);
          obj.primitive_poses.push_back(link_pose);
        } else if (geom->type == urdf::Geometry::SPHERE) {
          auto sph = std::dynamic_pointer_cast<urdf::Sphere>(geom);
          shape_msgs::msg::SolidPrimitive prim;
          prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
          prim.dimensions = {sph->radius};
          obj.primitives.push_back(prim);
          obj.primitive_poses.push_back(link_pose);
        }
      }
    }

    if (obj.meshes.empty() && obj.primitives.empty()) {
      throw std::runtime_error("No collision geometry found in URDF: " + urdf_uri);
    }
  }

  // ========== 从 YAML collision_shapes 添加虚拟碰撞体 ==========
  void addCollisionShapes(moveit_msgs::msg::CollisionObject& obj,
                          const geometry_msgs::msg::Pose& world_pose, const std::string& id) {
    if (scene_yaml_.IsNull()) return;
    auto params = scene_yaml_["scene_manager"]["ros__parameters"];
    if (!params) return;
    auto obs = params["obstacles"][id];
    if (!obs || !obs["collision_shapes"] || !obs["collision_shapes"].IsSequence()) return;

    // 父体姿态的四元数 (用于旋转局部偏移到世界坐标)
    tf2::Quaternion world_q(world_pose.orientation.x, world_pose.orientation.y,
                            world_pose.orientation.z, world_pose.orientation.w);
    tf2::Matrix3x3 world_rot(world_q);

    for (const auto& shape : obs["collision_shapes"]) {
      std::string st = shape["type"].as<std::string>("box");
      double lx = shape["position"][0].as<double>(0);
      double ly = shape["position"][1].as<double>(0);
      double lz = shape["position"][2].as<double>(0);

      // 局部位置 → 世界坐标: world_pos + world_rot * local_pos
      tf2::Vector3 local_pos(lx, ly, lz);
      tf2::Vector3 rotated = world_rot * local_pos;

      // 局部姿态
      tf2::Quaternion local_q;
      local_q.setRPY(0, 0, 0);
      if (shape["orientation_rpy"] && shape["orientation_rpy"].IsSequence()) {
        double r = shape["orientation_rpy"][0].as<double>(0);
        double p = shape["orientation_rpy"][1].as<double>(0);
        double y = shape["orientation_rpy"][2].as<double>(0);
        local_q.setRPY(r, p, y);
      }
      tf2::Quaternion final_q = world_q * local_q;
      final_q.normalize();

      geometry_msgs::msg::Pose shape_pose;
      shape_pose.position.x = world_pose.position.x + rotated.x();
      shape_pose.position.y = world_pose.position.y + rotated.y();
      shape_pose.position.z = world_pose.position.z + rotated.z();
      shape_pose.orientation.x = final_q.x();
      shape_pose.orientation.y = final_q.y();
      shape_pose.orientation.z = final_q.z();
      shape_pose.orientation.w = final_q.w();

      shape_msgs::msg::SolidPrimitive prim;
      if (st == "box") {
        auto d = shape["dimensions"];
        prim.type = shape_msgs::msg::SolidPrimitive::BOX;
        prim.dimensions = {d[0].as<double>(0.1), d[1].as<double>(0.1), d[2].as<double>(0.1)};
      } else if (st == "cylinder") {
        auto d = shape["dimensions"];
        prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        prim.dimensions = {d[0].as<double>(0.1), d[1].as<double>(0.05)};
      } else if (st == "sphere") {
        prim.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        prim.dimensions = {shape["dimensions"][0].as<double>(0.05)};
      } else {
        continue;
      }

      obj.primitives.push_back(prim);
      obj.primitive_poses.push_back(shape_pose);
    }

    RCLCPP_INFO(get_logger(), "'%s': added %zu collision shapes from YAML", id.c_str(),
                obs["collision_shapes"].size());
  }

  // 解析 package:// URI → 磁盘绝对路径
  std::string resolvePackageURI(const std::string& uri) {
    const std::string prefix = "package://";
    if (uri.find(prefix) != 0) return uri;

    size_t slash = uri.find('/', prefix.size());
    if (slash == std::string::npos) return uri;

    std::string pkg = uri.substr(prefix.size(), slash - prefix.size());
    try {
      std::string pkg_dir = ament_index_cpp::get_package_share_directory(pkg);
      return pkg_dir + uri.substr(slash);
    } catch (...) {
      return uri;
    }
  }

  // ========== 基本几何体 (从 YAML Node 读取) ==========
  void loadPrimitiveFromYAML(moveit_msgs::msg::CollisionObject& obj,
                             const geometry_msgs::msg::Pose& pose, const std::string& type,
                             const std::string& id, const YAML::Node& obs) {
    shape_msgs::msg::SolidPrimitive primitive;
    auto dims = obs["dimensions"];
    if (!dims || !dims.IsSequence()) {
      throw std::runtime_error("Missing dimensions for: " + id);
    }

    if (type == "box") {
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {dims[0].as<double>(), dims[1].as<double>(), dims[2].as<double>()};
    } else if (type == "cylinder") {
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {dims[0].as<double>(), dims[1].as<double>()};
    } else if (type == "sphere") {
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions = {dims[0].as<double>()};
    } else {
      throw std::runtime_error("Unknown type: " + type);
    }

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
  }

  // ========== Service: 重新加载 ==========
  void reloadCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    try {
      clearAllObjects();
      loadAllObstacles();
      res->success = true;
      res->message = "Scene reloaded";
    } catch (const std::exception& e) {
      res->success = false;
      res->message = std::string("Reload failed: ") + e.what();
    }
  }

  // ========== Service: 清空 ==========
  void clearCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
    clearAllObjects();
    res->success = true;
    res->message = "Scene cleared";
  }

  void clearAllObjects() {
    auto names = planning_scene_interface_->getKnownObjectNames();
    if (!names.empty()) {
      planning_scene_interface_->removeCollisionObjects(names);
      RCLCPP_INFO(get_logger(), "Removed %zu objects from scene", names.size());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SceneManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
