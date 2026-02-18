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
#include <tf2/LinearMath/Quaternion.h>
#include <urdf_parser/urdf_parser.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
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

  void declareParameters() {
    this->declare_parameter("reference_frame", "world");
    this->declare_parameter("auto_load", true);
    this->declare_parameter("obstacle_ids", std::vector<std::string>{});
  }

  // ========== 加载全部障碍物 ==========
  void loadAllObstacles() {
    auto ids = this->get_parameter("obstacle_ids").as_string_array();
    if (ids.empty()) {
      RCLCPP_WARN(get_logger(), "No obstacle_ids configured");
      return;
    }

    std::string ref_frame = this->get_parameter("reference_frame").as_string();
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    for (const auto& id : ids) {
      try {
        auto obj = loadSingleObstacle(id, ref_frame);
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

  // ========== 加载单个障碍物 ==========
  moveit_msgs::msg::CollisionObject loadSingleObstacle(const std::string& id,
                                                       const std::string& ref_frame) {
    std::string prefix = "obstacles." + id + ".";

    // 声明并读取参数（首次调用时声明）
    auto declare_if_needed = [&](const std::string& name, const auto& default_val) {
      if (!this->has_parameter(prefix + name)) {
        this->declare_parameter(prefix + name, default_val);
      }
    };

    declare_if_needed("type", std::string("box"));
    declare_if_needed("position", std::vector<double>{0, 0, 0});
    declare_if_needed("orientation_rpy", std::vector<double>{0, 0, 0});

    std::string type = this->get_parameter(prefix + "type").as_string();
    auto pos = this->get_parameter(prefix + "position").as_double_array();
    auto rpy = this->get_parameter(prefix + "orientation_rpy").as_double_array();

    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = ref_frame;
    obj.id = id;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    // 位姿
    geometry_msgs::msg::Pose pose;
    pose.position.x = pos[0];
    pose.position.y = pos[1];
    pose.position.z = pos[2];
    tf2::Quaternion q;
    q.setRPY(rpy[0], rpy[1], rpy[2]);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    if (type == "urdf") {
      loadURDFObstacle(obj, pose, id, prefix);
    } else if (type == "mesh") {
      loadMeshObstacle(obj, pose, id, prefix);
    } else {
      loadPrimitiveObstacle(obj, pose, type, id, prefix);
    }

    return obj;
  }

  // ========== URDF 障碍物 (SW-URDF导出, 与MuJoCo共用) ==========
  void loadURDFObstacle(moveit_msgs::msg::CollisionObject& obj,
                        const geometry_msgs::msg::Pose& world_pose, const std::string& id,
                        const std::string& prefix) {
    auto declare_if_needed = [&](const std::string& name, const auto& default_val) {
      if (!this->has_parameter(prefix + name)) {
        this->declare_parameter(prefix + name, default_val);
      }
    };

    declare_if_needed("urdf_path", std::string(""));
    std::string urdf_uri = this->get_parameter(prefix + "urdf_path").as_string();
    if (urdf_uri.empty()) {
      throw std::runtime_error("urdf_path is empty for obstacle: " + id);
    }

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

  // ========== Mesh 障碍物 (直接STL, 保留兼容) ==========
  void loadMeshObstacle(moveit_msgs::msg::CollisionObject& obj,
                        const geometry_msgs::msg::Pose& pose, const std::string& id,
                        const std::string& prefix) {
    auto declare_if_needed = [&](const std::string& name, const auto& default_val) {
      if (!this->has_parameter(prefix + name)) {
        this->declare_parameter(prefix + name, default_val);
      }
    };

    declare_if_needed("mesh_path", std::string(""));
    declare_if_needed("mesh_scale", std::vector<double>{1.0, 1.0, 1.0});

    std::string mesh_path = this->get_parameter(prefix + "mesh_path").as_string();
    auto scale_vec = this->get_parameter(prefix + "mesh_scale").as_double_array();

    if (mesh_path.empty()) {
      throw std::runtime_error("mesh_path is empty for obstacle: " + id);
    }

    Eigen::Vector3d scale(scale_vec[0], scale_vec[1], scale_vec[2]);
    shapes::Mesh* mesh = shapes::createMeshFromResource(mesh_path, scale);
    if (!mesh) {
      throw std::runtime_error("Failed to load mesh: " + mesh_path);
    }

    shapes::ShapeMsg shape_msg;
    shapes::constructMsgFromShape(mesh, shape_msg);
    obj.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    obj.mesh_poses.push_back(pose);

    delete mesh;
  }

  // ========== 基本几何体障碍物 ==========
  void loadPrimitiveObstacle(moveit_msgs::msg::CollisionObject& obj,
                             const geometry_msgs::msg::Pose& pose, const std::string& type,
                             const std::string& id, const std::string& prefix) {
    auto declare_if_needed = [&](const std::string& name, const auto& default_val) {
      if (!this->has_parameter(prefix + name)) {
        this->declare_parameter(prefix + name, default_val);
      }
    };

    declare_if_needed("dimensions", std::vector<double>{});

    auto dims = this->get_parameter(prefix + "dimensions").as_double_array();

    shape_msgs::msg::SolidPrimitive primitive;

    if (type == "box") {
      if (dims.size() != 3) {
        throw std::runtime_error("Box needs 3 dims [x,y,z] for: " + id);
      }
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {dims[0], dims[1], dims[2]};
    } else if (type == "cylinder") {
      if (dims.size() != 2) {
        throw std::runtime_error("Cylinder needs 2 dims [h,r] for: " + id);
      }
      primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      primitive.dimensions = {dims[0], dims[1]};
    } else if (type == "sphere") {
      if (dims.size() != 1) {
        throw std::runtime_error("Sphere needs 1 dim [r] for: " + id);
      }
      primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
      primitive.dimensions = {dims[0]};
    } else {
      throw std::runtime_error("Unknown obstacle type: " + type);
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
