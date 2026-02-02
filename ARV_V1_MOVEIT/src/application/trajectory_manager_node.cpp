/**
 * @file trajectory_manager_node.cpp
 * @brief 轨迹管理节点 - 提供轨迹保存、加载、执行功能
 *
 * Phase 1: 轨迹保存与加载
 * - /save_trajectory: 保存指定轨迹到YAML文件
 * - /save_last_trajectory: 保存最近执行的轨迹
 * - /load_trajectory: 加载并执行轨迹
 * - /list_trajectories: 列出已保存的轨迹
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "arv_v1_interfaces/srv/save_trajectory.hpp"
#include "arv_v1_interfaces/srv/save_last_trajectory.hpp"
#include "arv_v1_interfaces/srv/load_trajectory.hpp"
#include "arv_v1_interfaces/srv/list_trajectories.hpp"

#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <yaml-cpp/yaml.h>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class TrajectoryManagerNode : public rclcpp::Node
{
public:
    TrajectoryManagerNode()
        : Node("trajectory_manager_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("trajectory_dir", "");

        // 获取轨迹目录路径
        std::string traj_dir = this->get_parameter("trajectory_dir").as_string();
        if (traj_dir.empty()) {
            // 默认使用包内的config/trajectories目录
            traj_dir = "/home/huan/ros2_ws/src/ARV_V1_MOVEIT/config/trajectories";
        }
        trajectory_dir_ = traj_dir;

        // 确保目录存在
        if (!std::filesystem::exists(trajectory_dir_)) {
            std::filesystem::create_directories(trajectory_dir_);
            RCLCPP_INFO(this->get_logger(), "Created trajectory directory: %s", trajectory_dir_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Trajectory directory: %s", trajectory_dir_.c_str());

        // 初始化关节名称
        joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

        // 订阅关节状态
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryManagerNode::jointStateCallback, this, std::placeholders::_1));

        // 订阅MoveIt发送的轨迹（用于缓存最近执行的轨迹）
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/ARM_controller/joint_trajectory", 10,
            std::bind(&TrajectoryManagerNode::trajectoryCallback, this, std::placeholders::_1));

        // 创建Action Client
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/ARM_controller/follow_joint_trajectory");

        // 创建服务
        save_srv_ = this->create_service<arv_v1_interfaces::srv::SaveTrajectory>(
            "/save_trajectory",
            std::bind(&TrajectoryManagerNode::saveTrajectoryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        save_last_srv_ = this->create_service<arv_v1_interfaces::srv::SaveLastTrajectory>(
            "/save_last_trajectory",
            std::bind(&TrajectoryManagerNode::saveLastTrajectoryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        load_srv_ = this->create_service<arv_v1_interfaces::srv::LoadTrajectory>(
            "/load_trajectory",
            std::bind(&TrajectoryManagerNode::loadTrajectoryCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        list_srv_ = this->create_service<arv_v1_interfaces::srv::ListTrajectories>(
            "/list_trajectories",
            std::bind(&TrajectoryManagerNode::listTrajectoriesCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "     Trajectory Manager Node Started");
        RCLCPP_INFO(this->get_logger(), "==============================================");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Services:");
        RCLCPP_INFO(this->get_logger(), "  /save_last_trajectory - Save last executed trajectory");
        RCLCPP_INFO(this->get_logger(), "  /save_trajectory      - Save specified trajectory");
        RCLCPP_INFO(this->get_logger(), "  /load_trajectory      - Load and execute trajectory");
        RCLCPP_INFO(this->get_logger(), "  /list_trajectories    - List saved trajectories");
        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "Usage:");
        RCLCPP_INFO(this->get_logger(), "  1. Plan and execute in RViz");
        RCLCPP_INFO(this->get_logger(), "  2. Save: ros2 service call /save_last_trajectory \\");
        RCLCPP_INFO(this->get_logger(), "           arv_v1_interfaces/srv/SaveLastTrajectory \\");
        RCLCPP_INFO(this->get_logger(), "           \"{name: 'my_traj', description: 'desc'}\"");
        RCLCPP_INFO(this->get_logger(), "");
    }

private:
    // ========== 成员变量 ==========
    std::string trajectory_dir_;
    std::vector<std::string> joint_names_;
    std::array<double, 6> current_position_{};
    bool joint_state_received_ = false;
    std::mutex state_mutex_;

    // 缓存最近执行的轨迹
    trajectory_msgs::msg::JointTrajectory last_trajectory_;
    std::mutex trajectory_mutex_;
    bool has_last_trajectory_ = false;

    // ROS2通信
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

    rclcpp::Service<arv_v1_interfaces::srv::SaveTrajectory>::SharedPtr save_srv_;
    rclcpp::Service<arv_v1_interfaces::srv::SaveLastTrajectory>::SharedPtr save_last_srv_;
    rclcpp::Service<arv_v1_interfaces::srv::LoadTrajectory>::SharedPtr load_srv_;
    rclcpp::Service<arv_v1_interfaces::srv::ListTrajectories>::SharedPtr list_srv_;

    // ========== 回调函数 ==========

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // 按关节名称匹配更新位置
        for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
                if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
                    current_position_[j] = msg->position[i];
                }
            }
        }

        if (!joint_state_received_) {
            joint_state_received_ = true;
            RCLCPP_INFO(this->get_logger(), "First joint state received");
        }
    }

    void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        last_trajectory_ = *msg;
        has_last_trajectory_ = true;

        double duration = 0.0;
        if (!msg->points.empty()) {
            const auto& last_point = msg->points.back();
            duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
        }

        RCLCPP_INFO(this->get_logger(), "[Captured] Trajectory with %zu points, duration: %.2fs",
                   msg->points.size(), duration);
    }

    // ========== 轨迹保存核心函数 ==========

    bool saveTrajectoryToFile(const std::string& name, const std::string& description,
                              const trajectory_msgs::msg::JointTrajectory& trajectory,
                              std::string& saved_path, std::string& error_message)
    {
        // 构建文件路径
        std::string filename = trajectory_dir_ + "/" + name + ".yaml";

        try {
            // 构建YAML内容
            YAML::Emitter out;
            out << YAML::BeginMap;

            // Meta信息
            out << YAML::Key << "meta" << YAML::Value << YAML::BeginMap;
            out << YAML::Key << "name" << YAML::Value << name;
            out << YAML::Key << "description" << YAML::Value << description;

            // 获取当前时间
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
            out << YAML::Key << "saved_at" << YAML::Value << ss.str();

            // 计算总时长
            double duration = 0.0;
            if (!trajectory.points.empty()) {
                const auto& last_point = trajectory.points.back();
                duration = last_point.time_from_start.sec +
                          last_point.time_from_start.nanosec * 1e-9;
            }
            out << YAML::Key << "duration_sec" << YAML::Value << duration;
            out << YAML::EndMap;  // meta

            // 关节名称
            out << YAML::Key << "joint_names" << YAML::Value << YAML::BeginSeq;
            if (!trajectory.joint_names.empty()) {
                for (const auto& jname : trajectory.joint_names) {
                    out << jname;
                }
            } else {
                for (const auto& jname : joint_names_) {
                    out << jname;
                }
            }
            out << YAML::EndSeq;

            // 起始位置
            if (!trajectory.points.empty()) {
                out << YAML::Key << "start_position" << YAML::Value << YAML::Flow;
                out << YAML::BeginSeq;
                for (const auto& pos : trajectory.points[0].positions) {
                    out << pos;
                }
                out << YAML::EndSeq;
            }

            // 轨迹点
            out << YAML::Key << "points" << YAML::Value << YAML::BeginSeq;
            for (const auto& point : trajectory.points) {
                out << YAML::BeginMap;

                double time = point.time_from_start.sec +
                             point.time_from_start.nanosec * 1e-9;
                out << YAML::Key << "time" << YAML::Value << time;

                out << YAML::Key << "positions" << YAML::Value << YAML::Flow;
                out << YAML::BeginSeq;
                for (const auto& pos : point.positions) {
                    out << pos;
                }
                out << YAML::EndSeq;

                if (!point.velocities.empty()) {
                    out << YAML::Key << "velocities" << YAML::Value << YAML::Flow;
                    out << YAML::BeginSeq;
                    for (const auto& vel : point.velocities) {
                        out << vel;
                    }
                    out << YAML::EndSeq;
                }

                if (!point.accelerations.empty()) {
                    out << YAML::Key << "accelerations" << YAML::Value << YAML::Flow;
                    out << YAML::BeginSeq;
                    for (const auto& acc : point.accelerations) {
                        out << acc;
                    }
                    out << YAML::EndSeq;
                }

                out << YAML::EndMap;
            }
            out << YAML::EndSeq;  // points

            out << YAML::EndMap;  // root

            // 写入文件
            std::ofstream fout(filename);
            if (!fout.is_open()) {
                error_message = "Failed to open file for writing: " + filename;
                return false;
            }
            fout << out.c_str();
            fout.close();

            saved_path = filename;

            RCLCPP_INFO(this->get_logger(), "[Save] Saved to: %s (%zu points, %.2fs)",
                       filename.c_str(), trajectory.points.size(), duration);

            return true;

        } catch (const std::exception& e) {
            error_message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "[Save] Error: %s", e.what());
            return false;
        }
    }

    // ========== 服务实现 ==========

    void saveTrajectoryCallback(
        const std::shared_ptr<arv_v1_interfaces::srv::SaveTrajectory::Request> request,
        std::shared_ptr<arv_v1_interfaces::srv::SaveTrajectory::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[SaveTrajectory] Saving: %s", request->name.c_str());

        // 验证输入
        if (request->name.empty()) {
            response->success = false;
            response->message = "Trajectory name cannot be empty";
            return;
        }

        if (request->trajectory.points.empty()) {
            response->success = false;
            response->message = "Trajectory has no points";
            return;
        }

        std::string saved_path, error_message;
        if (saveTrajectoryToFile(request->name, request->description,
                                 request->trajectory, saved_path, error_message)) {
            response->success = true;
            response->message = "Trajectory saved successfully";
            response->saved_path = saved_path;
        } else {
            response->success = false;
            response->message = error_message;
        }
    }

    void saveLastTrajectoryCallback(
        const std::shared_ptr<arv_v1_interfaces::srv::SaveLastTrajectory::Request> request,
        std::shared_ptr<arv_v1_interfaces::srv::SaveLastTrajectory::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[SaveLastTrajectory] Saving: %s", request->name.c_str());

        // 验证输入
        if (request->name.empty()) {
            response->success = false;
            response->message = "Trajectory name cannot be empty";
            return;
        }

        // 获取缓存的轨迹
        trajectory_msgs::msg::JointTrajectory trajectory_to_save;
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (!has_last_trajectory_ || last_trajectory_.points.empty()) {
                response->success = false;
                response->message = "No trajectory captured. Please execute a trajectory in RViz first.";
                return;
            }
            trajectory_to_save = last_trajectory_;
        }

        std::string saved_path, error_message;
        if (saveTrajectoryToFile(request->name, request->description,
                                 trajectory_to_save, saved_path, error_message)) {
            response->success = true;
            response->message = "Last trajectory saved successfully";
            response->saved_path = saved_path;
        } else {
            response->success = false;
            response->message = error_message;
        }
    }

    void loadTrajectoryCallback(
        const std::shared_ptr<arv_v1_interfaces::srv::LoadTrajectory::Request> request,
        std::shared_ptr<arv_v1_interfaces::srv::LoadTrajectory::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Loading: %s, execute: %s",
                   request->name.c_str(), request->execute ? "true" : "false");

        // 构建文件路径
        std::string filename = trajectory_dir_ + "/" + request->name + ".yaml";

        if (!std::filesystem::exists(filename)) {
            response->success = false;
            response->message = "Trajectory file not found: " + filename;
            return;
        }

        try {
            // 加载YAML
            YAML::Node config = YAML::LoadFile(filename);

            trajectory_msgs::msg::JointTrajectory trajectory;

            // 解析关节名称
            if (config["joint_names"]) {
                for (const auto& name : config["joint_names"]) {
                    trajectory.joint_names.push_back(name.as<std::string>());
                }
            } else {
                trajectory.joint_names = joint_names_;
            }

            // 解析轨迹点
            double duration = 0.0;
            if (config["points"]) {
                for (const auto& pt : config["points"]) {
                    trajectory_msgs::msg::JointTrajectoryPoint point;

                    double time = pt["time"].as<double>();
                    point.time_from_start = rclcpp::Duration::from_seconds(time);
                    duration = std::max(duration, time);

                    for (const auto& pos : pt["positions"]) {
                        point.positions.push_back(pos.as<double>());
                    }

                    if (pt["velocities"]) {
                        for (const auto& vel : pt["velocities"]) {
                            point.velocities.push_back(vel.as<double>());
                        }
                    }

                    if (pt["accelerations"]) {
                        for (const auto& acc : pt["accelerations"]) {
                            point.accelerations.push_back(acc.as<double>());
                        }
                    }

                    trajectory.points.push_back(point);
                }
            }

            response->duration = duration;

            RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Loaded %zu points, duration: %.2fs",
                       trajectory.points.size(), duration);

            // 如果需要执行
            if (request->execute) {
                if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
                    response->success = false;
                    response->message = "Action server not available";
                    return;
                }

                // 发送轨迹
                auto goal = FollowJointTrajectory::Goal();
                goal.trajectory = trajectory;

                auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

                send_goal_options.goal_response_callback =
                    [this](const GoalHandleFJT::SharedPtr& goal_handle) {
                        if (!goal_handle) {
                            RCLCPP_ERROR(this->get_logger(), "[LoadTrajectory] Goal rejected");
                        } else {
                            RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Goal accepted, executing...");
                        }
                    };

                send_goal_options.result_callback =
                    [this](const GoalHandleFJT::WrappedResult& result) {
                        switch (result.code) {
                            case rclcpp_action::ResultCode::SUCCEEDED:
                                RCLCPP_INFO(this->get_logger(), "[LoadTrajectory] Execution completed successfully");
                                break;
                            case rclcpp_action::ResultCode::ABORTED:
                                RCLCPP_ERROR(this->get_logger(), "[LoadTrajectory] Execution aborted");
                                break;
                            case rclcpp_action::ResultCode::CANCELED:
                                RCLCPP_WARN(this->get_logger(), "[LoadTrajectory] Execution canceled");
                                break;
                            default:
                                RCLCPP_ERROR(this->get_logger(), "[LoadTrajectory] Unknown result code");
                                break;
                        }
                    };

                action_client_->async_send_goal(goal, send_goal_options);

                response->success = true;
                response->message = "Trajectory loaded and execution started";
            } else {
                response->success = true;
                response->message = "Trajectory loaded (not executed)";
            }

        } catch (const std::exception& e) {
            response->success = false;
            response->message = std::string("Exception: ") + e.what();
            RCLCPP_ERROR(this->get_logger(), "[LoadTrajectory] Error: %s", e.what());
        }
    }

    void listTrajectoriesCallback(
        const std::shared_ptr<arv_v1_interfaces::srv::ListTrajectories::Request> /*request*/,
        std::shared_ptr<arv_v1_interfaces::srv::ListTrajectories::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "[ListTrajectories] Scanning: %s", trajectory_dir_.c_str());

        try {
            for (const auto& entry : std::filesystem::directory_iterator(trajectory_dir_)) {
                if (entry.path().extension() == ".yaml") {
                    std::string name = entry.path().stem().string();
                    std::string description = "";

                    // 尝试读取描述
                    try {
                        YAML::Node config = YAML::LoadFile(entry.path().string());
                        if (config["meta"] && config["meta"]["description"]) {
                            description = config["meta"]["description"].as<std::string>();
                        }
                    } catch (...) {
                        description = "(unable to read description)";
                    }

                    response->names.push_back(name);
                    response->descriptions.push_back(description);
                }
            }

            RCLCPP_INFO(this->get_logger(), "[ListTrajectories] Found %zu trajectories",
                       response->names.size());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "[ListTrajectories] Error: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
