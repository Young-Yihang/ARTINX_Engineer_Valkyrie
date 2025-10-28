// 解算核心部分
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
#include <mutex>
#include <fstream>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include "dynamics_computer.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class TorqueControllerActionServer : public rclcpp::Node // 节点类生成
{
public: // 构造函数log
    TorqueControllerActionServer() : Node("torque_controller_action_server"),
                                     is_executing_(false),
                                     q_actual_(6),
                                     q_dot_actual_(6),
                                     state_received_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 力矩控制器节点启动");

        // ========== 初始化动力学求解器 ==========
        if (!initializeDynamics())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ 动力学求解器初始化失败");
            throw std::runtime_error("Failed to initialize dynamics");
        }

        // ========== 订阅关节状态 ==========
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",                                             // 话题名称
            10,                                                          // 队列大小
            std::bind(&TorqueControllerActionServer::jointStateCallback, // 把成员回调函数绑定
                      this,
                      std::placeholders::_1));

        // ========== 创建 Action Server ==========
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "follow_joint_trajectory",                                                                                /// 回调函数被传递给ros2服务器系统                                                                           // Action 名称，用于和moveit的action对齐
            std::bind(&TorqueControllerActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2), // 回调函数，收到新目标
            std::bind(&TorqueControllerActionServer::handleCancel, this, std::placeholders::_1),                      // 回调函数，收到取消请求
            std::bind(&TorqueControllerActionServer::handleAccepted, this, std::placeholders::_1));                   // 回调函数，决定开始执行力矩计算器

        RCLCPP_INFO(this->get_logger(), "话题订阅！ 📡 Action Server 已创建: /follow_joint_trajectory");
    }

    ~TorqueControllerActionServer()
    {
        RCLCPP_INFO(this->get_logger(), "动力学力矩计算正在析构...程序结束。");
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
    std::mutex state_mutex_;     // 保护共享数据
    bool state_received_;        // 是否收到过状态

    KDL::Chain kdl_chain_;                               // 运动链实例
    std::unique_ptr<DynamicsComputer> dynamic_computer_; // 动力学解算器实例

    // Action 回调函数
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal);

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle);

    void handleAccepted(
        const std::shared_ptr<GoalHandleFJT> goal_handle);

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg); // 关节状态回调

    bool initializeDynamics(); // 动力学初始化
};

rclcpp_action::GoalResponse TorqueControllerActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    (void)uuid; // 未使用的参数

    RCLCPP_INFO(this->get_logger(), "!!!!收到新轨迹目标!!!!!");
    RCLCPP_INFO(this->get_logger(), "   轨迹点数: %zu", goal->trajectory.points.size());

    if (is_executing_)
    {
        RCLCPP_WARN(this->get_logger(), "已有轨迹在执行，拒绝新目标");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 简单验证：至少有一个点
    if (goal->trajectory.points.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 轨迹为空，拒绝目标");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "收到目标，准备存入数组");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TorqueControllerActionServer::handleCancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "！规划被人为取消！");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TorqueControllerActionServer::handleAccepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "🎯 目标已接受，准备执行");

    if (!state_received_)
    { // 确认关节状态正确
        RCLCPP_ERROR(this->get_logger(), "❌ 未收到关节状态数据，拒绝执行");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
        goal_handle->abort(result);
        return;
    }

    const auto goal = goal_handle->get_goal();
    current_trajectory_ = goal->trajectory; // 往对象成员变量里存入轨迹

    current_goal_handle_ = goal_handle;
    trajectory_start_time_ = this->now();
    is_executing_ = true;
    if (!current_trajectory_.points.empty()) // 输出信息
    {
        const auto &first_point = current_trajectory_.points[0];
        RCLCPP_INFO(this->get_logger(), "   - 第一个点 (t=%.3f):",
                    first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9);
        RCLCPP_INFO(this->get_logger(), "     位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    first_point.positions[0], first_point.positions[1], first_point.positions[2],
                    first_point.positions[3], first_point.positions[4], first_point.positions[5]);
    }

    const auto &last_point = current_trajectory_.points.back();
    double total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;
    RCLCPP_INFO(this->get_logger(), "   - 最后一个点 (t=%.3f):", total_duration);
    RCLCPP_INFO(this->get_logger(), "     位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                last_point.positions[0], last_point.positions[1], last_point.positions[2],
                last_point.positions[3], last_point.positions[4], last_point.positions[5]);
    RCLCPP_INFO(this->get_logger(), "   - 总时长: %.3f 秒", total_duration);

    // 现在先简单返回成功
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "✅ 轨迹执行完成（当前为测试）");
}

void TorqueControllerActionServer::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 线程安全：加锁
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 首次接收数据
    if (!state_received_)
    {
        RCLCPP_INFO(this->get_logger(), "✅ 首次接收关节状态数据");
        state_received_ = true;
    }

    // 验证数据
    if (msg->position.size() != 6 || msg->velocity.size() != 6)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000, // 每秒最多打印一次
            "⚠️  关节状态数据不完整: position=%zu, velocity=%zu",
            msg->position.size(), msg->velocity.size());
        return;
    }

    // 提取位置和速度
    for (size_t i = 0; i < 6; i++)
    {
        q_actual_(i) = msg->position[i];
        q_dot_actual_(i) = msg->velocity[i];
    }

    // 调试输出（限流）
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // 每 5 秒打印一次
        "📊 关节状态: q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        q_actual_(0), q_actual_(1), q_actual_(2),
        q_actual_(3), q_actual_(4), q_actual_(5));
}

bool TorqueControllerActionServer::initializeDynamics()
{
    RCLCPP_INFO(this->get_logger(), "📦 开始初始化动力学求解器...");
    
    // 1. 读取 URDF 文件
    std::string urdf_path = "/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/ARV_V1_MODEL.urdf";
    std::ifstream urdf_file(urdf_path);
    if (!urdf_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法打开 URDF 文件: %s", urdf_path.c_str());
        return false;
    }
    
    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                            std::istreambuf_iterator<char>());
    urdf_file.close();
        
    // 2. 解析 URDF
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string)) {
        RCLCPP_ERROR(this->get_logger(), "❌ URDF 解析失败");
        return false;
    }
    
    // 3. 提取 KDL 树
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
        RCLCPP_ERROR(this->get_logger(), "❌ 从 URDF 构建 KDL 树失败");
        return false;
    }
    
    // 4. 获取运动链（从 base_link 到 link6_2006roll）
    if (!kdl_tree.getChain("base_link", "link6_2006roll", kdl_chain_)) {
        RCLCPP_ERROR(this->get_logger(), "❌ 提取运动链失败");
        return false;
    }
    
    
    // 5. 创建动力学计算工具
    KDL::Vector gravity(0.0, 0.0, -9.81);  // 重力向量
    dynamic_computer_ = std::make_unique<DynamicsComputer>(kdl_chain_, gravity);
    
    RCLCPP_INFO(this->get_logger(), "✅ 动力学求解器初始化完成");
    RCLCPP_INFO(this->get_logger(), "   - 重力: [%.2f, %.2f, %.2f] m/s²", 
                gravity.x(), gravity.y(), gravity.z());

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TorqueControllerActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}