// 解算核心部分
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
#include <mutex>
#include <fstream>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> // 力矩数组消息
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
                                     state_received_(false),
                                     Kp_(6),
                                     Kd_(6),
                                     control_frequency_(200.0)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 力矩控制器节点启动");

        // ========== 初始化 PD 增益 (新增) ==========
        // 位置增益（根据关节大小调整）
        Kp_(0) = 300.0; // joint_1: 大关节，需要较大增益
        Kp_(1) = 400.0; // joint_2: 抬臂关节，负载大
        Kp_(2) = 350.0; // joint_3: 肘关节
        Kp_(3) = 150.0; // joint_4: 小关节
        Kp_(4) = 100.0; // joint_5: 更小
        Kp_(5) = 80.0;  // joint_6: 末端，增益最小

        // 速度增益（通常是 Kp 的 1/10 ~ 1/5）
        Kd_(0) = 30.0;
        Kd_(1) = 40.0;
        Kd_(2) = 35.0;
        Kd_(3) = 15.0;
        Kd_(4) = 10.0;
        Kd_(5) = 8.0;

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
        // MoveIt 会发送到: /ARM_controller/follow_joint_trajectory
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "ARM_controller/follow_joint_trajectory", // 完整的 action 名称
            std::bind(&TorqueControllerActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TorqueControllerActionServer::handleCancel, this, std::placeholders::_1),
            std::bind(&TorqueControllerActionServer::handleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "📡 Action Server 已创建: /ARM_controller/follow_joint_trajectory");

        // ========== 创建力矩发布者 ==========
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands", // ros2_control 的 effort_controller 会订阅这个话题
            10);

        RCLCPP_INFO(this->get_logger(), "📡 力矩发布者已创建: /effort_controller/commands");
        RCLCPP_INFO(this->get_logger(), "⚙️  控制频率: %.1f Hz", control_frequency_);

        auto period = std::chrono::duration<double, std::milli>(1000.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            period,
            std::bind(&TorqueControllerActionServer::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "⚙️  控制循环定时器已启动 (%.1f Hz)", control_frequency_);
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

    KDL::JntArray Kp_; // P of position
    KDL::JntArray Kd_; // D of position

    rclcpp::TimerBase::SharedPtr control_timer_;                                // 控制循环定时器
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_; // 力矩发布者
    double control_frequency_;                                                  // 控制频率 (Hz)

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

    bool interpolateTrajectory(double t_now,
                               KDL::JntArray &q_d,
                               KDL::JntArray &qd_d,
                               KDL::JntArray &qdd_d);

    void computeFeedbackTorque(const KDL::JntArray &q_d,       // 期望位置
                               const KDL::JntArray &qd_d,      // 期望速度
                               const KDL::JntArray &q_actual,  // 实际位置
                               const KDL::JntArray &qd_actual, // 实际速度
                               KDL::JntArray &tau_fb);         // 输出：反馈力矩

    void controlLoop(); // 核心控制循环
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

    // 检查是否收到关节状态
    if (!state_received_)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 未收到关节状态数据，拒绝执行");
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
        goal_handle->abort(result);
        return;
    }

    // 缓存轨迹
    const auto goal = goal_handle->get_goal();
    current_trajectory_ = goal->trajectory;
    current_goal_handle_ = goal_handle;
    trajectory_start_time_ = this->now();
    is_executing_ = true;

    // 打印轨迹信息
    const auto &first_point = current_trajectory_.points[0];
    const auto &last_point = current_trajectory_.points.back();
    double total_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

    RCLCPP_INFO(this->get_logger(), "📦 轨迹已缓存:");
    RCLCPP_INFO(this->get_logger(), "   - 关节数: %zu", current_trajectory_.joint_names.size());
    RCLCPP_INFO(this->get_logger(), "   - 轨迹点数: %zu", current_trajectory_.points.size());
    RCLCPP_INFO(this->get_logger(), "   - 总时长: %.3f 秒", total_duration);


    
    // 计算定时器周期（单位：毫秒）
    auto period = std::chrono::duration<double, std::milli>(1000.0 / control_frequency_);

    // 创建定时器
    control_timer_ = this->create_wall_timer(
        period,
        std::bind(&TorqueControllerActionServer::controlLoop, this));
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
    if (!urdf_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法打开 URDF 文件: %s", urdf_path.c_str());
        return false;
    }

    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                            std::istreambuf_iterator<char>());
    urdf_file.close();

    // 2. 解析 URDF
    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ URDF 解析失败");
        return false;
    }

    // 3. 提取 KDL 树
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 从 URDF 构建 KDL 树失败");
        return false;
    }

    // 4. 获取运动链（从 base_link 到 link6_2006roll）
    if (!kdl_tree.getChain("base_link", "link6_2006roll", kdl_chain_))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 提取运动链失败");
        return false;
    }

    // 5. 创建动力学计算工具
    KDL::Vector gravity(0.0, 0.0, -9.81); // 重力向量
    dynamic_computer_ = std::make_unique<DynamicsComputer>(kdl_chain_, gravity);

    RCLCPP_INFO(this->get_logger(), "✅ 动力学求解器初始化完成");
    RCLCPP_INFO(this->get_logger(), "   - 重力: [%.2f, %.2f, %.2f] m/s²",
                gravity.x(), gravity.y(), gravity.z());

    return true;
}

bool TorqueControllerActionServer::interpolateTrajectory(
    double t_now,
    KDL::JntArray &q_d,
    KDL::JntArray &qd_d,
    KDL::JntArray &qdd_d)
{
    // 1. 检查轨迹是否存在
    if (current_trajectory_.points.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 轨迹为空，无法插值");
        return false;
    }

    // 2. 如果时间在第一个点之前，返回第一个点
    const auto &first_point = current_trajectory_.points[0];
    double t_first = first_point.time_from_start.sec + first_point.time_from_start.nanosec * 1e-9;

    if (t_now <= t_first)
    {
        // 返回第一个点的值
        for (size_t i = 0; i < 6; i++)
        {
            q_d(i) = first_point.positions[i];
            qd_d(i) = first_point.velocities.empty() ? 0.0 : first_point.velocities[i];
            qdd_d(i) = first_point.accelerations.empty() ? 0.0 : first_point.accelerations[i];
        }
        return true;
    }

    // 3. 如果时间在最后一个点之后，返回最后一个点
    const auto &last_point = current_trajectory_.points.back();
    double t_last = last_point.time_from_start.sec + last_point.time_from_start.nanosec * 1e-9;

    if (t_now >= t_last)
    {
        // 返回最后一个点的值（速度和加速度应该为 0）
        for (size_t i = 0; i < 6; i++)
        {
            q_d(i) = last_point.positions[i];
            qd_d(i) = 0.0;  // 停止时速度为 0
            qdd_d(i) = 0.0; // 停止时加速度为 0
        }
        return true;
    }

    // 4. 在轨迹中查找包围当前时间的两个点
    size_t idx_before = 0;
    size_t idx_after = 0;

    for (size_t i = 0; i < current_trajectory_.points.size() - 1; i++)
    {
        double t_i = current_trajectory_.points[i].time_from_start.sec +
                     current_trajectory_.points[i].time_from_start.nanosec * 1e-9;
        double t_i_next = current_trajectory_.points[i + 1].time_from_start.sec +
                          current_trajectory_.points[i + 1].time_from_start.nanosec * 1e-9;

        if (t_now >= t_i && t_now <= t_i_next)
        {
            idx_before = i;
            idx_after = i + 1;
            break;
        }
    }

    // 5. 获取两个点
    const auto &point_before = current_trajectory_.points[idx_before];
    const auto &point_after = current_trajectory_.points[idx_after];

    double t_before = point_before.time_from_start.sec + point_before.time_from_start.nanosec * 1e-9;
    double t_after = point_after.time_from_start.sec + point_after.time_from_start.nanosec * 1e-9;

    // 6. 计算插值比例 α
    double alpha = (t_now - t_before) / (t_after - t_before);

    // 防止除零
    if (t_after - t_before < 1e-9)
    {
        alpha = 0.0;
    }

    // 7. 线性插值
    for (size_t i = 0; i < 6; i++)
    {
        // 位置插值
        q_d(i) = point_before.positions[i] +
                 alpha * (point_after.positions[i] - point_before.positions[i]);

        // 速度插值
        if (!point_before.velocities.empty() && !point_after.velocities.empty())
        {
            qd_d(i) = point_before.velocities[i] +
                      alpha * (point_after.velocities[i] - point_before.velocities[i]);
        }
        else
        {
            qd_d(i) = 0.0;
        }

        // 加速度插值
        if (!point_before.accelerations.empty() && !point_after.accelerations.empty())
        {
            qdd_d(i) = point_before.accelerations[i] +
                       alpha * (point_after.accelerations[i] - point_before.accelerations[i]);
        }
        else
        {
            qdd_d(i) = 0.0;
        }
    }

    return true;
}

// ========== PD 反馈控制 ==========
void TorqueControllerActionServer::computeFeedbackTorque(
    const KDL::JntArray &q_d,
    const KDL::JntArray &qd_d,
    const KDL::JntArray &q_actual,
    const KDL::JntArray &qd_actual,
    KDL::JntArray &tau_fb)
{
    KDL::JntArray error_p(6);
    for (size_t i = 0; i < 6; i++)
    {
        error_p(i) = q_d(i) - q_actual(i);
    }

    KDL::JntArray error_v(6);
    for (size_t i = 0; i < 6; i++)
    {
        error_v(i) = qd_d(i) - qd_actual(i);
    }

    // 3. PD 控制律: τ_fb = Kp·e_p + Kd·e_v
    for (size_t i = 0; i < 6; i++)
    {
        tau_fb(i) = Kp_(i) * error_p(i) + Kd_(i) * error_v(i);
    }
}

void TorqueControllerActionServer::controlLoop()
{
    if (!is_executing_)
    {
        // 没有活动轨迹时，发送重力补偿力矩保持位置
        std::lock_guard<std::mutex> lock(state_mutex_);

        if (!state_received_)
        {
            return; // 还没收到状态，无法计算
        }

        // 计算重力补偿
        KDL::JntArray tau_gravity(6);
        dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);

        // 发布重力补偿力矩
        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data.resize(6);
        for (size_t i = 0; i < 6; i++)
        {
            torque_msg.data[i] = tau_gravity(i);
        }
        torque_pub_->publish(torque_msg);

        // 调试输出
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "🔧 保持模式：发送重力补偿 τ_g=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            tau_gravity(0), tau_gravity(1), tau_gravity(2),
            tau_gravity(3), tau_gravity(4), tau_gravity(5));

        return;
    }

    // 现在时间获取
    double t_now = (this->now() - trajectory_start_time_).seconds();

    // 获取轨迹TIME
    const auto &last_point = current_trajectory_.points.back();
    double total_duration = last_point.time_from_start.sec +
                            last_point.time_from_start.nanosec * 1e-9;

    if (t_now >= total_duration) // 检查是否完成
    {
        RCLCPP_INFO(this->get_logger(), "✅ 轨迹执行完成！");

        // 停止定时器
        control_timer_->cancel();

        // 发布重力补偿力矩（保持最终位置，而不是零力矩）
        KDL::JntArray tau_gravity(6);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            dynamic_computer_->computeGravityTorque(q_actual_, tau_gravity);
        }
        std_msgs::msg::Float64MultiArray hold_torque;
        hold_torque.data.resize(6);
        for (size_t i = 0; i < 6; i++)
        {
            hold_torque.data[i] = tau_gravity(i);
        }
        torque_pub_->publish(hold_torque);

        RCLCPP_INFO(this->get_logger(), "✅ 轨迹执行完成，保持最终位置（重力补偿: τ_g=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]）",
                    tau_gravity(0), tau_gravity(1), tau_gravity(2),
                    tau_gravity(3), tau_gravity(4), tau_gravity(5));

        // 返回成功结果
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        current_goal_handle_->succeed(result);

        // 清理状态
        is_executing_ = false;
        current_goal_handle_.reset();
        return;
    }

    // 插值（时间不连续）
    KDL::JntArray q_d(6), qd_d(6), qdd_d(6);
    if (!interpolateTrajectory(t_now, q_d, qd_d, qdd_d))
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 轨迹插值失败");
        return;
    }

    // 获取实际状态（线程安全）
    KDL::JntArray q_actual(6), qd_actual(6);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_actual = q_actual_;
        qd_actual = q_dot_actual_;
    }

    KDL::JntArray tau_ff(6);
    dynamic_computer_->computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff); // 计算前馈

    KDL::JntArray tau_fb(6);
    computeFeedbackTorque(q_d, qd_d, q_actual, qd_actual, tau_fb); // 计算反馈

    KDL::JntArray tau_total(6);
    for (size_t i = 0; i < 6; i++)
    {
        tau_total(i) = tau_ff(i) + tau_fb(i);
    } // PD+前馈力矩

    // 发送力矩到 ros2_control effort_controller
    std_msgs::msg::Float64MultiArray torque_msg;
    torque_msg.data.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
        torque_msg.data[i] = tau_total(i);
    }
    torque_pub_->publish(torque_msg);

    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->header.stamp = this->now(); // 获取反馈指针

    feedback->actual.positions.resize(6);
    feedback->actual.velocities.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
        feedback->actual.positions[i] = q_actual(i);
        feedback->actual.velocities[i] = qd_actual(i); // 更新q-当前
    }

    feedback->desired.positions.resize(6);
    feedback->desired.velocities.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
        feedback->desired.positions[i] = q_d(i);
        feedback->desired.velocities[i] = qd_d(i); // 更新q-期望
    }

    feedback->error.positions.resize(6);
    feedback->error.velocities.resize(6);
    for (size_t i = 0; i < 6; i++)
    {
        feedback->error.positions[i] = q_d(i) - q_actual(i);
        feedback->error.velocities[i] = qd_d(i) - qd_actual(i); // error
    }

    current_goal_handle_->publish_feedback(feedback);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000, // 每秒打印一次
        "⏱️  t=%.3f/%.3f | τ=[%.1f, %.1f, %.1f, ...] | err_p=[%.4f, %.4f, %.4f, ...]",
        t_now, total_duration,
        tau_total(0), tau_total(1), tau_total(2),
        feedback->error.positions[0], feedback->error.positions[1], feedback->error.positions[2]);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TorqueControllerActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}