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
#include "kalman_filter.hpp"

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class TorqueControllerActionServer : public rclcpp::Node // 节点类生成
{
public: // 构造函数log
    TorqueControllerActionServer() : Node("torque_controller_action_server"),
                                     is_executing_(false),
                                     q_actual_(6),
                                     q_dot_actual_(6),
                                     q_target_(6),
                                     state_received_(false),
                                     has_target_(false),
                                     Kp_(6),
                                     Kd_(6),
                                     control_frequency_(200.0),
                                     joint_filters_{
                                         KalmanFilter1D(1.0 / 200.0), // Joint 1
                                         KalmanFilter1D(1.0 / 200.0), // Joint 2
                                         KalmanFilter1D(1.0 / 200.0), // Joint 3
                                         KalmanFilter1D(1.0 / 200.0), // Joint 4
                                         KalmanFilter1D(1.0 / 200.0), // Joint 5
                                         KalmanFilter1D(1.0 / 200.0)  // Joint 6
                                     },
                                     q_dot_filtered_(6),
                                     kalman_filter_enabled_(false) // 默认启用卡尔曼滤波

    {
        RCLCPP_INFO(this->get_logger(), "🚀 力矩控制器节点启动");
        // ========== 参数声明：PD 增益（可动态调节）==========

        // 声明 Kp 参数（每个关节独立）
        this->declare_parameter("Kp.joint_1", 1000.0);
        this->declare_parameter("Kp.joint_2", 1500.0);
        this->declare_parameter("Kp.joint_3", 1550.0);
        this->declare_parameter("Kp.joint_4", 350.0);
        this->declare_parameter("Kp.joint_5", 100.0);
        this->declare_parameter("Kp.joint_6", 20.0);

        // 声明 Kd 参数
        this->declare_parameter("Kd.joint_1", 0.0);
        this->declare_parameter("Kd.joint_2", 0.0);
        this->declare_parameter("Kd.joint_3", 0.0);
        this->declare_parameter("Kd.joint_4", 0.0);
        this->declare_parameter("Kd.joint_5", 0.0);
        this->declare_parameter("Kd.joint_6", 0.0);

        // 读取初始值到成员变量
        Kp_(0) = this->get_parameter("Kp.joint_1").as_double();
        Kp_(1) = this->get_parameter("Kp.joint_2").as_double();
        Kp_(2) = this->get_parameter("Kp.joint_3").as_double();
        Kp_(3) = this->get_parameter("Kp.joint_4").as_double();
        Kp_(4) = this->get_parameter("Kp.joint_5").as_double();
        Kp_(5) = this->get_parameter("Kp.joint_6").as_double();

        Kd_(0) = this->get_parameter("Kd.joint_1").as_double();
        Kd_(1) = this->get_parameter("Kd.joint_2").as_double();
        Kd_(2) = this->get_parameter("Kd.joint_3").as_double();
        Kd_(3) = this->get_parameter("Kd.joint_4").as_double();
        Kd_(4) = this->get_parameter("Kd.joint_5").as_double();
        Kd_(5) = this->get_parameter("Kd.joint_6").as_double();

        RCLCPP_INFO(this->get_logger(), "✅ PD 增益已初始化:");
        RCLCPP_INFO(this->get_logger(), "   Kp=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                    Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5));
        RCLCPP_INFO(this->get_logger(), "   Kd=[%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]",
                    Kd_(0), Kd_(1), Kd_(2), Kd_(3), Kd_(4), Kd_(5));

        // ========== 卡尔曼滤波器开关参数 ==========
        this->declare_parameter("kalman.enabled", true); // 默认启用
        kalman_filter_enabled_ = this->get_parameter("kalman.enabled").as_bool();

        RCLCPP_INFO(this->get_logger(), "🎚️  卡尔曼滤波器状态: %s",
                    kalman_filter_enabled_ ? "✅ 启用" : "❌ 禁用");

        // ========== 新增：声明卡尔曼滤波器参数（必须在读取之前声明）==========
        this->declare_parameter("kalman.Q_pos", 1e-10);  // 过程噪声：位置
        this->declare_parameter("kalman.Q_vel", 1e-7);   // 过程噪声：速度
        this->declare_parameter("kalman.R_pos", 1e-3);   // 测量噪声：位置
        this->declare_parameter("kalman.R_vel", 2.5e-2); // 测量噪声：速度

        // 读取参数并设置滤波器
        double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
        double Q_vel = this->get_parameter("kalman.Q_vel").as_double();
        double R_pos = this->get_parameter("kalman.R_pos").as_double();
        double R_vel = this->get_parameter("kalman.R_vel").as_double();

        for (auto &filter : joint_filters_)
        {
            filter.setProcessNoise(Q_pos, Q_vel);
            filter.setMeasurementNoise(R_pos, R_vel);
        }

        RCLCPP_INFO(this->get_logger(), "✅ 卡尔曼滤波器已初始化:");
        RCLCPP_INFO(this->get_logger(), "   Q_pos=%.1e, Q_vel=%.1e", Q_pos, Q_vel);
        RCLCPP_INFO(this->get_logger(), "   R_pos=%.1e, R_vel=%.1e", R_pos, R_vel);

        // ========== 注册参数变化回调（必须在所有参数声明之后）==========
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&TorqueControllerActionServer::parametersCallback,
                      this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🔧 参数动态调节已启用（可通过 ros2 param set 命令修改）");

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
    KDL::JntArray q_target_;     // 目标关节位置（规划终点） [6]
    std::mutex state_mutex_;     // 保护共享数据
    bool state_received_;        // 是否收到过状态
    bool has_target_;            // 是否有目标位置

    KDL::Chain kdl_chain_;                               // 运动链实例
    std::unique_ptr<DynamicsComputer> dynamic_computer_; // 动力学解算器实例

    KDL::JntArray Kp_; // P of position
    KDL::JntArray Kd_; // D of position

    std::array<KalmanFilter1D, 6> joint_filters_;
    KDL::JntArray q_dot_filtered_;
    bool kalman_filter_enabled_; // 卡尔曼滤波开关

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

    // ========== 新增：参数动态调节 ==========
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 参数变化回调函数
    rcl_interfaces::msg::SetParametersResult parametersCallback(
        const std::vector<rclcpp::Parameter> &parameters);
};

rclcpp_action::GoalResponse TorqueControllerActionServer::handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    (void)uuid;

    RCLCPP_INFO(this->get_logger(), "🎯 收到新轨迹 (%zu点)", goal->trajectory.points.size());

    if (is_executing_)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ 检测到新轨迹，将抢占当前执行的轨迹");
        // 不再 REJECT，而是继续接受
    }

    if (goal->trajectory.points.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 轨迹为空，拒绝目标");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "✅ 接受新目标");
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

    if (is_executing_ && current_goal_handle_)
    {
        RCLCPP_WARN(this->get_logger(), "⚠️ 取消旧轨迹，切换到新轨迹");

        // 通知旧轨迹被抢占
        auto old_result = std::make_shared<FollowJointTrajectory::Result>();
        old_result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED; // 或其他错误码
        current_goal_handle_->abort(old_result);
    }

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

    // ========== 新增：保存规划终点位置 ==========
    if (last_point.positions.size() == 6)
    {
        for (size_t i = 0; i < 6; i++)
        {
            q_target_(i) = last_point.positions[i];
        }
        has_target_ = true;
        RCLCPP_INFO(this->get_logger(), "📍 规划终点: q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    q_target_(0), q_target_(1), q_target_(2),
                    q_target_(3), q_target_(4), q_target_(5));
    }

    RCLCPP_INFO(this->get_logger(), "📦 轨迹已缓存 (%zu点, %.3fs)",
                current_trajectory_.points.size(), total_duration);
}

void TorqueControllerActionServer::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // 线程安全：加锁
    std::lock_guard<std::mutex> lock(state_mutex_);

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

    // ========== 卡尔曼滤波处理 ==========
    if (kalman_filter_enabled_)
    {
        // 启用滤波：使用卡尔曼滤波器
        for (size_t i = 0; i < 6; i++)
        {
            // 首次接收：初始化滤波器
            if (!state_received_)
            {
                joint_filters_[i].initialize(msg->position[i], msg->velocity[i]);
            }
            else
            {
                // 后续：预测-更新循环
                joint_filters_[i].predict();
                joint_filters_[i].update(msg->position[i], msg->velocity[i]);
            }

            // 获取滤波后的状态
            q_actual_(i) = joint_filters_[i].getPosition();
            q_dot_filtered_(i) = joint_filters_[i].getVelocity();
        }
    }
    else
    {
        // 禁用滤波：直接使用原始测量值
        for (size_t i = 0; i < 6; i++)
        {
            q_dot_filtered_(i) = q_dot_actual_(i); // 直接使用原始速度
            // q_actual_ 已在上面赋值，保持不变
        }
    }

    // 首次接收数据：保存启动姿态作为初始目标
    if (!state_received_)
    {
        RCLCPP_INFO(this->get_logger(), "✅ 首次接收关节状态数据");
        state_received_ = true;

        // 保存启动姿态作为初始目标位置
        if (!has_target_)
        {
            q_target_ = q_actual_;
            has_target_ = true;
            RCLCPP_INFO(this->get_logger(), "📍 保存启动姿态作为初始目标:");
            RCLCPP_INFO(this->get_logger(), "   q_target=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        q_target_(0), q_target_(1), q_target_(2),
                        q_target_(3), q_target_(4), q_target_(5));
        }
        // ========== 调试：打印卡尔曼增益 ==========
        RCLCPP_INFO(this->get_logger(), "🔍 首次卡尔曼增益（Joint 1）:");
        auto K = joint_filters_[0].getKalmanGain();
        RCLCPP_INFO(this->get_logger(), "   K = [%.4f, %.4f]", K(0, 0), K(0, 1));
        RCLCPP_INFO(this->get_logger(), "       [%.4f, %.4f]", K(1, 0), K(1, 1));
    }
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

        // ========== 修改：PD 控制目标位置 ========== //添加滤波速度
        KDL::JntArray tau_pd(6);
        if (has_target_)
        {
            computeFeedbackTorque(q_target_, KDL::JntArray(6), q_actual_, q_dot_filtered_, tau_pd);
        }
        else
        {
            computeFeedbackTorque(q_actual_, KDL::JntArray(6), q_actual_, q_dot_filtered_, tau_pd);
        }

        // 发布力矩：重力补偿 + PD 控制
        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data.resize(6);
        for (size_t i = 0; i < 6; i++)
        {
            torque_msg.data[i] = tau_gravity(i) + tau_pd(i);
        }
        torque_pub_->publish(torque_msg);

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
        KDL::JntArray q_actual_copy(6), qd_filtered_copy(6);
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            q_actual_copy = q_actual_;
            qd_filtered_copy = q_dot_filtered_; // 从成员变量拷贝滤波后的速度
        }

        // 计算重力补偿
        KDL::JntArray tau_gravity(6);
        dynamic_computer_->computeGravityTorque(q_actual_copy, tau_gravity);

        KDL::JntArray tau_pd(6);
        if (has_target_)
        {
            // 期望位置 = 规划终点，期望速度 = 0
            computeFeedbackTorque(q_target_, KDL::JntArray(6), q_actual_copy, qd_filtered_copy, tau_pd);
        }
        else
        {
            // 如果没有目标，PD 输出为 0
            for (size_t i = 0; i < 6; i++)
            {
                tau_pd(i) = 0.0;
            }
        }

        // 发布力矩：重力补偿 + PD 控制
        std_msgs::msg::Float64MultiArray hold_torque;
        hold_torque.data.resize(6);
        for (size_t i = 0; i < 6; i++)
        {
            hold_torque.data[i] = tau_gravity(i) + tau_pd(i);
        }
        torque_pub_->publish(hold_torque);

        RCLCPP_INFO(this->get_logger(), "✅ 轨迹执行完成，切换到保持模式（目标位置: q_target）");
        RCLCPP_INFO(this->get_logger(), "   τ_total=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    hold_torque.data[0], hold_torque.data[1], hold_torque.data[2],
                    hold_torque.data[3], hold_torque.data[4], hold_torque.data[5]);

        // 返回成功结果
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        current_goal_handle_->succeed(result);

        // 清理状态（但保留 q_target_ 和 has_target_）
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
    KDL::JntArray q_actual(6), qd_filtered(6);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_actual = q_actual_;
        qd_filtered = q_dot_filtered_; // 从成员变量拷贝滤波后的速度
    }

    KDL::JntArray tau_ff(6); // PD的话需要输入期望和实际，前馈只需要期望
    dynamic_computer_->computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff);
    KDL::JntArray tau_fb(6);
    computeFeedbackTorque(q_d, qd_d, q_actual, qd_filtered, tau_fb); // 计算PD反馈力矩，使用滤波后的速度

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
        feedback->actual.velocities[i] = qd_filtered(i); // 反馈使用滤波后的速度
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
        feedback->error.velocities[i] = qd_d(i) - qd_filtered(i); // 速度误差基于滤波后的速度
    }

    current_goal_handle_->publish_feedback(feedback);
}

// ========== 参数变化回调函数实现 ==========
rcl_interfaces::msg::SetParametersResult TorqueControllerActionServer::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "参数更新成功";

    for (const auto &param : parameters)
    {
        const std::string &name = param.get_name();

        // 检查是否是 Kp 参数
        if (name.find("Kp.joint_") == 0)
        {
            // 提取关节编号："Kp.joint_1" -> 1
            std::string joint_num_str = name.substr(9);   // "joint_1" -> "1"
            int joint_idx = std::stoi(joint_num_str) - 1; // 索引从 0 开始

            if (joint_idx >= 0 && joint_idx < 6)
            {
                double new_value = param.as_double();

                // 合法性检查
                if (new_value < 0.0)
                {
                    result.successful = false;
                    result.reason = "Kp 值不能为负数";
                    RCLCPP_ERROR(this->get_logger(), "❌ 拒绝无效参数: %s = %.2f", name.c_str(), new_value);
                    return result;
                }

                // 更新增益
                Kp_(joint_idx) = new_value;

                RCLCPP_INFO(this->get_logger(),
                            "🔧 Kp[joint_%d] 已更新: %.2f", joint_idx + 1, new_value);
            }
        }
        // 检查是否是 Kd 参数
        else if (name.find("Kd.joint_") == 0)
        {
            std::string joint_num_str = name.substr(9);
            int joint_idx = std::stoi(joint_num_str) - 1;

            if (joint_idx >= 0 && joint_idx < 6)
            {
                double new_value = param.as_double();

                // 合法性检查
                if (new_value < 0.0)
                {
                    result.successful = false;
                    result.reason = "Kd 值不能为负数";
                    RCLCPP_ERROR(this->get_logger(), "❌ 拒绝无效参数: %s = %.2f", name.c_str(), new_value);
                    return result;
                }

                // 更新增益
                Kd_(joint_idx) = new_value;

                RCLCPP_INFO(this->get_logger(),
                            "🔧 Kd[joint_%d] 已更新: %.2f", joint_idx + 1, new_value);
            }
        }

        // ========== 新增：卡尔曼滤波器参数 ==========
        else if (name == "kalman.enabled")
        {
            // 更新卡尔曼滤波开关
            kalman_filter_enabled_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "🎚️  卡尔曼滤波器已%s",
                        kalman_filter_enabled_ ? "✅ 启用" : "❌ 禁用");
        }
        else if (name == "kalman.Q_pos" || name == "kalman.Q_vel")
        {
            // 安全检查：确保所有相关参数都已声明
            if (this->has_parameter("kalman.Q_pos") && this->has_parameter("kalman.Q_vel"))
            {
                double Q_pos = this->get_parameter("kalman.Q_pos").as_double();
                double Q_vel = this->get_parameter("kalman.Q_vel").as_double();

                for (auto &filter : joint_filters_)
                {
                    filter.setProcessNoise(Q_pos, Q_vel);
                }

                RCLCPP_INFO(this->get_logger(), "🔧 过程噪声已更新: Q_pos=%.1e, Q_vel=%.1e",
                            Q_pos, Q_vel);
            }
        }
        else if (name == "kalman.R_pos" || name == "kalman.R_vel")
        {
            // 安全检查：确保所有相关参数都已声明
            if (this->has_parameter("kalman.R_pos") && this->has_parameter("kalman.R_vel"))
            {
                double R_pos = this->get_parameter("kalman.R_pos").as_double();
                double R_vel = this->get_parameter("kalman.R_vel").as_double();

                for (auto &filter : joint_filters_)
                {
                    filter.setMeasurementNoise(R_pos, R_vel);
                }

                RCLCPP_INFO(this->get_logger(), "🔧 测量噪声已更新: R_pos=%.1e, R_vel=%.1e",
                            R_pos, R_vel);
            }
        }
    }

    return result;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TorqueControllerActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}