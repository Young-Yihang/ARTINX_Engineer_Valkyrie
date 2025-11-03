// ============================================================
// 动力学求解器观察者节点 (Dynamics Solver Observer Node)
// 
// 职责:
//   - 订阅 /joint_states 获取实时关节状态 (q, q̇)
//   - 数值微分计算加速度 q̈ = (q̇_current - q̇_previous) / Δt
//   - 计算动力学方程: τ = M(q)·q̈ + C(q,q̇) + G(q)
//   - 发布到 /computed_torques 供观察和调试
//
// 注意:
//   - 这是一个 **观察者节点**，不参与实际控制
//   - 用于验证动力学计算的正确性
//   - 实际控制由 torque_controller_node 负责 (Stage 4)
//
// Stage: 3 (completed)
// Author: ARV V1 Team
// Date: 2025-10-28
// ============================================================
//注意：这个cpp已经不使用
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <fstream>
#include <sstream>
#include <memory>
#include <mutex>
// ROS2 消息类型
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
// KDL 动力学库
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include "urdf_parser.cpp" // 包含 URDF 解析工具

class DynamicsSolverNode : public rclcpp::Node
{
public:
    DynamicsSolverNode() : Node("dynamics_solver_node"),
                           num_joints_(6),
                           first_msg_received_(false)
    {
        // 方法1: 尝试从参数获取URDF
        this->declare_parameter("robot_description", "");
        std::string urdf_string = this->get_parameter("robot_description").as_string();

        // 方法2: 如果参数为空，从文件读取（用于快速测试）
        if (urdf_string.empty())
        {
            RCLCPP_WARN(this->get_logger(), "robot_description 参数为空，尝试从文件读取...");

            // 使用相对于工作空间的路径
            std::string urdf_path = "/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/ARV_V1_MODEL.urdf";
            std::ifstream file(urdf_path);

            if (!file.is_open())
            {
                RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot open URDF file: %s", urdf_path.c_str());
                return;
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            urdf_string = buffer.str();

            RCLCPP_INFO(this->get_logger(), "[OK] Loaded URDF from file (%zu bytes)", urdf_string.size());
        }
        // 实例化对象
        URDFDynamicsParser parser;
        if (!parser.parseURDFString(urdf_string))
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] URDF parse failed");
            return;
        }

        // 获取KDL运动链
        kdl_chain_ = parser.getKDLChain();

    RCLCPP_INFO(this->get_logger(),
            "[OK] Extracted kinematic chain: %d joints",
            kdl_chain_.getNrOfJoints());

        // 打印摘要信息
        parser.printSummary();

        // 使用运动链初始化动力学求解器
        initDynamicsSolver();
        
        // ========== 创建订阅者和发布者 ==========
        
        // 订阅 /joint_states 话题
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 
            10,
            std::bind(&DynamicsSolverNode::jointStatesCallback, this, std::placeholders::_1)
        );
        
        // 发布计算的力矩
        torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/computed_torques", 
            10
        );
        
    RCLCPP_INFO(this->get_logger(), "[INFO] Subscribed: /joint_states");
    RCLCPP_INFO(this->get_logger(), "[INFO] Publishing torques to: /computed_torques");
    }
    
    ~DynamicsSolverNode()
    {
        RCLCPP_INFO(this->get_logger(), "[INFO] DynamicsSolverNode destructing");
    }

private:
    // ========== 成员变量 ==========
    
    // KDL 动力学组件
    KDL::Chain kdl_chain_; 
    std::shared_ptr<KDL::ChainDynParam> dyn_param_;
    
    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
    
    // 数据存储
    int num_joints_;
    bool first_msg_received_;
    
    // 当前状态 (线程安全)
    std::mutex state_mutex_;
    KDL::JntArray q_current_;       // 当前位置
    KDL::JntArray q_dot_current_;   // 当前速度
    KDL::JntArray q_dot_previous_;  // 上一次速度 (用于计算加速度)
    rclcpp::Time previous_time_;    // 上一次时间戳

    void initDynamicsSolver()
    {
        // ========== 步骤1: 创建 KDL 动力学参数计算器 ==========
        
        // 定义重力向量 (Z轴向下，-9.81 m/s²)
        KDL::Vector gravity(0.0, 0.0, -9.81);
        
        // 创建动力学参数计算器（使用提取的运动链和重力）
        dyn_param_ = std::make_shared<KDL::ChainDynParam>(kdl_chain_, gravity);
        
    RCLCPP_INFO(this->get_logger(), "[OK] Dynamics solver initialized");
    RCLCPP_INFO(this->get_logger(), "   - Gravity: [%.2f, %.2f, %.2f] m/s²", 
            gravity.x(), gravity.y(), gravity.z());
        
        // 初始化关节数组
        q_current_.resize(num_joints_);
        q_dot_current_.resize(num_joints_);
        q_dot_previous_.resize(num_joints_);
        
        // 初始化为零
        for (int i = 0; i < num_joints_; i++) {
            q_current_(i) = 0.0;
            q_dot_current_(i) = 0.0;
            q_dot_previous_(i) = 0.0;
        }
    }
    
    /**
     * @brief joint_states 回调函数
     * 
     * 当接收到 /joint_states 消息时调用
     * 1. 提取 q (位置) 和 q̇ (速度)
     * 2. 计算 q̈ (加速度) 通过数值微分
     * 3. 计算动力学
     * 4. 发布力矩
     */
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // ========== 步骤1: 数据验证 ==========
        
        if (msg->position.size() != num_joints_ || msg->velocity.size() != num_joints_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[WARN] joint_states incomplete: pos=%zu, vel=%zu, expected=%d",
                msg->position.size(), msg->velocity.size(), num_joints_);
            return;
        }
        
        // ========== 步骤2: 提取数据（线程安全）==========
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // 获取当前时间
        rclcpp::Time current_time = msg->header.stamp;
        
        // 提取位置 q 和速度 q̇
        for (int i = 0; i < num_joints_; i++) {
            q_current_(i) = msg->position[i];
            q_dot_current_(i) = msg->velocity[i];
        }
        
        // ========== 步骤3: 计算加速度 q̈ ==========
        
        KDL::JntArray q_ddot(num_joints_);
        
        if (!first_msg_received_) {
            // 第一次接收数据，加速度设为0
            for (int i = 0; i < num_joints_; i++) {
                q_ddot(i) = 0.0;
                q_dot_previous_(i) = q_dot_current_(i);
            }
            previous_time_ = current_time;
            first_msg_received_ = true;
            
            RCLCPP_INFO(this->get_logger(), "[OK] First joint_states received");
            return;
        }
        
        // 计算时间差 Δt
        double dt = (current_time - previous_time_).seconds();
        
        if (dt <= 0.0 || dt > 1.0) {
            // 时间异常（时间倒流或间隔太大）
            RCLCPP_WARN(this->get_logger(), "[WARN] Time interval abnormal: dt=%.3f s", dt);
            previous_time_ = current_time;
            return;
        }
        
        // 数值微分: q̈ ≈ (q̇_current - q̇_previous) / Δt
        for (int i = 0; i < num_joints_; i++) {
            q_ddot(i) = (q_dot_current_(i) - q_dot_previous_(i)) / dt;
        }
        
        // ========== 步骤4: 计算动力学参数 ==========
        
        // 1. 质量矩阵 M(q)
        KDL::JntSpaceInertiaMatrix M(num_joints_);
        dyn_param_->JntToMass(q_current_, M);
        
        // 2. 科氏力/离心力 C(q,q̇)
        KDL::JntArray C(num_joints_);
        dyn_param_->JntToCoriolis(q_current_, q_dot_current_, C);
        
        // 3. 重力项 G(q)
        KDL::JntArray G(num_joints_);
        dyn_param_->JntToGravity(q_current_, G);
        
        // ========== 步骤5: 计算所需力矩 τ = M·q̈ + C + G ==========
        
        KDL::JntArray tau(num_joints_);
        for (int i = 0; i < num_joints_; i++) {
            tau(i) = 0.0;
            // M矩阵的第i行 与 q̈ 相乘
            for (int j = 0; j < num_joints_; j++) {
                tau(i) += M(i, j) * q_ddot(j);
            }
            tau(i) += C(i) + G(i);
        }
        
        // ========== 步骤6: 发布力矩 ==========
        
        std_msgs::msg::Float64MultiArray torque_msg;
        torque_msg.data.resize(num_joints_);
        for (int i = 0; i < num_joints_; i++) {
            torque_msg.data[i] = tau(i);
        }
        torque_pub_->publish(torque_msg);
        
        // ========== 步骤7: 打印信息（限流：每秒1次）==========
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[DEBUG] Dynamics calc: τ=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] N·m | dt=%.3f s",
            tau(0), tau(1), tau(2), tau(3), tau(4), tau(5), dt);
        
        // ========== 步骤8: 保存当前数据供下次使用 ==========
        
        for (int i = 0; i < num_joints_; i++) {
            q_dot_previous_(i) = q_dot_current_(i);
        }
        previous_time_ = current_time;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicsSolverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
