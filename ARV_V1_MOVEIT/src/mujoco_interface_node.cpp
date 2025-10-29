// 主要作用：订阅effort_controller/commands力矩信号（TORQUE——CONTROLLER发布）,200Hz
// 同时作为MUJICO仿真器，发布反馈用话题 /Joint_state

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h> // 新增：窗口管理

#include <string>
#include <vector>
#include <thread>
#include <fstream>
#include <sstream>
#include <chrono>
#include <atomic>

class MuJoCoInterfaceNode : public rclcpp::Node
{
public:
    MuJoCoInterfaceNode() : Node("mujoco_interface"),
                            model_(nullptr),
                            data_(nullptr),
                            sim_frequency_(200.0),
                            urdf_path_("/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/ARV_V1_MODEL.urdf"),
                            received_first_command_(false)
    {
        RCLCPP_INFO(this->get_logger(), "🚀 MuJoCo接口节点启动");

        // ========== 步骤1: 加载MuJoCo模型 ==========
        if (!loadMuJoCoModel())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ MuJoCo模型加载失败");
            throw std::runtime_error("Failed to load MuJoCo model");
        }
        RCLCPP_INFO(this->get_logger(), "✅ MuJoCo模型加载成功");
        RCLCPP_INFO(this->get_logger(), "   关节数: %d", model_->nq);

        // ========== 步骤2: 设置初始位姿 ==========
        setInitialPose();

        // ========== 步骤3: 订阅力矩命令 ==========
        effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/effort_controller/commands",
            10,
            std::bind(&MuJoCoInterfaceNode::effortCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "📡 订阅话题: /effort_controller/commands");

        // ========== 步骤4: 创建关节状态发布者 ==========
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            10);

        RCLCPP_INFO(this->get_logger(), "📡 发布话题: /joint_states");

        // ========== 步骤5: 创建200Hz仿真定时器 ==========
        auto period = std::chrono::duration<double, std::milli>(1000.0 / sim_frequency_);
        sim_timer_ = this->create_wall_timer(
            period,
            std::bind(&MuJoCoInterfaceNode::simulationStep, this));

        RCLCPP_INFO(this->get_logger(), "⚙️  仿真频率: %.1f Hz", sim_frequency_);
        RCLCPP_INFO(this->get_logger(), "✅ MuJoCo接口节点初始化完成");

        // ========== 步骤6: 初始化并启动可视化 ==========
        render_running_ = false;
        window_ = nullptr;

        if (initializeVisualization())
        {
            RCLCPP_INFO(this->get_logger(), "✅ 可视化初始化成功");
            render_running_ = true;
            render_thread_ = std::thread(&MuJoCoInterfaceNode::renderLoop, this);
            RCLCPP_INFO(this->get_logger(), "🎨 渲染线程已启动");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "⚠️  可视化初始化失败，仅运行仿真");
        }
    }

    ~MuJoCoInterfaceNode()
    {
        RCLCPP_INFO(this->get_logger(), "🧹 清理MuJoCo资源...");

        if (data_)
        {
            mj_deleteData(data_);
        }

        if (model_)
        {
            mj_deleteModel(model_);
        }

        RCLCPP_INFO(this->get_logger(), "MuJoCo接口节点已关闭");

        // 清理渲染资源
        if (window_)
        {
            mjr_freeContext(&con_);
            mjv_freeScene(&scene_);
            glfwDestroyWindow(window_);
            glfwTerminate();
        }

        if (data_)
        {
            mj_deleteData(data_);
        }

        if (model_)
        {
            mj_deleteModel(model_);
        }

        RCLCPP_INFO(this->get_logger(), "垃圾全部扔出去");
    }

private:
    // ========== MuJoCo相关成员变量 ==========
    mjModel *model_; // MuJoCo模型指针
    mjData *data_;   // MuJoCo数据指针

    // ========== ROS2接口 ==========
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;

    // ========== 配置参数 ==========
    std::string urdf_path_;
    double sim_frequency_; // 200 Hz

    // ========== 可视化相关成员变量 ==========
    std::thread render_thread_;        // 渲染线程
    std::atomic<bool> render_running_; // 渲染线程运行标志

    GLFWwindow *window_;   // GLFW窗口指针
    mjvScene scene_;       // MuJoCo场景
    mjvCamera cam_;        // 相机
    mjvOption opt_;        // 渲染选项
    mjrContext con_;       // 渲染上下文
    std::mutex sim_mutex_; // 保护model_和data_的互斥锁

    // ========== 启动安全相关 ==========
    std::atomic<bool> received_first_command_; // 是否收到首次力矩命令

    // ========== 关节名称 ==========
    const std::vector<std::string> joint_names_ = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"};

    // ========== 成员函数声明 ==========

    /**
     * @brief 加载MuJoCo模型
     * @return true 加载成功, false 加载失败
     */
    bool loadMuJoCoModel();

    /**
     * @brief 设置初始关节位姿
     */
    void setInitialPose();

    /**
     * @brief 力矩命令回调函数
     */
    void effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /**
     * @brief 仿真步进函数 (200Hz)
     */
    void simulationStep();

    /**
     * @brief 发布关节状态到ROS2
     */
    void publishJointStates();

    /**
     * @brief 初始化可视化（GLFW窗口和MuJoCo渲染）
     */
    bool initializeVisualization();

    /**
     * @brief 渲染循环（在独立线程中运行）
     */
    void renderLoop();
};

// ========== 成员函数实现（暂时留空，后续填充） ==========
bool MuJoCoInterfaceNode::loadMuJoCoModel()
{
    RCLCPP_INFO(this->get_logger(), "📦 正在加载URDF: %s", urdf_path_.c_str());

    // ========== 步骤1: 读取URDF文件 ==========
    std::ifstream urdf_file(urdf_path_);
    if (!urdf_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法打开URDF文件: %s", urdf_path_.c_str());
        return false;
    }

    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                            std::istreambuf_iterator<char>());
    urdf_file.close();

    RCLCPP_INFO(this->get_logger(), "✓ URDF文件读取成功，大小: %zu 字节", urdf_string.size());

    // ========== 步骤2: 在<robot>标签后插入MuJoCo编译器设置 ==========
    std::string mujoco_compiler =
        "\n  <mujoco>\n"
        "    <compiler meshdir=\"/home/huan/ros2_ws/src/ARV_V1_MODEL/meshes\" strippath=\"false\"/>\n"
        "    <option timestep=\"0.005\"/>\n"
        "  </mujoco>\n";

    size_t robot_pos = urdf_string.find("<robot");
    if (robot_pos == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法找到<robot>标签");
        return false;
    }

    size_t bracket_pos = urdf_string.find(">", robot_pos);
    if (bracket_pos == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法找到<robot>标签的结束");
        return false;
    }

    urdf_string.insert(bracket_pos + 1, mujoco_compiler);
    RCLCPP_INFO(this->get_logger(), "✓ 已插入MuJoCo编译器设置");

    // URDF 中不插入 actuator，稍后通过 MJCF 添加

    // ========== 步骤3: 替换mesh路径为相对路径（只保留文件名） ==========
    // 查找所有的 package://ARV_V1_MODEL/meshes/ 并替换为空（只留文件名）
    std::string find_str = "package://ARV_V1_MODEL/meshes/";
    std::string replace_str = ""; // 空字符串，只保留文件名

    size_t pos = 0;
    int replace_count = 0;
    while ((pos = urdf_string.find(find_str, pos)) != std::string::npos)
    {
        urdf_string.replace(pos, find_str.length(), replace_str);
        pos += replace_str.length();
        replace_count++;
    }

    RCLCPP_INFO(this->get_logger(), "✓ 已替换 %d 处mesh路径为相对路径", replace_count);

    // 额外处理：替换可能存在的 package://ARV_V1_MODEL（没有/meshes/的情况）
    std::string find_str2 = "package://ARV_V1_MODEL/";
    std::string replace_str2 = "";

    pos = 0;
    while ((pos = urdf_string.find(find_str2, pos)) != std::string::npos)
    {
        urdf_string.replace(pos, find_str2.length(), replace_str2);
        pos += replace_str2.length();
    }

    // ========== 步骤4: 写入临时文件（放在urdf目录） ==========
    std::string temp_urdf_path = "/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/.mujoco_temp.urdf";

    std::ofstream temp_file(temp_urdf_path);
    if (!temp_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法创建临时文件: %s", temp_urdf_path.c_str());
        return false;
    }

    temp_file << urdf_string;
    temp_file.close();

    RCLCPP_INFO(this->get_logger(), "✓ 临时URDF文件已创建: %s", temp_urdf_path.c_str());

    // ========== 步骤5: 第一次加载URDF（生成MJCF） ==========
    char error[1000] = "Could not load XML model";
    mjModel *temp_model = mj_loadXML(temp_urdf_path.c_str(), nullptr, error, 1000);

    if (!temp_model)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ MuJoCo加载URDF失败: %s", error);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "✓ URDF加载成功，准备转换为MJCF");

    // ========== 步骤6: 保存为MJCF格式 ==========
    std::string mjcf_path = "/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/.mujoco_converted.xml";

    if (mj_saveLastXML(mjcf_path.c_str(), temp_model, error, 1000) == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 保存MJCF失败: %s", error);
        mj_deleteModel(temp_model);
        return false;
    }

    mj_deleteModel(temp_model);
    RCLCPP_INFO(this->get_logger(), "✓ 已转换为MJCF格式: %s", mjcf_path.c_str());

    // ========== 步骤7: 修改MJCF添加执行器 ==========
    std::ifstream mjcf_file(mjcf_path);
    if (!mjcf_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法打开MJCF文件");
        return false;
    }

    std::string mjcf_string((std::istreambuf_iterator<char>(mjcf_file)),
                            std::istreambuf_iterator<char>());
    mjcf_file.close();

    // 在 </mujoco> 前插入 actuator 定义
    std::string actuator_mjcf =
        "\n  <actuator>\n"
        "    <motor name=\"actuator_1\" joint=\"joint_1\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "    <motor name=\"actuator_2\" joint=\"joint_2\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "    <motor name=\"actuator_3\" joint=\"joint_3\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "    <motor name=\"actuator_4\" joint=\"joint_4\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "    <motor name=\"actuator_5\" joint=\"joint_5\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "    <motor name=\"actuator_6\" joint=\"joint_6\" gear=\"1\" ctrllimited=\"true\" ctrlrange=\"-20 20\"/>\n"
        "  </actuator>\n";

    size_t mujoco_end = mjcf_string.find("</mujoco>");
    if (mujoco_end != std::string::npos)
    {
        mjcf_string.insert(mujoco_end, actuator_mjcf);
        RCLCPP_INFO(this->get_logger(), "✓ 已在MJCF中添加执行器定义");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法找到</mujoco>标签");
        return false;
    }

    // 保存修改后的MJCF
    std::string final_mjcf_path = "/home/huan/ros2_ws/src/ARV_V1_MODEL/urdf/.mujoco_final.xml";
    std::ofstream final_mjcf_file(final_mjcf_path);
    if (!final_mjcf_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 无法创建最终MJCF文件");
        return false;
    }
    final_mjcf_file << mjcf_string;
    final_mjcf_file.close();

    RCLCPP_INFO(this->get_logger(), "✓ 最终MJCF文件已创建: %s", final_mjcf_path.c_str());

    // ========== 步骤8: 重新加载最终的MJCF模型 ==========
    model_ = mj_loadXML(final_mjcf_path.c_str(), nullptr, error, 1000);

    if (!model_)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 加载最终MJCF失败: %s", error);
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "✅ MuJoCo模型加载成功（含执行器）");
    RCLCPP_INFO(this->get_logger(), "   关节数 (nq): %d", model_->nq);
    RCLCPP_INFO(this->get_logger(), "   速度维度 (nv): %d", model_->nv);
    RCLCPP_INFO(this->get_logger(), "   执行器数 (nu): %d", model_->nu);

    // ========== 步骤9: 创建MuJoCo数据结构 ==========
    data_ = mj_makeData(model_);

    if (!data_)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 创建MuJoCo数据结构失败");
        mj_deleteModel(model_);
        model_ = nullptr;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "✅ MuJoCo数据结构创建成功");

    // ========== 步骤10: 打印关节信息 ==========
    RCLCPP_INFO(this->get_logger(), "📋 关节列表:");
    for (int i = 0; i < model_->njnt; i++)
    {
        const char *jnt_name = mj_id2name(model_, mjOBJ_JOINT, i);
        int qpos_adr = model_->jnt_qposadr[i];
        RCLCPP_INFO(this->get_logger(), "   [%d] %s (qpos索引: %d)", i, jnt_name ? jnt_name : "unknown", qpos_adr);
    }

    // ========== 步骤11: 打印执行器信息 ==========
    RCLCPP_INFO(this->get_logger(), "📋 执行器列表:");
    for (int i = 0; i < model_->nu; i++)
    {
        const char *act_name = mj_id2name(model_, mjOBJ_ACTUATOR, i);
        int actuator_dyntype = model_->actuator_dyntype[i];
        mjtNum gear = model_->actuator_gear[i * 6]; // gear 的第一个元素

        const char *type_str = "unknown";
        if (actuator_dyntype == mjDYN_NONE)
            type_str = "直接力矩";
        else if (actuator_dyntype == mjDYN_INTEGRATOR)
            type_str = "位置伺服";
        else if (actuator_dyntype == mjDYN_FILTER)
            type_str = "滤波";

        RCLCPP_INFO(this->get_logger(),
                    "   [%d] %s | 类型: %s | 传动比: %.1f",
                    i, act_name ? act_name : "unknown", type_str, gear);
    }

    if (model_->nu == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 警告：没有执行器！力矩控制将无效！");
    }

    return true;
}

void MuJoCoInterfaceNode::setInitialPose()
{
    RCLCPP_INFO(this->get_logger(), "设置0位置初始位姿");
    double initial_q[6] = {0.0, 2.06, 0.766, 1.718, 0.0, 0.0};

    for (int i = 0; i < 6; i++)
    {
        data_->qpos[i] = initial_q[i];
    }

    // 前向运动学（更新模型状态）
    mj_forward(model_, data_);

    RCLCPP_INFO(this->get_logger(), "初始位姿设置完成: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                data_->qpos[0], data_->qpos[1], data_->qpos[2],
                data_->qpos[3], data_->qpos[4], data_->qpos[5]);
}

void MuJoCoInterfaceNode::effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 接受控制力矩
    if (msg->data.size() != 6)
    {
        RCLCPP_ERROR(this->get_logger(), "马德，力矩数组不是6！");
        return;
    }

    // 检查是否首次收到命令
    if (!received_first_command_)
    {
        received_first_command_ = true;
        RCLCPP_INFO(this->get_logger(), "✅ 收到首次力矩命令，MuJoCo仿真现在启动！");
    }

    for (size_t i = 0; i < 6; i++)
    {
        data_->ctrl[i] = msg->data[i];
    }
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // 每5秒打印一次
        "🎮 收到力矩命令: τ=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        data_->ctrl[0], data_->ctrl[1], data_->ctrl[2],
        data_->ctrl[3], data_->ctrl[4], data_->ctrl[5]);
}

void MuJoCoInterfaceNode::simulationStep()
{
    // ========== 安全检查：等待首次力矩命令 ==========
    if (!received_first_command_)
    {
        // 还没收到力矩命令，不执行仿真步进
        // 这样可以防止机械臂在启动时因重力下落
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,  // 每2秒提示一次
            "⏸️  等待力矩控制器发送命令...");

        // 仍然发布当前状态（初始位姿）
        publishJointStates();
        return;
    }

    // 线程安全：渲染线程可能同时访问data_
    std::lock_guard<std::mutex> lock(sim_mutex_);

    // 执行MuJoCo单步仿真
    mj_step(model_, data_);

    // 发布关节状态
    publishJointStates();
}

void MuJoCoInterfaceNode::publishJointStates()
{
    // 开始发布关节反馈！
    auto msg = sensor_msgs::msg::JointState();

    msg.header.stamp = this->now();
    msg.header.frame_id = "";
    msg.name = joint_names_;

    for (int i = 0; i < 6; i++)
    {
        msg.position.push_back(data_->qpos[i]); // 填写关节位置字段
    }

    for (int i = 0; i < 6; i++)
    {
        msg.velocity.push_back(data_->qvel[i]); // 填写关节速度字段
    }

    // 每5秒打印一次（避免刷屏）
    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000, // 毫秒
        "📊 关节状态: q=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        data_->qpos[0], data_->qpos[1], data_->qpos[2],
        data_->qpos[3], data_->qpos[4], data_->qpos[5]);

    // 反馈力矩不用于控制律，不填写了
    joint_state_pub_->publish(msg);
}

bool MuJoCoInterfaceNode::initializeVisualization()
{
    RCLCPP_INFO(this->get_logger(), "🎨 初始化MuJoCo可视化...");

    // ========== 初始化GLFW ==========
    if (!glfwInit())
    {
        RCLCPP_ERROR(this->get_logger(), "❌ GLFW初始化失败");
        return false;
    }

    // ========== 创建窗口 ==========
    window_ = glfwCreateWindow(1200, 900, "MuJoCo - ARV Robot", nullptr, nullptr);
    if (!window_)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ 创建GLFW窗口失败");
        glfwTerminate();
        return false;
    }

    // 注意：不在这里调用 glfwMakeContextCurrent，留给渲染线程

    // ========== 初始化MuJoCo可视化结构（不需要OpenGL上下文） ==========
    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scene_);
    mjr_defaultContext(&con_);

    // 创建场景（最大支持1000个几何体）
    mjv_makeScene(model_, &scene_, 1000);

    // 注意：mjr_makeContext 需要在渲染线程中调用（需要OpenGL上下文）

    // ========== 设置初始相机位置 ==========
    cam_.azimuth = 90.0;      // 水平角度
    cam_.elevation = -20.0;   // 垂直角度
    cam_.distance = 3.0;      // 距离
    cam_.lookat[0] = 0.0;     // 看向原点
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.5;

    RCLCPP_INFO(this->get_logger(), "✅ MuJoCo可视化初始化完成");
    return true;
}

void MuJoCoInterfaceNode::renderLoop()
{
    RCLCPP_INFO(this->get_logger(), "🎬 渲染循环开始");

    // ========== 在渲染线程中绑定OpenGL上下文 ==========
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // 启用垂直同步

    // ========== 初始化OpenGL渲染上下文（需要在有上下文的线程中） ==========
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);
    RCLCPP_INFO(this->get_logger(), "✅ OpenGL渲染上下文已初始化（在渲染线程中）");

    while (render_running_ && !glfwWindowShouldClose(window_))
    {
        // ========== 更新场景（线程安全） ==========
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            
            // 更新MuJoCo场景
            mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, 
                          mjCAT_ALL, &scene_);
        }

        // ========== 获取窗口尺寸 ==========
        mjrRect viewport;
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        viewport.left = 0;
        viewport.bottom = 0;

        // ========== 渲染场景 ==========
        mjr_render(viewport, &scene_, &con_);

        // ========== 交换缓冲区并处理事件 ==========
        glfwSwapBuffers(window_);
        glfwPollEvents();

        // 渲染频率约60Hz（由垂直同步控制）
    }

    RCLCPP_INFO(this->get_logger(), "🎬 渲染循环结束");
}


// ========== main函数 ==========

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<MuJoCoInterfaceNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("mujoco_interface"), "节点启动失败: %s", e.what());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
