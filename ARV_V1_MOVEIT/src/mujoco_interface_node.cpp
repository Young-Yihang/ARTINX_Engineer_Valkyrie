// MuJoCo节点: 仿真模式(力矩→状态) / 孪生模式(状态→视觉化)

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>
#include <vector>
#include <thread>
#include <fstream>
#include <sstream>
#include <chrono>
#include <atomic>
#include <cmath>
#include <filesystem>

class MuJoCoInterfaceNode : public rclcpp::Node
{
public:
    MuJoCoInterfaceNode() : Node("mujoco_interface"),
                            model_(nullptr),
                            data_(nullptr),
                            sim_frequency_(200.0),
                            received_first_command_(false),
                            visualization_only_(false)
    {
        RCLCPP_INFO(this->get_logger(), "[START] MuJoCo interface node starting");

        // ========== 声明并获取运行模式参数 ==========
        this->declare_parameter("visualization_only", false);
        visualization_only_ = this->get_parameter("visualization_only").as_bool();

        if (visualization_only_)
        {
            RCLCPP_INFO(this->get_logger(), "[MODE] Digital Twin - visualization only (subscribing /joint_states)");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "[MODE] Physics Simulation (subscribing /effort_controller/commands)");
        }

        // 动态获取包路径（编译时查表，零运行时开销）
        try
        {
            pkg_share_dir_ = ament_index_cpp::get_package_share_directory("ARV_V1_MODEL");
            urdf_path_ = pkg_share_dir_ + "/urdf/ARV_V1_MODEL.urdf";
            mesh_dir_ = pkg_share_dir_ + "/meshes";

            RCLCPP_INFO(this->get_logger(), "[OK] Package path: %s", pkg_share_dir_.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to find ARV_V1_MODEL package: %s", e.what());
            throw;
        }

        // 创建临时目录（使用 /tmp 避免权限问题）
        temp_dir_ = std::filesystem::temp_directory_path() / "arv_v1_mujoco";
        std::filesystem::create_directories(temp_dir_);
        RCLCPP_INFO(this->get_logger(), "[OK] Temp directory: %s", temp_dir_.c_str());

        // ========== 步骤1: 加载MuJoCo模型 ==========
        if (!loadMuJoCoModel())
        {
            RCLCPP_ERROR(this->get_logger(), "[ERROR] MuJoCo model loading failed");
            throw std::runtime_error("Failed to load MuJoCo model");
        }
        RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo model loaded successfully");
        RCLCPP_INFO(this->get_logger(), "   Number of joints: %d", model_->nq);

        // ========== 步骤2: 设置初始位姿 ==========
        setInitialPose();

        // ========== 步骤3: 根据模式设置话题通信 ==========
        if (visualization_only_)
        {
            // 数字孪生模式: 订阅 /joint_states，不发布
            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&MuJoCoInterfaceNode::jointStateCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "[OK] Subscribing: /joint_states (digital twin)");
        }
        else
        {
            // 物理仿真模式: 订阅力矩，发布关节状态
            effort_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
                "/effort_controller/commands", 10,
                std::bind(&MuJoCoInterfaceNode::effortCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "[OK] Subscribing: /effort_controller/commands");

            joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);
            RCLCPP_INFO(this->get_logger(), "[OK] Publishing: /joint_states");

            // 仿真模式: 200Hz定时器
            auto period = std::chrono::duration<double, std::milli>(1000.0 / sim_frequency_);
            sim_timer_ = this->create_wall_timer(period,
                                                 std::bind(&MuJoCoInterfaceNode::simulationStep, this));
            RCLCPP_INFO(this->get_logger(), "[INFO] Simulation frequency: %.1f Hz", sim_frequency_);
        }
        RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo interface node initialization completed");

        // ========== 步骤5.5: 启动健康监控定时器 (5Hz) ==========
        health_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5000),
            std::bind(&MuJoCoInterfaceNode::healthCheck, this));

        // ========== 步骤6: 初始化并启动可视化 ==========
        render_running_ = false;
        window_ = nullptr;

        if (initializeVisualization())
        {
            RCLCPP_INFO(this->get_logger(), "[OK] Visualization initialized successfully");
            render_running_ = true;
            render_thread_ = std::thread(&MuJoCoInterfaceNode::renderLoop, this);
            RCLCPP_INFO(this->get_logger(), "[INFO] Rendering thread started");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "[WARN] Visualization initialization failed, simulation only");
        }
    }

    ~MuJoCoInterfaceNode()
    {
        RCLCPP_INFO(this->get_logger(), "[INFO] Cleaning up MuJoCo resources...");

        // 停止渲染线程
        render_running_ = false;
        if (render_thread_.joinable())
        {
            render_thread_.join();
        }

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

        RCLCPP_INFO(this->get_logger(), "[OK] Resource cleanup completed");
    }

private:
    // ========== MuJoCo相关成员变量 ==========
    mjModel *model_;
    mjData *data_;

    // ========== ROS2接口 ==========
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr effort_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; // 数字孪生模式
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr sim_timer_;

    // ========== 运行模式 ==========
    bool visualization_only_; // true=数字孪生模式，false=物理仿真模式

    // ========== 配置参数 ==========
    std::string pkg_share_dir_;      // ARV_V1_MODEL 包共享目录
    std::string urdf_path_;          // URDF 文件路径
    std::string mesh_dir_;           // Mesh 文件目录
    std::filesystem::path temp_dir_; // 临时文件目录
    double sim_frequency_;

    // ========== 可视化相关成员变量 ==========
    std::thread render_thread_;
    std::atomic<bool> render_running_;

    GLFWwindow *window_;
    mjvScene scene_;
    mjvCamera cam_;
    mjvOption opt_;
    mjrContext con_;
    std::mutex sim_mutex_;

    // ========== 启动安全相关 ==========
    std::atomic<bool> received_first_command_;

    // ========== Health monitoring ==========
    std::atomic<uint64_t> sim_step_count_{0};
    std::atomic<uint64_t> command_rx_count_{0};
    rclcpp::TimerBase::SharedPtr health_timer_;

    // ========== 关节名称 ==========
    const std::vector<std::string> joint_names_ = {
        "joint_1", "joint_2", "joint_3",
        "joint_4", "joint_5", "joint_6"};

    // ========== 交互状态变量 ==========
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_ = 0.0;
    double lasty_ = 0.0;

    bool paused_ = false;
    bool show_ui_ = true;
    bool show_contacts_ = false;
    bool show_forces_ = false;

    // ========== 静态回调函数声明 ==========
    static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods);
    static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow *window, double xoffset, double yoffset);
    static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods);

    // ========== 成员函数声明 ==========
    bool loadMuJoCoModel();
    void setInitialPose();
    void effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg); // 数字孪生
    void simulationStep();
    void publishJointStates();
    bool initializeVisualization();
    void renderLoop();
    void healthCheck();
};

// ========== 成员函数实现 ==========

bool MuJoCoInterfaceNode::loadMuJoCoModel()
{
    RCLCPP_INFO(this->get_logger(), "[INFO] Loading URDF: %s", urdf_path_.c_str());

    // 读取URDF文件
    std::ifstream urdf_file(urdf_path_);
    if (!urdf_file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot open URDF file: %s", urdf_path_.c_str());
        return false;
    }

    std::string urdf_string((std::istreambuf_iterator<char>(urdf_file)),
                            std::istreambuf_iterator<char>());
    urdf_file.close();

    RCLCPP_INFO(this->get_logger(), "[OK] URDF file read successfully");

    // 插入MuJoCo编译器设置（使用动态获取的 mesh 目录）
    std::string mujoco_compiler =
        "\n  <mujoco>\n"
        "    <compiler meshdir=\"" +
        mesh_dir_ + "\" strippath=\"false\"/>\n"
                    "    <option timestep=\"0.005\"/>\n"
                    "    <size nconmax=\"0\" njmax=\"0\"/>\n"
                    "  </mujoco>\n";

    size_t robot_pos = urdf_string.find("<robot");
    if (robot_pos == std::string::npos)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Cannot find <robot> tag");
        return false;
    }

    size_t bracket_pos = urdf_string.find(">", robot_pos);
    urdf_string.insert(bracket_pos + 1, mujoco_compiler);

    // 替换mesh路径
    std::string find_str = "package://ARV_V1_MODEL/meshes/";
    std::string replace_str = "";
    size_t pos = 0;
    while ((pos = urdf_string.find(find_str, pos)) != std::string::npos)
    {
        urdf_string.replace(pos, find_str.length(), replace_str);
        pos += replace_str.length();
    }

    // 写入临时文件（使用 /tmp 目录）
    std::string temp_urdf_path = temp_dir_ / ".mujoco_temp.urdf";
    std::ofstream temp_file(temp_urdf_path);
    temp_file << urdf_string;
    temp_file.close();

    // 加载URDF
    char error[1000] = "Could not load XML model";
    mjModel *temp_model = mj_loadXML(temp_urdf_path.c_str(), nullptr, error, 1000);
    if (!temp_model)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] MuJoCo failed to load URDF: %s", error);
        return false;
    }

    // 保存为MJCF（使用 /tmp 目录）
    std::string mjcf_path = temp_dir_ / ".mujoco_converted.xml";
    mj_saveLastXML(mjcf_path.c_str(), temp_model, error, 1000);
    mj_deleteModel(temp_model);

    // 修改MJCF添加执行器
    std::ifstream mjcf_file(mjcf_path);
    std::string mjcf_string((std::istreambuf_iterator<char>(mjcf_file)),
                            std::istreambuf_iterator<char>());
    mjcf_file.close();

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
    mjcf_string.insert(mujoco_end, actuator_mjcf);

    // 禁用碰撞
    pos = 0;
    while ((pos = mjcf_string.find("<geom", pos)) != std::string::npos)
    {
        size_t geom_end = mjcf_string.find("/>", pos);
        if (geom_end == std::string::npos)
            geom_end = mjcf_string.find(">", pos);

        std::string geom_tag = mjcf_string.substr(pos, geom_end - pos);
        if (geom_tag.find("contype") == std::string::npos)
        {
            mjcf_string.insert(geom_end, " contype=\"0\" conaffinity=\"0\"");
        }
        pos = geom_end + 1;
    }

    // 保存最终MJCF（使用 /tmp 目录）
    std::string final_mjcf_path = temp_dir_ / ".mujoco_final.xml";
    std::ofstream final_mjcf_file(final_mjcf_path);
    final_mjcf_file << mjcf_string;
    final_mjcf_file.close();

    // 加载最终模型
    model_ = mj_loadXML(final_mjcf_path.c_str(), nullptr, error, 1000);
    if (!model_)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to load final MJCF: %s", error);
        return false;
    }

    data_ = mj_makeData(model_);
    if (!data_)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Failed to create MuJoCo data structure");
        mj_deleteModel(model_);
        model_ = nullptr;
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo model loaded successfully");
    return true;
}

void MuJoCoInterfaceNode::setInitialPose()
{
    double initial_q[6] = {0.0, 2.06, 0.766, 1.718, 0.0, 0.0};
    for (int i = 0; i < 6; i++)
    {
        data_->qpos[i] = initial_q[i];
    }
    mj_forward(model_, data_);
    RCLCPP_INFO(this->get_logger(), "[OK] Initial pose set");
}

void MuJoCoInterfaceNode::effortCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != 6)
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] Torque array size mismatch!");
        return;
    }

    if (!received_first_command_)
    {
        received_first_command_ = true;
        RCLCPP_INFO(this->get_logger(), "[OK] First torque command received, MuJoCo simulation started");
    }

    command_rx_count_++;

    for (size_t i = 0; i < 6; i++)
    {
        data_->ctrl[i] = msg->data[i];
    }
}

// 数字孪生模式: 接收外部关节状态，更新MuJoCo显示
void MuJoCoInterfaceNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->position.size() < 6)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[WARN] JointState size < 6, ignoring");
        return;
    }

    std::lock_guard<std::mutex> lock(sim_mutex_);
    // 更新 MuJoCo qpos 用于3D渲染
    for (size_t i = 0; i < 6 && i < msg->position.size(); ++i)
    {
        data_->qpos[i] = msg->position[i];
    }
    // 更新前向运动学（仅用于渲染，不做物理仿真）
    mj_forward(model_, data_);
}

void MuJoCoInterfaceNode::simulationStep()
{
    if (!received_first_command_)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "[WARN] Waiting for torque command from controller...");
        publishJointStates();
        return;
    }

    if (paused_)
    {
        publishJointStates();
        return;
    }

    std::lock_guard<std::mutex> lock(sim_mutex_);
    mj_step(model_, data_);
    sim_step_count_++;
    publishJointStates();
}

void MuJoCoInterfaceNode::publishJointStates()
{
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = joint_names_;

    for (int i = 0; i < 6; i++)
    {
        msg.position.push_back(data_->qpos[i]);
        msg.velocity.push_back(data_->qvel[i]);
    }

    joint_state_pub_->publish(msg);
}

bool MuJoCoInterfaceNode::initializeVisualization()
{
    RCLCPP_INFO(this->get_logger(), "[INFO] Initializing MuJoCo visualization...");

    if (!glfwInit())
    {
        RCLCPP_ERROR(this->get_logger(), "[ERROR] GLFW initialization failed");
        return false;
    }

    window_ = glfwCreateWindow(1200, 900, "MuJoCo - ARV Robot", nullptr, nullptr);
    if (!window_)
    {
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

    opt_.flags[mjVIS_JOINT] = 1;
    opt_.flags[mjVIS_ACTUATOR] = 1;

    mjv_makeScene(model_, &scene_, 1000);

    cam_.azimuth = 90.0;
    cam_.elevation = -20.0;
    cam_.distance = 3.0;
    cam_.lookat[0] = 0.0;
    cam_.lookat[1] = 0.0;
    cam_.lookat[2] = 0.5;

    RCLCPP_INFO(this->get_logger(), "[OK] MuJoCo visualization initialized");
    RCLCPP_INFO(this->get_logger(), "[INFO] Keys: Space-Pause | H-Hide UI | R-Reset Camera | ESC-Exit");

    return true;
}
void MuJoCoInterfaceNode::renderLoop()
{
    RCLCPP_INFO(this->get_logger(), "[INFO] Render loop started");

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    while (render_running_ && !glfwWindowShouldClose(window_))
    {
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
        }

        mjrRect viewport;
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);
        viewport.left = 0;
        viewport.bottom = 0;

        mjr_render(viewport, &scene_, &con_);

        // ========== 修改：使用 mjr_text 手动指定每行位置 ==========
        if (show_ui_)
        {
            char status[512];
            std::lock_guard<std::mutex> lock(sim_mutex_);

            // 定义起始位置和行间距
            int start_x = 10;                   // 左边距（像素）
            int start_y = viewport.height - 30; // 从顶部开始（像素）
            int line_height = 20;               // 行高（像素）
            int current_line = 0;               // 当前行号

            // 第 1 行：仿真时间和状态
            snprintf(status, sizeof(status),
                     "时间: %.2f s | %s",
                     data_->time,
                     paused_ ? "暂停" : "运行");
            mjr_text(mjFONT_NORMAL, status, &con_,
                     start_x, start_y - (current_line++) * line_height,
                     1.0f, 1.0f, 1.0f); // 白色文本

            // 第 2 行：关节位置
            snprintf(status, sizeof(status),
                     "位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad",
                     data_->qpos[0], data_->qpos[1], data_->qpos[2],
                     data_->qpos[3], data_->qpos[4], data_->qpos[5]);
            mjr_text(mjFONT_NORMAL, status, &con_,
                     start_x, start_y - (current_line++) * line_height,
                     1.0f, 1.0f, 0.0f); // 黄色文本（位置信息）

            // 第 3 行：关节速度
            snprintf(status, sizeof(status),
                     "速度: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] rad/s",
                     data_->qvel[0], data_->qvel[1], data_->qvel[2],
                     data_->qvel[3], data_->qvel[4], data_->qvel[5]);
            mjr_text(mjFONT_NORMAL, status, &con_,
                     start_x, start_y - (current_line++) * line_height,
                     0.0f, 1.0f, 1.0f); // 青色文本（速度信息）

            // 第 4 行：执行器力矩
            snprintf(status, sizeof(status),
                     "力矩: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] N·m",
                     data_->ctrl[0], data_->ctrl[1], data_->ctrl[2],
                     data_->ctrl[3], data_->ctrl[4], data_->ctrl[5]);
            mjr_text(mjFONT_NORMAL, status, &con_,
                     start_x, start_y - (current_line++) * line_height,
                     1.0f, 0.5f, 0.0f); // 橙色文本（力矩信息）

            // ========== 右下角：快捷键提示 ==========
            snprintf(status, sizeof(status),
                     "快捷键: [空格]暂停 [H]隐藏UI [R]重置相机 [ESC]退出");
            mjr_text(mjFONT_NORMAL, status, &con_,
                     10, 10,            // 左下角位置
                     0.7f, 0.7f, 0.7f); // 灰色文本（提示信息）
        }

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    RCLCPP_INFO(this->get_logger(), "[INFO] Render loop ended");
}

// ========== 静态回调函数实现 ==========

void MuJoCoInterfaceNode::mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
    if (!node)
        return;

    if (action == GLFW_PRESS)
    {
        if (button == GLFW_MOUSE_BUTTON_LEFT)
            node->button_left_ = true;
        else if (button == GLFW_MOUSE_BUTTON_MIDDLE)
            node->button_middle_ = true;
        else if (button == GLFW_MOUSE_BUTTON_RIGHT)
            node->button_right_ = true;
        glfwGetCursorPos(window, &node->lastx_, &node->lasty_);
    }
    else if (action == GLFW_RELEASE)
    {
        node->button_left_ = false;
        node->button_middle_ = false;
        node->button_right_ = false;
    }
}

void MuJoCoInterfaceNode::mouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
{
    auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
    if (!node)
        return;

    if (!node->button_left_ && !node->button_middle_ && !node->button_right_)
        return;

    double dx = xpos - node->lastx_;
    double dy = ypos - node->lasty_;
    node->lastx_ = xpos;
    node->lasty_ = ypos;

    // 左键：旋转视角
    if (node->button_left_)
    {
        node->cam_.azimuth += dx * 0.3;
        node->cam_.elevation += dy * 0.3;
    }
    // 右键：平移（简化版，直接修改 lookat）
    else if (node->button_right_)
    {
        double moveScale = 0.001 * node->cam_.distance;

        // 计算相机方向（简化版）
        double azimuth_rad = node->cam_.azimuth * M_PI / 180.0;
        double right_x = -std::sin(azimuth_rad);
        double right_y = std::cos(azimuth_rad);

        node->cam_.lookat[0] -= moveScale * (dx * right_x);
        node->cam_.lookat[1] -= moveScale * (dx * right_y);
        node->cam_.lookat[2] += moveScale * dy;
    }
    // 中键：缩放
    else if (node->button_middle_)
    {
        node->cam_.distance *= (1.0 - dy * 0.01);
        if (node->cam_.distance < 0.1)
            node->cam_.distance = 0.1;
        if (node->cam_.distance > 50.0)
            node->cam_.distance = 50.0;
    }
}

void MuJoCoInterfaceNode::scrollCallback(GLFWwindow *window, double xoffset, double yoffset)
{
    auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
    if (!node)
        return;

    node->cam_.distance *= (1.0 - yoffset * 0.05);
    if (node->cam_.distance < 0.1)
        node->cam_.distance = 0.1;
    if (node->cam_.distance > 50.0)
        node->cam_.distance = 50.0;
}

void MuJoCoInterfaceNode::keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
    auto *node = static_cast<MuJoCoInterfaceNode *>(glfwGetWindowUserPointer(window));
    if (!node)
        return;

    if (action != GLFW_PRESS)
        return;

    switch (key)
    {
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

    case GLFW_KEY_R:
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

void MuJoCoInterfaceNode::healthCheck()
{
    static uint64_t last_step_count = 0;
    static uint64_t last_cmd_count = 0;

    uint64_t current_steps = sim_step_count_.load();
    uint64_t current_cmds = command_rx_count_.load();

    double step_rate = (current_steps - last_step_count) / 5.0;  // 5 sec interval
    double cmd_rate = (current_cmds - last_cmd_count) / 5.0;

    if (visualization_only_)
    {
        RCLCPP_INFO(this->get_logger(),
                    "[HEALTH] Mode: DIGITAL TWIN | Render: %s",
                    render_running_.load() ? "OK" : "STOPPED");
    }
    else
    {
        if (!received_first_command_.load())
        {
            RCLCPP_WARN(this->get_logger(),
                        "[HEALTH] Mode: SIMULATION | Waiting for first command...");
        }
        else if (paused_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[HEALTH] Mode: SIMULATION | Status: PAUSED | Render: %s",
                        render_running_.load() ? "OK" : "STOPPED");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(),
                        "[HEALTH] Mode: SIMULATION | Sim: %.1f Hz | Cmd RX: %.1f Hz | Render: %s",
                        step_rate, cmd_rate,
                        render_running_.load() ? "OK" : "STOPPED");
        }
    }

    last_step_count = current_steps;
    last_cmd_count = current_cmds;
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