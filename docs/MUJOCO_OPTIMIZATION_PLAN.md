# MuJoCo仿真节点优化规划

**创建日期**: 2026-02-02
**目标**: 将mujoco_interface_node升级为功能完善的数字孪生与仿真节点
**约束**: C++代码不变原则（仅扩展，不重构核心逻辑）

---

## 一、当前状态分析

### 1.1 已实现功能
| 功能 | 状态 | 文件位置 |
|------|------|----------|
| 机械臂3D渲染 | ✅ | `mujoco_interface_node.cpp:503-586` |
| 鼠标视角控制（旋转/平移/缩放） | ✅ | `mujoco_interface_node.cpp:590-670` |
| 键盘快捷键（暂停/UI/重置相机） | ✅ | `mujoco_interface_node.cpp:672-716` |
| 200Hz物理仿真定时器 | ✅ | `mujoco_interface_node.cpp:98-102` |
| 数字孪生模式（visualization_only） | ✅ | `mujoco_interface_node.cpp:78-84` |
| 基础状态文本显示 | ✅ | `mujoco_interface_node.cpp:526-579` |

### 1.2 缺失功能（本次优化目标）
| 功能 | 优先级 | 说明 |
|------|--------|------|
| ImGui实时曲线显示 | P0 | 力矩曲线、跟踪误差曲线 |
| 目标位置Topic订阅 | P0 | 用于计算跟踪误差 |
| 能量单元场景 | P2 | 2026赛季道具静态显示+基础拾取 |
| 地面/环境渲染 | P2 | 增强视觉效果 |

---

## 二、系统架构设计

### 2.1 DDS通信拓扑（优化后）

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ROS2 DDS Communication Layer                       │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────┐                                                    │
│  │ trajectory_manager  │                                                    │
│  │     _node           │                                                    │
│  └──────────┬──────────┘                                                    │
│             │ /ARM_controller/follow_joint_trajectory (Action)              │
│             ▼                                                               │
│  ┌─────────────────────┐                                                    │
│  │ torque_controller   │                                                    │
│  │      _node          │                                                    │
│  │  [200Hz控制循环]     │                                                    │
│  └──────────┬──────────┘                                                    │
│             │                                                               │
│             ├──────────────────────────────────────────┐                    │
│             │                                          │                    │
│             ▼                                          ▼                    │
│  ┌──────────────────────┐               ┌──────────────────────┐           │
│  │ /effort_controller   │               │ /target_positions    │ ◀─ 新增   │
│  │     /commands        │               │ (Float64MultiArray)  │           │
│  │ (Float64MultiArray)  │               │ 发布频率: 200Hz      │           │
│  └──────────┬───────────┘               └──────────┬───────────┘           │
│             │                                      │                        │
│             │                                      │                        │
│             ▼                                      ▼                        │
│  ┌──────────────────────────────────────────────────────────────┐          │
│  │                  mujoco_interface_node                        │          │
│  │  ┌────────────────────────────────────────────────────────┐  │          │
│  │  │  订阅:                                                  │  │          │
│  │  │  • /effort_controller/commands  (力矩命令)             │  │          │
│  │  │  • /target_positions            (目标位置) [新增]      │  │          │
│  │  │  • /joint_states                (孪生模式)             │  │          │
│  │  │                                                        │  │          │
│  │  │  发布:                                                  │  │          │
│  │  │  • /joint_states                (仿真模式)             │  │          │
│  │  └────────────────────────────────────────────────────────┘  │          │
│  │                                                               │          │
│  │  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐  │          │
│  │  │ MuJoCo引擎  │  │ ImGui/ImPlot│  │ 场景管理器           │  │          │
│  │  │ 200Hz物理   │  │ 实时曲线    │  │ (能量单元/地面)      │  │          │
│  │  └─────────────┘  └─────────────┘  └──────────────────────┘  │          │
│  └───────────────────────────────────────────────────────────────┘          │
│             │                                                               │
│             │ /joint_states (仿真模式发布)                                  │
│             ▼                                                               │
│  ┌─────────────────────┐                                                    │
│  │ torque_controller   │◀── 闭环反馈                                        │
│  └─────────────────────┘                                                    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.2 新增Topic定义

| Topic | 消息类型 | 发布者 | 订阅者 | 频率 | 用途 |
|-------|----------|--------|--------|------|------|
| `/target_positions` | `std_msgs/Float64MultiArray` | torque_controller | mujoco_interface | 200Hz | 目标关节位置，用于计算跟踪误差 |

### 2.3 数据流向

```
仿真模式:
  torque_controller ──[/effort_controller/commands]──▶ mujoco_interface
  torque_controller ──[/target_positions]────────────▶ mujoco_interface
  mujoco_interface  ──[/joint_states]────────────────▶ torque_controller

孪生模式:
  hardware_interface ──[/joint_states]───────────────▶ mujoco_interface (只读可视化)
  torque_controller  ──[/target_positions]───────────▶ mujoco_interface (误差显示)
```

---

## 三、阶段性开发计划

### 阶段1: ImGui集成与曲线显示 [P0]

#### 3.1.1 依赖安装
```bash
# Ubuntu系统包安装
sudo apt update
sudo apt install libimgui-dev libimplot-dev

# 如果apt没有implot，使用源码方式:
# 1. 下载 https://github.com/epezent/implot
# 2. 将 implot.h, implot.cpp, implot_items.cpp 放入项目
```

#### 3.1.2 CMakeLists.txt修改
```cmake
# 在 find_package 区域添加
find_package(imgui REQUIRED)
# 或手动指定源文件路径

# 在 target_link_libraries 添加
target_link_libraries(mujoco_interface_node
  # ... 现有依赖 ...
  imgui
  # implot (如果独立编译)
)
```
   
#### 3.1.3 数据结构设计
```cpp
// 环形缓冲区 - 存储最近5秒数据 (200Hz × 5s = 1000点)
class RingBuffer {
public:
    static constexpr size_t CAPACITY = 1000;

    void push(float value) {
        data_[write_idx_] = value;
        write_idx_ = (write_idx_ + 1) % CAPACITY;
        if (size_ < CAPACITY) size_++;
    }

    // ImPlot需要的接口
    float operator[](size_t idx) const {
        return data_[(write_idx_ - size_ + idx + CAPACITY) % CAPACITY];
    }

    size_t size() const { return size_; }

private:
    std::array<float, CAPACITY> data_{};
    size_t write_idx_ = 0;
    size_t size_ = 0;
};

// 曲线数据管理
struct PlotData {
    std::array<RingBuffer, 6> torque_cmd;     // 力矩命令
    std::array<RingBuffer, 6> torque_actual;  // 实际力矩(如有)
    std::array<RingBuffer, 6> pos_error;      // 位置跟踪误差
    std::array<RingBuffer, 6> pos_target;     // 目标位置
    std::array<RingBuffer, 6> pos_actual;     // 实际位置
    RingBuffer time_stamps;                   // 时间戳
};
```

#### 3.1.4 窗口布局设计
```
┌─────────────────────────────────────────────────────────────────────┐
│  MuJoCo - ARV Robot Simulation                                  [×] │
├───────────────────────────────────────────┬─────────────────────────┤
│                                           │ ┌─────────────────────┐ │
│                                           │ │ 力矩曲线 (N·m)    ▼│ │
│                                           │ │ ┌─────────────────┐ │ │
│        [3D机械臂渲染区域]                  │ │ │    J1-J6实时    │ │ │
│             ~70%宽度                       │ │ │    力矩曲线     │ │ │
│                                           │ │ │   (叠加显示)    │ │ │
│                                           │ │ └─────────────────┘ │ │
│                                           │ └─────────────────────┘ │
│                                           │ ┌─────────────────────┐ │
│                                           │ │ 跟踪误差 (rad)    ▼│ │
│                                           │ │ ┌─────────────────┐ │ │
│                                           │ │ │   目标-实际     │ │ │
│                                           │ │ │   位置差值      │ │ │
│                                           │ │ │  (6轴叠加)      │ │ │
│                                           │ │ └─────────────────┘ │ │
│                                           │ └─────────────────────┘ │
│                                           │                         │
│                                           │ [▶运行] [||暂停] [⟲重置]│
├───────────────────────────────────────────┴─────────────────────────┤
│ 状态: 运行中 | 仿真时间: 12.35s | 控制频率: 199.8Hz                  │
└─────────────────────────────────────────────────────────────────────┘
```

#### 3.1.5 ImGui渲染集成代码框架
```cpp
// 在renderLoop()中集成
void MuJoCoInterfaceNode::renderLoop() {
    // === OpenGL/GLFW初始化 (现有代码) ===
    glfwMakeContextCurrent(window_);
    mjr_makeContext(model_, &con_, mjFONTSCALE_150);

    // === ImGui初始化 [新增] ===
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // 设置ImGui样式
    ImGui::StyleColorsDark();

    while (render_running_ && !glfwWindowShouldClose(window_)) {
        // === MuJoCo场景更新与渲染 (现有代码) ===
        {
            std::lock_guard<std::mutex> lock(sim_mutex_);
            mjv_updateScene(model_, data_, &opt_, nullptr, &cam_, mjCAT_ALL, &scene_);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        // 计算3D视图区域 (左侧70%)
        mjrRect view3d = viewport;
        view3d.width = static_cast<int>(viewport.width * 0.7);
        mjr_render(view3d, &scene_, &con_);

        // === ImGui帧开始 [新增] ===
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 绘制右侧曲线面板
        drawPlotPanel(viewport);

        // === ImGui渲染 [新增] ===
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
        glfwPollEvents();
    }

    // === ImGui清理 [新增] ===
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
}

void MuJoCoInterfaceNode::drawPlotPanel(const mjrRect& viewport) {
    // 右侧面板位置和大小
    float panel_width = viewport.width * 0.3f;
    float panel_x = viewport.width * 0.7f;

    ImGui::SetNextWindowPos(ImVec2(panel_x, 0), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(panel_width, viewport.height), ImGuiCond_Always);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoCollapse;

    if (ImGui::Begin("数据监控", nullptr, flags)) {
        // 力矩曲线
        if (ImGui::CollapsingHeader("力矩曲线 (N·m)", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImPlot::BeginPlot("##Torque", ImVec2(-1, 200))) {
                ImPlot::SetupAxes("时间(s)", "力矩(N·m)");
                ImPlot::SetupAxisLimits(ImAxis_X1, -5, 0, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -25, 25);

                const char* labels[] = {"J1", "J2", "J3", "J4", "J5", "J6"};
                for (int i = 0; i < 6; i++) {
                    // 绘制每个关节的力矩曲线
                    // ImPlot::PlotLine(labels[i], ...);
                }
                ImPlot::EndPlot();
            }
        }

        // 跟踪误差曲线
        if (ImGui::CollapsingHeader("跟踪误差 (rad)", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImPlot::BeginPlot("##Error", ImVec2(-1, 200))) {
                ImPlot::SetupAxes("时间(s)", "误差(rad)");
                ImPlot::SetupAxisLimits(ImAxis_X1, -5, 0, ImGuiCond_Always);
                ImPlot::SetupAxisLimits(ImAxis_Y1, -0.1, 0.1);

                for (int i = 0; i < 6; i++) {
                    // 绘制每个关节的跟踪误差
                }
                ImPlot::EndPlot();
            }
        }

        // 状态信息
        ImGui::Separator();
        ImGui::Text("仿真时间: %.2f s", data_->time);
        ImGui::Text("状态: %s", paused_ ? "暂停" : "运行中");
    }
    ImGui::End();
}
```

---

### 阶段2: torque_controller新增Topic [P0]

#### 3.2.1 修改说明
需要在 `torque_controller_node.cpp` 中添加目标位置发布功能。

**修改位置**: `src/control/torque_controller_node.cpp`

#### 3.2.2 代码修改要点
```cpp
// 1. 添加Publisher成员变量
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_pub_;

// 2. 在构造函数中初始化
target_pos_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/target_positions", 10);

// 3. 在控制循环中发布 (与effort命令同步)
void publishTargetPositions() {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.resize(6);
    for (int i = 0; i < 6; i++) {
        msg.data[i] = target_positions_[i];  // 使用现有的目标位置变量
    }
    target_pos_pub_->publish(msg);
}
```

#### 3.2.3 MuJoCo节点订阅实现
```cpp
// 成员变量
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_pos_sub_;
std::array<double, 6> target_positions_{};
std::array<double, 6> position_errors_{};

// 初始化订阅
target_pos_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/target_positions", 10,
    std::bind(&MuJoCoInterfaceNode::targetPosCallback, this, std::placeholders::_1));

// 回调函数
void MuJoCoInterfaceNode::targetPosCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 6) return;

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < 6; i++) {
        target_positions_[i] = msg->data[i];
        position_errors_[i] = target_positions_[i] - data_->qpos[i];

        // 更新曲线数据
        plot_data_.pos_error[i].push(static_cast<float>(position_errors_[i]));
        plot_data_.pos_target[i].push(static_cast<float>(target_positions_[i]));
    }
}
```

---

### 阶段3: 能量单元场景 [P2]

#### 3.3.1 场景文件结构
```
arv_v1_model/
├── urdf/
│   └── arv_v1_model.urdf
├── meshes/
│   ├── ... (现有机械臂mesh)
│   └── energy_unit.stl        [新增] 能量单元模型
└── mjcf/                       [新增] MuJoCo场景目录
    ├── scene_base.xml          基础场景(地面、光照)
    ├── energy_units.xml        能量单元定义
    └── arv_simulation.xml      完整仿真场景(include all)
```

#### 3.3.2 基础场景定义 (scene_base.xml)
```xml
<mujoco>
  <!-- 全局设置 -->
  <option timestep="0.005" gravity="0 0 -9.81"/>

  <!-- 视觉资源 -->
  <asset>
    <texture name="grid" type="2d" builtin="checker"
             rgb1="0.2 0.2 0.2" rgb2="0.3 0.3 0.3"
             width="512" height="512"/>
    <material name="grid_mat" texture="grid" texrepeat="5 5"/>
  </asset>

  <!-- 地面 -->
  <worldbody>
    <light name="light1" pos="0 0 3" dir="0 0 -1" diffuse="1 1 1"/>
    <light name="light2" pos="2 2 2" dir="-1 -1 -1" diffuse="0.5 0.5 0.5"/>

    <geom name="floor" type="plane" size="3 3 0.1"
          material="grid_mat" contype="1" conaffinity="1"/>
  </worldbody>
</mujoco>
```

#### 3.3.3 能量单元定义 (energy_units.xml)
```xml
<mujoco>
  <asset>
    <!-- 如果有STL文件 -->
    <mesh name="energy_unit_mesh" file="meshes/energy_unit.stl" scale="0.001 0.001 0.001"/>

    <!-- 材质 -->
    <material name="gold" rgba="1.0 0.84 0 1" specular="0.8" shininess="0.9"/>
    <material name="silver" rgba="0.75 0.75 0.75 1" specular="0.9" shininess="0.95"/>
  </asset>

  <worldbody>
    <!-- 能量单元1 - 使用简单几何体(无STL时) -->
    <body name="energy_unit_1" pos="0.4 0.2 0.05">
      <freejoint name="eu1_joint"/>
      <geom name="eu1_geom" type="box" size="0.035 0.035 0.05"
            material="gold" mass="0.1"
            contype="1" conaffinity="1" friction="0.8 0.8 0.8"/>
      <site name="eu1_grasp" pos="0 0 0.05" size="0.01"/>
    </body>

    <!-- 能量单元2 -->
    <body name="energy_unit_2" pos="0.5 -0.1 0.05">
      <freejoint name="eu2_joint"/>
      <geom name="eu2_geom" type="box" size="0.035 0.035 0.05"
            material="silver" mass="0.1"
            contype="1" conaffinity="1" friction="0.8 0.8 0.8"/>
      <site name="eu2_grasp" pos="0 0 0.05" size="0.01"/>
    </body>
  </worldbody>
</mujoco>
```

#### 3.3.4 基础拾取逻辑实现
```cpp
// 抓取状态
struct GraspState {
    bool is_grasping = false;
    int grasped_body_id = -1;
    mjtNum grasp_offset[3] = {0, 0, 0};  // 相对于末端的偏移
};
GraspState grasp_state_;

// 检测末端是否接近物体
bool isEndEffectorNear(int body_id, double threshold = 0.05) {
    int ee_body_id = mj_name2id(model_, mjOBJ_BODY, "link_6");
    if (ee_body_id < 0 || body_id < 0) return false;

    mjtNum* ee_pos = data_->xpos + 3 * ee_body_id;
    mjtNum* obj_pos = data_->xpos + 3 * body_id;

    double dist = sqrt(
        pow(ee_pos[0] - obj_pos[0], 2) +
        pow(ee_pos[1] - obj_pos[1], 2) +
        pow(ee_pos[2] - obj_pos[2], 2)
    );
    return dist < threshold;
}

// 简化版抓取 - 通过mocap body实现
void updateGraspSimulation() {
    if (!grasp_state_.is_grasping) return;

    // 获取末端执行器位置和姿态
    int ee_id = mj_name2id(model_, mjOBJ_BODY, "link_6");
    mjtNum* ee_pos = data_->xpos + 3 * ee_id;
    mjtNum* ee_quat = data_->xquat + 4 * ee_id;

    // 更新被抓取物体的位置（跟随末端）
    int obj_id = grasp_state_.grasped_body_id;
    int jnt_adr = model_->body_jntadr[obj_id];

    // 设置物体位置 = 末端位置 + 偏移
    data_->qpos[jnt_adr + 0] = ee_pos[0] + grasp_state_.grasp_offset[0];
    data_->qpos[jnt_adr + 1] = ee_pos[1] + grasp_state_.grasp_offset[1];
    data_->qpos[jnt_adr + 2] = ee_pos[2] + grasp_state_.grasp_offset[2];

    // 设置物体姿态 = 末端姿态
    data_->qpos[jnt_adr + 3] = ee_quat[0];
    data_->qpos[jnt_adr + 4] = ee_quat[1];
    data_->qpos[jnt_adr + 5] = ee_quat[2];
    data_->qpos[jnt_adr + 6] = ee_quat[3];
}

// 键盘G键触发抓取/释放
case GLFW_KEY_G:
    if (!grasp_state_.is_grasping) {
        // 尝试抓取最近的能量单元
        for (const auto& name : {"energy_unit_1", "energy_unit_2"}) {
            int body_id = mj_name2id(model_, mjOBJ_BODY, name);
            if (isEndEffectorNear(body_id)) {
                grasp_state_.is_grasping = true;
                grasp_state_.grasped_body_id = body_id;
                RCLCPP_INFO(get_logger(), "Grasped: %s", name);
                break;
            }
        }
    } else {
        // 释放
        grasp_state_.is_grasping = false;
        grasp_state_.grasped_body_id = -1;
        RCLCPP_INFO(get_logger(), "Released object");
    }
    break;
```

---

## 四、文件修改清单

### 4.1 需要修改的文件

| 文件 | 修改类型 | 主要内容 | 预估代码量 |
|------|----------|----------|-----------|
| `mujoco_interface_node.cpp` | 大幅扩展 | ImGui集成、曲线显示、场景加载 | +400行 |
| `CMakeLists.txt` | 修改 | 添加ImGui/ImPlot依赖 | +20行 |
| `torque_controller_node.cpp` | 小幅修改 | 添加/target_positions发布 | +15行 |

### 4.2 需要新建的文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `arv_v1_model/mjcf/scene_base.xml` | MJCF | 基础场景定义 |
| `arv_v1_model/mjcf/energy_units.xml` | MJCF | 能量单元定义 |
| `arv_v1_model/meshes/energy_unit.stl` | 3D模型 | 能量单元mesh（用户提供） |

---

## 五、技术依赖

### 5.1 系统包依赖
```bash
# ImGui核心 (apt有,版本1.90.1)
sudo apt install libimgui-dev

# ImPlot (apt没有,需要源码)
# 下载到项目third_party目录
mkdir -p ~/ros2_ws/src/arv_v1_moveit/third_party
cd ~/ros2_ws/src/arv_v1_moveit/third_party
git clone --depth 1 --branch v0.16 https://github.com/epezent/implot.git
```

### 5.2 项目目录结构
```
arv_v1_moveit/
├── third_party/
│   └── implot/           [新增]
│       ├── implot.h
│       ├── implot.cpp
│       ├── implot_items.cpp
│       └── implot_internal.h
└── src/
    └── interfaces/
        └── mujoco_interface_node.cpp
```

### 5.3 CMake配置
```cmake
# ===== 在 CMakeLists.txt 中添加 =====

# 查找系统ImGui
find_package(PkgConfig REQUIRED)
pkg_check_modules(IMGUI REQUIRED imgui)

# ImPlot源码编译 (apt没有implot包)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot)
add_library(implot STATIC
  ${IMPLOT_DIR}/implot.cpp
  ${IMPLOT_DIR}/implot_items.cpp
)
target_include_directories(implot PUBLIC
  ${IMPLOT_DIR}
  ${IMGUI_INCLUDE_DIRS}
)
target_link_libraries(implot PUBLIC ${IMGUI_LIBRARIES})

# mujoco_interface_node链接
target_include_directories(mujoco_interface_node PRIVATE
  ${IMGUI_INCLUDE_DIRS}
)
target_link_libraries(mujoco_interface_node
  ${MUJOCO_LIBRARIES}
  glfw
  OpenGL::GL
  ${IMGUI_LIBRARIES}
  implot
)
```

### 5.4 ImGui GLFW/OpenGL后端
```bash
# ImGui后端文件需要手动复制(apt包不含后端实现)
# 从ImGui源码获取:
cd ~/ros2_ws/src/arv_v1_moveit/third_party
git clone --depth 1 --branch v1.90.1 https://github.com/ocornut/imgui.git imgui_backends

# 需要的文件:
# - imgui_backends/backends/imgui_impl_glfw.cpp
# - imgui_backends/backends/imgui_impl_glfw.h
# - imgui_backends/backends/imgui_impl_opengl3.cpp
# - imgui_backends/backends/imgui_impl_opengl3.h
# - imgui_backends/backends/imgui_impl_opengl3_loader.h
```

### 5.5 完整依赖安装脚本
```bash
#!/bin/bash
# install_imgui_deps.sh

set -e

echo "=== 安装ImGui系统包 ==="
sudo apt update
sudo apt install -y libimgui-dev libglfw3-dev

echo "=== 下载ImPlot源码 ==="
THIRD_PARTY_DIR=~/ros2_ws/src/arv_v1_moveit/third_party
mkdir -p $THIRD_PARTY_DIR
cd $THIRD_PARTY_DIR

if [ ! -d "implot" ]; then
    git clone --depth 1 --branch v0.16 https://github.com/epezent/implot.git
fi

echo "=== 下载ImGui后端文件 ==="
if [ ! -d "imgui_backends" ]; then
    git clone --depth 1 --branch v1.90.1 https://github.com/ocornut/imgui.git imgui_backends
fi

echo "=== 完成 ==="
echo "ImPlot: $THIRD_PARTY_DIR/implot"
echo "ImGui后端: $THIRD_PARTY_DIR/imgui_backends/backends/"
```

---

## 六、开发顺序建议

```
Week 1: ImGui基础集成
├── Day 1-2: 安装依赖，CMake配置
├── Day 3-4: ImGui初始化，基础窗口布局
└── Day 5: 与MuJoCo渲染整合，解决OpenGL上下文问题

Week 2: 曲线显示功能
├── Day 1-2: RingBuffer数据结构，力矩数据采集
├── Day 3: ImPlot集成，力矩曲线显示
├── Day 4: 新增/target_positions Topic
└── Day 5: 跟踪误差曲线显示

Week 3: 场景与道具 (可选)
├── Day 1-2: MJCF场景文件创建
├── Day 3: 场景加载逻辑
├── Day 4-5: 基础拾取功能
```

---

## 七、验收标准

### P0功能验收
- [ ] ImGui面板在MuJoCo窗口右侧正常显示
- [ ] 6个关节的力矩曲线实时更新，无明显延迟
- [ ] 6个关节的跟踪误差曲线实时更新
- [ ] 曲线可以正常缩放、平移
- [ ] 面板可以折叠/展开
- [ ] 不影响现有的3D渲染和鼠标交互

### P2功能验收
- [ ] 地面正确显示，有网格纹理
- [ ] 能量单元正确显示在场景中
- [ ] 按G键可以抓取/释放能量单元
- [ ] 抓取后物体跟随末端执行器移动

---

## 八、风险与注意事项

1. **OpenGL上下文**: ImGui和MuJoCo共享OpenGL上下文，需要注意渲染顺序
2. **线程安全**: 数据更新在ROS2回调线程，渲染在独立线程，需要mutex保护
3. **性能**: ImPlot在1000点数据下性能良好，避免超过此数量
4. **依赖版本**: 确保ImGui/ImPlot版本兼容，推荐ImGui 1.89+, ImPlot 0.14+

---

## 附录A: 快捷键规划

| 按键 | 现有功能 | 新增功能 |
|------|----------|----------|
| Space | 暂停/恢复 | - |
| H | 隐藏UI文本 | 隐藏ImGui面板 |
| R | 重置相机 | - |
| C | 显示接触点 | - |
| F | 显示接触力 | - |
| G | - | 抓取/释放物体 [新增] |
| P | - | 切换曲线面板显示 [新增] |
| ESC | 退出 | - |

---

**文档版本**: 1.0
**最后更新**: 2026-02-02
