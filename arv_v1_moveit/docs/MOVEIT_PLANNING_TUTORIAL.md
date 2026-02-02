# MoveIt 规划系统完整教程

> 从点到点规划到视觉伺服的学习路径

## 目录
1. [基础概念](#1-基础概念)
2. [规划器种类与切换](#2-规划器种类与切换)
3. [代码示例：动态切换规划器](#3-代码示例动态切换规划器)
4. [Pilz 工业规划器配置](#4-pilz-工业规划器配置)
5. [视觉伺服方案对比](#5-视觉伺服方案对比)
6. [MoveIt Servo 详解](#6-moveit-servo-详解)
7. [实战：集成视觉伺服](#7-实战集成视觉伺服)

---

## 1. 基础概念

### 1.1 你现在的理解（点到点规划）

```
目标点 → MoveIt 规划 → 完整轨迹 → 执行 → 完成
         (一次性)      (所有点)   (跟踪)
```

这是 **离线规划** 模式：
- 规划和执行是分开的
- 规划完成后才开始执行
- 执行过程中不会修改轨迹

### 1.2 MoveIt 的层次结构

```
┌─────────────────────────────────────────────────────────────┐
│                    MoveIt 软件栈                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────────────────────────────────────────┐   │
│  │            用户接口层 (User Interface)               │   │
│  │  - RViz MotionPlanning 插件                         │   │
│  │  - MoveGroupInterface (C++)                         │   │
│  │  - moveit_commander (Python)                        │   │
│  └─────────────────────────────────────────────────────┘   │
│                          ↓                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │           move_group 节点 (核心)                      │   │
│  │  - 管理机器人模型                                    │   │
│  │  - 调用规划管道                                      │   │
│  │  - 执行轨迹                                          │   │
│  └─────────────────────────────────────────────────────┘   │
│                          ↓                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │          规划管道 (Planning Pipeline)                │   │
│  │                                                      │   │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐            │   │
│  │  │  OMPL   │ │  Pilz   │ │  CHOMP  │  ...         │   │
│  │  │ (采样)  │ │ (工业)  │ │ (优化)  │              │   │
│  │  └──────────┘ └──────────┘ └──────────┘            │   │
│  └─────────────────────────────────────────────────────┘   │
│                          ↓                                  │
│  ┌─────────────────────────────────────────────────────┐   │
│  │         轨迹执行 (Trajectory Execution)              │   │
│  │  - FollowJointTrajectory Action                     │   │
│  │  - 发送到你的 torque_controller_node                │   │
│  └─────────────────────────────────────────────────────┘   │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### 1.3 关键术语

| 术语 | 解释 |
|------|------|
| **Planning Group** | 规划组，例如你的 "ARM"，定义哪些关节一起运动 |
| **Planning Pipeline** | 规划管道，包含规划器和前/后处理器 |
| **Planner** | 具体的规划算法，如 RRTConnect |
| **Motion Request** | 运动请求，包含目标和约束 |
| **Trajectory** | 轨迹，包含时间戳的关节位置序列 |

---

## 2. 规划器种类与切换

### 2.1 OMPL 规划器家族（你现在用的）

OMPL (Open Motion Planning Library) 是采样类规划器：

```
OMPL 规划器
├── 基于树的 (Tree-based)
│   ├── RRT      - 快速探索随机树
│   ├── RRTConnect - 双向 RRT（默认，最常用）
│   ├── RRT*     - 渐进最优 RRT
│   ├── TRRT     - 考虑代价的 RRT
│   └── ...
│
├── 基于图的 (Graph-based)
│   ├── PRM      - 概率路线图
│   ├── PRM*     - 渐进最优 PRM
│   └── LazyPRM  - 延迟碰撞检测
│
└── 其他
    ├── BKPIECE  - 适合狭窄通道
    ├── KPIECE   - 投影空间探索
    ├── EST      - 扩展空间树
    └── ...
```

**OMPL 特点**：
- ✅ 能处理复杂障碍物环境
- ✅ 概率完备（如果解存在，最终能找到）
- ❌ 规划时间不确定（50ms ~ 数秒）
- ❌ 路径质量不稳定，需要后处理平滑

### 2.2 Pilz 工业规划器

Pilz 是确定性规划器，专为工业应用设计：

```
Pilz 规划器
├── PTP  - Point-to-Point（关节空间插值）
├── LIN  - Linear（笛卡尔直线运动）
└── CIRC - Circular（圆弧运动）
```

**Pilz 特点**：
- ✅ 规划时间确定（通常 < 10ms）
- ✅ 路径完全可预测
- ✅ 适合视觉伺服的快速重规划
- ❌ 不能绕过障碍物
- ❌ 只支持简单运动

### 2.3 切换规划器的方法

#### 方法 A：RViz 界面（最简单）

1. 打开 RViz 中的 MotionPlanning 面板
2. 找到 "Planning" 标签页
3. 在 "Planner" 下拉菜单中选择不同规划器

#### 方法 B：C++ 代码中切换

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

// 创建 MoveGroupInterface
auto move_group = moveit::planning_interface::MoveGroupInterface(node, "ARM");

// 切换到 OMPL 的 RRT*
move_group.setPlanningPipelineId("ompl");
move_group.setPlannerId("RRTstar");

// 切换到 Pilz 的直线运动
move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
move_group.setPlannerId("LIN");

// 切换回默认
move_group.setPlanningPipelineId("ompl");
move_group.setPlannerId("RRTConnect");
```

#### 方法 C：Python 代码中切换

```python
from moveit_commander import MoveGroupCommander

move_group = MoveGroupCommander("ARM")

# 切换规划器
move_group.set_planning_pipeline_id("ompl")
move_group.set_planner_id("RRTConnect")

# 或切换到 Pilz
move_group.set_planning_pipeline_id("pilz_industrial_motion_planner")
move_group.set_planner_id("PTP")
```

---

## 3. 代码示例：动态切换规划器

### 3.1 完整 C++ 示例

以下是一个完整的示例节点，演示如何：
1. 连接 MoveIt
2. 动态切换规划器
3. 规划并执行运动

```cpp
// 文件: moveit_planner_demo.cpp
// 编译需要添加 moveit_ros_planning_interface 依赖

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PlannerDemoNode : public rclcpp::Node
{
public:
    PlannerDemoNode() : Node("planner_demo_node")
    {
        // 注意：MoveGroupInterface 需要在 spin 之后创建
        // 所以我们用定时器延迟初始化
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&PlannerDemoNode::initialize, this));
    }

private:
    void initialize()
    {
        init_timer_->cancel();  // 只执行一次

        // 创建 MoveGroupInterface（连接到 move_group 节点）
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "ARM");

        RCLCPP_INFO(get_logger(), "Connected to MoveIt!");
        RCLCPP_INFO(get_logger(), "Planning frame: %s",
                    move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "End effector: %s",
                    move_group_->getEndEffectorLink().c_str());

        // 演示不同规划器
        demoOmplPlanners();
        demoPilzPlanners();
    }

    void demoOmplPlanners()
    {
        RCLCPP_INFO(get_logger(), "\n===== OMPL 规划器演示 =====");

        // 设置目标（关节空间）
        std::vector<double> target_joints = {0.5, 2.5, 1.0, 0.0, 0.0, 0.0};
        move_group_->setJointValueTarget(target_joints);

        // 测试不同的 OMPL 规划器
        std::vector<std::string> ompl_planners = {
            "RRTConnect",  // 默认，双向 RRT
            "RRTstar",     // 渐进最优
            "PRM",         // 概率路线图
            "BKPIECE"      // 适合狭窄通道
        };

        for (const auto& planner : ompl_planners)
        {
            // 切换规划器
            move_group_->setPlanningPipelineId("ompl");
            move_group_->setPlannerId(planner);

            // 规划（不执行）
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto start = std::chrono::high_resolution_clock::now();
            bool success = (move_group_->plan(plan) ==
                           moveit::core::MoveItErrorCode::SUCCESS);
            auto end = std::chrono::high_resolution_clock::now();

            double planning_time = std::chrono::duration<double, std::milli>(
                end - start).count();

            RCLCPP_INFO(get_logger(),
                       "[%s] 规划%s, 耗时: %.1f ms, 轨迹点数: %zu",
                       planner.c_str(),
                       success ? "成功" : "失败",
                       planning_time,
                       plan.trajectory_.joint_trajectory.points.size());
        }
    }

    void demoPilzPlanners()
    {
        RCLCPP_INFO(get_logger(), "\n===== Pilz 规划器演示 =====");

        // Pilz 需要先配置（见下一节）
        // 这里仅演示 API 调用方式

        // PTP - 点到点（关节空间）
        move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
        move_group_->setPlannerId("PTP");

        std::vector<double> target = {0.0, 2.342, 0.989, 0.0, 0.0, 0.0};
        move_group_->setJointValueTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) ==
                       moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(get_logger(), "[Pilz PTP] 规划%s",
                   success ? "成功" : "失败（可能未配置 Pilz）");

        // LIN - 笛卡尔直线（需要设置 Pose 目标）
        move_group_->setPlannerId("LIN");

        // 设置笛卡尔目标
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.3;
        target_pose.position.y = 0.0;
        target_pose.position.z = 0.4;
        target_pose.orientation.w = 1.0;

        move_group_->setPoseTarget(target_pose);

        success = (move_group_->plan(plan) ==
                  moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(get_logger(), "[Pilz LIN] 规划%s",
                   success ? "成功" : "失败");
    }

    rclcpp::TimerBase::SharedPtr init_timer_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlannerDemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### 3.2 添加到 CMakeLists.txt（如果要编译上述示例）

```cmake
# 需要添加 MoveIt 依赖
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(moveit_planner_demo
  src/moveit_planner_demo.cpp
)

ament_target_dependencies(moveit_planner_demo
  rclcpp
  moveit_ros_planning_interface
)

install(TARGETS moveit_planner_demo
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## 4. Pilz 工业规划器配置

### 4.1 为什么要用 Pilz？

对于视觉伺服，Pilz 比 OMPL 更合适：

| 特性 | OMPL (RRTConnect) | Pilz (LIN/PTP) |
|------|-------------------|----------------|
| 规划时间 | 50-500ms（不确定） | < 10ms（确定） |
| 路径形状 | 随机 | 直线/关节插值 |
| 适合重规划 | ❌ 太慢 | ✅ 足够快 |
| 避障能力 | ✅ 强 | ❌ 无 |

### 4.2 配置 Pilz 规划器

创建配置文件 `config/pilz_planning.yaml`:

```yaml
# pilz_planning.yaml - Pilz 工业规划器配置

planning_pipelines:
  pipeline_names:
    - ompl
    - pilz_industrial_motion_planner

# Pilz 规划器配置
pilz_industrial_motion_planner:
  planning_plugins:
    - pilz_industrial_motion_planner/CommandPlanner
  request_adapters:
    - default_planning_request_adapters/ResolveConstraintFrames
    - default_planning_request_adapters/ValidateWorkspaceBounds
    - default_planning_request_adapters/CheckStartStateBounds
    - default_planning_request_adapters/CheckStartStateCollision
  response_adapters:
    - default_planning_response_adapters/ValidateSolution

# 默认规划器
default_planning_pipeline: ompl
```

### 4.3 笛卡尔限制配置

你已经有 `pilz_cartesian_limits.yaml`，确保它包含：

```yaml
# 笛卡尔空间运动限制
cartesian_limits:
  max_trans_vel: 1.0      # 最大平移速度 (m/s)
  max_trans_acc: 2.25     # 最大平移加速度 (m/s²)
  max_trans_dec: -5.0     # 最大平移减速度 (m/s²)
  max_rot_vel: 1.57       # 最大旋转速度 (rad/s)
```

### 4.4 在 Launch 文件中启用

修改 `mujoco_demo.launch.py`，确保加载 Pilz 配置：

```python
# 在 launch_setup 函数中
moveit_config = (
    MoveItConfigsBuilder("ARV_V1_MODEL", package_name="ARV_V1_MOVEIT")
    .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
    .to_moveit_configs()
)
```

---

## 5. 视觉伺服方案对比

### 5.1 你的需求

```
视觉系统检测目标（10-30 Hz）
        ↓
计算目标位姿误差
        ↓
控制机械臂跟踪（需要持续调整）
```

### 5.2 四种实现方案

```
┌─────────────────────────────────────────────────────────────────┐
│                    方案对比                                      │
├──────────────┬──────────┬──────────┬──────────┬────────────────┤
│    方案       │ 更新频率  │ 实现难度 │ 灵活性   │ 适用场景        │
├──────────────┼──────────┼──────────┼──────────┼────────────────┤
│ OMPL 重规划   │ ~2 Hz    │ ⭐       │ ⭐⭐⭐⭐  │ ❌ 太慢         │
├──────────────┼──────────┼──────────┼──────────┼────────────────┤
│ Pilz 重规划   │ ~10 Hz   │ ⭐⭐     │ ⭐⭐⭐    │ ✅ 简单伺服     │
├──────────────┼──────────┼──────────┼──────────┼────────────────┤
│ MoveIt Servo │ 100+ Hz  │ ⭐⭐⭐   │ ⭐⭐⭐⭐  │ ✅✅ 推荐       │
├──────────────┼──────────┼──────────┼──────────┼────────────────┤
│ 直接 IK 控制  │ 200 Hz   │ ⭐⭐⭐⭐ │ ⭐⭐⭐⭐⭐│ ✅ 最灵活      │
└──────────────┴──────────┴──────────┴──────────┴────────────────┘
```

### 5.3 方案详解

#### 方案 1：Pilz 重规划（简单场景可用）

```cpp
// 每次视觉更新时重新规划
void visionCallback(const geometry_msgs::msg::Pose& target)
{
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setPoseTarget(target);

    // Pilz 规划快（< 10ms），可以约 10Hz 重规划
    move_group_->asyncMove();
}
```

**局限**：
- 每次规划都从当前位置开始
- 可能有轨迹不连续问题
- 10Hz 对快速跟踪可能不够

#### 方案 2：MoveIt Servo（推荐）

MoveIt Servo 专门为实时伺服设计：

```
视觉系统 → 计算速度命令 → MoveIt Servo → 关节命令 → 控制器
          (Twist/JointJog)   (100Hz IK)    (增量更新)
```

**优点**：
- 100Hz+ 更新率
- 内置奇异点避免
- 内置碰撞检测
- 平滑的速度控制

#### 方案 3：直接 IK 控制（最灵活）

绕过 MoveIt，直接使用 KDL 求解 IK：

```cpp
// 视觉回调
void visionCallback(const geometry_msgs::msg::Pose& target)
{
    // 直接求解 IK
    KDL::Frame target_frame = poseToKdlFrame(target);
    KDL::JntArray q_out(6);

    int result = ik_solver_->CartToJnt(current_joints_, target_frame, q_out);

    if (result >= 0) {
        // 直接发送关节目标到你的 torque_controller
        publishJointTarget(q_out);
    }
}
```

**你已经有 KDL 集成**，这个方案实现起来很直接。

---

## 6. MoveIt Servo 详解

### 6.1 Servo 是什么？

MoveIt Servo 是一个实时控制模块：

```
┌────────────────────────────────────────────────────────────────┐
│                    MoveIt Servo 架构                            │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  输入命令（二选一）:                                             │
│  ┌─────────────────┐    ┌─────────────────┐                   │
│  │  TwistStamped   │    │   JointJog      │                   │
│  │  (笛卡尔速度)    │    │   (关节速度)     │                   │
│  └────────┬────────┘    └────────┬────────┘                   │
│           │                      │                             │
│           └──────────┬───────────┘                             │
│                      ↓                                         │
│  ┌─────────────────────────────────────────────────────────┐  │
│  │                 Servo 核心                               │  │
│  │  - 雅可比矩阵计算（每个周期）                              │  │
│  │  - 奇异点检测与避免                                       │  │
│  │  - 碰撞检测（可选）                                       │  │
│  │  - 关节限位检查                                          │  │
│  │  - 速度/加速度平滑                                        │  │
│  └─────────────────────────────────────────────────────────┘  │
│                      ↓                                         │
│  输出（三种模式）:                                              │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐          │
│  │ JointState   │ │  轨迹控制器   │ │   直接位置    │          │
│  │  (位置增量)   │ │   (Action)   │ │   (话题)     │          │
│  └──────────────┘ └──────────────┘ └──────────────┘          │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

### 6.2 Servo 配置文件

创建 `config/servo_config.yaml`:

```yaml
###############################################
# MoveIt Servo 配置
###############################################

# 基本设置
use_gazebo: false                    # 不使用 Gazebo
publish_period: 0.01                 # 发布周期 100Hz
planning_frame: "base_link"          # 规划参考系
move_group_name: "ARM"               # 规划组名称
ee_frame_name: "link6_2006roll"      # 末端执行器坐标系

# 输入话题
cartesian_command_in_topic: ~/delta_twist_cmds    # 笛卡尔速度输入
joint_command_in_topic: ~/delta_joint_cmds        # 关节速度输入
robot_link_command_frame: "link6_2006roll"        # 命令参考系

# 输出设置
command_out_topic: /ARM_controller/joint_trajectory  # 输出话题
command_out_type: trajectory_msgs/JointTrajectory    # 输出类型
publish_joint_positions: true
publish_joint_velocities: true
publish_joint_accelerations: false

# 安全限制
incoming_command_timeout: 0.1        # 命令超时（秒）
scale:
  linear: 0.4                        # 线速度缩放
  rotational: 0.8                    # 角速度缩放
  joint: 0.5                         # 关节速度缩放

# 关节限位
joint_limit_margins: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 关节限位边界

# 奇异点处理
lower_singularity_threshold: 17.0    # 奇异点下阈值
hard_stop_singularity_threshold: 30.0 # 奇异点硬停止阈值
leaving_singularity_threshold_multiplier: 2.0

# 碰撞检测
check_collisions: true               # 启用碰撞检测
collision_check_rate: 10.0           # 碰撞检测频率 (Hz)
self_collision_proximity_threshold: 0.01  # 自碰撞阈值
scene_collision_proximity_threshold: 0.02 # 场景碰撞阈值

# 状态
status_topic: ~/status               # 状态话题
```

### 6.3 Servo Launch 文件

创建 `launch/servo_demo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('ARV_V1_MOVEIT')

    # 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "ARV_V1_MODEL",
        package_name="ARV_V1_MOVEIT"
    ).to_moveit_configs()

    # Servo 配置
    servo_config = os.path.join(pkg_dir, 'config', 'servo_config.yaml')

    # Servo 节点
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([
        servo_node,
    ])
```

### 6.4 使用 Servo 的代码示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

class VisualServoNode : public rclcpp::Node
{
public:
    VisualServoNode() : Node("visual_servo_node")
    {
        // 笛卡尔速度命令发布者
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);

        // 关节速度命令发布者（可选）
        joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
            "/servo_node/delta_joint_cmds", 10);

        // 启动 Servo
        start_servo_client_ = create_client<std_srvs::srv::Trigger>(
            "/servo_node/start_servo");

        // 等待服务可用
        while (!start_servo_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(get_logger(), "Waiting for servo service...");
        }

        // 启动 Servo
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        start_servo_client_->async_send_request(request);

        RCLCPP_INFO(get_logger(), "Servo started!");

        // 模拟视觉伺服定时器 (30 Hz)
        timer_ = create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&VisualServoNode::visionCallback, this));
    }

private:
    void visionCallback()
    {
        // 模拟视觉检测结果 → 计算笛卡尔速度命令
        // 实际应用中，这里会从视觉话题获取目标位姿

        geometry_msgs::msg::TwistStamped twist;
        twist.header.stamp = now();
        twist.header.frame_id = "link6_2006roll";  // 末端执行器坐标系

        // 示例：沿 X 轴移动
        twist.twist.linear.x = 0.05;  // 5 cm/s
        twist.twist.linear.y = 0.0;
        twist.twist.linear.z = 0.0;
        twist.twist.angular.x = 0.0;
        twist.twist.angular.y = 0.0;
        twist.twist.angular.z = 0.0;

        twist_pub_->publish(twist);
    }

    // 或者使用关节速度控制
    void sendJointJog()
    {
        control_msgs::msg::JointJog jog;
        jog.header.stamp = now();
        jog.joint_names = {"joint_1", "joint_2", "joint_3",
                          "joint_4", "joint_5", "joint_6"};
        jog.velocities = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};  // 关节1 以 0.1 rad/s 转动

        joint_pub_->publish(jog);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 7. 实战：集成视觉伺服

### 7.1 推荐架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    视觉伺服系统架构                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐                                               │
│  │  相机节点    │  发布: /camera/image, /camera/depth          │
│  └──────┬──────┘                                               │
│         ↓                                                       │
│  ┌─────────────┐                                               │
│  │  视觉检测    │  发布: /target_pose (目标位姿)                │
│  │  (10-30 Hz) │        /visual_error (视觉误差)               │
│  └──────┬──────┘                                               │
│         ↓                                                       │
│  ┌─────────────────────────────────────────────────┐           │
│  │           视觉伺服控制器                          │           │
│  │  - 订阅 /target_pose 或 /visual_error           │           │
│  │  - 计算速度命令（PBVS 或 IBVS）                  │           │
│  │  - 发布到 Servo 或直接发送 IK 结果               │           │
│  └──────┬──────────────────────────────────────────┘           │
│         ↓                                                       │
│  ┌─────────────┐    或    ┌─────────────┐                      │
│  │ MoveIt Servo│          │ 直接 IK     │                      │
│  │  (100 Hz)   │          │ + 你的控制器 │                      │
│  └──────┬──────┘          └──────┬──────┘                      │
│         ↓                        ↓                              │
│  ┌─────────────────────────────────────────────────┐           │
│  │         torque_controller_node (200 Hz)         │           │
│  └──────┬──────────────────────────────────────────┘           │
│         ↓                                                       │
│  ┌─────────────┐                                               │
│  │ MuJoCo/硬件  │                                               │
│  └─────────────┘                                               │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 7.2 两种视觉伺服算法

#### PBVS (Position-Based Visual Servoing)

```
视觉 → 估计目标 3D 位姿 → 计算位姿误差 → 笛卡尔速度 → IK → 关节命令
```

```cpp
// PBVS 速度计算
geometry_msgs::msg::Twist computePBVSVelocity(
    const geometry_msgs::msg::Pose& current,
    const geometry_msgs::msg::Pose& target,
    double lambda = 0.5)  // 控制增益
{
    geometry_msgs::msg::Twist vel;

    // 位置误差 → 线速度
    vel.linear.x = lambda * (target.position.x - current.position.x);
    vel.linear.y = lambda * (target.position.y - current.position.y);
    vel.linear.z = lambda * (target.position.z - current.position.z);

    // 姿态误差 → 角速度（简化版，实际需要用四元数）
    // ... 省略姿态误差计算

    return vel;
}
```

#### IBVS (Image-Based Visual Servoing)

```
视觉 → 图像特征点 → 计算图像误差 → 图像雅可比 → 笛卡尔速度 → ...
```

更复杂，但不需要精确的相机标定。

### 7.3 快速开始步骤

1. **最简单方案：Pilz 重规划**
   - 不需要额外配置
   - 适合 ~5-10 Hz 更新

2. **推荐方案：MoveIt Servo**
   - 需要添加 servo 配置
   - 适合 30-100 Hz 更新

3. **最灵活方案：直接 IK**
   - 利用你现有的 KDL 代码
   - 可以 200 Hz 更新

### 7.4 依赖安装

```bash
# 安装 MoveIt Servo（如果没有）
sudo apt install ros-jazzy-moveit-servo

# 安装 ViSP（视觉伺服库，可选）
sudo apt install ros-jazzy-visp
```

---

## 总结

| 你的问题 | 答案 |
|----------|------|
| 规划器能直接切换吗？ | ✅ 能，代码中调用 `setPlannerId()` 即可 |
| 需要重新运行 Setup Assistant 吗？ | ❌ 不需要，那只是修改 SRDF/规划组用的 |
| MoveIt 能 10Hz 持续规划吗？ | ⚠️ OMPL 不行，Pilz 勉强，Servo 可以 |
| 视觉伺服用什么方案？ | 🎯 推荐 MoveIt Servo 或直接 IK |

**推荐学习路径**：
1. 先用 RViz 界面切换不同 OMPL 规划器，感受差异
2. 配置并测试 Pilz 规划器
3. 集成 MoveIt Servo
4. 最终实现完整视觉伺服系统

---

*最后更新: 2026-01-19*
