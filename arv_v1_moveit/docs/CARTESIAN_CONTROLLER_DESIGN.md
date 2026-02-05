# 笛卡尔坐标控制节点设计文档

> **版本**: 1.0
> **日期**: 2026-02-05
> **状态**: 设计阶段

---

## 1. 概述

### 1.1 背景

当前 ARV_V1 机械臂系统通过预录制的关节空间轨迹进行运动控制。为支持视觉伺服和手动指定末端位姿的应用场景，需要新增笛卡尔坐标输入控制模式。

### 1.2 目标

- 支持 **geometry_msgs/Pose** 格式的笛卡尔位姿输入
- 提供 **RPY (Roll-Pitch-Yaw)** 姿态输入接口，便于人工操作
- 支持 **10-30Hz** 视觉伺服连续更新
- 使用 **MoveIt2 Pilz LIN** 规划器生成确定性路径

### 1.3 设计约束

| 约束项 | 描述 |
|--------|------|
| 运动方式 | 强制使用 MoveIt 重规划（不使用直接 IK 插值） |
| 参考坐标系 | base_link（机械臂基座） |
| 姿态表示 | ZYX 欧拉角（Yaw-Pitch-Roll），内部转四元数 |
| 规划器 | Pilz Industrial Motion Planner (LIN 模式) |

---

## 2. 系统架构

### 2.1 节点拓扑

```
┌─────────────────────────────────────────────────────────────────┐
│                       Application Layer                          │
│  ┌──────────────────┐        ┌──────────────────────────────┐  │
│  │ mission_executor │        │  cartesian_controller_node   │  │
│  │    (TUI 界面)    │        │      (新增节点)              │  │
│  └────────┬─────────┘        └──────────────┬───────────────┘  │
│           │                                  │                   │
│           │ /load_trajectory                 │ MoveIt Planning   │
│           ↓                                  ↓                   │
│  ┌──────────────────┐        ┌──────────────────────────────┐  │
│  │trajectory_manager│        │        move_group            │  │
│  │     (服务端)     │        │   (Pilz LIN Planner)         │  │
│  └────────┬─────────┘        └──────────────┬───────────────┘  │
└───────────│──────────────────────────────────│───────────────────┘
            │                                  │
            └──────────────┬───────────────────┘
                           ↓
            ┌──────────────────────────────────┐
            │    /ARM_controller/              │
            │    follow_joint_trajectory       │
            │         (Action)                 │
            └──────────────┬───────────────────┘
                           ↓
┌──────────────────────────────────────────────────────────────────┐
│                       Control Layer                               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │              torque_controller_node                       │   │
│  │  • 轨迹插值 (200Hz)                                       │   │
│  │  • 动力学前馈 (KDL)                                       │   │
│  │  • 级联 P+PI 反馈                                         │   │
│  └──────────────────────────────────────────────────────────┘   │
└──────────────────────────────────────────────────────────────────┘
```

### 2.2 数据流

```
视觉系统 (10-30Hz)                    用户手动调用
/cartesian_target_pose               /move_to_cartesian_rpy
(geometry_msgs/PoseStamped)          (Service Request)
         │                                  │
         │  订阅                             │  服务回调
         └──────────────┬──────────────────┘
                        ↓
            ┌─────────────────────────────┐
            │   cartesian_controller_node │
            │                             │
            │  1. 输入验证                │
            │     • 工作空间边界检查      │
            │     • 四元数归一化检查      │
            │                             │
            │  2. 坐标转换                │
            │     • RPY → Quaternion      │
            │     • 使用 tf2 库           │
            │                             │
            │  3. 运动规划                │
            │     • MoveGroupInterface    │
            │     • Pilz LIN 规划器       │
            │     • 规划时间 ~50-200ms    │
            │                             │
            │  4. 轨迹执行                │
            │     • asyncExecute() 异步   │
            │     • 支持运动抢占          │
            └──────────────┬──────────────┘
                           ↓
              /ARM_controller/follow_joint_trajectory
              (control_msgs/FollowJointTrajectory)
                           ↓
            ┌─────────────────────────────┐
            │   torque_controller_node    │
            │   (200Hz 力矩控制循环)      │
            └──────────────┬──────────────┘
                           ↓
              /effort_controller/commands
              (std_msgs/Float64MultiArray)
                           ↓
            ┌─────────────────────────────┐
            │  mujoco_interface_node 或   │
            │  hardware_interface_node    │
            └─────────────────────────────┘
```

---

## 3. 接口设计

### 3.1 服务接口

#### MoveToCartesianRPY.srv

```srv
# 笛卡尔位姿目标（RPY输入格式）
# 文件: arv_v1_interfaces/srv/MoveToCartesianRPY.srv

# ========== 请求 ==========
float64 x                    # 目标位置 X (米, base_link 坐标系)
float64 y                    # 目标位置 Y (米)
float64 z                    # 目标位置 Z (米)
float64 roll                 # 绕 X 轴旋转角度 (弧度)
float64 pitch                # 绕 Y 轴旋转角度 (弧度)
float64 yaw                  # 绕 Z 轴旋转角度 (弧度)
float64 velocity_scaling     # 速度缩放因子 (0.0-1.0), 默认 1.0
float64 acceleration_scaling # 加速度缩放因子 (0.0-1.0), 默认 1.0
bool async                   # true: 规划成功后立即返回, 不等待执行完成
---
# ========== 响应 ==========
bool success                 # 操作是否成功
string message               # 状态消息或错误信息
float64 planning_time        # 规划耗时 (秒)
float64 trajectory_duration  # 轨迹执行时长 (秒)
```

#### StopCartesianMotion.srv

```srv
# 停止当前笛卡尔运动
# 文件: arv_v1_interfaces/srv/StopCartesianMotion.srv

# ========== 请求 ==========
# (无参数)
---
# ========== 响应 ==========
bool success                 # 停止是否成功
string message               # 状态消息
```

### 3.2 话题接口

| 话题名称 | 消息类型 | 方向 | 频率 | 描述 |
|----------|----------|------|------|------|
| `/cartesian_target_pose` | `geometry_msgs/PoseStamped` | 订阅 | 10-30Hz | 视觉伺服目标位姿输入 |
| `/cartesian_controller/current_pose` | `geometry_msgs/PoseStamped` | 发布 | 30Hz | 当前末端执行器位姿 |
| `/cartesian_controller/status` | `std_msgs/String` | 发布 | 事件触发 | 控制器状态 |

### 3.3 状态定义

| 状态 | 描述 |
|------|------|
| `IDLE` | 空闲，等待目标输入 |
| `PLANNING` | 正在规划轨迹 |
| `EXECUTING` | 正在执行轨迹 |
| `ERROR` | 发生错误 |

---

## 4. 核心算法

### 4.1 RPY → 四元数转换

使用 ZYX 欧拉角顺序（先 Yaw，再 Pitch，最后 Roll）：

```cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

geometry_msgs::msg::Quaternion rpyToQuaternion(
    double roll, double pitch, double yaw) {

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);  // tf2 内部使用 ZYX 顺序
    q.normalize();

    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}
```

### 4.2 MoveIt 规划配置

```cpp
void initializeMoveGroup() {
    // 创建 MoveGroupInterface
    move_group_ = std::make_shared<MoveGroupInterface>(
        shared_from_this(), "ARM");

    // 配置 Pilz 规划器
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");  // 直线运动

    // 设置参考坐标系
    move_group_->setPoseReferenceFrame("base_link");
    move_group_->setEndEffectorLink("link6_2006roll");

    // 规划时间限制（Pilz 很快，1秒足够）
    move_group_->setPlanningTime(1.0);
}
```

### 4.3 运动抢占机制

当新目标到达时，如果正在执行旧轨迹，先停止再规划新路径：

```cpp
void poseTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 限速检查 (避免过高频率)
    auto elapsed = this->now() - last_command_time_;
    if (elapsed.seconds() * 1000.0 < min_interval_ms_) {
        return;  // 跳过此次更新
    }
    last_command_time_ = this->now();

    // 如果正在执行，先停止当前运动
    if (state_.load() == State::EXECUTING) {
        move_group_->stop();
        RCLCPP_INFO(get_logger(), "Preempting current motion");
    }

    // 规划并异步执行新目标
    planAndExecute(msg->pose, velocity_scaling_, accel_scaling_,
                   /*async=*/true, ...);
}
```

---

## 5. 参数配置

### 5.1 配置文件

**文件**: `config/cartesian_controller_params.yaml`

```yaml
cartesian_controller_node:
  ros__parameters:
    # ========== 运动学配置 ==========
    planning_group: "ARM"
    end_effector_link: "link6_2006roll"
    reference_frame: "base_link"

    # ========== 运动参数 ==========
    default_velocity_scaling: 0.5      # 默认速度缩放 (保守值)
    default_acceleration_scaling: 0.5  # 默认加速度缩放

    # ========== 视觉伺服配置 ==========
    pose_topic_min_interval_ms: 50.0   # 最小更新间隔 (对应 20Hz 上限)
    enable_continuous_mode: true       # 启用话题连续输入模式

    # ========== 规划参数 ==========
    planning_time_limit: 1.0           # 规划超时 (秒)
    goal_position_tolerance: 0.001     # 位置容差 (1mm)
    goal_orientation_tolerance: 0.01   # 姿态容差 (~0.6°)

    # ========== 工作空间边界 (米) ==========
    workspace_bounds:
      min_x: -0.8
      max_x: 0.8
      min_y: -0.8
      max_y: 0.8
      min_z: -0.2
      max_z: 1.0
```

### 5.2 Pilz 笛卡尔限制

**文件**: `config/pilz_cartesian_limits.yaml` (已存在)

```yaml
cartesian_limits:
  max_trans_vel: 1.0    # 最大平移速度 (m/s)
  max_trans_acc: 2.25   # 最大平移加速度 (m/s²)
  max_trans_dec: -5.0   # 最大平移减速度 (m/s²)
  max_rot_vel: 1.57     # 最大旋转速度 (rad/s)
```

---

## 6. 错误处理

### 6.1 错误类型与处理策略

| 错误类型 | 检测方式 | 处理策略 | 恢复方式 |
|----------|----------|----------|----------|
| IK 求解失败 | MoveIt 返回错误码 | 记录日志，返回失败 | 用户调整目标 |
| 规划超时 | 超过 planning_time_limit | 尝试 PTP 回退 | 自动切换规划器 |
| 工作空间越界 | 边界检查 | 拒绝请求 | 用户调整目标 |
| 关节限位 | MoveIt 碰撞检测 | 拒绝请求 | 用户调整目标 |
| 奇异点 | 规划失败 | 尝试 PTP 回退 | 自动处理 |
| 执行被抢占 | 新目标到达 | 停止旧轨迹，执行新轨迹 | 自动处理 |

### 6.2 回退策略

当 Pilz LIN 规划失败时，自动尝试 PTP（关节空间）规划：

```cpp
bool planWithFallback(const geometry_msgs::msg::Pose& target,
                      MoveGroupInterface::Plan& plan,
                      std::string& error_message) {

    // 首选: Pilz LIN (笛卡尔直线)
    move_group_->setPlannerId("LIN");
    if (move_group_->plan(plan) == MoveItErrorCode::SUCCESS) {
        return true;
    }

    RCLCPP_WARN(get_logger(), "LIN planning failed, trying PTP fallback");

    // 回退: Pilz PTP (关节空间，更鲁棒)
    move_group_->setPlannerId("PTP");
    if (move_group_->plan(plan) == MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "PTP fallback successful");
        return true;
    }

    error_message = "Both LIN and PTP planning failed";
    return false;
}
```

---

## 7. 文件清单

### 7.1 新建文件

| 文件路径 | 描述 | 预估行数 |
|----------|------|----------|
| `arv_v1_interfaces/srv/MoveToCartesianRPY.srv` | RPY 输入服务接口 | 18 |
| `arv_v1_interfaces/srv/StopCartesianMotion.srv` | 停止运动服务接口 | 8 |
| `arv_v1_moveit/src/application/cartesian_controller_node.cpp` | 主节点实现 | ~400 |
| `arv_v1_moveit/config/cartesian_controller_params.yaml` | 参数配置 | 25 |

### 7.2 修改文件

| 文件路径 | 修改内容 |
|----------|----------|
| `arv_v1_interfaces/CMakeLists.txt` | 添加新服务文件，添加 geometry_msgs 依赖 |
| `arv_v1_moveit/CMakeLists.txt` | 添加新节点，添加 MoveIt/tf2 依赖 |

---

## 8. 构建配置

### 8.1 arv_v1_interfaces/CMakeLists.txt 修改

```cmake
cmake_minimum_required(VERSION 3.22)
project(arv_v1_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # 新增

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
  "srv/SaveLastTrajectory.srv"
  "srv/LoadTrajectory.srv"
  "srv/ListTrajectories.srv"
  "srv/MoveToCartesianRPY.srv"     # 新增
  "srv/StopCartesianMotion.srv"    # 新增
  DEPENDENCIES trajectory_msgs geometry_msgs  # 添加 geometry_msgs
)

ament_package()
```

### 8.2 arv_v1_moveit/CMakeLists.txt 修改

在依赖查找部分添加：
```cmake
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
```

添加新节点：
```cmake
# -------------------------------------------
# 笛卡尔控制器节点 (Application Layer)
add_executable(cartesian_controller_node
  src/application/cartesian_controller_node.cpp
)

target_link_libraries(cartesian_controller_node
  ${rclcpp_LIBRARIES}
)

target_include_directories(cartesian_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cartesian_controller_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
  arv_v1_interfaces
  std_msgs
)
```

在安装部分添加：
```cmake
install(TARGETS
  # ... 现有目标 ...
  cartesian_controller_node  # 新增
  DESTINATION lib/${PROJECT_NAME}
)
```

---

## 9. 使用指南

### 9.1 启动节点

```bash
# 1. 启动 MuJoCo 仿真系统 (包含 move_group)
./start_mujoco_system.sh

# 2. 启动笛卡尔控制器
ros2 run arv_v1_moveit cartesian_controller_node \
  --ros-args --params-file \
  $(ros2 pkg prefix arv_v1_moveit)/share/arv_v1_moveit/config/cartesian_controller_params.yaml
```

### 9.2 服务调用示例

```bash
# 移动到位置 (0.3, 0.0, 0.4)，姿态为水平向下 (pitch = π/2)
ros2 service call /move_to_cartesian_rpy \
  arv_v1_interfaces/srv/MoveToCartesianRPY \
  "{x: 0.3, y: 0.0, z: 0.4,
    roll: 0.0, pitch: 1.5708, yaw: 0.0,
    velocity_scaling: 0.5, acceleration_scaling: 0.5,
    async: false}"
```

### 9.3 话题发布示例（模拟视觉伺服）

```bash
# 发布目标位姿 (需要四元数格式)
ros2 topic pub /cartesian_target_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.3, y: 0.0, z: 0.4},
      orientation: {x: 0.0, y: 0.707, z: 0.0, w: 0.707}
    }}" --rate 10
```

### 9.4 监控状态

```bash
# 查看控制器状态
ros2 topic echo /cartesian_controller/status

# 查看当前末端位姿
ros2 topic echo /cartesian_controller/current_pose
```

---

## 10. 测试计划

### 10.1 单元测试

| 测试项 | 描述 | 预期结果 |
|--------|------|----------|
| RPY 转换 | 验证各种角度的四元数转换 | 转换正确，四元数归一化 |
| 工作空间检查 | 测试边界内外的目标 | 边界内通过，边界外拒绝 |
| 参数加载 | 验证参数文件读取 | 所有参数正确加载 |

### 10.2 集成测试

| 测试项 | 描述 | 预期结果 |
|--------|------|----------|
| 服务调用 | 发送服务请求移动到目标 | 机械臂平滑到达目标 |
| 话题连续输入 | 10Hz 发布目标位姿 | 机械臂跟随目标运动 |
| 运动抢占 | 执行中发送新目标 | 平滑切换到新目标 |
| 停止服务 | 执行中调用停止 | 立即停止运动 |

### 10.3 性能测试

| 指标 | 目标 | 测试方法 |
|------|------|----------|
| 规划延迟 | < 200ms | 测量服务响应时间 |
| 更新频率支持 | 20Hz | 连续发布话题测试 |
| CPU 占用 | < 30% (单核) | htop 监控 |

---

## 11. 后续扩展

### 11.1 规划中

- [ ] 增量运动模式（相对于当前位置的偏移）
- [ ] TF2 坐标变换支持（支持任意参考坐标系）
- [ ] 轨迹保存功能（保存笛卡尔路径点）

### 11.2 未来考虑

- 基于图像的视觉伺服 (IBVS) 直接控制
- 力/力矩混合控制
- 碰撞避障实时规划

---

## 附录 A: 参考文档

- [MoveIt2 MoveGroupInterface API](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [Pilz Industrial Motion Planner](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html)
- [tf2 坐标变换库](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)
- [geometry_msgs/Pose 消息定义](https://docs.ros2.org/latest/api/geometry_msgs/msg/Pose.html)

---

## 附录 B: 相关文件索引

| 文件 | 用途 |
|------|------|
| `src/application/trajectory_manager_node.cpp` | 服务实现参考 |
| `src/control/torque_controller_node.cpp` | Action Server 实现参考 |
| `config/kinematics.yaml` | KDL 运动学求解器配置 |
| `config/pilz_cartesian_limits.yaml` | 笛卡尔速度限制 |
| `config/ARV_V1_MODEL.srdf` | 规划组定义 |
