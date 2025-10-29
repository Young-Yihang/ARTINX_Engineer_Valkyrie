# ARV V1 力矩控制系统 - 技术要点

> **项目目标**: 基于 MoveIt 规划，实现自定义动力学解算的力矩控制
> **更新时间**: 2025-10-29
> **当前状态**: MuJoCo 仿真集成完成 ✅ | 控制效果需优化 ⚠️

---

## ✅ 系统架构

### 数据流

```
MoveIt 规划器
    ↓ /ARM_controller/follow_joint_trajectory (Action)
Torque Controller Node (200Hz)
    ├─ 动力学计算: τ_ff = M(q)·q̈ + C(q,q̇) + G(q)
    ├─ PD 反馈: τ_fb = Kp·(q_d - q) + Kd·(q̇_d - q̇)
    └─ 总力矩: τ = τ_ff + τ_fb
    ↓ /effort_controller/commands
MuJoCo Interface Node (200Hz)
    ├─ 接收力矩命令
    ├─ 仿真物理（mj_step）
    ├─ 3D 可视化（GLFW + OpenGL）
    └─ 发布关节状态
    ↓ /joint_states
回到 Torque Controller (闭环反馈)
```

### 核心节点

| 节点 | 功能 | 频率 | 状态 |
|------|------|------|------|
| `torque_controller_node` | 动力学计算 + PD 控制 | 200Hz | ✅ |
| `mujoco_interface_node` | MuJoCo 仿真 + 可视化 | 200Hz | ✅ |
| `move_group` | MoveIt 运动规划 | - | ✅ |
| `joint_state_broadcaster` | ~~关节状态广播~~ | - | ❌ 禁用 |

---

## 🔧 核心实现

### 1. 动力学计算 (dynamics_computer.cpp)

**功能**: 使用 KDL 库计算机器人动力学

```cpp
// 前馈力矩计算
void computeFeedforwardTorque(q, qd, qdd, tau_ff) {
    M(q);              // 惯性矩阵
    C(q, qd);          // 科氏力/离心力
    G(q);              // 重力项
    tau_ff = M*qdd + C + G;
}

// 重力补偿（空闲模式）
void computeGravityTorque(q, tau_g) {
    tau_g = G(q);
}
```

**参数来源**: URDF `<inertial>` 标签 → KDL::Chain → ChainDynParam

### 2. 力矩控制器 (torque_controller_node.cpp)

**关键功能**:
- Action Server: 接收 MoveIt 轨迹
- 轨迹插值: 200Hz 实时插值期望状态
- 空闲重力补偿: `is_executing_ = false` 时发送 G(q)
- 轨迹跟踪: 前馈 + PD 反馈

**控制循环**:
```cpp
void controlLoop() {
    if (!is_executing_) {
        // 空闲模式：发送重力补偿
        tau = computeGravityTorque(q_actual);
        publish(tau);
        return;
    }

    // 轨迹跟踪模式
    interpolate(t_now, q_d, qd_d, qdd_d);
    tau_ff = computeFeedforwardTorque(q_d, qd_d, qdd_d);
    tau_fb = Kp*(q_d - q) + Kd*(qd_d - qd);
    tau_total = tau_ff + tau_fb;
    publish(tau_total);
}
```

**PD 增益** (config/controller_params.yaml):
```yaml
Kp: [300, 400, 350, 150, 100, 80]   # 位置增益
Kd: [30, 40, 35, 15, 10, 8]         # 速度增益
```

### 3. MuJoCo 接口 (mujoco_interface_node.cpp)

**模型加载流程**:
```
URDF → 插入 <mujoco> compiler → mj_loadXML
    → mj_saveLastXML → MJCF
    → 插入 <actuator> 定义 → 重新加载
    → 创建 mjData
```

**执行器配置** (6 个关节):
```xml
<motor name="actuator_X" joint="joint_X"
       gear="1" ctrllimited="true" ctrlrange="-20 20"/>
```

**可视化**:
- GLFW 窗口 (1200x900)
- 独立渲染线程 (60Hz)
- OpenGL 上下文在渲染线程中初始化

**关键**:
- `sim_mutex_` 保护 model_ 和 data_
- 仿真线程 200Hz，渲染线程 60Hz 独立运行

---

## ⚠️ 已解决的关键问题

### 问题1: 控制器不发送任何输出

**根因**: 控制定时器在 `handleAccepted()` 中创建，如果没收到轨迹则永不启动

**解决**: 在构造函数中立即创建定时器
```cpp
// 构造函数中（83行之后）
auto period = std::chrono::duration<double, std::milli>(5.0);
control_timer_ = this->create_wall_timer(period, ...);
```

### 问题2: 关节漂移导致 START_STATE_INVALID

**根因**: 空闲时不发送力矩 → 重力导致漂移 → 超出关节限位

**解决**: 实现空闲重力补偿 (456-489行)
```cpp
if (!is_executing_) {
    tau_gravity = computeGravityTorque(q_actual_);
    publish(tau_gravity);
}
```

### 问题3: RViz 机械臂闪烁

**根因**: 两个节点同时发布 /joint_states
- `joint_state_broadcaster`
- `mujoco_interface_node`

**解决**: 创建 `mujoco_demo.launch.py`，不启动 joint_state_broadcaster

### 问题4: MuJoCo 执行器数量为 0

**根因**: MuJoCo URDF 加载器不支持 `<actuator>` 标签

**解决**: 两步转换
1. URDF → MJCF
2. 修改 MJCF 插入 `<actuator>` → 重新加载

### 问题5: MuJoCo 窗口黑屏

**根因**: OpenGL 上下文在主线程创建，渲染线程无法访问

**解决**: 在渲染线程开始时绑定上下文
```cpp
void renderLoop() {
    glfwMakeContextCurrent(window_);     // 在渲染线程中
    mjr_makeContext(model_, &con_, ...);  // 初始化 OpenGL

    while (...) {
        mjv_updateScene(...);
        mjr_render(...);
    }
}
```

### 问题6: 变量作用域错误

**问题**: 使用未定义的局部变量 `q_actual` 而非成员变量 `q_actual_`

**位置**: torque_controller_node.cpp:508

**解决**: 使用 `q_actual_` 并加锁
```cpp
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    computeGravityTorque(q_actual_, tau_gravity);
}
```

---

## 🔥 当前问题

### 主要问题: 控制效果差

**现象**: 机械臂运动不稳定、抖动、跟踪误差大

**可能原因**:
1. **PD 增益未调优** - 当前增益可能不匹配实际系统
2. **动力学模型误差** - URDF 惯性参数可能不准确
3. **传感器噪声** - /joint_states 数据可能有噪声
4. **控制频率** - 200Hz 可能不够高
5. **MuJoCo 仿真参数** - timestep, solver 参数需优化

**调试方向**:
1. 查看力矩命令波形（是否剧烈振荡）
2. 查看位置跟踪误差大小
3. 单关节测试（隔离问题）
4. 尝试调整 PD 增益
5. 检查 MuJoCo 仿真 timestep (当前 0.005s)

---

## 🛠️ 调试工具

### 查看系统状态

```bash
# 查看话题
ros2 topic list

# 监控力矩命令
ros2 topic echo /effort_controller/commands

# 监控关节状态
ros2 topic echo /joint_states

# 查看控制器
ros2 control list_controllers

# 查看 Action Server
ros2 action list
```

### 可视化数据

```bash
# 使用 PlotJuggler 绘制实时数据
ros2 run plotjuggler plotjuggler

# 建议绘制的信号：
# - /joint_states/position[0-5]
# - /effort_controller/commands/data[0-5]
# - 跟踪误差 = desired - actual
```

### 调参建议

**保守增益** (稳定但响应慢):
```yaml
Kp: [50, 50, 50, 30, 20, 10]
Kd: [5, 5, 5, 3, 2, 1]
```

**激进增益** (响应快但可能振荡):
```yaml
Kp: [500, 600, 500, 300, 200, 150]
Kd: [50, 60, 50, 30, 20, 15]
```

---

## 📊 关键配置文件

| 文件 | 作用 | 关键参数 |
|------|------|----------|
| `ros2_controllers.yaml` | ros2_control 配置 | `update_rate: 200` |
| `joint_limits.yaml` | 关节限位 | `min/max_position: ±3.2` |
| `ARV_V1_MODEL.urdf` | 机器人模型 | `<inertial>`, `<joint>` |
| `dynamics_computer.cpp` | 动力学计算 | KDL 调用 |
| `torque_controller_node.cpp` | 控制器核心 | PD 增益，控制律 |
| `mujoco_interface_node.cpp` | MuJoCo 仿真 | 执行器配置 |
| `mujoco_demo.launch.py` | 启动文件 | 无 joint_state_broadcaster |
| `start_mujoco_system.sh` | 一键启动脚本 | 节点启动顺序 |

---

## 📝 技术笔记

### 为什么 joint_1 的重力项接近 0？

joint_1 是基座旋转关节（绕 Z 轴），重力方向 (0, 0, -9.81) 与旋转轴平行，因此力臂为 0，重力力矩 ≈ 0。

### MuJoCo vs Mock vs Gazebo

| 模式 | 物理仿真 | 可视化 | 状态 |
|------|---------|--------|------|
| Mock | ❌ | RViz | ✅ 可用 |
| MuJoCo | ✅ | 3D 窗口 | ✅ 当前使用 |
| Gazebo | ✅ | 3D 窗口 | ❌ ROS2 Jazzy Bug |

**选择 MuJoCo 的原因**:
- Gazebo Harmonic + ROS2 Jazzy 有系统级 Bug
- MuJoCo 轻量快速，适合快速迭代调参
- Python 安装方便 (pip install mujoco)

### 控制频率选择

| 频率 | 适用场景 | 计算负载 |
|------|---------|---------|
| 100Hz | 慢速运动 | 低 |
| 200Hz | 标准控制（当前） | 中 |
| 1000Hz | 高速/高精度 | 高 |

当前选择 200Hz：平衡性能和计算量。

---

## 🎯 下一步工作

### 优先级1: 改善控制效果 🔥

1. **系统辨识**
   - 记录单关节阶跃响应
   - 分析系统特性（阻尼、惯性）
   - 调整 PD 增益

2. **数据分析**
   - 使用 PlotJuggler 绘制波形
   - 分析力矩命令是否合理
   - 检查跟踪误差模式

3. **参数优化**
   - 从单关节开始调参
   - 逐步增加多关节联动
   - 记录最优参数

### 优先级2: MuJoCo 仿真优化

1. **仿真精度**
   - 调整 timestep (0.005 → 0.002?)
   - 配置 solver 参数
   - 添加摩擦和阻尼

2. **可视化改进**
   - 添加坐标轴显示
   - 实时显示关节角度
   - 力矩可视化

### 优先级3: 系统稳定性

1. **异常处理**
   - 力矩限幅验证
   - 关节限位保护
   - 节点崩溃恢复

2. **性能监控**
   - 控制循环延迟统计
   - CPU 占用率监控
   - 内存泄漏检查

---

## 🚀 快速启动

### 编译

```bash
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT
source install/setup.bash
```

### 启动系统

```bash
# 方式1: 使用启动脚本（推荐）
bash src/ARV_V1_MOVEIT/bash/start_mujoco_system.sh

# 方式2: 手动启动
# 终端1: MuJoCo 接口
ros2 run ARV_V1_MOVEIT mujoco_interface_node

# 终端2: 力矩控制器
ros2 run ARV_V1_MOVEIT torque_controller_node

# 终端3: MoveIt + RViz
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py
```

### 测试

```bash
# 1. 检查节点
ros2 node list

# 2. 检查 Action Server
ros2 action list

# 3. 在 RViz 中规划并执行轨迹
# Motion Planning → Planning → Plan
# → Execute
```

---

**最后更新**: 2025-10-29
**当前状态**: MuJoCo 仿真工作 ✅ | 控制效果需优化 ⚠️
**下一步**: 系统辨识 + PD 参数调优
