# ARV V1 力矩控制系统 - 核心要点

> **更新**: 2025-10-31 | **状态**: 轨迹跟踪效果良好 ✅ | D 项调优中 ⚙️

---

## 📊 系统架构

```
┌─────────────────┐
│  MoveIt 规划器  │
└────────┬────────┘
         │ Action: /ARM_controller/follow_joint_trajectory
         ↓
┌─────────────────────────────────────────────────┐
│    Torque Controller Node (200Hz)              │
│  ┌─────────────┐    ┌──────────────┐          │
│  │ 动力学解算   │ →  │  PD 控制     │ → τ_total│
│  │ τ_ff=M·q̈+C+G│    │ τ_fb=Kp·e+Kd·ė│          │
│  └─────────────┘    └──────────────┘          │
└────────┬────────────────────────────────────────┘
         │ /effort_controller/commands
         ↓
┌─────────────────────────────────────────────────┐
│    MuJoCo Interface Node (200Hz)                │
│  ┌──────────┐  ┌──────────┐  ┌──────────────┐ │
│  │ 力矩执行  │→ │ 物理仿真  │→ │ 3D 可视化    │ │
│  │ ctrl[6]  │  │ mj_step  │  │ GLFW/OpenGL  │ │
│  └──────────┘  └──────────┘  └──────────────┘ │
└────────┬────────────────────────────────────────┘
         │ /joint_states
         ↓ (闭环反馈)
    [回到 Torque Controller]
```

---

## 🎯 控制律框图

### 状态机

```
    ┌─────────────┐
    │  系统启动    │
    └──────┬──────┘
           │ 收到首个关节状态
           ↓
    ┌─────────────────────┐
    │  保存启动姿态        │ q_target_ = q_actual_
    │  has_target_ = true │
    └──────┬──────────────┘
           │
           ↓
    ┌──────────────────────────────┐
    │   保持模式                    │ is_executing_ = false
    │   • τ = G(q) + PD(q_target_) │
    │   • 维持启动姿态/规划终点     │
    └──────┬───────────────────────┘
           │ 收到 MoveIt 轨迹
           ↓
    ┌──────────────────────────────┐
    │   执行模式                    │ is_executing_ = true
    │   • 更新 q_target_ = 终点    │
    │   • τ = τ_ff + PD(q_d - q)   │
    │   • 轨迹插值 & 跟踪          │
    └──────┬───────────────────────┘
           │ t ≥ t_end
           ↓
    [返回保持模式] → [收到新轨迹] → [立刻抢占切换]
```

### 控制律详细

**保持模式** (`!is_executing_`):
```
输入: q_actual_, q_dot_actual_, q_target_

重力补偿: τ_g = G(q_actual_)

PD 控制:   e_p = q_target_ - q_actual_
          e_v = 0 - q_dot_actual_
          τ_pd = Kp·e_p + Kd·e_v

输出: τ_total = τ_g + τ_pd
```

**执行模式** (`is_executing_`):
```
输入: 轨迹, t_now, q_actual_, q_dot_actual_

插值:     q_d, qd_d, qdd_d = interpolate(t_now)

前馈:     τ_ff = G(q_actual_)  [简化版]
         [完整版: τ_ff = M(q_d)·qdd_d + C(q_d,qd_d) + G(q_d)]

PD 反馈:  e_p = q_d - q_actual_
         e_v = qd_d - q_dot_actual_
         τ_fb = Kp·e_p + Kd·e_v

输出: τ_total = τ_ff + τ_fb
```

---

## ⚙️ 核心参数

### PD 增益 (torque_controller_node.cpp:35-47)
```cpp
// 位置增益
Kp: [600, 1000, 550, 150, 100, 20]   // N·m/rad

// 速度增益 (当前全部设为 0，因为 D 项异常)
Kd: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   // N·m·s/rad
```

**⚠️ 当前问题**:
- 第一关节 D 项在轨迹完成时计算出 500+ N·m（异常大）
- 临时方案：将所有 Kd 设为 0
- 仅使用 P 控制 + 完整动力学前馈，效果良好 ✅

### 控制频率
- 力矩控制器: **200 Hz**
- MuJoCo 仿真: **200 Hz**
- 渲染线程: **60 Hz**

### 执行器限制
- 最大力矩: **±20 N·m** (MuJoCo 配置)

---

## 🔧 关键代码位置

| 功能 | 文件 | 行数范围 |
|------|------|----------|
| 状态机变量 | torque_controller_node.cpp | 100-115 |
| PD 增益初始化 | torque_controller_node.cpp | 35-47 |
| 保存启动姿态 | torque_controller_node.cpp | 270-284 |
| 保存规划终点 | torque_controller_node.cpp | 220-236 |
| 保持模式控制 | torque_controller_node.cpp | 490-545 |
| 执行模式控制 | torque_controller_node.cpp | 547-669 |
| 动力学计算 | dynamics_computer.cpp | 11-52 |

---

## ✅ 已解决的关键问题

### 1. 死锁导致控制停止
**问题**: `controlLoop()` 嵌套加锁 → 永久阻塞
**解决**: 删除嵌套锁，外层加锁一次即可

### 2. 启动时机械臂漂移
**问题**: 启动后无目标位置 → PD 无效 → 重力漂移
**解决**: 首次收到状态时保存为 `q_target_`，立刻启用 PD

### 3. 轨迹完成后掉落
**问题**: 规划终点未保存 → 完成后 PD 目标错误
**解决**: 在 `handleAccepted()` 保存轨迹终点

### 4. RViz 机械臂闪烁
**问题**: 两个节点同时发布 `/joint_states`
**解决**: `mujoco_demo.launch.py` 禁用 `joint_state_broadcaster`

### 5. MuJoCo 窗口黑屏
**问题**: OpenGL 上下文在错误线程创建
**解决**: 在渲染线程中 `glfwMakeContextCurrent()` + `mjr_makeContext()`

### 6. 执行模式控制律错误
**问题**: 前馈只用重力补偿，PD 期望位置用了实际位置
**解决**: 
- 前馈改用完整动力学：`computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff)`
- PD 使用正确期望值：`computeFeedbackTorque(q_d, qd_d, q_actual, qd_actual, tau_fb)`

### 7. 轨迹完成时缺少 PD 控制
**问题**: 轨迹完成瞬间只发送重力补偿，没有 PD
**解决**: 轨迹完成时也计算 PD 控制，使用 `q_target_` 作为期望位置

---

## 🚧 待优化项

### 🔥 高优先级

1. **D 项异常问题诊断** ⚠️
   
   **现象**: 
   - 第一关节 D 项在轨迹完成时达到 500+ N·m
   - 临时方案：所有 Kd 设为 0（仅 P 控制 + 动力学前馈）
   - 当前效果：轨迹跟踪良好 ✅
   
   **可能原因**:
   - 速度误差异常大（qd_d - qd_actual ≈ 166 rad/s？）
   - 轨迹完成瞬间的速度跳变
   - MuJoCo 速度数据噪声或数值积分误差
   - Joint 1 转动惯量大，速度累积效应
   
   **调试方法**:
   ```cpp
   // 在轨迹完成时添加日志
   RCLCPP_WARN("实际速度: qd=[%.3f, %.3f, ...]", qd_actual_copy(0), ...);
   RCLCPP_WARN("D 项力矩: τ_d=[%.2f, %.2f, ...]", tau_pd(0), ...);
   ```
   
   **解决方案**:
   - [ ] 方案 A: 添加速度死区（小速度误差不产生 D 项）
   - [ ] 方案 B: 对速度信号进行滤波（低通滤波器）
   - [ ] 方案 C: 限制 D 项最大输出
   - [x] 方案 D: 调小 Kd 增益（临时：设为 0）

2. **速度测量质量改进**
   - 检查 MuJoCo 速度输出是否有噪声
   - 考虑对 `/joint_states` 的速度数据进行滤波
   - 验证速度数值范围是否合理

### ⚙️ 中优先级

3. **力矩限幅**
   ```cpp
   const double MAX_TORQUE[6] = {20, 20, 20, 20, 20, 20};
   for (size_t i = 0; i < 6; i++) {
       tau_total(i) = std::clamp(tau_total(i), -MAX_TORQUE[i], MAX_TORQUE[i]);
   }
   ```

4. **性能监控**
   - 控制循环耗时统计
   - CPU 占用率监控
   - 频率偏差检测

### 📊 已完成的优化

- [x] 完整动力学前馈（已实现）
- [x] 轨迹抢占机制（已实现）
- [x] 轨迹完成时的 PD 控制（已实现）
- [x] 启动姿态保存（已实现）
- [x] 规划终点保持（已实现）


---

## � 待优化项

### 高优先级

1. **完整动力学前馈** (当前仅重力补偿)
   ```cpp
   // 当前: torque_controller_node.cpp:595
   dynamic_computer_->computeGravityTorque(q_actual, tau_ff);
   
   // 改进:
   dynamic_computer_->computeFeedforwardTorque(q_d, qd_d, qdd_d, tau_ff);
   ```

2. **轨迹抢占机制**
   - 修改 `handleGoal()`: 删除拒绝新轨迹的逻辑
   - 修改 `handleAccepted()`: 取消旧轨迹，立刻切换

3. **PD 参数调优**
   - 单关节阶跃响应测试
   - 分析超调、稳态误差
   - 使用 PlotJuggler 绘制波形

### 中优先级

4. **力矩限幅**
   ```cpp
   const double MAX_TORQUE[6] = {20, 20, 20, 20, 20, 20};
   for (size_t i = 0; i < 6; i++) {
       tau_total(i) = std::clamp(tau_total(i), -MAX_TORQUE[i], MAX_TORQUE[i]);
   }
   ```

5. **性能监控**
   - 控制循环耗时统计
   - CPU 占用率监控
   - 频率偏差检测

---

## �️ 快速启动

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT
source install/setup.bash
```

### 启动
```bash
# 一键启动（推荐）
bash src/ARV_V1_MOVEIT/bash/start_mujoco_system.sh

# 或手动启动三个终端：
# 1. MuJoCo 仿真
ros2 run ARV_V1_MOVEIT mujoco_interface_node

# 2. 力矩控制器
ros2 run ARV_V1_MOVEIT torque_controller_node

# 3. MoveIt + RViz
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py
```

### 测试
```bash
# 检查节点
ros2 node list | grep -E "(torque|mujoco)"

# 监控力矩输出
ros2 topic echo /effort_controller/commands

# 监控关节状态
ros2 topic hz /joint_states

# 在 RViz 中规划并执行
# Motion Planning → Plan → Execute
```

---

## 📚 技术笔记

### 为什么 joint_1 重力项接近 0？
joint_1 绕 Z 轴旋转，重力 (0,0,-9.81) 平行于旋转轴 → 力臂=0 → τ_g≈0

### 控制频率选择
- **100Hz**: 慢速运动，计算负载低
- **200Hz**: 标准控制（当前），平衡性能
- **1000Hz**: 高速/高精度，计算负载高

### MuJoCo vs Gazebo
| 模式 | 物理仿真 | 可视化 | 状态 |
|------|---------|--------|------|
| MuJoCo | ✅ | 3D 窗口 | ✅ 当前 |
| Gazebo | ✅ | 3D 窗口 | ❌ ROS2 Jazzy Bug |

**选择原因**: Gazebo Harmonic + ROS2 Jazzy 有系统级 Bug，MuJoCo 轻量快速

---

**最后更新**: 2025-10-31  
**状态**: 轨迹跟踪效果良好 ✅ | D 项异常需诊断 ⚠️  
**当前配置**: P 控制 + 完整动力学前馈（Kd 全部为 0）  
**下一步**: D 项问题诊断 → 速度滤波 → 恢复 D 控制
