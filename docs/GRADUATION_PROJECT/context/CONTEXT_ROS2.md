# ROS2 上位机项目架构上下文

## 项目概览

- **包名**: ARV_V1_MOVEIT
- **路径**: `~/ros2_ws/src/ARV_V1_MOVEIT`
- **框架**: ROS2 Jazzy + MoveIt2 + MuJoCo
- **代码规模**: 4,199行核心代码
- **控制频率**: 200Hz (5ms周期)

## 分层架构

```
┌─ 应用层 (Application)      # Launch文件、系统初始化
├─ 规划层 (Planning)         # MoveIt2 OMPL规划器
├─ 控制层 (Control)          # 力矩控制、PID、Kalman (受保护)
├─ 接口层 (Interface)        # MuJoCo仿真、硬件串口
└─ 核心层 (Core)             # 工具函数、协议定义
```

## 核心控制算法 (受保护文件)

### 1. torque_controller_node.cpp (1409行)
**位置**: `src/core/torque_controller_node.cpp`
**功能**: 完整动力学前馈 + 级联PID反馈控制

**控制律**:
```
τ = τ_ff + τ_fb
  = [M(q)q̈ + C(q,qdot) + G(q)] + [级联PID(位置误差, 速度误差)]
```

**级联控制结构**:
- 外环(位置): e_pos = q_ref - q_fdb → v_ref = Kp·e_pos
- 内环(速度): e_vel = v_ref - v_est → τ = Kp·e_vel + Ki∫e_vel

**安全机制**:
- 100ms关节状态超时 → 紧急停止
- 力矩限幅 (per-joint)
- NaN/Inf检测 → 安全降级
- 位置误差阈值 (0.8 rad) → 自动恢复

### 2. dynamics_computer.cpp (109行)
**位置**: `src/core/dynamics_computer.cpp`
**功能**: KDL动力学计算

**接口**:
```cpp
// 质量矩阵 M(q)
Eigen::MatrixXd computeMassMatrix(const KDL::JntArray& q);

// 科氏力 C(q, qdot)
Eigen::VectorXd computeCoriolisVector(const KDL::JntArray& q, const KDL::JntArray& qdot);

// 重力补偿 G(q)
Eigen::VectorXd computeGravityVector(const KDL::JntArray& q);
```

### 3. cascade_pid.cpp (184行)
**位置**: `src/core/cascade_pid.cpp`
**功能**: 级联P+PI控制器

**参数** (controller_params.yaml):
```yaml
cascade_pid:
  joint_1:
    pos_Kp: 3.0      # 外环位置比例增益
    vel_Kp: 7.0      # 内环速度比例增益
    vel_Ki: 0.5      # 内环速度积分增益
```

**特性**:
- 外环无积分 (避免积分饱和)
- 内环PI (消除静差)
- 速度饱和保护
- 条件积分抗饱和 (误差<0.1 rad时积分)

### 4. kalman_filter.cpp (92行)
**位置**: `src/core/kalman_filter.cpp`
**功能**: 速度估计滤波

**状态方程**:
```
状态: x = [q, qdot]
预测: x_{k|k-1} = F·x_{k-1|k-1}
更新: x_{k|k} = x_{k|k-1} + K·(z_k - H·x_{k|k-1})
```

**参数调节**:
```yaml
kalman:
  Q_pos: 1e-5   # 过程噪声-位置
  Q_vel: 1e-4   # 过程噪声-速度
  R_pos: 1e-3   # 测量噪声-位置
  R_vel: 2.5e-2 # 测量噪声-速度
```

## 接口层实现

### 1. mujoco_interface_node.cpp (779行)
**位置**: `src/interfaces/mujoco_interface_node.cpp`

**双模式支持**:
- **物理仿真模式**: 订阅力矩命令 → MuJoCo仿真 → 发布joint_states
- **数字孪生模式**: 订阅joint_states → OpenGL可视化 (visualization_only=true)

**特性**:
- 200Hz仿真频率
- GLFW OpenGL渲染
- URDF→MuJoCo XML转换

### 2. hardware_interface_node.cpp (718行)
**位置**: `src/interfaces/hardware_interface_node.cpp`

**串口通信**:
- 波特率: 921600 baud
- 协议: Seasky (CRC16校验)
- 频率: 200Hz双向

**Seasky协议帧格式**:
```
[SOF(0xA5)][Len(2)][CRC8][CmdID][Flags][Payload][CRC16]
```

## 节点拓扑

```
MoveIt2 规划器
    ↓
    └─→ /ARM_controller/follow_joint_trajectory (Action)
            ↓
    ┌──────────────────────────────────────────────────┐
    │   torque_controller_node (Action Server)         │
    │   • 订阅 /joint_states                           │
    │   • 发布 /effort_controller/commands             │
    └──────────────────────────────────────────────────┘
            ↓
    ┌─────────────────────────────────────────┐
    │  mujoco_interface_node (仿真)           │
    │  OR                                      │
    │  hardware_interface_node (真机)         │
    └─────────────────────────────────────────┘
```

## 关键配置文件

| 文件 | 路径 | 用途 |
|------|------|------|
| controller_params.yaml | config/ | Kalman、PID、安全参数 |
| ros2_controllers.yaml | config/ | 力矩控制器200Hz |
| joint_limits.yaml | config/ | 关节限位 |
| initial_positions.yaml | config/ | 初始位姿 |

## 启动命令

```bash
# 编译
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT ARV_V1_MODEL

# 仿真模式
ros2 launch ARV_V1_MOVEIT mujoco_demo.launch.py
ros2 run ARV_V1_MOVEIT torque_controller_node
ros2 run ARV_V1_MOVEIT mujoco_interface_node

# 真机模式
ros2 run ARV_V1_MOVEIT hardware_interface_node --ros-args -p serial_port:=/dev/ttyACM0
ros2 run ARV_V1_MOVEIT torque_controller_node
```

## 调试命令

```bash
# 参数动态调节
ros2 param set /torque_controller_action_server cascade_pid.joint_1.pos_Kp 5.0
ros2 param set /torque_controller_action_server kalman.Q_vel 1e-5

# 话题监控
ros2 topic echo /joint_states
ros2 topic echo /effort_controller/commands
```

## 技术栈

| 组件 | 版本 | 用途 |
|------|------|------|
| ROS2 | Jazzy | 中间件 |
| MoveIt2 | - | 运动规划 |
| KDL | Orocos | 动力学计算 |
| MuJoCo | 3.x | 物理仿真 |
| Eigen3 | 3.x | 矩阵运算 |

## 完成度

| 模块 | 状态 | 行数 |
|------|------|------|
| 核心控制 | ✅ 95% | 1409 |
| 动力学计算 | ✅ 100% | 176 |
| 级联PID | ✅ 100% | 360 |
| Kalman滤波 | ✅ 100% | 135 |
| MuJoCo接口 | ✅ 100% | 779 |
| 硬件接口 | ✅ 100% | 718 |
| 轨迹管理 | ✅ 100% | 535 |
| **视觉系统** | 🔴 20% | - |
