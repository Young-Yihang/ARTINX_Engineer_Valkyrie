# ARV_V1 系统架构与核心实现

## 一、系统概览

**项目**: ARV_V1 6自由度机械臂控制系统
**平台**: ROS2 Jazzy + MoveIt2 + MuJoCo
**控制频率**: 200Hz
**通信协议**: USB串口 Seasky协议 (921600 bps)

### 当前分支状态
- **分支**: feature/ros2_components
- **已完成**: ✅ 双模式架构 | ✅ 级联PID | ✅ Kalman滤波 | ✅ 串口通信 | ✅ 数字孪生

---

## 二、三层架构设计

```
┌─────────────────────────────────────────────────────────────────┐
│                    规划层 (Planning)                              │
│  MoveIt2 (move_group) → /ARM_controller/follow_joint_trajectory │
└────────────────────────┬────────────────────────────────────────┘
                         │ Action接口
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    控制层 (Control) - 200Hz                      │
│  torque_controller_node                                          │
│  • 动力学前馈: τ_ff = M(q)q̈ + C(q,q̇) + G(q)                    │
│  • 级联反馈: τ_fb = 外环P(位置→速度) + 内环PI(速度→力矩)         │
│  • 安全保护: 力矩饱和、超时检测、NaN检查                          │
└────────────────────────┬────────────────────────────────────────┘
                         │ /effort_controller/commands
        ┌────────────────┴────────────────┐
        ▼                                  ▼
┌──────────────────────────┐    ┌────────────────────────┐
│    执行层-仿真            │    │    执行层-硬件          │
│  mujoco_interface_node    │    │ hardware_interface_node│
│  • 物理仿真 200Hz         │    │ • 串口通信 200Hz        │
│  • 可视化渲染 60Hz        │    │ • 双定时器解耦架构      │
└──────────────────────────┘    └────────────────────────┘
```

---

## 三、核心节点实现

### 3.1 torque_controller_node (1400行)

**控制律实现**:
```cpp
// 级联控制架构
// 外环: 位置P控制
v_cmd = Kp_pos * (q_target - q_actual)
v_cmd = clamp(v_cmd, -vel_limit, +vel_limit)  // 速度饱和

// 内环: 速度PI控制
e_v = v_cmd - v_actual
τ_cascade = Kp_vel * e_v + Ki_vel * ∫e_v dt

// 动力学前馈 (KDL计算)
τ_ff = M(q)*q̈_d + C(q,q̇)*q̇ + G(q)

// 最终输出
τ_total = τ_ff + τ_cascade
τ_output = clamp(τ_total, τ_min, τ_max)  // 力矩限幅
```

**关键特性**:
- Kalman滤波器降噪 (可选启用)
- 100ms超时保护
- 参数热重载 (`ros2 param set`)

### 3.2 hardware_interface_node (600行)

**双定时器解耦架构**:
```cpp
// 定时器1: 200Hz发送循环
void sendLoop() {
    // 读取缓存的力矩指令
    float torques[6];
    {
        std::lock_guard lock(mutex_);
        memcpy(torques, cached_torques_, sizeof(torques));
    }

    // Seasky协议封装
    auto packet = buildSeaskyPacket(0x0002, torques);
    serial_port_->write(packet);
}

// 定时器2: 异步接收线程
void receiveLoop() {
    while (running_) {
        auto data = serial_port_->read();
        if (parseSeaskyPacket(data, &feedback)) {
            publishJointStates(feedback);
        }
    }
}
```

**通信协议** (Seasky格式):
```
[SOF:0xA5][Len:2B][CRC8][CmdID:2B][Payload][CRC16:2B]

命令ID:
- 0x0002: 力矩控制 (6×float32)
- 0x0001: 关节反馈 (位置+速度+状态)
```

### 3.3 mujoco_interface_node (700行)

**双模式支持**:
1. **仿真模式**: 接收力矩→物理仿真→发布状态
2. **数字孪生模式**: 订阅真实状态→同步3D显示

```cpp
if (visualization_only_) {
    // 数字孪生: 跟随真实反馈
    for (int i = 0; i < 6; ++i) {
        data_->qpos[i] = msg->position[i];
        data_->qvel[i] = msg->velocity[i];
    }
    // 不调用mj_step, 仅更新可视化
} else {
    // 仿真模式: 物理步进
    mj_step(model_, data_);  // 5ms步长
}
```

---

## 四、控制参数配置

### 4.1 级联PID增益 (controller_params.yaml)

| 关节 | 外环P | 内环P | 内环I | 速度限制 | 力矩限制 |
|------|-------|-------|-------|----------|----------|
| J1 | 3.0 | 7.0 | 0.5 | 10 rad/s | 20 Nm |
| J2 | 5.0 | 7.0 | 0.5 | 10 rad/s | 20 Nm |
| J3 | 3.0 | 6.0 | 0.5 | 10 rad/s | 20 Nm |
| J4 | 1.5 | 6.3 | 0.3 | 10 rad/s | 1.2 Nm |
| J5 | 0.05 | 1.0 | 0.1 | 10 rad/s | 7.0 Nm |
| J6 | 0.05 | 1.0 | 0.1 | 10 rad/s | 1.0 Nm |

### 4.2 Kalman滤波参数

```yaml
kalman:
  Q_vel: 1e-4    # 过程噪声 (大→响应快)
  R_vel: 2.5e-2  # 测量噪声 (大→更平滑)
  enabled: true
```

调参判据: Kalman增益K∈[0.1, 0.3]为平衡配置

---

## 五、硬件配置

| 关节 | 电机型号 | 额定扭矩 | 通信接口 |
|------|----------|----------|----------|
| 1-3 | J8009 | 25 Nm | CAN |
| 4 | GM6020 | 2.5 Nm | CAN |
| 5 | J4310 | 15 Nm | CAN |
| 6 | M2006 | 0.72 Nm | CAN |

**上下位机架构**:
```
Intel NUC (ROS2) ←USB→ STM32 ←CAN→ 6个电机
```

---

## 六、启动与运行

### 6.1 编译
```bash
cd ~/ros2_ws
colcon build --packages-select ARV_V1_MOVEIT ARV_V1_MODEL
source install/setup.bash
```

### 6.2 运行模式

**模式1: 纯仿真**
```bash
./start_mujoco_system.sh
# 选择 [1]
# 启动: MoveIt2 + torque_controller + mujoco_interface
```

**模式2: 硬件+数字孪生**
```bash
./start_mujoco_system.sh
# 选择 [2]
# 启动: MoveIt2 + torque_controller + hardware_interface + mujoco(viz_only)
```

### 6.3 调试命令
```bash
# 监控关节状态
ros2 topic echo /joint_states

# 查看力矩命令
ros2 topic echo /effort_controller/commands

# 实时调参
ros2 param set /torque_controller_action_server cascade_pid.joint_1.vel_Kp 8.0

# 参数热重载
./reload_params.sh
```

---

## 七、性能指标

| 指标 | 目标值 | 实测值 | 状态 |
|------|--------|--------|------|
| 控制频率 | 200Hz | 198-202Hz | ✅ |
| 控制延迟 | <10ms | ~8ms | ✅ |
| 位置精度 | <0.01rad | 0.008rad | ✅ |
| CPU占用 | <70% | 65% | ✅ |
| 内存使用 | <2GB | 1.3GB | ✅ |

---

## 八、安全机制

### 8.1 多层保护
1. **力矩限幅**: 每个关节独立配置
2. **速度限制**: 防止失控加速
3. **超时检测**: 100ms无反馈触发保护
4. **NaN/Inf检查**: 数值异常立即停止
5. **紧急停止**: force_zero_torque参数

### 8.2 故障处理
```cpp
// 超时保护
if ((now - last_state_time_).seconds() > 0.1) {
    RCLCPP_ERROR("Joint state timeout!");
    // 切换到重力补偿模式
    computeGravityCompensation(tau_cmd);
}

// 数值检查
if (std::isnan(torque) || std::isinf(torque)) {
    emergency_stop();
    cascade_pid_.reset();  // 清除积分项
}
```

---

## 九、技术亮点

### 9.1 解耦架构
- 控制计算与硬件传输完全独立
- 双200Hz定时器保证确定性
- 单层故障不影响系统

### 9.2 级联控制
- 外环生成平滑速度指令
- 内环精确跟踪+消除稳态误差
- 关节独立调参

### 9.3 实时性优化
- 预分配内存避免动态申请
- 锁最小化(仅保护关键数据)
- 编译优化(-O3 -march=native)

---

## 十、常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 启动漂移 | 初始状态未保存 | 首次joint_states自动作为目标 |
| 轨迹完成后下坠 | 目标丢失 | 保存轨迹终点作为新目标 |
| 抖动/振荡 | 增益过高 | 降低Kp或增加Kd |
| 响应迟缓 | 增益过低 | 提高Kp_pos和Kp_vel |
| 串口通信失败 | 权限不足 | sudo chmod 666 /dev/ttyACM0 |

---

**最后更新**: 2026-01-19
**维护者**: Young-Yihang
**版本**: 2.0