# 上位机控制律迁移：级联PID → 阻抗控制

> 本文档供 agent/开发者阅读，包含完整的 Why/How/What/Where 信息，足以独立完成改动。

## Why：为什么要改

### 问题根因

STM32 下位机已将 J1(Yaw1)、J2(Pitch1)、J5(Pitch3) 从级联 PID 改为阻抗控制。原因：

1. **级联 PID 的 vel_limit 饱和产生极限环振荡（果冻）**：大位置误差 → vel_ref 饱和 → 恒力加速 → 超调 → 反向饱和 → 循环。这是结构性问题，无法通过调参解决。

2. **MIT 协议电机是力矩源，不需要速度环**：J8009/J4340/J4310 通过 MIT 协议直接接受力矩指令，内部电流环已闭合。中间的速度环是多余的，而且引入了 12-bit 速度量化噪声和滤波相位延迟。

3. **阻抗控制 `tau = K*(qt-q) - D*qdot` 没有中间层饱和**：力矩与误差线性相关，接近目标自然减速。参数物理直观（K=弹簧刚度，D=黏性阻尼），正交可调。

4. **速度反馈延迟导致 D 项正反馈**：CAN 速度反馈 + 低通滤波器的相位延迟使阻尼项变为激励源。下位机通过位置差分（4帧窗口≥2×CAN更新周期）解决。上位机从 joint_states topic 获取速度，无此问题（1kHz 已够平滑）。

### 当前状态不一致

- **STM32 本地控制（RC/SDC 模式）**：J1/J2/J5 已用阻抗控制
- **上位机 HostControl 模式**：仍用级联 PID 算力矩透传给 STM32

两端控制律不一致会导致：切换模式时动态行为差异、参数维护混乱、调试时无法对比。

---

## How：怎么改

### 架构不变的部分

```
上位机力矩计算 → 串口透传 → STM32 直接 SetTorque()
```

串口协议（0x0002 力矩包）、hardware_interface_node、STM32 接收端**全部不改**。只改上位机内部如何计算这个力矩。

### 控制律变更

**之前（级联 PID + 动力学前馈）：**
```
τ_total = dynamics_ff + cascade_pid.compute(q_ref, q_fdb, qdot_fdb, dt)

其中 cascade_pid 内部:
  pos_error → Kp*e + Ki*∫e + Kd*de/dt → vel_ref (clamp to vel_limit)
  vel_error = vel_ref - vel_fdb → Kp_vel*e + Ki_vel*∫e → τ_pid
```

**之后（阻抗控制 + 动力学前馈）：**
```
τ_total = dynamics_ff + impedance.compute(q_ref, q_fdb, qdot_fdb)

其中 impedance 内部:
  tau_d = -D * vel_fdb
  k_budget = tau_limit - |tau_d|            ← D 优先占用力矩额度
  tau_k = clamp(K * (q_ref - q_fdb), ±k_budget)
  return clamp(tau_k + tau_d, ±tau_limit)
```

**动力学前馈（dynamics_ff）完全不变**：M(q)*q̈_d + C(q,q̇) + G(q) 仍由 KDL 计算。阻抗控制只替代了"反馈修正"部分。

### 关节分配

| 关节 | 索引 | 控制律 | 原因 |
|------|------|--------|------|
| J1 (Yaw1) | 0 | **阻抗** | MIT 直驱，无减速器 |
| J2 (Pitch1) | 1 | **阻抗** | MIT 直驱，无减速器 |
| J3 (Pitch2) | 2 | 级联 PID | 有 2:1 外啮合减速器，级联 PID 适用 |
| J4 (Roll1) | 3 | 级联 PID | DJI GM6020，有内置减速器 |
| J5 (Pitch3) | 4 | **阻抗** | MIT 直驱 J4310 |
| J6 (Roll2) | 5 | 级联 PID | DJI M2006，有内置减速器 |

### 参数对照

| 关节 | K (Nm/rad) | D (Nm·s/rad) | tau_limit (Nm) |
|------|-----------|-------------|----------------|
| J1 | 60.0 | 10.5 | 28.0 |
| J2 | 80.0 | 11.0 | 40.0 |
| J5 | 10.0 | 0.63 | 7.0 |

参数设计依据：
- K：期望刚度，手掰不动的程度
- D = 2 × 0.7 × √(J_max × K)：ζ≈0.7 临界阻尼
- tau_limit：电机物理上限

---

## 具体改动文件

### 1. `/home/wuhuan/repositories/src/arv_v1_moveit/src/core/impedance_controller.hpp`

**已创建**。Header-only 实现，包含 `JointImpedance` 和 `MultiJointImpedance` 类。

### 2. `/home/wuhuan/repositories/src/arv_v1_moveit/config/controller_params.yaml`

在 `/torque_controller_action_server/ros__parameters/` 下新增：

```yaml
    # ========== 阻抗控制 (J1/J2/J5): tau = K*(qt-q) - D*qdot ==========
    # 与 STM32 本地阻抗参数同步，D-priority 抗饱和
    impedance:
      joint_1:
        K: 60.0
        D: 10.5
        tau_limit: 28.0
      joint_2:
        K: 80.0
        D: 11.0
        tau_limit: 40.0
      joint_5:
        K: 10.0
        D: 0.63
        tau_limit: 7.0
```

`cascade_pid` 节保留，J3/J4/J6 仍使用。

### 3. `/home/wuhuan/repositories/src/arv_v1_moveit/src/control/torque_controller_node.cpp`

#### 3.1 头文件

```cpp
#include "core/impedance_controller.hpp"
```

#### 3.2 成员变量（类内新增）

```cpp
// 阻抗控制器（J1, J2, J5）
std::unique_ptr<MultiJointImpedance> impedance_controller_;
// 标记哪些关节用阻抗（索引 0,1,4）
static constexpr bool kUseImpedance[6] = {true, true, false, false, true, false};
```

#### 3.3 初始化（从 YAML 读参数）

在已有的 cascade_pid 参数加载之后添加：

```cpp
// 阻抗控制器初始化
impedance_controller_ = std::make_unique<MultiJointImpedance>(6);

auto load_impedance = [&](size_t idx, const std::string& joint_name) {
    ImpedanceGains g;
    g.K = this->get_parameter("impedance." + joint_name + ".K").as_double();
    g.D = this->get_parameter("impedance." + joint_name + ".D").as_double();
    g.tau_limit = this->get_parameter("impedance." + joint_name + ".tau_limit").as_double();
    impedance_controller_->setJointGains(idx, g);
};
load_impedance(0, "joint_1");
load_impedance(1, "joint_2");
load_impedance(4, "joint_5");
```

#### 3.4 控制循环修改

找到当前调用 `cascade_pid_->compute(...)` 产生 `tau_cascade` 的位置。改为混合调用：

```cpp
// 原来：
// cascade_pid_->compute(pos_ref, pos_fdb, vel_fdb, dt, tau_cascade);

// 改为：
std::vector<double> tau_feedback(6, 0.0);
for (size_t i = 0; i < 6; ++i)
{
    if (kUseImpedance[i])
    {
        tau_feedback[i] = impedance_controller_->getJointController(i)
                              .compute(pos_ref[i], pos_fdb[i], vel_fdb[i]);
    }
    else
    {
        tau_feedback[i] = cascade_pid_->getJointController(i)
                              .compute(pos_ref[i], pos_fdb[i], vel_fdb[i], dt);
    }
}

// 最终力矩 = 动力学前馈 + 反馈修正（不变）
for (size_t i = 0; i < 6; ++i)
{
    tau_cmd[i] = dynamics_ff[i] + tau_feedback[i];
}
```

#### 3.5 Reset 路径

在状态切换（RELAX → ARMED 等）时 reset：
```cpp
impedance_controller_->resetAll();
```

---

## D-priority 抗饱和解释

为什么阻尼项（D）优先占用力矩额度：

当力矩总量超过 `tau_limit` 时，如果 K 和 D 平等截断，高速时 D 的制动力被 K 挤掉 → 无法刹车 → 极限环。

D-priority 策略：先无条件分配 D 项的力矩需求，剩余额度才给 K。这保证了任何时刻阻尼力都完整存在，高速时 K 自动降低 → 不再恒力加速 → 无极限环。

---

## 验证清单

1. `colcon build --packages-select arv_v1_moveit` 编译通过
2. `ros2 param get /torque_controller_action_server impedance.joint_2.K` 返回 80.0
3. ARMED hold 模式：各关节静止时力矩输出 ≈ G(q)（阻抗反馈 ≈ 0）
4. 手推测试：推动关节后松手，行为与 STM32 本地 RC 模式一致
5. 轨迹跟踪：执行 MoveIt 规划轨迹，跟踪误差 ≤ 0.01 rad
