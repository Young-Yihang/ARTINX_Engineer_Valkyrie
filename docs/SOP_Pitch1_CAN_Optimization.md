# Pitch1 CAN 频率 & 优先级优化 SOP

## 目标

将 Pitch1 (J8009) 的等效环路延迟从 ~2.3ms 降至 ~0.8ms，使阻抗控制刚度上限提升约 3 倍。

---

## 背景 (Why)

阻抗控制的最大刚度受限于环路延迟：`Kp_max = Kd / (2 × τ_loop)`。

当前 Pitch1 延迟高的两个原因：
1. **UpdatePeriod=2** → 只有 500Hz 发送，每 2ms 才和电机通信一次
2. **FIFO 排最后** → J8009 的 MotorType 枚举值最大，CanMotorManager 遍历时最后处理，入 FDCAN TX FIFO 排第 5 位，前面 4 帧占线 1040µs

---

## 改动内容 (What)

共两处改动，按顺序实施。

### 改动 1: Pitch1 发送频率 2ms → 1ms

文件: `ArmController.cpp` 的 `Init()` 函数

```cpp
// 改前
Pitch1.SetUpdatePeriod(2);

// 改后
Pitch1.SetUpdatePeriod(1);
```

### 改动 2: Pitch1 优先发送（排第一个进 FIFO）

核心思路：让 Pitch1 的 `Update()`（里面会组包并 publish CAN 帧）在所有其他电机之前执行。

方案 A（推荐，改动最小）：在 `ControllerTask.cpp` 的主循环中，把 Pitch1 单独提前调用。

文件: `ControllerTask.cpp`

```cpp
for (;;)
{
    controller->Update();
    boardPacket->Update();
    hostCom->Update();
    judgeSystem->Update();

    // ===== 新增：Pitch1 优先发送 =====
    controller->UpdatePitch1Motor();
    // ===== 新增结束 =====

    canMotorManager->Update();
    djiCanMotorCommander->Update();
    osDelay(1);
}
```

文件: `ArmController.hpp` 新增接口

```cpp
public:
    void UpdatePitch1Motor() { Pitch1.Update(); }
```

文件: `CanMotorManager::Update()` 跳过 Pitch1（避免重复发送）

在 Pitch1 的 Update() 里加一个「本 tick 已发送」的 guard，或者在 CanMotorManager 的遍历里识别并跳过 Pitch1。最简单的方式是让 Pitch1 **不注册进 CanMotorManager**：

文件: `ArmController.cpp` 的 `Init()`

```cpp
// 改前
Pitch1.RegistMotor(CAN1, 0x04, 0x03);
// 这会触发 CanMotorManager::AddMotor()（在 CanMotor 构造函数中）

// 改后方案：Pitch1 仍然调用 RegistMotor 设置 CAN 通道和 ID，
// 但需要从 CanMotorManager 中移除。
// 最简单：在 CanMotor 构造函数中加一个参数控制是否注册，
// 或者在 Init() 最后手动移除。
```

如果改构造函数太侵入，替代方案：利用 J8009 已有的 `m_LastTxTick` 机制——既然 `UpdatePitch1Motor()` 已经发了，`CanMotorManager::Update()` 再调 `Pitch1.Update()` 时，`(nowTick - m_LastTxTick) >= currentPeriod` 条件不满足（刚发过），自然跳过。**Period=1 时两次调用间隔 <1ms，条件不满足，不会重复发。无需额外改动。**

---

## 实施步骤 (When)

### Step 1: 验证当前状态（不改代码）

1. 编译烧录当前代码，确认 Pitch1 正常工作（阻抗控制 PD + G(q) 已生效）
2. 用 Keil 调试器观察 `Pitch1.m_LastTxTick` 确认当前是每 2ms 递增

### Step 2: 改 UpdatePeriod（单独验证频率）

1. 只改 `Pitch1.SetUpdatePeriod(1)`
2. 编译烧录
3. 验证：观察 `Pitch1.m_LastTxTick` 每 1ms 递增
4. 观察电机行为：应该和之前基本一致（可能略稳，不会更差）
5. 观察 CAN1 负载：如果有 `busy_count` 在涨说明 FIFO 溢出（不应该发生，如果发生则回退）

### Step 3: 改优先级（叠加 Step 2）

1. 在 `ControllerTask.cpp` 加入 `controller->UpdatePitch1Motor()`
2. 在 `ArmController.hpp` 暴露 `UpdatePitch1Motor()` 接口
3. 编译烧录
4. 验证：Pitch1 的力矩响应更快、同样参数下更稳定
5. GPIO 实测（可选）：在 `UpdatePitch1Motor()` 前置 HIGH，在 `Pitch1.HandleNewCanRxMsg()` 置 LOW，示波器量高电平宽度应 < 500µs

### Step 4: 调参（可选，在 Step 3 验证通过后）

改完后系统更稳，可以逐步提升 Kp：
- 当前 Kp 不变 → 确认不震
- Kp × 1.5 → 测试
- Kp × 2.0 → 测试
- 直到找到新的震荡边界

**注意**：不要同时改频率和参数，一次只改一个变量。

---

## CAN1 负载验证

改后 CAN1 负载：

| 时刻 | 帧数 | 占用 |
|------|------|------|
| 偶数ms (全部电机) | 12帧 (6TX+6RX) | 1560µs |
| 奇数ms (仅Pitch1) | 2帧 | 260µs |
| 2ms 平均 | ~7帧/ms | **78%** |

78% 是安全的（FDCAN TX FIFO 28 深，不会溢出）。如果观察到 `busy_count` 持续增长，说明有问题需要回退排查。

---

## 回退方案

任何一步出问题：
1. `Pitch1.SetUpdatePeriod(2)` 恢复
2. 删除 `controller->UpdatePitch1Motor()` 行
3. 回到改前状态

改动完全可逆，不影响其他电机。

---

## 不需要改的东西

- **PD 参数**: 不需要立刻改。τ_loop 降低只会增加稳定裕度，不会让现有参数变危险
- **D 项 4ms 差分窗口**: 1kHz 下 4ms = 4 个采样点（原来 500Hz 下是 2 个），更平滑，不需要调
- **重力补偿**: 与频率无关，不需要改
- **其他电机**: 全部保持 Period=2 不动，它们对延迟不敏感
- **速度反馈滤波器**: 保持不动，后续调参时再考虑
