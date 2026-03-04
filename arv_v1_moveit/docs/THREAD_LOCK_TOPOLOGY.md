# ARV_V1 线程模型与锁拓扑 (2026-03-02)

## 1. 全系统进程-线程总览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  ROS2 Executor Process (单进程, 多节点, 共享线程池)                           │
│                                                                             │
│  ┌─── torque_controller ───┐  ┌── mujoco_interface ──┐  ┌─ hardware_if ──┐ │
│  │ T1: ROS2 executor pool  │  │ T1: ROS2 executor    │  │ T1: ROS2 exec  │ │
│  │   ├─ controlLoop 200Hz  │  │   ├─ simStep 200Hz   │  │   ├─ send 200Hz│ │
│  │   ├─ jointStateCB       │  │   ├─ effortCB        │  │   ├─ torqueCB  │ │
│  │   ├─ controlModeCB      │  │   ├─ jointStateCB    │  │   └─ health 5s │ │
│  │   ├─ gripperCB          │  │   └─ health 5s       │  │                │ │
│  │   ├─ handleGoal/Cancel  │  │                       │  │ T2: receive    │ │
│  │   ├─ handleAccepted     │  │ T2: render thread     │  │   (std::thread)│ │
│  │   └─ parametersCB       │  │   (std::thread)       │  └────────────────┘ │
│  └──────────────────────────┘  └───────────────────────┘                     │
│                                                                             │
│  ┌── mission_executor ──────────────────┐  ┌── trajectory_manager ────────┐ │
│  │ T1: Main thread (ncurses UI loop)    │  │ T1: ROS2 executor            │ │
│  │   ├─ drawUI() 10Hz                   │  │   ├─ jointStateCB            │ │
│  │   ├─ handleInput()                   │  │   ├─ trajectoryCB            │ │
│  │   └─ spin_some()                     │  │   ├─ saveCB / loadCB         │ │
│  │                                       │  │   └─ listCB                  │ │
│  │ T2: AsyncTaskRunner worker            │  └─────────────────────────────┘ │
│  │   (std::thread, 条件变量驱动)          │                                  │
│  │                                       │                                  │
│  │ T3: ROS2 callback (pose_sub, cmd_sub) │                                  │
│  └───────────────────────────────────────┘                                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 节点内锁拓扑图

### 2.1 torque_controller_node — 4 把锁

想象这个节点是一个**工厂**，有 4 间上锁的仓库和 7 个工人（回调）。
工人们轮班干活，同一时刻 ROS2 executor 只派一个工人上班（单线程 executor），
但**工人之间可以被打断**——Timer 回调可以在 Subscriber 回调之间插入。

#### 四间仓库（互斥量）各自存什么

```
  ┌─ state_mutex_ ───────────────────────────────────────────────────────┐
  │  "传感器数据间" — 机械臂此刻在哪、在动多快                             │
  │                                                                      │
  │  q_actual_[6]          ← 六个关节的实际角度                           │
  │  q_dot_actual_[6]      ← 六个关节的原始角速度                         │
  │  q_dot_filtered_[6]    ← Kalman 滤波后的角速度                        │
  │  q_target_[6]          ← "我要去哪" (HOLD/OVERDRIVE 的目标)          │
  │  has_target_           ← 有没有设过目标                               │
  │  state_received_       ← 开机后是否收到过第一帧 joint_states           │
  │  last_joint_state_time ← 最后一次收到传感器的时间戳                    │
  └──────────────────────────────────────────────────────────────────────┘

  ┌─ filter_mutex_ ──────────────────────────────────────────────────────┐
  │  "Kalman 滤波器间" — 6 个独立的一阶 Kalman 滤波器                     │
  │                                                                      │
  │  joint_filters_[6]     ← 每个关节一个, 内含状态向量和协方差矩阵        │
  └──────────────────────────────────────────────────────────────────────┘

  ┌─ action_mutex_ ──────────────────────────────────────────────────────┐
  │  "轨迹执行间" — 正在跑的轨迹和 MoveIt Action 的句柄                   │
  │                                                                      │
  │  current_trajectory_    ← 当前正在执行的轨迹点序列                     │
  │  trajectory_start_time_ ← 轨迹开始的时间戳                            │
  │  current_goal_handle_   ← ROS2 Action 的 GoalHandle (发反馈/完成用)   │
  │  is_executing_ (写)     ← 原子 bool, 读不需要锁, 写在锁内做           │
  └──────────────────────────────────────────────────────────────────────┘

  ┌─ gripper_mutex_ ─────────────────────────────────────────────────────┐
  │  "夹爪指令间" — 就一个 double                                        │
  │                                                                      │
  │  gripper_torque_cmd_    ← 夹爪力指令 (N)                              │
  └──────────────────────────────────────────────────────────────────────┘
```

#### 七个工人（回调函数）各自做什么、怎么拿钥匙

用时间线图表示每个工人一次完整执行中，**何时持锁、何时释放**。
横轴是时间从左到右，色块表示持锁区间：

```
 ① controlLoop (200Hz timer, 每 5ms 触发一次 — 最繁忙的工人)
 ───────────────────────────────────────────────────────────────────
 典型的 HOLD 模式执行:

 时间 ─→  0μs                    100μs                        5000μs
          │                       │                              │
          ├─ 读 atomic ──────────┤  (无锁, 纯计算)               │
          │  is_executing_?       │  重力补偿 + PD               │
          │  control_mode_?       │  NaN 检查                    │
          │                       │  力矩限幅                    │
          ├─ ██ state ██─→释放   │                              │
          │  copy-out:            │                              │
          │  q_copy=q_actual_     │                              │
          │  qd_copy=q_dot_filt  │                              │
          │  (~10μs)              │                              │
          │                       │                              │
          │                       ├─ █ gripper █─→释放          │
          │                       │  读 gripper_cmd (~1μs)      │
          │                       │                              │
          │                       ├─ publish(torque_msg) ────────┤
          │                       │                              │
          └───────────────────────┴──────────────────────────────┘

 典型的 EXECUTE 模式执行:

 时间 ─→  0μs         50μs            200μs                  5000μs
          │            │                │                        │
          ├─ ██ state ██─→释放         │                        │
          │  copy-out                   │                        │
          │                             │                        │
          ├─ ██ action ██─→释放        │                        │
          │  copy-out:                  │                        │
          │  traj_copy=trajectory_      │                        │
          │  goal_copy=goal_handle_     │                        │
          │                             │                        │
          │           ├─ ██ state ██──→释放                     │
          │           │  copy-out:      │                        │
          │           │  q_actual       │                        │
          │           │  qd_filtered    │  (无锁, 纯计算)        │
          │           │                 │  插值+前馈+PD          │
          │           │                 │                        │
          │           │                 ├─ █ gripper █─→释放    │
          │           │                 │                        │
          │           │                 ├─ publish(torque) ──────┤
          └───────────┴─────────────────┴────────────────────────┘

 关键: 所有锁都是 "进去拿数据, 出来算", 从不在持锁时做计算
       这就是 "copy-out" 模式的精髓


 如果出异常 (NaN / 超时 / dynamics异常):

 时间 ─→                       异常点
                                 │
          ├─ publish(零力矩/上次有效力矩) ← 无锁, 直接发
          │
          ├─ emergencyStop() 被调用:
          │  ├─ ████████ action ████████────────→释放
          │  │  executionRecoveryCeremony():
          │  │  ├─ 清空轨迹, abort goal_handle
          │  │  ├─ is_executing_ = false
          │  │  ├─ PID reset (无锁)
          │  │  ├─ ██ state ⊃ filter ██─→释放  Kalman 重初始化
          │  │  └─ ██ state ██─→释放           发重力补偿
          │  └──────────────────────────────────
          │
          │  这是唯一的嵌套锁: action 持有时, 内部获取 state 和 filter
          └──────────────────────────────────────────────────────────
```

```
 ② jointStateCallback (每收到一帧 /joint_states 触发 — 200Hz)
 ───────────────────────────────────────────────────────────────────
 这是唯一使用 unique_lock (手动 unlock) 的工人:

 时间 ─→  0μs              30μs           50μs         60μs
          │                  │               │            │
          ├─ ████████ state (unique_lock) ████████─unlock─┤
          │  ├─ 校验数据 (NaN/Inf/范围)                    │
          │  ├─ 写入 q_actual_, q_dot_actual_              │
          │  ├─ 更新 last_joint_state_time_                │
          │  │                                             │
          │  ├─ ██ filter ██─→释放                        │
          │  │  Kalman predict → update                    │
          │  │  写回 q_dot_filtered_                       │
          │  │                                             │
          │  ├─ state.unlock()  ← 手动释放!                │
          │                                                │
          ├─ (仅首次) █ filter █─→释放  打印 Kalman 增益   │
          ├─ (仅首次) █ action █─→释放  设置初始 target     │
          └────────────────────────────────────────────────┘

 为什么要手动 unlock?
 因为后面要拿 action_mutex_, 如果还持着 state_mutex_,
 就和 executionRecoveryCeremony (action→state) 形成反向 = 死锁!
```

```
 ③ controlModeCallback (mission_executor 发 /control_mode 时触发)
 ───────────────────────────────────────────────────────────────────

 时间 ─→  0μs              20μs                40μs
          │                  │                    │
          ├─ █ action █─→释放                    │  (仅在执行中切模式)
          │  double-check is_executing_           │
          │  executionRecoveryCeremony()          │
          │  (内部: action持有→state→filter)       │
          │                                       │
          ├─ █ state █─→释放                     │  (仅切到 OVERDRIVE)
          │  q_target_ = q_actual_                │
          │                                       │
          ├─ PID reset (无锁)                     │
          ├─ control_mode_.store(new_mode)         │  (atomic, 无锁)
          └───────────────────────────────────────┘

 注意: action 和 state 是两次独立获取, 不嵌套 ✓
```

```
 ④ handleAccepted (MoveIt 发来新轨迹被接受时触发)
 ───────────────────────────────────────────────────────────────────

 时间 ─→  0μs    10μs   20μs                          100μs
          │       │       │                               │
          ├─ █ state █─→释放  检查 state_received_       │
          │                                               │
          ├─ █ state █─→释放  拷贝 q_actual_ (起点检查)   │
          │                                               │
          ├─ █████████ action █████████─→释放             │
          │  ├─ abort 旧轨迹 (如果有)                      │
          │  ├─ 缓存新轨迹, 设 is_executing_=true         │
          │  ├─ 转发轨迹到 topic                           │
          │  ├─ 位置连续性检查 + PID reset                 │
          │  └─ 保存终点为 q_target_                       │
          └────────────────────────────────────────────────┘

 注意: state 拿了两次但都是独立的 copy-out, 不与 action 嵌套 ✓
```

```
 ⑤ gripperControlCallback (TUI 发 /gripper_control service 时触发)
 ──────────────────────────────────────────────────────────────
 █ gripper █─→释放  写入 gripper_torque_cmd_  (~1μs, 最简单的工人)
```

```
 ⑥ handleGoal / ⑦ handleCancel (Action Server 内部回调)
 ──────────────────────────────────────────────────────────────
 handleGoal:  █ action █─→释放  检查 is_executing_ 决定接受/拒绝
 handleCancel: (无锁, 直接 ACCEPT)
```

#### 锁之间的"交通规则"——为什么不会死锁

```
  把 4 把锁想象成 4 条单行道, 只允许从上往下走:

       ┌──────────┐
       │ action   │  ◄── 最高级 (轨迹调度权)
       └────┬─────┘
            │  只有 executionRecoveryCeremony 会在持有 action 时进入 state
            ▼
       ┌──────────┐
       │ state    │  ◄── 中级 (传感器数据)
       └────┬─────┘
            │  只有 jointStateCallback 和 executionRecoveryCeremony
            │  会在持有 state 时进入 filter
            ▼
       ┌──────────┐
       │ filter   │  ◄── 低级 (Kalman 内部状态)
       └──────────┘

       ┌──────────┐
       │ gripper  │  ◄── 独立 (永远不和任何锁嵌套)
       └──────────┘

  死锁的条件: A 持锁1等锁2, B 持锁2等锁1 (反向)

  我们的规则: 谁都只能从上往下拿, 永远不能从下往上拿
              → 不可能出现反向 → 不会死锁

  验证每个工人:
  ┌─────────────────────┬─────────────────────────────────────┬──────┐
  │ 工人                 │ 拿锁顺序                            │ 安全 │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ controlLoop         │ state→释放, action→释放, gripper    │  ✓  │
  │                     │ (从不嵌套, 全部 copy-out)            │      │
  │                     │ 异常时: action ⊃ state ⊃ filter     │  ✓  │
  │                     │ (从上到下)                            │      │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ jointStateCallback  │ state ⊃ filter→释放→state.unlock() │  ✓  │
  │                     │ 然后单独: action→释放                │  ✓  │
  │                     │ (state 已释放才拿 action, 不反向)    │      │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ controlModeCallback │ action(⊃recovery内 state⊃filter)   │  ✓  │
  │                     │ 释放后单独: state→释放               │  ✓  │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ handleAccepted      │ state→释放, state→释放, action→释放 │  ✓  │
  │                     │ (全部独立, 不嵌套)                   │      │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ gripperCB           │ gripper→释放 (孤立)                 │  ✓  │
  ├─────────────────────┼─────────────────────────────────────┼──────┤
  │ parametersCB        │ filter→释放 (孤立)                  │  ✓  │
  └─────────────────────┴─────────────────────────────────────┴──────┘
```

#### 和单片机的对比——帮你建立直觉

```
  ┌─────────────────────────────────────────────────────────────────┐
  │                    单片机 (STM32)                                │
  │                                                                 │
  │   main loop (while 1)          中断 (ISR)                      │
  │   ─────────────────           ──────────                       │
  │   读传感器                     编码器中断 → 更新位置             │
  │   PID 计算                     定时器中断 → 触发 PID            │
  │   写 PWM                       UART 中断 → 收数据               │
  │                                                                 │
  │   "锁" = __disable_irq()      优先级抢占 = 自然排序             │
  │   只有两级: 中断 vs 非中断      没有死锁 (单向抢占)              │
  └─────────────────────────────────────────────────────────────────┘
                          ↕ 对应关系 ↕
  ┌─────────────────────────────────────────────────────────────────┐
  │                    ROS2 (torque_controller)                     │
  │                                                                 │
  │   controlLoop (200Hz timer)    jointStateCallback (subscriber)  │
  │   ─────────────────────       ──────────────────────────       │
  │   拷贝传感器数据 (锁)          收到 joint_states → 更新数据     │
  │   PID + 动力学计算 (无锁)      Kalman 滤波 (锁)                 │
  │   发布力矩 (无锁)                                               │
  │                                                                 │
  │   "锁" = mutex                 "中断" = ROS2 回调调度            │
  │   但可以有多级嵌套              必须手动管理顺序                  │
  │   → 需要 "单行道" 规则         → 否则死锁                       │
  └─────────────────────────────────────────────────────────────────┘

  核心区别:
  ┌──────────────────┬────────────────────┬───────────────────────┐
  │                  │ 单片机              │ ROS2 多线程            │
  ├──────────────────┼────────────────────┼───────────────────────┤
  │ 共享数据保护      │ 关中断 (全局)       │ mutex (细粒度)         │
  │ 保护粒度          │ 全有或全无          │ 每个数据域一把锁        │
  │ 死锁可能          │ 不可能 (单核)       │ 嵌套拿锁时可能          │
  │ 防死锁方法        │ 不需要              │ 锁排序 (单行道)         │
  │ 性能影响          │ 关中断=丢数据       │ 持锁太久=阻塞其他工人   │
  │ 你的 copy-out    │ ≈ 中断里快速拷到    │ 锁里快速拷到局部变量,   │
  │                  │ 全局 buffer          │ 锁外慢慢算              │
  └──────────────────┴────────────────────┴───────────────────────┘

  你在单片机里的习惯:
    __disable_irq();
    my_copy = sensor_data;   // 快速拷贝
    __enable_irq();
    result = compute(my_copy);  // 慢慢算

  在这个代码里完全对应:
    {
      std::lock_guard<std::mutex> lock(state_mutex_);  // ≈ 关中断
      q_copy = q_actual_;                              // ≈ 快速拷贝
    }                                                  // ≈ 开中断
    tau = computeDynamics(q_copy);                     // ≈ 慢慢算
```

### 2.2 mujoco_interface_node — 1 把锁

```
线程入口:
  ╔════════════════╗  ╔════════════════╗  ╔═══════════════════╗
  ║ simulationStep ║  ║ effortCallback ║  ║ renderLoop        ║
  ║ (200Hz timer)  ║  ║ (subscriber)   ║  ║ (std::thread)     ║
  ╚═══════╤════════╝  ╚═══════╤════════╝  ╚═══════╤═══════════╝
          │                   │                    │
          ▼                   ▼                    ▼

  ┌─────────────────────────────────────────────────────────────┐
  │                      sim_mutex_                              │
  │  保护: model_, data_ (MuJoCo 物理状态)                       │
  └─────────────────────────────────────────────────────────────┘

  simulationStep:
  ┌────────────────────────────────────────────┐
  │  sim_mutex_ ──→ 释放                       │
  │  (applyMagnetForces + mj_step, ~5ms)       │
  └────────────────────────────────────────────┘

  jointStateCallback (数字孪生模式):
  ┌────────────────────────────────────────────┐
  │  sim_mutex_ ──→ 释放                       │
  │  (更新 qpos + mj_forward, <10μs)          │
  └────────────────────────────────────────────┘

  effortCallback:
  ┌────────────────────────────────────────────┐
  │  ⚠ 无锁! 直接写 data_->ctrl[]             │
  │  (数据竞争风险, 但实践中影响可忽略)         │
  └────────────────────────────────────────────┘

  renderLoop (渲染线程):
  ┌────────────────────────────────────────────┐
  │  sim_mutex_ ──→ 释放 (mjv_updateScene)     │
  │  ... OpenGL 渲染 (无锁) ...                │
  │  sim_mutex_ ──→ 释放 (snapshot 数据拷贝)   │
  │  ... ImGui 绘制 (无锁) ...                 │
  └────────────────────────────────────────────┘
  两次获取是顺序的, 不是嵌套 ✓
```

### 2.3 hardware_interface_node — 3 把锁

```
线程入口:
  ╔═══════════════╗  ╔═══════════════╗  ╔════════════════╗
  ║ sendLoop      ║  ║ receiveLoop   ║  ║ torqueCallback ║
  ║ (200Hz timer) ║  ║ (std::thread) ║  ║ (subscriber)   ║
  ╚═══════╤═══════╝  ╚═══════╤═══════╝  ╚═══════╤════════╝
          │                  │                   │
          ▼                  ▼                   ▼

  ┌───────────────────────────────────────────────────────┐
  │  serial_mutex_      保护: serial_port_ 跨线程操作      │
  ├───────────────────────────────────────────────────────┤
  │  torque_cache_mutex_ 保护: cached_torques_[7]         │
  ├───────────────────────────────────────────────────────┤
  │  data_mutex_        保护: current_positions/velocities │
  └───────────────────────────────────────────────────────┘

  sendLoop (200Hz):
  ┌──────────────────────────────────────────────────┐
  │  serial    ──→ 释放  (检查端口状态)               │
  │  torque    ──→ 释放  (读取 6 轴力矩)             │
  │  (构建数据包, 无锁)                               │
  │  serial    ──→ 释放  (sendRaw 发送, 内含 lambda)  │
  │  torque    ──→ 释放  (读取夹爪力矩)              │
  │  serial    ──→ 释放  (sendRaw 发送夹爪包)         │
  └──────────────────────────────────────────────────┘
  所有锁均为顺序获取, 从不嵌套 ✓

  receiveLoop (独立线程):
  ┌──────────────────────────────────────────────────┐
  │  循环:                                            │
  │    ensureSerialOpen:                              │
  │      serial ──→ 释放 (检查+重连)                  │
  │      (sleep 在锁外) ✓                             │
  │                                                   │
  │    readExact:                                     │
  │      serial ──→ 释放 (receive 数据)               │
  │                                                   │
  │    processPacket:                                 │
  │      data   ──→ 释放 (更新 joint 数据)            │
  │      (publish, 无锁)                              │
  │                                                   │
  │    异常处理:                                       │
  │      serial ──→ 释放 (close 端口)                 │
  └──────────────────────────────────────────────────┘

  析构函数:
  ┌──────────────────────────────────────────────────┐
  │  serial ──→ swap-out port ──→ 释放               │
  │  (锁外 close + join)  ✓                          │
  └──────────────────────────────────────────────────┘
```

### 2.4 mission_executor_node — 4 把锁 (含内部类)

```
线程入口:
  ╔════════════════════╗  ╔══════════════════╗  ╔═══════════════╗
  ║ Main UI Loop       ║  ║ AsyncTaskRunner  ║  ║ ROS2 Callback ║
  ║ drawUI+handleInput ║  ║ worker thread    ║  ║ (pose, cmd)   ║
  ╚════════╤═══════════╝  ╚════════╤═════════╝  ╚═══════╤═══════╝
           │                      │                     │
           ▼                      ▼                     ▼

  ┌────────────────────────────────────────────────────────────┐
  │  status_mu_     保护: states_[], current_idx_,             │
  │                       trajectories_[], traj_page_          │
  ├────────────────────────────────────────────────────────────┤
  │  pose_mu_       保护: current_ee_pose_, has_ee_pose_       │
  ├────────────────────────────────────────────────────────────┤
  │  LogBuffer::mu_ 保护: logs_ 环形日志缓冲                    │
  ├────────────────────────────────────────────────────────────┤
  │  AsyncTask::mu_ 保护: tasks_ 任务队列 + CV                  │
  └────────────────────────────────────────────────────────────┘

  Main UI Loop (~10Hz):
  ┌──────────────────────────────────────────────────┐
  │  drawUI():                                        │
  │    status_mu_ ──→ (读状态机, 轨迹列表)            │
  │                ──→ pose_mu_ ──→ 释放 (读末端位姿) │
  │                ──→ 释放                           │
  │    LogBuffer::mu_ ──→ 释放 (读日志)               │
  │                                                   │
  │  handleInput():                                   │
  │    (无锁, 通过 atomic executing_ 做互斥)           │
  │    调用 executeTrajectoryByKey / executeTraj 等    │
  │    → compare_exchange_strong(executing_)           │
  │    → async_.post() → AsyncTask::mu_               │
  └──────────────────────────────────────────────────┘

  AsyncTaskRunner worker:
  ┌──────────────────────────────────────────────────┐
  │  AsyncTask::mu_ (unique_lock + CV wait)           │
  │  ──→ 释放 (取出 task)                             │
  │  执行 task:                                       │
  │    ├─ future.wait_for(30s)                        │
  │    ├─ status_mu_ ──→ 释放 (更新 current_idx_)    │
  │    └─ executing_ = false                          │
  └──────────────────────────────────────────────────┘

  ROS2 Callbacks:
  ┌──────────────────────────────────────────────────┐
  │  pose_sub CB:                                     │
  │    pose_mu_ ──→ 释放 (写末端位姿)                 │
  │                                                   │
  │  cmd_sub CB (onTaskCommand):                      │
  │    status_mu_ ──→ 释放 (更新 current_idx_)        │
  └──────────────────────────────────────────────────┘
```

### 2.5 trajectory_manager_node — 2 把锁

```
  ┌────────────────────────────────────────────────────────────┐
  │  state_mutex_       保护: current_position_[6],            │
  │                           joint_state_received_            │
  ├────────────────────────────────────────────────────────────┤
  │  trajectory_mutex_  保护: last_trajectory_,                │
  │                           has_last_trajectory_             │
  └────────────────────────────────────────────────────────────┘
  两把锁从不嵌套, 从不交叉, 各自只在单个回调中使用 ✓
```

---

## 3. 全局锁依赖图 (死锁分析)

```
                  torque_controller 内部锁序
                  ═══════════════════════════

                    ┌──────────┐
    ┌──────────────→│ action   │──────────────────────┐
    │               └────┬─────┘                      │
    │                    │ (executionRecoveryCeremony  │
    │                    │  在持有 action 时调用)       │
    │                    ▼                             │
    │               ┌──────────┐                      │
    │               │ state    │                      │
    │               └────┬─────┘                      │
    │                    │ (仅在 executionRecovery     │
    │                    │  Ceremony 内嵌套)            │
    │                    ▼                             │
    │               ┌──────────┐                      │
    │               │ filter   │                      │
    │               └──────────┘                      │
    │                                                  │
    │  jointStateCallback:                             │
    │    state(unique_lock)                            │
    │    ⊃ filter ──→ 释放                             │
    │    state.unlock()                                │
    │    action ──→ 释放    ← 不构成反向              │
    │                        (state 已释放)             │
    │                                                  │
    │  controlLoop:                                    │
    │    state ──→ 释放                                │
    │    action ──→ 释放    ← 不构成反向              │
    │                        (state 已释放)             │
    │                                                  │
    └─ gripper (永远最后, 从不与其他锁嵌套) ◄──────────┘


  全局统一锁序:   action ──→ state ──→ filter ──→ gripper
                  (高)        (中)       (低)      (最低)

  ✓ 无环路 = 无死锁
```

### 跨线程锁争用热点

```
  Thread A (controlLoop 200Hz)     Thread B (jointStateCallback 200Hz)
  ──────────────────────────       ────────────────────────────────
  state_mutex_ (copy-out, <10μs)   state_mutex_ (update, <50μs)
       ↕ 争用                           ↕ 争用
  action_mutex_ (copy-out, <5μs)   action_mutex_ (首次, 仅1次)

  最坏延迟: controlLoop 等待 jointStateCallback 释放 state ≈ 50μs
  控制周期: 5000μs (5ms @ 200Hz)
  占比: < 1%, 无实际影响 ✓
```

---

## 4. 跨节点通信拓扑 (无锁, 通过 ROS2 话题/服务解耦)

```
  ┌──────────────────┐  /joint_states     ┌─────────────────────┐
  │ mujoco_interface │ ═══════════════════→│ torque_controller   │
  │ (或 hardware_if) │                     │                     │
  │                  │ /effort_controller  │                     │
  │                  │◄═══════════════════ │                     │
  └──────────────────┘  /commands          └──────────┬──────────┘
                                                      │
                                              /ARM_controller/
                                           follow_joint_trajectory
                                              (Action)
                                                      ↑
  ┌───────────────────┐  /load_trajectory  ┌──────────┴──────────┐
  │ mission_executor  │ ═════════════════→ │ trajectory_manager  │
  │                   │  (Service)          │                     │
  │                   │ /control_mode       │                     │
  │                   │ ═══════════════════→│ torque_controller   │
  │                   │  (Topic)            │ (subscriber)        │
  └───────────────────┘                    └─────────────────────┘

  节点间通信全部通过 ROS2 DDS, 自带序列化+线程安全 ✓
  不存在跨节点共享内存或直接指针传递
```

---

## 5. 关键不变量 (Invariants)

| 编号 | 不变量 | 违反后果 |
|:----:|--------|---------|
| I1 | `action_mutex_` 持有时可获取 `state_mutex_`, 反之绝不 | 死锁 |
| I2 | `state_mutex_` 持有时可获取 `filter_mutex_`, 反之绝不 | 死锁 |
| I3 | `gripper_mutex_` 永远单独获取, 不与任何锁嵌套 | 死锁 |
| I4 | `controlLoop` 从不在持锁时调用 `emergencyStop` | 死锁 (已通过 copy-out 修复) |
| I5 | `jointStateCallback` 在获取 `action_mutex_` 前必须先 `unlock` state | 死锁 |
| I6 | 200Hz 控制循环内无阻塞 I/O, 无动态分配, 无 sleep | 实时性破坏 |
| I7 | `hardware_interface` 析构时锁内仅 swap, 锁外 close+join | 析构死锁 |
| I8 | `ensureSerialOpen` 的 sleep 在锁外 | 全系统串口阻塞 |
