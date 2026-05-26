# ARV_V1 项目交接文档

> 写给:**接手 ARV_V1 但没参与过开发的下一任**.
> 不是 spec, 不是 API doc — 是"我希望当年开始时有人塞给我的那个文件".
> 看 git log / 看代码能搞清"是什么"; 这里只写 **"为什么"** 和**"哪里能踩坑"**.
>
> **作者**: Young-Yihang (毕设 + RoboMaster 2026 项目主) + Claude Opus 4.7 (2026-05-20 维护性 pass)
> **状态**: 比赛封档 snapshot, master = `b77a227`

---

## 0. 30 秒速览

```text
PC (上位机)                          MCU (下位机, 锁版本)            一体化关节模组
┌──────────────────┐                ┌──────────────────┐         ┌──────────────┐
│  trajectory      │                │                  │  CAN    │  内部电流环   │
│  规划 (MoveIt2)  │    Seasky      │   位置环 PID +   │  ~1kHz  │  ~20-40kHz   │
│       ↓          │   0x0002 24B   │   重力前馈 (CTC) │ ───────>│  (黑盒)      │
│  q_target (rad)  │  ──1kHz 1Mbps──>│       ↓          │         │              │
│  /joint_position │      USB CDC   │   torque cmd     │         │              │
│  _target_to_mcu  │                │                  │         │              │
└──────────────────┘                └──────────────────┘         └──────────────┘
       ↑                                    │
       │  /joint_states (pos+vel) ←─────────┘  Seasky 0x0001 84B
```

**关键认知**: 上位机**不算闭环 PID**, 仅算 trajectory + 把 q_target 转发给 MCU.
MCU 在下面跑实际的位置/速度环, CTC + 重力前馈. 电流环在关节模组内部 (无法触及).

之前的版本曾试过"上位机 1kHz 算力矩", 跟踪效果远不如 MCU 自跑闭环. **不要再走回头路**.

---

## 1. Decision Log — 为什么是这样不是另一种

### 1.1 上下位机职责: PC 算 q_target, MCU 跑闭环 (`route_mode=true`)

**Why**: PC 即使 RT-PREEMPT, ROS DDS + USB CDC 链路 jitter 加起来仍远大于 MCU 直驱电机的能力. 1kHz 闭环在 PC 上的实测跟踪误差比 MCU 闭环差一个数量级.

**生产路径**: torque_controller_node 内 `route_mode_=true`, computeFeedforwardTorque / cascade_pid / kalman 全部 bypass. **代码保留是因为受保护文件不便删**, 不是 fallback — 比赛绝不能 `route_mode:=false` 切回去.

**Where**: `src/control/torque_controller_node.cpp` 看 `routeTargetToHardware()` (line ~1230 附近), 顶端有 banner 说明.

### 1.2 Topic / 协议字段命名: `effort` → `position_target`

**Why**: 2026-05-20 之前, route_mode 路径上发出去的 topic 叫 `/effort_controller/commands`, Seasky 字段叫 `TorqueCommand`, 字面上是"力矩", 实质载的是 `q_target (rad)`. 新人看名字会以为是 effort 闭环, 误导极大.

**现在**:
- ROS topic: `/joint_position_target_to_mcu` (Float64MultiArray, 7 元素)
- Seasky 协议: `CMD_JOINT_POSITION_TARGET = 0x0002`, `struct JointPositionTarget { positions[6] }`
- **CmdID 0x0002 字节布局严格不变** (6×float = 24B), MCU 固件不感知 PC 端名字

**Where**: `src/interfaces/serial_protocol.hpp:18-30` CmdID 块有 banner 解释 "byte-level frozen; 名字反映 PC 端语义".

### 1.3 GripperControl srv 字段: `torque` → `force`

**Why**: 夹爪是 prismatic joint, 单位本来就是 N (不是 Nm). 字段名是历史误导.

**Where**: `arv_v1_interfaces/srv/GripperControl.srv`. ABI 变化, 任何依赖此 srv 的 client/server 必须 clean rebuild.

### 1.4 `/joint_position_target_to_mcu` 必须 7 元素不能 6

**Why**: `data[0..5]` 是 6 关节 q_target (rad), `data[6]` 是 gripper 三态信号:
- `> +0.1` → GRIP
- `< -0.1` → RELEASE
- 否则 → STOP

`hardware_interface_node` 拿 `data[6]` 转换成 Seasky 0x0004 的离散 enum 发给 MCU. **改成 6 元素等于改协议**.

### 1.5 ROS discovery 必须限定 LOCALHOST (`start_mujoco_system.sh`)

**Why**: 这个项目是 PC 和 NUC 双部署 ("PC↔NUC 交替使用 MCU"). 同子网 WiFi 下, ROS2 默认 `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET` 会让 fastdds multicast 跨主机自动 discover. 结果 PC 上看到 NUC 上的 publisher (ghost participant), 双发 cmd → MCU 收到方波 → J4/J5 抽搐.

**症状**: `ros2 node list` 同名 node 出现 2 次, `/joint_position_target_to_mcu` publisher count=2, 但只有 1 个 hardware_interface 进程.

**Fix**: `start_mujoco_system.sh` 的 `setup_environment()` 强制:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 daemon stop && ros2 daemon start  # 必须重启 daemon, 旧 discovery 缓存会持续
```

**调试模板**: 任何"双 publisher / 关节抽搐"问题, 第一件事 `ros2 node list | sort | uniq -c | awk '$1>1'`.

### 1.6 SRDF ACM 扩 17 对碰撞豁免 (commit `dc342b3`)

**Why**: URDF mesh 是 CAD 导出的近似几何, link 之间的紧邻装配在 mesh 层会有亚毫米级穿透. PTP joint-space 插值时, 中间点的 collision check 会假阳性触发 self-collision 报错.

**审计依据**: 17 对都是物理验证过"永不可能真撞"的相邻 link (J2↔J3 housing, gripper finger ↔ palm 等), 在 SRDF 顶部注释里有完整列表 + reason.

**Danger**: **换硬件 (新臂几何) 必须重新做扫描验证**, 这 17 对清单是为当前 arv_v1.urdf 标定的, 不可移植.

**Where**: `config/ARV_V1_MODEL.srdf` 顶端注释块.

### 1.7 ComposeTrajectory 段间速度归零是 feature 不是 bug

**Why**: 比赛流程里 waypoint 不是"经过的中间路径点", 而是**取/存矿的精确对准点**. 段间不停顿, 夹爪闭合时机会漂.

**实现**: 后端用 E2 (分段 plan + 拼接), MoveIt 每段 plan 默认终点 v=0, 拼接后段间自然停顿.

**不要做的**:
- 不要用 Pilz MotionSequence 做"段间速度连续"
- 不要给 ComposeTrajectory 加 `smooth_blend` / `via_radius`
- 不要"优化"掉停顿

**Where**: `arv_v1_interfaces/srv/trajectory_manager/ComposeTrajectory.srv` header 注释.

### 1.8 夹爪动作锚 action 起跑时刻, 不是 trajectory header.stamp

**Why**: `follow_joint_trajectory` action 排队后实际 start 会延迟数百 ms (controller 接受 + 调度). 用 `header.stamp` 作为时间锚, 夹爪动作会和实际轨迹错位.

**Fix**: `mission_executor::scheduleGripperActions` 用 `steady_clock` 在 action 真实起跑时刻打基准. commit `c447ab5` 第一次发现这个 bug.

**Where**: `src/application/mission_executor_node.cpp` `scheduleGripperActions()`.

### 1.9 mission_executor 是 headless C++ FSM, TUI 在 Python sibling

**Why**: C++ 节点不该承担 UI 渲染. 之前 mission_executor_node.cpp 1325 行里 400+ 行是 ncurses TUI, AsyncTaskRunner 只是为了不阻塞 ncurses refresh, 笛卡尔 jogging 嵌在"mission"节点里 — 全是反向耦合.

**现在**: C++ 端只有 FSM 和 service client; UI 在 `scripts/mission_panel.py` 用 tkinter / customtkinter.

### 1.10 cascade_pid + Kalman + 阻抗代码保留但 bypass

**Why**: 这些是 `route_mode=false` (上位机闭环力矩控制) 路径的依赖, 已退役但代码留作回滚保险. 受保护文件不便删.

**风险**: 这些代码看起来在跑 (节点会构造它们), 但**生产路径不调用**. 不要因为"参数调不出效果"就去改 cascade_pid 增益 — 在 `route_mode=true` 下根本不生效, 真正影响的是 MCU 端的 PID 增益 (MCU 端代码不在此 repo).

**Where**: `src/core/cascade_pid.*`, `src/core/kalman_filter.*`, `src/control/torque_controller_node.cpp` 内 `computeFeedforwardTorque()`.

### 1.11 scene_obstacles.yaml 保留, scene_manager_node 删除

**Why**: 历史上 scene_manager_node 是从 yaml 加载障碍物到 MoveIt2 planning scene 的独立节点, 但 `start_mujoco_system.sh` 任何模式都不调用它. 后来 `mujoco_interface_node` 直接读 yaml 自己加载到 MuJoCo, scene_manager 成了死代码.

**注意**: yaml 顶层 key 仍是 `scene_manager:` (历史保留), `mujoco_interface_node.cpp` `config["scene_manager"]["ros__parameters"]` 这样读. **不是死引用**, 只是名字奇怪.

### 1.12 1kHz 是 transport 频率, 不是 PID 频率

**Why**: PC 不再跑 PID 闭环, 但 `/joint_position_target_to_mcu` 仍以 1kHz 发送 (transport rate). MCU 端的 PID 用接收到的最新 q_target 在自己时基上跑, 频率不和 PC transport rate 强绑定.

**监控**: `scripts/verify_dataflow_frequency.sh` 检查这个 topic 频率, 是真机关键指标 (掉了说明 transport 链路出问题).

---

## 2. Failure Log — 我们踩过的坑

### 2.1 PC↔NUC ghost publisher → 关节抽搐

参见 [1.5](#15-ros-discovery-必须限定-localhost-start_mujoco_systemsh).

调试耗时 ~2 天. root cause 不在控制律, 在 fastdds discovery. **教训**: 任何"双源" / "幽灵 publisher" / "节点重名"症状, 优先排查跨主机 discovery, 不要先怀疑控制律.

### 2.2 URDF mass 误差 2-7x → 重力前馈算不准

历史上有 3 版 URDF (`arv_v1.urdf` / `arv_v1_old.urdf` / `arv_v1_new.urdf`), 因为 SolidWorks 导出时人手设置过 material density, 不同 link 的质量/惯量误差 2-7x. MCU 端 G 补偿基于 URDF 算, 误差直接传到力矩输出.

**Fix**: commit `45b1151` 恢复 `arv_v1.urdf` canonical, 用 CAD 校验过的密度重新导出. 2026-05-20 pass 删除冗余备份, 顶端注释保留迭代史.

**教训**: SolidWorks URDF Exporter 默认 density 不可信, 每个 link 都要 override 自己的 density. 比较两个 URDF 是否一致用 `diff` 看 inertial block (mass / ixx-izz), 不看 `<visual>`.

### 2.3 cascade_pid vel_Kp=0 → 控制链断开

在尝试做"上位机闭环力矩控制"期间 (`route_mode=false` 历史阶段), 调 cascade PID 时把内环 vel_Kp 改成 0 想"只用外环". 结果内环输出恒为零, 整条控制链断, 力矩输出为 0.

**教训**: cascade PID 的内环不能关. 等价单环 PD 的映射是:
- `Kp(vel) = 1, Kp(pos) = Kp_old, Kd(pos) = Kd_old - 1`

原始单环 PD 参数见 commit `566971e`. 这个等价关系写在 [[memory/Cascade PID 等价]] 里, 仅供历史回溯参考 (生产路径不依赖).

### 2.4 ComposeTrajectory 重录后 `meta.gripper_actions` 丢失

录制完一条新轨迹后, `meta.gripper_actions` 字段没自动从原轨迹拷贝, yaml 里这个字段被清空. 现象: 轨迹能跑但夹爪不动作.

**检测**: 录新轨迹后必须 `grep gripper_actions config/trajectories/*.yaml` 验证, 或在 mission_panel.py 看 trajectory metadata.

### 2.5 高频震荡来源定位需要全链路排查

调试某次 J5/J6 高频震荡, 单看控制律 / 单看波形都看不出来. 最后是 RT-PREEMPT cpu isolation + DDS discovery + USB CDC latency + MCU 端定时器全链路对齐才找到 root cause (跨主机污染, 见 [2.1](#21-pcnuc-ghost-publisher--关节抽搐)).

**教训**: 实时系统的故障定位需要**仪器化**, 不是"在比赛代码里加 printf 看 J5 抖不抖". 建议补:
- 一条 Seasky "诊断模式" CmdID, 让 PC 能注入测试信号 + 高速 dump 回来
- `analyze_latency_timeline.py` / `verify_hardware_loop_latency.sh` 这类延迟链路工具齐用

---

## 3. New Arm Bring-up Runbook

**适用场景**: 你拿到一台新 arv_v1 / 类似拓扑的臂, 要从零让它能跑.

**前置假设**:
- MCU 固件 (CTC PID + G 前馈) 已 flash
- 一体化关节模组已上 CAN 总线
- USB CDC 链路通

**总耗时估计**: 单关节 1-2 天, 全臂 1-2 周.

### Phase 0: 静力学 sanity check (0.5 天)

**目的**: 确认 URDF 与实际几何一致.

**步骤**:
1. 把臂摆成 [0, π/2, 0, 0, 0, 0] (J2 水平), 用拉力计测 J2 在重力下的静扭矩
2. 跑 `python3 scripts/compute_static_torque.py` (TODO: 这个脚本目前还没写) 算 URDF 预测的 J2 扭矩
3. 比较测量 vs 预测, 误差 > 30% → URDF mass/inertia 有错, **必须先改 URDF 才能继续**

**Pass 条件**: 单关节静扭矩误差 < 20%.

### Phase 1: 重力补偿标定 (1 天)

**目的**: 校准 MCU 端的 G(q) 前馈.

**步骤**:
1. 启动系统但 control_mode 设 `RELAX` (零扭矩输出)
2. 手动把臂摆到若干 sample pose (覆盖 J2/J3 工作空间)
3. MCU 端记录每个 pose 下的静扭矩 (跑 G 前馈但 P/D 增益置零)
4. 用最小二乘拟合 mass/length 校准量

**参考脚本**: `scripts/gravity_calibration.py` (注: 该脚本是为 `route_mode=false` 力矩路径设计的, 在当前 `route_mode=true` 路径下需要从 MCU 侧读数据, 不能直接用 PC 侧 topic).

**Pass 条件**: 任意 pose 下, 释放手让臂自由静止, 漂移 < 2°/秒.

### Phase 2: 单关节速度环手推测阻尼 (2-3 天, 逐关节)

**目的**: 调 MCU 端 velocity loop Kp/Kd 让响应稳定不震.

**步骤** (单关节, 锁住其他):
1. control_mode 切 `FREEDRIVE` (仅 G 补偿)
2. MCU 端打开 velocity loop, position loop 关
3. 手推关节, 感受阻尼:
   - 推不动 → vel_Kp 太大
   - 推完反弹 → vel_Kp + Kd 失衡
   - 推完震荡 → 增益太大, 减半
   - 推完慢慢飘 → 阻尼太低, 加 Kd
4. 调到"推动有阻力, 撒手不震不飘"为止

**Pass 条件**: 手推阻尼感觉"扎实", 撒手无可见震荡, 无明显死区.

**Danger**: 调增益前先确认**位置软限位**和**扭矩硬限位**在 MCU 端正确, 否则可能撞机械限位.

### Phase 3: 叠加位置环 (1-2 天, 逐关节)

**目的**: 在已稳定的速度内环上加位置外环.

**步骤**:
1. control_mode 切 `ARMED` (G + 位置闭环)
2. MCU 端打开 position loop, 增益从估计值的 1/10 起
3. 通过 PC 端 `/joint_position_target_to_mcu` 发小幅 step (5°)
4. 观察响应:
   - 上升时间 > 1 秒 → pos_Kp 太小, 加倍
   - 超调 > 20% → pos_Kp 太大或 pos_Kd 太小
   - 稳态误差 > 1° → pos_Ki 加一点 (注意 anti-windup)
5. 调到上升 < 300ms, 超调 < 10%, 稳态 < 0.1°

**Pass 条件**: step response 满足上述指标, 多关节同时小动作无耦合震荡.

### Phase 4: 轨迹执行集成 (1-2 天)

**目的**: 跑完整 MoveIt2 轨迹.

**步骤**:
1. PC 端启动 `mujoco_demo.launch.py` (sim 模式), 跑一条短轨迹, 看 MuJoCo 内是否跟随
2. **重要**: MuJoCo actuator 类型仍是 `<motor>`, 在 `route_mode=true` 下 sim 行为和真机不一致, 仅可信几何/碰撞
3. 切到 hardware deploy, 用 `force_zero_torque:=true` 起步 (扭矩输出为零, 但完整测试通讯链路)
4. 关闭 `force_zero_torque`, 跑小幅运动确认链路正常
5. 全速跑预录轨迹

**Pass 条件**: 跑完一条 30 秒比赛轨迹, 跟踪误差 < 2°, 无 watchdog 触发.

### Phase 5: 比赛级集成 (>=1 周)

教 trajectory + tune gripper schedule + 多次失败录制 + 比赛流程联调. 这部分没有 runbook, 是项目工程.

---

## 4. Danger Zones — 碰前必读

### 4.1 Seasky 0x0002 byte 布局

`CmdID 0x0002` 的 payload 必须是 `6 × float (24B)`, **MCU 固件锁版本不可改**. PC 端可以重命名符号 (struct / 函数 / 字段), 但任何改变字节顺序 / 元素数量 / 类型的修改都会断 MCU 通讯.

验证: `serial_protocol.hpp:185-190` 的 `buildPositionTargetPacket` 必须确保 `append_float` 调 6 次, 顺序与 `NUM_ARM_JOINTS` 对应.

### 4.2 `/joint_position_target_to_mcu` 7 元素布局

`data[6]` 是 gripper 信号, 不是第 7 关节. 任何把数组改成 6 元素的"清理"都会让 gripper 失效.

### 4.3 Lock ordering

`torque_controller_node` 内严格遵守: `action_mutex_ → state_mutex_ → filter_mutex_`. **不可反序**, banner 写在 `controlModeCallback` 处.

### 4.4 受保护文件清单 (2026-05-20 pass 后恢复保护)

- `src/control/torque_controller_node.cpp`
- `src/core/dynamics_computer.cpp/hpp`
- `src/core/cascade_pid.cpp/hpp`
- `src/core/kalman_filter.cpp/hpp`

这些文件改动需要明确理由和测试, 不要顺手"清理"(参见 `CLAUDE.md` 顶部规则).

### 4.5 比赛代码不要切 `route_mode=false`

退役路径, 切回去会发现:
- cascade_pid / Kalman 没在最新硬件上 tune 过
- 上位机 1kHz 力矩控制效果比 MCU 闭环差一截
- MCU 端可能已配置为不接受 Nm 解释

如果**必须**用 route_mode=false (比如要诊断某个 MCU 端 bug), 先在仿真验证, 然后小幅手动测试, 不要直接上比赛场.

### 4.6 SRDF ACM 17 对豁免

换硬件 (新臂 / 新 gripper / 新 mesh) 必须重新做物理碰撞扫描, 不能照搬当前 SRDF 的 17 对.

### 4.7 MuJoCo NaN reset 路径被显式禁用

`mujoco_interface_node.cpp:1043-1055` 的 `mj_resetData` 调用被注释掉 (TODO 待恢复). 比赛走 hardware 不影响, 但**sim 调试时仿真发散不会自动 reset**, 需要手动重启.

---

## 5. 残留 TODO (比赛后处理)

按优先级排:

1. **`mujoco_interface_node.cpp` 1500 行拆三件套** (sim / GLFW renderer / ImGui overlay) — 现在三件混在一个文件 + 一个 cpp 里, 维护成本高
2. **MuJoCo actuator 类型** 改成 `<position>` 让 sim 行为和真机 (route_mode=true) 一致 — 当前 sim 把 q_target 当 torque 喂, sim 路径不可信
3. **`scheduleGripperActions` bounds check** (CodeRabbit 提过) — `commands` 和 `times` 长度不一致时会越界
4. **`mission_executor` 30s 硬超时改 dynamic** — 长轨迹 (> 30s) 会假超时
5. **`trajectory_manager` YAML 缺 key 防护** — 当前 `ga["time"]` / `ga["action"]` 不查存在就读, 缺字段抛 `YAML::InvalidNode`
6. **`safety.max_torque_per_joint` YAML key** 拆 arm/gripper (gripper 应 `max_force`, arm 是 `max_torque`)
7. **mujoco_interface_node:1043-1055 NaN reset** 路径恢复

参见 v3 重写计划 (memory `project_post_competition_refactor.md`).

---

## 6. 外部资源

- **MIT Mini Cheetah Software** (github.com/mit-biomimetics/Cheetah-Software): 同类硬件 (smart joint module + 自写 MCU), 比 Spong 教科书更对口
- **SimpleFOC** (github.com/simplefoc/Arduino-FOC): 级联控制结构的干净参考
- **Kollmorgen / Maxon servo tuning whitepapers**: 工业实战调机手册
- **ARTINX/engineer_2026 repo**: MCU 端固件源 (本 repo 不含)

---

## 7. 紧急联系

**作者**: Young-Yihang

看完这份文档还有不懂的, 直接问我。

---

**END.** 祝下任少踩坑.
