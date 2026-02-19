# 兑矿站仿真系统设计文档

> 日期: 2026-02-19 | 分支: feature/pick_ores

---

## 1. 背景与需求

### 1.1 赛场任务

RoboMaster 工程机器人需完成：
- **取矿**：从取矿框架上拔出能量单元
- **兑矿**：将能量单元送入兑矿站的可活动棒子上

### 1.2 物理模型

```
取矿框架 (固定):
  六边形体 + 6根棒子 (Φ38×75mm, 60°间隔, Φ450外径)
  每根棒子上磁吸1个能量单元 (~10N)

兑矿站:
  独立可活动棒子 (slide + hinge, 被能量单元间接驱动)

操作流程:
  夹爪夹住能量单元 → 沿轴向拔出(克服重力+10N磁力) → 送到兑矿站棒子
```

### 1.3 涉及组件

| 组件 | 性质 | 碰撞需求 |
|------|------|----------|
| 机械臂 (link1-5) | 受控运动 | 无 (MoveIt管避障) |
| 夹爪 | 受控末端 | 需与能量单元接触 |
| 能量单元 (×6) | 自由体, 带孔 | 碰棒子+夹爪+地面 |
| 取矿框架 + 棒子 | 固定体 | 碰能量单元 |
| 兑矿站棒子 | 受约束活动体 | 碰能量单元 |

---

## 2. 核心架构矛盾与解决

### 2.1 现有架构的碰撞分工

| 层 | 职责 | 碰撞 |
|----|------|------|
| **MoveIt** | 运动规划 + 避障 | ✅ PlanningScene 几何碰撞检测 |
| **MuJoCo** | 力矩仿真 + 可视化 | ❌ 全禁用 (`nconmax=0`, `contype=0`) |
| **torque_controller** | 动力学前馈+PID | 不关心碰撞 |

**现有禁用碰撞的代码**:
- `mujoco_interface_node.cpp` L264: `<size nconmax="0" njmax="0"/>`
- `mujoco_interface_node.cpp` L335-346: 循环给所有 geom 注入 `contype="0" conaffinity="0"`

### 2.2 为什么 MoveIt 碰撞不够

MoveIt 碰撞检测是**规划时的布尔判断**（"这条路径会不会撞到？"），无法做：
- 接触力传递 — 夹着能量单元推棒子
- 约束耦合运动 — 能量单元套在棒上滑动
- 摩擦/滑移 — 从棒上拔出能量单元

MoveIt 说"别碰"，兑矿站需要"碰，用合适的力碰"。

### 2.3 解决方案：分组碰撞 (Selective Collision)

**不需要全局开碰撞**。利用 MuJoCo 的 `contype`/`conaffinity` bitmask 分组：

```
碰撞规则: geom A 与 geom B 碰撞 ⟺ (A.contype & B.conaffinity) || (B.contype & A.conaffinity)
```

| 组 | 体 | contype | conaffinity | 碰谁 |
|----|-----|---------|-------------|-------|
| 0 | 机械臂 link1-5 | `0` | `0` | 谁都不碰 |
| 1 | 地面 | `2` (bit1) | `2` | 碰掉落的能量单元 |
| 2 | 取矿框架棒子 | `4` (bit2) | `4` | 碰能量单元 |
| 3 | 能量单元 | `4` (bit2) | `14` (bit1+2+3) | 碰棒子+地面+夹爪 |
| 4 | 夹爪 | `8` (bit3) | `8` | 碰能量单元 |
| 5 | 兑矿站棒子 | `4` (bit2) | `4` | 碰能量单元 |

**对现有系统的影响：零侵入**
- 机械臂 link1-5 保持 `contype=0`，力矩回路纯净
- `torque_controller` 只读 `data_->qpos[0..5]`，障碍物 freejoint qpos 在更高索引
- MoveIt PlanningScene 独立于 MuJoCo
- 200Hz 仿真新增 ~20 接触对，MuJoCo 轻松处理

---

## 3. MuJoCo 关键认知

### 3.1 运行时拓扑不可变

MuJoCo 的 body 树、joint、tendon 在 `mj_loadXML` 时固定，运行时不能增删。
"套在棒上 → 拔出变自由体" **不能**通过 "删除 slide joint + 添加 freejoint" 实现。

### 3.2 equality constraint 可运行时开关

`data_->eq_active[i]` 可在运行时开关 equality constraint（0=关闭, 1=激活）。
这是官方推荐方式，用于模拟磁吸连接/断开。

### 3.3 凹面体碰撞限制

MuJoCo 碰撞只支持凸体。带孔的能量单元 STL 直接做碰撞 → 凸包 → 孔消失。
需要用 guide geoms（多个小 capsule 围成圈）近似内孔壁。

---

## 4. 建模方案：weld 约束 + guide geoms 碰撞导向

### 4.1 设计原则

```
磁吸力 10N   → weld equality constraint (通过 solref 调硬度)
不穿透/导向   → guide geoms 碰撞 (contype/conaffinity 分组)
夹取连接      → weld equality constraint (动态 eq_active)
初始定位      → weld 保证精确 + 碰撞保证物理合理
拔出滑动      → guide geoms 沿 bar 的碰撞导向
```

### 4.2 能量单元建模 — guide geoms 近似内孔

不用完整 mesh 做碰撞, 在能量单元 body 内部放 4 个不可见 capsule 围成内孔:

```
      俯视图 (能量单元内部)

         ╭── capsule_0
        ╱
  ●   ●       ← 4个capsule围成内径
  │ ○ │       ← ○是棒子截面 (Φ38)
  ●   ●
        ╲
         ╰── capsule_3

capsule 内切圆直径 ≈ 能量单元内孔直径 (略大于Φ38)
棒子外径 < 内切圆直径 → 有间隙 → 可滑动
偏移时被 capsule 挡住 → 歪拔会卡
```

MJCF 示例:
```xml
<body name="energy_unit_0" pos="...">
  <freejoint name="fj_eu_0"/>

  <!-- 外观: mesh (仅可视化) -->
  <geom name="eu0_visual" type="mesh" mesh="energy_unit"
        contype="0" conaffinity="0" rgba="1 0.84 0 0.9"/>

  <!-- 碰撞: 4个 guide capsule 近似内孔壁 -->
  <geom name="eu0_guide_0" type="capsule" size="0.003 0.03"
        pos="0.019 0 0" euler="0 90 0"
        contype="4" conaffinity="4" rgba="1 0 0 0" group="2"/>
  <geom name="eu0_guide_1" type="capsule" size="0.003 0.03"
        pos="-0.019 0 0" euler="0 90 0"
        contype="4" conaffinity="4" rgba="1 0 0 0" group="2"/>
  <geom name="eu0_guide_2" type="capsule" size="0.003 0.03"
        pos="0 0.019 0" euler="0 90 0"
        contype="4" conaffinity="4" rgba="1 0 0 0" group="2"/>
  <geom name="eu0_guide_3" type="capsule" size="0.003 0.03"
        pos="0 -0.019 0" euler="0 90 0"
        contype="4" conaffinity="4" rgba="1 0 0 0" group="2"/>

  <!-- 外部碰撞面 (被夹爪接触) -->
  <geom name="eu0_grip" type="cylinder" size="0.04 0.035"
        contype="4" conaffinity="12" rgba="1 0.84 0 0" group="2"/>
</body>
```

### 4.3 取矿框架建模

```xml
<!-- 固定体: 六边形 + 6根棒子 -->
<body name="ore_frame" pos="0.5 0 0.3">
  <geom type="mesh" mesh="hex_body" contype="0" conaffinity="0"/>  <!-- 仅可视化 -->
  <!-- 6根棒子, 每隔60° -->
  <geom name="bar_0" type="cylinder" size="0.019 0.0375"
        pos="0.225 0 0" euler="0 90 0"
        contype="4" conaffinity="4" friction="0.3 0.005 0.001"/>
  <geom name="bar_1" type="cylinder" size="0.019 0.0375"
        pos="0.1125 0.1948 0" euler="0 90 60"
        contype="4" conaffinity="4" friction="0.3 0.005 0.001"/>
  <!-- ... bar_2 ~ bar_5 -->
</body>
```

### 4.4 磁吸约束

```xml
<equality>
  <weld name="mag_eu_0" body1="energy_unit_0" body2="ore_frame"
        anchor="0.225 0 0" relpose="0 0 0 1 0 0 0"
        solref="0.02 1" solimp="0.9 0.95 0.001"/>
  <!-- ... 6个 -->
</equality>
```

### 4.5 夹爪抓取约束

```xml
<equality>
  <weld name="grasp_eu_0" body1="energy_unit_0" body2="gripper_link"
        active="false"/>  <!-- 初始不激活 -->
  <!-- ... 6个 -->
</equality>
```

### 4.6 兑矿站棒子

```xml
<body name="exchange_station" pos="-0.5 0 0.3">
  <geom type="box" size="0.05 0.05 0.15" contype="0" conaffinity="0"/>
  <body name="exchange_bar">
    <joint name="ex_bar_slide" type="slide" axis="0 0 1"
           range="-0.1 0.1" damping="5.0" frictionloss="2.0"/>
    <joint name="ex_bar_rotate" type="hinge" axis="0 0 1"
           damping="1.0" frictionloss="0.5"/>
    <geom type="cylinder" size="0.019 0.0375"
          contype="4" conaffinity="4"/>
  </body>
</body>
```

---

## 5. 运行时状态机

### 5.1 能量单元状态

```
SEATED (初始)                    EXTRACTING (拔出中)
├─ bar_weld: active              ├─ bar_weld: INACTIVE
├─ guide碰bar: 有接触            ├─ guide碰bar: 沿轴滑动
├─ grasp_weld: inactive          ├─ grasp_weld: active
│                                │
│  夹爪夹住+拉力>10N             │  接触数=0(离开棒子)
│  ──────────────────→           │  ──────────────────→
                                 
HELD (持有)                      PLACED / FREE
├─ bar_weld: inactive            ├─ grasp_weld: inactive
├─ guide碰bar: 无接触            ├─ freejoint自由
├─ grasp_weld: active            │
│                                │
│  送到兑矿站,释放               │
│  ──────────────────→           │
```

### 5.2 C++ 运行时逻辑 (simulationStep 中)

```cpp
// 磁吸脱离检测
for (int i = 0; i < 6; i++) {
    if (!eu_attached_to_bar_[i]) continue;
    double pull_force = computePullForce(i);
    if (pull_force > MAGNETIC_FORCE_THRESHOLD) {  // > 10N
        data_->eq_active[eq_index_bar_[i]] = 0;
        eu_attached_to_bar_[i] = false;
    }
}

// 夹爪抓取检测
if (gripper_closing_ && !holding_eu_) {
    for (int c = 0; c < data_->ncon; c++) {
        int g1 = data_->contact[c].geom1;
        int g2 = data_->contact[c].geom2;
        int eu_id = identifyEnergyUnit(g1, g2);
        if (eu_id >= 0) {
            mjtNum force[6];
            mj_contactForce(model_, data_, c, force);
            if (force[0] > GRASP_FORCE_THRESHOLD) {
                data_->eq_active[eq_index_grasp_[eu_id]] = 1;
                holding_eu_ = true;
                held_eu_id_ = eu_id;
            }
        }
    }
}
```

---

## 6. 需修改的代码清单

| 文件 | 修改 |
|------|------|
| `mujoco_interface_node.cpp` L264 | `nconmax=0` → `nconmax=200 njmax=100` |
| `mujoco_interface_node.cpp` L335-346 | 碰撞禁用循环改为**分组注入** |
| `mujoco_interface_node.cpp` buildObstacleMJCF() | 注入 equality constraints + guide geoms |
| `mujoco_interface_node.cpp` simulationStep() | 新增约束管理逻辑 |
| `scene_obstacles.yaml` | 新增取矿框架 + 能量单元 + 兑矿站配置 |
| `arv_v1_model/urdf/obstacles/` | 新增 STL + URDF 文件 |

---

## 7. 实施步骤

```
Step 1: STL 导入 + 独立 MJCF 测试
├─ STL 放入 arv_v1_model/meshes/obstacles/
├─ 写独立 test_scene.xml (纯 MuJoCo, 不经 ROS)
├─ 验证: 加载显示、碰撞分组、freejoint 物理
└─ 用 mujoco simulate 可执行文件测试

Step 2: 磁吸约束 + 拔出测试
├─ 加入 equality weld 约束
├─ MuJoCo GUI perturbation 手动施力测试脱离
├─ 调参: solref/solimp, 磁力阈值
└─ 验证拔出手感

Step 3: 集成 mujoco_interface_node
├─ 修改 loadMuJoCoModel(): nconmax, 分组碰撞
├─ 修改 buildObstacleMJCF(): equality + 碰撞属性
├─ simulationStep() 加入约束管理
└─ 验证 200Hz 无掉帧, ROS2 回路正常

Step 4: 夹爪 + 完整流程
├─ 夹爪 geom 碰撞启用 (contype=8)
├─ 抓取/释放状态机
├─ 取矿→持有→兑矿 全流程
└─ 与 trajectory_manager 联调
```

---

## 8. 待确认参数

| 参数 | 说明 | 状态 |
|------|------|------|
| 能量单元内孔直径 | 决定 guide geom 布局, 应略大于 Φ38 | ❓待确认 |
| 夹爪模型 | 独立 STL/URDF 还是简单几何近似 | ❓待确认 |
| STL 坐标系 | 原点位置、单位 (mm/m) | ❓待确认 |
| 兑矿站棒子行程 | slide range, hinge range | ❓待确认 |
| MuJoCo 版本 | 影响 SDF 等高级特性可用性 | ❓待确认 |
