# AI 助手开发规范 - ARV_V1 机械臂项目

## 📌 基本规则

- 构建工具: `colcon build`，工作空间 `~/ros2_ws`
- 启动脚本: `./start_mujoco_system.sh`
- **禁止生成额外文件** (md/sh/py/cpp) 除非明确允许

## 🚫 禁止事项

1. **不要修改 CPP 解算相关代码！**
   - `torque_controller_node.cpp`
   - `dynamics_computer.cpp`
   - `cascade_pid.cpp`
   - `kalman_filter.cpp`

2. **不要随意创建文件**
   - 不生成额外 markdown 文档
   - 不生成 shell 脚本
   - 不生成 python/cpp 文件

## ✅ 推荐行为

1. **解释逻辑思路** - 每次行动说清楚为什么这样做，便于学习
2. **优先编辑现有文件** - 不创建新文件
3. **小步修改** - 单次修改控制在 150 行以内

## 🏗️ 项目架构

```
/home/wuhuan/repositories/src/
├── ARV_V1_MODEL/          # URDF模型
├── ARV_V1_MOVEIT/         # 核心控制包
│   ├── src/               # C++源码 (禁止随意修改)
│   ├── config/            # 配置文件
│   └── launch/            # 启动文件
├── docs/                  # 文档目录
│   ├── TODO_KDL.md        # 技术文档
│   ├── VISION_GRASP.md    # 视觉方案
│   ├── VISION_LEARNING.md # 学习路径
│   └── ARCHITECTURE_RT.md # RT架构
└── 启动脚本
    ├── start_mujoco_system.sh
    ├── stop_all_nodes.sh
    └── reload_params.sh
```

## 📋 关键节点 (只读参考)

| 节点 | 频率 | 功能 |
|------|------|------|
| torque_controller | 200Hz | 力矩控制 |
| hardware_interface | 200Hz | 串口通信 |
| mujoco_interface | 200Hz | 仿真/孪生 |

## 📚 文档位置

- 技术文档: `docs/TODO_KDL.md`
- 视觉方案: `docs/VISION_GRASP.md`
- 学习指南: `docs/VISION_LEARNING.md`
- RT架构: `docs/ARCHITECTURE_RT.md`

---

**最后更新**: 2026-01-09
