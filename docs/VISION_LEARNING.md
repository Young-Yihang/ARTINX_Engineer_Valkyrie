# 视觉模块学习路径指南

## 核心原则

**独立验证 → 最小集成 → 逐步扩展**

1. 先独立验证，后集成框架
2. 每步都有可运行的demo
3. 模块化隔离，可独立测试

---

## 阶段一：相机选型验证 (2-3天)

### 目标
用最简单的方式验证相机硬件能力，**不用ROS2**

### 方法
```bash
# 纯Python + OpenCV 或 厂商SDK
python3 camera_test.py
```

### 验证清单
| 项目 | 验证方法 | 通过标准 |
|------|----------|----------|
| 图像质量 | 保存10张图手动检查 | 清晰无畸变 |
| 帧率 | 计时1000帧 | 达到标称值 |
| 曝光控制 | SDK设置不同曝光 | 能调节亮度 |
| 红蓝光抗干扰 | 手机红蓝LED照射 | 滤光片有效 |

### 产出物
- `camera_test.py` 能跑通

---

## 阶段二：SDK封装为ROS2节点 (3-5天)

### 目标
把相机变成ROS2话题，**不涉及检测算法**

### 方法
```cpp
// camera_driver_node.cpp
// 只做一件事：发布 /camera/image_raw
```

### 验证方式
```bash
ros2 run your_pkg camera_driver_node
ros2 run rqt_image_view rqt_image_view  # 看到图像=成功
```

### 产出物
- RViz/rqt 能看到实时图像

---

## 阶段三：算法独立验证 (1-2周)

### 目标
用录制的数据或合成数据调试算法，**不启动真实相机**

### 方法
```bash
# 1. 录制真实数据
ros2 bag record /camera/image_raw -o test_bag

# 2. 离线调试
python3 detection_test.py --input test_bag/

# 3. 合成数据训练
python3 train_yolo.py --data synthetic_dataset/
```

### 为什么这样做？
- 不依赖硬件，宿舍也能调算法
- 同样数据可复现，便于对比参数
- 快速迭代，不用等相机启动

### 验证清单
| 项目 | 验证方法 | 通过标准 |
|------|----------|----------|
| 检测精度 | 离线测试100张图 | mAP > 0.9 |
| 推理速度 | 计时1000次推理 | < 30ms |
| 位姿精度 | 对比真值 | 误差 < 5mm |

### 产出物
- 检测准确率达标的模型

---

## 阶段四：算法封装为ROS2节点 (3-5天)

### 目标
把调试好的算法变成ROS2节点

### 方法
```cpp
// detection_node.cpp
// 订阅: /camera/image_raw
// 发布: /detections
// 核心算法直接复用阶段三的代码
```

### 验证方式
```bash
# 用rosbag回放测试（不需要真实相机）
ros2 bag play test_bag/
ros2 run your_pkg detection_node
ros2 topic echo /detections  # 看到检测结果=成功
```

### 产出物
- rosbag回放能正确检测

---

## 阶段五：融入控制框架 (1周)

### 目标
视觉输出 → MoveIt规划 → 抓取

### 节点设计
```
pose_estimation_node
├─ 订阅: /detections, /camera/depth
├─ 发布: /target_pose
└─ 坐标变换: camera_frame → base_link

grasp_action_node
├─ 订阅: /target_pose
└─ 调用: MoveIt action server
```

### 验证顺序
1. **MuJoCo仿真** 中测试
2. **真机 + 数字孪生** 验证
3. 真机实际抓取

### 产出物
- MuJoCo仿真抓取成功

---

## 学习工具

### 必备
| 工具 | 用途 | 优先级 |
|------|------|--------|
| OpenCV | 图像处理基础 | ⭐⭐⭐ |
| rosbag | 录制/回放数据 | ⭐⭐⭐ |
| rqt_image_view | 实时查看图像 | ⭐⭐⭐ |
| Blender | 合成数据渲染 | ⭐⭐ |

### 学习资源
1. **相机SDK**: 厂商官方示例 > 第三方教程
2. **YOLOv8**: Ultralytics 官方文档
3. **手眼标定**: easy_handeye2 GitHub
4. **ROS2图像**: image_transport 官方教程

---

## 关键心得

### 1. 永远先录数据
```bash
ros2 bag record /camera/image_raw /camera/depth -o debug_$(date +%Y%m%d)
```

### 2. 用最简单的方式验证
- 先 Python，后 C++
- 先静态图片，后实时视频
- 先仿真，后真机

### 3. 模块边界清晰
```
相机驱动 (只管采图)
    ↓ /camera/image_raw
检测算法 (只管检测)
    ↓ /detections
位姿估计 (只管转换)
    ↓ /target_pose
运动规划 (只管抓取)
```

---

## 时间总览

| 阶段 | 时间 | 产出物 |
|------|------|--------|
| 相机选型验证 | 2-3天 | camera_test.py |
| SDK封装 | 3-5天 | RViz看到图像 |
| 算法离线验证 | 1-2周 | 模型准确率达标 |
| 算法ROS2集成 | 3-5天 | rosbag回放检测 |
| 框架融入 | 1周 | 仿真抓取成功 |
| **总计** | **4-5周** | |

---

**最后更新**: 2026-01-08
