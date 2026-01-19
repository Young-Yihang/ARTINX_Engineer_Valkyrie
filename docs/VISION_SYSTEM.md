# ARV_V1 视觉系统技术方案

## 一、系统架构概览

### 1.1 核心目标
- **任务**: RoboMaster 2026 哑铃状物体抓取
- **需求**: 6DoF位姿估计用于精确抓取
- **挑战**: 红蓝LED强光干扰(620nm/450nm)、10-10000 lux亮度变化

### 1.2 技术路线
```
相机采集 → 2D检测(YOLOv8) → 深度获取 → 6DoF位姿(FoundationPose/ICP) → MoveIt规划
```

---

## 二、硬件方案

### 2.1 推荐配置
| 设备 | 型号 | 价格 | 关键特性 |
|------|------|------|---------|
| 工业相机 | 海康MV-CE060-10UC | ¥1500 | 全局快门,100fps,抗频闪 |
| 深度相机 | Orbbec Astra+ | ¥800 | 940nm结构光,避开红蓝干扰 |
| 滤光片 | 940nm窄带 | ¥200 | 屏蔽可见光 |
| 补光灯 | 940nm LED环形灯 | ¥300 | 主动光源 |

### 2.2 安装方式
- **Eye-in-Hand**: 相机固定在link6末端
- **手眼标定**: easy_handeye2 (至少15组位姿)

### 2.3 为什么不用RealSense?
- RealSense使用850nm红外，会被红蓝LED严重干扰
- 工业相机+940nm结构光可完全避开620nm/450nm波段

---

## 三、算法实现

### 3.1 检测流水线
```
1. YOLOv8n目标检测 (5ms, mAP>0.9)
   ↓
2. ROI裁剪 + 深度图对齐
   ↓
3. FoundationPose 6DoF估计 (50ms) 或 ICP配准 (20ms)
   ↓
4. 坐标变换 camera_frame → base_link
   ↓
5. 发布 /target_pose 给MoveIt
```

### 3.2 合成数据训练 (Sim2Real)

**域随机化策略**:
```python
# Blender Python脚本核心
for i in range(10000):
    # 光照随机化
    light_intensity = random.uniform(100, 10000)  # lux
    light_color = random.choice(["red", "blue", "white"])

    # 背景随机化
    background_texture = random.choice(texture_library)

    # 物体位姿随机化
    obj_rotation = random.uniform(-180, 180, size=3)
    obj_position = random.uniform(workspace_bounds)

    # 噪声添加
    add_gaussian_noise(sigma=0.01)
    add_motion_blur(velocity=random.uniform(0, 0.5))

    # 渲染并自动标注
    render_image()
    export_annotations()  # COCO格式
```

**训练配置**:
- 数据集: 10000张合成 + 100张真实微调
- 模型: YOLOv8n (轻量化，适合嵌入式)
- GPU: RTX 40系列
- 导出: ONNX/TensorRT for deployment

---

## 四、ROS2集成

### 4.1 节点架构
```
camera_driver_node (30Hz)
    ├─ 发布: /camera/image_raw, /camera/depth
    └─ 参数: exposure_time, gain, trigger_mode

object_detection_node (30Hz)
    ├─ 订阅: /camera/image_raw
    ├─ 发布: /detections (BoundingBox2D[])
    └─ 模型: YOLOv8n.onnx

pose_estimation_node (10Hz)
    ├─ 订阅: /detections, /camera/depth
    ├─ 发布: /target_pose (PoseStamped)
    └─ 算法: FoundationPose或ICP

grasp_planner_node
    ├─ 订阅: /target_pose
    ├─ 服务: /compute_grasp_pose
    └─ 调用: MoveIt move_group
```

### 4.2 关键实现代码

**相机驱动** (camera_driver_node.cpp):
```cpp
// 海康MVS SDK集成
class CameraDriverNode : public rclcpp::Node {
    MV_CC_DEVICE_INFO_LIST device_list_;
    void* camera_handle_;

    void captureLoop() {
        MV_FRAME_OUT frame;
        while (rclcpp::ok()) {
            MV_CC_GetImageBuffer(camera_handle_, &frame, 1000);

            // 转换为ROS消息
            sensor_msgs::msg::Image img_msg;
            img_msg.header.stamp = this->now();
            img_msg.encoding = "bgr8";
            img_msg.data.assign(frame.pBufAddr,
                               frame.pBufAddr + frame.nFrameLen);

            image_pub_->publish(img_msg);
            MV_CC_FreeImageBuffer(camera_handle_, &frame);
        }
    }
};
```

**检测节点** (Python实现更快速迭代):
```python
import rclpy
from ultralytics import YOLO
import cv_bridge

class ObjectDetectionNode(Node):
    def __init__(self):
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.detect_callback, 10)
        self.pub = self.create_publisher(
            Detection2DArray, '/detections', 10)

    def detect_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image, conf=0.7)

        detections = Detection2DArray()
        for r in results[0].boxes:
            det = Detection2D()
            det.bbox.center.x = float(r.xyxy[0][0] + r.xyxy[0][2]) / 2
            det.bbox.center.y = float(r.xyxy[0][1] + r.xyxy[0][3]) / 2
            det.bbox.size_x = float(r.xyxy[0][2] - r.xyxy[0][0])
            det.bbox.size_y = float(r.xyxy[0][3] - r.xyxy[0][1])
            detections.detections.append(det)

        self.pub.publish(detections)
```

---

## 五、实施计划

### 5.1 阶段式开发 (4-5周)

**第1周: 硬件验证**
- [ ] 采购相机和滤光片
- [ ] Python脚本验证相机图像质量
- [ ] 测试红蓝光干扰下的成像效果

**第2周: 数据准备**
- [ ] STEP模型导入Blender
- [ ] 域随机化脚本开发
- [ ] 生成10000张训练图像

**第3周: 模型训练**
- [ ] YOLOv8训练 (RTX 40)
- [ ] 6DoF位姿模型选型测试
- [ ] ONNX导出和TensorRT优化

**第4周: ROS2集成**
- [ ] camera_driver_node开发
- [ ] detection_node集成
- [ ] pose_estimation_node实现

**第5周: 系统联调**
- [ ] 手眼标定
- [ ] MoveIt抓取规划
- [ ] 端到端测试

### 5.2 调试技巧

**数据录制优先**:
```bash
# 录制用于离线调试
ros2 bag record /camera/image_raw /camera/depth -o vision_debug

# 回放测试算法
ros2 bag play vision_debug --loop
```

**模块化测试**:
1. 先用静态图片测试检测
2. 再用rosbag测试实时性
3. 最后接入真实相机

**性能监控**:
```bash
# 查看推理延迟
ros2 topic delay /detections

# CPU/GPU使用率
nvidia-smi -l 1
htop
```

---

## 六、关键技术点

### 6.1 手眼标定精度提升
- 使用ChArUco标定板(棋盘+ArUco混合)
- 采集20+组不同位姿
- 标定误差目标: <2mm

### 6.2 光照鲁棒性
- 主动940nm补光克服环境光变化
- HDR模式应对高动态范围
- 自适应曝光控制

### 6.3 实时性保证
- YOLOv8n量化部署(INT8)
- TensorRT推理优化
- 多线程流水线处理

### 6.4 Sim2Real迁移
- 合成数据预训练建立基础
- 少量真实数据微调适应
- 在线学习持续优化

---

## 七、故障排查

| 问题 | 可能原因 | 解决方案 |
|------|---------|----------|
| 检测不到物体 | 光照过强/过弱 | 调整曝光时间和增益 |
| 位姿估计不准 | 深度图噪声大 | 使用中值滤波预处理 |
| 抓取失败 | 手眼标定误差 | 重新标定,增加采样点 |
| 推理速度慢 | 模型太大 | 改用YOLOv8n或量化 |
| 红蓝光干扰 | 滤光片失效 | 检查940nm窄带滤光片 |

---

## 八、性能指标

| 指标 | 目标值 | 当前状态 |
|------|--------|----------|
| 检测精度 | mAP > 0.9 | 待测试 |
| 检测速度 | < 30ms | YOLOv8n约5ms |
| 位姿精度 | < 5mm, < 5° | 待测试 |
| 端到端延迟 | < 100ms | 估计60-80ms |
| 抓取成功率 | > 90% | 待测试 |

---

**最后更新**: 2026-01-19
**状态**: 方案设计完成，待硬件采购实施