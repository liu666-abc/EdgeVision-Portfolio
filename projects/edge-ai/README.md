# RDK X5 端侧视觉

基于 YOLOv8n、OpenExplorer、RDK X5 BPU 和 C++ ROS2 的端侧视觉推理与测试项目。

## 技术链路

```text
Image / Camera
  -> NV12 preprocessing
  -> RDK X5 BPU inference
  -> DFL decode + NMS
  -> ROS2 detections
  -> status API / dashboard
```

## 实现内容

- YOLOv8n ONNX 导出、INT8 PTQ、BPU 模型部署。
- C++ NV12 预处理、推理、DFL/NMS 后处理和 ROS2 消息发布。
- 检测结果、逐帧延迟、系统资源、温度和 QoS 状态采集。
- 单路/双路视频、真实 Astra S、异构并发、Soak 和回归测试。

## 关键结果

| 指标 | 结果 |
|---|---|
| 项目内固定阈值 AP50:95 | FP32 `0.2780` / INT8 `0.2541`；`score>=0.25`，不是官方 COCO 基线 |
| 项目内配对保留率 | `91.38%`；只适合同口径 FP32/INT8 比较 |
| BPU 单帧推理 | 约 `6.3 ms` |
| ROS2 检测发布 | 约 `30 Hz` |
| 双路视频 | `depth=5 + reliable`，10 分钟约 `25 FPS` |
| 真实 Astra S | `29.602 FPS`，p95 `24.364 ms`，40 ms 超时 `0` |
| 10 分钟异构并发 | CPU 均值 `46.156%`，40 ms 超时 `0/120` |

## 目录

- `src/evi_cpp_detector/`：C++ ROS2 检测节点与 tracker 单元测试。
- `tools/`：CSV 分析、双路监控、Soak、故障恢复和回归脚本。
- `docs/`：封版基线、证据边界和 QoS 根因分析。
- `evidence/`：双路、异构并发、Soak、Astra 和 QoS 的原始摘要数据。
- `screenshots/`：实时控制台截图。

## 构建与运行

构建依赖 RDK X5 BPU 运行库、OpenExplorer、OpenCV、TROS 和 ROS2 Humble。

```bash
cd src
colcon build --packages-select evi_cpp_detector
source install/setup.bash
ros2 run evi_cpp_detector detector_node --ros-args \
  -p image_topic:=/camera/color/image_raw
```

具体模型和运行库不随仓库分发，需从合法来源获取。

## 边界

- COCO 数值使用 `score>=0.25` 后处理截断，尚未按标准低阈值重跑 FP32/INT8；
  详见 [COCO 基线审计](docs/coco-baseline-audit.md)。
- 30 分钟 Soak 为风扇开启工况。
- Astra S 测试画面无目标，不能据此声称召回率或 MOT 达标。
- 8 帧 MOT17-mini 仅用于链路冒烟，不作为 MOT benchmark。
- 故障恢复由测试脚本触发，不是生产级 watchdog。
