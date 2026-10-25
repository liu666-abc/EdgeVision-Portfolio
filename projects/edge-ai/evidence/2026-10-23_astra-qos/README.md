# 2026-10-23 真实 Astra QoS 与 2 分钟稳定性验证

## 目的

此前 QoS 根因实验使用的是公开视频 publisher。为确认修复参数能否用于真实相机，本次只启动静止 Astra S，执行：

- 读取彩色和深度图像 publisher 的 QoS；
- 让检测器以匹配的 QoS 订阅真实彩色图像；
- 连续采样 2 分钟，观察 FPS、端到端耗时、温度、内存和 API 稳定性；
- 不启动底盘、雷达、真实深度处理链路或 STM32。

## 硬件与软件

- 板端：RDK X5。
- 系统：Linux `6.1.83`，aarch64。
- 相机：Orbbec Astra S，USB ID `2bc5:0402`。
- 相机序列号：`17120911684`。
- USB 总线：`Bus 001`，即 USB 2.0 链路。
- 彩色/深度：`640x480@30Hz`。
- 模型：YOLOv8n INT8，`yolov8_640x640_nv12.bin`。
- ROS domain：`59`；启用本机通信。
- 图像订阅参数：`image_qos_depth=5`、`image_qos_reliable=true`。
- 采样间隔：5 秒。
- 帧预算：40 ms。
- 温度保护阈值：65 °C。

## QoS 核验

`ros2 topic info -v` 结果：

| Topic | Publisher reliability |
|---|---|
| `/camera/color/image_raw` | `RELIABLE` |
| `/camera/depth/image_raw` | `RELIABLE` |

彩色话题当时有两个订阅者：

- 检测器：`RELIABLE`；
- 上位机图像预览：`BEST_EFFORT`。

因此检测器显式设置 `depth=5 + reliable` 与真实 Astra publisher 兼容。
CLI 对 publisher 的 History Depth 显示为 `UNKNOWN`，不能据此声称相机端队列深度为 5；
`depth=5` 是检测器订阅端的设置。

补充短测：

- 彩色 `ros2 topic hz` 约 `29.58 FPS`；
- 深度 `ros2 topic hz` 约 `29.65 FPS`。

## 复现配置

启动静止相机：

```bash
source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
export ROS_DOMAIN_ID=59
export ROS_LOCALHOST_ONLY=1

ros2 launch astra_camera astra.launch.xml \
  depth_registration:=true \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false \
  enable_ir:=false
```

启动检测器：

```bash
source /opt/tros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
source /home/wheeltec/evi/upper_computer_ws/install/setup.bash
export ROS_DOMAIN_ID=59
export ROS_LOCALHOST_ONLY=1

ros2 run evi_cpp_detector detector_node --ros-args \
  -p mode:=camera \
  -p tracking:=true \
  -p model:=/opt/hobot/model/x5/basic/yolov8_640x640_nv12.bin \
  -p image_topic:=/camera/color/image_raw \
  -p image_qos_depth:=5 \
  -p image_qos_reliable:=true \
  -p confidence:=0.25 \
  -p iou:=0.45 \
  -p top_k:=300
```

采样：

```bash
python3 /home/wheeltec/evi/evi_dashboard/tools/soak_monitor.py \
  --minutes 2 \
  --interval 5 \
  --frame-budget-ms 40 \
  --csv /home/wheeltec/evi/evi_dashboard/evidence/2026-10-23_astra-qos/raw/evi_astra_qos.csv \
  --summary /home/wheeltec/evi/evi_dashboard/evidence/2026-10-23_astra-qos/raw/evi_astra_qos.json
```

## 结果

| 指标 | 结果 |
|---|---:|
| 计划时长 | `2 min` |
| 实际时长 | `115.347 s` |
| 样本数 | `24` |
| 完成状态 | `completed` |
| API 失败 | `0` |
| 相机在线率 | `100%` |
| 检测器在线率 | `100%` |
| 40 ms 帧预算超时 | `0` |
| 内存起点 | `958 MB` |
| 内存终点 | `957 MB` |
| 内存净变化 | `-1 MB` |

相机与检测器：

| 指标 | camera | detector |
|---|---:|---:|
| min | `28.62 FPS` | `29.57 FPS` |
| mean | `29.429 FPS` | `29.602 FPS` |
| p50 | `29.59 FPS` | `29.60 FPS` |
| p95 | `29.62 FPS` | `29.64 FPS` |
| max | `29.64 FPS` | `29.66 FPS` |

检测器单帧耗时：

| 阶段 | mean | p95 | max |
|---|---:|---:|---:|
| preprocess | `12.511 ms` | `13.855 ms` | `13.896 ms` |
| infer | `6.272 ms` | `6.647 ms` | `6.699 ms` |
| decode | `3.481 ms` | `4.053 ms` | `4.250 ms` |
| total | `22.266 ms` | `24.364 ms` | `24.454 ms` |

资源与温度：

| 指标 | mean | p95 | max |
|---|---:|---:|---:|
| CPU ratio | `30.715%` | `32.99%` | `34.82%` |
| BPU ratio | `11.125%` | `12%` | `12%` |
| CPU temperature | `45.338 °C` | `45.6 °C` | `45.6 °C` |
| BPU temperature | `45.192 °C` | `45.3 °C` | `45.5 °C` |
| DDR temperature | `46.825 °C` | `47.0 °C` | `47.0 °C` |

频率：

- BPU：全程 `1000 MHz`；
- DDR：全程 `4266 MHz`。

## 判读

- 真实 Astra 彩色 publisher 为 `RELIABLE`，与检测器的 reliable 订阅匹配。
- 检测均值 `29.602 FPS`，与相机均值 `29.429 FPS` 基本一致，未观察到公开视频双路实验中那种检测饥饿。
- 单帧 p95 `24.364 ms`，低于 40 ms 帧预算。
- 24 个采样中没有 API 失败、节点离线、热保护或频率下降。
- 本次内存首尾基本不变，但 2 分钟不足以判断泄漏。

## 边界

- **画面在测试期间没有检测目标，detections 和 active_tracks 全程为 0。**
- 因无目标，本次不能评价召回率、误检率、MOT 指标或跟踪稳定性。
- 这是 2 分钟、单台 Astra、彩色检测验证，不是双相机或 24 小时稳定性结论。
- 没有运行真实深度算法、点云配准、Nav2、雷达、底盘或 STM32。
- 没有覆盖 USB 断连、掉帧后恢复或跨机 DDS 网络。
- 相机当前在 USB 2.0 总线上；启动前后的内核日志中存在 `buffer overrun` 告警。本次采样稳定，但不能据此排除 USB 带宽风险，也不能外推到 USB 3.0。
- 本次没有切换风扇供电状态，不能用于风扇收益结论。
- 板端 RTC 返回 `2000-01-01`，与本次实际日期 `2026-10-23` 不符；原始文件的板端时间戳不能作为可信实验时间。

## 文件

关键原始文件：

原始文本证据在入库前按仓库规则统一为 LF，下列 SHA-256 按 LF 文件计算。

- `raw/evi_astra_qos.csv`：24 个逐 5 秒样本，SHA-256 `81777AF768646D7C62B6FD6868013D4D340B364812F2B3DD08668C72115EF020`
- `raw/evi_astra_qos.json`：完整统计摘要，SHA-256 `ABC4B45EC18649E306304E6FC04263B8F0D53E273B93416CC050A1F55705D0E0`
- `raw/astra_color_qos.txt`：彩色话题 QoS，SHA-256 `6CDB6CF4447CAEB65BCDA546998E6625EEF9817CB4A7CDCC24A3177E99B4922C`
- `raw/astra_depth_qos.txt`：深度话题 QoS，SHA-256 `1F2344C0798EF6466E9AB7A888651A8C5CB8F93ACC94DEACB31E5B467ECFA024`

其他现场文件：

- `raw/astra_launch.log`
- `raw/detector.log`
- `raw/dashboard.log`
- `raw/lsusb.txt`
- `raw/dmesg_tail.txt`
- `raw/uname.txt`
- `raw/board_date.txt`
- `raw/uptime.txt`
- `raw/ip_brief.txt`

