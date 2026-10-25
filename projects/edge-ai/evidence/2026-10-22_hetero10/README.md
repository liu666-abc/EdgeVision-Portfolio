# 2026-10-22 异构并发测试

## 目的

在 25 FPS 视频输入基础上，同时运行：

- YOLOv8n INT8 BPU 检测；
- CPU 预处理、DFL/NMS 和跟踪；
- CPU MJPEG 编码与上位机状态服务；
- 30 Hz 合成深度图生成；
- CPU 中值滤波、浮点高斯滤波和 ROI 分位数统计；
- ROS2/DDS 消息传输。

用于观察 CPU 竞争对检测 FPS、端到端耗时和帧预算的影响。

## 工况

- 输入：`vtest.avi` 强制 25 FPS。
- 分辨率：`768x576`。
- 合成深度：`640x480 16UC1 @ 30 Hz`。
- ROS domain：`59`。
- 不启动 Astra 相机、底盘、雷达或 STM32。
- 采样间隔：5 秒。
- 帧预算：40 ms，对应 25 FPS。
- BPU 温度保护：65 °C。

## 结果

- 实际时长：`596.921 s`
- 样本：`120`
- 完成状态：`completed`
- API 失败：`0`
- 相机在线率：`100%`
- 检测器在线率：`100%`
- 帧预算超时：`0`
- 热保护：未触发
- 内存：`1158 MB → 1161 MB`，净增 `3 MB`

| 指标 | baseline 30 min | hetero 10 min |
|---|---:|---:|
| camera FPS mean | `24.899` | `24.896` |
| detector FPS mean | `24.904` | `24.893` |
| total latency p95 | `26.948 ms` | `26.268 ms` |
| CPU temperature max | `49.9 °C` | `52.6 °C` |
| BPU temperature max | `49.7 °C` | `52.1 °C` |
| DDR temperature max | `51.1 °C` | `53.5 °C` |
| BPU ratio mean | `9.11%` | `9.01%` |
| CPU ratio mean | 未采集 | `46.16%` |

异构并发 CPU 利用率：

- mean `46.16%`
- p95 `47.24%`
- max `47.63%`

检测器单帧：

| 阶段 | mean | p95 | max |
|---|---:|---:|---:|
| preprocess | `14.477 ms` | `15.987 ms` | `19.044 ms` |
| infer | `6.190 ms` | `6.459 ms` | `6.610 ms` |
| decode | `3.876 ms` | `4.075 ms` | `4.584 ms` |
| total | `24.546 ms` | `26.268 ms` | `28.880 ms` |

## 判读

- 加入 30 Hz 合成深度 CPU 负载后，CPU 利用率约为 `46%`。
- 检测器仍未出现帧预算超时，p95 总耗时 `26.268 ms < 40 ms`。
- 与无额外深度负载的 30 分钟测试相比，检测 FPS 均值差异小于 `0.02 FPS`。
- CPU/BPU/DDR 峰值温度上升约 2–3 °C，但远低于热保护阈值。
- BPU 占用率没有明显变化，说明额外深度负载主要消耗 CPU。
- 内存只有 `3 MB` 漂移，没有泄漏趋势。

## 边界

- 合成深度负载不是 Astra S 真实驱动，也不包含真实点云配准。
- `vtest.avi` 原始帧率约 10 FPS，本次强制 25 FPS，属于压力重放。
- 没有运行 Nav2、雷达或底盘控制，不是整车并发测试。
- BPU 占用仍约 9%，不是 NPU 饱和测试。
- 结果证明当前多任务 CPU 竞争可控，不代表更高分辨率或更多模型仍能保持 25 FPS。

## 文件

- `raw/evi_hetero10.csv`：120 个逐 5 秒样本
- `raw/evi_hetero10.json`：统计摘要
- `raw/evi_depth_workload.log`：合成深度节点日志

