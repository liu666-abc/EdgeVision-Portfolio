# 结果汇总与证据边界

## ROS2/Nav2

| 指标 | 实测 | 证据 |
|---|---:|---|
| 地图尺寸 | 708×260 @ 0.05 m/格 | `projects/ros2-nav2/evidence/raw/map_a4_20261018.yaml` |
| scan-to-map 自洽性 | 自建脚本：97.6% 在 1 格内，平均最近占据格距离 2.4 cm；无外部真值 | `projects/ros2-nav2/evidence/README.md` |
| 静态到点 | 19 s，0 次恢复，6.5 cm | `projects/ros2-nav2/evidence/raw/amcl_run1.csv` |
| 动态绕障 | 单次成功，绕行 0.45 m，回中线 8 cm | `projects/ros2-nav2/evidence/raw/goto_run4_summary.txt` |
| 控制台 | ACTION READY，零目标保持 | `projects/ros2-nav2/console/README.md` |

ROS2 结论限定于 A4 走廊、地图原点为建图起点、低速和当前轮式底盘。动态绕障为单次
受控实验，不代表多场景成功率。

## RDK X5 端侧视觉

| 指标 | 实测 | 证据 |
|---|---:|---|
| 项目内固定阈值 FP32/INT8 | 0.2780 / 0.2541；score≥0.25，非官方 COCO 基线 | `projects/edge-ai/docs/coco-baseline-audit.md` |
| 项目内配对保留率 | 91.38%；仅限同口径比较 | 同上 |
| BPU 单帧推理 | 约 6.3 ms | 同上 |
| 双路 10 分钟 | 约 25 FPS | `projects/edge-ai/evidence/2026-10-23_qos-root-cause/` |
| Astra S | 29.602 FPS，p95 24.364 ms | `projects/edge-ai/evidence/2026-10-23_astra-qos/` |
| 10 分钟异构并发 | CPU 46.156%，超时 0/120 | `projects/edge-ai/evidence/2026-10-22_hetero10/` |
| 30 分钟 Soak | 风扇开启，未触发 65°C 自停 | `projects/edge-ai/evidence/2026-10-22_soak30/` |

端侧视觉结果区分视频源、分辨率、线程数、QoS 和散热条件。无目标 Astra S 画面不用于
召回率、MOT 或实际避障能力结论。
