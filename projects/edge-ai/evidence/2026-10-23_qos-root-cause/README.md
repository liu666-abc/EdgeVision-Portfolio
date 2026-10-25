# 2026-10-23 双路检测饥饿根因：图像 QoS 可靠性

> 完整调查过程、排除项、面试口径和边界见
> `../../docs/qos-root-cause-investigation.md`。

## 结论

复现出的双路检测严重饥饿，根因是 **best-effort 图像订阅在并发条件下丢消息**。

- 把队列深度从 `1` 增加到 `5` 仍不能解决；
- 保持 `depth=5`，把图像订阅从 `best_effort` 改为 `reliable` 后恢复；
- 修复后连续 10 分钟 A/B 检测均约 `25.01 FPS`。

之前 2026-10-22 的 B 路 `14.8 Hz` 记录已经因遗留进程污染而作废；
本次通过可控变量实验确认了同类严重饥饿的机制。

## 条件

- A 路：`vtest.avi @ 25 FPS`。
- B 路：Intel `one-by-one-person-detection.mp4 @ 25 FPS`。
- 两个视频源均由 `evi_video_source` 发布，publisher QoS 为 reliable。
- 两个检测器均启用跟踪，逐帧 CSV/track 日志关闭。
- 上位机只订阅 A 路。
- ROS domain `59`。

## A/B 对照

| 图像订阅 QoS | 检测 A mean | 检测 B mean | CPU mean | BPU mean |
|---|---:|---:|---:|---:|
| `depth=1, best_effort` | `24.812` | `7.551` | `49.685%` | `12.833%` |
| `depth=5, best_effort` | `24.214` | `16.130` | `52.175%` | `15.417%` |
| `depth=5, reliable`, 2 min | `25.007` | `25.013` | `54.569%` | `18.917%` |
| `depth=5, reliable`, 10 min | `25.011` | `25.014` | `49.932%` | `18.883%` |

## 关键证据

1. `depth=1 → depth=5`，保持 best-effort：
   - B 从约 `7.55` 提升到 `16.13 FPS`；
   - 说明队列深度能缓解一部分突发，但不是根因。

2. `best_effort → reliable`，保持 `depth=5`：
   - B 从 `16.13` 恢复到 `25.01 FPS`；
   - A 也保持在约 `25.01 FPS`；
   - 变量只改了可靠性，因此可确认可靠性是修复关键。

3. 10 分钟复用：
   - A 检测 mean `25.011 FPS`，p95 `25.127 FPS`；
   - B 检测 mean `25.014 FPS`，p95 `25.106 FPS`；
   - A/B 单帧 p95 分别为 `29.266 / 27.493 ms`；
   - CPU mean `49.932%`，BPU mean `18.883%`；
   - 无降频、无热保护。

## 修复

公开视频双路演示默认改为：

```bash
EVI_DUAL_IMAGE_QOS_DEPTH=5
EVI_DUAL_IMAGE_QOS_RELIABLE=1
```

真实 Astra 相机仍需先检查 publisher QoS。如果 publisher 是 best-effort，
检测器不能强制使用 reliable，否则 QoS 不兼容。检测器 C++ 默认仍保留
best-effort，双路视频脚本显式启用 reliable。

## 边界

- 这确认的是“可靠 publisher + best-effort subscriber”在并发压力下的丢帧机制。
- 历史 14.8 Hz 原始现场没有以完全相同条件重放，但本次可控实验解释了同类饥饿。
- 真实 Astra 相机、DDS 网络传输和跨机通信需单独验证 QoS。
- 10 分钟不是 24 小时稳定性结论。

## 文件

- `raw/evi_dual_default_restart.csv` / `.json`
- `raw/evi_dual_qos5.csv` / `.json`
- `raw/evi_dual_qos5_reliable.csv` / `.json`
- `raw/evi_dual_qos5_reliable10.csv` / `.json`

