# EVI 端侧感知子系统封版基线

> 封版日期：2026-10-23
> 定位：深圳求职主项目中的 RDK X5 端侧感知子系统
> 用途：作为简历、项目主页、面试复述和后续开发的稳定证据入口
> 当前面试口径见 `../../INTERVIEW_QA_CURRENT_2026-10-23.md`。

## 1. 封版决定

从本文件起，端侧感知子系统停止继续扩展性能研究，转入求职材料整理。

封版范围：

- YOLOv8n ONNX 到 OpenExplorer INT8 PTQ；
- RDK X5 BPU 推理；
- C++ 预处理、DFL/NMS 后处理；
- Kalman + IoU/中心距离跟踪基线；
- ROS2 图像输入、检测结果发布和只读上位机；
- 单路、双路公开视频压力测试；
- 真实 Astra S 相机 QoS 与短时稳定性验证；
- 资源、温度、延迟、回归和故障注入证据。

不纳入本次封版结论：

- 任意词开放词汇；
- VLA、强化学习或大模型训练；
- 生产级 watchdog；
- 完整 MOT benchmark 达标；
- 双真实相机、真实深度算法或跨机 DDS；
- Nav2 语义导航闭环。

## 2. 系统链路

```text
Astra S / 公开视频
        |
        v
ROS2 sensor_msgs/Image
        |
        v
C++ detector_node
  NV12 预处理 -> BPU INT8 推理 -> DFL/NMS -> 跟踪
        |
        v
/evi/detections
        |
        v
只读上位机 /api/status + MJPEG
```

工程目标不是单独跑通 YOLO demo，而是形成可测量、可回归、可解释边界的 ROS2 端侧感知链路。

## 3. 已验收结果

### 3.1 模型与量化

| 项目 | 结果 | 边界 |
|---|---:|---|
| 同源 OpenExplorer PTQ | COCO AP50:95 FP32 `0.2780403`，INT8 `0.2540833` | 全 5,000 图，同源模型和评估口径 |
| FP32 精度保留率 | `91.38%` | 不能外推到所有数据集和类别 |
| 现场 BPU 推理均值 | 约 `6.3 ms` | 640x640 YOLOv8n，当前频率与输入条件 |
| 算子边界 | Conv/heads 可下沉 BPU，DFL/NMS 保留 CPU | 不能宣称整张 ONNX 全部在 BPU 执行 |

证据：

- `../../evidence/03_benchmark/2026-09-18_c-group-same-source-ptq/`
- `../../evidence/05_ai_deploy/2026-09-19_onnx-export/`
- `../../evidence/03_benchmark/2026-09-17_c-group-map-fp32-int8/`

### 3.2 单路与双路视频

| 工况 | 相机均值 | 检测均值 | 延迟 p95 | 结果 |
|---|---:|---:|---:|---|
| 单路 `vtest.avi @25 FPS`，2 分钟 | `25.000 FPS` | `24.984 FPS` | `27.338 ms` | 40 ms 超时 0 |
| 双路 `depth=5 + reliable`，10 分钟 | A/B `25.003/24.996 FPS` | A/B `25.011/25.014 FPS` | A/B `29.266/27.493 ms` | 两路保持输入帧率 |

双路对照：

| 图像订阅 QoS | 检测 A | 检测 B |
|---|---:|---:|
| `depth=1, best_effort` | `24.812 FPS` | `7.551 FPS` |
| `depth=5, best_effort` | `24.214 FPS` | `16.130 FPS` |
| `depth=5, reliable`，10 分钟 | `25.011 FPS` | `25.014 FPS` |

结论：

- 增大队列只能部分缓解，不是根因修复；
- reliable publisher 下的 best-effort 图像订阅会在并发抖动时丢消息；
- 匹配 publisher 的 reliable 订阅是当前可控实验中的因果修复；
- 历史 14.8 Hz 污染数据继续标作无效，不拿它冒充最终结果。

证据：

- `../evidence/2026-10-23_single-qos/`
- `../evidence/2026-10-23_qos-root-cause/`
- `qos-root-cause-investigation.md`

### 3.3 真实 Astra S

| 项目 | 结果 |
|---|---|
| 彩色 publisher QoS | `RELIABLE` |
| 深度 publisher QoS | `RELIABLE` |
| 分辨率与目标帧率 | `640x480@30Hz` |
| 2 分钟相机均值 | `29.429 FPS` |
| 2 分钟检测均值 | `29.602 FPS` |
| 单帧总耗时 p95 | `24.364 ms` |
| 40 ms 帧预算超时 | `0` |
| API 失败 | `0` |

边界：

- 测试画面无目标，detections 与 active_tracks 全程为 0；
- 不能评价真实召回率、误检率、MOT 或跟踪稳定性；
- 相机当前在 USB 2.0 总线上，内核历史日志有 `buffer overrun` 告警；
- 本次只验证单台 Astra 彩色检测，不是双真实相机或跨机网络。

证据：`../evidence/2026-10-23_astra-qos/`。

### 3.4 稳定性、异构并发与回归

| 项目 | 结果 | 可引用结论 |
|---|---|---|
| 30 分钟 soak | 相机/检测 `24.899/24.904 FPS`，延迟 p95 `26.948 ms` | 风扇开启条件下稳定，不是无风扇测试 |
| 10 分钟异构并发 | CPU 均值 `46.156%`，检测 `24.893 FPS`，超时 `0/120` | 合成深度 CPU 负载下仍满足 40 ms 预算 |
| QoS 修复回归 | 跟踪器、MOT 自检、10 图检测、实时契约全部 PASS | 只能证明当前公开视频链路无回归 |
| 故障恢复 | SIGINT 后可检测离线并由测试脚本重启 | 不是生产级守护或硬件掉线恢复 |

证据：

- `../evidence/2026-10-22_soak30/`
- `../evidence/2026-10-22_hetero10/`
- `../evidence/2026-10-23_qos-regression/`
- `../evidence/2026-10-22_fault-recovery/`

### 3.5 跟踪基线

MOT17-mini 只有 8 帧，仅用于验证评估链路：

- MOTA `15.48%`；
- IDF1 `26.80%`；
- ID Switch `0`；
- Recall `15.48%`。

不能写成“MOT 指标达标”。真实目标连续序列的 MOTA、IDF1、ID Switch 和漏检恢复时间仍未完成。

证据：

- `../evidence/2026-10-22_mot17-mini/`
- `tracking-baseline.md`

## 4. 工程亮点

### 4.1 不是只跑模型

- 完成 ONNX 导出、校准、INT8 PTQ 和 RDK X5 部署；
- C++ 实现 NV12 预处理、BPU 推理、DFL/NMS 和 ROS2 输出；
- 保留精度、延迟、资源、温度和原始日志；
- 建立一键回归和可重复脚本。

### 4.2 用实验而不是猜测定位 QoS

- 先发现旧结果受遗留视频源污染，主动将旧数据作废；
- 排除日志 I/O 作为主因；
- 通过视频对调、decoy 扫描和 A/B/A/C/A 确认异常会迁移；
- 用 QoS 单变量实验确认 reliability 是关键变量；
- 修复后做单路、双路 10 分钟和真实 Astra 补充验证；
- 明确真实 Astra publisher 是 reliable，不做盲目配置。

### 4.3 保留证据边界

- 风扇开启的 30 分钟不能写成无风扇；
- 8 帧 MOT 冒烟不能写成 MOT 达标；
- 测试脚本恢复不能写成生产 watchdog；
- 合成深度不能写成真实 Astra 深度链路；
- 无检测目标的 Astra 吞吐测试不能写成检测召回验证。

## 5. 简历可写口径

### 5.1 端侧 AI 部署版

> 在 RDK X5 上完成 YOLOv8n 的 ONNX 导出、OpenExplorer INT8 PTQ、BPU 部署与 C++ ROS2 推理链路；同源 COCO 验证集 AP50:95 从 FP32 `0.2780` 降至 INT8 `0.2541`，精度保留 `91.38%`，现场 BPU 推理均值约 `6.3 ms`；实现预处理、DFL/NMS、目标跟踪、状态 API 和原始证据归档。

> 定位双路检测吞吐异常：在 reliable 视频 publisher 下，best-effort 图像订阅并发丢帧使 B 路检测降至 `16.13 FPS`；改为 `depth=5 + reliable` 后，双路 10 分钟稳定在 `25.011/25.014 FPS`。进一步核验真实 Astra 彩色 publisher 为 reliable，2 分钟检测均值 `29.602 FPS`、延迟 p95 `24.364 ms`、40 ms 超时 0。

> 建立 ROS2 感知回归与稳定性测试：完成 30 分钟风扇工况 soak、10 分钟 BPU 检测与 CPU 合成深度异构并发、10 图检测回归、实时状态契约和故障注入；异构并发下 CPU 均值 `46.156%`、检测 `24.893 FPS`、40 ms 超时 `0/120`。

### 5.2 ROS2 机器人系统版

> 构建只读 ROS2 端侧感知链路，接入 Astra S 彩色/深度流与公开视频源，发布 `/evi/detections`，通过 MJPEG 上位机展示检测、track_id、FPS、时延、CPU/BPU 温度与频率；明确代理不发布控制指令，不接触 STM32。

> 使用 QoS、话题频率、CPU/BPU 占用、单帧耗时和原始 CSV/JSON 交叉定位双路并发检测饥饿；修复后完成单路、双路、真实相机和回归验证，并保留污染数据与有效数据的边界。

## 6. 面试复述

可以按下面的顺序讲：

1. 目标：在 RDK X5 上做可部署的 ROS2 端侧视觉链路，而不是只跑 YOLO demo。
2. 实现：ONNX 导出、OpenExplorer INT8、C++ 前后处理、BPU 推理、跟踪和上位机。
3. 问题：双路视频时 B 路检测从约 25 FPS 掉到 15 FPS 以下，但 CPU/BPU 没有饱和。
4. 排错：先发现遗留进程污染并作废旧数据，再排除日志 I/O，做视频对调和 decoy 实验，最后做 QoS 单变量对照。
5. 根因：reliable publisher 下，best-effort subscriber 在并发抖动时丢消息；队列深度只能部分缓解。
6. 修复：`depth=5 + reliable` 使双路 10 分钟稳定在约 25 FPS，并通过回归。
7. 真实设备：Astra S publisher 实测为 reliable，2 分钟彩色检测均值 `29.602 FPS`，40 ms 超时 0；但画面无目标，所以不宣称召回或 MOT 达标。
8. 收获：会测量、会做单变量实验、会保留证据边界，也知道生产 watchdog 和完整 MOT 仍是缺口。

## 7. 求职匹配

优先投递：

- 端侧 AI 部署、嵌入式 AI；
- 机器人感知部署、视觉部署；
- ROS2 机器人软件、系统集成；
- 机器人测试开发、现场应用和调试。

不作为第一目标：

- VLA、世界模型、纯算法研究；
- 明确要求硕士、985/211 或顶会论文的岗位；
- 只做训练、不接触部署和真机的岗位。

第二端侧平台继续延期。当前不购买 Jetson、RK3576、RK3568 或第二块地平线板。等第一轮深圳岗位反馈明确出现 RKNN/RK3588 缺口后再决策。

## 8. 封版后的下一步

端侧感知子系统冻结后，主项目回到 V2 计划中的 ROS2 具身机器人闭环：

- 整理 A4 Nav2 地图、参数、脚本和原始数据入口；
- 完成语义地图标记和预设词表映射；
- 接入 Nav2 到达与到达后视觉复检；
- 做 5 次受控真实运行，再扩展到 10 次；
- 同步补齐项目一页纸、Benchmark、简历和投递记录。

地面动作前仍必须由用户确认“场地清空、手在电机开关旁”。

