# 证据边界与当前结论

本文件用于防止把“实验现象”写成“能力达标”。引用项目数据时以本表为准。

| 项目 | 当前可说 | 当前不可说 | 证据 |
|---|---|---|---|
| 双路 14.8 Hz | 2026-10-22 旧实验受遗留视频源污染，不能作为最终调度结论 | 不能把 14.8 Hz 直接归因于 BPU 或 CPU 饱和 | `evidence/2026-10-22_dual-stream5/` |
| 14.8 Hz decoy 实验 | 空闲/持续 decoy 都会改变资源状态，但异常会迁移到 A 路；遗留进程不是充分原因 | 不能把 decoy 阶段异常归因于遗留节点；根因由后续 QoS 单变量实验确认 | `evidence/2026-10-23_decoy-causality/` |
| 双路检测饥饿根因 | 可控 A/B 确认 reliable publisher 下的 best-effort 图像订阅会丢消息；depth 5 + reliable 使两路 10 分钟稳定约 25 FPS | 不能把这个结论直接套到 best-effort 相机的跨网通信，真实 Astra 仍需匹配 QoS | `evidence/2026-10-23_qos-root-cause/` |
| 单路 QoS 修复后验证 | 公开视频单路深度 5 + reliable 运行 2 分钟，相机均值 25.000 FPS、检测均值 24.984 FPS、40 ms 超时 0 | 这是 2 分钟压力回放，不是真实 Astra、双路真实相机或长稳结论；也不是无风扇对照 | `evidence/2026-10-23_single-qos/` |
| 真实 Astra QoS | Astra S 彩色/深度 publisher 均为 reliable；彩色 depth 5 + reliable 订阅运行 2 分钟，相机均值 29.429 FPS、检测均值 29.602 FPS、40 ms 超时 0 | 画面无目标，不能评价检测召回或跟踪；不是双相机、深度算法、跨机网络或长稳结论 | `evidence/2026-10-23_astra-qos/` |
| QoS 修复回归 | 跟踪器、MOT 自检、10 图检测和实时契约均 PASS | 只能说明当前公开视频链路无回归；不能替代双真实相机/跨机网络验证 | `evidence/2026-10-23_qos-regression/` |
| 干净双路 10 分钟 | QoS 修复前：两路相机约 25 FPS，检测 A/B 约 22.3/23.9 FPS；未复现单路崩到 15 FPS | 这是修复前数据，不能作为最终吞吐；后续已用 reliable QoS 恢复到约 25 FPS | `evidence/2026-10-23_dual-stream10/` |
| MOT17-mini | 8 帧冒烟评估：MOTA 15.48%、IDF1 26.80%、ID Switch 0 | 不能说 MOT 指标已达标；不能与 MOT17 官方全序列榜单比较；不能外推召回率 | `evidence/2026-10-22_mot17-mini/` |
| 30 分钟 soak | 在已安装风扇、25 FPS 压力回放下连续运行 30 分钟稳定 | 不能把它计入“无风扇 30 分钟”；不能外推出整车、RGB-D、雷达并发 | `evidence/2026-10-22_soak30/` |
| COCO FP32/INT8 精度 | 在同一 `score>=0.25` 后处理截断下，封版配对 AP50:95 为 `0.2780403/0.2540833`，可做项目内配对比较 | 不是官方 COCO 基线；`AR@100=0.331` 受低分框预过滤影响；不能与 Ultralytics 官方低阈值结果直接比较 | [coco-baseline-audit.md](coco-baseline-audit.md) |
| 故障恢复 | 测试脚本注入 SIGINT 后，可检测离线并由脚本重新拉起 | 不能宣称生产级自动守护、断电恢复、USB 掉线恢复或串口锁死恢复 | `evidence/2026-10-22_fault-recovery/` |
| 网络地址 | 192.168.128.10 是有线维护地址；192.168.0.100 是车自身 Wi-Fi AP 地址 | 不能把两个地址当成同一链路；URL 取决于 PC 当前接入哪个网络 | `README.md` |

