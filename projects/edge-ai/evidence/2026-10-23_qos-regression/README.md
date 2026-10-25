# 2026-10-23 QoS 修复后回归

## 目的

确认图像 QoS 从 best-effort 改为 reliable 后，没有破坏：

- C++ 跟踪器；
- MOT 评估器；
- 10 图静态检测结果；
- 实时上位机状态契约。

## 结果

总耗时：`45.838 s`

| 测试 | 结果 | 耗时 |
|---|---|---:|
| C++ 跟踪器自检 | PASS | `6.569 s` |
| MOT 评估器自检 | PASS | `0.231 s` |
| 10 图静态检测回归 | PASS | `9.047 s` |
| 实时上位机契约 | PASS | `2.386 s` |

静态检测数量仍为：

```text
[6, 1, 4, 1, 3, 2, 2, 2, 4, 10]
```

实时契约：

- 相机在线；
- 检测器在线；
- 活跃轨迹 `13`；
- CPU 状态可用。

## 修复配置

公开视频双路默认：

```text
image_qos_depth=5
image_qos_reliable=true
```

根因和前后对照见：

`../2026-10-23_qos-root-cause/README.md`

## 边界

- 回归通过不代表真实 Astra best-effort 相机可以直接改成 reliable；
- 回归使用的实时契约来自公开视频源；
- 尚未做真实相机 QoS 协商和跨机网络验证。

## 文件

- `evi_regression_qos_20261023/summary.json`
- `evi_regression_qos_20261023/live_contract.json`
- `evi_regression_qos_20261023/static_detector.csv`
- `evi_regression_qos_20261023/*.log`

