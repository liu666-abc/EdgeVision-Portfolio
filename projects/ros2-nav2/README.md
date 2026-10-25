# ROS2/Nav2 移动机器人

基于 Wheeltec mini Ackermann 平台的 ROS2 建图、定位、导航与可视化控制台项目。

## 硬件与软件

- RDK X5、Wheeltec mini Ackermann 底盘、激光雷达、IMU、编码器。
- ROS2 Humble/TROS、Nav2、AMCL、slam_toolbox、rosbag。
- 控制台：Python、aiohttp、WebSocket、Canvas、HTML/CSS/JavaScript。

## 实现范围

- 联调现有 ROS2 驱动与 launch，检查 topic 频率、TF、AMCL、lifecycle 和地图发布。
- 使用 A4 走廊地图进行 Nav2 到点和动态障碍实验。
- 在厂家驱动和底盘参数基础上，完成 Ackermann 速度/转角约束，并将含 `Spin` 的默认 Nav2
  行为树切换为内置 `navigate_w_replanning_distance.xml`。该方案无自动恢复动作，
  规划失败时直接 abort；参数为 `vx_max 0.15 m/s`、`wz_max 0.43 rad/s`、
  `min_turning_r 0.35 m`。
- 实现独立 ROS2 桥、FastDDS 预热节点和 WebSocket 控制台，支持建图、保存地图、
  Nav2 切换、初始位姿、目标下发与安全停止。

## 实测结果

| 项目 | 结果 |
|---|---|
| 地图 | 708×260 @ 0.05 m/格 |
| scan-to-map 自洽性 | 自建脚本：97.6% 采样点在 1 格内，平均最近占据格距离 2.4 cm；不含外部真值 |
| 静态到点 | 19 s，0 次恢复，最大横向误差 6.5 cm |
| 开环对照 | 同场地横向漂移 36 cm |
| 动态绕障 | 单次成功，绕行约 0.45 m，回中线约 8 cm |
| 控制台 | `ACTION SERVER READY`，遥测 20 Hz，场景 10 Hz，地图 2 Hz |

## 目录

- `console/`：WebSocket 控制台、独立 ROS2 桥、Web 前端和 smoke 工具。
- `launch/`：建图与导航专用 launch。
- `scripts/`：控制台、建图和导航起停脚本。
- `evidence/`：A4 结果说明、地图、AMCL 轨迹、Nav2 日志和验证脚本。

## 运行前提

需要已安装的 ROS2/TROS、Wheeltec 驱动、雷达驱动和目标底盘。设备文件名、话题和
地图路径应从 `console/config.example.json` 与环境变量确认后再运行。

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r console/requirements.txt
python3 console/server.py --config console/config.json --web-root console/web
```

详细说明见 `console/README.md`。

## 边界

结果绑定 `0.10 m/s` 附近、走廊贴墙场景、`map_a4_20261018` 和建图起点原点。
动态绕障为单次受控实验，不代表多场景成功率或功能安全认证。
控制台已验证项、未验证项和已知限制见
[console/LIMITATIONS.md](console/LIMITATIONS.md)。
