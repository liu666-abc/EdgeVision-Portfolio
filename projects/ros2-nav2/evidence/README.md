# 2026-10-18 A4（Nav2）方案与配置（走廊首测）

## 目标

在走廊里用 **map 系闭环**完成"到点导航 + 车道保持 + 地图外静态纸箱绕行"，替代"开环/贴墙走直"。
依据 10-16 结论：贴墙闭环只适用"窄长直走廊"（横向 36→5.9 cm），通用直线必须交给地图闭环；
侧滑、横坡、执行器旷量都是**扰动**，由闭环吃掉，精度上限受执行器限制（横向 ~10 cm 量级）。

## 实测约束 → 配置

| 约束 | 实测/来源 | 配置 |
|---|---|---|
| 右转死区 | ω<0.12 无响应（≈5° 方向盘旷量） | `a3_cmd_shaper.py`（可选 A/B，见下） |
| 转向限幅 | 固件 `AngleR ∈ [−0.49, +0.32] rad` | `AckermannConstraints.min_turning_r: 0.35` |
| 最小转弯半径 | 固件 `MINI_AKM_MIN_TURN_RADIUS=0.350` | 同上 |
| 速度 | 已验证 0.10 m/s；地面协议上限 0.15 | `vx_max: 0.15` |
| 角速度上限 | `ω_max = vx/0.35` → 0.15 时 0.43 | `wz_max: 0.43`（厂家原值 2.0 不可达） |
| 倒车 | 阿克曼倒车转向反效，未验证 | `vx_min: 0.0`（暂不允许倒车） |
| footprint | 厂家 0.24×0.186 m | 保留 |
| odom / AMCL | `/odom_combined`（EKF）/ `likelihood_field` | 保留 |
| 代价地图 | 局部 voxel：`/scan /scan2 /camera/depth_registered/points` | 保留（10-13 已验收深度入图） |

`min_turning_r=0.35 m` 来自固件/底盘配置，不是实车最小转弯半径实测值；离线规划对比和
地面转弯半径验证尚未完成。

## 地图

- **首测用**：`~/evi/a3_map_20261014/map_a3_20000101-082541.yaml`
  （10-14 走廊 15.57 m 一趟成图，仓库副本 `2026-10-14_a3-mapping/raw/corridor_15m/map_15m.pgm`，最干净）。
- 备选：`~/evi/a3_wf_20261016/map_corridor_20261016.yaml`（10-16 贴墙驱动，9 m，有伪影）。
- **不要用**大厅那张 `map_after_run2`（已判不可用）。
- 不覆盖厂家 `src/wheeltec_robot_nav2/map/WHEELTEC.*`。

## 参数生成（不改厂家文件）

```
python3 ~/evi/a3_nav2_make_params.py \
  /home/wheeltec/wheeltec_ros2/src/wheeltec_robot_nav2/param/wheeltec_params/param_mini_akm.yaml \
  /home/wheeltec/evi/a3_nav2/param_mini_akm_a3.yaml
```

脚本用 YAML 解析器读取厂家文件、施加实测覆盖、写出**我们自己的副本**并打印差异表（可复现、可审计）。

## 启动

```
source /opt/ros/humble/setup.bash
source /home/wheeltec/wheeltec_ros2/install/setup.bash
ros2 launch wheeltec_nav2 wheeltec_nav2.launch.py \
  map:=/home/wheeltec/evi/a3_map_20261014/map_a3_20000101-082541.yaml \
  params:=/home/wheeltec/evi/a3_nav2/param_mini_akm_a3.yaml
```

RViz 用 `wheeltec_rviz2/rviz/wheeltec.rviz`（固定坐标系 `map`）；启动后用 **2D Pose Estimate** 给一次初始位姿（车头朝走廊纵深）。

### 初始位姿（无 GUI 也能给）

AMCL 必须有初始位姿。两条路：

1. RViz 的 **2D Pose Estimate**（有桌面时）；
2. 命令行（本方案默认）：
   ```
   bash ~/evi/a3_set_initial_pose.sh 0.0 0.0 0
   ```

**首测用哪张图、站在哪**：用 **`map_corridor_20261016.yaml`**（10-16 建图趟），
把车放回**那次建图的起点**（右壁 1.03 m / 左壁 1.06 m 的那个位置）、车头朝走廊纵深。
理由：`slam_toolbox` 建图时 **map 系原点 ≈ 建图起点的机器人位姿**，且当时 `/odom_combined` 从 0 开始
⇒ 初始位姿就是 **(0, 0, 0)**，不用猜坐标。（10-14 那张 `map_15m` 更干净，但起点位姿没记录，留作后续用特征对齐后再上。）

## cmd_vel 塑形（可选 A/B）

- **目的**：MPPI 的小右修（ω<0.12）会被机械旷量吃掉，控制器看不到响应就会加码，最后变成"不修→猛修"的极限环。
- **规则**：导航输出 `ω<0` 且 `|ω|<0.12` → 提到 `−0.12`；`0<ω<0.02` → 0（左转本来就灵）；其余原样；记录补偿次数。
- **实现**：`a3_cmd_shaper.py`（订阅 `/nav_cmd_vel` → 发布 `/cmd_vel`）；Nav2 侧 `-r cmd_vel:=nav_cmd_vel`。
- **A/B**：同一路线跑两趟（有/无塑形），比较横向误差与修正次数。

## 验收（草案）

| 项 | 判据 |
|---|---|
| A4-1 静态走廊到点 | 目标 5 m 外；`/amcl_pose` 相对起终点直线的横向偏差 **max ≤0.15 m**（目标 0.10）；到点（厂家 xy 0.25 / yaw 0.25）；全程无接触 |
| A4-2 地图外静态纸箱 | 走廊 3 m 处放一个静止的 ~0.3 m 高箱子；绕行并通过，绕过后回到车道（偏差回 ≤0.15 m）；**不得触发 Spin**（阿克曼不能原地转） |
| A4-3 分岔 | 目标放在左侧支路（只用左转，符合 10-15 结论） |

记录：`/amcl_pose`、`/cmd_vel`、`/odom_combined`、`/local_costmap/costmap` 快照 + 视频。

## ODD（运行设计域）

- 环境：走廊/房间，**至少一侧有连续墙面或门框等地标**（开阔大厅不作精度承诺）；
- 速度：≤0.15 m/s；
- 地面：抛光地砖可（横向推力由闭环吸收），但**建图须在走廊干净时**（Nav2 静态层不会被雷达清除，会留"幽灵障碍"）；
- 低矮静态障碍：地图外高度 ≤0.3 m 的固定纸箱可绕行；动态障碍未验证，整条通道被堵死需人工介入。

## 风险

1. **恢复行为（已处理）**：厂家参数未指定 BT → 用的是默认 `navigate_to_pose_w_replanning_and_recovery.xml`，**该树含 `Spin`**，
   而阿克曼不能原地转，恢复动作会卡死。已改为
   `navigate_w_replanning_distance.xml`（无 Spin、无恢复动作；规划失败直接 abort，fail-fast）。
   后果：卡住时不会自动脱困，需要人工/后续自定义 BT 处理。
2. **初始位姿**：AMCL 初始误差大 → 用 2D Pose Estimate 人工给一次，误差大的话先原地静止 3–5 s 收敛。
3. **地图与现场不一致**：墙上门牌/杂物变化会进静态层 → 需要重扫或做静态层过滤。

## 文件

| 文件 | 说明 |
|---|---|
| `scripts/a3_nav2_make_params.py` | 从厂家参数生成 A4 参数副本（打印差异表） |
| `scripts/a3_cmd_shaper.py` | cmd_vel 死区塑形（可选 A/B） |
| `scripts/a3_nav2_checks.sh` | 起栈后只读自检（AMCL/TF/代价地图/发布者） |
| `scripts/a3_set_initial_pose.sh` | 命令行给 AMCL 初始位姿（无 GUI） |
| `scripts/a3_nav2_goto.sh` | 命令行发 NavigateToPose 目标并跟随反馈 |

## 2026-10-18 实测结果

### 环境

- 地图：`map_a4_20261018`（708×260 @0.05 = 35.4×13.0 m；贴墙闭环 5 m 一趟建出，**原点 = 建图起点**）。
- 参数：`~/evi/a3_nav2/param_mini_akm_a3.yaml`（`vx_max 0.15`、`wz_max 0.43`、`vx_min 0`、BT 无 Spin、`xy_goal_tolerance 0.35`）。
- 定位：AMCL `set_initial_pose: true, (0,0,0)`；项目自建采样评分脚本 `a3_loc_score.py` 输出
  **GOOD**（97.6% 点在 1 格内、平均最近占据格距离 2.4 cm、位姿 (0.05,0.00,−0.12°)）。

### A4-1 静态到点 —— 通过

- 目标 `(3.0, 0, 0)`：**SUCCEEDED**、19 s、**0 次恢复**、末 AMCL 采样 `(2.73, −0.046)`（距目标 0.27 m；采样间隔 ≥0.25 m，停车位未实测）；
- **最大横向偏差 6.5 cm**（10 个 AMCL 采样，y ∈ [−6.5, +2.3] cm），判据 ≤10 cm ✓；
- 对照：同车同地砖**开环**横向 36 cm（10-16）→ 闭环 6.5 cm。
- 记录：`raw/amcl_run1.csv`。

### A4-2 地图外静态纸箱绕行 —— 通过（修正到点容差后）

- 纸箱约 0.3 m 高、位于车前 ~2 m 一侧。
- 第一次（旧图 + 容差 0.25 + 目标 x=5.0）：**ABORTED**（`Failed to make progress`），末 AMCL 采样 `(5.14, −0.30)`——
  末采样点 x 比目标多 0.14 m、y 偏 −0.30 m；阿克曼最小转弯半径 0.35 m 使其难以原路回线（推测，未直接验证）。
- **修正后（容差 0.35 + 目标 x=4.5）：SUCCEEDED**，0 次恢复，无碰撞（人工观察）。
- 轨迹：`(0.53,0.10) → (1.76,0.45) → (3.00,0.24) → (4.05,0.08)`
  → **采样内绕行最大 0.45 m、末采样点距中线 8 cm** ✓（末采样点距目标约 0.45 m；Nav2 判定到点，停车位未卷尺实测）。
- 记录：`raw/amcl_run4.csv`、`raw/goto_run4_summary.txt`。

### 本阶段踩到的坑（都已解决，务必记住）

1. **`/dev/shm` 残留 33 个 FastRTPS 段** → 连板内 action 响应都超时（`Failed to send goal response (timeout)`）；
   清 `fastrtps_*` / `sem.fastrtps_*` 后立刻恢复。这是反复启停后的本机共享内存卫生问题，
   不把它写成 DDS 层根因；可复现的首参与者异常单独见控制台启动章节。
2. **僵死的 `ros2 action send_goal` / `ros2 topic echo` 客户端**会一直挂着并占 DDS；收尾要按名字清。
3. **`xy_goal_tolerance 0.25` 那趟 ABORTED**（`Failed to make progress`）：该趟用旧图与目标 x=5.0，容差的单独影响未隔离；现用 `0.35`。
4. **大厅/弯道不适合建图**：转弯时 SLAM 失配（`map_a4b_20261018` 作废）；建图要选**直段**。
5. **地图 latched（TRANSIENT_LOCAL）跨机晚加入不补发**：RViz 先开、再重启一次 Nav2 让 map_server 重发即可。
6. 走廊纵向对称：**AMCL 靠墙线无法恢复 1 m 以上的纵向错位**；换场地/朝向必须重扫（原点=起点）。

### ODD 修订

- 先建图、后导航；地图原点 = 建图起点；换朝向/场地需重扫。
- 直段优先（弯道建图会失配）；速度 ≤0.15 m/s；到点容差 0.35 m。
- 地图外静态低矮障碍：本次 0.3 m 高固定纸箱 ✓；动态障碍未验证，整条通道被堵死仍需人工。

### 本阶段新增文件

| 文件 | 说明 |
|---|---|
| `raw/map_a4_20261018.pgm/.yaml/.png` | A4 可用地图（本阶段验证所用） |
| `raw/map_a4b_20261018.*` | 弯道重扫图（**作废**，留作反例） |
| `raw/amcl_run1.csv`、`raw/amcl_run4.csv` | 静态到点与静态纸箱绕行的 AMCL 轨迹 |
| `raw/goto_run4_summary.txt` | 静态纸箱绕行目标、路径采样与结果摘要 |
| `scripts/a3_loc_score.py` | 定位贴合度评分（GOOD/MARGINAL/BAD） |
| `raw/nav2_run2.log` | `nav2_run2` 会话的 Nav2 日志：3.0 目标 `Goal succeeded`（第 590 行）；5.0 目标 `Failed to make progress` → `Goal failed`（第 883、885 行） |
| `raw/maprun_a4b.csv` | 掉头建图趟的贴墙控制日志（对应作废图 `map_a4b`） |
