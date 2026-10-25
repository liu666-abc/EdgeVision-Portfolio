#!/usr/bin/env python3
"""从厂家 Nav2 参数生成本项目 A4 用的副本（不覆盖厂家文件），并打印差异表。

用法：python3 a3_nav2_make_params.py <厂家param_mini_akm.yaml> <输出yaml>

覆盖依据（2026-10-16 实测，见同目录 README）：
  vx_max 0.35 -> 0.15   地面协议上限，已验证速度 0.10
  wz_max 2.0  -> 0.43   = vx_max / min_turning_r(0.35)，厂家原值不可达
  vx_min -0.35 -> 0.0   阿克曼倒车转向反效，暂不允许
  vx_std 0.2  -> 0.10   收紧采样，减少无效候选
  wz_std 0.2  -> 0.12
"""
import sys

import yaml


OVERRIDES = {
    "vx_max": 0.15,
    "wz_max": 0.43,
    "vx_min": 0.0,
    "vx_std": 0.10,
    "wz_std": 0.12,
}

# 默认 BT 含 Spin（阿克曼不能原地转，会卡住）→ 换成无 Spin、无恢复动作的树，失败直接 abort
BT_XML = "/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_distance.xml"

# 到点容差：车长仅 0.24 m；0.25 m 时阿克曼冲过目标后回不来（A4-2 实测 0.34 m 处 abort）
GOAL_XY_TOL = 0.35


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as fh:
        cfg = yaml.safe_load(fh)

    follow = cfg["controller_server"]["ros__parameters"]["FollowPath"]
    print(f"source: {src}")
    print(f"{'param':10s} {'old':>8s} -> {'new':>8s}")
    for key, new in OVERRIDES.items():
        old = follow.get(key, "<missing>")
        follow[key] = new
        print(f"{key:10s} {old!s:>8s} -> {new:>8.3f}")

    akm = follow.get("AckermannConstraints", {})
    print(f"{'min_turning_r':10s} {akm.get('min_turning_r', '<missing>')!s:>8s} -> (kept)")

    bt = cfg["bt_navigator"]["ros__parameters"]
    old_bt = bt.get("default_nav_to_pose_bt_xml", "<default(nav2 builtin)>")
    bt["default_nav_to_pose_bt_xml"] = BT_XML
    print(f"{'bt_xml':10s} {old_bt!s:>8s} -> {BT_XML}")

    gc = cfg["controller_server"]["ros__parameters"]["goal_checker"]
    print(f"{'goal_xy_t':10s} {gc.get('xy_goal_tolerance', '<missing>')!s:>8s} -> {GOAL_XY_TOL:.2f}")
    gc["xy_goal_tolerance"] = GOAL_XY_TOL

    with open(dst, "w") as fh:
        yaml.safe_dump(cfg, fh, sort_keys=False, allow_unicode=True)
    print(f"wrote {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
