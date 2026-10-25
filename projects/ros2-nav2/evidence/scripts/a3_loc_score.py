#!/usr/bin/env python3
"""定位贴合度评分：把 /scan 变换到 map 系，统计雷达点落在已有墙上的比例。

用途：判断 AMCL 定位是否正确（比肉眼看 RViz 可靠），也可作为 A4 验收指标。
用法：python3 a3_loc_score.py <map.yaml> [采样秒数] [最大量程m]

输出：
  n_used       参与统计的点数（距离 ≤ max_range）
  <=1/<=2/<=4  与最近占据格的距离（格，1 格 = 0.05 m）在阈值内的比例
  mean_d       平均距离（格）
  verdict      判读：GOOD(≤1 格 ≥70%) / MARGINAL(40-70%) / BAD(<40%)
"""
import math
import sys

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

MAP_YAML = sys.argv[1] if len(sys.argv) > 1 else \
    "/home/wheeltec/evi/a3_wf_20261016/map_corridor_20261016.yaml"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
MAX_RANGE = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0

meta = yaml.safe_load(open(MAP_YAML))
img_path = str(meta["image"])
if not img_path.startswith("/"):
    img_path = MAP_YAML.rsplit("/", 1)[0] + "/" + img_path
img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
occ = (img < 10).astype(np.uint8)          # 0 = 占据
res = float(meta["resolution"])
ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
H, W = img.shape
print(f"map {W}x{H} res={res} origin=({ox:.2f},{oy:.2f}) occupied={int(occ.sum())}")

rclpy.init()
node = Node("a3_loc_score")
buf = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(buf, node)
scans = []
node.create_subscription(LaserScan, "/scan", lambda m: scans.append(m), 10)

pts = None
end = node.get_clock().now().nanoseconds + int(SECS * 1e9)
while node.get_clock().now().nanoseconds < end:
    rclpy.spin_once(node, timeout_sec=0.05)

if not scans:
    print("no /scan received")
    sys.exit(2)
scan = scans[-1]

try:
    tr = buf.lookup_transform("map", scan.header.frame_id, rclpy.time.Time())
except Exception as exc:  # noqa: BLE001
    print(f"TF failed: {exc}")
    sys.exit(3)

tx = tr.transform.translation
q = tr.transform.rotation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
c, s = math.cos(yaw), math.sin(yaw)

cols = []
for i, r in enumerate(scan.ranges):
    if not math.isfinite(r) or r < scan.range_min or r > min(scan.range_max, MAX_RANGE):
        continue
    a = scan.angle_min + i * scan.angle_increment
    lx, ly = r * math.cos(a), r * math.sin(a)
    mx = tx.x + c * lx - s * ly
    my = tx.y + s * lx + c * ly
    col = int((mx - ox) / res)
    row = int(H - 1 - (my - oy) / res)
    if 0 <= row < H and 0 <= col < W:
        cols.append((row, col))

if not cols:
    print("no valid points")
    sys.exit(4)

k = np.ones((7, 7), np.uint8)
dist = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 3)   # 距最近占据格的距离（格）
d = np.array([dist[r, c] for r, c in cols])
n = len(d)
print(f"n_used={n}  <=1:{np.mean(d <= 1):.2%}  <=2:{np.mean(d <= 2):.2%}  "
      f"<=4:{np.mean(d <= 4):.2%}  mean_d={d.mean():.2f} cells "
      f"({d.mean() * res * 100:.1f} cm)")
v = np.mean(d <= 1)
print("verdict:", "GOOD" if v >= 0.70 else ("MARGINAL" if v >= 0.40 else "BAD"))
print(f"pose: x={tx.x:.3f} y={tx.y:.3f} yaw={math.degrees(yaw):+.2f} deg")
