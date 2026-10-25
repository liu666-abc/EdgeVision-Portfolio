#!/usr/bin/env python3
"""cmd_vel 死区塑形：把 Nav2 的小右修补到"能真正转动"的量级。

背景（2026-10-16 实测）：右转 |ω|<0.12 rad/s 时车几乎不动（≈5° 方向盘旷量）；
1.5 s 的 −0.12 修正里只有 0.85 s 产生转动。MPPI 看不到响应会继续加码，
最终表现为"不修 → 猛修"的极限环。本节点只做一层补偿，便于 A/B 对比。

规则：
  ω<0 且 |ω|<min_right  -> −min_right      （补右转死区）
  0<ω<min_left          -> 0               （左转本来就灵，不放大）
  其余原样；|ω|>max_w 限幅；v 原样

用法：python3 a3_cmd_shaper.py [in_topic] [out_topic] [min_right] [min_left] [max_w]
      默认 /nav_cmd_vel -> /cmd_vel, 0.12 / 0.03 / 0.30
"""
import sys

import rclpy
from geometry_msgs.msg import Twist

IN = sys.argv[1] if len(sys.argv) > 1 else "/nav_cmd_vel"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/cmd_vel"
MIN_R = float(sys.argv[3]) if len(sys.argv) > 3 else 0.12
MIN_L = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03
MAX_W = float(sys.argv[5]) if len(sys.argv) > 5 else 0.30

rclpy.init()
node = rclpy.create_node("a3_cmd_shaper")
pub = node.create_publisher(Twist, OUT, 10)
stats = {"n": 0, "right": 0, "left_zero": 0}


def on_cmd(msg: Twist) -> None:
    w = msg.angular.z
    if w < 0 and abs(w) < MIN_R:
        w = -MIN_R
        stats["right"] += 1
    elif 0 < w < MIN_L:
        w = 0.0
        stats["left_zero"] += 1
    w = max(-MAX_W, min(MAX_W, w))
    out = Twist()
    out.linear.x = msg.linear.x
    out.angular.z = w
    pub.publish(out)
    stats["n"] += 1
    if stats["n"] % 100 == 0:
        node.get_logger().info(f"shaped={stats['n']} right_boost={stats['right']} "
                               f"left_zeroed={stats['left_zero']}")


node.create_subscription(Twist, IN, on_cmd, 20)
node.get_logger().info(f"cmd_shaper: {IN} -> {OUT}  min_right={MIN_R} min_left={MIN_L} max_w={MAX_W}")
try:
    rclpy.spin(node)
finally:
    node.get_logger().info(f"final: shaped={stats['n']} right_boost={stats['right']} "
                           f"left_zeroed={stats['left_zero']}")
    node.destroy_node()
    rclpy.shutdown()
