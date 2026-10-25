#!/usr/bin/env python3
"""Keep one lightweight ROS participant alive for FastDDS startup ordering."""

from __future__ import annotations

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def main() -> int:
    rclpy.init()
    node = Node("evi_nav_dds_warmup")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
