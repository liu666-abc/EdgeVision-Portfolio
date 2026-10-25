#!/usr/bin/env python3
"""Synthetic 30 Hz depth publisher and CPU filtering workload."""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class SyntheticDepthWorkload(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_depth_cpu_workload")
        self.width = int(self.declare_parameter("width", 640).value)
        self.height = int(self.declare_parameter("height", 480).value)
        self.fps = float(self.declare_parameter("fps", 30.0).value)
        self.kernel = int(self.declare_parameter("median_kernel", 7).value)
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(
            Image,
            "/evi/workload/depth/image_raw",
            5,
        )
        self.depth_publisher = self.create_publisher(
            Float32,
            "/evi/workload/depth/min_m",
            10,
        )
        self.timer = self.create_timer(1.0 / self.fps, self.tick)
        self.sequence = 0
        self.started = time.monotonic()
        self.total_process_ms = 0.0
        self.yy, self.xx = np.mgrid[0:self.height, 0:self.width]
        self.base = (
            1800.0
            + 120.0 * np.sin(self.xx / 90.0)
            + 80.0 * np.cos(self.yy / 75.0)
        ).astype(np.uint16)
        self.get_logger().info(
            f"synthetic depth workload {self.width}x{self.height}@{self.fps} "
            f"median_kernel={self.kernel}"
        )

    def tick(self) -> None:
        started = time.perf_counter()
        t = time.monotonic() - self.started
        frame = self.base.copy()
        x0 = int(120 + (self.width - 260) * (0.5 + 0.5 * np.sin(t * 0.8)))
        y0 = int(130 + (self.height - 280) * (0.5 + 0.5 * np.cos(t * 0.6)))
        frame[y0:y0 + 140, x0:x0 + 180] = 620
        frame[::3, ::3] += 7

        message = self.bridge.cv2_to_imgmsg(frame, encoding="16UC1")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "synthetic_depth"
        self.image_publisher.publish(message)

        depth8 = np.clip(frame / 8, 0, 255).astype(np.uint8)
        filtered8 = cv2.medianBlur(depth8, max(3, self.kernel | 1))
        filtered = cv2.GaussianBlur(
            filtered8.astype(np.float32),
            (0, 0),
            1.2,
        )
        roi = filtered[140:420, 190:500]
        valid = roi[roi > 0]
        nearest = (
            float(np.percentile(valid, 2)) * 8.0 / 1000.0
            if valid.size
            else 0.0
        )
        self.depth_publisher.publish(Float32(data=nearest))

        elapsed_ms = (time.perf_counter() - started) * 1000.0
        self.total_process_ms += elapsed_ms
        self.sequence += 1
        if self.sequence % int(max(1, self.fps * 5)) == 0:
            average_ms = self.total_process_ms / self.sequence
            self.get_logger().info(
                f"frames={self.sequence} avg_process={average_ms:.2f}ms "
                f"roi_min={nearest:.3f}m"
            )


def main() -> None:
    rclpy.init()
    node = SyntheticDepthWorkload()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

