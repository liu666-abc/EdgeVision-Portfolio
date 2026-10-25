#!/usr/bin/env python3
"""Standalone ROS2 bridge process for the Nav2 web console."""

from __future__ import annotations

import base64
import json
import math
import queue
import sys
import threading
import time
from collections import deque
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= math.tau
    while angle < -math.pi:
        angle += math.tau
    return angle


def compose_transform(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    x1, y1, yaw1 = first
    x2, y2, yaw2 = second
    cos_yaw = math.cos(yaw1)
    sin_yaw = math.sin(yaw1)
    return (
        x1 + cos_yaw * x2 - sin_yaw * y2,
        y1 + sin_yaw * x2 + cos_yaw * y2,
        normalize_angle(yaw1 + yaw2),
    )


def invert_transform(
    transform: tuple[float, float, float],
) -> tuple[float, float, float]:
    x, y, yaw = transform
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -cos_yaw * x - sin_yaw * y,
        sin_yaw * x - cos_yaw * y,
        normalize_angle(-yaw),
    )


def lookup_tf_chain(
    edges: dict[tuple[str, str], tuple[float, float, float]],
    target: str,
    source: str,
) -> tuple[float, float, float] | None:
    if target == source:
        return (0.0, 0.0, 0.0)
    pending: deque[tuple[str, tuple[float, float, float]]] = deque(
        [(target, (0.0, 0.0, 0.0))]
    )
    visited = {target}
    while pending:
        current, accumulated = pending.popleft()
        for (parent, child), transform in edges.items():
            if parent == current and child not in visited:
                visited.add(child)
                composed = compose_transform(accumulated, transform)
                if child == source:
                    return composed
                pending.append((child, composed))
            elif child == current and parent not in visited:
                visited.add(parent)
                composed = compose_transform(
                    accumulated,
                    invert_transform(transform),
                )
                if parent == source:
                    return composed
                pending.append((parent, composed))
    return None


class BridgeNode(Node):
    def __init__(self, config: dict[str, Any]) -> None:
        super().__init__("evi_nav_console_bridge")
        self.config = config
        self.topics = config["topics"]
        self.map_frame = config["map_frame"]
        self.base_frame = config["base_frame"]
        self.commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self.stop_event = threading.Event()
        self.emit_lock = threading.Lock()
        self.tf_lock = threading.RLock()
        self.subscription_group = ReentrantCallbackGroup()

        self.pose = {"x": 0.0, "y": 0.0, "yaw": 0.0, "updated": 0.0}
        self.speed = {"linear_x": 0.0, "angular_z": 0.0}
        self.scan_map: list[list[float]] = []
        self.scan_polar: list[list[float]] = []
        self.scan_frame = ""
        self.scan_transformed = False
        self.scan_updated = 0.0
        self.amcl_updated = 0.0
        self.global_path: list[list[float]] = []
        self.local_path: list[list[float]] = []
        self.path_updated = 0.0
        self.scene_revision = 0
        self.map_updated = 0.0
        self.callback_counts = {"map": 0, "scan": 0, "tf": 0, "tf_static": 0}
        self.started_at = time.monotonic()
        self.resubscribe_count = 0
        self.subscription_attrs = (
            "map_subscription",
            "scan_subscription",
            "amcl_subscription",
            "global_path_subscription",
            "local_path_subscription",
            "odom_subscription",
            "tf_subscription",
            "tf_static_subscription",
        )
        self.map_meta = {
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "origin_yaw": 0.0,
        }
        self.tf_edges: dict[tuple[str, str], tuple[float, float, float]] = {}
        self.goal_handle: Any = None
        self.goal_total_distance: float | None = None
        self.nav = {
            "mode": "IDLE",
            "server_ready": False,
            "goal_active": False,
            "distance_remaining": None,
            "eta_s": None,
            "recoveries": 0,
            "last_result": "",
            "goal_total_distance": None,
        }
        self.navigation_enabled = False
        self.nav_client: ActionClient | None = None
        self.nav_client_created_at = 0.0
        self.last_teleop_at = 0.0

        self._create_subscriptions()
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.global_localization_client = self.create_client(
            Empty,
            "/reinitialize_global_localization",
        )
        self.nomotion_update_client = self.create_client(
            Empty,
            "/request_nomotion_update",
        )
        self.get_logger().info(
            f"bridge ready map={self.topics['map']} scan={self.topics['scan']}"
        )

    def _create_subscriptions(self) -> None:
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.topics["map"],
            self.on_map,
            map_qos,
            callback_group=self.subscription_group,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.topics["scan"],
            self.on_scan,
            qos_profile_sensor_data,
            callback_group=self.subscription_group,
        )
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topics["amcl_pose"],
            self.on_amcl_pose,
            10,
            callback_group=self.subscription_group,
        )
        self.global_path_subscription = self.create_subscription(
            Path,
            self.topics["global_path"],
            self.on_global_path,
            10,
            callback_group=self.subscription_group,
        )
        self.local_path_subscription = self.create_subscription(
            Path,
            self.topics["local_path"],
            self.on_local_path,
            10,
            callback_group=self.subscription_group,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.topics["odom"],
            self.on_odom,
            10,
            callback_group=self.subscription_group,
        )
        self.tf_subscription = self.create_subscription(
            TFMessage,
            "/tf",
            self.on_tf,
            100,
            callback_group=self.subscription_group,
        )
        self.tf_static_subscription = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.on_tf_static,
            static_qos,
            callback_group=self.subscription_group,
        )

    def rebuild_subscriptions(self) -> None:
        for name in self.subscription_attrs:
            subscription = getattr(self, name, None)
            if subscription is not None:
                self.destroy_subscription(subscription)
                setattr(self, name, None)
        self._create_subscriptions()
        self.resubscribe_count += 1
        self.get_logger().warn(
            f"rebuilt subscriptions after startup timeout: {self.callback_counts}"
        )

    def emit(self, payload: dict[str, Any]) -> None:
        with self.emit_lock:
            sys.stdout.write(json.dumps(payload, separators=(",", ":")) + "\n")
            sys.stdout.flush()

    def record_tf(self, message: TFMessage) -> None:
        with self.tf_lock:
            for item in message.transforms:
                parent = item.header.frame_id.lstrip("/")
                child = item.child_frame_id.lstrip("/")
                translation = item.transform.translation
                rotation = item.transform.rotation
                self.tf_edges[(parent, child)] = (
                    float(translation.x),
                    float(translation.y),
                    yaw_from_quaternion(
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    ),
                )
            for frame in (self.base_frame, "base_link"):
                transform = lookup_tf_chain(
                    self.tf_edges,
                    self.map_frame,
                    frame.lstrip("/"),
                )
                if transform is not None:
                    self.pose.update(
                        {
                            "x": round(transform[0], 4),
                            "y": round(transform[1], 4),
                            "yaw": round(transform[2], 5),
                            "updated": time.monotonic(),
                        }
                    )
                    break

    def on_tf(self, message: TFMessage) -> None:
        self.callback_counts["tf"] += 1
        self.record_tf(message)

    def on_tf_static(self, message: TFMessage) -> None:
        self.callback_counts["tf_static"] += 1
        self.record_tf(message)

    def on_map(self, message: OccupancyGrid) -> None:
        self.callback_counts["map"] += 1
        origin = message.info.origin
        self.map_meta = {
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": round(float(message.info.resolution), 6),
            "origin_x": round(float(origin.position.x), 5),
            "origin_y": round(float(origin.position.y), 5),
            "origin_yaw": round(
                yaw_from_quaternion(
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                    origin.orientation.w,
                ),
                5,
            ),
        }
        payload = bytes((int(value) & 0xFF) for value in message.data)
        self.emit(
            {
                "type": "map",
                "meta": self.map_meta,
                "data": base64.b64encode(payload).decode("ascii"),
            }
        )
        self.map_updated = time.monotonic()

    def on_scan(self, message: LaserScan) -> None:
        self.callback_counts["scan"] += 1
        finite: list[tuple[float, float]] = []
        angle = message.angle_min
        for distance in message.ranges:
            if (
                math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
            ):
                finite.append((angle, distance))
            angle += message.angle_increment
        stride = max(1, len(finite) // 720)
        sampled = finite[::stride]
        self.scan_polar = [
            [round(math.degrees(angle), 2), round(distance, 3)]
            for angle, distance in sampled
        ]
        self.scan_frame = message.header.frame_id
        with self.tf_lock:
            transform = lookup_tf_chain(
                self.tf_edges,
                self.map_frame,
                message.header.frame_id.lstrip("/"),
            )
        if transform is None:
            self.scan_map = []
            self.scan_transformed = False
        else:
            tx, ty, yaw = transform
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            self.scan_map = [
                [
                    round(tx + cos_yaw * distance * math.cos(angle)
                          - sin_yaw * distance * math.sin(angle), 3),
                    round(ty + sin_yaw * distance * math.cos(angle)
                          + cos_yaw * distance * math.sin(angle), 3),
                ]
                for angle, distance in sampled
            ]
            self.scan_transformed = True
        self.scan_updated = time.monotonic()
        self.scene_revision += 1

    def on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        pose = message.pose.pose
        rotation = pose.orientation
        self.pose.update(
            {
                "x": round(float(pose.position.x), 4),
                "y": round(float(pose.position.y), 4),
                "yaw": round(
                    yaw_from_quaternion(
                        rotation.x,
                        rotation.y,
                        rotation.z,
                        rotation.w,
                    ),
                    5,
                ),
                "updated": time.monotonic(),
            }
        )
        self.amcl_updated = time.monotonic()

    @staticmethod
    def path_points(message: Path) -> list[list[float]]:
        points = [
            [
                round(float(pose.pose.position.x), 3),
                round(float(pose.pose.position.y), 3),
            ]
            for pose in message.poses
        ]
        if len(points) > 2000:
            stride = max(1, len(points) // 2000)
            points = points[::stride]
        return points

    def on_global_path(self, message: Path) -> None:
        self.global_path = self.path_points(message)
        self.path_updated = time.monotonic()
        self.scene_revision += 1

    def on_local_path(self, message: Path) -> None:
        self.local_path = self.path_points(message)
        self.path_updated = time.monotonic()
        self.scene_revision += 1

    def on_odom(self, message: Odometry) -> None:
        self.speed = {
            "linear_x": round(float(message.twist.twist.linear.x), 4),
            "angular_z": round(float(message.twist.twist.angular.z), 4),
        }

    def emit_telemetry(self) -> None:
        now = time.monotonic()
        self.emit(
            {
                "type": "telemetry",
                "pose": {
                    **self.pose,
                    "age_ms": (
                        round((now - self.pose["updated"]) * 1000.0, 1)
                        if self.pose["updated"] > 0.0
                        else None
                    ),
                },
                "speed": self.speed,
                "nav": self.nav,
                "map_meta": self.map_meta,
                "map_age_ms": (
                    round((now - self.map_updated) * 1000.0, 1)
                    if self.map_updated > 0.0
                    else None
                ),
                "scan_age_ms": (
                    round((now - self.scan_updated) * 1000.0, 1)
                    if self.scan_updated > 0.0
                    else None
                ),
                "amcl_age_ms": (
                    round((now - self.amcl_updated) * 1000.0, 1)
                    if self.amcl_updated > 0.0
                    else None
                ),
                "path_age_ms": (
                    round((now - self.path_updated) * 1000.0, 1)
                    if self.path_updated > 0.0
                    else None
                ),
                "publishers": {
                    "map": self.count_publishers(self.topics["map"]),
                    "scan": self.count_publishers(self.topics["scan"]),
                    "tf": self.count_publishers("/tf"),
                    "tf_static": self.count_publishers("/tf_static"),
                },
                "callback_counts": dict(self.callback_counts),
                "resubscribe_count": self.resubscribe_count,
                "tf_frames": sorted(
                    f"{parent}->{child}"
                    for parent, child in list(self.tf_edges)
                ),
            }
        )

    def emit_scene(self) -> None:
        self.emit(
            {
                "type": "scene",
                "revision": self.scene_revision,
                "scan": self.scan_map,
                "scan_polar": self.scan_polar,
                "scan_frame": self.scan_frame,
                "scan_transformed": self.scan_transformed,
                "global_path": self.global_path,
                "local_path": self.local_path,
                "footprint": [],
                "goal": None,
                "initial_pose": None,
            }
        )

    def ensure_nav_client(self) -> None:
        if self.nav_client is None:
            self.nav_client = ActionClient(
                self,
                NavigateToPose,
                self.config["actions"]["navigate_to_pose"],
            )
            self.nav_client_created_at = time.monotonic()

    def set_nav(self, **values: Any) -> None:
        self.nav.update(values)

    def send_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        coarse: bool,
    ) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self.map_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        covariance = [0.0] * 36
        covariance[0] = 1.0 if coarse else 0.25
        covariance[7] = 1.0 if coarse else 0.25
        covariance[35] = 0.25 if coarse else 0.07
        message.pose.covariance = covariance
        self.initial_pose_pub.publish(message)
        self.emit(
            {
                "type": "event",
                "level": "info",
                "message": f"初始位姿已发送 ({x:.2f}, {y:.2f})",
            }
        )

    def send_goal(self, x: float, y: float, yaw: float) -> None:
        if self.nav_client is None:
            self.set_nav(mode="ACTION_UNAVAILABLE")
            return
        if not self.nav_client.server_is_ready():
            self.set_nav(mode="ACTION_UNAVAILABLE")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        self.goal_total_distance = math.hypot(x - self.pose["x"], y - self.pose["y"])
        self.set_nav(
            mode="SENDING",
            goal_active=True,
            goal_total_distance=self.goal_total_distance,
            last_result="",
        )
        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.on_goal_feedback,
        )
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.set_nav(mode="FAILED", last_result=str(exc))
            return
        if not goal_handle.accepted:
            self.set_nav(mode="REJECTED", goal_active=False)
            return
        self.goal_handle = goal_handle
        self.set_nav(mode="NAVIGATING", goal_active=True)
        goal_handle.get_result_async().add_done_callback(self.on_goal_result)

    def on_goal_feedback(self, message: Any) -> None:
        feedback = message.feedback
        eta = getattr(feedback, "estimated_time_remaining", None)
        eta_s = None
        if eta is not None:
            eta_s = float(eta.sec) + float(eta.nanosec) / 1e9
        self.set_nav(
            mode="NAVIGATING",
            goal_active=True,
            distance_remaining=(
                round(float(feedback.distance_remaining), 3)
                if getattr(feedback, "distance_remaining", None) is not None
                else None
            ),
            eta_s=eta_s,
            recoveries=int(getattr(feedback, "number_of_recoveries", 0)),
        )

    def on_goal_result(self, future: Any) -> None:
        try:
            status = int(future.result().status)
        except Exception as exc:  # noqa: BLE001
            self.set_nav(mode="FAILED", last_result=str(exc))
            return
        self.goal_handle = None
        labels = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }
        label = labels.get(status, f"STATUS_{status}")
        self.set_nav(
            mode=label,
            goal_active=False,
            last_result=label,
            distance_remaining=0.0 if label == "SUCCEEDED" else None,
        )

    def call_empty(self, client: Any, label: str) -> None:
        if not client.service_is_ready():
            self.emit(
                {
                    "type": "event",
                    "level": "warn",
                    "message": f"{label}服务未就绪",
                }
            )
            return
        future = client.call_async(Empty.Request())
        future.add_done_callback(
            lambda result: self.emit(
                {
                    "type": "event",
                    "level": "info" if result.exception() is None else "warn",
                    "message": f"{label}请求完成",
                }
            )
        )

    def send_teleop(self, linear_x: float, angular_z: float) -> None:
        linear_x = max(0.0, min(0.12, linear_x))
        angular_z = max(-0.45, min(0.45, angular_z))
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = angular_z
        self.cmd_vel_pub.publish(message)
        self.last_teleop_at = time.monotonic()
        self.speed = {
            "linear_x": round(linear_x, 4),
            "angular_z": round(angular_z, 4),
        }

    def process_commands(self) -> None:
        while True:
            try:
                command = self.commands.get_nowait()
            except queue.Empty:
                return
            action = command.get("action")
            try:
                if action == "goal":
                    self.send_goal(
                        float(command["x"]),
                        float(command["y"]),
                        float(command["yaw"]),
                    )
                elif action == "initial_pose":
                    self.send_initial_pose(
                        float(command["x"]),
                        float(command["y"]),
                        float(command["yaw"]),
                        bool(command.get("coarse", False)),
                    )
                elif action == "cancel" and self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                elif action == "teleop":
                    self.send_teleop(
                        float(command.get("linear_x", 0.0)),
                        float(command.get("angular_z", 0.0)),
                    )
                elif action == "zero_velocity":
                    self.cmd_vel_pub.publish(Twist())
                    self.last_teleop_at = 0.0
                    self.speed = {"linear_x": 0.0, "angular_z": 0.0}
                elif action == "enable_navigation":
                    self.navigation_enabled = True
                    self.ensure_nav_client()
                elif action == "disable_navigation":
                    self.navigation_enabled = False
                    self.set_nav(server_ready=False)
                elif action == "global_localize":
                    self.call_empty(
                        self.global_localization_client,
                        "全局重定位",
                    )
                elif action == "nomotion_update":
                    self.call_empty(
                        self.nomotion_update_client,
                        "静止更新",
                    )
            except Exception as exc:  # noqa: BLE001
                self.emit(
                    {
                        "type": "event",
                        "level": "warn",
                        "message": f"{action} 失败: {exc}",
                    }
                )

    def poll(self) -> None:
        self.process_commands()
        if (
            self.resubscribe_count == 0
            and time.monotonic() - self.started_at > 4.0
            and not any(self.callback_counts.values())
        ):
            self.rebuild_subscriptions()
        if self.navigation_enabled and self.nav_client is not None:
            ready = self.nav_client.server_is_ready()
            if (
                not ready
                and time.monotonic() - self.nav_client_created_at > 5.0
            ):
                self.nav_client = None
                self.ensure_nav_client()
                ready = False
            self.set_nav(server_ready=ready)
        if (
            self.last_teleop_at > 0.0
            and time.monotonic() - self.last_teleop_at > 0.35
        ):
            self.cmd_vel_pub.publish(Twist())
            self.last_teleop_at = 0.0
            self.speed = {"linear_x": 0.0, "angular_z": 0.0}


def stdin_reader(node: BridgeNode) -> None:
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            command = json.loads(line)
        except json.JSONDecodeError:
            continue
        if command.get("action") == "shutdown":
            node.stop_event.set()
            return
        node.commands.put(command)


def main() -> int:
    config = json.loads(sys.argv[1])
    rclpy.init()
    node = BridgeNode(config)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    threading.Thread(target=stdin_reader, args=(node,), daemon=True).start()
    last_telemetry = 0.0
    last_scene = 0.0
    last_scene_revision = -1
    try:
        while rclpy.ok() and not node.stop_event.is_set():
            executor.spin_once(timeout_sec=0.02)
            node.poll()
            now = time.monotonic()
            if now - last_telemetry >= 0.05:
                node.emit_telemetry()
                last_telemetry = now
            if (
                node.scene_revision != last_scene_revision
                and now - last_scene >= 0.1
            ):
                node.emit_scene()
                last_scene = now
                last_scene_revision = node.scene_revision
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

