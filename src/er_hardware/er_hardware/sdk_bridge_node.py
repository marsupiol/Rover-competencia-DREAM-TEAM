#!/usr/bin/env python3
"""Minimal ROS 2 bridge for the Earth Rovers SDK (Mission 1 stack).

This node is intentionally slim: GPS, heading, IMU, battery, and cmd_vel only.
It is used by ``ros2 launch er_bringup mission1.launch.py``.

Do NOT run this together with ``earth-rovers-sdk/examples/ros2/earth_rover_bridge.py``.
Both subscribe to ``cmd_vel`` and publish the same ``earth_rover/*`` topics, so running
both will fight over ``POST /control`` and duplicate telemetry publishers.

For camera feed and the full SDK bridge, use ``earth_rover_bridge.py`` from the SDK
repo instead of this node (but not at the same time).
"""

import json
import math
import threading
import time
import requests

try:
    import websocket
except ImportError:
    websocket = None

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu, BatteryState
from std_msgs.msg import Float32

CONTROL_RATE_HZ = 10.0
CMD_VEL_TIMEOUT_S = 0.5
CONTROL_HTTP_TIMEOUT_S = 1.0


class SDKBridgeNode(Node):
    def __init__(self):
        super().__init__("sdk_bridge_node")
        self.declare_parameter("sdk_url", "http://localhost:8000")
        self.sdk_url = self.get_parameter("sdk_url").value.rstrip("/")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)

        self.gps_pub = self.create_publisher(NavSatFix, "earth_rover/gps", sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "earth_rover/imu", sensor_qos)
        self.battery_pub = self.create_publisher(BatteryState, "earth_rover/battery", sensor_qos)
        self.heading_pub = self.create_publisher(Float32, "earth_rover/heading", sensor_qos)

        self._latest_cmd = None
        self._last_cmd_at = 0.0
        self._stopped = True
        self._cmd_lock = threading.Lock()

        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, command_qos)

        self._session = requests.Session()
        self._running = True
        self._stop_event = threading.Event()

        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()

        self._telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telemetry_thread.start()

        self.get_logger().info(f"Earth Rover SDK Bridge active on {self.sdk_url}")
        self.get_logger().info(
            "Minimal bridge for Mission 1. Do not run alongside earth_rover_bridge.py."
        )

    def _on_cmd_vel(self, msg: Twist):
        with self._cmd_lock:
            self._latest_cmd = {
                "linear": max(-1.0, min(1.0, float(msg.linear.x))),
                "angular": max(-1.0, min(1.0, float(msg.angular.z))),
            }
            self._last_cmd_at = time.monotonic()
            self._stopped = False

    def _control_tick(self):
        with self._cmd_lock:
            quiet = time.monotonic() - self._last_cmd_at > CMD_VEL_TIMEOUT_S
            if self._latest_cmd is None or (quiet and self._stopped):
                return
            command = {"linear": 0, "angular": 0} if quiet else dict(self._latest_cmd)
            last_cmd_at = self._last_cmd_at

        try:
            res = self._session.post(
                f"{self.sdk_url}/control",
                json={"command": command},
                timeout=CONTROL_HTTP_TIMEOUT_S,
            )
            res.raise_for_status()
            if quiet:
                with self._cmd_lock:
                    if (
                        self._last_cmd_at == last_cmd_at
                        and time.monotonic() - self._last_cmd_at > CMD_VEL_TIMEOUT_S
                    ):
                        self._stopped = True
        except requests.RequestException as e:
            self.get_logger().warning(f"Control dispatch failed: {e}", throttle_duration_sec=5)

    def _control_loop(self):
        interval = 1.0 / CONTROL_RATE_HZ
        deadline = time.monotonic()
        while self._running and rclpy.ok():
            self._control_tick()
            deadline += interval
            wait = max(0.0, deadline - time.monotonic())
            if self._stop_event.wait(wait):
                break
            if time.monotonic() - deadline > interval:
                deadline = time.monotonic()

    def _telemetry_loop(self):
        ws_url = self.sdk_url.replace("http", "ws", 1) + "/ws/data"
        if websocket is None:
            self.get_logger().warning(
                "websocket-client is not installed; polling SDK /data for telemetry instead"
            )
            while self._running and rclpy.ok():
                self._poll_data_fallback()
                if self._stop_event.wait(1.0):
                    break
            return

        while self._running and rclpy.ok():
            ws = None
            try:
                ws = websocket.create_connection(ws_url, timeout=5)
                self.get_logger().info("Connected to SDK telemetry WebSocket")
                while self._running and rclpy.ok():
                    msg = json.loads(ws.recv())
                    if msg.get("type") in ("snapshot", "telemetry") and msg.get("data"):
                        self._publish_telemetry(msg["data"])
            except Exception:
                self._poll_data_fallback()
                time.sleep(1.0)
            finally:
                if ws is not None:
                    try:
                        ws.close()
                    except Exception:
                        pass

    def _poll_data_fallback(self):
        try:
            res = self._session.get(f"{self.sdk_url}/data", timeout=2.0)
            if res.status_code == 200:
                self._publish_telemetry(res.json())
        except Exception:
            pass

    def _publish_telemetry(self, data: dict):
        now = self.get_clock().now().to_msg()

        lat, lng = data.get("latitude"), data.get("longitude")
        if lat is not None and lng is not None:
            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = "earth_rover_gps"
            gps.latitude = float(lat)
            gps.longitude = float(lng)
            self.gps_pub.publish(gps)

        orientation = data.get("orientation")
        if orientation is not None:
            heading = Float32()
            heading.data = float(orientation)
            self.heading_pub.publish(heading)

        battery = data.get("battery")
        if battery is not None:
            batt = BatteryState()
            batt.header.stamp = now
            batt.percentage = float(battery) / 100.0
            batt.present = True
            self.battery_pub.publish(batt)

        accels, gyros = data.get("accels") or [], data.get("gyros") or []
        if accels or gyros:
            imu = Imu()
            imu.header.stamp = now
            imu.header.frame_id = "earth_rover_imu"
            if accels:
                sample = accels[-1]
                imu.linear_acceleration.x = float(sample[0])
                imu.linear_acceleration.y = float(sample[1])
                imu.linear_acceleration.z = float(sample[2])
            if gyros:
                sample = gyros[-1]
                imu.angular_velocity.x = math.radians(float(sample[0]))
                imu.angular_velocity.y = math.radians(float(sample[1]))
                imu.angular_velocity.z = math.radians(float(sample[2]))
            self.imu_pub.publish(imu)

    def destroy_node(self):
        self._running = False
        self._stop_event.set()
        self._control_thread.join(timeout=1.0)
        for _ in range(3):
            try:
                res = self._session.post(
                    f"{self.sdk_url}/control",
                    json={"command": {"linear": 0, "angular": 0}},
                    timeout=CONTROL_HTTP_TIMEOUT_S,
                )
                res.raise_for_status()
                break
            except requests.RequestException:
                continue
        self._session.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SDKBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
