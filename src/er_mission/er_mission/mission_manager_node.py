#!/usr/bin/env python3
"""Mission Manager Node for Earth Rover Mission 1."""

import requests
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("sdk_url", "http://localhost:8000")
        self.sdk_url = self.get_parameter("sdk_url").value.rstrip("/")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.target_pub = self.create_publisher(NavSatFix, "earth_rover/target_waypoint", 10)
        self.create_subscription(NavSatFix, "earth_rover/gps", self._on_gps, sensor_qos)
        self.create_subscription(String, "earth_rover/waypoint_status", self._on_waypoint_status, 10)

        self.current_lat = None
        self.current_lon = None

        self.checkpoints = []
        self.current_checkpoint_idx = 0
        self.latest_scanned_checkpoint = 0
        self.state = "STARTING_MISSION"
        self._start_retry_period_s = 10.0
        self._next_start_attempt_at = 0.0

        self.timer = self.create_timer(1.0, self._state_machine_loop)
        self.get_logger().info("Mission Manager Node Initialized")

    def _on_gps(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _on_waypoint_status(self, msg: String):
        if msg.data == "REACHED" and self.state == "NAVIGATING_CHECKPOINT":
            self.get_logger().info(f"Received REACHED signal in state {self.state}")
            self._handle_checkpoint_reached()

    def _state_machine_loop(self):
        if self.state == "STARTING_MISSION":
            now = self.get_clock().now().nanoseconds / 1e9
            if now < self._next_start_attempt_at:
                return
            self._next_start_attempt_at = now + self._start_retry_period_s

            self.get_logger().info("Requesting start-mission from SDK...")
            try:
                res = requests.post(f"{self.sdk_url}/start-mission", timeout=3.0)
                if res.status_code == 200:
                    self.get_logger().info(f"Start mission response: {res.json()}")
                    self.state = "FETCHING_CHECKPOINTS"
                elif res.status_code == 422:
                    self.get_logger().warning(
                        "SDK says the bot is unavailable. Check BOT_SLUG, MISSION_SLUG, "
                        "reservation/availability, and whether another session is active. "
                        f"Retrying in {self._start_retry_period_s:.0f}s. Response: {res.text}"
                    )
                else:
                    self.get_logger().warning(
                        f"Unexpected status code starting mission: {res.status_code} {res.text}"
                    )
            except Exception as e:
                self.get_logger().error(f"Error starting mission: {e}")

        elif self.state == "FETCHING_CHECKPOINTS":
            self.get_logger().info("Fetching mission checkpoints list...")
            try:
                res = requests.get(f"{self.sdk_url}/checkpoints-list", timeout=3.0)
                if res.status_code == 200:
                    data = res.json()
                    if isinstance(data, dict):
                        self.checkpoints = data.get("checkpoints_list", [])
                        self.latest_scanned_checkpoint = int(data.get("latest_scanned_checkpoint") or 0)
                    elif isinstance(data, list):
                        self.checkpoints = data
                        self.latest_scanned_checkpoint = 0

                    self.get_logger().info(
                        f"Fetched {len(self.checkpoints)} checkpoints. "
                        f"Latest scanned: {self.latest_scanned_checkpoint}"
                    )
                    self.state = "WAITING_FOR_GPS"
            except Exception as e:
                self.get_logger().error(f"Error fetching checkpoints: {e}")

        elif self.state == "WAITING_FOR_GPS":
            if self.current_lat is None or self.current_lon is None:
                self.get_logger().info(
                    "Waiting for first GPS fix before navigating checkpoints...",
                    throttle_duration_sec=5,
                )
                return

            if self.checkpoints:
                self.current_checkpoint_idx = self._first_pending_checkpoint_index()
                if self.current_checkpoint_idx < len(self.checkpoints):
                    self._publish_current_checkpoint_goal()
                    self.state = "NAVIGATING_CHECKPOINT"
                else:
                    self.get_logger().info("All checkpoints are already completed. Mission finished.")
                    self.state = "FINISHED"
            else:
                self.get_logger().warning("No checkpoints found. Mission finished without navigation.")
                self.state = "FINISHED"

    def _publish_current_checkpoint_goal(self):
        if self.current_checkpoint_idx < len(self.checkpoints):
            cp = self.checkpoints[self.current_checkpoint_idx]
            target_msg = NavSatFix()
            target_msg.latitude = float(cp.get("latitude", cp.get("lat", 0)))
            target_msg.longitude = float(cp.get("longitude", cp.get("lon", 0)))
            self.target_pub.publish(target_msg)
            self.get_logger().info(
                f"Navigating to Checkpoint sequence {cp.get('sequence', self.current_checkpoint_idx + 1)} "
                f"({self.current_checkpoint_idx + 1}/{len(self.checkpoints)}): "
                f"({target_msg.latitude}, {target_msg.longitude})"
            )
        else:
            self.get_logger().info("No pending checkpoints found. Mission finished.")
            self.state = "FINISHED"

    def _first_pending_checkpoint_index(self):
        for idx, checkpoint in enumerate(self.checkpoints):
            try:
                sequence = int(checkpoint.get("sequence", idx + 1))
            except (TypeError, ValueError):
                sequence = idx + 1
            if sequence > self.latest_scanned_checkpoint:
                return idx
        return len(self.checkpoints)

    def _handle_checkpoint_reached(self):
        self.get_logger().info("Notifying SDK of reached checkpoint...")
        try:
            res = requests.post(f"{self.sdk_url}/checkpoint-reached", json={}, timeout=3.0)
            self.get_logger().info(f"Checkpoint reached response: {res.text}")
        except Exception as e:
            self.get_logger().error(f"Error sending checkpoint-reached: {e}")

        self.current_checkpoint_idx += 1
        if self.current_checkpoint_idx < len(self.checkpoints):
            self._publish_current_checkpoint_goal()
            self.state = "NAVIGATING_CHECKPOINT"
        else:
            self.get_logger().info("All checkpoints completed. Mission finished.")
            self.state = "FINISHED"


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
