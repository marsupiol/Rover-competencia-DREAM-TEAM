#!/usr/bin/env python3
"""Mission Manager Node for Earth Rover Mission 1."""

import math
import requests
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String, Bool


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__("mission_manager_node")

        self.declare_parameter("sdk_url", "http://localhost:8000")
        self.declare_parameter("checkpoint_post_retries", 15)
        self.declare_parameter("checkpoint_post_min_interval_s", 2.0)
        self.declare_parameter("checkpoint_max_distance_m", 14.0)
        self.declare_parameter("proximity_dwell_s", 2.0)
        self.declare_parameter("min_navigation_time_s", 8.0)
        self.declare_parameter("pre_post_stop_s", 2.5)

        self.sdk_url = self.get_parameter("sdk_url").value.rstrip("/")
        self.checkpoint_post_retries = int(self.get_parameter("checkpoint_post_retries").value)
        self.checkpoint_post_min_interval_s = float(
            self.get_parameter("checkpoint_post_min_interval_s").value
        )
        self.checkpoint_max_distance_m = float(
            self.get_parameter("checkpoint_max_distance_m").value
        )
        self.proximity_dwell_s = float(self.get_parameter("proximity_dwell_s").value)
        self.min_navigation_time_s = float(self.get_parameter("min_navigation_time_s").value)
        self.pre_post_stop_s = float(self.get_parameter("pre_post_stop_s").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.target_pub = self.create_publisher(NavSatFix, "earth_rover/target_waypoint", 10)
        self.pause_pub = self.create_publisher(Bool, "earth_rover/navigation_pause", 10)
        self.create_subscription(NavSatFix, "earth_rover/gps", self._on_gps, sensor_qos)
        self.create_subscription(String, "earth_rover/waypoint_status", self._on_waypoint_status, status_qos)

        self.current_lat = None
        self.current_lon = None

        self.checkpoints = []
        self.current_checkpoint_idx = 0
        self.latest_scanned_checkpoint = 0
        self.state = "STARTING_MISSION"
        self._start_retry_period_s = 10.0
        self._next_start_attempt_at = 0.0
        self._checkpoint_post_attempts = 0
        self._pending_confirmation_sequence = None
        self._checkpoint_post_ok = False
        self._last_post_response = None
        self._last_post_attempt_at = None
        self._handling_checkpoint = False
        self._near_checkpoint_since = None
        self._checkpoint_target_since = None
        self._controller_reached = False
        self._pre_post_stop_until = None
        self._mission_completed_by_sdk = False
        self.status_pub = self.create_publisher(String, "earth_rover/waypoint_status", 10)

        self.timer = self.create_timer(1.0, self._state_machine_loop)
        self.get_logger().info("Mission Manager Node Initialized")

    def _on_gps(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _on_waypoint_status(self, msg: String):
        if msg.data == "REACHED":
            self._controller_reached = True
            if self.state in ("NAVIGATING_CHECKPOINT", "AWAITING_SDK_CONFIRMATION", "CONFIRMING_CHECKPOINT", "PRE_POST_STOP"):
                self.get_logger().info(f"Controller REACHED in state {self.state}")
                self._begin_checkpoint_post()
            elif self.state != "FINISHED":
                self.get_logger().warning(
                    f"Ignoring REACHED signal in state {self.state}",
                    throttle_duration_sec=5,
                )
            return

        if msg.data.startswith("ALIGN/") or msg.data.startswith("DRIVE/"):
            return

    def _begin_checkpoint_post(self):
        if self.state in ("FINISHED", "PRE_POST_STOP"):
            return
        pause = Bool()
        pause.data = True
        self.pause_pub.publish(pause)
        self._pre_post_stop_until = self.get_clock().now() + Duration(seconds=self.pre_post_stop_s)
        self.state = "PRE_POST_STOP"
        self.get_logger().info(
            f"Stopping robot for {self.pre_post_stop_s:.1f}s before POST /checkpoint-reached"
        )

    def _pre_post_stop_elapsed(self):
        if self._pre_post_stop_until is None:
            return True
        return self.get_clock().now() >= self._pre_post_stop_until

    def _state_machine_loop(self):
        if self.state == "PRE_POST_STOP":
            if not self._pre_post_stop_elapsed():
                return
            self._pre_post_stop_until = None
            self._handle_checkpoint_reached()
            return

        if self.state == "AWAITING_SDK_CONFIRMATION":
            if self._pre_post_stop_until is None:
                self._begin_checkpoint_post()
                return
            if not self._pre_post_stop_elapsed():
                return
            self._pre_post_stop_until = None
            self._handle_checkpoint_reached()
            return

        if self.state == "CONFIRMING_CHECKPOINT":
            return

        if self.state == "NAVIGATING_CHECKPOINT":
            self._check_proximity_to_checkpoint()

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
            if self._refresh_checkpoints_from_sdk():
                self.state = "WAITING_FOR_GPS"

        elif self.state == "WAITING_FOR_GPS":
            if self.current_lat is None or self.current_lon is None:
                self.get_logger().info(
                    "Waiting for first GPS fix before navigating checkpoints...",
                    throttle_duration_sec=5,
                )
                return

            if not self.checkpoints:
                self.get_logger().warning("No checkpoints found. Mission finished without navigation.")
                self._finish_mission()
                return

            self.current_checkpoint_idx = self._first_pending_checkpoint_index()
            if self.current_checkpoint_idx < len(self.checkpoints):
                self._publish_current_checkpoint_goal()
                self.state = "NAVIGATING_CHECKPOINT"
            else:
                self.get_logger().info("All checkpoints already completed.")
                self._finish_mission()

    @staticmethod
    def _haversine_m(lat1, lon1, lat2, lon2):
        r = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (
            math.sin(dlat / 2.0) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2.0) ** 2
        )
        return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    def _distance_to_current_checkpoint(self):
        if self.current_checkpoint_idx >= len(self.checkpoints):
            return None
        if self.current_lat is None or self.current_lon is None:
            return None
        cp = self.checkpoints[self.current_checkpoint_idx]
        lat = float(cp.get("latitude", cp.get("lat", 0)))
        lon = float(cp.get("longitude", cp.get("lon", 0)))
        return self._haversine_m(self.current_lat, self.current_lon, lat, lon)

    def _refresh_checkpoints_from_sdk(self):
        self.get_logger().info("Fetching mission checkpoints list...")
        try:
            res = requests.get(f"{self.sdk_url}/checkpoints-list", timeout=3.0)
            if res.status_code != 200:
                self.get_logger().error(
                    f"Failed to fetch checkpoints: {res.status_code} {res.text}"
                )
                return False

            data = res.json()
            if isinstance(data, dict):
                if isinstance(data.get("checkpoints_list"), dict):
                    nested = data["checkpoints_list"]
                    self.checkpoints = nested.get("checkpoints_list", [])
                    self.latest_scanned_checkpoint = int(
                        nested.get("latest_scanned_checkpoint") or 0
                    )
                else:
                    self.checkpoints = data.get("checkpoints_list", [])
                    self.latest_scanned_checkpoint = int(
                        data.get("latest_scanned_checkpoint") or 0
                    )
            elif isinstance(data, list):
                self.checkpoints = data
                self.latest_scanned_checkpoint = 0
            else:
                self.checkpoints = []
                self.latest_scanned_checkpoint = 0

            self.get_logger().info(
                f"Fetched {len(self.checkpoints)} checkpoints. "
                f"Latest scanned: {self.latest_scanned_checkpoint}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Error fetching checkpoints: {e}")
            return False

    def _check_proximity_to_checkpoint(self):
        # Antes esto solo corria DESPUES de que gps_waypoint_controller ya
        # habia declarado "Target reached" (frenado incluido). Ahora chequea
        # la distancia real en todo momento mientras navega -- si entra en
        # rango del SDK ANTES de que el controller termine de alinearse/
        # frenar, reporta altiro y avanza al siguiente checkpoint, sin
        # esperar el ciclo completo de parada.
        if self._checkpoint_target_since is None:
            return

        elapsed_nav = (
            self.get_clock().now() - self._checkpoint_target_since
        ).nanoseconds / 1e9
        if elapsed_nav < self.min_navigation_time_s:
            return

        dist = self._distance_to_current_checkpoint()
        if dist is None:
            self._near_checkpoint_since = None
            return

        if dist > self.checkpoint_max_distance_m:
            self._near_checkpoint_since = None
            return

        now = self.get_clock().now()
        if self._near_checkpoint_since is None:
            self._near_checkpoint_since = now
            self.get_logger().info(
                f"Within {dist:.1f}m of checkpoint (<= {self.checkpoint_max_distance_m:.0f}m), "
                f"waiting {self.proximity_dwell_s:.0f}s before POST",
                throttle_duration_sec=5,
            )
            return

        elapsed = (now - self._near_checkpoint_since).nanoseconds / 1e9
        if elapsed < self.proximity_dwell_s:
            return

        self._near_checkpoint_since = None
        self.get_logger().info(
            f"Proximity backup: {dist:.1f}m for {elapsed:.1f}s — triggering POST"
        )
        self._begin_checkpoint_post()

    def _publish_current_checkpoint_goal(self):
        if self.current_checkpoint_idx >= len(self.checkpoints):
            self.get_logger().info("No pending checkpoints found.")
            self._finish_mission()
            return

        cp = self.checkpoints[self.current_checkpoint_idx]
        target_msg = NavSatFix()
        target_msg.latitude = float(cp.get("latitude", cp.get("lat", 0)))
        target_msg.longitude = float(cp.get("longitude", cp.get("lon", 0)))
        self.target_pub.publish(target_msg)
        resume = Bool()
        resume.data = False
        self.pause_pub.publish(resume)
        self._near_checkpoint_since = None
        self._checkpoint_target_since = self.get_clock().now()
        self._controller_reached = False
        self._pre_post_stop_until = None
        self._checkpoint_post_attempts = 0
        self._pending_confirmation_sequence = None
        self._checkpoint_post_ok = False
        self._last_post_response = None
        self.get_logger().info(
            f"Navigating to Checkpoint sequence {cp.get('sequence', self.current_checkpoint_idx + 1)} "
            f"({self.current_checkpoint_idx + 1}/{len(self.checkpoints)}): "
            f"({target_msg.latitude}, {target_msg.longitude})"
        )

    def _first_pending_checkpoint_index(self):
        for idx, checkpoint in enumerate(self.checkpoints):
            try:
                sequence = int(checkpoint.get("sequence", idx + 1))
            except (TypeError, ValueError):
                sequence = idx + 1
            if sequence > self.latest_scanned_checkpoint:
                return idx
        return len(self.checkpoints)

    def _current_checkpoint_sequence(self):
        if self.current_checkpoint_idx >= len(self.checkpoints):
            return None
        cp = self.checkpoints[self.current_checkpoint_idx]
        try:
            return int(cp.get("sequence", self.current_checkpoint_idx + 1))
        except (TypeError, ValueError):
            return self.current_checkpoint_idx + 1

    def _notify_checkpoint_reached(self, sequence):
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().error("Cannot POST checkpoint-reached without GPS fix")
            return False, None

        now = self.get_clock().now()
        if self._last_post_attempt_at is not None:
            elapsed = (now - self._last_post_attempt_at).nanoseconds / 1e9
            if elapsed < self.checkpoint_post_min_interval_s:
                return False, None

        self._checkpoint_post_attempts += 1
        self._last_post_attempt_at = now
        dist = self._distance_to_current_checkpoint()
        dist_text = f"{dist:.1f}m" if dist is not None else "unknown"

        payload = {
            "latitude": self.current_lat,
            "longitude": self.current_lon,
        }
        self.get_logger().info(
            f"Posting /checkpoint-reached for checkpoint sequence {sequence} "
            f"(attempt {self._checkpoint_post_attempts}, distance={dist_text}, "
            f"GPS=({self.current_lat:.8f}, {self.current_lon:.8f}))"
        )

        try:
            res = requests.post(
                f"{self.sdk_url}/checkpoint-reached",
                json=payload,
                timeout=15.0,
            )
            if res.status_code == 200:
                data = res.json()
                self.get_logger().info(f"Checkpoint POST OK: {data}")
                return True, data

            detail = res.text
            try:
                detail = res.json()
            except Exception:
                pass
            self.get_logger().error(
                f"checkpoint-reached failed ({res.status_code}): {detail}"
            )
            if res.status_code == 400 and "start-mission" in str(detail).lower():
                self.get_logger().warning(
                    "SDK mission already ended (likely completed on previous checkpoint)"
                )
                return "mission_ended", None
        except Exception as e:
            self.get_logger().error(f"checkpoint-reached error: {e}")

        return False, None

    def _confirm_checkpoint(self, sequence, post_data):
        if post_data:
            if post_data.get("mission_completed"):
                self.latest_scanned_checkpoint = max(self.latest_scanned_checkpoint, sequence)
                return True

            if post_data.get("message") == "Checkpoint reached successfully":
                self.latest_scanned_checkpoint = max(self.latest_scanned_checkpoint, sequence)
                next_seq = post_data.get("next_checkpoint_sequence")
                try:
                    if next_seq not in (None, "") and int(next_seq) > sequence:
                        return True
                except (TypeError, ValueError):
                    pass
                return True

        if self._refresh_checkpoints_from_sdk():
            if self.latest_scanned_checkpoint >= sequence:
                self.get_logger().info(
                    f"SDK list confirms checkpoint {sequence} "
                    f"(latest_scanned_checkpoint={self.latest_scanned_checkpoint})"
                )
                return True

        return False

    def _handle_checkpoint_reached(self):
        if self._handling_checkpoint:
            return
        self._handling_checkpoint = True
        try:
            self._handle_checkpoint_reached_impl()
        finally:
            self._handling_checkpoint = False

    def _handle_checkpoint_reached_impl(self):
        sequence = self._pending_confirmation_sequence
        if sequence is None:
            sequence = self._current_checkpoint_sequence()
            self._pending_confirmation_sequence = sequence

        dist = self._distance_to_current_checkpoint()
        if dist is not None and dist > self.checkpoint_max_distance_m:
            self.get_logger().warning(
                f"REACHED ignored for checkpoint {sequence}: still {dist:.1f}m away "
                f"(need <= {self.checkpoint_max_distance_m:.1f}m for SDK)",
                throttle_duration_sec=3,
            )
            self.state = "NAVIGATING_CHECKPOINT"
            return

        self.state = "CONFIRMING_CHECKPOINT"

        if not self._checkpoint_post_ok:
            ok, data = self._notify_checkpoint_reached(sequence)
            if ok == "mission_ended":
                self._mission_completed_by_sdk = True
                self._finish_mission()
                return
            if not ok:
                self.state = "AWAITING_SDK_CONFIRMATION"
                if self._checkpoint_post_attempts >= self.checkpoint_post_retries:
                    self.get_logger().warning(
                        f"Checkpoint {sequence} POST still failing after "
                        f"{self._checkpoint_post_attempts} attempts; will keep retrying on REACHED"
                    )
                    self._checkpoint_post_attempts = 0
                return
            self._checkpoint_post_ok = True
            self._last_post_response = data
        else:
            data = self._last_post_response

        if not self._confirm_checkpoint(sequence, data):
            self.state = "AWAITING_SDK_CONFIRMATION"
            self.get_logger().warning(
                f"Checkpoint {sequence} POST ok but not confirmed yet; will retry"
            )
            return

        self.get_logger().info(f"Checkpoint {sequence} fully confirmed.")
        self._pending_confirmation_sequence = None
        self._checkpoint_post_attempts = 0
        self._checkpoint_post_ok = False
        self._last_post_response = None
        self._last_post_attempt_at = None

        if data and data.get("mission_completed"):
            self.get_logger().info("SDK reports mission completed — finishing.")
            self._mission_completed_by_sdk = True
            self._finish_mission()
            return

        self.current_checkpoint_idx += 1

        if self.current_checkpoint_idx < len(self.checkpoints):
            self.get_logger().info("Advancing to next checkpoint.")
            self._publish_current_checkpoint_goal()
            self.state = "NAVIGATING_CHECKPOINT"
        else:
            self.get_logger().info("All checkpoints completed.")
            self._finish_mission()

    def _finish_mission(self):
        if self.state == "FINISHED":
            return
        self.state = "FINISHED"
        pause = Bool()
        pause.data = True
        self.pause_pub.publish(pause)
        done = String()
        done.data = "MISSION_FINISHED"
        self.status_pub.publish(done)

        if self._mission_completed_by_sdk:
            self.get_logger().info("Mission already completed by SDK on last checkpoint POST.")
            return

        self.get_logger().info("Ending mission via SDK...")
        try:
            res = requests.post(f"{self.sdk_url}/end-mission", timeout=3.0)
            self.get_logger().info(f"End mission response ({res.status_code}): {res.text}")
        except Exception as e:
            self.get_logger().error(f"Error ending mission: {e}")


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