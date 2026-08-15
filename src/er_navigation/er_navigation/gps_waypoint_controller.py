#!/usr/bin/env python3
"""GPS Waypoint Navigation Controller for Earth Rover.

Web-safe control:
1. ALIGN  - short turn bursts, then full stop while telemetry catches up
2. DRIVE  - forward once aligned
3. WAIT   - stopped until mission manager confirms checkpoint and sends next target
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool


class GPSWaypointController(Node):
    def __init__(self):
        super().__init__("gps_waypoint_controller")

        self.declare_parameter("goal_tolerance_m", 3.0)
        self.declare_parameter("goal_dwell_s", 1.5)
        self.declare_parameter("align_threshold_deg", 15.0)
        self.declare_parameter("coarse_align_threshold_deg", 45.0)
        self.declare_parameter("approach_align_distance_m", 8.0)
        self.declare_parameter("forward_speed", 0.35)
        self.declare_parameter("turn_speed", 0.25)
        self.declare_parameter("drive_correction_gain", 0.002)
        self.declare_parameter("max_drive_angular", 0.10)
        self.declare_parameter("invert_angular", True)
        self.declare_parameter("control_loop_hz", 2.0)
        self.declare_parameter("turn_burst_s", 0.35)
        self.declare_parameter("pause_after_turn_s", 0.9)
        self.declare_parameter("max_heading_jump_deg", 40.0)
        self.declare_parameter("heading_filter_alpha", 0.35)
        self.declare_parameter("reached_publish_period_s", 1.0)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.goal_dwell_s = float(self.get_parameter("goal_dwell_s").value)
        self.align_threshold = float(self.get_parameter("align_threshold_deg").value)
        self.coarse_align_threshold = float(
            self.get_parameter("coarse_align_threshold_deg").value
        )
        self.approach_align_distance = float(
            self.get_parameter("approach_align_distance_m").value
        )
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.drive_correction_gain = float(self.get_parameter("drive_correction_gain").value)
        self.max_drive_angular = float(self.get_parameter("max_drive_angular").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)
        self.turn_burst_s = float(self.get_parameter("turn_burst_s").value)
        self.pause_after_turn_s = float(self.get_parameter("pause_after_turn_s").value)
        self.max_heading_jump = float(self.get_parameter("max_heading_jump_deg").value)
        self.heading_filter_alpha = float(self.get_parameter("heading_filter_alpha").value)
        self.reached_publish_period_s = float(self.get_parameter("reached_publish_period_s").value)
        loop_hz = max(1.0, float(self.get_parameter("control_loop_hz").value))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(NavSatFix, "earth_rover/gps", self._on_gps, sensor_qos)
        self.create_subscription(Float32, "earth_rover/heading", self._on_heading, sensor_qos)
        self.create_subscription(NavSatFix, "earth_rover/target_waypoint", self._on_target, sensor_qos)
        self.create_subscription(Bool, "earth_rover/navigation_pause", self._on_navigation_pause, 10)
        self.create_subscription(String, "earth_rover/waypoint_status", self._on_mission_status, 10)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "earth_rover/waypoint_status", 10)

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self._raw_heading = None

        self.target_lat = None
        self.target_lon = None
        self.active_goal = False
        self._reached_since = None
        self._awaiting_next_target = False
        self._last_reached_publish_at = None
        self._align_phase = "PAUSE"
        self._align_phase_started_at = None
        self._burst_turn_sign = 0
        self._navigation_paused = False

        self.timer = self.create_timer(1.0 / loop_hz, self._control_loop)
        self.get_logger().info(
            "GPS Waypoint Controller ready "
            f"(burst={self.turn_burst_s:.2f}s, pause={self.pause_after_turn_s:.2f}s, "
            f"invert={self.invert_angular})"
        )

    def _on_gps(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _on_heading(self, msg: Float32):
        raw = float(msg.data) % 360.0
        self._raw_heading = raw

        if self.current_heading is None:
            self.current_heading = raw
            return

        jump = abs(self.angle_error_deg(raw, self.current_heading))
        if jump > self.max_heading_jump:
            return

        delta = self.angle_error_deg(raw, self.current_heading)
        self.current_heading = (self.current_heading + self.heading_filter_alpha * delta) % 360.0

    def _on_target(self, msg: NavSatFix):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.active_goal = True
        self._reached_since = None
        self._awaiting_next_target = False
        self._last_reached_publish_at = None
        self._align_phase = "PAUSE"
        self._align_phase_started_at = None
        self._burst_turn_sign = 0
        self._navigation_paused = False
        self._stop_robot()
        self.get_logger().info(f"New target: ({self.target_lat}, {self.target_lon})")

    def _on_navigation_pause(self, msg: Bool):
        self._navigation_paused = bool(msg.data)
        if self._navigation_paused:
            self._stop_robot()
            self.get_logger().info("Navigation paused by mission manager")

    def _on_mission_status(self, msg: String):
        if msg.data == "MISSION_FINISHED":
            self._awaiting_next_target = False
            self._navigation_paused = True
            self.active_goal = False
            self._stop_robot()
            self.get_logger().info("Mission finished — stopped publishing REACHED")

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
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

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        dlon = math.radians(lon2 - lon1)
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)
        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def angle_error_deg(target_deg, current_deg):
        return (target_deg - current_deg + 540.0) % 360.0 - 180.0

    def _apply_angular_sign(self, angular):
        return -angular if self.invert_angular else angular

    def _phase_elapsed(self, now):
        if self._align_phase_started_at is None:
            return 0.0
        return (now - self._align_phase_started_at).nanoseconds / 1e9

    def _begin_align_phase(self, phase, now):
        self._align_phase = phase
        self._align_phase_started_at = now

    def _publish_stop_and_status(self, status_text):
        twist = Twist()
        self.cmd_pub.publish(twist)
        out = String()
        out.data = status_text
        self.status_pub.publish(out)
        self.get_logger().info(status_text, throttle_duration_sec=2)

    def _publish_reached(self, now):
        self._stop_robot()
        msg = String()
        msg.data = "REACHED"
        self.status_pub.publish(msg)
        self._last_reached_publish_at = now
        self.get_logger().info("Signaled REACHED to mission_manager (not SDK POST yet)")

    def _control_loop(self):
        now = self.get_clock().now()

        if self._navigation_paused:
            self._stop_robot()
            return

        if self._awaiting_next_target:
            self._stop_robot()
            elapsed = 0.0
            if self._last_reached_publish_at is not None:
                elapsed = (now - self._last_reached_publish_at).nanoseconds / 1e9
            if elapsed >= self.reached_publish_period_s:
                self._publish_reached(now)
            return

        if not self.active_goal or self.current_lat is None or self.current_lon is None:
            return

        distance = self.haversine_distance(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )

        if distance <= self.goal_tolerance:
            self._stop_robot()
            if self._reached_since is None:
                self._reached_since = now
            if (now - self._reached_since).nanoseconds / 1e9 >= self.goal_dwell_s:
                self.get_logger().info(f"Target reached ({distance:.1f}m)")
                self.active_goal = False
                self._awaiting_next_target = True
                self._navigation_paused = True
                self._publish_reached(now)
            return

        self._reached_since = None

        if self.current_heading is None:
            self._stop_robot()
            return

        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )
        heading_error = self.angle_error_deg(bearing, self.current_heading)

        align_threshold = (
            self.align_threshold
            if distance <= self.approach_align_distance
            else self.coarse_align_threshold
        )

        twist = Twist()
        if abs(heading_error) > align_threshold:
            mode = "ALIGN"
            twist.linear.x = 0.0
            elapsed = self._phase_elapsed(now)

            if self._align_phase == "PAUSE":
                twist.angular.z = 0.0
                if self._align_phase_started_at is None or elapsed >= self.pause_after_turn_s:
                    self._burst_turn_sign = 1 if heading_error > 0.0 else -1
                    self._begin_align_phase("TURN", now)
                    twist.angular.z = self._apply_angular_sign(self._burst_turn_sign * self.turn_speed)
            else:
                twist.angular.z = self._apply_angular_sign(self._burst_turn_sign * self.turn_speed)
                if elapsed >= self.turn_burst_s:
                    self._begin_align_phase("PAUSE", now)
                    twist.angular.z = 0.0
        else:
            mode = "DRIVE"
            self._align_phase = "PAUSE"
            self._align_phase_started_at = None
            self._burst_turn_sign = 0
            twist.linear.x = self.forward_speed
            correction = self.drive_correction_gain * heading_error
            twist.angular.z = self._apply_angular_sign(
                max(-self.max_drive_angular, min(self.max_drive_angular, correction))
            )

        self.cmd_pub.publish(twist)

        raw_h = self._raw_heading if self._raw_heading is not None else float("nan")
        status = (
            f"{mode}/{self._align_phase}: dist={distance:.1f}m, bearing={bearing:.1f}, "
            f"heading={self.current_heading:.1f}, raw={raw_h:.1f}, err={heading_error:+.1f}, "
            f"thr={align_threshold:.0f}, linear={twist.linear.x:.2f}, angular={twist.angular.z:+.2f}"
        )
        out = String()
        out.data = status
        self.status_pub.publish(out)
        self.get_logger().info(status, throttle_duration_sec=2)

    def _stop_robot(self):
        self._align_phase = "PAUSE"
        self._align_phase_started_at = None
        self._burst_turn_sign = 0
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointController()
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
