#!/usr/bin/env python3
"""GPS Waypoint Navigation Controller for Earth Rover."""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String


class GPSWaypointController(Node):
    def __init__(self):
        super().__init__("gps_waypoint_controller")

        self.declare_parameter("goal_tolerance_m", 3.0)
        self.declare_parameter("goal_dwell_s", 1.5)
        self.declare_parameter("max_linear_speed", 0.55)
        self.declare_parameter("min_linear_speed", 0.22)
        self.declare_parameter("max_angular_speed", 0.45)
        self.declare_parameter("max_drive_angular_speed", 0.22)
        self.declare_parameter("kp_angular", 0.006)
        self.declare_parameter("kd_angular", 0.04)
        self.declare_parameter("kp_linear", 0.05)
        self.declare_parameter("turn_in_place_threshold_deg", 28.0)
        self.declare_parameter("drive_heading_deadband_deg", 6.0)
        self.declare_parameter("max_steering_error_deg", 90.0)
        self.declare_parameter("heading_filter_alpha", 0.18)
        self.declare_parameter("angular_filter_alpha", 0.5)
        self.declare_parameter("heading_offset_deg", 0.0)
        self.declare_parameter("invert_angular", False)
        self.declare_parameter("stuck_timeout_s", 8.0)
        self.declare_parameter("progress_epsilon_m", 0.8)
        self.declare_parameter("recovery_backup_s", 1.8)
        self.declare_parameter("recovery_turn_s", 2.5)
        self.declare_parameter("recovery_reverse_speed", -0.25)
        self.declare_parameter("recovery_turn_speed", 0.45)

        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.goal_dwell_s = float(self.get_parameter("goal_dwell_s").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.min_linear = float(self.get_parameter("min_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.max_drive_angular = float(self.get_parameter("max_drive_angular_speed").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)
        self.kd_angular = float(self.get_parameter("kd_angular").value)
        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.turn_in_place_threshold_deg = float(
            self.get_parameter("turn_in_place_threshold_deg").value
        )
        self.drive_heading_deadband_deg = float(self.get_parameter("drive_heading_deadband_deg").value)
        self.max_steering_error_deg = float(self.get_parameter("max_steering_error_deg").value)
        self.heading_filter_alpha = float(self.get_parameter("heading_filter_alpha").value)
        self.angular_filter_alpha = float(self.get_parameter("angular_filter_alpha").value)
        self.heading_offset_deg = float(self.get_parameter("heading_offset_deg").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)
        self.stuck_timeout_s = float(self.get_parameter("stuck_timeout_s").value)
        self.progress_epsilon_m = float(self.get_parameter("progress_epsilon_m").value)
        self.recovery_backup_s = float(self.get_parameter("recovery_backup_s").value)
        self.recovery_turn_s = float(self.get_parameter("recovery_turn_s").value)
        self.recovery_reverse_speed = float(self.get_parameter("recovery_reverse_speed").value)
        self.recovery_turn_speed = float(self.get_parameter("recovery_turn_speed").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(NavSatFix, "earth_rover/gps", self._on_gps, sensor_qos)
        self.create_subscription(Float32, "earth_rover/heading", self._on_heading, sensor_qos)
        self.create_subscription(NavSatFix, "earth_rover/target_waypoint", self._on_target, sensor_qos)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "earth_rover/waypoint_status", 10)

        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.filtered_heading = None

        self.target_lat = None
        self.target_lon = None
        self.active_goal = False
        self._target_received_at = None
        self._reached_since = None
        self._best_distance = None
        self._last_progress_at = None
        self._recovery_phase = None
        self._recovery_started_at = None
        self._recovery_turn_sign = 1.0
        self._last_angular_cmd = 0.0
        self._last_heading_error = 0.0
        self._last_control_at = None

        self.timer = self.create_timer(0.1, self._control_loop)  # 10 Hz control loop
        self.get_logger().info("GPS Waypoint Controller Initialized")

    def _on_gps(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def _on_heading(self, msg: Float32):
        self.current_heading = msg.data
        if self.filtered_heading is None:
            self.filtered_heading = self.current_heading % 360.0
            return

        delta = self._angle_error_deg(self.current_heading, self.filtered_heading)
        self.filtered_heading = (self.filtered_heading + self.heading_filter_alpha * delta) % 360.0

    def _on_target(self, msg: NavSatFix):
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.active_goal = True
        self._target_received_at = self.get_clock().now()
        self._reached_since = None
        self._best_distance = None
        self._last_progress_at = self._target_received_at
        self._recovery_phase = None
        self._recovery_started_at = None
        self._last_heading_error = 0.0
        self._last_angular_cmd = 0.0
        self.get_logger().info(f"New Target Waypoint Received: ({self.target_lat}, {self.target_lon})")

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        R = 6371000.0  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (
            math.sin(dlat / 2.0) ** 2
            + math.cos(math.radians(lat1))
            * math.cos(math.radians(lat2))
            * math.sin(dlon / 2.0) ** 2
        )
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return R * c

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        dlon = math.radians(lon2 - lon1)
        lat1_r = math.radians(lat1)
        lat2_r = math.radians(lat2)

        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360.0) % 360.0

    def _control_loop(self):
        if not self.active_goal or self.current_lat is None or self.current_lon is None:
            return

        distance = self.haversine_distance(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )

        now = self.get_clock().now()
        if self._run_recovery_if_active(now):
            return

        if distance <= self.goal_tolerance:
            if self._reached_since is None:
                self._reached_since = now

            reached_for = (now - self._reached_since).nanoseconds / 1e9
            if reached_for >= self.goal_dwell_s:
                self.get_logger().info(
                    f"Target reached! Distance: {distance:.2f}m <= {self.goal_tolerance}m "
                    f"for {reached_for:.1f}s"
                )
                self._stop_robot()
                self.active_goal = False
                status_msg = String()
                status_msg.data = "REACHED"
                self.status_pub.publish(status_msg)
                return
        else:
            self._reached_since = None

        if self._is_stuck(distance, now):
            self._start_recovery(now)
            return

        if self.filtered_heading is None:
            # If no heading available, move slowly forward
            twist = Twist()
            twist.linear.x = self.min_linear
            self.cmd_pub.publish(twist)
            return

        bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, self.target_lat, self.target_lon
        )

        corrected_heading = (self.filtered_heading + self.heading_offset_deg) % 360.0
        heading_error = self._angle_error_deg(bearing, corrected_heading)
        steering_error = max(
            -self.max_steering_error_deg,
            min(self.max_steering_error_deg, heading_error),
        )

        dt = 0.1
        if self._last_control_at is not None:
            dt = max(0.05, (now - self._last_control_at).nanoseconds / 1e9)
        self._last_control_at = now

        heading_error_rate = self._angle_error_deg(heading_error, self._last_heading_error) / dt
        self._last_heading_error = heading_error

        twist = Twist()
        abs_error = abs(heading_error)

        if abs_error > self.turn_in_place_threshold_deg:
            mode = "TURN_IN_PLACE"
            twist.linear.x = 0.0
            angular = self.kp_angular * steering_error + self.kd_angular * heading_error_rate
            angular_limit = self.max_angular
        else:
            mode = "DRIVE"
            alignment = max(0.25, 1.0 - abs_error / self.turn_in_place_threshold_deg)
            linear = min(self.max_linear, self.kp_linear * distance) * alignment
            twist.linear.x = max(self.min_linear * alignment, linear)

            if abs_error <= self.drive_heading_deadband_deg:
                angular = 0.0
            else:
                angular = self.kp_angular * steering_error + self.kd_angular * heading_error_rate
            angular_limit = self.max_drive_angular

        if self.invert_angular:
            angular *= -1.0

        angular = max(-angular_limit, min(angular_limit, angular))
        twist.angular.z = (
            self.angular_filter_alpha * angular
            + (1.0 - self.angular_filter_alpha) * self._last_angular_cmd
        )
        self._last_angular_cmd = twist.angular.z

        self.cmd_pub.publish(twist)

        status_msg = String()
        status_msg.data = (
            f"{mode}: dist={distance:.1f}m, bearing={bearing:.1f}deg, "
            f"heading={self.current_heading:.1f}deg, filtered={self.filtered_heading:.1f}deg, "
            f"corrected={corrected_heading:.1f}deg, "
            f"err={heading_error:.1f}deg, "
            f"linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}"
        )
        self.status_pub.publish(status_msg)
        self.get_logger().info(status_msg.data, throttle_duration_sec=2)

    def _is_stuck(self, distance, now):
        if self._best_distance is None:
            self._best_distance = distance
            self._last_progress_at = now
            return False

        if distance < self._best_distance - self.progress_epsilon_m:
            self._best_distance = distance
            self._last_progress_at = now
            return False

        if self._last_progress_at is None:
            self._last_progress_at = now
            return False

        no_progress_for = (now - self._last_progress_at).nanoseconds / 1e9
        return no_progress_for >= self.stuck_timeout_s and distance > self.goal_tolerance

    def _start_recovery(self, now):
        self._recovery_phase = "BACKUP"
        self._recovery_started_at = now
        self._recovery_turn_sign *= -1.0
        self._stop_robot()
        self.get_logger().warning(
            "No progress toward waypoint; starting recovery: backup then turn. "
            f"best_dist={self._best_distance:.1f}m"
        )

    def _run_recovery_if_active(self, now):
        if self._recovery_phase is None or self._recovery_started_at is None:
            return False

        elapsed = (now - self._recovery_started_at).nanoseconds / 1e9
        twist = Twist()

        if self._recovery_phase == "BACKUP":
            if elapsed < self.recovery_backup_s:
                twist.linear.x = self.recovery_reverse_speed
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                self._publish_status("RECOVERY: backing up")
                return True

            self._recovery_phase = "TURN"
            self._recovery_started_at = now
            return True

        if self._recovery_phase == "TURN":
            if elapsed < self.recovery_turn_s:
                twist.linear.x = 0.0
                twist.angular.z = self._recovery_turn_sign * self.recovery_turn_speed
                if self.invert_angular:
                    twist.angular.z *= -1.0
                self.cmd_pub.publish(twist)
                self._publish_status("RECOVERY: turning away")
                return True

            self._recovery_phase = None
            self._recovery_started_at = None
            self._best_distance = None
            self._last_progress_at = now
            self._stop_robot()
            self.get_logger().info("Recovery complete; resuming waypoint navigation")
            return True

        self._recovery_phase = None
        return False

    def _publish_status(self, text):
        status_msg = String()
        status_msg.data = text
        self.status_pub.publish(status_msg)
        self.get_logger().warning(text, throttle_duration_sec=1)

    @staticmethod
    def _angle_error_deg(target, current):
        return (target - current + 540.0) % 360.0 - 180.0

    def _stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)


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
