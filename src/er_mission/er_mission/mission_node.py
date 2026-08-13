#!/usr/bin/env python3
"""ARCHIVO NO USADO — no está registrado en setup.py como entry point y no se ejecuta.
Contiene además un bug (se suscribe a Float64 en vez de Float32 para heading).
Se mantiene solo como referencia histórica; usar mission_manager_node.py + gps_waypoint_controller.py en su lugar.

Mission manager node for Earth Rover Mission 1.

This node orchestrates the full mission lifecycle:
1. Starts the mission via the Earth Rovers SDK REST API.
2. Retrieves the target waypoint (hard‑coded via parameters for the minimal implementation).
3. Uses a simple proportional controller (Haversine distance + bearing) to drive the rover
   towards the waypoint, publishing geometry_msgs/Twist on `/cmd_vel`.
4. When the waypoint is reached, notifies the SDK with `POST /checkpoint-reached`.
5. Returns to the HOME location (also provided via parameters).
6. Ends the mission with `POST /end-mission`.

All network interaction is performed with the `requests` library. Errors are logged
and the node shuts down gracefully.
"""

import math
import time
import threading
import requests

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# ---------------------------------------------------------------------------
# Helper functions for geodesic calculations (Haversine & bearing)
# ---------------------------------------------------------------------------

def haversine(lat1, lon1, lat2, lon2):
    """Return distance in meters between two lat/lon points using the Haversine formula."""
    R = 6371000.0  # Earth radius in metres
    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)
    a = math.sin(Δφ / 2) ** 2 + math.cos(φ1) * math.cos(φ2) * math.sin(Δλ / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def bearing(lat1, lon1, lat2, lon2):
    """Return bearing (rad) from point 1 to point 2."""
    φ1 = math.radians(lat1)
    φ2 = math.radians(lat2)
    Δλ = math.radians(lon2 - lon1)
    y = math.sin(Δλ) * math.cos(φ2)
    x = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(Δλ)
    return math.atan2(y, x)  # radians

# ---------------------------------------------------------------------------
# MissionNode definition
# ---------------------------------------------------------------------------

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sdk_base_url', 'http://127.0.0.1:8000'),
                ('home_lat', 0.0),
                ('home_lon', 0.0),
                ('checkpoint_lat', 0.0),
                ('checkpoint_lon', 0.0),
                ('distance_tolerance', 0.5),  # meters
                ('linear_kp', 0.5),
                ('angular_kp', 1.0),
            ],
        )
        # Load parameters
        self.sdk_base_url = self.get_parameter('sdk_base_url').get_parameter_value().string_value
        self.home_lat = self.get_parameter('home_lat').get_parameter_value().double_value
        self.home_lon = self.get_parameter('home_lon').get_parameter_value().double_value
        self.checkpoint_lat = self.get_parameter('checkpoint_lat').get_parameter_value().double_value
        self.checkpoint_lon = self.get_parameter('checkpoint_lon').get_parameter_value().double_value
        self.tol = self.get_parameter('distance_tolerance').get_parameter_value().double_value
        self.kp_lin = self.get_parameter('linear_kp').get_parameter_value().double_value
        self.kp_ang = self.get_parameter('angular_kp').get_parameter_value().double_value

        # Publishers / Subscribers
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', qos)
        self.gps_sub = self.create_subscription(NavSatFix, '/earth_rover/gps', self.gps_cb, qos)
        self.heading_sub = self.create_subscription(Float64, '/earth_rover/heading', self.heading_cb, qos)

        # State variables
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None  # radians, 0 = north
        self.mission_phase = 'START'  # START, TO_CHECKPOINT, TO_HOME, DONE
        self.timer = self.create_timer(0.2, self.control_loop)

        # Start mission on node creation
        self.start_mission()

    # ---------------------------------------------------------------------
    # ROS callbacks
    # ---------------------------------------------------------------------
    def gps_cb(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_cb(self, msg: Float64):
        # SDK sends heading in degrees; convert to radians
        self.current_heading = math.radians(msg.data)

    # ---------------------------------------------------------------------
    # SDK Interaction helpers
    # ---------------------------------------------------------------------
    def _post(self, endpoint, json_payload=None):
        url = f"{self.sdk_base_url}{endpoint}"
        try:
            resp = requests.post(url, json=json_payload, timeout=5)
            resp.raise_for_status()
            self.get_logger().info(f"POST {endpoint} succeeded: {resp.text}")
            return resp.json() if resp.text else None
        except Exception as e:
            self.get_logger().error(f"POST {endpoint} failed: {e}")
            return None

    def start_mission(self):
        self.get_logger().info('Starting mission via SDK...')
        self._post('/start-mission')
        self.mission_phase = 'TO_CHECKPOINT'

    def report_checkpoint(self):
        self.get_logger().info('Reporting checkpoint reached to SDK...')
        self._post('/checkpoint-reached')

    def end_mission(self):
        self.get_logger().info('Ending mission via SDK...')
        self._post('/end-mission')

    # ---------------------------------------------------------------------
    # Control logic
    # ---------------------------------------------------------------------
    def control_loop(self):
        if self.current_lat is None or self.current_lon is None or self.current_heading is None:
            return  # wait for all data

        # Choose target based on phase
        if self.mission_phase == 'TO_CHECKPOINT':
            target_lat = self.checkpoint_lat
            target_lon = self.checkpoint_lon
        elif self.mission_phase == 'TO_HOME':
            target_lat = self.home_lat
            target_lon = self.home_lon
        else:
            return

        dist = haversine(self.current_lat, self.current_lon, target_lat, target_lon)
        bear = bearing(self.current_lat, self.current_lon, target_lat, target_lon)
        angle_error = self._angle_diff(bear, self.current_heading)

        # Simple proportional controller
        twist = Twist()
        if dist > self.tol:
            twist.linear.x = min(self.kp_lin * dist, 1.0)  # cap speed
            twist.angular.z = self.kp_ang * angle_error
        else:
            # Reached target
            if self.mission_phase == 'TO_CHECKPOINT':
                self.report_checkpoint()
                self.mission_phase = 'TO_HOME'
            elif self.mission_phase == 'TO_HOME':
                self.end_mission()
                self.mission_phase = 'DONE'
                self.get_logger().info('Mission completed. Shutting down node.')
                self.destroy_node()
                rclpy.shutdown()
                return
        self.cmd_pub.publish(twist)

    @staticmethod
    def _angle_diff(target, current):
        """Smallest signed angle difference (target - current) in radians."""
        a = (target - current + math.pi) % (2 * math.pi) - math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
