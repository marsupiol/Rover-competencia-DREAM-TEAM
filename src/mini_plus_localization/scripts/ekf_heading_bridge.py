#!/usr/bin/env python3
"""Republishes the EKF's fused absolute yaw (ENU, from ekf_filter_node_map's
/odometry/global) as a compass heading in degrees (0=North, clockwise), on
the same topic/type gps_waypoint_controller already expects
(earth_rover/heading, std_msgs/Float32) — heading publisher in BEST_EFFORT.
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32


class EkfHeadingBridge(Node):
    def __init__(self):
        super().__init__("ekf_heading_bridge")

        # 1. Suscripción al EKF global (Mantiene RELIABLE porque emite datos críticos)
        in_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # 2. Publicación hacia el controlador (Cambiado a BEST_EFFORT para alinearse con tu configuración)
        out_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.heading_pub = self.create_publisher(Float32, "earth_rover/heading", out_qos)
        self.create_subscription(Odometry, "odometry/global", self._on_odom, in_qos)

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # Extracción de yaw desde el cuaternión (ENU, 0=East CCW)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_enu = math.atan2(siny_cosp, cosy_cosp)

        # Conversión a rumbo de brújula (0=North, clockwise)
        heading_deg = (90.0 - math.degrees(yaw_enu)) % 360.0

        out = Float32()
        out.data = heading_deg
        self.heading_pub.publish(out)


def main(argv=None):
    rclpy.init(args=argv)
    node = EkfHeadingBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()