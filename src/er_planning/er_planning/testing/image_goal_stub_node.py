#!/usr/bin/env python3
"""
image_goal_stub_node.py — Nodo stub desechable para pruebas de Image-Goal (Track 04).

Simula la interfaz del futuro modelo de navegación por metas visuales (image-goal):
  - Publica earth_rover/image_goal_relative (geometry_msgs/Vector3: x=x_right_m, y=y_forward_m).
  - Publica earth_rover/image_goal_ready (std_msgs/Bool).

Permite conmutar 'ready' y la posición relativa de la meta en tiempo de ejecución
vía parámetros de ROS para validar la máquina de selección de meta de bev_planner_node.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class ImageGoalStubNode(Node):
    def __init__(self):
        super().__init__("image_goal_stub_node")

        # Declaración de Parámetros
        self.declare_parameter("ready", False)
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 3.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("relative_goal_topic", "earth_rover/image_goal_relative")
        self.declare_parameter("ready_topic", "earth_rover/image_goal_ready")

        self.ready = bool(self.get_parameter("ready").value)
        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        relative_goal_topic = str(self.get_parameter("relative_goal_topic").value)
        ready_topic = str(self.get_parameter("ready_topic").value)

        # Configurar callback de cambio de parámetros en caliente
        self.add_on_set_parameters_callback(self._on_set_params)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.goal_pub = self.create_publisher(Vector3, relative_goal_topic, qos)
        self.ready_pub = self.create_publisher(Bool, ready_topic, qos)

        period = 1.0 / max(1.0, self.publish_rate_hz)
        self._timer = self.create_timer(period, self._publish_stub_data)

        self.get_logger().info(
            f"ImageGoalStubNode iniciado | ready={self.ready} | goal=({self.goal_x:+.2f}m, {self.goal_y:+.2f}m) @ {self.publish_rate_hz}Hz"
        )

    def _on_set_params(self, params):
        for p in params:
            if p.name == "ready":
                self.ready = bool(p.value)
                self.get_logger().info(f"[PARAM] ready actualizado a: {self.ready}")
            elif p.name == "goal_x":
                self.goal_x = float(p.value)
                self.get_logger().info(f"[PARAM] goal_x actualizado a: {self.goal_x:+.2f}m")
            elif p.name == "goal_y":
                self.goal_y = float(p.value)
                self.get_logger().info(f"[PARAM] goal_y actualizado a: {self.goal_y:+.2f}m")
        return SetParametersResult(successful=True)

    def _publish_stub_data(self):
        # 1. Publicar meta relativa (GeNIE frame: +X derecha, +Y adelante)
        goal_msg = Vector3()
        goal_msg.x = float(self.goal_x)
        goal_msg.y = float(self.goal_y)
        goal_msg.z = 0.0
        self.goal_pub.publish(goal_msg)

        # 2. Publicar estado de disponibilidad
        ready_msg = Bool()
        ready_msg.data = bool(self.ready)
        self.ready_pub.publish(ready_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageGoalStubNode()
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
