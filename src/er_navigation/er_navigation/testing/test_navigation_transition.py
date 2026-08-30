#!/usr/bin/env python3
"""
test_navigation_transition.py — Test offline de verificación de estados de transición (Brief 3).

Valida la máquina de estados de gps_waypoint_controller ante cambios en navigation_mode="auto":
  1. Fase 1: Operación normal Outdoor (gps_reliable=True, goal_source_ready=True) -> DRIVE (cmd_v > 0)
  2. Fase 2: Forzado gps_reliable=False -> Entra en TRANSITIONING_TO_INDOOR:
     - FRENADO TOTAL CONFIRMADO: cmd_v == 0.00 y cmd_w == 0.00 (sin giro alguno)
     - Permanece frenado mientras goal_source_ready == False
  3. Fase 3: goal_source_ready=True (Image-Goal listo) -> Sale de transición y retoma movimiento (DRIVE/ALIGN)
  4. Fase 4: Recuperación de GPS (gps_reliable=True) -> Entra en TRANSITIONING_TO_OUTDOOR (frenado total)
  5. Fase 5: goal_source_ready=True (GPS listo) -> Sale de transición y retoma movimiento normal
"""

from __future__ import annotations

import json
import math
import os
import sys
import time

# Agregar directorio del paquete al path
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_THIS_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String


class TransitionTestHarness(Node):
    def __init__(self):
        super().__init__("transition_test_harness")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # Publicadores hacia el controlador
        self.gps_pub = self.create_publisher(NavSatFix, "gps/filtered", sensor_qos)
        self.heading_pub = self.create_publisher(Float32, "earth_rover/heading", sensor_qos)
        self.target_pub = self.create_publisher(NavSatFix, "earth_rover/target_waypoint", reliable_qos)
        self.path_pub = self.create_publisher(Path, "earth_rover/planned_path", sensor_qos)
        self.valid_pub = self.create_publisher(Bool, "earth_rover/planner_valid", sensor_qos)
        self.gps_reliable_pub = self.create_publisher(Bool, "earth_rover/gps_reliable", reliable_qos)
        self.goal_source_ready_pub = self.create_publisher(Bool, "earth_rover/goal_source_ready", sensor_qos)

        # Suscriptores a salidas del controlador
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, reliable_qos)
        self.debug_sub = self.create_subscription(String, "earth_rover/control_debug", self._on_debug, sensor_qos)

        self.last_cmd_v: float | None = None
        self.last_cmd_w: float | None = None
        self.last_mode: str | None = None
        self.cmd_history: list[tuple[float, float, str | None]] = []

    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd_v = float(msg.linear.x)
        self.last_cmd_w = float(msg.angular.z)
        self.cmd_history.append((self.last_cmd_v, self.last_cmd_w, self.last_mode))

    def _on_debug(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.last_mode = data.get("mode")
        except Exception:
            pass

    def publish_cycle(
        self,
        gps_reliable: bool,
        goal_source_ready: bool,
        heading_deg: float = 0.0,
    ):
        now = self.get_clock().now().to_msg()

        # GPS y Target (20m al Norte)
        gps = NavSatFix()
        gps.header.stamp = now
        gps.latitude = -34.603700
        gps.longitude = -58.381600
        self.gps_pub.publish(gps)

        h = Float32()
        h.data = float(heading_deg)
        self.heading_pub.publish(h)

        # Señales de confiabilidad y disponibilidad
        rel_msg = Bool()
        rel_msg.data = bool(gps_reliable)
        self.gps_reliable_pub.publish(rel_msg)

        rdy_msg = Bool()
        rdy_msg.data = bool(goal_source_ready)
        self.goal_source_ready_pub.publish(rdy_msg)

        # Path BEV simulado hacia adelante (2m recto)
        path = Path()
        path.header.stamp = now
        path.header.frame_id = "base_link"
        for i in range(5):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(i) * 0.5
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

        valid_msg = Bool()
        valid_msg.data = True
        self.valid_pub.publish(valid_msg)

    def publish_target(self):
        now = self.get_clock().now().to_msg()
        target = NavSatFix()
        target.header.stamp = now
        target.latitude = -34.603500  # ~22m al Norte
        target.longitude = -58.381600
        self.target_pub.publish(target)


def run_transition_tests():
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init()

    harness = TransitionTestHarness()

    from er_navigation.gps_waypoint_controller import GPSWaypointController
    controller = GPSWaypointController()

    # Configurar navigation_mode = "auto"
    controller.navigation_mode = "auto"
    param = Parameter("navigation_mode", Parameter.Type.STRING, "auto")
    controller.set_parameters([param])

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(controller)

    print("\n" + "=" * 80)
    print(" INICIANDO TEST OFFLINE: ESTADOS DE TRANSICIÓN Y FRENADO TOTAL (BRIEF 3)")
    print("=" * 80)

    # Fijar meta inicial una sola vez
    harness.publish_target()
    for _ in range(5):
        executor.spin_once(timeout_sec=0.02)
        time.sleep(0.02)

    def step(duration_s: float, gps_rel: bool, goal_ready: bool, heading_deg: float = 0.0):
        end_t = time.time() + duration_s
        while time.time() < end_t:
            harness.publish_cycle(gps_reliable=gps_rel, goal_source_ready=goal_ready, heading_deg=heading_deg)
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.05)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.01)

    # --------------------------------------------------------------------------
    # FASE 1: NAVEGACIÓN NORMAL OUTDOOR
    # --------------------------------------------------------------------------
    print("\n[FASE 1] Operación normal Outdoor (gps_reliable=True, goal_source_ready=True)")
    step(1.5, gps_rel=True, goal_ready=True)
    print(f"         cmd_v={harness.last_cmd_v:.2f} m/s, cmd_w={harness.last_cmd_w:+.2f} rad/s, mode={harness.last_mode}")
    assert harness.last_cmd_v is not None and harness.last_cmd_v > 0.0, "Fallo Fase 1: El rover debe estar avanzando en DRIVE"
    assert harness.last_mode == "DRIVE" or harness.last_mode == "ALIGN", f"Fallo Fase 1: Modo inesperado {harness.last_mode}"

    # --------------------------------------------------------------------------
    # FASE 2: DEGRADACIÓN GPS -> TRANSITIONING_TO_INDOOR (FRENADO TOTAL)
    # --------------------------------------------------------------------------
    print("\n[FASE 2] Corte de GPS (gps_reliable=False, goal_source_ready=False)")
    harness.cmd_history.clear()
    step(1.5, gps_rel=False, goal_ready=False)
    print(f"         cmd_v={harness.last_cmd_v:.2f} m/s, cmd_w={harness.last_cmd_w:+.2f} rad/s, mode={harness.last_mode}")

    assert harness.last_mode == "TRANSITIONING_TO_INDOOR", f"Fallo Fase 2: Debe entrar en TRANSITIONING_TO_INDOOR, actual={harness.last_mode}"
    assert harness.last_cmd_v == 0.0, f"Fallo Fase 2: Linear velocity debe ser 0.00, actual={harness.last_cmd_v}"
    assert harness.last_cmd_w == 0.0, f"Fallo Fase 2: Angular velocity debe ser 0.00 (SIN GIRO), actual={harness.last_cmd_w}"

    # Verificar que en TODOS los ciclos de la Fase 2 ambos comandos fueron estrictamente cero
    for v, w, mode in harness.cmd_history:
        if mode == "TRANSITIONING_TO_INDOOR":
            assert v == 0.0 and w == 0.0, f"Fallo Fase 2: Comando no cero detectado durante transición: v={v}, w={w}"
    print("         -> Verificación exitosa: 100% de los comandos durante TRANSITIONING_TO_INDOOR tuvieron v=0.00 y w=0.00")

    # --------------------------------------------------------------------------
    # FASE 3: IMAGE GOAL LISTO -> RETOMA MOVIMIENTO
    # --------------------------------------------------------------------------
    print("\n[FASE 3] Image-Goal Confirmado Listo (goal_source_ready=True, gps_reliable=False)")
    step(1.5, gps_rel=False, goal_ready=True)
    print(f"         cmd_v={harness.last_cmd_v:.2f} m/s, cmd_w={harness.last_cmd_w:+.2f} rad/s, mode={harness.last_mode}")
    assert harness.last_cmd_v is not None and harness.last_cmd_v > 0.0, "Fallo Fase 3: El rover debe retomar el avance"
    assert harness.last_mode in ("DRIVE", "ALIGN"), f"Fallo Fase 3: Debe estar en DRIVE/ALIGN, actual={harness.last_mode}"

    # --------------------------------------------------------------------------
    # FASE 4: RECUPERACIÓN GPS -> TRANSITIONING_TO_OUTDOOR (FRENADO TOTAL)
    # --------------------------------------------------------------------------
    print("\n[FASE 4] Recuperación de GPS (gps_reliable=True, goal_source_ready=False)")
    harness.cmd_history.clear()
    step(1.5, gps_rel=True, goal_ready=False)
    print(f"         cmd_v={harness.last_cmd_v:.2f} m/s, cmd_w={harness.last_cmd_w:+.2f} rad/s, mode={harness.last_mode}")

    assert harness.last_mode == "TRANSITIONING_TO_OUTDOOR", f"Fallo Fase 4: Debe entrar en TRANSITIONING_TO_OUTDOOR, actual={harness.last_mode}"
    assert harness.last_cmd_v == 0.0, f"Fallo Fase 4: Linear velocity debe ser 0.00, actual={harness.last_cmd_v}"
    assert harness.last_cmd_w == 0.0, f"Fallo Fase 4: Angular velocity debe ser 0.00 (SIN GIRO), actual={harness.last_cmd_w}"

    for v, w, mode in harness.cmd_history:
        if mode == "TRANSITIONING_TO_OUTDOOR":
            assert v == 0.0 and w == 0.0, f"Fallo Fase 4: Comando no cero detectado durante transición: v={v}, w={w}"
    print("         -> Verificación exitosa: 100% de los comandos durante TRANSITIONING_TO_OUTDOOR tuvieron v=0.00 y w=0.00")

    # --------------------------------------------------------------------------
    # FASE 5: GPS LISTO -> RETOMA NAVEGACIÓN NORMAL OUTDOOR
    # --------------------------------------------------------------------------
    print("\n[FASE 5] Fuente Outdoor Confirmada Lista (goal_source_ready=True, gps_reliable=True)")
    step(1.5, gps_rel=True, goal_ready=True)
    print(f"         cmd_v={harness.last_cmd_v:.2f} m/s, cmd_w={harness.last_cmd_w:+.2f} rad/s, mode={harness.last_mode}")
    assert harness.last_cmd_v is not None and harness.last_cmd_v > 0.0, "Fallo Fase 5: El rover debe retomar el avance normal"
    assert harness.last_mode in ("DRIVE", "ALIGN"), f"Fallo Fase 5: Debe estar en DRIVE/ALIGN, actual={harness.last_mode}"

    print("\n" + "=" * 80)
    print(" TODAS LAS FASES DE TRANSICIÓN Y FRENADO FUERON VALIDADAS EXITOSAMENTE (PASSED)")
    print("=" * 80 + "\n")

    executor.shutdown()
    controller.destroy_node()
    harness.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    run_transition_tests()
