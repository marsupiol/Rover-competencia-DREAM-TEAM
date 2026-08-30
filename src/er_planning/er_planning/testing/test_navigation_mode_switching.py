#!/usr/bin/env python3
"""
test_navigation_mode_switching.py — Test offline de verificación de navegación multimodo (Brief 2).

Prueba la máquina de selección de meta de 3 vías de bev_planner_node junto al stub image_goal_stub_node:
  1. Modo 'outdoor':
     - gps_reliable=True / image_goal_ready=False -> Fuente: GPS/Global -> goal_source_ready=True, valid=True
     - gps_reliable=True / image_goal_ready=True  -> Fuente: GPS/Global (ignora image-goal) -> ready=True, valid=True
  2. Modo 'indoor':
     - image_goal_ready=False -> Sin fallback geodésico -> goal_source_ready=False, valid=False
     - image_goal_ready=True  -> Fuente: image_goal -> goal_source_ready=True, valid=True
  3. Modo 'auto':
     - gps_reliable=True / image_goal_ready=False  -> Fuente: GPS/Global -> ready=True, valid=True
     - gps_reliable=False / image_goal_ready=False -> Fuente: Ninguna -> ready=False, valid=False
     - gps_reliable=False / image_goal_ready=True  -> Fuente: image_goal -> ready=True, valid=True
"""

import math
import os
import sys
import time
import numpy as np
import cv2

# Agregar directorio del paquete al path
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_THIS_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float32


class TestNavModeHarness(Node):
    def __init__(self):
        super().__init__("test_nav_mode_harness")
        self.bridge = CvBridge()

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

        # Publicadores de estímulo
        self.image_pub = self.create_publisher(Image, "earth_rover/front/image_raw", sensor_qos)
        self.gps_pub = self.create_publisher(NavSatFix, "gps/filtered", sensor_qos)
        self.heading_pub = self.create_publisher(Float32, "earth_rover/heading", sensor_qos)
        self.target_pub = self.create_publisher(NavSatFix, "earth_rover/target_waypoint", reliable_qos)
        self.gps_reliable_pub = self.create_publisher(Bool, "earth_rover/gps_reliable", reliable_qos)
        self.img_goal_pub = self.create_publisher(Vector3, "earth_rover/image_goal_relative", sensor_qos)
        self.img_ready_pub = self.create_publisher(Bool, "earth_rover/image_goal_ready", sensor_qos)

        # Suscriptores a salidas del planner
        self.path_sub = self.create_subscription(Path, "earth_rover/planned_path", self._on_path, reliable_qos)
        self.valid_sub = self.create_subscription(Bool, "earth_rover/planner_valid", self._on_valid, sensor_qos)
        self.ready_sub = self.create_subscription(Bool, "earth_rover/goal_source_ready", self._on_ready, sensor_qos)

        self.last_path_count = 0
        self.last_valid: bool | None = None
        self.last_source_ready: bool | None = None

        # Imagen sintética transitable (terreno uniforme)
        self.test_img = np.ones((480, 640, 3), dtype=np.uint8) * 128

    def _on_path(self, msg: Path):
        self.last_path_count = len(msg.poses)

    def _on_valid(self, msg: Bool):
        self.last_valid = bool(msg.data)

    def _on_ready(self, msg: Bool):
        self.last_source_ready = bool(msg.data)

    def publish_stimulus(
        self,
        gps_reliable: bool,
        image_goal_ready: bool,
        image_goal_xy: tuple[float, float] = (0.0, 3.0),
    ):
        now = self.get_clock().now().to_msg()

        # 1. Imagen frontal
        img_msg = self.bridge.cv2_to_imgmsg(self.test_img, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "earth_rover_front_camera"
        self.image_pub.publish(img_msg)

        # 2. GPS crudo / filtrado
        gps = NavSatFix()
        gps.header.stamp = now
        gps.latitude = -34.6037
        gps.longitude = -58.3816
        self.gps_pub.publish(gps)

        # 3. Target GPS (20m adelante)
        target = NavSatFix()
        target.header.stamp = now
        target.latitude = -34.6035
        target.longitude = -58.3816
        self.target_pub.publish(target)

        # 4. Heading
        h = Float32()
        h.data = 0.0
        self.heading_pub.publish(h)

        # 5. Señal de Confiabilidad GPS (Brief 1)
        rel_msg = Bool()
        rel_msg.data = bool(gps_reliable)
        self.gps_reliable_pub.publish(rel_msg)

        # 6. Image Goal Stub
        goal_msg = Vector3()
        goal_msg.x = float(image_goal_xy[0])
        goal_msg.y = float(image_goal_xy[1])
        goal_msg.z = 0.0
        self.img_goal_pub.publish(goal_msg)

        ready_msg = Bool()
        ready_msg.data = bool(image_goal_ready)
        self.img_ready_pub.publish(ready_msg)


from rclpy.parameter import Parameter


def set_planner_navigation_mode(planner_node, mode: str):
    planner_node.navigation_mode = mode.lower()
    param = Parameter(
        "navigation_mode",
        Parameter.Type.STRING,
        mode,
    )
    planner_node.set_parameters([param])


def run_tests():
    sys.stdout.reconfigure(line_buffering=True)
    rclpy.init()
    harness = TestNavModeHarness()

    from er_planning.bev_planner_node import BEVPlannerNode
    planner = BEVPlannerNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(harness)
    executor.add_node(planner)

    print("\n" + "=" * 80)
    print(" INICIANDO TEST OFFLINE: CONMUTACIÓN DE NAVEGACIÓN (OUTDOOR / INDOOR / AUTO)")
    print("=" * 80)

    # Warmup inicial de inferencia GPU
    print("[WARMUP] Calentando pipeline SAM-TP y GeNIE (2s)...")
    end_warmup = time.time() + 2.0
    while time.time() < end_warmup:
        harness.publish_stimulus(gps_reliable=True, image_goal_ready=False)
        executor.spin_once(timeout_sec=0.05)
        time.sleep(0.05)

    def step(duration_s: float, gps_rel: bool, img_ready: bool, img_xy=(0.0, 3.0)):
        # Dar un momento para procesar cualquier frame previo
        time.sleep(0.1)
        harness.last_source_ready = None
        harness.last_valid = None
        harness.last_path_count = 0

        end_t = time.time() + duration_s
        while time.time() < end_t:
            harness.publish_stimulus(gps_reliable=gps_rel, image_goal_ready=img_ready, image_goal_xy=img_xy)
            executor.spin_once(timeout_sec=0.02)
            time.sleep(0.05)
        # Drenar mensajes pendientes del hilo de inferencia
        for _ in range(15):
            executor.spin_once(timeout_sec=0.05)
            time.sleep(0.02)

    # --------------------------------------------------------------------------
    # ESCENARIO 1: MODO OUTDOOR
    # --------------------------------------------------------------------------
    print("\n--- ESCENARIO 1: navigation_mode = 'outdoor' ---")
    set_planner_navigation_mode(planner, "outdoor")

    # 1a. Outdoor con GPS Confiable
    print("[1a] outdoor | gps_reliable=True, image_goal_ready=False")
    step(1.5, gps_rel=True, img_ready=False)
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 1a: goal_source_ready debe ser True en outdoor con GPS"
    assert harness.last_valid is True, "Fallo 1a: planner_valid debe ser True"

    # 1b. Outdoor con Image Goal presente (debe ser ignorado y seguir en GPS)
    print("[1b] outdoor | gps_reliable=True, image_goal_ready=True (ignora image-goal)")
    step(1.0, gps_rel=True, img_ready=True, img_xy=(1.5, 3.0))
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 1b: goal_source_ready debe seguir siendo True"
    assert harness.last_valid is True, "Fallo 1b: planner_valid debe ser True"

    # --------------------------------------------------------------------------
    # ESCENARIO 2: MODO INDOOR
    # --------------------------------------------------------------------------
    print("\n--- ESCENARIO 2: navigation_mode = 'indoor' ---")
    set_planner_navigation_mode(planner, "indoor")

    # 2a. Indoor sin Image-Goal listo (NO debe inventar fallback geodésico)
    print("[2a] indoor | image_goal_ready=False (sin meta válida explícita)")
    step(1.5, gps_rel=True, img_ready=False)  # Aunque haya GPS, en indoor se ignora
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is False, "Fallo 2a: goal_source_ready debe ser False en indoor sin image_goal"
    assert harness.last_valid is False, "Fallo 2a: planner_valid debe ser False (sin meta inventada)"
    assert harness.last_path_count == 0, "Fallo 2a: el path debe estar vacío sin meta"

    # 2b. Indoor con Image-Goal listo
    print("[2b] indoor | image_goal_ready=True, goal_xy=(0.0, 3.5)")
    step(1.5, gps_rel=False, img_ready=True, img_xy=(0.0, 3.5))
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 2b: goal_source_ready debe ser True con image_goal listo"
    assert harness.last_valid is True, "Fallo 2b: planner_valid debe ser True"
    assert harness.last_path_count > 0, "Fallo 2b: debe planificarse un camino válido hacia el image-goal"

    # --------------------------------------------------------------------------
    # ESCENARIO 3: MODO AUTO
    # --------------------------------------------------------------------------
    print("\n--- ESCENARIO 3: navigation_mode = 'auto' ---")
    set_planner_navigation_mode(planner, "auto")

    # 3a. Auto con GPS Confiable -> Comportamiento Outdoor
    print("[3a] auto | gps_reliable=True, image_goal_ready=False (comporta como outdoor)")
    step(1.5, gps_rel=True, img_ready=False)
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 3a: auto con GPS confiable debe tener goal_source_ready=True"
    assert harness.last_valid is True, "Fallo 3a: planner_valid debe ser True"

    # 3b. Auto con GPS Degradado e Image-Goal NO listo -> Comporta como indoor unready
    print("[3b] auto | gps_reliable=False, image_goal_ready=False (GPS degradado, sin image-goal)")
    step(1.5, gps_rel=False, img_ready=False)
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is False, "Fallo 3b: auto con GPS degradado y sin image-goal debe ser False"
    assert harness.last_valid is False, "Fallo 3b: planner_valid debe ser False"

    # 3c. Auto con GPS Degradado e Image-Goal LISTO -> Conmuta a Image-Goal
    print("[3c] auto | gps_reliable=False, image_goal_ready=True (GPS degradado -> conmuta a image-goal)")
    step(1.5, gps_rel=False, img_ready=True, img_xy=(-0.8, 3.0))
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 3c: auto degradado con image-goal listo debe ser True"
    assert harness.last_valid is True, "Fallo 3c: planner_valid debe ser True"
    assert harness.last_path_count > 0, "Fallo 3c: debe generar trayectoria válida"

    # 3d. Auto: Recuperación de GPS -> Vuelve automáticamente a Outdoor
    print("[3d] auto | gps_reliable=True (GPS recuperado -> vuelve a modo outdoor)")
    step(1.5, gps_rel=True, img_ready=True)
    print(f"     Resultado -> goal_source_ready={harness.last_source_ready}, valid={harness.last_valid}, path_points={harness.last_path_count}")
    assert harness.last_source_ready is True, "Fallo 3d: retorno a outdoor debe tener ready=True"
    assert harness.last_valid is True, "Fallo 3d: planner_valid debe ser True"

    print("\n" + "=" * 80)
    print(" TODOS LOS ESCENARIOS DE CONMUTACIÓN DE MODO FUERON VALIDADOS EXITOSAMENTE (PASSED)")
    print("=" * 80 + "\n")

    executor.shutdown()
    planner.destroy_node()
    harness.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    run_tests()
