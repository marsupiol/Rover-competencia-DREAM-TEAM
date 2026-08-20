#!/usr/bin/env python3
"""
Nodo ROS 2 de Planificación de Rutas BEV (GeNIE / SAM-TP) para Earth Rover.

Arquitectura:
- Se suscribe a la cámara frontal ('earth_rover/front/image_raw'), odometría/GPS
  ('gps/filtered'), heading ('earth_rover/heading') y meta ('earth_rover/target_waypoint').
- Corre la predicción de transitabilidad (SAM-TP) y la proyección a vista aérea (BEV)
  junto con el planificador de trayectorias polinomiales (GeNIE) en un hilo desacoplado
  en segundo plano para no bloquear el executor de ROS.
- Publica el camino planificado ('earth_rover/planned_path') en el marco estándar 'base_link'
  (REP-103: +X adelante, +Y izquierda), la visualización de depuración y el estado de validez.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Bool, Float32


def genie_xy_to_ros_base_link(x_right_m: float, y_forward_m: float) -> tuple[float, float]:
    """
    Convierte un punto del marco local de GeNIE al marco estándar ROS REP-103 (base_link).

    Convenciones de ejes:
    - GeNIE BEV: +X = Derecha (lateral right), +Y = Adelante (forward).
    - ROS (REP-103 base_link): +X = Adelante (forward), +Y = Izquierda (lateral left).

    Transformación matemática:
      x_ros (adelante)   = y_forward_m
      y_ros (izquierda)  = -x_right_m

    Ejemplo numérico de verificación:
      Punto GeNIE: (x_right=+2.0m, y_forward=+5.0m) [2m a la derecha, 5m adelante]
      -> x_ros = +5.0m (adelante)
      -> y_ros = -2.0m (derecha en ROS, ya que +Y es izquierda)

      Punto GeNIE: (x_right=-1.5m, y_forward=+3.0m) [1.5m a la izquierda, 3m adelante]
      -> x_ros = +3.0m (adelante)
      -> y_ros = +1.5m (izquierda en ROS)
    """
    x_ros = float(y_forward_m)
    y_ros = -float(x_right_m)
    return x_ros, y_ros


class BEVPlannerNode(Node):
    def __init__(self):
        super().__init__("bev_planner_node")

        # ----------------------------------------------------------------------
        # 1. Declaración y Extracción de Parámetros
        # ----------------------------------------------------------------------
        self.declare_parameter("image_topic", "earth_rover/front/image_raw")
        self.declare_parameter("gps_topic", "gps/filtered")
        self.declare_parameter("heading_topic", "earth_rover/heading")
        self.declare_parameter("target_topic", "earth_rover/target_waypoint")
        self.declare_parameter("planned_path_topic", "earth_rover/planned_path")
        self.declare_parameter("visualization_topic", "earth_rover/planner_visualization")
        self.declare_parameter("valid_topic", "earth_rover/planner_valid")
        self.declare_parameter("publish_visualization", True)

        self.declare_parameter("planning_min_period_s", 0.1)
        self.declare_parameter("checkpoint_path", "")
        self.declare_parameter("hf_repo", "")
        self.declare_parameter("device", "")
        self.declare_parameter("contrast_refine", True)

        # Parámetros de proyección BEV
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("resolution_m_per_px", 0.03)
        self.declare_parameter("forward_range_m", 4.0)
        self.declare_parameter("side_range_m", 2.0)
        self.declare_parameter("max_ray_distance_m", 6.0)
        self.declare_parameter("camera_k_path", "")
        self.declare_parameter("camera_t_base_camera_path", "")

        # Parámetros del planificador GeNIE
        self.declare_parameter("grid_size", 240)
        self.declare_parameter("unknown_cost", 0.2)
        self.declare_parameter("smooth_kernel", 3)
        self.declare_parameter("num_goals", 30)
        self.declare_parameter("num_mid_points_per_goal", 20)
        self.declare_parameter("path_num_samples", 100)
        self.declare_parameter("footprint_px", 10)  # TODO: confirmar dimensión real del Mini+
        self.declare_parameter("threshold_cost", 0.50)
        self.declare_parameter("threshold_points_ratio", 0.05)
        self.declare_parameter("number_of_points_to_filter", 60)
        self.declare_parameter("alpha", 1.0)
        self.declare_parameter("best_k", 12)
        self.declare_parameter("use_clustering", True)
        self.declare_parameter("max_clusters", 4)
        self.declare_parameter("cluster_angle_threshold_deg", 40.0)
        self.declare_parameter("random_seed", 42)
        self.declare_parameter("include_goal_in_path_bank", False)
        self.declare_parameter("include_random_goals", True)

        # Extracción
        image_topic = str(self.get_parameter("image_topic").value)
        gps_topic = str(self.get_parameter("gps_topic").value)
        heading_topic = str(self.get_parameter("heading_topic").value)
        target_topic = str(self.get_parameter("target_topic").value)
        planned_path_topic = str(self.get_parameter("planned_path_topic").value)
        visualization_topic = str(self.get_parameter("visualization_topic").value)
        valid_topic = str(self.get_parameter("valid_topic").value)
        self.publish_visualization = bool(self.get_parameter("publish_visualization").value)

        self.planning_min_period_s = float(self.get_parameter("planning_min_period_s").value)
        checkpoint_path = str(self.get_parameter("checkpoint_path").value) or None
        hf_repo = str(self.get_parameter("hf_repo").value) or None
        device = str(self.get_parameter("device").value) or None
        contrast_refine = bool(self.get_parameter("contrast_refine").value)

        self.ground_z = float(self.get_parameter("ground_z").value)
        self.bev_resolution = float(self.get_parameter("resolution_m_per_px").value)
        self.forward_range = float(self.get_parameter("forward_range_m").value)
        self.side_range = float(self.get_parameter("side_range_m").value)
        self.max_ray_distance = float(self.get_parameter("max_ray_distance_m").value)

        camera_k_path = str(self.get_parameter("camera_k_path").value)
        camera_t_path = str(self.get_parameter("camera_t_base_camera_path").value)

        # ----------------------------------------------------------------------
        # 2. Carga del Modelo de Percepción, Calibración y Módulos de Planificación
        # ----------------------------------------------------------------------
        try:
            from rover_traversability.calibration import load_camera_K, load_T_base_camera
            from rover_traversability.predictor import TraversabilityPredictor
            from rover_traversability.weights import SamNotInstalledError
            from genie_path_planner.planner import PlannerConfig, plan_on_bev
            from genie_path_planner.projection import project_score_to_bev
        except ImportError as exc:
            self.get_logger().error(
                "No se pudo importar rover_traversability / genie_path_planner. Instalá las "
                "dependencias en el entorno:\n"
                "    pip install torch torchvision\n"
                "    pip install --no-build-isolation -e ./genie\n"
                "    pip install -e './traversability[hf]'\n"
                f"Error original: {exc}"
            )
            raise

        self._plan_on_bev = plan_on_bev
        self._project_score_to_bev = project_score_to_bev

        # Cargar calibración de cámara una sola vez
        if camera_k_path:
            self._camera_k = np.load(camera_k_path).astype(np.float64)
        else:
            self._camera_k = load_camera_K().astype(np.float64)

        if camera_t_path:
            self._camera_t = np.load(camera_t_path).astype(np.float64)
        else:
            self._camera_t = load_T_base_camera().astype(np.float64)

        # Configuración del algoritmo de caminos
        seed_val = self.get_parameter("random_seed").value
        random_seed = int(seed_val) if seed_val is not None else None

        self._planner_cfg = PlannerConfig(
            grid_size=int(self.get_parameter("grid_size").value),
            unknown_cost=float(self.get_parameter("unknown_cost").value),
            smooth_kernel=int(self.get_parameter("smooth_kernel").value),
            num_goals=int(self.get_parameter("num_goals").value),
            num_mid_points_per_goal=int(self.get_parameter("num_mid_points_per_goal").value),
            path_num_samples=int(self.get_parameter("path_num_samples").value),
            footprint_px=int(self.get_parameter("footprint_px").value),
            threshold_cost=float(self.get_parameter("threshold_cost").value),
            threshold_points_ratio=float(self.get_parameter("threshold_points_ratio").value),
            number_of_points_to_filter=int(self.get_parameter("number_of_points_to_filter").value),
            alpha=float(self.get_parameter("alpha").value),
            best_k=int(self.get_parameter("best_k").value),
            use_clustering=bool(self.get_parameter("use_clustering").value),
            max_clusters=int(self.get_parameter("max_clusters").value),
            cluster_angle_threshold_deg=float(self.get_parameter("cluster_angle_threshold_deg").value),
            random_seed=random_seed,
            include_goal_in_path_bank=bool(self.get_parameter("include_goal_in_path_bank").value),
            include_random_goals=bool(self.get_parameter("include_random_goals").value),
        )

        try:
            self._predictor = TraversabilityPredictor(
                checkpoint=checkpoint_path,
                device=device,
                hf_repo=hf_repo,
                contrast_refine=contrast_refine,
            )
        except SamNotInstalledError as exc:
            self.get_logger().error(str(exc))
            raise

        self.get_logger().info(
            f"BEV Planner cargado con SAM-TP en device={self._predictor.device} | "
            f"BEV: {self.forward_range}m x {2*self.side_range}m @ {self.bev_resolution}m/px"
        )

        # ----------------------------------------------------------------------
        # 3. Perfiles QoS, Suscriptores y Publicadores
        # ----------------------------------------------------------------------
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

        self.bridge = CvBridge()
        self.create_subscription(Image, image_topic, self._on_image, sensor_qos)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, sensor_qos)
        self.create_subscription(Float32, heading_topic, self._on_heading, sensor_qos)
        self.create_subscription(NavSatFix, target_topic, self._on_target, reliable_qos)

        self.path_pub = self.create_publisher(Path, planned_path_topic, reliable_qos)
        self.valid_pub = self.create_publisher(Bool, valid_topic, sensor_qos)
        self.vis_pub = (
            self.create_publisher(Image, visualization_topic, sensor_qos)
            if self.publish_visualization
            else None
        )

        # ----------------------------------------------------------------------
        # 4. Estado de Navegación y Sincronización del Hilo de Planificación
        # ----------------------------------------------------------------------
        self._current_lat: float | None = None
        self._current_lon: float | None = None
        self._current_heading: float | None = None
        self._target_lat: float | None = None
        self._target_lon: float | None = None

        self._frame_lock = threading.Lock()
        self._latest_rgb: np.ndarray | None = None
        self._stop_event = threading.Event()
        self._infer_thread = threading.Thread(target=self._planning_loop, daemon=True)
        self._infer_thread.start()

        self.get_logger().info(
            f"BEV Planner Node inicializado | image={image_topic} | path={planned_path_topic}"
        )

    # --------------------------------------------------------------------------
    # Callbacks de Sensores
    # --------------------------------------------------------------------------
    def _on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"No se pudo convertir el frame de cámara: {exc}", throttle_duration_sec=5.0)
            return

        rgb = bgr[:, :, ::-1]
        with self._frame_lock:
            self._latest_rgb = rgb

    def _on_gps(self, msg: NavSatFix):
        self._current_lat = float(msg.latitude)
        self._current_lon = float(msg.longitude)

    def _on_heading(self, msg: Float32):
        self._current_heading = float(msg.data) % 360.0

    def _on_target(self, msg: NavSatFix):
        self._target_lat = float(msg.latitude)
        self._target_lon = float(msg.longitude)

    # --------------------------------------------------------------------------
    # Motor Matemático Geodésico (Idéntico a gps_waypoint_controller)
    # --------------------------------------------------------------------------
    @staticmethod
    def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
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
    def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        dlon = math.radians(lon2 - lon1)
        lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
        y = math.sin(dlon) * math.cos(lat2_r)
        x = (
            math.cos(lat1_r) * math.sin(lat2_r)
            - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        )
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def angle_error_deg(target_deg: float, current_deg: float) -> float:
        return (target_deg - current_deg + 540.0) % 360.0 - 180.0

    # --------------------------------------------------------------------------
    # Bucle Principal de Planificación (Hilo desacoplado)
    # --------------------------------------------------------------------------
    def _planning_loop(self):
        last_start = 0.0
        while not self._stop_event.is_set():
            with self._frame_lock:
                frame = self._latest_rgb
                self._latest_rgb = None  # Consumir frame para evitar re-procesamiento

            if frame is None:
                time.sleep(0.02)
                continue

            elapsed_since_last = time.monotonic() - last_start
            if elapsed_since_last < self.planning_min_period_s:
                time.sleep(self.planning_min_period_s - elapsed_since_last)

            last_start = time.monotonic()
            try:
                self._run_planning(frame)
            except Exception as exc:
                self.get_logger().error(f"Fallo en iteración de planificación BEV: {exc}", throttle_duration_sec=5.0)

    def _compute_relative_goal(self) -> tuple[float, float]:
        """
        Calcula la posición de la meta en coordenadas relativas de GeNIE (x_right, y_forward) en metros.

        Fórmula y Derivación:
        1. Distancia geodésica 'd' vía Haversine.
        2. Bearing absoluto hacia la meta (0=Norte, 90=Este, en sentido horario).
        3. Error angular: theta = bearing - current_heading (en [-180, 180] deg).
           - theta > 0: meta a la derecha del rover.
           - theta < 0: meta a la izquierda del rover.
        4. Descomposición polar en marco GeNIE (+X derecha, +Y adelante):
           - goal_x_m (derecha)   = d * sin(theta)
           - goal_y_m (adelante)  = d * cos(theta)
        """
        if (
            self._current_lat is None
            or self._current_lon is None
            or self._current_heading is None
            or self._target_lat is None
            or self._target_lon is None
        ):
            # Fallback seguro: si aún no hay GPS o meta, apuntar recto hacia adelante dentro del horizonte local
            return 0.0, float(self.forward_range)

        dist = self.haversine_distance(
            self._current_lat, self._current_lon, self._target_lat, self._target_lon
        )
        bearing = self.calculate_bearing(
            self._current_lat, self._current_lon, self._target_lat, self._target_lon
        )
        heading_error = self.angle_error_deg(bearing, self._current_heading)
        theta = math.radians(heading_error)

        goal_x_m = dist * math.sin(theta)
        goal_y_m = dist * math.cos(theta)
        return float(goal_x_m), float(goal_y_m)

    def _run_planning(self, rgb: np.ndarray):
        # 1. Inferencia neuronal de transitabilidad sobre la imagen frontal
        predict_res = self._predictor.predict(rgb)
        score_mask = predict_res.mask.astype(np.float32)

        # 2. Proyección de la máscara de transitabilidad a vista aérea (BEV)
        bev_flat, observed, _stats = self._project_score_to_bev(
            score_map=score_mask,
            camera_k=self._camera_k,
            camera_pose=self._camera_t,
            ground_z=self.ground_z,
            bev_resolution_m_per_px=self.bev_resolution,
            bev_forward_range_m=self.forward_range,
            bev_side_range_m=self.side_range,
            max_ray_distance_m=self.max_ray_distance,
        )

        # 3. Cálculo de la meta relativa (x_right, y_forward)
        goal_x_m, goal_y_m = self._compute_relative_goal()

        # 4. Planificación del camino sobre la grilla de costos BEV
        planned = self._plan_on_bev(
            bev_traversability=bev_flat,
            observed_mask=observed,
            goal_x_m=goal_x_m,
            goal_y_m=goal_y_m,
            bev_resolution_m=self.bev_resolution,
            config=self._planner_cfg,
        )

        now = self.get_clock().now()
        is_valid = bool(
            planned.final_path_xy_m is not None
            and planned.final_path_xy_m.shape[0] > 0
            and planned.metadata.get("status") == "ok"
        )

        # 5. Publicación del estado de validez
        valid_msg = Bool()
        valid_msg.data = is_valid
        self.valid_pub.publish(valid_msg)

        # 6. Publicación del camino planificado en nav_msgs/Path (marco 'base_link')
        path_msg = Path()
        path_msg.header.stamp = now.to_msg()
        path_msg.header.frame_id = "base_link"

        if is_valid:
            for pt in planned.final_path_xy_m:
                x_genie_right = float(pt[0])
                y_genie_forward = float(pt[1])
                x_ros, y_ros = genie_xy_to_ros_base_link(x_genie_right, y_genie_forward)

                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = x_ros
                pose_stamped.pose.position.y = y_ros
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = 1.0
                path_msg.poses.append(pose_stamped)

        self.path_pub.publish(path_msg)

        # 7. Publicación de imagen de depuración si está habilitada
        if self.vis_pub is not None and isinstance(planned.visualization, np.ndarray):
            vis_bgr = planned.visualization[:, :, ::-1]  # RGB a BGR para OpenCV / cv_bridge
            vis_msg = self.bridge.cv2_to_imgmsg(vis_bgr, encoding="bgr8")
            vis_msg.header.stamp = now.to_msg()
            vis_msg.header.frame_id = "base_link"
            self.vis_pub.publish(vis_msg)

        self.get_logger().debug(
            f"BEV Plan: valid={is_valid} points={len(path_msg.poses)} "
            f"goal_rel=({goal_x_m:+.2f}m, {goal_y_m:+.2f}m) infer={predict_res.inference_s*1000:.0f}ms"
        )

    def destroy_node(self):
        self._stop_event.set()
        self._infer_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BEVPlannerNode()
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
