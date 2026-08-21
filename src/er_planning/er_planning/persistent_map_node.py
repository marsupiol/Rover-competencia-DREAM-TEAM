#!/usr/bin/env python3
"""
Persistent Map Node for Earth Rover (IROS 2026 / FrodoBots).

Acumula las grillas locales BEV ('earth_rover/local_bev_grid') en un mapa global
persistente ('earth_rover/persistent_map') en el marco 'map' (REP-105) con
lógica de confianza Bayesiana acumulativa y decaimiento exponencial periódico.
"""

from __future__ import annotations

import math
import threading
from typing import Any

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros


class PersistentMapNode(Node):
    def __init__(self):
        super().__init__("persistent_map_node")

        # ----------------------------------------------------------------------
        # 1. Declaración y Extracción de Parámetros
        # ----------------------------------------------------------------------
        self.declare_parameter("local_grid_topic", "earth_rover/local_bev_grid")
        self.declare_parameter("map_topic", "earth_rover/persistent_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("map_width_m", 400.0)
        self.declare_parameter("map_height_m", 400.0)
        self.declare_parameter("map_resolution_m_per_px", 0.20)
        self.declare_parameter("map_origin_x_m", -200.0)
        self.declare_parameter("map_origin_y_m", -200.0)
        self.declare_parameter("hit_gain", 15.0)
        self.declare_parameter("miss_gain", 10.0)
        self.declare_parameter("confidence_max", 100.0)
        self.declare_parameter("occupied_threshold", 55.0)
        self.declare_parameter("free_threshold", -20.0)
        self.declare_parameter("occupied_cost_cutoff", 50.0)
        self.declare_parameter("decay_period_s", 1.0)
        self.declare_parameter("decay_factor", 0.95)
        self.declare_parameter("map_publish_period_s", 1.0)
        self.declare_parameter("tf_lookup_timeout_s", 0.2)

        local_grid_topic = str(self.get_parameter("local_grid_topic").value)
        map_topic = str(self.get_parameter("map_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)

        self.map_width_m = float(self.get_parameter("map_width_m").value)
        self.map_height_m = float(self.get_parameter("map_height_m").value)
        self.map_resolution = float(self.get_parameter("map_resolution_m_per_px").value)
        self.map_origin_x = float(self.get_parameter("map_origin_x_m").value)
        self.map_origin_y = float(self.get_parameter("map_origin_y_m").value)

        self.hit_gain = float(self.get_parameter("hit_gain").value)
        self.miss_gain = float(self.get_parameter("miss_gain").value)
        self.confidence_max = float(self.get_parameter("confidence_max").value)
        self.occupied_threshold = float(self.get_parameter("occupied_threshold").value)
        self.free_threshold = float(self.get_parameter("free_threshold").value)
        self.occupied_cost_cutoff = float(self.get_parameter("occupied_cost_cutoff").value)

        self.decay_period_s = float(self.get_parameter("decay_period_s").value)
        self.decay_factor = float(self.get_parameter("decay_factor").value)
        self.map_publish_period_s = float(self.get_parameter("map_publish_period_s").value)
        self.tf_lookup_timeout_s = float(self.get_parameter("tf_lookup_timeout_s").value)

        # ----------------------------------------------------------------------
        # 2. Inicialización de la Grilla de Confianza Persistente
        # ----------------------------------------------------------------------
        self.grid_w = max(1, int(round(self.map_width_m / self.map_resolution)))
        self.grid_h = max(1, int(round(self.map_height_m / self.map_resolution)))

        # Grilla float32: >0 = evidencia ocupada, <0 = evidencia libre, 0 = desconocido
        self._lock = threading.Lock()
        self._confidence = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

        # ----------------------------------------------------------------------
        # 3. Transform Buffer y QoS
        # ----------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.create_subscription(OccupancyGrid, local_grid_topic, self._on_local_grid, sensor_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, map_qos)

        # ----------------------------------------------------------------------
        # 4. Temporizadores de Decaimiento y Publicación
        # ----------------------------------------------------------------------
        self.decay_timer = self.create_timer(self.decay_period_s, self._decay_timer_cb)
        self.publish_timer = self.create_timer(self.map_publish_period_s, self._publish_timer_cb)

        self.get_logger().info(
            f"PersistentMapNode inicializado | Mapa: {self.map_width_m}x{self.map_height_m}m "
            f"({self.grid_w}x{self.grid_h} px @ {self.map_resolution}m/px) | Frame: {self.map_frame}"
        )

    # --------------------------------------------------------------------------
    # Callback de Grilla BEV Local
    # --------------------------------------------------------------------------
    def _on_local_grid(self, msg: OccupancyGrid):
        """
        Integra la grilla BEV local recibida en el mapa global usando TF map -> base_link.

        Transformación de Coordenadas y Convención de Signos:
          Sea p_local = [x_local, y_local] la posición métrica de una celda en base_link.
          La pose del robot en el frame 'map' obtenida vía TF es (tx, ty) con rotación yaw theta:
            x_map = tx + x_local * cos(theta) - y_local * sin(theta)
            y_map = ty + x_local * sin(theta) + y_local * cos(theta)

          Ejemplo numérico de verificación:
            Robot en (tx=10.0m, ty=20.0m), orientado al Norte (theta = +90 deg, cos=0, sin=1):
            - Celda 2m adelante (x_local=+2.0, y_local=0.0):
                x_map = 10.0 + 2.0*0 - 0*1 = 10.0m
                y_map = 20.0 + 2.0*1 + 0*0 = 22.0m (2m al Norte en 'map')
            - Celda 2m a la izquierda (x_local=0.0, y_local=+2.0):
                x_map = 10.0 + 0*0 - 2.0*1 = 8.0m (2m al Oeste en 'map')
                y_map = 20.0 + 0*1 + 2.0*0 = 20.0m

        Lógica Bayesiana de Confianza y Ejemplos Numéricos:
          hit_gain = 15.0, miss_gain = 10.0, occupied_threshold = 55.0, free_threshold = -20.0
          - Acumulación de Obstáculo:
              Celda arranca en confidence = 0.0 (desconocido).
              Frame 1: +15 -> 15.0 (desconocido)
              Frame 2: +15 -> 30.0 (desconocido)
              Frame 3: +15 -> 45.0 (desconocido)
              Frame 4: +15 -> 60.0 >= 55.0 -> Supera occupied_threshold -> Pasa a OCUPADA (100).
              -> Requiere ceil(55/15) = 4 observaciones consecutivas para marcar obstáculo.
          - Acumulación de Terreno Libre:
              Celda arranca en confidence = 0.0 (desconocido).
              Frame 1: -10 -> -10.0 (desconocido)
              Frame 2: -10 -> -20.0 <= -20.0 -> Pasa a LIBRE (0).
              -> Requiere ceil(20/10) = 2 observaciones consecutivas para confirmar libre.
        """
        # 1. Lookup de la transformación map -> base_link
        target_frame = self.map_frame
        source_frame = msg.header.frame_id if msg.header.frame_id else "base_link"

        try:
            lookup_stamp = Time.from_msg(msg.header.stamp)
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                lookup_stamp,
                timeout=Duration(seconds=self.tf_lookup_timeout_s),
            )
        except Exception as exc:
            # Fallback seguro con tiempo cero si la interpolación por timestamp exacto falla
            try:
                tf_msg = self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_s),
                )
            except Exception as exc2:
                self.get_logger().warn(
                    f"No se pudo obtener TF {target_frame} -> {source_frame}: {exc2}",
                    throttle_duration_sec=3.0,
                )
                return

        # 2. Extracción de traslación y ángulo de rumbo (yaw) desde el cuaternión TF
        tx = float(tf_msg.transform.translation.x)
        ty = float(tf_msg.transform.translation.y)
        qx = float(tf_msg.transform.rotation.x)
        qy = float(tf_msg.transform.rotation.y)
        qz = float(tf_msg.transform.rotation.z)
        qw = float(tf_msg.transform.rotation.w)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # 3. Vectorización de celdas observadas
        local_w = int(msg.info.width)
        local_h = int(msg.info.height)
        local_res = float(msg.info.resolution)
        orig_x = float(msg.info.origin.position.x)
        orig_y = float(msg.info.origin.position.y)

        raw_data = np.asarray(msg.data, dtype=np.int16).reshape((local_h, local_w))
        obs_rows, obs_cols = np.where(raw_data != -1)
        if obs_rows.size == 0:
            return

        # Coordenadas locales en metros (centro del píxel local)
        x_local = orig_x + (obs_cols.astype(np.float64) + 0.5) * local_res
        y_local = orig_y + (obs_rows.astype(np.float64) + 0.5) * local_res

        # 4. Proyección rígida 2D al marco global 'map'
        x_map = tx + x_local * cos_yaw - y_local * sin_yaw
        y_map = ty + x_local * sin_yaw + y_local * cos_yaw

        # 5. Mapeo a índices discretos de la grilla persistente
        map_cols = np.floor((x_map - self.map_origin_x) / self.map_resolution).astype(np.int32)
        map_rows = np.floor((y_map - self.map_origin_y) / self.map_resolution).astype(np.int32)

        # 6. Filtrado de límites del mapa
        in_bounds = (
            (map_cols >= 0)
            & (map_cols < self.grid_w)
            & (map_rows >= 0)
            & (map_rows < self.grid_h)
        )
        dropped_cells = int(np.count_nonzero(~in_bounds))
        if dropped_cells > 0:
            self.get_logger().warn(
                f"{dropped_cells}/{obs_rows.size} celdas BEV cayeron fuera de los límites del mapa persistente.",
                throttle_duration_sec=5.0,
            )

        valid_cols = map_cols[in_bounds]
        valid_rows = map_rows[in_bounds]
        valid_costs = raw_data[obs_rows[in_bounds], obs_cols[in_bounds]]

        # 7. Actualización de confianza (Hit vs Miss)
        is_hit = valid_costs >= self.occupied_cost_cutoff
        deltas = np.where(is_hit, self.hit_gain, -self.miss_gain).astype(np.float32)

        with self._lock:
            # np.add.at acumula de forma segura cuando múltiples puntos locales mapean al mismo píxel global
            np.add.at(self._confidence, (valid_rows, valid_cols), deltas)
            np.clip(self._confidence, -self.confidence_max, self.confidence_max, out=self._confidence)

    # --------------------------------------------------------------------------
    # Temporizador de Decaimiento Exponencial
    # --------------------------------------------------------------------------
    def _decay_timer_cb(self):
        """
        Aplica decaimiento exponencial periódico a toda la grilla de confianza.

        Fórmula Matemática:
          confidence[t] = confidence[t - 1] * decay_factor

        Ejemplo Numérico y Cálculo de Tiempo de Olvido:
          Con decay_factor = 0.95 y decay_period_s = 1.0s:
          - Caso 1: Celda recién clasificada como obstáculo en el umbral (confidence = 55.0).
              t=1s: 55.0 * 0.95 = 52.25 < 55.0 -> Pasa a DESCONOCIDA tras solo 1 segundo.
          - Caso 2: Obstáculo persistente saturado al máximo (confidence = 100.0).
              Buscamos k períodos para que 100.0 * (0.95)^k < 55.0:
                (0.95)^k < 0.55
                k * ln(0.95) < ln(0.55)
                k > ln(0.55) / ln(0.95) = (-0.5978) / (-0.05129) = 11.65 períodos.
              -> Tras ~12 segundos sin re-observación, un obstáculo totalmente saturado (ej. auto parado)
                 decae por debajo del umbral ocupado y vuelve a desconocido, permitiendo el paso del rover.
        """
        with self._lock:
            self._confidence *= np.float32(self.decay_factor)

    # --------------------------------------------------------------------------
    # Temporizador de Publicación del Mapa Persistente
    # --------------------------------------------------------------------------
    def _publish_timer_cb(self):
        """
        Publica la grilla persistente como nav_msgs/OccupancyGrid en el marco 'map'.

        Esquema de estados en el mensaje publicado:
          - -1 (Desconocido): free_threshold (-20.0) < confidence < occupied_threshold (55.0)
          -  0 (Confirmado Libre): confidence <= free_threshold (-20.0)
          - 100 (Confirmado Ocupado): confidence >= occupied_threshold (55.0)
        """
        with self._lock:
            grid_copy = self._confidence.copy()

        # Vectorización rápida de los tres estados
        occ_grid = np.full(grid_copy.shape, -1, dtype=np.int8)
        occ_grid[grid_copy <= self.free_threshold] = 0
        occ_grid[grid_copy >= self.occupied_threshold] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = float(self.map_resolution)
        msg.info.width = int(self.grid_w)
        msg.info.height = int(self.grid_h)

        msg.info.origin.position.x = float(self.map_origin_x)
        msg.info.origin.position.y = float(self.map_origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = occ_grid.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersistentMapNode()
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
