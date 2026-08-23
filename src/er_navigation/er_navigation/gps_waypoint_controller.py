#!/usr/bin/env python3
"""
GPS Waypoint Navigation Controller for Earth Rover (IROS 2026).
Arquitectura Híbrida: Máquina de estados reactiva con mitigación de latencia de red (Burst & Wait)
y filtrado pasa-bajos para brújula ruidosa.
"""

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool

class GPSWaypointController(Node):
    def __init__(self):
        super().__init__("gps_waypoint_controller")

        # 1. DECLARACIÓN ESTRICTA DE PARÁMETROS (Tipado Fuerte)
        self.declare_parameter("goal_tolerance_m", 14.0) # Se frena 1 metro adentro del perímetro
        self.declare_parameter("goal_dwell_s", 1.5)
        self.declare_parameter("align_threshold_deg", 15.0)
        self.declare_parameter("coarse_align_threshold_deg", 45.0)
        self.declare_parameter("approach_align_distance_m", 8.0)
        self.declare_parameter("forward_speed", 0.35)
        self.declare_parameter("turn_speed", 0.25)
        self.declare_parameter("angular_speed", 0.80)
        self.declare_parameter("drive_correction_gain", 0.002)
        self.declare_parameter("max_drive_angular", 0.10)
        self.declare_parameter("invert_angular", True)
        self.declare_parameter("control_loop_hz", 5.0)
        self.declare_parameter("turn_burst_s", 0.35)
        self.declare_parameter("pause_after_turn_s", 2.2)
        self.declare_parameter("max_heading_jump_deg", 150.0)
        self.declare_parameter("heading_filter_alpha", 0.35)
        self.declare_parameter("reached_publish_period_s", 1.0)
        self.declare_parameter("gps_max_stale_s", 2.0)
        self.declare_parameter("path_topic", "earth_rover/planned_path")
        self.declare_parameter("path_valid_topic", "earth_rover/planner_valid")
        self.declare_parameter("path_max_stale_s", 1.0)
        self.declare_parameter("lookahead_distance_m", 1.0)
        self.declare_parameter("path_following_enabled", True)
        self.declare_parameter("recovery_turn_speed", 0.3)
        self.declare_parameter("max_total_drive_angular", 0.8)
        self.declare_parameter("publish_control_debug", True)

        # 2. EXTRACCIÓN DIRECTA DE PARÁMETROS (sin clamps, valores tal cual el yaml)

        # --- Tolerancias y Distancias ---
        self.goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.base_goal_tolerance = self.goal_tolerance  # Respaldo de la tolerancia original
        self.goal_dwell_s = float(self.get_parameter("goal_dwell_s").value)
        self.approach_align_distance = float(self.get_parameter("approach_align_distance_m").value)

        # --- Umbrales de Alineación ---
        self.align_threshold = float(self.get_parameter("align_threshold_deg").value)
        self.coarse_align_threshold = float(self.get_parameter("coarse_align_threshold_deg").value)

        # --- Dinámica de Conducción y Giro ---
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)

        # --- Dinámica de Conducción en Curva ---
        self.drive_correction_gain = float(self.get_parameter("drive_correction_gain").value)
        self.max_drive_angular = float(self.get_parameter("max_drive_angular").value)
        self.invert_angular = bool(self.get_parameter("invert_angular").value)
        self.max_total_drive_angular = float(self.get_parameter("max_total_drive_angular").value)

        # --- Guard de GPS y Seguimiento de Trayectorias BEV ---
        self.gps_max_stale_s = float(self.get_parameter("gps_max_stale_s").value)
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.path_valid_topic = str(self.get_parameter("path_valid_topic").value)
        self.path_max_stale_s = float(self.get_parameter("path_max_stale_s").value)
        self.lookahead_distance_m = float(self.get_parameter("lookahead_distance_m").value)
        self.path_following_enabled = bool(self.get_parameter("path_following_enabled").value)
        self.recovery_turn_speed = float(self.get_parameter("recovery_turn_speed").value)

        # --- Tiempos de Ráfaga y Filtros ---
        self.turn_burst_s = float(self.get_parameter("turn_burst_s").value)
        self.pause_after_turn_s = float(self.get_parameter("pause_after_turn_s").value)
        self.max_heading_jump = float(self.get_parameter("max_heading_jump_deg").value)

        self.heading_filter_alpha = float(self.get_parameter("heading_filter_alpha").value)
        self.reached_publish_period_s = float(self.get_parameter("reached_publish_period_s").value)
        self.loop_hz = float(self.get_parameter("control_loop_hz").value)
        self.publish_control_debug = bool(self.get_parameter("publish_control_debug").value)

        # 3. Perfiles QoS Diferenciados (Crítico para Jazzy)
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

        # 4. Suscriptores y Publicadores
        self.create_subscription(
            NavSatFix,
            "gps/filtered",          # <--- Salida directa de navsat_transform (EKF)
            self._on_gps,
            sensor_qos               # BEST_EFFORT
        )
        self.create_subscription(
            Float32,
            "earth_rover/heading",   # <--- Salida directa de ekf_heading_bridge
            self._on_heading,
            sensor_qos               # BEST_EFFORT
        )
        self.create_subscription(
            Path,
            self.path_topic,
            self._on_planned_path,
            sensor_qos,              # BEST_EFFORT
        )
        self.create_subscription(
            Bool,
            self.path_valid_topic,
            self._on_path_valid,
            sensor_qos,              # BEST_EFFORT
        )
        self.create_subscription(NavSatFix, "earth_rover/target_waypoint", self._on_target, reliable_qos)
        self.create_subscription(Bool, "earth_rover/navigation_pause", self._on_navigation_pause, reliable_qos)
        self.create_subscription(String, "earth_rover/waypoint_status", self._on_mission_status, reliable_qos)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", reliable_qos)
        self.status_pub = self.create_publisher(String, "earth_rover/waypoint_status", reliable_qos)
        self.control_debug_pub = self.create_publisher(String, "earth_rover/control_debug", sensor_qos)

        # 5. Inicialización de Vectores de Estado
        self.current_lat = None
        self.current_lon = None
        self._gps_last_update = None
        self.current_heading = None
        self._raw_heading = None
        self._heading_last_rx = None
        self.target_lat = None
        self.target_lon = None

        # Seguimiento de Trayectorias Planificadas (BEV)
        self._path_poses: list[tuple[float, float]] = []
        self._path_valid: bool = False
        self._path_last_update = None
        
        # Flags de Máquina de Estados
        self.active_goal = False
        self._reached_since = None
        self._awaiting_next_target = False
        self._last_reached_publish_at = None
        self._align_phase = "PAUSE"
        self._align_phase_started_at = None
        self._burst_turn_sign = 0
        self._navigation_paused = False

        # 6. Bucle de Control Principal
        self.timer = self.create_timer(1.0 / self.loop_hz, self._control_loop)
        self.get_logger().info(
            f"Controlador Híbrido Iniciado | Loop: {self.loop_hz} Hz | Burst: {self.turn_burst_s}s | Filter Alpha: {self.heading_filter_alpha}"
        )

    # --- CALLBACKS DE SENSORES Y PERCEPCIÓN ---
    def _on_gps(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self._gps_last_update = self.get_clock().now()

    def _on_heading(self, msg: Float32):
        self._heading_last_rx = self.get_clock().now()
        raw = float(msg.data) % 360.0
        self._raw_heading = raw

        if self.current_heading is None:
            self.current_heading = raw
            return

        jump = abs(self.angle_error_deg(raw, self.current_heading))
        if jump > self.max_heading_jump:
            # AHORA SABREMOS SI LA BRÚJULA SE ESTÁ RECHAZANDO
            self.get_logger().warn(f"Salto magnético gigante rechazado: {jump:.1f}°")
            return

        delta = self.angle_error_deg(raw, self.current_heading)
        self.current_heading = (self.current_heading + self.heading_filter_alpha * delta) % 360.0

    def _on_planned_path(self, msg: Path):
        poses = []
        for pose_stamped in msg.poses:
            poses.append((float(pose_stamped.pose.position.x), float(pose_stamped.pose.position.y)))
        self._path_poses = poses
        self._path_last_update = self.get_clock().now()

    def _on_path_valid(self, msg: Bool):
        self._path_valid = bool(msg.data)
        self._path_last_update = self.get_clock().now()

    # --- CALLBACKS DE MÁQUINA DE ESTADOS ---
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
        
        # CRÍTICO: Restaurar tolerancia original al cambiar a una nueva meta
        self.goal_tolerance = self.base_goal_tolerance
        
        self._stop_robot()
        self.get_logger().info(f"Target Fijado: ({self.target_lat:.6f}, {self.target_lon:.6f}) | Tolerancia: {self.goal_tolerance}m")

    def _on_navigation_pause(self, msg: Bool):
        was_paused = self._navigation_paused
        self._navigation_paused = bool(msg.data)
        
        # --- DETECTOR DE RECHAZO DEL SDK ---
        # Si estábamos pausados, nos despausan, y seguimos esperando una meta nueva...
        if was_paused and not self._navigation_paused and self._awaiting_next_target:
            # ¡Significa que el SDK dijo que NO! Estrangulamos la tolerancia a la mitad.
            self.goal_tolerance = max(0.5, self.goal_tolerance * 0.5)
            self.get_logger().warn(
                f"¡Rechazo del SDK detectado! Estrangulando tolerancia geodésica a {self.goal_tolerance:.1f}m"
            )
            self._awaiting_next_target = False
            self._reached_since = None

        if self._navigation_paused:
            self._stop_robot()
            self.get_logger().info("Pausa comandada por mission_manager.")

    def _on_mission_status(self, msg: String):
        if msg.data == "MISSION_FINISHED":
            self._awaiting_next_target = False
            self._navigation_paused = True
            self.active_goal = False
            self._stop_robot()
            self.get_logger().info("Misión finalizada. Controlador inactivo.")

    # --- MOTOR MATEMÁTICO GEODÉSICO ---
    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        r = 6371000.0
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2.0) ** 2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2.0) ** 2)
        return r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        dlon = math.radians(lon2 - lon1)
        lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
        y = math.sin(dlon) * math.cos(lat2_r)
        x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
        return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    @staticmethod
    def angle_error_deg(target_deg, current_deg):
        return (target_deg - current_deg + 540.0) % 360.0 - 180.0

    def _apply_angular_sign(self, angular):
        return -angular if self.invert_angular else angular

    # --- HELPERS DE TIEMPO, FRESCURA Y PATH FOLLOWING ---
    def _phase_elapsed(self, now):
        if self._align_phase_started_at is None:
            return 0.0
        return (now - self._align_phase_started_at).nanoseconds / 1e9

    def _begin_align_phase(self, phase, now):
        self._align_phase = phase
        self._align_phase_started_at = now

    def _publish_reached(self, now):
        self._stop_robot()
        msg = String()
        msg.data = "REACHED"
        self.status_pub.publish(msg)
        self._last_reached_publish_at = now
        self.get_logger().info("Señal REACHED publicada -> Esperando al manager.")

    def _stop_robot(self):
        self._align_phase = "PAUSE"
        self._align_phase_started_at = None
        self._burst_turn_sign = 0
        self.cmd_pub.publish(Twist())

    def _gps_is_fresh(self) -> bool:
        if self._gps_last_update is None:
            return False
        age_s = (self.get_clock().now() - self._gps_last_update).nanoseconds / 1e9
        return age_s <= self.gps_max_stale_s

    def _path_is_fresh(self) -> bool:
        if not self.path_following_enabled or self._path_last_update is None:
            return False
        age_s = (self.get_clock().now() - self._path_last_update).nanoseconds / 1e9
        return age_s <= self.path_max_stale_s

    def _compute_path_heading_error_deg(self) -> float | None:
        """
        Recorre self._path_poses (metros, base_link: x=adelante, y=izquierda)
        acumulando distancia hasta encontrar el primer punto a >= 
        lookahead_distance_m del origen (0,0, la posición actual del robot).
        Si el path es más corto que el lookahead, usa el último punto.
        Devuelve el heading_error en grados, MISMA convención que
        angle_error_deg ya usada en el resto del archivo (positivo = el
        objetivo está a la derecha, coherente con cómo se usa heading_error
        en el resto de _control_loop).

        Convención de signos y derivación:
        - Marco ROS REP-103 (base_link): +X = Adelante, +Y = Izquierda, -Y = Derecha.
        - math.atan2(y, x): da ángulo positivo hacia la izquierda (+Y) y negativo hacia la derecha (-Y).
        - Convención de heading_error en gps_waypoint_controller:
            heading_error = angle_error_deg(target_bearing, current_heading)
            -> Si la meta está a la derecha del rumbo actual: heading_error > 0 (positivo).
            -> Si la meta está a la izquierda del rumbo actual: heading_error < 0 (negativo).
        - Por lo tanto, para convertir el ángulo de base_link a heading_error:
            heading_error = -math.degrees(math.atan2(y, x))

        Ejemplos numéricos de verificación:
          1. Punto lookahead a la derecha: (x=+2.0m adelante, y=-1.0m derecha)
             -> atan2(-1.0, 2.0) = -26.57° (ángulo base_link)
             -> heading_error = -(-26.57°) = +26.57° (positivo -> el controlador gira a la DERECHA)
          2. Punto lookahead a la izquierda: (x=+2.0m adelante, y=+1.0m izquierda)
             -> atan2(+1.0, 2.0) = +26.57° (ángulo base_link)
             -> heading_error = -(+26.57°) = -26.57° (negativo -> el controlador gira a la IZQUIERDA)
          3. Punto lookahead recto adelante: (x=+2.0m adelante, y=0.0m centro)
             -> atan2(0.0, 2.0) = 0.0°
             -> heading_error = 0.0°

        Devuelve None si self._path_poses tiene menos de 2 puntos.
        """
        if len(self._path_poses) < 2:
            return None

        accumulated_dist = 0.0
        target_x, target_y = self._path_poses[-1]

        prev_x, prev_y = 0.0, 0.0
        for x, y in self._path_poses:
            seg_dist = math.hypot(x - prev_x, y - prev_y)
            accumulated_dist += seg_dist
            prev_x, prev_y = x, y
            if accumulated_dist >= self.lookahead_distance_m:
                target_x, target_y = x, y
                break

        angle_base_link_rad = math.atan2(target_y, target_x)
        heading_error = -math.degrees(angle_base_link_rad)
        return float(heading_error)

    # --- BUCLE CENTRAL DE CONTROL ---
    def _control_loop(self):
        now = self.get_clock().now()

        # Guarda de seguridad 1: Pausa externa
        if self._navigation_paused:
            self._stop_robot()
            return

        # Guarda de seguridad 2: Esperando procesamiento de SDK
        if self._awaiting_next_target:
            self._stop_robot()
            elapsed = 0.0
            if self._last_reached_publish_at is not None:
                elapsed = (now - self._last_reached_publish_at).nanoseconds / 1e9
            # Re-publicar REACHED si el manager tardó en procesar
            if elapsed >= self.reached_publish_period_s:
                self._publish_reached(now)
            return

        # Guarda de seguridad 3: Datos insuficientes
        if not self.active_goal or self.current_lat is None or self.current_lon is None:
            return

        # Guarda de seguridad 4: GPS Stale Guard
        if not self._gps_is_fresh():
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("GPS stale: frenando y esperando.", throttle_duration_sec=2.0)
            return

        distance = self.haversine_distance(self.current_lat, self.current_lon, self.target_lat, self.target_lon)

        # 1. EVALUACIÓN DE META ALCANZADA
        if distance <= self.goal_tolerance:
            self._stop_robot()
            if self._reached_since is None:
                self._reached_since = now
            
            # Filtro anti-rebote espacial (Dwell Time)
            if (now - self._reached_since).nanoseconds / 1e9 >= self.goal_dwell_s:
                self.get_logger().info(f"¡Meta Alcanzada! ({distance:.1f}m error geodésico)")
                self.active_goal = False
                self._awaiting_next_target = True
                self._navigation_paused = True
                self._publish_reached(now)
            return

        self._reached_since = None

        if self.current_heading is None:
            self._stop_robot()
            return

        # 2. SELECCIÓN DE FUENTE DE HEADING ERROR (Path BEV con Fallback a GPS)
        heading_error = None
        heading_source = "none"
        if self._path_is_fresh():
            if self._path_valid and len(self._path_poses) >= 2:
                heading_error = self._compute_path_heading_error_deg()
                heading_source = "bev_path"
            elif not self._path_valid:
                # Modo RECOVERY: el planner no encontró camino válido
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self._apply_angular_sign(self.recovery_turn_speed)
                self.cmd_pub.publish(twist)
                self.get_logger().warn(
                    "Planner: sin camino válido (recovery turn activo).",
                    throttle_duration_sec=2.0,
                )
                status = f"[RECOVERY] dist={distance:.1f}m, cmd_v=0.00, cmd_w={twist.angular.z:+.2f}"
                out = String()
                out.data = status
                self.status_pub.publish(out)

                if self.publish_control_debug:
                    now_sec = now.nanoseconds / 1e9
                    heading_rx_sec = (
                        (self._heading_last_rx.nanoseconds / 1e9)
                        if self._heading_last_rx is not None
                        else None
                    )
                    gps_age = (
                        ((now - self._gps_last_update).nanoseconds / 1e9)
                        if self._gps_last_update is not None
                        else None
                    )
                    path_age = (
                        ((now - self._path_last_update).nanoseconds / 1e9)
                        if self._path_last_update is not None
                        else None
                    )
                    debug_payload = {
                        "timestamp_sec": now_sec,
                        "mode": "RECOVERY",
                        "heading_error": None,
                        "heading_source": "none",
                        "current_heading": float(self.current_heading) if self.current_heading is not None else None,
                        "heading_rx_sec": heading_rx_sec,
                        "cmd_linear_x": 0.0,
                        "cmd_angular_z": float(twist.angular.z),
                        "align_phase": self._align_phase,
                        "gps_age_s": gps_age,
                        "path_age_s": path_age,
                        "distance_m": float(distance),
                    }
                    dbg_msg = String()
                    dbg_msg.data = json.dumps(debug_payload)
                    self.control_debug_pub.publish(dbg_msg)
                return

        if heading_error is None:
            # Fallback: cálculo GPS puro de bearing
            bearing = self.calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
            heading_error = self.angle_error_deg(bearing, self.current_heading)
            heading_source = "gps"

        # Umbral dinámico de alineación (más estricto al acercarse)
        align_threshold = (
            self.align_threshold
            if distance <= self.approach_align_distance
            else self.coarse_align_threshold
        )

        twist = Twist()
        
        # 3. MÁQUINA DE ESTADOS: ALIGN vs DRIVE
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
            else: # Fase TURN
                twist.angular.z = self._apply_angular_sign(self._burst_turn_sign * self.turn_speed)
                
                # --- INYECCIÓN DE CONTROL DINÁMICO ---
                # Calculamos el burst on-the-fly según la gravedad del error
                abs_err = abs(heading_error)
                if abs_err > 30.0:
                    dynamic_burst = 0.65  # Giro rápido y agresivo para grandes desviaciones
                elif abs_err > 15.0:
                    dynamic_burst = 0.40  # Giro medio para acercamiento
                else:
                    dynamic_burst = 0.22  # Micro-toque de francotirador para no pasarse del umbral
                
                # Evaluamos el corte contra nuestro burst dynamic, no el estático
                if elapsed >= dynamic_burst:
                    self._begin_align_phase("PAUSE", now)
                    twist.angular.z = 0.0
        else:
            mode = "DRIVE"
            self._align_phase = "PAUSE"
            self._align_phase_started_at = None
            self._burst_turn_sign = 0
            
            twist.linear.x = self.forward_speed
            # Corrección suave sobre la marcha (Proporcional débil)
            correction = self.drive_correction_gain * heading_error
            clamped_angular = max(-self.max_drive_angular, min(self.max_drive_angular, correction))
            twist.angular.z = self._apply_angular_sign(
                max(-self.max_total_drive_angular, min(self.max_total_drive_angular, clamped_angular))
            )

        self.cmd_pub.publish(twist)

        # Telemetría interna para Depuración
        raw_h = self._raw_heading if self._raw_heading is not None else float("nan")
        status = (
            f"[{mode}] dist={distance:.1f}m, "
            f"head_err={heading_error:+.1f}°, "
            f"cmd_v={twist.linear.x:.2f}, cmd_w={twist.angular.z:+.2f}"
        )
        out = String()
        out.data = status
        self.status_pub.publish(out)

        # Publicación de Telemetría para Identificación de Sistema (Fase 5.B)
        if self.publish_control_debug:
            now_sec = now.nanoseconds / 1e9
            heading_rx_sec = (
                (self._heading_last_rx.nanoseconds / 1e9)
                if self._heading_last_rx is not None
                else None
            )
            gps_age = (
                ((now - self._gps_last_update).nanoseconds / 1e9)
                if self._gps_last_update is not None
                else None
            )
            path_age = (
                ((now - self._path_last_update).nanoseconds / 1e9)
                if self._path_last_update is not None
                else None
            )
            debug_payload = {
                "timestamp_sec": now_sec,
                "mode": mode,
                "heading_error": float(heading_error) if heading_error is not None else None,
                "heading_source": heading_source,
                "current_heading": float(self.current_heading) if self.current_heading is not None else None,
                "heading_rx_sec": heading_rx_sec,
                "cmd_linear_x": float(twist.linear.x),
                "cmd_angular_z": float(twist.angular.z),
                "align_phase": self._align_phase,
                "gps_age_s": gps_age,
                "path_age_s": path_age,
                "distance_m": float(distance),
            }
            dbg_msg = String()
            dbg_msg.data = json.dumps(debug_payload)
            self.control_debug_pub.publish(dbg_msg)


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