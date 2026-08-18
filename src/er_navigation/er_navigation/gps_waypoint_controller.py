#!/usr/bin/env python3
"""
GPS Waypoint Navigation Controller for Earth Rover (IROS 2026).
Arquitectura Híbrida: Máquina de estados reactiva con mitigación de latencia de red (Burst & Wait)
y filtrado pasa-bajos para brújula ruidosa.
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

        # --- Tiempos de Ráfaga y Filtros ---
        self.turn_burst_s = float(self.get_parameter("turn_burst_s").value)
        self.pause_after_turn_s = float(self.get_parameter("pause_after_turn_s").value)
        self.max_heading_jump = float(self.get_parameter("max_heading_jump_deg").value)

        self.heading_filter_alpha = float(self.get_parameter("heading_filter_alpha").value)
        self.reached_publish_period_s = float(self.get_parameter("reached_publish_period_s").value)
        self.loop_hz = float(self.get_parameter("control_loop_hz").value)

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
        self.create_subscription(NavSatFix, "earth_rover/target_waypoint", self._on_target, reliable_qos)
        self.create_subscription(Bool, "earth_rover/navigation_pause", self._on_navigation_pause, reliable_qos)
        self.create_subscription(String, "earth_rover/waypoint_status", self._on_mission_status, reliable_qos)

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", reliable_qos)
        self.status_pub = self.create_publisher(String, "earth_rover/waypoint_status", reliable_qos)

        # 5. Inicialización de Vectores de Estado
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self._raw_heading = None
        self.target_lat = None
        self.target_lon = None
        
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

    # --- CALLBACKS DE SENSORES ---
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
            # AHORA SABREMOS SI LA BRÚJULA SE ESTÁ RECHAZANDO
            self.get_logger().warn(f"Salto magnético gigante rechazado: {jump:.1f}°")
            return

        delta = self.angle_error_deg(raw, self.current_heading)
        self.current_heading = (self.current_heading + self.heading_filter_alpha * delta) % 360.0

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

    # --- HELPERS DE TIEMPO Y PUBLICACIÓN ---
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

        bearing = self.calculate_bearing(self.current_lat, self.current_lon, self.target_lat, self.target_lon)
        heading_error = self.angle_error_deg(bearing, self.current_heading)

        # Umbral dinámico de alineación (más estricto al acercarse)
        align_threshold = (
            self.align_threshold
            if distance <= self.approach_align_distance
            else self.coarse_align_threshold
        )

        twist = Twist()
        
        # 2. MÁQUINA DE ESTADOS: ALIGN vs DRIVE
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
                
                # Evaluamos el corte contra nuestro burst dinámico, no el estático
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
            twist.angular.z = self._apply_angular_sign(
                max(-self.max_drive_angular, min(self.max_drive_angular, correction))
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