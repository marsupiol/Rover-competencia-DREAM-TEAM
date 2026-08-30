#!/usr/bin/env python3
"""
gps_quality_monitor.py — Detector de calidad de GPS con histéresis incorporada.

Evalúa la confiabilidad del sensor GNSS a partir de /gps/fix crudo:
  1. Estado del fix (NavSatStatus: STATUS_NO_FIX -> degradado).
  2. Covarianza horizontal escalada por HDOP^2 (degradado si >= enter_threshold).
  3. Frescura temporal (degradado si edad > gps_max_stale_s).

Implementa máquina de estados de histéresis con umbrales y tiempos diferenciados:
  - Outdoor -> Indoor: Requiere condición degradada sostenida por degraded_enter_duration_s.
  - Indoor -> Outdoor: Requiere condición recuperada sostenida por degraded_exit_duration_s.

Publica el booleano estabilizado en earth_rover/gps_reliable.
"""

from __future__ import annotations

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String


class GPSQualityMonitor(Node):
    def __init__(self):
        super().__init__("gps_quality_monitor")

        # ----------------------------------------------------------------------
        # Declaración de Parámetros de ROS (Configurables en tiempo de ejecución)
        # ----------------------------------------------------------------------
        # 1. Umbral de covarianza horizontal para considerar señal degradada (m^2)
        # En bridge_node, cov = 1.0 * HDOP^2. 16.0 m^2 equivale a HDOP >= 4.0.
        self.declare_parameter("degraded_enter_threshold", 16.0)

        # 2. Umbral de covarianza horizontal para considerar señal recuperada (m^2)
        # 4.0 m^2 equivale a HDOP <= 2.0 (condición excelente requerida para volver a outdoor).
        self.declare_parameter("degraded_exit_threshold", 4.0)

        # 3. Duración continua con señal degradada para disparar transición a Indoor (s)
        # Filtra parpadeos y micro-oclusiones momentáneas.
        self.declare_parameter("degraded_enter_duration_s", 3.0)

        # 4. Duración continua con señal recuperada para disparar transición a Outdoor (s)
        # Confirmación para evitar oscilaciones en bordes de edificios.
        self.declare_parameter("degraded_exit_duration_s", 1.5)

        # 5. Antigüedad máxima admisible sin recibir mensajes antes de marcar stale (s)
        self.declare_parameter("gps_max_stale_s", 2.0)

        # 6. Frecuencia del watchdog de evaluación (Hz)
        self.declare_parameter("check_rate_hz", 10.0)

        # 7. Asunción inicial de confiabilidad al inicializar el nodo
        self.declare_parameter("initial_state_reliable", True)

        # 8. Nombres de tópicos de entrada y salida
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("reliable_topic", "earth_rover/gps_reliable")
        self.declare_parameter("publish_debug", True)
        self.declare_parameter("publish_on_change_only", True)

        # Lectura de parámetros
        self.degraded_enter_threshold = float(self.get_parameter("degraded_enter_threshold").value)
        self.degraded_exit_threshold = float(self.get_parameter("degraded_exit_threshold").value)
        self.degraded_enter_duration_s = float(self.get_parameter("degraded_enter_duration_s").value)
        self.degraded_exit_duration_s = float(self.get_parameter("degraded_exit_duration_s").value)
        self.gps_max_stale_s = float(self.get_parameter("gps_max_stale_s").value)
        self.check_rate_hz = float(self.get_parameter("check_rate_hz").value)
        self.initial_state_reliable = bool(self.get_parameter("initial_state_reliable").value)
        self.gps_topic = str(self.get_parameter("gps_topic").value)
        self.reliable_topic = str(self.get_parameter("reliable_topic").value)
        self.publish_debug = bool(self.get_parameter("publish_debug").value)
        self.publish_on_change_only = bool(self.get_parameter("publish_on_change_only").value)

        # ----------------------------------------------------------------------
        # Estado Interno de la Máquina de Histéresis
        # ----------------------------------------------------------------------
        self._is_reliable: bool = self.initial_state_reliable
        self._last_published_state: bool | None = None
        self._last_msg: NavSatFix | None = None
        self._last_rx_time: rclpy.time.Time | None = None
        self._degraded_since: float | None = None
        self._recovered_since: float | None = None

        # ----------------------------------------------------------------------
        # Perfiles de QoS y Comunicaciones ROS
        # ----------------------------------------------------------------------
        qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self._on_gps_fix,
            qos_reliable,
        )

        self.reliable_pub = self.create_publisher(
            Bool,
            self.reliable_topic,
            qos_reliable,
        )

        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                String,
                "earth_rover/gps_quality_debug",
                qos_reliable,
            )

        # Watchdog periódico para evaluar expiración de frescura y timers
        timer_period = 1.0 / max(1.0, self.check_rate_hz)
        self._timer = self.create_timer(timer_period, self._evaluate_state)

        self.get_logger().info(
            f"GPSQualityMonitor iniciado | enter_th={self.degraded_enter_threshold:.1f} (dur={self.degraded_enter_duration_s:.1f}s) | "
            f"exit_th={self.degraded_exit_threshold:.1f} (dur={self.degraded_exit_duration_s:.1f}s) | "
            f"stale={self.gps_max_stale_s:.1f}s | state={'RELIABLE' if self._is_reliable else 'DEGRADED'}"
        )

    def _extract_horizontal_covariance(self, msg: NavSatFix) -> float:
        """Extrae la covarianza horizontal máxima del tensor de posición de NavSatFix."""
        cov = msg.position_covariance
        if cov is not None and len(cov) >= 5:
            cov_x = float(cov[0])
            cov_y = float(cov[4])
            if math.isnan(cov_x) or math.isinf(cov_x):
                return float("inf")
            if math.isnan(cov_y) or math.isinf(cov_y):
                return float("inf")
            return max(cov_x, cov_y)
        return float("inf")

    def _on_gps_fix(self, msg: NavSatFix):
        self._last_msg = msg
        self._last_rx_time = self.get_clock().now()
        # Evaluación inmediata al recibir muestra para mínima latencia
        self._evaluate_state()

    def _evaluate_state(self):
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        # 1. Verificación de Frescura Temporal
        is_fresh = False
        gps_age_s = float("inf")
        if self._last_rx_time is not None:
            gps_age_s = (now - self._last_rx_time).nanoseconds / 1e9
            is_fresh = (gps_age_s <= self.gps_max_stale_s)

        # 2. Clasificación Instantánea de la Muestra
        is_bad = False
        is_good = False
        current_cov = float("inf")
        fix_status = NavSatStatus.STATUS_NO_FIX

        if not is_fresh or self._last_msg is None:
            is_bad = True
        else:
            fix_status = self._last_msg.status.status
            current_cov = self._extract_horizontal_covariance(self._last_msg)

            if fix_status < 0 or fix_status == NavSatStatus.STATUS_NO_FIX:
                is_bad = True
            elif current_cov >= self.degraded_enter_threshold:
                is_bad = True
            elif current_cov <= self.degraded_exit_threshold:
                is_good = True
            else:
                # Zona de histéresis (degraded_exit_threshold < cov < degraded_enter_threshold)
                pass

        # 3. Máquina de Estados con Histéresis Temporal
        state_changed = False

        if self._is_reliable:
            # Modo actual: OUTDOOR (Reliable)
            self._recovered_since = None

            if is_bad:
                if self._degraded_since is None:
                    self._degraded_since = now_sec
                elapsed_bad = now_sec - self._degraded_since
                if elapsed_bad >= self.degraded_enter_duration_s:
                    self._is_reliable = False
                    self._degraded_since = None
                    state_changed = True
                    self.get_logger().warn(
                        "GPS degradado detectado, entrando a modo indoor"
                    )
            else:
                self._degraded_since = None

        else:
            # Modo actual: INDOOR (Degraded)
            self._degraded_since = None

            if is_good:
                if self._recovered_since is None:
                    self._recovered_since = now_sec
                elapsed_good = now_sec - self._recovered_since
                if elapsed_good >= self.degraded_exit_duration_s:
                    self._is_reliable = True
                    self._recovered_since = None
                    state_changed = True
                    self.get_logger().info(
                        "GPS recuperado, volviendo a modo outdoor"
                    )
            else:
                self._recovered_since = None

        # 4. Publicación en earth_rover/gps_reliable (en transiciones y primer tick)
        should_publish = False
        if self._last_published_state is None:
            should_publish = True
        elif state_changed:
            should_publish = True
        elif not self.publish_on_change_only:
            should_publish = True

        if should_publish:
            msg_out = Bool()
            msg_out.data = self._is_reliable
            self.reliable_pub.publish(msg_out)
            self._last_published_state = self._is_reliable

        # 5. Publicación de Telemetría de Depuración (Opcional)
        if self.publish_debug:
            deg_timer = (now_sec - self._degraded_since) if self._degraded_since is not None else 0.0
            rec_timer = (now_sec - self._recovered_since) if self._recovered_since is not None else 0.0
            debug_payload = {
                "timestamp_sec": now_sec,
                "is_reliable": self._is_reliable,
                "fix_status": int(fix_status),
                "horizontal_covariance": float(current_cov) if not math.isinf(current_cov) else -1.0,
                "gps_age_s": float(gps_age_s) if not math.isinf(gps_age_s) else -1.0,
                "is_fresh": bool(is_fresh),
                "is_bad": bool(is_bad),
                "is_good": bool(is_good),
                "degraded_timer_s": float(deg_timer),
                "recovered_timer_s": float(rec_timer),
            }
            dbg_msg = String()
            dbg_msg.data = json.dumps(debug_payload)
            self.debug_pub.publish(dbg_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSQualityMonitor()
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
