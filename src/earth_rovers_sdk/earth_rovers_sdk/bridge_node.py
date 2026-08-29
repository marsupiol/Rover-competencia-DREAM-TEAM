"""ROS2 bridge node for the Earth Rovers SDK.

This module is adapted from `mini_plus_localization/scripts/earth_rover_bridge.py`
and exposed as a `console_scripts` entrypoint so it can be run with
`ros2 run earth_rovers_sdk earth_rover_bridge` after `colcon build`.

Changes vs. the original version (see chat for details):
  - Compass/magnetometer heading is converted from the "0=North, clockwise"
    convention to the ROS/REP-103 ENU yaw convention (0=East, counter-clockwise)
    before it's used anywhere (IMU orientation, odom orientation, dead-reckoning).
  - IMU / Odometry / GPS publishers use RELIABLE QoS to match robot_localization's
    default subscriber QoS (BEST_EFFORT publisher + RELIABLE subscriber = no data
    delivered at all, which silently starves the EKF).
  - GPS topic renamed to /gps/fix, the default navsat_transform_node expects.
"""

import json
import math
import threading
import time

import cv2
import rclpy
import requests
import websocket
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Image, Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String

CONTROL_RATE_HZ = 10.0
CMD_VEL_TIMEOUT_S = 0.5
CONTROL_HTTP_TIMEOUT_S = 1.0
GRAVITY_M_S2 = 9.80665

# ==============================================================================
# TENSOR DE COVARIANZAS (Recalibración IROS 2026)
# ==============================================================================
# ODOMETRÍA DE RUEDAS (Cinemática)
# Penalizamos la velocidad lineal Y (vy) porque el rover no debería patinar 
# lateralmente. Confiamos moderadamente en vx (avance) pero asumimos deslizamiento.
ODOM_POSE_COVARIANCE = [
    0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.5, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
]
ODOM_TWIST_COVARIANCE = [
    0.5,  0.0,  0.0,  0.0,  0.0,  0.0, # vx: deslizamiento longitudinal moderado
    0.0,  2.0,  0.0,  0.0,  0.0,  0.0, # vy: penalización por resbalón lateral
    0.0,  0.0, 10.0,  0.0,  0.0,  0.0, # vz: ignorado (rover terrestre)
    0.0,  0.0,  0.0, 10.0,  0.0,  0.0, # roll rate: ruidoso, se ignora
    0.0,  0.0,  0.0,  0.0, 10.0,  0.0, # pitch rate: ruidoso, se ignora
    0.0,  0.0,  0.0,  0.0,  0.0,  0.2, # yaw rate: confiable pero sujeto a derrapes
]

# IMU MPU-6050 (Fusión Inercial)
# Varianzas infladas para absorber la vibración estructural de los 4 motores DC
# y el jitter introducido por la latencia de la red 4G/LTE.
IMU_ORIENTATION_COVARIANCE = [
    0.01, 0.0,  0.0,
    0.0,  0.01, 0.0,
    0.0,  0.0,  0.01, # Magnetómetro absoluto: El ancla principal de nuestro yaw
]
IMU_ANGULAR_VELOCITY_COVARIANCE = [
    0.01, 0.0,  0.0,
    0.0,  0.01, 0.0,
    0.0,  0.0,  0.05, # Yaw rate: penalizado para mitigar el retardo de fase de red
]
IMU_LINEAR_ACCELERATION_COVARIANCE = [
    0.05, 0.0,  0.0,
    0.0,  0.05, 0.0,
    0.0,  0.0,  0.1,  # Z-accel: absorbe los impactos mecánicos contra el terreno
]

# SISTEMA GNSS (Posicionamiento Global)
# Ajustado a 1 metro de precisión autónoma (Varianza = 1.0^2 = 1.0)
GPS_POSITION_COVARIANCE = [
    1.0,  0.0,  0.0,
    0.0,  1.0,  0.0,
    0.0,  0.0, 100.0, # Altitud sigue penalizada masivamente
]
# ==============================================================================

class EarthRoverBridge(Node):
    def __init__(self):
        super().__init__("earth_rover_bridge")
        self.declare_parameter("sdk_url", "http://localhost:8000")
        self.declare_parameter("feed_fps", 15)
        self.sdk_url = self.get_parameter("sdk_url").value.rstrip("/")
        self.feed_fps = int(self.get_parameter("feed_fps").value)

        self.bridge = CvBridge()

        # Camera: best-effort is fine and desirable here, we don't want the
        # feed thread blocked waiting for slow consumers.
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # IMU / Odom / GPS feed robot_localization, whose subscriptions default
        # to RELIABLE. A BEST_EFFORT publisher + RELIABLE subscriber pair is an
        # incompatible QoS combination in ROS2 -- messages get silently dropped
        # at the DDS layer and the EKF never receives anything, even though the
        # topics look "connected". Use RELIABLE here to match.
        filter_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        command_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1)

        self.image_pub = self.create_publisher(
            Image, "earth_rover/front/image_raw", image_qos
        )
        # navsat_transform_node's default input topic is /gps/fix.
        # If your launch file remaps this differently, adjust the topic name
        # here (or add a remap in the launch file) so they match.
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", filter_qos)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", filter_qos)
        self.odom_pub = self.create_publisher(Odometry, "/wheel_odom", filter_qos)
        self.battery_pub = self.create_publisher(
            BatteryState, "earth_rover/battery", image_qos
        )
        # Heading crudo de la brújula (sin filtrar). Tópico exclusivo de diagnóstico/debug.
        # Los nodos de navegación (gps_waypoint_controller, bev_planner_node) consumen
        # exclusivamente el heading fusionado por el EKF en 'earth_rover/heading'.
        self.heading_pub = self.create_publisher(
            Float32, "earth_rover/heading_raw", image_qos
        )

        self.declare_parameter("publish_bridge_debug", True)
        self.publish_bridge_debug = bool(self.get_parameter("publish_bridge_debug").value)
        self.bridge_debug_pub = self.create_publisher(
            String, "earth_rover/bridge_debug", image_qos
        )

        self.declare_parameter("odom_pose_covariance", ODOM_POSE_COVARIANCE)
        self.declare_parameter("odom_twist_covariance", ODOM_TWIST_COVARIANCE)
        self.declare_parameter(
            "imu_orientation_covariance", IMU_ORIENTATION_COVARIANCE
        )
        self.declare_parameter(
            "imu_angular_velocity_covariance", IMU_ANGULAR_VELOCITY_COVARIANCE
        )
        self.declare_parameter(
            "imu_linear_acceleration_covariance", IMU_LINEAR_ACCELERATION_COVARIANCE
        )
        self.declare_parameter("gps_position_covariance", GPS_POSITION_COVARIANCE)
        # Set this to your local magnetic declination (radians, positive = East)
        # ONLY if you'd rather correct it here instead of in navsat_transform's
        # `magnetic_declination_radians` param (that's the more idiomatic place,
        # this param is provided as a convenience / fallback).
        self.declare_parameter("magnetic_declination_radians", 0.0)

        self._odom_pose_covariance = self.get_parameter(
            "odom_pose_covariance"
        ).value
        self._odom_twist_covariance = self.get_parameter(
            "odom_twist_covariance"
        ).value
        self._imu_orientation_covariance = self.get_parameter(
            "imu_orientation_covariance"
        ).value
        self._imu_angular_velocity_covariance = self.get_parameter(
            "imu_angular_velocity_covariance"
        ).value
        self._imu_linear_acceleration_covariance = self.get_parameter(
            "imu_linear_acceleration_covariance"
        ).value
        self._gps_position_covariance = self.get_parameter(
            "gps_position_covariance"
        ).value
        self._magnetic_declination_radians = float(
            self.get_parameter("magnetic_declination_radians").value
        )

        self._latest_cmd = None
        self._last_cmd_at = 0.0
        self._last_cmd_rx_ros_sec = None
        self._stopped = True
        self._cmd_lock = threading.Lock()

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._last_odom_time = None
        self._last_yaw = None
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, command_qos)

        self._session = requests.Session()
        self._running = True
        self._stop_event = threading.Event()
        self._control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._control_thread.start()
        threading.Thread(target=self._feed_loop, daemon=True).start()
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

        self.get_logger().info(f"Bridging Earth Rovers SDK at {self.sdk_url}")

    def _on_cmd_vel(self, msg: Twist):
        with self._cmd_lock:
            self._latest_cmd = {
                "linear": max(-1.0, min(1.0, msg.linear.x)),
                "angular": max(-1.0, min(1.0, msg.angular.z)),
            }
            self._last_cmd_at = time.monotonic()
            self._last_cmd_rx_ros_sec = self.get_clock().now().nanoseconds / 1e9
            self._stopped = False

    def _control_tick(self):
        with self._cmd_lock:
            quiet = time.monotonic() - self._last_cmd_at > CMD_VEL_TIMEOUT_S
            if self._latest_cmd is None or (quiet and self._stopped):
                return
            command = {"linear": 0, "angular": 0} if quiet else dict(self._latest_cmd)
            last_cmd_at = self._last_cmd_at
            last_rx_ros_sec = self._last_cmd_rx_ros_sec

        send_mono = time.monotonic()
        send_ros_sec = self.get_clock().now().nanoseconds / 1e9
        status_code = None
        error_str = None

        try:
            response = self._session.post(
                f"{self.sdk_url}/control",
                json={"command": command},
                timeout=CONTROL_HTTP_TIMEOUT_S,
            )
            status_code = response.status_code
            response.raise_for_status()

            if quiet:
                with self._cmd_lock:
                    if (
                        self._last_cmd_at == last_cmd_at
                        and time.monotonic() - self._last_cmd_at > CMD_VEL_TIMEOUT_S
                    ):
                        self._stopped = True
        except requests.RequestException as e:
            error_str = str(e)
            if hasattr(e, "response") and e.response is not None:
                status_code = getattr(e.response, "status_code", status_code)
            self.get_logger().warning(f"/control failed: {e}", throttle_duration_sec=5)
        finally:
            resp_mono = time.monotonic()
            resp_ros_sec = self.get_clock().now().nanoseconds / 1e9
            roundtrip_ms = (resp_mono - send_mono) * 1000.0

            if self.publish_bridge_debug:
                dbg_payload = {
                    "type": "control_actuation",
                    "cmd_rx_ros_sec": last_rx_ros_sec,
                    "http_send_ros_sec": send_ros_sec,
                    "http_resp_ros_sec": resp_ros_sec,
                    "roundtrip_ms": roundtrip_ms,
                    "status_code": status_code,
                    "command": command,
                    "error": error_str,
                }
                dbg_msg = String()
                dbg_msg.data = json.dumps(dbg_payload)
                self.bridge_debug_pub.publish(dbg_msg)

    def _control_loop(self):
        interval = 1.0 / CONTROL_RATE_HZ
        deadline = time.monotonic()
        while self._running and rclpy.ok():
            self._control_tick()
            deadline += interval
            wait = max(0.0, deadline - time.monotonic())
            if self._stop_event.wait(wait):
                break
            if time.monotonic() - deadline > interval:
                deadline = time.monotonic()

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _compass_heading_to_enu_yaw(self, heading_deg: float) -> float:
        """Convert a compass heading (degrees, 0=North, clockwise-positive) into
        a ROS/REP-103 ENU yaw (radians, 0=East, counter-clockwise-positive).

        This is the conversion robot_localization / navsat_transform assume for
        the IMU orientation they fuse. Getting this wrong flips or offsets the
        heading used to rotate GPS readings into the odom/map frame.
        """
        heading_rad = math.radians(heading_deg)
        yaw = (math.pi / 2.0) - heading_rad
        yaw += self._magnetic_declination_radians
        return self._normalize_angle(yaw)

    def _feed_loop(self):
        url = f"{self.sdk_url}/feed?view=front&fps={self.feed_fps}"
        while self._running and rclpy.ok():
            capture = cv2.VideoCapture(url)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                self.get_logger().warning(
                    "/feed not available, retrying in 3s", throttle_duration_sec=10
                )
                time.sleep(3)
                continue
            self.get_logger().info("Connected to /feed")
            while self._running and rclpy.ok():
                ok, frame = capture.read()
                if not ok:
                    break
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "earth_rover_front_camera"
                self.image_pub.publish(msg)
            capture.release()
            time.sleep(1)

    def _telemetry_loop(self):
        ws_url = self.sdk_url.replace("http", "ws", 1) + "/ws/data"
        while self._running and rclpy.ok():
            ws = None
            try:
                ws = websocket.create_connection(ws_url, timeout=10)
                self.get_logger().info("Connected to /ws/data")
                while self._running and rclpy.ok():
                    msg = json.loads(ws.recv())
                    if msg.get("type") in ("snapshot", "telemetry") and msg.get("data"):
                        self._publish_telemetry(msg["data"])
            except Exception as e:
                self.get_logger().warning(
                    f"/ws/data reconnecting: {e}", throttle_duration_sec=10
                )
                time.sleep(2)
            finally:
                if ws is not None:
                    ws.close()

    def _publish_telemetry(self, data: dict):
        now = self.get_clock().now().to_msg()

        # ---------------------------------------------------------
        # 1. GNSS (GPS) CON COVARIANZA DINÁMICA AVANZADA
        # ---------------------------------------------------------
        lat, lng = data.get("latitude"), data.get("longitude")
        if lat is not None and lng is not None:
            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = "earth_rover_gps"

            # 1.1 Extracción de Metadatos del Hardware
            gps_signal = data.get("gps_signal")   # Cantidad de satélites
            fix_quality = data.get("fix_quality") # Flag NMEA oficial de la placa
            
            try:
                hdop = float(data.get("hdop", 1.0))
            except (TypeError, ValueError):
                hdop = 1.0

            # 1.2 Máquina de Estados de Validación de Fix
            is_fix_valid = False
            
            if fix_quality is not None:
                # Prioridad Absoluta: El hardware reporta si logró resolver la ecuación
                if int(fix_quality) > 0:
                    is_fix_valid = True
            elif gps_signal is not None:
                # Respaldo: Heurística matemática si el hardware no expone fix_quality
                try:
                    sats = float(gps_signal)
                    # 4 satélites es el mínimo algebraico real para resolver x, y, z, t
                    if sats >= 4.0: 
                        is_fix_valid = True
                    
                    # Generación de HDOP sintético si el SDK no lo empaqueta
                    if "hdop" not in data:
                        hdop = max(1.0, 10.0 / (sats + 1e-6))
                except (TypeError, ValueError):
                    pass

            if is_fix_valid:
                gps.status.status = NavSatStatus.STATUS_FIX
            else:
                gps.status.status = NavSatStatus.STATUS_NO_FIX
                hdop = 50.0 # Castigo masivo a la covarianza ante pérdida de anclaje

            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.latitude = float(lat)
            gps.longitude = float(lng)

            # 1.3 Escalado Tensorial de Covarianza
            # Multiplicamos la matriz original completa por HDOP^2 mediante list comprehension.
            # Esto preserva el tensor original inyectado por ROS 2 intacto.
            hdop_factor = hdop ** 2
            gps.position_covariance = [
                cov * hdop_factor for cov in self._gps_position_covariance
            ]
            # Cambiamos a APPROXIMATED porque el HDOP es una dilución geométrica, no una varianza directa
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            
            # 1.4 Guarda de Seguridad del Grafo Computacional
            if is_fix_valid and hdop < 20.0:
                self.gps_pub.publish(gps)
            else:
                self.get_logger().warn(
                    f"GNSS descartado (HDOP: {hdop:.1f}, Sats: {gps_signal}, Fix: {fix_quality}). "
                    "Forzando EKF a Dead-Reckoning.",
                    throttle_duration_sec=2.0
                )

        # ---------------------------------------------------------
        # 2. ORIENTACIÓN MAGNÉTICA (Compass)
        # ---------------------------------------------------------
        orientation = data.get("orientation")
        yaw = None
        if orientation is not None:
            try:
                heading_deg = float(orientation)
            except (TypeError, ValueError):
                heading_deg = None
            if heading_deg is not None:
                heading = Float32()
                heading.data = heading_deg
                self.heading_pub.publish(heading)
                yaw = self._compass_heading_to_enu_yaw(heading_deg)

        # ---------------------------------------------------------
        # 3. BATERÍA
        # ---------------------------------------------------------
        battery = data.get("battery")
        if battery is not None:
            batt = BatteryState()
            batt.header.stamp = now
            batt.percentage = float(battery) / 100.0
            batt.present = True
            self.battery_pub.publish(batt)

        # ---------------------------------------------------------
        # 4. IMU Y ODOMETRÍA DE RUEDAS
        # ---------------------------------------------------------
        accels = data.get("accels") or []
        gyros = data.get("gyros") or []
        
        # Verificamos que tengamos orientación absoluta para cumplir REP-105
        if accels and yaw is not None:
            # Iteramos sobre el array completo de 100 muestras
            num_samples = len(accels)
            # Frecuencia estimada: 100 muestras en 2 segundos = 50 Hz = 20ms por muestra
            dt_sample_sec = 2.0 / num_samples 
            
            # Timestamp del paquete actual (la muestra más reciente)
            base_time_sec = now.sec + (now.nanosec / 1e9)

            for i, sample in enumerate(accels):
                imu = Imu()
                # Interpolar hacia atrás en el tiempo: la muestra 0 ocurrió hace 2 segundos
                time_offset = dt_sample_sec * (num_samples - 1 - i)
                sample_time = base_time_sec - time_offset
                
                imu.header.stamp.sec = int(sample_time)
                imu.header.stamp.nanosec = int((sample_time - int(sample_time)) * 1e9)
                imu.header.frame_id = "base_link"
                
                imu.linear_acceleration.x = float(sample[0]) * GRAVITY_M_S2
                imu.linear_acceleration.y = float(sample[1]) * GRAVITY_M_S2
                imu.linear_acceleration.z = float(sample[2]) * GRAVITY_M_S2

                # Reutilizamos el único giro y yaw disponible en este reporte de 2s
                if gyros:
                    imu.angular_velocity.x = math.radians(float(gyros[-1][0]))
                    imu.angular_velocity.y = math.radians(float(gyros[-1][1]))
                    imu.angular_velocity.z = math.radians(float(gyros[-1][2]))

                imu.orientation.x = 0.0
                imu.orientation.y = 0.0
                imu.orientation.z = math.sin(yaw / 2.0)
                imu.orientation.w = math.cos(yaw / 2.0)

                imu.orientation_covariance = self._imu_orientation_covariance
                imu.angular_velocity_covariance = self._imu_angular_velocity_covariance
                imu.linear_acceleration_covariance = self._imu_linear_acceleration_covariance
                
                self.imu_pub.publish(imu)

        speed = data.get("speed")
        speed_m_s = None
        if speed is not None:
            try:
                speed_m_s = float(speed)
            except (TypeError, ValueError):
                speed_m_s = None

        yaw_rate = None
        if gyros:
            try:
                yaw_rate = math.radians(float(gyros[-1][2]))
            except (TypeError, ValueError, IndexError):
                yaw_rate = None

        current_time = time.monotonic()
        dt = None
        if self._last_odom_time is not None:
            dt = current_time - self._last_odom_time
            
        if speed_m_s is not None and yaw is not None and dt is not None and dt > 0:
            self._odom_x += speed_m_s * math.cos(yaw) * dt
            self._odom_y += speed_m_s * math.sin(yaw) * dt

        if speed_m_s is not None and yaw is not None:
            if yaw_rate is None and self._last_yaw is not None and dt is not None and dt > 0:
                yaw_rate = self._normalize_angle(yaw - self._last_yaw) / dt
            self._last_yaw = yaw
            self._last_odom_time = current_time

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self._odom_x
            odom.pose.pose.position.y = self._odom_y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            )
            odom.pose.covariance = self._odom_pose_covariance
            odom.twist.twist.linear.x = speed_m_s
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.z = yaw_rate if yaw_rate is not None else 0.0
            odom.twist.covariance = self._odom_twist_covariance
            self.odom_pub.publish(odom)

    def destroy_node(self):
        self._running = False
        self._stop_event.set()
        self._control_thread.join(timeout=1.0)
        for _ in range(3):
            try:
                response = self._session.post(
                    f"{self.sdk_url}/control",
                    json={"command": {"linear": 0, "angular": 0}},
                    timeout=CONTROL_HTTP_TIMEOUT_S,
                )
                response.raise_for_status()
                break
            except requests.RequestException:
                continue
        self._session.close()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = EarthRoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()