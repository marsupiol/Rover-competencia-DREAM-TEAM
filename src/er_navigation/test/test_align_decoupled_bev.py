#!/usr/bin/env python3
"""
test_align_decoupled_bev.py — Test de regresión para desacople ALIGN vs BEV y FIX 1, 2 y 3.

Verifica:
X.4.1 — En modo ALIGN, el controlador ignora la curvatura de bev_path y calcula el error
         exclusivamente con el rumbo geodésico hacia el objetivo.
X.4.2 — En modo DRIVE, el controlador usa bev_path acotado por max_bev_deviation_deg (bev_clamped).
X.4.3 — En transición DRIVE -> ALIGN por desvío grande sostenido, el controlador entra en ALIGN usando
         el error geodésico fresco sin arrastrar el valor de bev_path.
Tests nuevos (Brief FIX 1, 2, 3):
a. Anti-flapping: no sale a ALIGN con 50° (<65°), ni con 70° a 1.0s (<1.5s dwell), pero sí a 2.0s (>1.5s).
b. Clamp de desviación BEV: curvatura de 80° es acotada a exactamente 45°.
c. Convergencia de alineación: error de 60° converge en 3 ráfagas o menos en <8 segundos.
d. RECOVERY sticky: permanece en RECOVERY ante desvío geodésico de 90° hasta que path_valid sea True.
e. Guard de heading: incertidumbre baja (5°) autoriza ráfaga; incertidumbre alta (15°) espera al compás.
"""

import math
import json
import pytest
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

from er_navigation.gps_waypoint_controller import GPSWaypointController


@pytest.fixture
def ros_context():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def create_mock_controller():
    controller = GPSWaypointController()
    controller.require_velocity_governor = False
    controller.path_following_enabled = True
    controller.pause_after_turn_s = 0.0  # Sin retardo para testing rápido de lógica
    controller.heading_fresh_wait_timeout_s = 2.5
    controller.coarse_align_threshold = 25.0
    controller.align_threshold = 18.0
    controller.approach_align_distance = 8.0
    controller.heading_filter_alpha = 1.0  # Respuesta inmediata para tests unitarios
    controller.drive_abort_threshold_deg = 65.0
    controller.drive_abort_dwell_s = 1.5
    controller.max_bev_deviation_deg = 45.0
    controller.turn_burst_min_s = 0.15
    controller.turn_burst_max_s = 1.20
    controller.yaw_rate_deg_s = 17.0
    controller.turn_burst_damping = 0.6
    controller.heading_trust_threshold_deg = 10.0
    controller.recovery_max_duration_s = 20.0
    controller._safe_velocity_limit = 1.111
    controller._safe_velocity_limit_last_rx = controller.get_clock().now()
    return controller


def make_curved_bev_path(controller, curvature_deg: float = 64.5, length_m: float = 2.0):
    """Genera un Path en base_link con curvatura constante."""
    path_msg = Path()
    path_msg.header.stamp = controller.get_clock().now().to_msg()
    path_msg.header.frame_id = "base_link"

    # En base_link: x = adelante, y = izquierda.
    # Un ángulo theta respecto a +X (adelante):
    # theta_rad = math.radians(-curvature_deg) porque:
    # heading_error = -math.degrees(atan2(y, x)) = curvature_deg -> atan2(y, x) = -curvature_deg
    theta_rad = math.radians(-curvature_deg)
    num_pts = 20
    for i in range(num_pts):
        dist = (i + 1) * (length_m / num_pts)
        pose = PoseStamped()
        pose.header = path_msg.header
        pose.pose.position.x = dist * math.cos(theta_rad)
        pose.pose.position.y = dist * math.sin(theta_rad)
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)
    return path_msg


def test_align_ignores_bev_path_curvature_and_converges_on_geodesic(ros_context):
    """X.4.1: ALIGN ignora la curvatura de bev_path y transiciona a DRIVE por rumbo geodésico."""
    controller = create_mock_controller()

    # Rover en lat0, lon0. Meta al Norte (bearing = 0.0°, distancia ~55m > 8m -> coarse_align = 25°)
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005  # Norte
    target.longitude = 0.0
    controller._on_target(target)

    # Inyectar bev_path constante con curvatura de +64.5°
    bev_path = make_curved_bev_path(controller, curvature_deg=64.5)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    def on_debug(msg: String):
        nonlocal last_debug
        last_debug = json.loads(msg.data)
    controller.control_debug_pub.publish = on_debug

    published_cmds = []
    controller.cmd_pub.publish = lambda twist: published_cmds.append(
        (twist.linear.x, twist.angular.z)
    )

    # 1. Rover apuntando al Este (heading = 90.0°). Meta está a 0°.
    # Error geodésico = 0° - 90° = -90° (abs > 25° -> ALIGN).
    controller._on_heading(Float32(data=90.0))
    controller._control_loop()

    assert last_debug.get("mode") == "ALIGN"
    assert last_debug.get("heading_source") == "geodesic"
    assert math.isclose(last_debug.get("heading_error"), -90.0, abs_tol=0.1)
    # Debe tener velocidad lineal CERO en ALIGN
    assert published_cmds[-1][0] == 0.0

    # 2. El rover gira progresivamente hacia el Norte: 60° -> 35° -> 20°
    controller._on_heading(Float32(data=35.0))
    controller._control_loop()
    assert last_debug.get("mode") == "ALIGN"
    assert last_debug.get("heading_source") == "geodesic"
    assert math.isclose(last_debug.get("heading_error"), -35.0, abs_tol=0.1)

    # 3. El compás cruza el umbral de alineación: heading = 10.0° (error = -10° < 25°)
    # ¡A pesar de que bev_path sigue reportando +64.5°!
    controller._on_heading(Float32(data=10.0))
    controller._control_loop()

    # Debe haber transicionado a DRIVE
    assert last_debug.get("mode") == "DRIVE"
    # En DRIVE, ahora sí puede usar bev_path con corrección acotada para el avance
    assert last_debug.get("heading_source") == "bev_clamped"
    # Y comanda velocidad lineal > 0 para avanzar
    assert published_cmds[-1][0] > 0.0


def test_drive_consumes_bev_path_for_steering(ros_context):
    """X.4.2: En modo DRIVE, el controlador sí consume bev_path para evasión reactiva (bev_clamped)."""
    controller = create_mock_controller()

    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005
    target.longitude = 0.0
    controller._on_target(target)

    # Rover ya alineado con la meta (heading = 0.0°, error geodésico = 0°)
    controller._on_heading(Float32(data=0.0))

    # Inyectar bev_path con leve desvío hacia la derecha (+15° en heading_error)
    bev_path = make_curved_bev_path(controller, curvature_deg=15.0)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    def on_debug(msg: String):
        nonlocal last_debug
        last_debug = json.loads(msg.data)
    controller.control_debug_pub.publish = on_debug

    controller._control_loop()

    assert last_debug.get("mode") == "DRIVE"
    assert last_debug.get("heading_source") == "bev_clamped"
    assert math.isclose(last_debug.get("heading_error"), 15.0, abs_tol=1.0)
    assert math.isclose(last_debug.get("geodesic_heading_error"), 0.0, abs_tol=0.1)


def test_drive_to_align_transition_uses_geodesic_without_drag(ros_context):
    """X.4.3: Si el rover se desvía en DRIVE más del umbral por más del dwell, entra a ALIGN con error geodésico."""
    controller = create_mock_controller()

    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005
    target.longitude = 0.0
    controller._on_target(target)

    # Arrancar en DRIVE con heading = 0°
    controller._on_heading(Float32(data=0.0))
    bev_path = make_curved_bev_path(controller, curvature_deg=5.0)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    def on_debug(msg: String):
        nonlocal last_debug
        last_debug = json.loads(msg.data)
    controller.control_debug_pub.publish = on_debug

    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE"
    assert last_debug.get("heading_source") == "bev_clamped"

    # Simular que el rover se desvía bruscamente a heading = 70° (error geodésico = -70° > 65°)
    controller._on_heading(Float32(data=70.0))
    controller._control_loop()

    # Dwell no cumplido todavía: sigue en DRIVE
    assert last_debug.get("mode") == "DRIVE"
    assert controller._drive_abort_started_at is not None

    # Simular paso del dwell time (> 1.5s)
    controller._drive_abort_started_at = controller.get_clock().now() - Duration(nanoseconds=int(2.0 * 1e9))
    controller._control_loop()

    # Debe transicionar a ALIGN
    assert last_debug.get("mode") == "ALIGN"
    # El heading_error debe ser el geodésico (-70°), NO el del bev_path
    assert last_debug.get("heading_source") == "geodesic"
    assert math.isclose(last_debug.get("heading_error"), -70.0, abs_tol=0.1)


# ==============================================================================
# TESTS NUEVOS OBLIGATORIOS (FIX 1, FIX 2, FIX 3)
# ==============================================================================

def test_anti_flapping_drive_to_align_dwell(ros_context):
    """3.a Anti-flapping: en DRIVE con bev_path a +64.5°, desvío a 50° no aborta.
    Desvío a 70° por 1.0s tampoco aborta (<1.5s dwell).
    Desvío a 70° por 2.0s sí conmuta a ALIGN."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005  # bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Iniciar en DRIVE alineado (heading = 0.0°)
    controller._on_heading(Float32(data=0.0))
    bev_path = make_curved_bev_path(controller, curvature_deg=64.5)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    controller.control_debug_pub.publish = lambda msg: last_debug.update(json.loads(msg.data))

    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE"

    # 1. Simular que el rover gira siguiendo la corrección hasta que geodesic_heading_error llega a 50°
    controller._on_heading(Float32(data=50.0))
    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE", "No debe volver a ALIGN con error = 50° (< 65°)"

    # 2. Llevarlo a heading = 70.0° (|geodesic_error| = 70° > 65°) por 1.0 s (< 1.5 s dwell)
    controller._on_heading(Float32(data=70.0))
    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE", "No debe conmutar a ALIGN en el primer ciclo a 70°"
    assert controller._drive_abort_started_at is not None

    # Simular que transcurrió 1.0s de desvío continuo
    controller._drive_abort_started_at = controller.get_clock().now() - Duration(nanoseconds=int(1.0 * 1e9))
    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE", "No debe conmutar con 1.0s de desvío (< 1.5s dwell)"

    # 3. Sostener 70° por 2.0 s (> 1.5 s dwell)
    controller._drive_abort_started_at = controller.get_clock().now() - Duration(nanoseconds=int(2.0 * 1e9))
    controller._control_loop()
    assert last_debug.get("mode") == "ALIGN", "Debe conmutar a ALIGN tras sostener 70° por 2.0s (> 1.5s dwell)"


def test_bev_deviation_clamped_to_max_allowed(ros_context):
    """3.b Clamp de desviación BEV: bev_path pidiendo 80° respecto a bearing resulta en exactamente 45°."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005  # bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Rover alineado con bearing (heading = 0.0°)
    controller._on_heading(Float32(data=0.0))

    # BEV pide curvatura de +80.0° respecto a base_link (desviación de 80° respecto a bearing 0°)
    bev_path = make_curved_bev_path(controller, curvature_deg=80.0)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    controller.control_debug_pub.publish = lambda msg: last_debug.update(json.loads(msg.data))

    controller._control_loop()
    assert last_debug.get("mode") == "DRIVE"
    assert last_debug.get("heading_source") == "bev_clamped"
    # Debe ser exactamente 45.0°, no 80.0°
    assert math.isclose(last_debug.get("heading_error"), 45.0, abs_tol=1.0)


def test_align_convergence_in_three_bursts(ros_context):
    """3.c Convergencia de alineación: ALIGN con error inicial de 60°, girando a 17°/s.
    Converge por debajo del umbral en 3 ráfagas o menos, en < 8.0 segundos."""
    controller = create_mock_controller()
    controller.pause_after_turn_s = 0.5
    controller.yaw_rate_deg_s = 17.0
    controller.turn_burst_damping = 0.6
    controller.turn_burst_min_s = 0.15
    controller.turn_burst_max_s = 1.20
    controller.heading_trust_threshold_deg = 10.0
    controller._heading_uncertainty_deg = 5.0  # Propagación confiable (no espera al compás)

    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005  # bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Heading inicial: 60.0° (bearing=0.0°, error = -60.0°)
    current_head = 60.0
    controller._on_heading(Float32(data=current_head))

    burst_count = 0
    total_simulated_time_s = 0.0
    max_iterations = 20

    for _ in range(max_iterations):
        controller._control_loop()
        if controller._control_mode == "DRIVE":
            break

        if controller._align_phase == "TURN":
            burst_count += 1
            duration = controller._current_burst_duration
            total_simulated_time_s += duration
            # Simular giro físico del rover a 17°/s
            turn_deg = duration * controller.yaw_rate_deg_s
            current_head = (current_head - turn_deg) % 360.0
            controller._on_heading(Float32(data=current_head))
            # Simular expiración de la ráfaga
            controller._align_phase_started_at = controller.get_clock().now() - Duration(nanoseconds=int(duration * 1e9 + 1e6))
            controller._control_loop()

        if controller._align_phase == "PAUSE":
            total_simulated_time_s += controller.pause_after_turn_s
            controller._align_phase_started_at = controller.get_clock().now() - Duration(nanoseconds=int(controller.pause_after_turn_s * 1e9 + 1e6))

    assert controller._control_mode == "DRIVE", "El controlador debió converger a DRIVE"
    assert burst_count <= 3, f"Convergió en {burst_count} ráfagas (> 3)"
    assert total_simulated_time_s < 8.0, f"Tiempo total {total_simulated_time_s:.2f}s (>= 8.0s)"


def test_recovery_sticky_prevents_align_oscillation(ros_context):
    """3.d RECOVERY sticky: rover en RECOVERY con path_valid=False.
    Giro de recovery lleva error a 90°. NO sale a ALIGN.
    Al marcar path_valid=True, sale a DRIVE."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005
    target.longitude = 0.0
    controller._on_target(target)

    # Iniciar alineado en DRIVE
    controller._on_heading(Float32(data=0.0))
    bev_path = make_curved_bev_path(controller, curvature_deg=0.0)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    last_debug = {}
    controller.control_debug_pub.publish = lambda msg: last_debug.update(json.loads(msg.data))

    controller._control_loop()
    assert controller._control_mode == "DRIVE"

    # Simular que el path deja de ser válido (bloqueo frontal)
    controller._on_path_valid(Bool(data=False))
    controller._control_loop()
    assert controller._control_mode == "RECOVERY"
    assert last_debug.get("mode") == "RECOVERY"

    # Simular que el giro de recovery lleva el rumbo a 90° (error geodésico = -90° >> 65°)
    controller._on_heading(Float32(data=90.0))
    controller._control_loop()

    # FIX 3: NO debe salir a ALIGN mientras el path siga inválido y timeout no se cumpla
    assert controller._control_mode == "RECOVERY", "RECOVERY debe ser sticky y no salir a ALIGN por error geodésico"
    assert last_debug.get("mode") == "RECOVERY"

    # Ahora el planner encuentra camino transitable
    controller._on_path_valid(Bool(data=True))
    controller._on_planned_path(bev_path)
    controller._control_loop()

    # Debe salir inmediatamente a DRIVE
    assert controller._control_mode == "DRIVE", "Al validar path, debe salir a DRIVE"
    assert last_debug.get("mode") == "DRIVE"


def test_heading_uncertainty_guard(ros_context):
    """3.e Guard de heading: incertidumbre baja (5.0°) autoriza ráfaga sin compás fresco.
    Incertidumbre alta (15.0°) espera al compás."""
    controller = create_mock_controller()
    controller.pause_after_turn_s = 0.5
    controller.heading_trust_threshold_deg = 10.0

    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0005
    target.longitude = 0.0
    controller._on_target(target)

    # Iniciar con error de 45°
    controller._on_heading(Float32(data=45.0))
    controller._control_loop()
    assert controller._align_phase == "TURN"

    # Pasar a PAUSE y simular que transcurrió pause_after_turn_s (0.6s > 0.5s)
    controller._begin_align_phase("PAUSE", controller.get_clock().now() - Duration(nanoseconds=int(0.6 * 1e9)))
    assert controller._last_turn_heading_seq is not None

    # Caso 1: _compass_seq sin cambios, pero heading_uncertainty = 5.0° (< 10.0°)
    controller._heading_uncertainty_deg = 5.0
    controller._heading_uncertainty_last_rx = controller.get_clock().now()
    controller._control_loop()

    # SÍ debe autorizar la ráfaga (pasa a TURN)
    assert controller._align_phase == "TURN", "Incertidumbre baja (5.0°) debe autorizar ráfaga sin esperar compás"

    # Volver a pasar a PAUSE transcurriendo el tiempo mínimo
    controller._begin_align_phase("PAUSE", controller.get_clock().now() - Duration(nanoseconds=int(0.6 * 1e9)))

    # Caso 2: _compass_seq sin cambios, pero heading_uncertainty = 15.0° (>= 10.0°)
    controller._heading_uncertainty_deg = 15.0
    controller._heading_uncertainty_last_rx = controller.get_clock().now()
    controller._control_loop()

    # Debe permanecer en PAUSE esperando al compás
    assert controller._align_phase == "PAUSE", "Incertidumbre alta (15.0°) debe mantener PAUSE esperando compás"


def test_closed_loop_constant_bev_curvature_sustains_drive_without_flapping(ros_context):
    """5. Verificación en lazo cerrado: con bev_path devolviendo curvatura constante de +64.5°,
    la cinemática de giro y avance se sostiene en modo DRIVE sin entrar en el bucle ALIGN<->DRIVE."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0010  # ~111m al Norte, bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Rover inicialmente alineado (heading = 0.0°, en DRIVE)
    current_heading = 0.0
    controller._on_heading(Float32(data=current_heading))

    # Path con curvatura de +64.5°
    bev_path = make_curved_bev_path(controller, curvature_deg=64.5)
    controller._on_planned_path(bev_path)
    controller._on_path_valid(Bool(data=True))

    published_modes = []
    def on_debug(msg: String):
        data = json.loads(msg.data)
        published_modes.append(data.get("mode"))
    controller.control_debug_pub.publish = on_debug

    published_twists = []
    controller.cmd_pub.publish = lambda twist: published_twists.append(
        (twist.linear.x, twist.angular.z)
    )

    # Simular 30 ciclos de control a dt = 0.2s (6 segundos simulados)
    dt = 0.2
    for _ in range(30):
        controller._control_loop()
        cmd_v, cmd_w = published_twists[-1]
        # Integrar cinemática: el comando cmd_w gira el rover
        # En el Mini+, cmd_w positivo gira a la derecha (heading aumenta)
        current_heading = (current_heading - cmd_w * 17.0 * dt) % 360.0
        controller._on_heading(Float32(data=current_heading))

    # Verificar que tras el ciclo inicial, se mantuvo 100% en DRIVE sin un solo flapping a ALIGN
    assert len(published_modes) == 30
    assert all(m == "DRIVE" for m in published_modes), f"Ocurrió flapping: modos={published_modes}"
    # Y que avanzó linealmente (cmd_v > 0)
    assert all(v > 0.0 for v, w in published_twists), "El rover debe sostener tracción continua en DRIVE"


def test_drive_proportional_no_saturation_in_operating_range(ros_context):
    """4.a Sin saturación en el rango operativo:
    Con heading_error de 10°, 25° y 45°, verificar que twist.angular.z es proporcional
    en los tres casos (no saturado), y que ninguno excede 1.0."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    # Meta al Norte (bearing = 0.0°)
    target = NavSatFix()
    target.latitude = 0.0010
    target.longitude = 0.0
    controller._on_target(target)

    # Forzar modo DRIVE y habilitar límite de velocidad seguro
    controller._control_mode = "DRIVE"
    controller.path_following_enabled = False  # Para probar heading_error geodésico directamente
    controller._safe_velocity_limit = 1.111
    controller._safe_velocity_limit_last_rx = controller.get_clock().now()

    published_twists = []
    controller.cmd_pub.publish = lambda twist: published_twists.append(twist)

    errors = [10.0, 25.0, 45.0]
    angular_cmds = []

    for err in errors:
        # Rumbo para producir heading_error = +err:
        # angle_error_deg(0.0, heading) = err -> heading = (360.0 - err) % 360.0
        controller.current_heading = (360.0 - err) % 360.0
        controller._heading_last_rx = controller.get_clock().now()
        controller._control_loop()
        twist = published_twists[-1]
        angular_cmds.append(abs(twist.angular.z))

    # 1. Proporcionalidad estricta en todo el rango
    for err, cmd in zip(errors, angular_cmds):
        expected_cmd = controller.drive_correction_gain * err
        assert math.isclose(cmd, expected_cmd, rel_tol=1e-3), (
            f"Para error={err}°, cmd={cmd:.4f} debe ser estrictamente proporcional a {expected_cmd:.4f}"
        )
        assert cmd <= 1.0, f"cmd={cmd} no debe exceder 1.0"

    # 2. No saturado (cada valor es estrictamente mayor que el anterior)
    assert angular_cmds[0] < angular_cmds[1] < angular_cmds[2], (
        f"Comandos angulares deben crecer monotónicamente sin saturar: {angular_cmds}"
    )


def test_recovery_bidirectional_turns_towards_goal(ros_context):
    """4.b RECOVERY bidireccional:
    Con geodesic_heading_error = +50°, verificar que RECOVERY gira en un sentido;
    con -50°, verificar que gira en el opuesto."""
    target = NavSatFix()
    target.latitude = 0.0010  # Norte, bearing = 0.0°
    target.longitude = 0.0

    # Caso 1: Heading = 310.0° -> geodesic_heading_error = +50.0° (meta a la derecha)
    controller1 = create_mock_controller()
    controller1.current_lat = 0.0
    controller1.current_lon = 0.0
    controller1._gps_last_update = controller1.get_clock().now()
    controller1._on_target(target)
    controller1.current_heading = 310.0
    controller1._heading_last_rx = controller1.get_clock().now()
    controller1._control_mode = "DRIVE"
    controller1._path_valid = False
    controller1._path_last_update = controller1.get_clock().now()

    published_twists_1 = []
    controller1.cmd_pub.publish = lambda twist: published_twists_1.append(twist)

    controller1._control_loop()
    assert controller1._control_mode == "RECOVERY"
    cmd_w_pos = published_twists_1[-1].angular.z

    # Caso 2: Heading = 50.0° -> geodesic_heading_error = -50.0° (meta a la izquierda)
    controller2 = create_mock_controller()
    controller2.current_lat = 0.0
    controller2.current_lon = 0.0
    controller2._gps_last_update = controller2.get_clock().now()
    controller2._on_target(target)
    controller2.current_heading = 50.0
    controller2._heading_last_rx = controller2.get_clock().now()
    controller2._control_mode = "DRIVE"
    controller2._path_valid = False
    controller2._path_last_update = controller2.get_clock().now()

    published_twists_2 = []
    controller2.cmd_pub.publish = lambda twist: published_twists_2.append(twist)

    controller2._control_loop()
    assert controller2._control_mode == "RECOVERY"
    cmd_w_neg = published_twists_2[-1].angular.z

    # Ambos deben girar con recovery_turn_throttle y con signos opuestos
    assert math.isclose(abs(cmd_w_pos), controller1.recovery_turn_throttle, rel_tol=1e-3)
    assert math.isclose(abs(cmd_w_neg), controller2.recovery_turn_throttle, rel_tol=1e-3)
    assert (cmd_w_pos * cmd_w_neg) < 0.0, (
        f"Giro de RECOVERY debe ser de signos opuestos (+50° -> {cmd_w_pos:+.2f}, -50° -> {cmd_w_neg:+.2f})"
    )


def test_governor_expired_stops_both_linear_and_angular_in_drive(ros_context):
    """4.c Gobernador expirado detiene giro en DRIVE:
    Simular age_safe_vel > 3.0s en DRIVE con heading_error != 0,
    verificar que TANTO twist.linear.x COMO twist.angular.z quedan en 0.0."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0010  # Norte, bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Modo DRIVE con heading desalineado 20° y path_following activo
    controller._control_mode = "DRIVE"
    controller.current_heading = 20.0
    controller._heading_last_rx = controller.get_clock().now()
    controller.path_following_enabled = True
    controller._path_valid = True
    controller._path_last_update = controller.get_clock().now()
    controller._path_poses = [(1.0, 0.0), (2.0, 0.0)]

    # Simular gobernador expirado (4.0s > 3.0s)
    now = controller.get_clock().now()
    controller._safe_velocity_limit_last_rx = now - Duration(nanoseconds=int(4.0 * 1e9))

    published_twists = []
    controller.cmd_pub.publish = lambda twist: published_twists.append(twist)

    controller._control_loop()

    assert len(published_twists) == 1
    twist = published_twists[0]
    assert twist.linear.x == 0.0, f"Lineal debe ser 0.0 ante gobernador expirado, obtenido {twist.linear.x}"
    assert twist.angular.z == 0.0, f"Angular debe ser 0.0 ante gobernador expirado en DRIVE, obtenido {twist.angular.z}"


def test_align_and_recovery_unaffected_by_governor_expiration(ros_context):
    """4.d ALIGN y RECOVERY no afectados por FIX 4:
    Verificar que en esos modos el giro se mantiene aunque el gobernador esté expirado."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0010
    target.longitude = 0.0
    controller._on_target(target)

    # Simular gobernador expirado (5.0s > 3.0s)
    now = controller.get_clock().now()
    controller._safe_velocity_limit_last_rx = now - Duration(nanoseconds=int(5.0 * 1e9))

    published_twists = []
    controller.cmd_pub.publish = lambda twist: published_twists.append(twist)

    # 1. En ALIGN durante fase TURN:
    controller._control_mode = "ALIGN"
    controller._align_phase = "TURN"
    controller._align_phase_started_at = now
    controller._burst_turn_sign = 1
    controller.current_heading = 40.0
    controller._heading_last_rx = now

    controller._control_loop()
    assert len(published_twists) == 1
    twist_align = published_twists[-1]
    assert twist_align.linear.x == 0.0
    assert math.isclose(abs(twist_align.angular.z), controller.turn_throttle, rel_tol=1e-3), (
        f"ALIGN TURN debe comandar turn_throttle={controller.turn_throttle}, obtenido {twist_align.angular.z}"
    )

    # 2. En RECOVERY:
    controller._control_mode = "RECOVERY"
    controller._recovery_started_at = now
    controller._recovery_turn_sign = 1

    controller._control_loop()
    twist_recovery = published_twists[-1]
    assert twist_recovery.linear.x == 0.0
    assert math.isclose(abs(twist_recovery.angular.z), controller.recovery_turn_throttle, rel_tol=1e-3), (
        f"RECOVERY debe comandar recovery_turn_throttle={controller.recovery_turn_throttle}, obtenido {twist_recovery.angular.z}"
    )


def test_closed_loop_sil_virtual_obstacle_evasion_and_recovery(ros_context):
    """6. Verificación en lazo cerrado SIL:
    Confirmar que ante un obstáculo virtual el rover ejecuta una maniobra de evasión completa
    (desvío reactivo BEV, evasión en RECOVERY ante bloqueo total, despeje y retorno al rumbo en DRIVE)
    sin flapping y sin quedar trabado."""
    controller = create_mock_controller()
    controller.current_lat = 0.0
    controller.current_lon = 0.0
    controller._gps_last_update = controller.get_clock().now()

    target = NavSatFix()
    target.latitude = 0.0010  # ~111m al Norte, bearing = 0.0°
    target.longitude = 0.0
    controller._on_target(target)

    # Iniciar alineado en DRIVE (heading = 0.0°)
    current_heading = 0.0
    controller._on_heading(Float32(data=current_heading))

    published_modes = []
    controller.control_debug_pub.publish = lambda msg: published_modes.append(json.loads(msg.data).get("mode"))

    published_twists = []
    controller.cmd_pub.publish = lambda twist: published_twists.append((twist.linear.x, twist.angular.z))

    dt = 0.2
    # Fase 1: Marcha normal recta en DRIVE (10 ciclos = 2s)
    straight_path = make_curved_bev_path(controller, curvature_deg=0.0)
    controller._on_planned_path(straight_path)
    controller._on_path_valid(Bool(data=True))

    for _ in range(10):
        controller._control_loop()
        cmd_v, cmd_w = published_twists[-1]
        current_heading = (current_heading - cmd_w * 17.0 * dt) % 360.0
        controller._on_heading(Float32(data=current_heading))

    assert all(m == "DRIVE" for m in published_modes[:10])

    # Fase 2: Obstáculo detectado -> BEV comanda desvío reactivo de +35° (15 ciclos = 3s)
    obstacle_path = make_curved_bev_path(controller, curvature_deg=35.0)
    controller._on_planned_path(obstacle_path)

    for _ in range(15):
        controller._control_loop()
        cmd_v, cmd_w = published_twists[-1]
        current_heading = (current_heading - cmd_w * 17.0 * dt) % 360.0
        controller._on_heading(Float32(data=current_heading))

    # Verificar que el desvío reactivo se ejecutó 100% en DRIVE sin flapping a ALIGN
    assert all(m == "DRIVE" for m in published_modes[10:25]), (
        f"Ocurrió flapping durante el desvío BEV: {published_modes[10:25]}"
    )

    # Fase 3: Bloqueo completo repentino (path_valid = False) -> entra a RECOVERY (5 ciclos = 1s)
    controller._on_path_valid(Bool(data=False))
    for _ in range(5):
        controller._control_loop()
        cmd_v, cmd_w = published_twists[-1]
        current_heading = (current_heading - cmd_w * 17.0 * dt) % 360.0
        controller._on_heading(Float32(data=current_heading))

    assert all(m == "RECOVERY" for m in published_modes[25:30]), (
        f"Debe permanecer en RECOVERY mientras path_valid sea False: {published_modes[25:30]}"
    )

    # Fase 4: Despeje del obstáculo -> path_valid = True con path recto
    controller._on_path_valid(Bool(data=True))
    controller._on_planned_path(straight_path)

    for _ in range(15):
        controller._control_loop()
        cmd_v, cmd_w = published_twists[-1]
        current_heading = (current_heading - cmd_w * 17.0 * dt) % 360.0
        controller._on_heading(Float32(data=current_heading))

    # Debe retornar a DRIVE inmediatamente al despejarse y sostener avance
    assert all(m == "DRIVE" for m in published_modes[30:]), (
        f"Debe reanudar DRIVE sin quedarse trabado: {published_modes[30:]}"
    )
