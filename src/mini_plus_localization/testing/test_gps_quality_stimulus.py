#!/usr/bin/env python3
"""
test_gps_quality_stimulus.py — Test offline de verificación de histéresis en gps_quality_monitor.

Simula las 5 fases requeridas por el Brief 1:
  Fase 1: Señal buena sostenida (HDOP=1.0, cov=1.0) -> gps_reliable = True.
  Fase 2: Degradación breve (1.0s, HDOP=5.0, cov=25.0 < enter_duration=2.0s) -> NO debe disparar cambio.
  Fase 3: Degradación sostenida (3.0s, HDOP=5.0, cov=25.0 > enter_duration=2.0s) -> Dispara True -> False.
  Fase 4: Mejora breve (0.8s, HDOP=1.0, cov=1.0 < exit_duration=1.5s) + retorno a mala -> NO debe disparar cambio.
  Fase 5: Mejora sostenida (3.0s, HDOP=1.0, cov=1.0 > exit_duration=1.5s) -> Dispara False -> True.

Valida que el número total de transiciones sea exactamente 2 (sin oscilaciones intermedias).
"""

import math
import os
import sys
import time

# Agregar directorio scripts/ al path de búsqueda de módulos
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS_DIR = os.path.join(os.path.dirname(_THIS_DIR), "scripts")
if _SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, _SCRIPTS_DIR)

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String


class TestGPSQualityStimulus(Node):
    def __init__(self):
        super().__init__("test_gps_quality_stimulus")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", qos)
        self.reliable_sub = self.create_subscription(
            Bool, "earth_rover/gps_reliable", self._on_reliable, qos
        )

        self.transitions: list[tuple[float, bool, str]] = []
        self.current_reliable: bool | None = None
        self._start_time = time.time()

    def _on_reliable(self, msg: Bool):
        now_rel = time.time() - self._start_time
        val = bool(msg.data)
        if self.current_reliable is None:
            self.current_reliable = val
            print(f"[TEST LISTENER] Estado inicial recibido en t={now_rel:.2f}s: gps_reliable = {val}")
        elif val != self.current_reliable:
            prev = self.current_reliable
            self.current_reliable = val
            desc = f"{prev} -> {val}"
            self.transitions.append((now_rel, val, desc))
            print(f"[TEST LISTENER] TRANSICIÓN DETECTADA en t={now_rel:.2f}s: {desc}")

    def publish_sample(self, hdop: float, status_val: int = NavSatStatus.STATUS_FIX):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "earth_rover_gps"
        msg.status.status = status_val
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = -34.6037
        msg.longitude = -58.3816
        msg.altitude = 25.0

        cov_horiz = float(hdop ** 2)
        msg.position_covariance = [
            cov_horiz, 0.0, 0.0,
            0.0, cov_horiz, 0.0,
            0.0, 0.0, 100.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(msg)


def run_test():
    rclpy.init()
    tester = TestGPSQualityStimulus()

    # Importamos e instanciamos el monitor directamente en el mismo proceso para test determinístico
    from gps_quality_monitor import GPSQualityMonitor

    monitor = GPSQualityMonitor()
    # Parámetros calibrados para el test
    monitor.degraded_enter_threshold = 16.0  # HDOP >= 4.0
    monitor.degraded_exit_threshold = 4.0    # HDOP <= 2.0
    monitor.degraded_enter_duration_s = 2.0  # 2.0s sostenido para indoor
    monitor.degraded_exit_duration_s = 1.5   # 1.5s sostenido para outdoor
    monitor.gps_max_stale_s = 2.0
    monitor.check_rate_hz = 20.0

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(tester)
    executor.add_node(monitor)

    print("\n" + "=" * 78)
    print(" INICIANDO TEST OFFLINE DE HISTÉRESIS: gps_quality_monitor")
    print(" Parámetros de prueba: enter_th=16.0 (dur=2.0s) | exit_th=4.0 (dur=1.5s)")
    print("=" * 78)

    tester._start_time = time.time()
    t_rate = 0.1  # 10 Hz publicación

    def step_simulation(duration_s: float, hdop: float, phase_name: str, status_val: int = NavSatStatus.STATUS_FIX):
        print(f"\n--- {phase_name} (Duración: {duration_s:.1f}s, HDOP: {hdop:.1f}, cov: {hdop**2:.1f}) ---")
        end_t = time.time() + duration_s
        while time.time() < end_t:
            tester.publish_sample(hdop, status_val)
            executor.spin_once(timeout_sec=0.01)
            time.sleep(t_rate)

    # --------------------------------------------------------------------------
    # FASE 1: Señal buena sostenida (3.0s)
    # --------------------------------------------------------------------------
    step_simulation(3.0, hdop=1.0, phase_name="FASE 1: Señal buena sostenida")
    assert tester.current_reliable is True, f"Error Fase 1: se esperaba True, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 0, f"Error Fase 1: no debe haber transiciones, hubo {len(tester.transitions)}"

    # --------------------------------------------------------------------------
    # FASE 2: Degradación breve (1.0s < enter_duration 2.0s)
    # --------------------------------------------------------------------------
    step_simulation(1.0, hdop=5.0, phase_name="FASE 2: Degradación breve (1.0s < 2.0s)")
    # Volvemos a buena para confirmar que el timer de degradación se resetea
    step_simulation(1.5, hdop=1.0, phase_name="FASE 2b: Recuperación inmediata (evita falso positivo)")
    assert tester.current_reliable is True, f"Error Fase 2: se esperaba True, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 0, f"Error Fase 2: degradación breve NO debe disparar cambio, hubo {len(tester.transitions)}"

    # --------------------------------------------------------------------------
    # FASE 3: Degradación sostenida real (3.5s > enter_duration 2.0s)
    # --------------------------------------------------------------------------
    step_simulation(3.5, hdop=5.0, phase_name="FASE 3: Degradación sostenida (3.5s > 2.0s)")
    assert tester.current_reliable is False, f"Error Fase 3: se esperaba False, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 1, f"Error Fase 3: se esperaba exactamente 1 transición (True -> False), hubo {len(tester.transitions)}"
    assert tester.transitions[0][1] is False

    # --------------------------------------------------------------------------
    # FASE 4: Mejora breve (0.8s < exit_duration 1.5s) seguida de recaída
    # --------------------------------------------------------------------------
    step_simulation(0.8, hdop=1.0, phase_name="FASE 4: Mejora breve (0.8s < 1.5s)")
    step_simulation(1.5, hdop=5.0, phase_name="FASE 4b: Recaída en mala señal")
    assert tester.current_reliable is False, f"Error Fase 4: se esperaba False, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 1, f"Error Fase 4: mejora breve NO debe disparar cambio prematuro, hubo {len(tester.transitions)}"

    # --------------------------------------------------------------------------
    # FASE 5: Mejora sostenida real (3.0s > exit_duration 1.5s)
    # --------------------------------------------------------------------------
    step_simulation(3.0, hdop=1.0, phase_name="FASE 5: Mejora sostenida (3.0s > 1.5s)")
    assert tester.current_reliable is True, f"Error Fase 5: se esperaba True, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 2, f"Error Fase 5: se esperaban exactamente 2 transiciones acumuladas, hubo {len(tester.transitions)}"
    assert tester.transitions[1][1] is True

    # --------------------------------------------------------------------------
    # FASE 6 (BONUS): Corte total de señal GPS / Stale Timeout (6.0s sin publicar > stale 2.0s + enter 3.0s)
    # --------------------------------------------------------------------------
    print("\n--- FASE 6: Corte total de señal GPS / Stale (6.0s sin publicar > stale 2.0s + enter 3.0s) ---")
    end_t = time.time() + 6.0
    while time.time() < end_t:
        # NO publicamos /gps/fix
        executor.spin_once(timeout_sec=0.05)

    assert tester.current_reliable is False, f"Error Fase 6: se esperaba False por timeout de GPS, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 3, f"Error Fase 6: se esperaban 3 transiciones acumuladas, hubo {len(tester.transitions)}"
    assert tester.transitions[2][1] is False

    # --------------------------------------------------------------------------
    # FASE 7 (BONUS): Reconexión de GPS y recuperación
    # --------------------------------------------------------------------------
    step_simulation(3.0, hdop=1.0, phase_name="FASE 7: Reconexión de GPS y recuperación sostenida")
    assert tester.current_reliable is True, f"Error Fase 7: se esperaba True, obtenido {tester.current_reliable}"
    assert len(tester.transitions) == 4, f"Error Fase 7: se esperaban 4 transiciones acumuladas, hubo {len(tester.transitions)}"
    assert tester.transitions[3][1] is True

    print("\n" + "=" * 78)
    print(" RESULTADO DEL TEST OFFLINE: EXITOSO (PASSED)")
    print(f" Total de transiciones de estado registradas: {len(tester.transitions)}")
    for i, (t_rel, val, desc) in enumerate(tester.transitions, 1):
        print(f"   Transición {i}: t={t_rel:.2f}s | {desc} | gps_reliable={val}")
    print(" Oscilaciones espurias durante picos breves: 0")
    print("=" * 78 + "\n")

    tester.destroy_node()
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    run_test()
