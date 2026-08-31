#!/usr/bin/env python3
"""
Suite de tests unitarios offline (Brief 13 / M.5 y M.4.2)
Verificación puramente numérica para ejecución en CPU sin requerir hardware físico ni GPU.
"""

import math
import numpy as np
import pytest

from genie_path_planner.planner import traversability_to_cost


# ==============================================================================
# M.5.1 — Tests del Filtro Complementario Roll / Pitch
# ==============================================================================
class ComplementaryFilterSimulator:
    def __init__(self, alpha=0.20, gyro_noise_density=0.005, sigma_acc_deg=0.85):
        self.alpha = alpha
        self.q_gyro = gyro_noise_density
        self.sigma_acc_rad = math.radians(sigma_acc_deg)
        self.roll = 0.0
        self.pitch = 0.0
        self.sigma_tilt_rad = math.radians(1.5)

    def step(self, dt, omega_x, omega_y, roll_acc, pitch_acc, gate_open):
        if gate_open:
            # Propagación gyro + Corrección accel
            roll_pred = self.roll + omega_x * dt
            pitch_pred = self.pitch + omega_y * dt
            self.roll = (1.0 - self.alpha) * roll_pred + self.alpha * roll_acc
            self.pitch = (1.0 - self.alpha) * pitch_pred + self.alpha * pitch_acc

            sigma_pred_sq = self.sigma_tilt_rad**2 + (self.q_gyro**2) * dt
            self.sigma_tilt_rad = math.sqrt(
                (1.0 - self.alpha)**2 * sigma_pred_sq + (self.alpha**2) * (self.sigma_acc_rad**2)
            )
        else:
            # Dead-reckoning angular con gyro
            self.roll += omega_x * dt
            self.pitch += omega_y * dt
            self.sigma_tilt_rad = math.sqrt(self.sigma_tilt_rad**2 + (self.q_gyro**2) * dt)

        return self.roll, self.pitch, self.sigma_tilt_rad


def test_complementary_filter_static_convergence():
    """Rover inclinado estático: el filtro debe converger al ángulo real y la incertidumbre reducirse."""
    filt = ComplementaryFilterSimulator(alpha=0.20)
    target_pitch = math.radians(10.0)
    dt = 2.0  # cadencia del SDK FrodoBots

    for _ in range(25):  # 50 segundos de convergencia
        roll, pitch, sigma = filt.step(dt, omega_x=0.0, omega_y=0.0, roll_acc=0.0, pitch_acc=target_pitch, gate_open=True)

    assert math.isclose(pitch, target_pitch, abs_tol=math.radians(0.2))
    assert math.isclose(roll, 0.0, abs_tol=math.radians(0.1))
    assert sigma < math.radians(1.0)  # La incertidumbre debe bajar


def test_complementary_filter_dynamic_ramp():
    """Rampa de inclinación creciente con gate abierto: debe seguir la rampa continuamente."""
    filt = ComplementaryFilterSimulator(alpha=0.20)
    dt = 2.0
    slope_rad_per_s = math.radians(0.5)  # rampa subiendo a 0.5 deg/s

    pitch_acc = 0.0
    for _ in range(15):
        pitch_acc += slope_rad_per_s * dt
        omega_y = slope_rad_per_s  # el gyro registra la velocidad angular
        roll, pitch, _ = filt.step(dt, omega_x=0.0, omega_y=omega_y, roll_acc=0.0, pitch_acc=pitch_acc, gate_open=True)

    assert math.isclose(pitch, pitch_acc, abs_tol=math.radians(0.5))


def test_complementary_filter_gate_closed_dead_reckoning():
    """Gate cerrado durante 30s con gyro en cero: ángulo se mantiene y la incertidumbre crece."""
    filt = ComplementaryFilterSimulator(alpha=0.20)
    filt.pitch = math.radians(8.0)
    initial_sigma = filt.sigma_tilt_rad
    dt = 2.0

    for _ in range(15):  # 30 segundos
        roll, pitch, sigma = filt.step(dt, omega_x=0.0, omega_y=0.0, roll_acc=math.radians(0.0), pitch_acc=math.radians(0.0), gate_open=False)

    assert math.isclose(pitch, math.radians(8.0), abs_tol=1e-5)  # No se resetea a 0
    assert sigma > initial_sigma  # La incertidumbre creció por deriva


def test_complementary_filter_gate_closed_gyro_integration():
    """Gate cerrado con gyro rotando: debe integrar omega * dt correctamente."""
    filt = ComplementaryFilterSimulator(alpha=0.20)
    filt.roll = 0.0
    dt = 2.0
    omega_x = math.radians(1.0)  # 1 deg/s

    for _ in range(5):  # 10 segundos -> debe rotar 10 grados
        roll, pitch, _ = filt.step(dt, omega_x=omega_x, omega_y=0.0, roll_acc=0.0, pitch_acc=0.0, gate_open=False)

    assert math.isclose(roll, math.radians(10.0), abs_tol=math.radians(0.1))


def test_complementary_filter_gate_transition_continuity():
    """Transición de gate cerrado a abierto: sin discontinuidades instantáneas."""
    filt = ComplementaryFilterSimulator(alpha=0.20)
    filt.pitch = math.radians(10.0)
    dt = 2.0

    # 10s en gate cerrado
    for _ in range(5):
        filt.step(dt, omega_x=0.0, omega_y=0.0, roll_acc=0.0, pitch_acc=0.0, gate_open=False)

    pitch_before_open = filt.pitch
    # Se abre el gate con lectura de acelerómetro en 9.0 deg
    roll_after, pitch_after, _ = filt.step(dt, omega_x=0.0, omega_y=0.0, roll_acc=0.0, pitch_acc=math.radians(9.0), gate_open=True)

    delta = abs(pitch_after - pitch_before_open)
    # Con alpha = 0.20, delta = 0.20 * (10 - 9) = 0.2 deg, nunca un salto de 10 deg a 0 deg
    assert delta <= math.radians(0.3)


# ==============================================================================
# M.5.2 — Tests del Gobernador Dinámico de Velocidad
# ==============================================================================
def compute_v_safe(t_plan_s, t_rtt=0.061, t_delay=0.080, d_horizon=2.40, a_brake=1.5, margin=1.5, min_effective_speed=0.15):
    b_term = max(0.0, float(t_plan_s)) + t_rtt + t_delay
    discrim = (b_term ** 2) + (2.0 * float(d_horizon)) / (margin * a_brake)
    if discrim > 0.0:
        v_safe = a_brake * (math.sqrt(discrim) - b_term)
    else:
        v_safe = 0.0

    if v_safe < min_effective_speed:
        v_safe = 0.0
    return float(v_safe)


def test_velocity_governor_table_values():
    """Verificar valores de la tabla teórica de frenado corregida."""
    # Para d_horizon=2.40m, margin=1.5, a_brake=1.5, t_rtt=0.061, t_delay=0.080 (b = t_plan + 0.141):
    # 0.28s -> 1.65 m/s, 1.0s -> 1.07 m/s, 2.0s -> 0.68 m/s, 4.5s -> 0.33 m/s, 8.13s -> 0.19 m/s
    assert math.isclose(compute_v_safe(0.28), 1.65, abs_tol=0.02)
    assert math.isclose(compute_v_safe(1.00), 1.07, abs_tol=0.02)
    assert math.isclose(compute_v_safe(2.00), 0.68, abs_tol=0.02)
    assert math.isclose(compute_v_safe(4.50), 0.33, abs_tol=0.02)
    assert math.isclose(compute_v_safe(8.13), 0.19, abs_tol=0.02)


def test_velocity_governor_floor_cutoff():
    """Si v_safe cae por debajo de 0.15 m/s, debe retornar 0.0 m/s (Stop & Wait)."""
    # A t_plan = 12.0s, v_safe continuo sería ~0.129 m/s < 0.15 m/s
    assert compute_v_safe(12.0) == 0.0
    assert compute_v_safe(20.0) == 0.0


def test_velocity_governor_p95_spike_reaction():
    """El gobernador con P95 reacciona de inmediato ante un pico de latencia."""
    times = [280.0] * 9 + [5000.0]  # 9 ciclos nominales y 1 pico de 5s
    p95_lat_s = float(np.percentile(times, 95)) / 1000.0
    mean_lat_s = float(np.mean(times)) / 1000.0

    v_safe_p95 = compute_v_safe(p95_lat_s)
    v_safe_mean = compute_v_safe(mean_lat_s)

    assert p95_lat_s > mean_lat_s
    assert v_safe_p95 < v_safe_mean
    assert v_safe_p95 <= 0.55


def test_velocity_governor_degenerate_latencies():
    """Latencia cero o negativa no debe romper ni arrojar excepciones."""
    v_zero = compute_v_safe(0.0)
    v_neg = compute_v_safe(-1.0)
    assert v_zero > 1.50
    assert v_neg == v_zero


def test_fail_safe_velocity_governor_states():
    """Verificar los casos del gobernador fail-safe con require_velocity_governor (Brief 14 / N.1 & Brief 15 / O.2)."""
    forward_speed = 0.35
    geodesic_fallback = 0.20

    def resolve_effective_speed(safe_limit_last_rx, safe_limit_val, age_s, path_following_enabled, require_governor=True):
        if safe_limit_last_rx is None:
            if require_governor:
                # Caso 1a: Esperado pero nunca recibido -> 0.0 m/s
                return 0.0
            else:
                # Caso 1b: No requerido (modo geodésico puro) -> velocidad conservadora
                return max(0.0, min(geodesic_fallback, forward_speed))
        if age_s <= 3.0:
            # Caso 2: Recibido y vigente
            return max(0.0, min(forward_speed, safe_limit_val))
        # Caso 3: Expirado (>3.0s)
        if path_following_enabled:
            return 0.0
        return max(0.0, min(geodesic_fallback, forward_speed))

    # 1a. Nunca recibido y require_governor=True -> 0.0
    assert resolve_effective_speed(None, 0.0, 0.0, True, require_governor=True) == 0.0
    assert resolve_effective_speed(None, 0.0, 0.0, False, require_governor=True) == 0.0

    # 1b. Nunca recibido y require_governor=False (modo geodésico puro) -> 0.20 m/s
    assert math.isclose(resolve_effective_speed(None, 0.0, 0.0, False, require_governor=False), 0.20)

    # 2. Recibido y vigente (0.5s de antigüedad, límite 0.28 m/s) -> 0.28
    assert math.isclose(resolve_effective_speed(True, 0.28, 0.5, True), 0.28)

    # 3. Recibido pero expirado (4.0s) con path following activo -> 0.0
    assert resolve_effective_speed(True, 0.28, 4.0, True) == 0.0

    # 4. Recibido pero expirado (4.0s) en modo geodésico puro -> 0.20
    assert math.isclose(resolve_effective_speed(True, 0.28, 4.0, False), 0.20)


def test_heading_stale_guard():
    """Verificar la guarda de heading stale (Brief 14 / N.2)."""
    max_stale_s = 2.0

    def heading_is_fresh(last_rx, age_s):
        if last_rx is None:
            return False
        return age_s <= max_stale_s

    # Nunca recibido
    assert heading_is_fresh(None, 0.0) is False
    # Vigente (0.5s <= 2.0s)
    assert heading_is_fresh(True, 0.5) is True
    # En el borde (2.0s)
    assert heading_is_fresh(True, 2.0) is True
    # Expirado (2.1s > 2.0s)
    assert heading_is_fresh(True, 2.1) is False


# ==============================================================================
# M.5.3 — Tests de Footprint Derivado
# ==============================================================================
def derive_footprint_px(robot_length_m=0.250, robot_width_m=0.190, resolution=0.03, grid_size=240, bev_h=80, margin=1.05):
    d_circ = math.sqrt(robot_length_m**2 + robot_width_m**2)
    fp = math.ceil((d_circ / resolution) * (grid_size / bev_h) * margin)
    return int(fp)


def test_derived_footprint_px_calculations():
    """Verificar derivación matemática de footprint_px para distintas resoluciones."""
    assert derive_footprint_px(0.250, 0.190, 0.03, 240, 80, 1.05) == 33
    assert derive_footprint_px(0.250, 0.190, 0.05, 240, 56, 1.05) == 29
    assert derive_footprint_px(0.250, 0.190, 0.03, 160, 80, 1.05) == 22


# ==============================================================================
# M.5.4 — Test de Validación de Isotropía
# ==============================================================================
def validate_isotropy(bev_h, bev_w):
    if bev_h != bev_w:
        raise ValueError(
            f"La grilla BEV debe ser cuadrada para garantizar isotropía en GeNIE. "
            f"Dimensiones recibidas: bev_h={bev_h}, bev_w={bev_w}."
        )
    return True


def test_isotropy_validation():
    """Confirmar que grillas rectangulares disparan ValueError y cuadradas pasan."""
    assert validate_isotropy(80, 80) is True
    assert validate_isotropy(134, 134) is True

    with pytest.raises(ValueError, match="La grilla BEV debe ser cuadrada"):
        validate_isotropy(56, 72)

    with pytest.raises(ValueError, match="La grilla BEV debe ser cuadrada"):
        validate_isotropy(80, 120)


# ==============================================================================
# M.5.5 — Tests de Compensación de Tilt en Compás Magnético
# ==============================================================================
def compute_tilt_compensated_heading(mx, my, mz, roll_rad, pitch_rad):
    sin_phi = math.sin(roll_rad)
    cos_phi = math.cos(roll_rad)
    sin_theta = math.sin(pitch_rad)
    cos_theta = math.cos(pitch_rad)

    bx = mx * cos_theta + my * sin_phi * sin_theta + mz * cos_phi * sin_theta
    by = my * cos_phi - mz * sin_phi
    yaw = math.atan2(-by, bx)
    return (math.degrees(yaw) + 360.0) % 360.0


def test_tilt_compensation_synthetic():
    """Verificar recuperación exacta de heading en plano y con inclinaciones de 10 deg."""
    b_mag = 0.5
    inc = math.radians(60.0)
    true_heading_deg = 45.0
    psi_rad = math.radians(true_heading_deg)

    bx_h = b_mag * math.cos(inc) * math.cos(psi_rad)
    by_h = -b_mag * math.cos(inc) * math.sin(psi_rad)
    bz_h = b_mag * math.sin(inc)

    # Caso 1: Plano (roll = 0, pitch = 0)
    h_flat = compute_tilt_compensated_heading(bx_h, by_h, bz_h, roll_rad=0.0, pitch_rad=0.0)
    assert math.isclose(h_flat, true_heading_deg, abs_tol=1e-4)

    # Caso 2: 10 deg Pitch
    pitch_10 = math.radians(10.0)
    mx_pitch = bx_h * math.cos(pitch_10) - bz_h * math.sin(pitch_10)
    my_pitch = by_h
    mz_pitch = bx_h * math.sin(pitch_10) + bz_h * math.cos(pitch_10)

    h_uncompensated = (math.degrees(math.atan2(-my_pitch, mx_pitch)) + 360.0) % 360.0
    assert abs(h_uncompensated - true_heading_deg) > 10.0

    h_compensated_pitch = compute_tilt_compensated_heading(mx_pitch, my_pitch, mz_pitch, roll_rad=0.0, pitch_rad=pitch_10)
    assert math.isclose(h_compensated_pitch, true_heading_deg, abs_tol=1e-4)

    # Caso 3: 10 deg Roll
    roll_10 = math.radians(10.0)
    mx_roll = bx_h
    my_roll = by_h * math.cos(roll_10) + bz_h * math.sin(roll_10)
    mz_roll = -by_h * math.sin(roll_10) + bz_h * math.cos(roll_10)

    h_compensated_roll = compute_tilt_compensated_heading(mx_roll, my_roll, mz_roll, roll_rad=roll_10, pitch_rad=0.0)
    assert math.isclose(h_compensated_roll, true_heading_deg, abs_tol=1e-4)


# ==============================================================================
# M.4.2 — Test del Canal de Confianza
# ==============================================================================
def test_confidence_channel_interpolation():
    """Verificar canal de confianza en traversability_to_cost."""
    trav = np.array([[0.9, 0.9, 0.9]], dtype=np.float32)
    conf = np.array([[1.0, 0.0, 0.5]], dtype=np.float32)
    unknown_cost = 0.20

    cost_map = traversability_to_cost(trav, unknown_cost=unknown_cost, confidence=conf)

    # Celda 0: 100% observada -> cost = 1.0 - 0.9 = 0.10
    assert math.isclose(float(cost_map[0, 0]), 0.10, abs_tol=1e-5)

    # Celda 1: 0% observada -> cost = unknown_cost = 0.20
    assert math.isclose(float(cost_map[0, 1]), unknown_cost, abs_tol=1e-5)

    # Celda 2: 50% observada -> cost = 0.20 + 0.5 * (0.10 - 0.20) = 0.15
    assert math.isclose(float(cost_map[0, 2]), 0.15, abs_tol=1e-5)
