#!/usr/bin/env python3
"""
Benchmark formal de latencias para Brief 17 en GPU RTX 5060.
Mide de forma desacoplada y combinada:
- infer_nn (SAM-TP)
- bev_proj (proyección geométrica ray-casting BEV)
- plan_genie (evaluación de caminos, K-Means, costo y selección)
- frame_total (pipeline completo)
- conteo de caminos vivos (alive_paths)
- desglose interno de plan_genie en terreno despejado
"""

import math
import os
import random
import time
from typing import Any, Dict, List, Tuple

import cv2
import numpy as np
import torch

from genie_path_planner.path_sampling import sample_paths_polynomial
from genie_path_planner.planner import (
    PlannerConfig,
    PlannedPath,
    _planner_path_to_bev_path,
    _resize_float_map,
    _resize_mask,
    _resize_pixel,
    _smooth_cost,
    adaptive_kmeans,
    bev_pixel_to_xy,
    compute_paths_costs,
    filter_paths_with_high_costs,
    goal_xy_to_bev_pixel,
    merge_centroids_by_angle,
    pick_final_path,
    plan_on_bev,
    select_best_group_by_closest_path_angle,
    traversability_to_cost,
)
from genie_path_planner.projection import project_score_to_bev
from rover_traversability.calibration import load_camera_K, load_T_base_camera
from rover_traversability.predictor import TraversabilityPredictor


def create_test_scenarios() -> Dict[str, np.ndarray]:
    """Genera las imágenes RGB (1024x576 BGR8) para los 4 escenarios de prueba."""
    h, w = 576, 1024
    horizon = int(h * 0.45)

    def base_ground():
        img = np.zeros((h, w, 3), dtype=np.uint8)
        # Cielo (azul claro / grisáceo)
        for y in range(horizon):
            ratio = y / horizon
            b = int(220 * (1 - 0.3 * ratio))
            g = int(180 * (1 - 0.2 * ratio))
            r = int(140 * (1 - 0.1 * ratio))
            img[y, :] = (b, g, r)
        # Suelo transitable
        for y in range(horizon, h):
            ratio = (y - horizon) / (h - horizon)
            b = int(70 + 40 * ratio)
            g = int(90 + 50 * ratio)
            r = int(110 + 60 * ratio)
            img[y, :] = (b, g, r)
        noise = np.random.RandomState(42).randint(-15, 15, (h - horizon, w, 3))
        ground_patch = img[horizon:, :].astype(np.int16) + noise
        img[horizon:, :] = np.clip(ground_patch, 0, 255).astype(np.uint8)
        return img

    # Escenario 1: Terreno despejado (todo transitable)
    img_clear = base_ground()

    # Escenario 2: Obstáculo frontal centrado (bloque no transitable en el centro de la trayectoria)
    img_frontal = base_ground()
    # Obstáculo prominente: caja / roca frontal a ~2m de distancia
    cv2.rectangle(img_frontal, (w // 2 - 130, horizon + 60), (w // 2 + 130, h - 30), (25, 25, 180), -1)
    cv2.rectangle(img_frontal, (w // 2 - 130, horizon + 60), (w // 2 + 130, h - 30), (10, 10, 80), 3)

    # Escenario 3: Obstáculos laterales (corredor tipo vereda / callejón estrecho)
    img_lateral = base_ground()
    # Vallas/paredes laterales que bloquean izquierda y derecha
    cv2.rectangle(img_lateral, (0, horizon), (w // 2 - 90, h), (30, 30, 160), -1)
    cv2.rectangle(img_lateral, (w // 2 + 90, horizon), (w, h), (30, 30, 160), -1)

    # Escenario 4: Escena mixta realista (obstáculo asimétrico + elemento lateral)
    img_mixed = base_ground()
    # Obstáculo a la izquierda en media distancia y obstáculo a la derecha en lejana distancia
    cv2.rectangle(img_mixed, (0, horizon + 20), (w // 2 - 50, h - 80), (35, 35, 160), -1)
    cv2.circle(img_mixed, (w // 2 + 180, horizon + 100), 75, (20, 20, 175), -1)

    return {
        "1. Terreno despejado": img_clear,
        "2. Obstaculo frontal centrado": img_frontal,
        "3. Obstaculos laterales (corredor)": img_lateral,
        "4. Escena mixta realista": img_mixed,
    }


def compute_stats(arr: List[float]) -> Dict[str, float]:
    """Calcula media, P95, max y min de una lista de valores."""
    a = np.array(arr, dtype=np.float64)
    return {
        "mean": float(np.mean(a)),
        "p95": float(np.percentile(a, 95)),
        "max": float(np.max(a)),
        "min": float(np.min(a)),
    }


def plan_on_bev_with_profiling(
    bev_traversability: np.ndarray,
    observed_mask: np.ndarray | None,
    goal_x_m: float,
    goal_y_m: float,
    bev_resolution_m: float,
    config: PlannerConfig,
    candidate_path_bank: List[np.ndarray],
) -> Tuple[PlannedPath, Dict[str, float]]:
    """Ejecuta plan_on_bev midiendo con precisión cada etapa interna."""
    t0 = time.perf_counter()
    cfg = config
    bev = np.asarray(bev_traversability, dtype=np.float32)
    known0 = np.isfinite(bev) & (bev >= 0.0)
    if observed_mask is not None:
        known0 &= np.asarray(observed_mask, dtype=bool)

    start0 = (bev.shape[0] - 1, bev.shape[1] // 2)
    goal0 = goal_xy_to_bev_pixel(float(goal_x_m), float(goal_y_m), bev.shape, float(bev_resolution_m))
    cost0 = traversability_to_cost(bev, unknown_cost=float(cfg.unknown_cost))
    cost0 = _smooth_cost(cost0, int(cfg.smooth_kernel))

    planner_cost = _resize_float_map(cost0, int(cfg.grid_size))
    planner_known = _resize_mask(known0, int(cfg.grid_size))
    planner_start = _resize_pixel(start0, bev.shape, int(cfg.grid_size))
    planner_goal = _resize_pixel(goal0, bev.shape, int(cfg.grid_size))

    candidate_paths = candidate_path_bank

    t_filter_start = time.perf_counter()
    num_points_to_filter = int(min(max(1, int(cfg.number_of_points_to_filter)), int(cfg.path_num_samples) + 1))
    filtered_paths = filter_paths_with_high_costs(
        candidate_paths,
        planner_cost,
        num_points=num_points_to_filter,
        footprint_px=int(cfg.footprint_px),
        threshold_points_ratio=float(cfg.threshold_points_ratio),
        threshold_cost=float(cfg.threshold_cost),
    )
    t_filter_end = time.perf_counter()

    t_kmeans_start = time.perf_counter()
    selected_paths = filtered_paths
    if bool(cfg.use_clustering) and len(filtered_paths) >= 2:
        try:
            labels, centroids, best_k = adaptive_kmeans(
                filtered_paths,
                min_clusters=1,
                max_clusters=int(cfg.max_clusters),
            )
            if len(centroids) >= 2:
                _merged, groups, _angles = merge_centroids_by_angle(
                    centroids,
                    angle_threshold_deg=float(cfg.cluster_angle_threshold_deg),
                )
                dr = float(planner_goal[0] - planner_start[0])
                dc = float(planner_goal[1] - planner_start[1])
                robot_goal_angle = math.degrees(math.atan2(dr, dc))
                best_group_idx, _best_diff = select_best_group_by_closest_path_angle(
                    paths=filtered_paths,
                    labels=labels,
                    groups=groups,
                    robot_goal_angle=robot_goal_angle,
                    lookahead_index=30,
                )
                best_group = groups[best_group_idx]
                selected_paths = [p for p, lab in zip(filtered_paths, labels) if int(lab) in best_group]
        except Exception:
            selected_paths = filtered_paths
    if len(selected_paths) == 0:
        selected_paths = filtered_paths
    t_kmeans_end = time.perf_counter()

    t_cost_start = time.perf_counter()
    paths_costed = compute_paths_costs(
        selected_paths,
        planner_cost,
        alpha=float(cfg.alpha),
        footprint_px=int(cfg.footprint_px),
    )
    t_cost_end = time.perf_counter()

    t_pick_start = time.perf_counter()
    final_num_samples = int(selected_paths[0].shape[0])
    final_rows, final_cols = pick_final_path(
        paths_costed,
        best_k=int(min(max(1, int(cfg.best_k)), len(paths_costed))),
        num_samples=final_num_samples,
        cost_map=planner_cost,
        alpha=float(cfg.alpha),
        footprint_px=int(cfg.footprint_px),
    )
    final_path_px = np.stack([np.asarray(final_rows), np.asarray(final_cols)], axis=1).astype(np.float32)
    final_path_bev = _planner_path_to_bev_path(final_path_px, src_shape=bev.shape, grid_size=int(cfg.grid_size))
    x_right, y_forward = bev_pixel_to_xy(final_path_bev[:, 0], final_path_bev[:, 1], bev.shape, float(bev_resolution_m))
    final_path_xy = np.stack([x_right, y_forward], axis=1).astype(np.float32)
    t_pick_end = time.perf_counter()

    total_plan_ms = (t_pick_end - t0) * 1000.0
    breakdown = {
        "filter_paths_ms": (t_filter_end - t_filter_start) * 1000.0,
        "adaptive_kmeans_ms": (t_kmeans_end - t_kmeans_start) * 1000.0,
        "compute_paths_costs_ms": (t_cost_end - t_cost_start) * 1000.0,
        "pick_final_path_ms": (t_pick_end - t_pick_start) * 1000.0,
        "total_plan_ms": total_plan_ms,
    }

    planned = PlannedPath(
        visualization=None,
        cost_map=planner_cost,
        known_mask=planner_known,
        final_path_pixels=final_path_px,
        final_path_xy_m=final_path_xy,
        candidate_paths=candidate_paths,
        filtered_paths=filtered_paths,
        selected_paths=selected_paths,
        metadata={
            "status": "ok",
            "candidate_paths": len(candidate_paths),
            "filtered_paths": len(filtered_paths),
            "selected_paths": len(selected_paths),
        },
    )
    return planned, breakdown


def run_benchmark():
    print("=" * 100)
    print("BRIEF 17: MEDICIÓN FORMAL DE LATENCIAS EN RTX 5060")
    print("=" * 100)

    # 1. Hardware and Environment info
    print(f"CUDA Available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"Device Name: {torch.cuda.get_device_name(0)}")
        print(f"Device Capability: {torch.cuda.get_device_capability(0)}")

    predictor = TraversabilityPredictor()
    print(f"TraversabilityPredictor Device: {predictor.device}")

    camera_k = load_camera_K().astype(np.float64)
    camera_t = load_T_base_camera().astype(np.float64)

    # Precomputar path bank
    grid_size = 240
    start_rc = (grid_size - 1, grid_size // 2)
    path_bank = sample_paths_polynomial(
        robot=start_rc,
        num_goals=30,
        num_mid_points_per_goal=20,
        num_samples=100,
        grid_size=grid_size,
        goal=None,
        include_random_goals=True,
        random_seed=42,
    )
    print(f"Precomputed Path Bank Size: {len(path_bank)} curves")

    cfg = PlannerConfig(
        grid_size=240,
        unknown_cost=0.2,
        smooth_kernel=3,
        footprint_px=10,
        threshold_cost=0.50,
        threshold_points_ratio=0.05,
        number_of_points_to_filter=60,
        alpha=1.0,
        best_k=12,
        use_clustering=True,
        max_clusters=4,
        cluster_angle_threshold_deg=40.0,
        random_seed=42,
    )

    scenarios = create_test_scenarios()
    num_warmup = 5
    num_measured = 35
    total_iters = num_warmup + num_measured

    results: Dict[str, Dict[str, Any]] = {}

    print(f"\nEjecutando {total_iters} iteraciones por escenario ({num_warmup} warmup + {num_measured} medidas)...")

    # Q.4: Medición de los 4 escenarios
    for name, img in scenarios.items():
        print(f"\n>>> Midiendo Escenario: {name}...")
        infer_times = []
        bev_times = []
        plan_times = []
        total_times = []
        alive_counts = []

        for i in range(total_iters):
            t_start = time.perf_counter()

            # 1. Inferencia NN
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            t_infer_0 = time.perf_counter()
            predict_res = predictor.predict(img)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            t_infer_1 = time.perf_counter()

            mask = predict_res.mask.astype(np.float32)

            # 2. Proyección BEV
            t_bev_0 = time.perf_counter()
            bev, obs, _ = project_score_to_bev(
                score_map=mask,
                camera_k=camera_k,
                camera_pose=camera_t,
                ground_z=0.0,
                bev_resolution_m_per_px=0.03,
                bev_forward_range_m=4.0,
                bev_side_range_m=2.0,
                max_ray_distance_m=6.0,
            )
            t_bev_1 = time.perf_counter()

            # 3. Plan GeNIE
            t_plan_0 = time.perf_counter()
            planned = plan_on_bev(
                bev_traversability=bev,
                observed_mask=obs,
                goal_x_m=0.0,
                goal_y_m=4.0,
                bev_resolution_m=0.03,
                config=cfg,
                candidate_path_bank=path_bank,
            )
            t_plan_1 = time.perf_counter()

            t_end = time.perf_counter()

            infer_ms = (t_infer_1 - t_infer_0) * 1000.0
            bev_ms = (t_bev_1 - t_bev_0) * 1000.0
            plan_ms = (t_plan_1 - t_plan_0) * 1000.0
            total_ms = (t_end - t_start) * 1000.0
            alive = len(planned.filtered_paths) if planned.filtered_paths is not None else 0

            # Descartar warmup
            if i >= num_warmup:
                infer_times.append(infer_ms)
                bev_times.append(bev_ms)
                plan_times.append(plan_ms)
                total_times.append(total_ms)
                alive_counts.append(alive)

        results[name] = {
            "alive": alive_counts[0] if len(alive_counts) > 0 else 0,
            "infer_nn": compute_stats(infer_times),
            "bev_proj": compute_stats(bev_times),
            "plan_genie": compute_stats(plan_times),
            "frame_total": compute_stats(total_times),
        }
        print(f"Completado {name}: alive={results[name]['alive']} | frame_total(mean)={results[name]['frame_total']['mean']:.2f}ms | frame_total(P95)={results[name]['frame_total']['p95']:.2f}ms")

    # Q.4.1: Desglose interno de plan_genie en escenario despejado
    print("\n>>> Midiendo desglose interno de plan_genie en Escenario 1 (Terreno Despejado)...")
    img_clear = scenarios["1. Terreno despejado"]
    res_clear = predictor.predict(img_clear)
    mask_clear = res_clear.mask.astype(np.float32)
    bev_clear, obs_clear, _ = project_score_to_bev(
        mask_clear, camera_k, camera_t, 0.0, 0.03, 4.0, 2.0, 6.0
    )

    filter_times = []
    kmeans_times = []
    cost_times = []
    pick_times = []
    plan_total_times = []

    for i in range(total_iters):
        _planned, breakdown = plan_on_bev_with_profiling(
            bev_traversability=bev_clear,
            observed_mask=obs_clear,
            goal_x_m=0.0,
            goal_y_m=4.0,
            bev_resolution_m=0.03,
            config=cfg,
            candidate_path_bank=path_bank,
        )
        if i >= num_warmup:
            filter_times.append(breakdown["filter_paths_ms"])
            kmeans_times.append(breakdown["adaptive_kmeans_ms"])
            cost_times.append(breakdown["compute_paths_costs_ms"])
            pick_times.append(breakdown["pick_final_path_ms"])
            plan_total_times.append(breakdown["total_plan_ms"])

    genie_breakdown = {
        "filter_paths_with_high_costs": compute_stats(filter_times),
        "adaptive_kmeans": compute_stats(kmeans_times),
        "compute_paths_costs": compute_stats(cost_times),
        "pick_final_path_and_conversions": compute_stats(pick_times),
        "total_plan_genie": compute_stats(plan_total_times),
    }

    # Imprimir tablas de resultados estructuradas
    print("\n" + "=" * 110)
    print("TABLA PRINCIPAL Q.4 — LATENCIAS POR ESCENARIO EN RTX 5060 (N=35 muestras, 5 warmup descartadas)")
    print("=" * 110)
    header = f"{'Escenario':<35} | {'Caminos':<8} | {'infer_nn (ms)':<18} | {'bev_proj (ms)':<18} | {'plan_genie (ms)':<18} | {'frame_total (ms)':<18}"
    print(header)
    print("-" * 110)

    for name, data in results.items():
        alive = data["alive"]
        inf = data["infer_nn"]
        bev = data["bev_proj"]
        pln = data["plan_genie"]
        tot = data["frame_total"]

        print(f"{name:<35} | {alive:<8} | mean: {inf['mean']:5.2f} (P95:{inf['p95']:5.2f}) | mean: {bev['mean']:5.2f} (P95:{bev['p95']:5.2f}) | mean: {pln['mean']:5.2f} (P95:{pln['p95']:5.2f}) | mean: {tot['mean']:5.2f} (P95:{tot['p95']:5.2f})")
        print(f"{'':<35} | {'':<8} | [min:{inf['min']:4.1f} max:{inf['max']:4.1f}]  | [min:{bev['min']:4.1f} max:{bev['max']:4.1f}]  | [min:{pln['min']:4.1f} max:{pln['max']:4.1f}]  | [min:{tot['min']:4.1f} max:{tot['max']:4.1f}]")
        print("-" * 110)

    print("\n" + "=" * 110)
    print("TABLA Q.4.1 — DESGLOSE INTERNO DE PLAN_GENIE (Terreno Despejado, 277 caminos vivos)")
    print("=" * 110)
    header_breakdown = f"{'Fase Interna GeNIE':<35} | {'Media (ms)':<12} | {'P95 (ms)':<12} | {'Min (ms)':<12} | {'Max (ms)':<12} | {'% de plan_genie':<15}"
    print(header_breakdown)
    print("-" * 110)
    tot_plan_mean = genie_breakdown["total_plan_genie"]["mean"]

    for stage, sdata in genie_breakdown.items():
        pct = (sdata["mean"] / tot_plan_mean) * 100.0 if tot_plan_mean > 0 else 0.0
        print(f"{stage:<35} | {sdata['mean']:10.2f}ms  | {sdata['p95']:10.2f}ms  | {sdata['min']:10.2f}ms  | {sdata['max']:10.2f}ms  | {pct:13.1f}%")
    print("=" * 110)

    return results, genie_breakdown


if __name__ == "__main__":
    run_benchmark()
