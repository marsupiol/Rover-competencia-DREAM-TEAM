#!/usr/bin/env python3
"""Unit tests for RollingWindowGrid."""

import numpy as np

from er_planning.rolling_window import RollingWindowGrid


def test_fixed_memory_after_recenter():
    grid = RollingWindowGrid(width_m=100.0, height_m=100.0, resolution_m_per_px=1.0)
    grid.confidence[50, 50] = 42.0

    assert grid.maybe_recenter(200.0, 200.0, threshold_m=10.0) is True
    assert grid.confidence.shape == (100, 100)
    assert grid.shift_count == 1

    row, col = grid.world_to_cell(200.0, 200.0)
    assert grid.in_bounds(row, col)
    assert grid.confidence[row, col] == 42.0

    old_row, old_col = grid.world_to_cell(150.0, 150.0)
    assert not grid.in_bounds(old_row, old_col) or grid.confidence[old_row, old_col] == 0.0


def test_data_preserved_in_overlap_after_shift():
    grid = RollingWindowGrid(width_m=40.0, height_m=40.0, resolution_m_per_px=1.0)
    wx, wy = grid.cell_to_world(20, 20)
    row, col = grid.world_to_cell(wx, wy)
    grid.confidence[row, col] = 99.0

    assert grid.maybe_recenter(15.0, 15.0, threshold_m=5.0) is True

    row_after, col_after = grid.world_to_cell(wx, wy)
    assert grid.confidence[row_after, col_after] == 99.0


def test_no_shift_within_threshold():
    grid = RollingWindowGrid(width_m=100.0, height_m=100.0, resolution_m_per_px=1.0)
    grid.confidence[10, 10] = 7.0
    initial_origin = (grid.origin_x, grid.origin_y)

    assert grid.maybe_recenter(5.0, 5.0, threshold_m=50.0) is False
    assert (grid.origin_x, grid.origin_y) == initial_origin
    assert grid.shift_count == 0
    assert grid.confidence[10, 10] == 7.0


def test_resample_from_seed_map():
    grid = RollingWindowGrid(width_m=20.0, height_m=20.0, resolution_m_per_px=1.0)
    seed = np.zeros((20, 20), dtype=np.float32)
    seed[5, 5] = -80.0

    grid.resample_from(seed, grid.origin_x, grid.origin_y, grid.resolution)
    assert grid.confidence[5, 5] == -80.0
