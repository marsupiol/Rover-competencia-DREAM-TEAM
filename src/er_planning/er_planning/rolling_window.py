"""
Rolling-window occupancy grid for persistent mapping at constant memory/compute.

Follows the Nav2 costmap rolling_window pattern: a fixed-size grid re-centers
around the rover and discards cells that fall behind the trailing edge.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class RollingWindowState:
    origin_x: float
    origin_y: float
    center_x: float
    center_y: float
    shift_count: int = 0


class RollingWindowGrid:
    """Fixed-size float32 confidence grid with optional re-centering."""

    def __init__(
        self,
        width_m: float,
        height_m: float,
        resolution_m_per_px: float,
        origin_x: float | None = None,
        origin_y: float | None = None,
    ) -> None:
        self.width_m = float(width_m)
        self.height_m = float(height_m)
        self.resolution = float(resolution_m_per_px)

        self.grid_w = max(1, int(round(self.width_m / self.resolution)))
        self.grid_h = max(1, int(round(self.height_m / self.resolution)))

        if origin_x is None:
            origin_x = -self.width_m / 2.0
        if origin_y is None:
            origin_y = -self.height_m / 2.0

        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.center_x = self.origin_x + self.width_m / 2.0
        self.center_y = self.origin_y + self.height_m / 2.0
        self.shift_count = 0

        self.confidence = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)

    @property
    def state(self) -> RollingWindowState:
        return RollingWindowState(
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            center_x=self.center_x,
            center_y=self.center_y,
            shift_count=self.shift_count,
        )

    def world_to_cell(self, wx: float, wy: float) -> tuple[int, int]:
        col = int(math.floor((wx - self.origin_x) / self.resolution))
        row = int(math.floor((wy - self.origin_y) / self.resolution))
        return row, col

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        wx = self.origin_x + (float(col) + 0.5) * self.resolution
        wy = self.origin_y + (float(row) + 0.5) * self.resolution
        return wx, wy

    def in_bounds(self, row: int, col: int) -> bool:
        return 0 <= row < self.grid_h and 0 <= col < self.grid_w

    def distance_from_center(self, wx: float, wy: float) -> float:
        return math.hypot(wx - self.center_x, wy - self.center_y)

    def set_center(self, wx: float, wy: float) -> None:
        """Initialize or force the window center without discarding data."""
        self.center_x = float(wx)
        self.center_y = float(wy)
        self.origin_x = self.center_x - self.width_m / 2.0
        self.origin_y = self.center_y - self.height_m / 2.0

    def maybe_recenter(self, rover_x: float, rover_y: float, threshold_m: float) -> bool:
        """
        Re-center the window around the rover when it drifts beyond threshold_m
        from the current center. Returns True if a shift occurred.
        """
        if self.distance_from_center(rover_x, rover_y) < threshold_m:
            return False
        self._recenter(rover_x, rover_y)
        return True

    def _recenter(self, rover_x: float, rover_y: float) -> None:
        old_origin_x = self.origin_x
        old_origin_y = self.origin_y
        old_conf = self.confidence

        new_center_x = float(rover_x)
        new_center_y = float(rover_y)
        new_origin_x = new_center_x - self.width_m / 2.0
        new_origin_y = new_center_y - self.height_m / 2.0

        cols = np.arange(self.grid_w, dtype=np.float64)
        rows = np.arange(self.grid_h, dtype=np.float64)
        col_grid, row_grid = np.meshgrid(cols, rows)

        wx = new_origin_x + (col_grid + 0.5) * self.resolution
        wy = new_origin_y + (row_grid + 0.5) * self.resolution

        old_cols = np.floor((wx - old_origin_x) / self.resolution).astype(np.int32)
        old_rows = np.floor((wy - old_origin_y) / self.resolution).astype(np.int32)

        new_conf = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        valid = (
            (old_cols >= 0)
            & (old_cols < self.grid_w)
            & (old_rows >= 0)
            & (old_rows < self.grid_h)
        )
        new_conf[valid] = old_conf[old_rows[valid], old_cols[valid]]

        self.confidence = new_conf
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y
        self.center_x = new_center_x
        self.center_y = new_center_y
        self.shift_count += 1

    def resample_from(
        self,
        source: np.ndarray,
        source_origin_x: float,
        source_origin_y: float,
        source_resolution: float,
    ) -> None:
        """Load confidence values from another grid aligned in world coordinates."""
        if source.shape != (self.grid_h, self.grid_w):
            raise ValueError(
                f"Source shape {source.shape} does not match window {(self.grid_h, self.grid_w)}"
            )
        if not math.isclose(source_resolution, self.resolution, rel_tol=0.0, abs_tol=1e-6):
            raise ValueError("Source resolution must match window resolution for resample_from")

        cols = np.arange(self.grid_w, dtype=np.float64)
        rows = np.arange(self.grid_h, dtype=np.float64)
        col_grid, row_grid = np.meshgrid(cols, rows)

        wx = self.origin_x + (col_grid + 0.5) * self.resolution
        wy = self.origin_y + (row_grid + 0.5) * self.resolution

        src_cols = np.floor((wx - source_origin_x) / source_resolution).astype(np.int32)
        src_rows = np.floor((wy - source_origin_y) / source_resolution).astype(np.int32)

        new_conf = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        valid = (
            (src_cols >= 0)
            & (src_cols < self.grid_w)
            & (src_rows >= 0)
            & (src_rows < self.grid_h)
        )
        new_conf[valid] = source[src_rows[valid], src_cols[valid]]
        self.confidence = new_conf
