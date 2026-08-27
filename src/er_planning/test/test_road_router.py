#!/usr/bin/env python3
"""Unit tests for road routing helpers."""

from er_planning.road_router_node import _haversine_m, _interpolate_route_points


def test_haversine_zero_distance():
    assert _haversine_m(-34.0, -58.0, -34.0, -58.0) == 0.0


def test_interpolate_route_spacing():
    coords = [(-34.0, -58.0), (-34.01, -58.0)]
    samples = _interpolate_route_points(coords, spacing_m=400.0)
    assert len(samples) >= 3
    assert samples[0] == coords[0]
    assert samples[-1] == coords[-1]


def test_interpolate_short_route():
    coords = [(-34.0, -58.0), (-34.0001, -58.0)]
    samples = _interpolate_route_points(coords, spacing_m=500.0)
    assert samples == coords
