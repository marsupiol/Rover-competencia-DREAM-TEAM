#!/usr/bin/env python3
"""
Road Router Node — Level 1 hierarchical routing for long-distance missions.

Computes street-level routes via OSRM (OpenStreetMap road graph) and publishes
intermediate geodetic waypoints for the rolling-window D* Lite planner (Level 2)
and the local BEV planner (Level 3).

For short legs (below long_route_threshold_m) the mission checkpoint is forwarded
unchanged (fail-open passthrough).
"""

from __future__ import annotations

import math
import threading
from typing import Any

import rclpy
import requests
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    earth_r = 6371000.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * math.sin(dlon / 2.0) ** 2
    )
    return earth_r * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def _interpolate_route_points(
    coords: list[tuple[float, float]],
    spacing_m: float,
) -> list[tuple[float, float]]:
    """Sample (lat, lon) pairs along a polyline at roughly fixed geodesic spacing."""
    if len(coords) < 2:
        return list(coords)

    spacing_m = max(1.0, spacing_m)
    samples: list[tuple[float, float]] = [coords[0]]
    carry = 0.0

    for idx in range(len(coords) - 1):
        lat_a, lon_a = coords[idx]
        lat_b, lon_b = coords[idx + 1]
        seg_len = _haversine_m(lat_a, lon_a, lat_b, lon_b)
        if seg_len <= 1e-3:
            continue

        dist = spacing_m - carry
        while dist <= seg_len:
            t = dist / seg_len
            lat = lat_a + t * (lat_b - lat_a)
            lon = lon_a + t * (lon_b - lon_a)
            samples.append((lat, lon))
            dist += spacing_m
        carry = max(0.0, spacing_m - (seg_len - (dist - spacing_m)))

    if samples[-1] != coords[-1]:
        samples.append(coords[-1])
    return samples


class RoadRouterNode(Node):
    def __init__(self) -> None:
        super().__init__("road_router_node")

        self.declare_parameter("mission_target_topic", "earth_rover/target_waypoint")
        self.declare_parameter("routing_waypoint_topic", "earth_rover/routing_waypoint")
        self.declare_parameter("routing_active_topic", "earth_rover/routing_active")
        self.declare_parameter("gps_topic", "gps/filtered")
        self.declare_parameter("osrm_base_url", "https://router.project-osrm.org")
        self.declare_parameter("osrm_profile", "foot")
        self.declare_parameter("long_route_threshold_m", 500.0)
        self.declare_parameter("segment_spacing_m", 150.0)
        self.declare_parameter("advance_radius_m", 40.0)
        self.declare_parameter("route_refresh_period_s", 2.0)
        self.declare_parameter("http_timeout_s", 15.0)

        mission_target_topic = str(self.get_parameter("mission_target_topic").value)
        routing_waypoint_topic = str(self.get_parameter("routing_waypoint_topic").value)
        routing_active_topic = str(self.get_parameter("routing_active_topic").value)
        gps_topic = str(self.get_parameter("gps_topic").value)

        self.osrm_base_url = str(self.get_parameter("osrm_base_url").value).rstrip("/")
        self.osrm_profile = str(self.get_parameter("osrm_profile").value)
        self.long_route_threshold_m = float(self.get_parameter("long_route_threshold_m").value)
        self.segment_spacing_m = float(self.get_parameter("segment_spacing_m").value)
        self.advance_radius_m = float(self.get_parameter("advance_radius_m").value)
        self.http_timeout_s = float(self.get_parameter("http_timeout_s").value)
        route_refresh_period_s = float(self.get_parameter("route_refresh_period_s").value)

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

        self._lock = threading.Lock()
        self._current_lat: float | None = None
        self._current_lon: float | None = None
        self._mission_lat: float | None = None
        self._mission_lon: float | None = None
        self._route_points: list[tuple[float, float]] = []
        self._route_idx = 0
        self._routing_active = False

        self.create_subscription(NavSatFix, mission_target_topic, self._on_mission_target, reliable_qos)
        self.create_subscription(NavSatFix, gps_topic, self._on_gps, sensor_qos)

        self.routing_pub = self.create_publisher(NavSatFix, routing_waypoint_topic, reliable_qos)
        self.active_pub = self.create_publisher(Bool, routing_active_topic, reliable_qos)

        self.create_timer(route_refresh_period_s, self._route_timer_cb)

        self.get_logger().info(
            f"RoadRouterNode inicializado | OSRM={self.osrm_base_url} | "
            f"long_route>={self.long_route_threshold_m:.0f}m | spacing={self.segment_spacing_m:.0f}m"
        )

    def _on_gps(self, msg: NavSatFix) -> None:
        with self._lock:
            self._current_lat = float(msg.latitude)
            self._current_lon = float(msg.longitude)

    def _on_mission_target(self, msg: NavSatFix) -> None:
        lat = float(msg.latitude)
        lon = float(msg.longitude)

        with self._lock:
            if (
                self._mission_lat is not None
                and self._mission_lon is not None
                and math.isclose(lat, self._mission_lat, abs_tol=1e-7)
                and math.isclose(lon, self._mission_lon, abs_tol=1e-7)
            ):
                return
            self._mission_lat = lat
            self._mission_lon = lon

        self._plan_route_for_mission(lat, lon)

    def _plan_route_for_mission(self, mission_lat: float, mission_lon: float) -> None:
        with self._lock:
            cur_lat = self._current_lat
            cur_lon = self._current_lon

        if cur_lat is None or cur_lon is None:
            self.get_logger().info(
                "Esperando GPS antes de calcular ruta vial...",
                throttle_duration_sec=5.0,
            )
            self._publish_passthrough(mission_lat, mission_lon)
            return

        leg_dist = _haversine_m(cur_lat, cur_lon, mission_lat, mission_lon)
        if leg_dist <= self.long_route_threshold_m:
            self.get_logger().info(
                f"Tramo corto ({leg_dist:.0f}m <= {self.long_route_threshold_m:.0f}m): "
                "passthrough directo al checkpoint."
            )
            with self._lock:
                self._route_points = [(mission_lat, mission_lon)]
                self._route_idx = 0
                self._routing_active = False
            self._publish_passthrough(mission_lat, mission_lon)
            return

        route_coords = self._fetch_osrm_route(cur_lat, cur_lon, mission_lat, mission_lon)
        if not route_coords:
            self.get_logger().warn(
                "Fallo en ruteo OSRM — fail-open al checkpoint directo.",
                throttle_duration_sec=5.0,
            )
            self._publish_passthrough(mission_lat, mission_lon)
            return

        sampled = _interpolate_route_points(route_coords, self.segment_spacing_m)
        with self._lock:
            self._route_points = sampled
            self._route_idx = 0
            self._routing_active = len(sampled) > 1

        self.get_logger().info(
            f"Ruta OSRM: {len(route_coords)} vértices -> {len(sampled)} waypoints intermedios "
            f"({leg_dist / 1000.0:.1f} km total)"
        )
        self._publish_current_routing_waypoint()

    def _fetch_osrm_route(
        self,
        start_lat: float,
        start_lon: float,
        end_lat: float,
        end_lon: float,
    ) -> list[tuple[float, float]] | None:
        # OSRM expects lon,lat ordering
        url = (
            f"{self.osrm_base_url}/route/v1/{self.osrm_profile}/"
            f"{start_lon:.7f},{start_lat:.7f};{end_lon:.7f},{end_lat:.7f}"
        )
        params = {"overview": "full", "geometries": "geojson", "steps": "false"}

        try:
            response = requests.get(url, params=params, timeout=self.http_timeout_s)
            response.raise_for_status()
            payload: dict[str, Any] = response.json()
        except Exception as exc:
            self.get_logger().error(f"Error HTTP OSRM: {exc}")
            return None

        if payload.get("code") != "Ok":
            self.get_logger().error(f"OSRM rechazó la ruta: {payload.get('message', payload)}")
            return None

        routes = payload.get("routes") or []
        if not routes:
            return None

        geometry = routes[0].get("geometry") or {}
        raw_coords = geometry.get("coordinates") or []
        if not raw_coords:
            return None

        # GeoJSON coordinates are [lon, lat]
        return [(float(lat), float(lon)) for lon, lat in raw_coords]

    def _publish_passthrough(self, lat: float, lon: float) -> None:
        with self._lock:
            self._routing_active = False
        self._publish_waypoint(lat, lon, active=False)

    def _publish_current_routing_waypoint(self) -> None:
        with self._lock:
            if not self._route_points:
                return
            idx = min(self._route_idx, len(self._route_points) - 1)
            lat, lon = self._route_points[idx]
            active = self._routing_active
        self._publish_waypoint(lat, lon, active=active)

    def _publish_waypoint(self, lat: float, lon: float, active: bool) -> None:
        msg = NavSatFix()
        msg.latitude = lat
        msg.longitude = lon
        self.routing_pub.publish(msg)

        active_msg = Bool()
        active_msg.data = bool(active)
        self.active_pub.publish(active_msg)

        self.get_logger().info(
            f"Routing waypoint publicado: ({lat:.6f}, {lon:.6f}) active={active}"
        )

    def _route_timer_cb(self) -> None:
        with self._lock:
            cur_lat = self._current_lat
            cur_lon = self._current_lon
            mission_lat = self._mission_lat
            mission_lon = self._mission_lon
            route_points = list(self._route_points)
            route_idx = self._route_idx
            routing_active = self._routing_active

        if cur_lat is None or cur_lon is None or mission_lat is None or mission_lon is None:
            return

        if not routing_active or not route_points:
            return

        if route_idx >= len(route_points) - 1:
            self._publish_passthrough(mission_lat, mission_lon)
            return

        wp_lat, wp_lon = route_points[route_idx]
        dist = _haversine_m(cur_lat, cur_lon, wp_lat, wp_lon)
        if dist > self.advance_radius_m:
            self._publish_waypoint(wp_lat, wp_lon, active=True)
            return

        with self._lock:
            if self._route_idx < len(self._route_points) - 1:
                self._route_idx += 1
                next_lat, next_lon = self._route_points[self._route_idx]
            else:
                next_lat, next_lon = mission_lat, mission_lon

        self.get_logger().info(
            f"Avanzando a waypoint intermedio {self._route_idx + 1}/{len(route_points)} "
            f"(dist={dist:.1f}m)"
        )
        self._publish_waypoint(next_lat, next_lon, active=True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadRouterNode()
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
