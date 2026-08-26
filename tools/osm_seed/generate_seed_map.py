#!/usr/bin/env python3
"""
Generate Seed Map from OpenStreetMap (OSM) for PersistentMapNode.

Fetches real-world street and sidewalk geometries from OpenStreetMap within a
bounding box or radius around the competition datum, converts coordinates to
the 'map' frame using navsat_transform_node's /fromLL service, rasterizes them
onto the persistent map grid, and outputs an unscaled .npy confidence prior.

Vias Classification:
  - "vereda-like" (prefer / drivable): footway, path, pedestrian, sidewalk, steps -> -80.0
  - "calle-like" (avoid but don't block): residential, service, tertiary, secondary,
    primary, unclassified (masked to not overwrite parallel sidewalks) -> +60.0
  - others: 0.0 (neutral/unknown)

Caching:
  Raw OSM Overpass JSON queries are cached in tools/osm_seed/cache/ keyed by
  bounding box hash to ensure fast and reliable mission preparation without
  relying on external network availability for every run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from typing import Any

import numpy as np
import requests
import scipy.ndimage
import yaml

# ROS 2 imports
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from robot_localization.srv import FromLL


VEREDA_TAGS = {"footway", "path", "pedestrian", "sidewalk", "steps"}
CALLE_TAGS = {"residential", "service", "tertiary", "secondary", "primary", "unclassified"}

OVERPASS_SERVERS = [
    "https://overpass-api.de/api/interpreter",
    "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]


def parse_datum_yaml(datum_file: str) -> tuple[float, float, float]:
    """
    Parses latitude, longitude, and yaw from datum_resolved.yaml.
    """
    if not os.path.isfile(datum_file):
        raise FileNotFoundError(f"Datum file not found: {datum_file}")

    with open(datum_file, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    # Search for datum parameter under navsat_transform or navsat_transform_node
    datum = None
    for top_key in ("navsat_transform", "navsat_transform_node", "/**"):
        if isinstance(data, dict) and top_key in data:
            params = data[top_key].get("ros__parameters", {})
            if "datum" in params:
                datum = params["datum"]
                break

    if not datum or len(datum) < 2:
        raise ValueError(f"Could not find valid [lat, lon, yaw] datum in {datum_file}")

    lat = float(datum[0])
    lon = float(datum[1])
    yaw = float(datum[2]) if len(datum) > 2 else 0.0
    return lat, lon, yaw


def parse_grid_config(config_file: str) -> dict[str, float]:
    """
    Parses grid dimensions, resolution, and origins from persistent_map_params.yaml.
    """
    defaults = {
        "map_width_m": 400.0,
        "map_height_m": 400.0,
        "map_resolution_m_per_px": 0.20,
        "map_origin_x_m": -200.0,
        "map_origin_y_m": -200.0,
    }

    if os.path.isfile(config_file):
        with open(config_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if isinstance(data, dict):
            params = data.get("persistent_map_node", {}).get("ros__parameters", {})
            for k in defaults:
                if k in params:
                    defaults[k] = float(params[k])
    return defaults


def fetch_osm_data(
    south: float,
    west: float,
    north: float,
    east: float,
    cache_dir: str,
    force_download: bool = False,
    timeout: float = 30.0,
) -> dict[str, Any]:
    """
    Downloads raw OSM data via Overpass API for the given bounding box with local caching.
    """
    os.makedirs(cache_dir, exist_ok=True)
    bbox_key = f"{south:.5f}_{west:.5f}_{north:.5f}_{east:.5f}"
    bbox_hash = hashlib.sha256(bbox_key.encode("utf-8")).hexdigest()[:16]
    cache_file = os.path.join(cache_dir, f"osm_bbox_{bbox_hash}.json")

    if not force_download and os.path.isfile(cache_file):
        print(f"[generate_seed_map] Reusando datos OSM en caché: '{cache_file}'")
        try:
            with open(cache_file, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            print(f"[generate_seed_map] Error leyendo caché ({exc}), procediendo a descargar...")

    overpass_query = f"""
    [out:json][timeout:{int(timeout)}];
    (
      way["highway"]({south:.6f},{west:.6f},{north:.6f},{east:.6f});
    );
    out body;
    >;
    out skel qt;
    """

    print(f"[generate_seed_map] Consultando Overpass API para bbox [{south:.5f}, {west:.5f}, {north:.5f}, {east:.5f}]...")
    last_error = None

    for server_url in OVERPASS_SERVERS:
        try:
            print(f"[generate_seed_map] Intentando servidor: {server_url} ...")
            response = requests.post(
                server_url,
                data={"data": overpass_query},
                timeout=timeout,
            )
            if response.status_code == 200:
                osm_json = response.json()
                with open(cache_file, "w", encoding="utf-8") as f:
                    json.dump(osm_json, f)
                n_elements = len(osm_json.get("elements", []))
                print(f"[generate_seed_map] Descarga exitosa ({n_elements} elementos). Guardado en caché: '{cache_file}'")
                return osm_json
            else:
                print(f"[generate_seed_map] Servidor {server_url} respondió HTTP {response.status_code}")
                last_error = f"HTTP {response.status_code}"
        except Exception as exc:
            print(f"[generate_seed_map] Error consultando {server_url}: {exc}")
            last_error = str(exc)

    # Fallback to existing cache if download failed
    if os.path.isfile(cache_file):
        print(f"[generate_seed_map] WARNING: Descarga falló ({last_error}), pero existe caché previo. Reusando caché '{cache_file}'.")
        with open(cache_file, "r", encoding="utf-8") as f:
            return json.load(f)

    raise RuntimeError(f"Fallo total descargando datos de OpenStreetMap: {last_error}")


def convert_coords_with_fromll(
    nodes_latlon: dict[int, tuple[float, float]],
    service_name: str = "/fromLL",
    timeout_s: float = 10.0,
) -> dict[int, tuple[float, float]]:
    """
    Converts (lat, lon) nodes to local 'map' Cartesian coordinates using navsat_transform_node's /fromLL.
    """
    if not nodes_latlon:
        return {}

    rclpy.init()
    node = Node("osm_seed_converter")

    try:
        client = node.create_client(FromLL, service_name)
        print(f"[generate_seed_map] Esperando servicio '{service_name}' de navsat_transform_node (timeout {timeout_s}s)...")
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError(
                f"El servicio '{service_name}' no está disponible. "
                "Asegurate de que navsat_transform_node esté corriendo con el datum cargado."
            )

        print(f"[generate_seed_map] Convirtiendo {len(nodes_latlon)} vértices OSM a coordenadas del frame 'map'...")
        results: dict[int, tuple[float, float]] = {}

        for node_id, (lat, lon) in nodes_latlon.items():
            req = FromLL.Request()
            req.ll_point = GeoPoint()
            req.ll_point.latitude = float(lat)
            req.ll_point.longitude = float(lon)
            req.ll_point.altitude = 0.0

            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

            if future.done() and future.result() is not None:
                resp = future.result()
                results[node_id] = (resp.map_point.x, resp.map_point.y)
            else:
                print(f"[generate_seed_map] Advertencia: Llamada a /fromLL falló para nodo {node_id} ({lat}, {lon})")

        return results
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def rasterize_line(r0: int, c0: int, r1: int, c1: int, grid: np.ndarray) -> None:
    """
    Draws a 1-pixel Bresenham/linear line between (r0, c0) and (r1, c1) onto grid in-place.
    """
    h, w = grid.shape
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    num_pts = max(dr, dc) + 1

    if num_pts <= 1:
        if 0 <= r0 < h and 0 <= c0 < w:
            grid[r0, c0] = True
        return

    r_pts = np.round(np.linspace(r0, r1, num_pts)).astype(np.int32)
    c_pts = np.round(np.linspace(c0, c1, num_pts)).astype(np.int32)

    valid = (r_pts >= 0) & (r_pts < h) & (c_pts >= 0) & (c_pts < w)
    grid[r_pts[valid], c_pts[valid]] = True


def make_circular_disk(radius_cells: int) -> np.ndarray:
    """
    Creates a 2D boolean circular disk structuring element of given radius in cells.
    """
    y, x = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
    return (x * x + y * y) <= (radius_cells * radius_cells)


def generate_seed_array(
    osm_data: dict[str, Any],
    node_map_coords: dict[int, tuple[float, float]],
    grid_w: int,
    grid_h: int,
    map_res: float,
    origin_x: float,
    origin_y: float,
    sidewalk_width_m: float = 1.5,
    street_width_m: float = 6.0,
) -> np.ndarray:
    """
    Rasterizes and inflates OSM ways into a grid_h x grid_w confidence prior array.
    """
    vereda_grid = np.zeros((grid_h, grid_w), dtype=bool)
    calle_grid = np.zeros((grid_h, grid_w), dtype=bool)

    elements = osm_data.get("elements", [])
    ways = [el for el in elements if el.get("type") == "way"]

    count_vereda_ways = 0
    count_calle_ways = 0

    for way in ways:
        tags = way.get("tags", {})
        highway = tags.get("highway", "")
        node_ids = way.get("nodes", [])

        is_vereda = highway in VEREDA_TAGS
        is_calle = highway in CALLE_TAGS

        if not (is_vereda or is_calle):
            continue

        target_grid = vereda_grid if is_vereda else calle_grid
        if is_vereda:
            count_vereda_ways += 1
        else:
            count_calle_ways += 1

        for i in range(len(node_ids) - 1):
            n0 = node_ids[i]
            n1 = node_ids[i + 1]
            if n0 in node_map_coords and n1 in node_map_coords:
                x0, y0 = node_map_coords[n0]
                x1, y1 = node_map_coords[n1]

                c0 = int(math.floor((x0 - origin_x) / map_res))
                r0 = int(math.floor((y0 - origin_y) / map_res))
                c1 = int(math.floor((x1 - origin_x) / map_res))
                r1 = int(math.floor((y1 - origin_y) / map_res))

                rasterize_line(r0, c0, r1, c1, target_grid)

    print(f"[generate_seed_map] Rasterizadas {count_vereda_ways} vías vereda-like y {count_calle_ways} vías calle-like.")

    # --------------------------------------------------------------------------
    # Inflación por Dilatación Binaria (scipy.ndimage.binary_dilation)
    # --------------------------------------------------------------------------
    # Ejemplo numérico con resolución 0.20m/px:
    #   - Vereda (1.5m de ancho -> semiancho 0.75m):
    #       sidewalk_radius_cells = max(1, round(0.75 / 0.20)) = 4 celdas.
    #       Estructura disk(4): ancho total = (2 * 4 + 1) * 0.20m = 1.8m (franja de vereda).
    #   - Calle (6.0m de ancho -> semiancho 3.0m):
    #       street_radius_cells = max(1, round(3.0 / 0.20)) = 15 celdas.
    #       Estructura disk(15): ancho total = (2 * 15 + 1) * 0.20m = 6.2m (franja de calle).
    # --------------------------------------------------------------------------
    sidewalk_radius_cells = max(1, int(round((sidewalk_width_m / 2.0) / map_res)))
    street_radius_cells = max(1, int(round((street_width_m / 2.0) / map_res)))

    print(f"[generate_seed_map] Dilatando veredas (radio {sidewalk_radius_cells} px = {sidewalk_radius_cells * map_res:.2f}m)...")
    vereda_structure = make_circular_disk(sidewalk_radius_cells)
    vereda_dilated = scipy.ndimage.binary_dilation(vereda_grid, structure=vereda_structure)

    print(f"[generate_seed_map] Dilatando calles (radio {street_radius_cells} px = {street_radius_cells * map_res:.2f}m)...")
    calle_structure = make_circular_disk(street_radius_cells)
    calle_dilated = scipy.ndimage.binary_dilation(calle_grid, structure=calle_structure)

    # --------------------------------------------------------------------------
    # Ensamblado de Matriz de Confianza Prior (Derivación hacia atrás desde D* Lite)
    # --------------------------------------------------------------------------
    # Cadena Matemática Inversa:
    # 1. Vereda (Costo D* Lite objetivo = 1.0):
    #    - Queremos cost = 1.0 -> norm = 0.0 -> raw_data <= 40 (free_ref_value = 40).
    #    - En persistent_map: scaled <= 42.5 cuantiza a <= 40 (ej. scaled = 38.0).
    #    - confidence = (38.0 * 2.0) - 100.0 = -24.0 (<= free_threshold -20.0).
    #    - raw_seed_vereda = confidence / seed_confidence_scale = -24.0 / 0.3 = -80.0.
    #
    # 2. Calle (Costo D* Lite objetivo ≈ 2.84, apenas por encima de desconocido 2.5):
    #    - Queremos cost = 2.84 (ligeramente superior a unknown_cell_cost = 2.50).
    #    - norm = (2.84 - 1.0) / 14.0 = 0.13158 -> raw_data = 40 + (0.13158 * 38) = 45.0.
    #    - En persistent_map: scaled = 47.0 cuantiza a raw_data = 45.
    #    - confidence = (47.0 * 2.0) - 100.0 = -6.0 (fuera de banda desconocida [-5, 5]).
    #    - raw_seed_calle = confidence / seed_confidence_scale = -6.0 / 0.3 = -20.0.
    # --------------------------------------------------------------------------
    seed_array = np.zeros((grid_h, grid_w), dtype=np.float32)

    # Calles: aplicar sólo donde NO haya vereda cercana (costo D* Lite objetivo = 2.84)
    calle_effective = calle_dilated & (~vereda_dilated)
    seed_array[calle_effective] = -20.0

    # Veredas: prioridad absoluta sobre la calle (costo D* Lite objetivo = 1.00)
    seed_array[vereda_dilated] = -80.0

    n_vereda = int(np.count_nonzero(vereda_dilated))
    n_calle = int(np.count_nonzero(calle_effective))
    print(f"[generate_seed_map] Grilla de prior generada: {n_vereda} celdas vereda (-80.0 -> D* cost 1.00), {n_calle} celdas calle (-20.0 -> D* cost 2.84).")

    return seed_array


def main() -> int:
    parser = argparse.ArgumentParser(description="Genera el mapa semilla OSM para persistent_map_node.")
    parser.add_argument(
        "--datum-file",
        default="src/mini_plus_localization/config/datum_resolved.yaml",
        help="Ruta al archivo datum_resolved.yaml",
    )
    parser.add_argument(
        "--config",
        default="src/er_planning/config/persistent_map_params.yaml",
        help="Ruta al archivo persistent_map_params.yaml",
    )
    parser.add_argument(
        "--output",
        default="tools/osm_seed/seed_map.npy",
        help="Ruta de salida para el archivo .npy generado",
    )
    parser.add_argument(
        "--cache-dir",
        default="tools/osm_seed/cache",
        help="Directorio de caché para descargas crudas de OSM",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=None,
        help="Radio de consulta en metros alrededor del datum (default: diagonal del mapa + margen)",
    )
    parser.add_argument(
        "--bbox",
        nargs=4,
        type=float,
        metavar=("SOUTH", "WEST", "NORTH", "EAST"),
        help="Bounding box manual (south west north east en grados)",
    )
    parser.add_argument(
        "--sidewalk-width",
        type=float,
        default=1.5,
        help="Ancho nominal de vereda en metros (default: 1.5)",
    )
    parser.add_argument(
        "--street-width",
        type=float,
        default=6.0,
        help="Ancho nominal de calle en metros (default: 6.0)",
    )
    parser.add_argument(
        "--service-name",
        default="/fromLL",
        help="Nombre del servicio de navsat_transform_node (default: /fromLL)",
    )
    parser.add_argument(
        "--service-timeout",
        type=float,
        default=10.0,
        help="Timeout para esperar el servicio /fromLL en segundos (default: 10.0)",
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        help="Fuerza la descarga ignorando el caché local de OSM",
    )

    args = parser.parse_args()

    # 1. Cargar parámetros de la grilla
    grid_cfg = parse_grid_config(args.config)
    map_width = grid_cfg["map_width_m"]
    map_height = grid_cfg["map_height_m"]
    map_res = grid_cfg["map_resolution_m_per_px"]
    origin_x = grid_cfg["map_origin_x_m"]
    origin_y = grid_cfg["map_origin_y_m"]

    grid_w = max(1, int(round(map_width / map_res)))
    grid_h = max(1, int(round(map_height / map_res)))
    print(f"[generate_seed_map] Configuración de grilla: {grid_w}x{grid_h} celdas ({map_width}x{map_height}m @ {map_res}m/px)")

    # 2. Cargar datum
    try:
        datum_lat, datum_lon, datum_yaw = parse_datum_yaml(args.datum_file)
        print(f"[generate_seed_map] Datum cargado: lat={datum_lat:.8f}, lon={datum_lon:.8f}, yaw={datum_yaw:.1f}")
    except Exception as exc:
        print(f"[generate_seed_map] ERROR FATAL al leer datum: {exc}", file=sys.stderr)
        return 1

    # 3. Calcular Bounding Box
    if args.bbox:
        south, west, north, east = args.bbox
    else:
        # Calcular radio si no fue provisto: diagonal del mapa / 2 + 50m de buffer
        if args.radius is None:
            radius_m = (math.sqrt(map_width**2 + map_height**2) / 2.0) + 50.0
        else:
            radius_m = float(args.radius)

        dlat = radius_m / 111320.0
        dlon = radius_m / (111320.0 * math.cos(math.radians(datum_lat)))
        south = datum_lat - dlat
        north = datum_lat + dlat
        west = datum_lon - dlon
        east = datum_lon + dlon

    print(f"[generate_seed_map] Bounding Box de búsqueda: S={south:.5f}, W={west:.5f}, N={north:.5f}, E={east:.5f}")

    # 4. Obtener datos crudos de OSM (con caché)
    try:
        osm_data = fetch_osm_data(
            south=south,
            west=west,
            north=north,
            east=east,
            cache_dir=args.cache_dir,
            force_download=args.force_download,
        )
    except Exception as exc:
        print(f"[generate_seed_map] ERROR FATAL obteniendo OSM: {exc}", file=sys.stderr)
        return 1

    # 5. Extraer vértices requeridos
    elements = osm_data.get("elements", [])
    nodes_dict: dict[int, tuple[float, float]] = {}
    for el in elements:
        if el.get("type") == "node":
            nodes_dict[el["id"]] = (float(el["lat"]), float(el["lon"]))

    # Identificar sólo los nodos referenciados por vías clasificadas
    relevant_node_ids: set[int] = set()
    for el in elements:
        if el.get("type") == "way":
            tags = el.get("tags", {})
            hway = tags.get("highway", "")
            if hway in VEREDA_TAGS or hway in CALLE_TAGS:
                for nid in el.get("nodes", []):
                    if nid in nodes_dict:
                        relevant_node_ids.add(nid)

    relevant_nodes = {nid: nodes_dict[nid] for nid in relevant_node_ids}
    print(f"[generate_seed_map] {len(relevant_nodes)} nodos únicos identificados en vías de interés.")

    if not relevant_nodes:
        print("[generate_seed_map] Advertencia: No se encontraron vías de vereda/calle en esta zona OSM.")
        empty_seed = np.zeros((grid_h, grid_w), dtype=np.float32)
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        np.save(args.output, empty_seed)
        print(f"[generate_seed_map] Archivo semilla neutral guardado en '{args.output}'.")
        return 0

    # 6. Convertir coordenadas geodésicas vía /fromLL
    try:
        node_map_coords = convert_coords_with_fromll(
            relevant_nodes,
            service_name=args.service_name,
            timeout_s=args.service_timeout,
        )
    except Exception as exc:
        print(f"[generate_seed_map] ERROR FATAL en conversión /fromLL: {exc}", file=sys.stderr)
        return 1

    # 7. Rasterizar y generar mapa semilla
    seed_array = generate_seed_array(
        osm_data=osm_data,
        node_map_coords=node_map_coords,
        grid_w=grid_w,
        grid_h=grid_h,
        map_res=map_res,
        origin_x=origin_x,
        origin_y=origin_y,
        sidewalk_width_m=args.sidewalk_width,
        street_width_m=args.street_width,
    )

    # 8. Guardar .npy
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    np.save(args.output, seed_array)
    print(f"[generate_seed_map] ¡Mapa semilla generado con éxito y guardado en '{args.output}'! (shape={seed_array.shape})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
