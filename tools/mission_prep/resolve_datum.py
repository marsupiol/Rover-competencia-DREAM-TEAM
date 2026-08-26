#!/usr/bin/env python3
"""
Resolve Datum Script for Earth Rover Missions.

Queries the SDK server (same HTTP endpoint and schema as mission_manager_node)
to extract the initial checkpoint (Checkpoint #1) coordinates, and writes a
deterministic datum configuration file for navsat_transform_node.

Rationale for Datum Yaw = 0.0:
  In the ROS REP-105 standard ENU (East-North-Up) convention, datum yaw is set
  to 0.0 to align the local 'map' frame X axis with East, Y axis with true North,
  and Z axis with Up. Aligning 'map' to true North ensures consistent Cartesian
  coordinates across runs and eliminates heading drift/offsets in offline maps.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import time
import requests


def fetch_checkpoints(sdk_url: str, retries: int = 5, timeout: float = 5.0) -> list[dict]:
    """
    Reuses the HTTP logic from mission_manager_node to retrieve checkpoints from the SDK server.
    Only queries GET /checkpoints-list without initiating or altering the mission state.
    """
    clean_url = sdk_url.rstrip("/")

    for attempt in range(1, retries + 1):
        try:
            print(f"[resolve_datum] Consultando {clean_url}/checkpoints-list (intento {attempt}/{retries})...")
            res = requests.get(f"{clean_url}/checkpoints-list", timeout=timeout)
            if res.status_code == 200:
                data = res.json()
                if isinstance(data, dict):
                    if isinstance(data.get("checkpoints_list"), dict):
                        nested = data["checkpoints_list"]
                        checkpoints = nested.get("checkpoints_list", [])
                    else:
                        checkpoints = data.get("checkpoints_list", [])
                elif isinstance(data, list):
                    checkpoints = data
                else:
                    checkpoints = []

                if checkpoints:
                    return checkpoints
                else:
                    print(f"[resolve_datum] Lista de checkpoints vacía recibida del SDK.")
            else:
                print(f"[resolve_datum] Error HTTP {res.status_code} al consultar checkpoints: {res.text}")
        except Exception as exc:
            print(f"[resolve_datum] Error de red consultando SDK: {exc}")

        if attempt < retries:
            time.sleep(1.5)

    raise RuntimeError(f"No se pudieron obtener los checkpoints del SDK en {clean_url} tras {retries} intentos.")


def write_datum_yaml(output_path: str, latitude: float, longitude: float, yaw: float = 0.0) -> None:
    """
    Writes the resolved datum to the specified YAML file.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    yaml_content = f"""# ==============================================================================
# Datum dinámico resuelto determinísticamente a partir del Checkpoint inicial (#1)
# Generado automáticamente por tools/mission_prep/resolve_datum.py
# ==============================================================================
navsat_transform:
  ros__parameters:
    datum: [{latitude:.8f}, {longitude:.8f}, {yaw:.1f}]
    wait_for_datum: false

navsat_transform_node:
  ros__parameters:
    datum: [{latitude:.8f}, {longitude:.8f}, {yaw:.1f}]
    wait_for_datum: false
"""
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"[resolve_datum] Datum escrito exitosamente en '{output_path}':")
    print(f"  Latitud:  {latitude:.8f}")
    print(f"  Longitud: {longitude:.8f}")
    print(f"  Yaw:      {yaw:.1f} rad (0.0 = alineado al Norte verdadero / REP-105 ENU)")

    # Sincronizar automáticamente con el directorio share instalado si existe
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("mini_plus_localization")
        install_target = os.path.join(share_dir, "config", "datum_resolved.yaml")
        if os.path.isdir(os.path.dirname(install_target)):
            shutil.copyfile(output_path, install_target)
            print(f"[resolve_datum] Datum sincronizado con install space: '{install_target}'")
    except Exception as exc:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description="Resuelve el datum geodésico determinista desde el SDK.")
    parser.add_argument(
        "--sdk-url",
        default=os.getenv("SDK_URL", "http://localhost:8000"),
        help="URL base del servidor SDK (default: http://localhost:8000)",
    )
    parser.add_argument(
        "--mission-slug",
        default=os.getenv("MISSION_SLUG", None),
        help="Slug de la misión (opcional)",
    )
    parser.add_argument(
        "--output",
        default="src/mini_plus_localization/config/datum_resolved.yaml",
        help="Ruta de salida del archivo YAML con el datum",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout de peticiones HTTP en segundos (default: 5.0)",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=5,
        help="Número de reintentos ante fallos de conexión (default: 5)",
    )

    args = parser.parse_args()

    try:
        checkpoints = fetch_checkpoints(
            sdk_url=args.sdk_url,
            retries=args.retries,
            timeout=args.timeout,
        )
    except Exception as exc:
        print(f"[resolve_datum] ERROR FATAL: {exc}", file=sys.stderr)
        return 1

    if not checkpoints:
        print("[resolve_datum] ERROR FATAL: Lista de checkpoints vacía.", file=sys.stderr)
        return 1

    cp0 = checkpoints[0]
    try:
        lat = float(cp0.get("latitude", cp0.get("lat")))
        lon = float(cp0.get("longitude", cp0.get("lon")))
    except (TypeError, ValueError, KeyError) as exc:
        print(f"[resolve_datum] ERROR FATAL: Checkpoint inicial inválido ({cp0}): {exc}", file=sys.stderr)
        return 1

    seq = cp0.get("sequence", 1)
    print(f"[resolve_datum] Checkpoint inicial resuelto (Secuencia {seq}): lat={lat:.8f}, lon={lon:.8f}")

    # Convención de yaw: 0.0 fijo (frame map alineado con Norte verdadero / REP-105)
    write_datum_yaml(args.output, latitude=lat, longitude=lon, yaw=0.0)
    return 0


if __name__ == "__main__":
    sys.exit(main())
