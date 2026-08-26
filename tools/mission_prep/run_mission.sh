#!/usr/bin/env bash
# ==============================================================================
# Earth Rover - Mission Preparation & Orchestrated Launcher
# ==============================================================================
# Secuencia:
#   1. Resuelve datum dinámico desde el Checkpoint #1 del SDK (resolve_datum.py).
#   2. Levanta navsat_transform_node temporal con el datum resuelto.
#   3. Genera mapa semilla OSM llamando a /fromLL (generate_seed_map.py).
#   4. Detiene el nodo temporal SIEMPRE (garantizado con trap bash).
#   5. Lanza la misión completa en ROS 2 (mission1.launch.py).
#
# Uso:
#   ./tools/mission_prep/run_mission.sh --mission-slug <slug>
#   ./tools/mission_prep/run_mission.sh --mission-slug <slug> --skip-seed-map
#   ./tools/mission_prep/run_mission.sh --mission-slug <slug> enable_global_planning:=false
# ==============================================================================

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

# Cargar entorno de ROS 2 y workspace si está disponible
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "/opt/ros/jazzy/setup.bash"
fi
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
    # shellcheck source=/dev/null
    source "${WORKSPACE_ROOT}/install/setup.bash"
fi

MISSION_SLUG=""
SKIP_SEED_MAP=false
FORCE_DOWNLOAD=false
SDK_URL="http://localhost:8000"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mission-slug)
            MISSION_SLUG="$2"
            shift 2
            ;;
        --skip-seed-map)
            SKIP_SEED_MAP=true
            shift
            ;;
        --force-download)
            FORCE_DOWNLOAD=true
            shift
            ;;
        --sdk-url)
            SDK_URL="$2"
            shift 2
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ -n "$MISSION_SLUG" ]; then
    export MISSION_SLUG="$MISSION_SLUG"
fi

# Variables de control del nodo temporal
TEMP_NODE_PID=""

cleanup_temp_node() {
    local exit_code=$?
    if [ -n "$TEMP_NODE_PID" ] && kill -0 "$TEMP_NODE_PID" 2>/dev/null; then
        echo ""
        echo "[run_mission.sh] Deteniendo navsat_transform_node temporal (PID: $TEMP_NODE_PID)..."
        kill -TERM "$TEMP_NODE_PID" 2>/dev/null || true
        # Esperar brevemente a que el proceso termine
        for _ in {1..10}; do
            if ! kill -0 "$TEMP_NODE_PID" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        kill -KILL "$TEMP_NODE_PID" 2>/dev/null || true
        wait "$TEMP_NODE_PID" 2>/dev/null || true
        echo "[run_mission.sh] navsat_transform_node temporal detenido limpiamente."
        TEMP_NODE_PID=""
    fi
    return $exit_code
}

# Configuración de trap para garantizar la limpieza ante cualquier salida o señal
trap cleanup_temp_node EXIT INT TERM ERR

echo "========================================================================"
echo "  EARTH ROVER - ORQUESTADOR DE MISIÓN"
echo "========================================================================"
if [ -n "$MISSION_SLUG" ]; then
    echo "  Misión: $MISSION_SLUG"
fi
echo "  SDK URL: $SDK_URL"
echo "  Saltear Mapa Semilla: $SKIP_SEED_MAP"
echo "========================================================================"

# ------------------------------------------------------------------------------
# PASO 1: Resolver el datum dinámico
# ------------------------------------------------------------------------------
echo ""
echo "[PASO 1/4] Resolviendo datum dinámico desde el checkpoint inicial del SDK..."
RESOLVE_ARGS=(
    --sdk-url "$SDK_URL"
    --output "${WORKSPACE_ROOT}/src/mini_plus_localization/config/datum_resolved.yaml"
)
if [ -n "$MISSION_SLUG" ]; then
    RESOLVE_ARGS+=(--mission-slug "$MISSION_SLUG")
fi

if ! python3 "${WORKSPACE_ROOT}/tools/mission_prep/resolve_datum.py" "${RESOLVE_ARGS[@]}"; then
    echo ""
    echo "[run_mission.sh] ERROR FATAL: No se pudo resolver el datum desde el SDK." >&2
    echo "[run_mission.sh] Abortando antes de iniciar la misión (sin datum determinista no se puede garantizar la navegación)." >&2
    exit 1
fi

SEED_MAP_FILE="${WORKSPACE_ROOT}/tools/osm_seed/seed_map.npy"
SEED_MAP_PATH_PARAM=""

# ------------------------------------------------------------------------------
# PASO 2 y 3: Generación del Mapa Semilla OSM (opcional / con degradación suave)
# ------------------------------------------------------------------------------
if [ "$SKIP_SEED_MAP" = false ]; then
    echo ""
    echo "[PASO 2/4] Levantando navsat_transform_node temporal con datum resuelto..."

    DATUM_YAML="${WORKSPACE_ROOT}/src/mini_plus_localization/config/datum_resolved.yaml"
    EKF_YAML="${WORKSPACE_ROOT}/src/mini_plus_localization/config/ekf.yaml"

    ros2 run robot_localization navsat_transform_node \
        --ros-args \
        --params-file "$EKF_YAML" \
        --params-file "$DATUM_YAML" \
        -r imu:=/imu/data -r gps/fix:=/gps/fix -r odometry/filtered:=/odometry/global \
        > /tmp/temp_navsat_transform.log 2>&1 &
    TEMP_NODE_PID=$!

    echo "[run_mission.sh] navsat_transform_node iniciado en background (PID $TEMP_NODE_PID)."
    echo "[run_mission.sh] Esperando disponibilidad del servicio /fromLL..."

    SERVICE_READY=false
    for i in $(seq 1 40); do
        if ! kill -0 "$TEMP_NODE_PID" 2>/dev/null; then
            echo "[run_mission.sh] ERROR: navsat_transform_node temporal terminó prematuramente. Log:" >&2
            cat /tmp/temp_navsat_transform.log >&2
            break
        fi

        if ros2 service list 2>/dev/null | grep -q "^/fromLL$"; then
            SERVICE_READY=true
            echo "[run_mission.sh] Servicio /fromLL disponible tras $((i * 250))ms."
            break
        fi
        sleep 0.25
    done

    if [ "$SERVICE_READY" = false ]; then
        echo "[run_mission.sh] WARNING: Timeout esperando servicio /fromLL de navsat_transform_node." >&2
        echo "[run_mission.sh] Continuando sin mapa semilla (fail open)..." >&2
        cleanup_temp_node
    else
        echo ""
        echo "[PASO 3/4] Generando mapa semilla OSM..."
        OSM_GEN_ARGS=(
            --datum-file "$DATUM_YAML"
            --config "${WORKSPACE_ROOT}/src/er_planning/config/persistent_map_params.yaml"
            --output "$SEED_MAP_FILE"
        )
        if [ "$FORCE_DOWNLOAD" = true ]; then
            OSM_GEN_ARGS+=(--force-download)
        fi

        # Ejecución del generador de mapa semilla con degradación controlada
        if python3 "${WORKSPACE_ROOT}/tools/osm_seed/generate_seed_map.py" "${OSM_GEN_ARGS[@]}"; then
            echo "[run_mission.sh] Mapa semilla OSM generado correctamente en '$SEED_MAP_FILE'."
            SEED_MAP_PATH_PARAM="seed_map_path:=${SEED_MAP_FILE}"
        else
            echo ""
            echo "************************************************************************" >&2
            echo "  [run_mission.sh] WARNING: Falló la generación del mapa semilla OSM." >&2
            echo "  (Servidor de OSM caído, timeout de red o zona sin datos de vías)." >&2
            echo "  DEGRADACIÓN: Continuando con la misión sin mapa semilla (fail open)." >&2
            echo "************************************************************************" >&2
            SEED_MAP_PATH_PARAM=""
        fi

        # Bajar el nodo temporal una vez terminado el paso de generación
        cleanup_temp_node
    fi
else
    echo ""
    echo "[PASO 2/4 y 3/4] Generación de mapa semilla OSM omitida (--skip-seed-map)."
fi

# Desactivar trap de error antes de ceder control al launch final
trap - EXIT INT TERM ERR

# ------------------------------------------------------------------------------
# PASO 4: Lanzar la misión completa en ROS 2
# ------------------------------------------------------------------------------
echo ""
echo "[PASO 4/4] Lanzando misión completa en ROS 2..."
LAUNCH_CMD=(ros2 launch er_bringup mission1.launch.py)

if [ -n "$SEED_MAP_PATH_PARAM" ]; then
    LAUNCH_CMD+=("$SEED_MAP_PATH_PARAM")
fi

if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    LAUNCH_CMD+=("${EXTRA_ARGS[@]}")
fi

echo "[run_mission.sh] Ejecutando: ${LAUNCH_CMD[*]}"
echo "========================================================================"
exec "${LAUNCH_CMD[@]}"
