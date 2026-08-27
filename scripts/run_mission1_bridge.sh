#!/usr/bin/env bash
# Opción A — Bridge directo (sin EKF)
# SDK -> earth_rover_bridge -> mission1.launch.py (gps_waypoint_controller + mission_manager_node)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="${ROOT_DIR:-$(cd "$SCRIPT_DIR/.." && pwd)}"
PID_DIR="$ROOT_DIR/.mission1_pids"
mkdir -p "$PID_DIR"

cleanup() {
    echo ""
    echo "Deteniendo procesos..."
    [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
    [ -n "$SDK_PID" ] && kill "$SDK_PID" 2>/dev/null || true
    rm -f "$PID_DIR"/*.pid
    echo "Listo."
}
trap cleanup EXIT INT TERM

echo "=== 1. Verificando MISSION_SLUG en .env ==="
if ! grep -qE "^MISSION_SLUG=" "$ROOT_DIR/src/sdk_server/.env"; then
    echo "ADVERTENCIA: no encontré una línea MISSION_SLUG=... activa (sin '#') en"
    echo "  $ROOT_DIR/src/sdk_server/.env"
    echo "El SDK puede fallar en /start-mission si no está seteada. Editá el .env y reintentá."
    echo "Continuando de todos modos en 5s (Ctrl+C para cancelar)..."
    sleep 5
fi

echo "=== 2. Iniciando SDK ==="
if [ -f "$ROOT_DIR/.venv/bin/activate" ]; then
    source "$ROOT_DIR/.venv/bin/activate"
fi
cd "$ROOT_DIR"
export PYTHONPATH="$ROOT_DIR/src/sdk_server:$PYTHONPATH"
python3 "$ROOT_DIR/run_sdk.py" &
SDK_PID=$!
echo "$SDK_PID" > "$PID_DIR/sdk.pid"
echo "SDK iniciado con PID $SDK_PID"

echo "Esperando a que el SDK responda en :8000..."
for i in $(seq 1 15); do
    if curl -sf http://localhost:8000/data > /dev/null 2>&1; then
        echo "SDK OK."
        break
    fi
    sleep 1
    if [ "$i" -eq 15 ]; then
        echo "ADVERTENCIA: el SDK no respondió en 15s. Sigo igual, pero revisá los logs si algo falla."
    fi
done

echo "=== 3. Iniciando misión vía SDK (/start-mission) ==="
START_RESPONSE=$(curl -sf -X POST http://localhost:8000/start-mission)
if [ $? -ne 0 ]; then
    echo "ERROR: no se pudo iniciar la misión en el SDK."
    echo "Revisá SDK_API_TOKEN / BOT_SLUG / MISSION_SLUG en $ROOT_DIR/src/sdk_server/.env"
    exit 1
fi
echo "Misión iniciada: $START_RESPONSE"

echo "=== 4. Iniciando bridge ROS (earth_rover_bridge) ==="
source /opt/ros/jazzy/setup.bash
source "$ROOT_DIR/install/setup.bash"

ros2 run earth_rovers_sdk earth_rover_bridge --ros-args -p sdk_url:=http://localhost:8000 &
BRIDGE_PID=$!
echo "$BRIDGE_PID" > "$PID_DIR/bridge.pid"
echo "Bridge iniciado con PID $BRIDGE_PID"

sleep 2

echo "=== 5. Lanzando misión 1 (gps_waypoint_controller + mission_manager_node) ==="
echo "---"
echo "SDK PID:    $SDK_PID"
echo "Bridge PID: $BRIDGE_PID"
echo ""
echo "Ctrl+C detiene todo (SDK + bridge + nodos de misión)."
echo "---"

ros2 launch er_bringup mission1.launch.py