mini_plus_ros2 — Contenedor ROS 2 Jazzy + SDK

Resumen rápido

- Este repo contiene un SDK web (FastAPI) y un bridge ROS2 (Python) integrados.
- CI (GitHub Actions) puede construir la imagen Docker y subir un artefacto `mini_plus_ros2_ci.tar.gz`.

Instrucciones para desarrollar / ejecutar localmente

1) Crear y activar virtualenv (opcional para desarrollo Python):

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r src/sdk_server/requirements.txt
# si usás Playwright:
python -m playwright install chromium
```

2) Ejecutar SDK localmente (desde la raíz del repo):

```bash
# SDK sólo
./scripts/run_sdk.sh
# o todo (SDK + bridge + ekf)
./scripts/run_all.sh
```

Construir la imagen Docker localmente

```bash
# desde la raíz del repo
docker build -t youruser/mini_plus_ros2:jazzy -f Dockerfile .
```

