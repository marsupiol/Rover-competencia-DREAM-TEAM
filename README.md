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

Opciones para compartir con amigos

A) Usar el workflow de GitHub Actions (recomendado):
- Push al repo en GitHub y ejecutar el workflow `Build and save Docker image artifact` desde la interfaz.
- Descargar el artefacto `mini_plus_ros2_ci.tar.gz` desde la ejecución del workflow.
- En la máquina de tu amigo:

```bash
gunzip -c mini_plus_ros2_ci.tar.gz | docker load
docker run -d --name mini_ros -p 8000:8000 youruser/mini_plus_ros2:jazzy
```

B) Subir la imagen a Docker Hub / GHCR:

```bash
# después de hacer login
docker tag mini_plus_ros2:ci youruser/mini_plus_ros2:jazzy
docker push youruser/mini_plus_ros2:jazzy
# tus amigos la ejecutan con:
docker run -d --name mini_ros -p 8000:8000 youruser/mini_plus_ros2:jazzy
```

C) Si no usan Docker, pueden ejecutar localmente con los scripts de `./scripts`.

Cómo empujar este repo a GitHub (pasos sencillos)

1) Crear un repo vacío en GitHub (usa la web o `gh repo create`).
2) Ejecutar (sustituir URL del repo remoto):

```bash
git remote add origin git@github.com:TU_USUARIO/TU_REPO.git
git branch -M main
git push -u origin main
```

Si querés que lo suba yo usando `gh` CLI, decímelo y lo intento (necesitaré permisos/credenciales en el entorno).

Contacto rápido

Si querés, puedo también:
- Subir la imagen a Docker Hub desde CI (necesitarás configurar secrets),
- Ajustar el Dockerfile para optimizar tamaño, o
- Añadir un workflow que publique la imagen en GHCR automáticamente.
