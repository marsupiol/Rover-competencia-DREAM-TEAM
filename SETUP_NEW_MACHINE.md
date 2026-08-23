# Guía de Setup en Máquina Nueva (Windows + WSL2 + Docker Desktop + NVIDIA GPU)

Esta guía documenta los pasos para replicar exactamente el entorno de desarrollo y ejecución de **Earth Rover Mini+** en una PC nueva con GPU NVIDIA bajo Windows y WSL2.

---

## 1. Prerrequisitos del Host (Windows)

> [!NOTE]
> Esta configuración se realiza una sola vez en el sistema operativo anfitrión (Windows), antes de iniciar Docker:
* **WSL2** instalado (`wsl --install` o `wsl --update` en PowerShell como Administrador).
* **Driver NVIDIA para Windows** actualizado (GeForce / RTX / Quadro con soporte de GPU Passthrough para WSL2).
* **Docker Desktop** instalado con la opción **"Use the WSL 2 based engine"** activada y la integración activada en *Settings > Resources > WSL Integration > [Tu Distro Ubuntu]*.

---

## 2. Clonar el Proyecto en el Filesystem Nativo de WSL2

> [!IMPORTANT]
> **NUNCA** clones ni ejecutes el proyecto dentro de `/mnt/c/...` (directorio montado de Windows). El puente de archivos 9P degrada el rendimiento de compilación de ROS 2 y genera problemas de permisos e inotify.

Abre tu terminal de WSL2 (Ubuntu) y clona el repositorio dentro de tu directorio home de Linux:
```bash
# Navegar al home nativo de WSL2
cd ~

# Clonar el repositorio
git clone git@github.com:marsupiol/Rover-competencia-DREAM-TEAM.git ros2_ws
cd ros2_ws
```

---

## 3. Verificar Passthrough de GPU antes de Construir el Proyecto

Antes de construir las imágenes del proyecto, verifica que Docker Desktop y el runtime de NVIDIA tengan acceso directo a tu GPU desde WSL2 ejecutando un contenedor de prueba con la **misma imagen base de ROS 2 Jazzy**:

```bash
docker run --rm --gpus all osrf/ros:jazzy-ros-base nvidia-smi
```

* **Resultado esperado:** Debes ver la tabla de `nvidia-smi` reportando tu GPU NVIDIA (Driver Version y CUDA Version del host). Si este comando falla, no continúes al siguiente paso (consulta la sección de Troubleshooting).

---

## 4. Construcción y Ejecución con Docker Compose

> [!TIP]
> Los archivos [`requirements-ai.txt`](requirements-ai.txt), [`requirements-ros.txt`](requirements-ros.txt) y [`apt-packages.txt`](apt-packages.txt) documentan las dependencias exactas que el Dockerfile resuelve e instala de forma automática. Al utilizar Docker, **NO** es necesario instalar nada a mano en tu máquina host.

### 4.1. Configuración de Credenciales
Copia el archivo de variables de entorno de ejemplo para el servidor SDK:
```bash
cp src/sdk_server/.env.sample src/sdk_server/.env
# Editar src/sdk_server/.env con las credenciales reales de FrodoBots si aplica
```

### 4.2. Construir la Imagen
```bash
docker compose build
```

### 4.3. Iniciar el Stack
```bash
docker compose up -d
```

*(Para ver los logs en tiempo real: `docker compose logs -f`)*

---

## 5. Verificación de la Instalación dentro del Contenedor

Ejecuta los siguientes 3 comandos para validar que el contenedor tiene acceso a la GPU, que PyTorch reconoce CUDA y que los paquetes ROS 2 están compilados:

1. **Verificar GPU dentro del contenedor:**
   ```bash
   docker exec -it mini_plus_rover nvidia-smi
   ```

2. **Verificar aceleración CUDA en PyTorch:**
   ```bash
   docker exec -it mini_plus_rover python3 -c "import torch; print('PyTorch Version:', torch.__version__); print('CUDA Available:', torch.cuda.is_available()); print('Device Name:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
   ```

3. **Verificar paquetes ROS 2 compilados e indexados:**
   ```bash
   docker exec -it mini_plus_rover bash -c "source install/setup.bash && ros2 pkg list | grep -E 'er_|mini_plus|earth_rover'"
   ```
   *Debe listar:* `earth_rovers_sdk`, `er_bringup`, `er_mission`, `er_navigation`, `er_perception`, `er_planning`, `mini_plus_localization`.

---

## 6. Troubleshooting Común

1. **Error: `docker: Error response from daemon: could not select device driver "" with capabilities: [[gpu]]`**
   * **Causa:** Docker Desktop no tiene habilitada la integración con WSL2 o falta el backend NVIDIA Container Toolkit en Docker Desktop.
   * **Solución:** En Docker Desktop, ir a *Settings > Resources > WSL Integration*, asegurarse de que el switch de tu distribución Ubuntu esté activado y reiniciar Docker Desktop.

2. **Error: `nvidia-smi` no detecta la GPU dentro de WSL2 o devuelve error de comunicación:**
   * **Causa:** Driver de NVIDIA en Windows desactualizado o instalación de drivers de Linux dentro de WSL2 (no se deben instalar drivers gráficos dentro de WSL).
   * **Solución:** Descargar e instalar el último driver GeForce / Studio desde el sitio oficial de NVIDIA en Windows. No instales drivers `.run` de NVIDIA dentro de WSL2.

3. **Compilación extremadamente lenta (`colcon build`) o errores de permisos:**
   * **Causa:** El repositorio fue clonado en la ruta de Windows (`/mnt/c/...` o `C:\...`) en lugar del filesystem nativo ext4 de WSL2.
   * **Solución:** Mover el directorio a `~/ros2_ws` dentro de WSL2 y volver a compilar.
