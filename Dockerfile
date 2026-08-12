FROM osrf/ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Instalar herramientas de build, dependencias de OpenCV y paquetes ROS necesarios
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    cmake \
    git \
    pkg-config \
    libopencv-dev \
    python3-opencv \
    ros-jazzy-robot-localization \
    ros-jazzy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

# Asegurar que el entorno ROS se cargue en shells interactivos
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

# Copiar todo el workspace
COPY . /root/ros2_ws

# Instalar dependencias Python del SDK
RUN pip3 install --upgrade pip setuptools && \
    pip3 install -r src/sdk_server/requirements.txt

# Inicializar rosdep y resolver dependencias del workspace antes de compilar
RUN rosdep update || true
RUN rosdep install --from-paths src --ignore-src -r -y || true

# Construir el workspace
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Exponer puerto del SDK (Hypercorn) y cualquier puerto ROS que quieras mapear
EXPOSE 8000

# Copiar script de entrada y marcar ejecutable
COPY entrypoint.sh /root/ros2_ws/entrypoint.sh
RUN chmod +x /root/ros2_ws/entrypoint.sh

ENTRYPOINT ["/root/ros2_ws/entrypoint.sh"]
