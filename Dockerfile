FROM ros:jazzy-ros-base

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
    wget \
    gnupg \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Instalar Google Chrome estable para soporte completo de códec H.264
RUN wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | gpg --dearmor -o /usr/share/keyrings/google-chrome.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/google-chrome.gpg] http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list && \
    apt-get update && apt-get install -y --no-install-recommends google-chrome-stable && \
    rm -rf /var/lib/apt/lists/*

ENV CHROME_EXECUTABLE_PATH=/usr/bin/google-chrome-stable

# Asegurar que el entorno ROS se cargue en shells interactivos
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
 
# 1. Copiar manifiestos de dependencias Python (SDK e IA)
COPY src/sdk_server/requirements.txt src/sdk_server/requirements.txt
COPY src/earth_rovers_sdk/requirements.txt src/earth_rovers_sdk/requirements.txt
COPY requirements-ai.txt requirements-ai.txt

# 2. Copiar código fuente de submódulos requeridos para instalación editable (-e)
COPY src/third_party/sana-earth-rover-policy/ src/third_party/sana-earth-rover-policy/

# 3. Instalar dependencias Python del SDK y stack de IA
RUN pip3 install --break-system-packages -r src/sdk_server/requirements.txt
RUN pip3 install --break-system-packages -r src/earth_rovers_sdk/requirements.txt
RUN python3 -m playwright install --with-deps chromium
RUN pip3 install --break-system-packages -r requirements-ai.txt
RUN pip3 install --break-system-packages --no-build-isolation \
    -e src/third_party/sana-earth-rover-policy/genie
RUN pip3 install --break-system-packages -e \
    'src/third_party/sana-earth-rover-policy/traversability[hf]'

# 4. Copiar todo el workspace de ROS 2
COPY . /root/ros2_ws

# 5. Inicializar rosdep y resolver dependencias del workspace antes de compilar
RUN rosdep update || true
RUN rosdep install --from-paths src --ignore-src -r -y || true

# 6. Construir el workspace
RUN /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# Exponer puerto del SDK (Hypercorn) y cualquier puerto ROS que quieras mapear
EXPOSE 8000

# Copiar script de entrada y marcar ejecutable
COPY entrypoint.sh /root/ros2_ws/entrypoint.sh
RUN chmod +x /root/ros2_ws/entrypoint.sh

ENTRYPOINT ["/root/ros2_ws/entrypoint.sh"]
