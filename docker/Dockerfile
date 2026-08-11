# syntax=docker/dockerfile:1.5
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Docker setea esto automaticamente (amd64 en PC, arm64 en Raspi)
ARG TARGETARCH

# === Base tools ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git curl wget bash-completion locales tzdata \
    python3-colcon-common-extensions python3-vcstool python3-argcomplete \
    build-essential cmake clang-format clang-tidy \
    python3-numpy python3-matplotlib \
    libopencv-dev libgtk-3-0 libgl1-mesa-glx \
    libgpiod-dev \
    && rm -rf /var/lib/apt/lists/*

# === ROS & vision (comun: PC + Raspi) ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-realsense2-camera \
    ros-humble-vision-opencv \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-imu-filter-madgwick \
    ros-humble-rtabmap-ros \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# === Solo en PC (amd64): GUI tools (viewer/rqt/imshow) ===
RUN if [ "$TARGETARCH" = "amd64" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rqt ros-humble-rqt-image-view \
    libopencv-dev libgtk-3-0 libgl1-mesa-glx \
    python3-matplotlib python3-pygame \
    && rm -rf /var/lib/apt/lists/* ; \
    fi
# === Usuario dev ===
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# === Shell init ===
# El paquete ROS instala el wrapper y librealsense con versiones compatibles.
# En Raspi usa el backend V4L2 del kernel; el host debe cargar uvcvideo.
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV RUNNING_IN_CONTAINER=1
RUN echo '[ -n "$WS_PATH" ] && [ -f "$WS_PATH/tools/dev/context_env.sh" ] && source "$WS_PATH/tools/dev/context_env.sh"' >> /etc/bash.bashrc

WORKDIR /workspaces/tesis_nav_assistant_ws
USER $USERNAME
