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
    ros-humble-rqt ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# === SOLO Raspi (arm64): build librealsense en /usr/local ===
RUN if [ "$TARGETARCH" = "arm64" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential pkg-config \
    libusb-1.0-0-dev libudev-dev \
    libgl1-mesa-dev libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/* \
    && cd /tmp \
    && git clone --depth 1 --branch v2.57.6 https://github.com/IntelRealSense/librealsense.git \
    && cd librealsense \
    && mkdir build && cd build \
    && cmake .. \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_GRAPHICAL_EXAMPLES=OFF \
    -DBUILD_WITH_OPENMP=OFF \
    && make -j2 \
    && make install \
    && ldconfig \
    && rm -rf /tmp/librealsense ; \
    fi

# === SOLO Raspi (arm64): forzar que el wrapper de ROS use la librealsense nueva ===
RUN if [ "$TARGETARCH" = "arm64" ]; then \
    set -e; \
    LATEST="$(ls -1 /usr/local/lib/librealsense2.so.2.*.* 2>/dev/null | sort -V | tail -n1)"; \
    echo "Using $LATEST"; \
    if [ -f /opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.56 ]; then \
    rm -f /opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.56; \
    fi; \
    ln -sf "$LATEST" /opt/ros/humble/lib/aarch64-linux-gnu/librealsense2.so.2.56; \
    ldconfig; \
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
# /usr/local/bin y /usr/local/lib deben tener precedencia sobre los de ROS apt
# para que librealsense buildeada desde source (v2.57.6, RSUSB backend) sea la
# que usa el wrapper de ROS, tanto para binarios (rs-enumerate-devices, etc.)
# como para la carga dinámica.
ENV PATH="/usr/local/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "export PATH=/usr/local/bin:\$PATH" >> /etc/bash.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /etc/bash.bashrc
ENV RUNNING_IN_CONTAINER=1
RUN echo '[ -f /workspaces/tesis_nav_assistant_ws/tools/dev/context_env.sh ] && source /workspaces/tesis_nav_assistant_ws/tools/dev/context_env.sh' >> /etc/bash.bashrc

WORKDIR /workspaces/tesis_nav_assistant_ws
USER $USERNAME
