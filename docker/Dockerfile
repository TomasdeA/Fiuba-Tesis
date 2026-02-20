FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo git curl wget bash-completion locales tzdata \
    python3-colcon-common-extensions python3-vcstool python3-argcomplete \
    build-essential cmake clang-format clang-tidy \
    python3-numpy python3-matplotlib \
    # === ROS & visión ===
    ros-humble-realsense2-camera \
    ros-humble-vision-opencv \
    # cv_bridge + image_geometry
    ros-humble-rqt ros-humble-rqt-image-view \
    # === OpenCV GUI (imshow) ===
    libopencv-dev libgtk-3-0 libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
 && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
 && usermod -aG sudo $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
ENV RUNNING_IN_CONTAINER=1
RUN echo '[ -f /workspaces/tesis_nav_assistant_ws/tools/dev/context_env.sh ] && source /workspaces/tesis_nav_assistant_ws/tools/dev/context_env.sh' >> /etc/bash.bashrc

WORKDIR /workspaces/tesis_nav_assistant_ws
USER $USERNAME
