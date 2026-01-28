FROM nvidia/cuda:12.2.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# ---------- Base system ----------
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    build-essential \
    git \
    wget \
    nano \
    ca-certificates \
    dbus-x11 \
    x11-apps \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---------- ROS 2 Humble ----------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true
RUN rosdep update

# ---------- Gazebo Classic ----------
RUN apt update && apt install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# ---------- Environment ----------
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
