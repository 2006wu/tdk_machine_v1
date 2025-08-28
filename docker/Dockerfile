# =========================
# Base: ROS Humble
# =========================
FROM osrf/ros:humble-desktop-full AS base
ARG DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm-256color LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# 基本工具
COPY ./scripts/entrypoint/ros_entrypoint.sh /ros_entrypoint.sh
RUN chown root:root /ros_entrypoint.sh && chmod 755 /ros_entrypoint.sh

# 安裝 pip + 常用工具
RUN apt-get update && apt-get install -y \
    python3-pip git \
    && pip3 install pyserial colcon-common-extensions \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*

# =========================
# Target 1: wu-ros2 (lightweight)
# =========================
FROM base AS wu-ros2

WORKDIR /ros2_ws

# 自動 source + 彩色 shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && \
    echo 'export PS1="\[\e[32m\]wu-ros2\[\e[m\]@\[\e[34m\]\w# \[\e[m\]"' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# =========================
# librealsense builder
# =========================
FROM base AS librealsense-builder
ARG LIBREALSENSE_VERSION=2.55.1
RUN apt-get update && apt-get install -y \
    cmake pkg-config build-essential python3 python3-dev ca-certificates \
    libssl-dev libusb-1.0-0-dev libudev-dev libgtk-3-dev \
    && apt-get clean -y && rm -rf /var/lib/apt/lists/*
WORKDIR /usr/src
RUN curl -L https://codeload.github.com/IntelRealSense/librealsense/tar.gz/refs/tags/v$LIBREALSENSE_VERSION -o librealsense.tar.gz && \
    tar -zxf librealsense.tar.gz && rm librealsense.tar.gz && \
    ln -s /usr/src/librealsense-$LIBREALSENSE_VERSION /usr/src/librealsense
RUN cd /usr/src/librealsense && mkdir build && cd build && \
    cmake -DPYTHON_EXECUTABLE=$(which python3) \
          -DCMAKE_INSTALL_PREFIX=/opt/librealsense \
          -DBUILD_GRAPHICAL_EXAMPLES=OFF \
          -DBUILD_PYTHON_BINDINGS=ON \
          -DCMAKE_BUILD_TYPE=Release ../ && \
    make -j$(($(nproc)-1)) all && make install

# =========================
# Target 2: realsense
# =========================
FROM base AS realsense
ARG REALSENSE_ROS_VERSION=4.55.1

ENV ROS_WS_PATH=/ros2_ws
ENV PYTHONPATH=$PYTHONPATH:/usr/local/lib
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 複製 librealsense runtime
COPY --from=librealsense-builder /opt/librealsense /usr/local/
COPY --from=librealsense-builder /usr/lib/python3/dist-packages/pyrealsense2 /usr/lib/python3/dist-packages/pyrealsense2
COPY --from=librealsense-builder /usr/src/librealsense/config/*.rules /etc/udev/rules.d/

# 安裝 realsense-ros 依賴
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 udev ca-certificates \
    ros-humble-diagnostic-updater ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    python3-tqdm python3-requests \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    git && \
    apt-get clean -y && rm -rf /var/lib/apt/lists/*

# 建立使用者
WORKDIR /ros2_ws

ENV ROSDEP_SKIP_KEYS="librealsense2 ros-humble-librealsense2"
ENV ROSDEP_ADDITIONAL_OPTIONS="--rosdistro humble --skip-keys=librealsense2 --skip-keys=ros-humble-librealsense2"
ENV ROSDEP_APT_EXEC="apt-get" \
    ROSDEP_YES=1

# Clone realsense-ros
# Clone realsense-ros (skip rosdep)
RUN mkdir -p src && \
    git clone --branch $REALSENSE_ROS_VERSION https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros


# Auto-source ROS setup + 彩色 shell
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && \
    echo 'export PS1="\[\e[32m\]realsense\[\e[m\]@\[\e[34m\]\w# \[\e[m\]"' >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
