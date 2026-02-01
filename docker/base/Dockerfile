# ==============================================================================
# ROS2 Humble 基础镜像
# ==============================================================================
# 支持 3 种构建目标:
#   1. standard (默认) - 完整开发环境
#   2. slim - 精简镜像，移除开发工具
#   3. runtime - 最小运行时依赖
#
# 构建: docker build -f docker/base/Dockerfile -t l4t-humble-base:latest .
#       docker build -f docker/base/Dockerfile --target slim -t l4t-humble-base:slim .
# ==============================================================================

ARG BASE_IMAGE=nvcr.io/nvidia/l4t-base:r36.2.0

# ==============================================================================
# 阶段 1: 基础镜像
# ==============================================================================
FROM ${BASE_IMAGE} AS base

LABEL maintainer="snowolf2012" \
      description="ROS2 Humble Base for NVIDIA Jetson" \
      version="1.0.0"

# 构建参数
ARG TZ=Asia/Shanghai
ARG UBUNTU_CODENAME=jammy

# 环境变量
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=${TZ} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,display,graphics,video \
    DISPLAY=:0 \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# ============================================================================
# Layer 1: 系统基础配置
# ============================================================================
RUN set -eux; \
    echo "LANG=en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 && \
    ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone && \
    add-apt-repository -y universe && \
    apt-get update

# ============================================================================
# Layer 2: ROS2 源配置
# ============================================================================
RUN set -eux; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu ${UBUNTU_CODENAME} main" \
        > /etc/apt/sources.list.d/ros2.list; \
    apt-get update

# ============================================================================
# Layer 3: 核心软件包安装
# ============================================================================
RUN set -eux; \
    apt-get install -y --no-install-recommends \
    # Python 运行时
    python3 python3-pip \
    # ROS2 核心
    ros-humble-desktop \
    ros-humble-rqt-common-plugins \
    ros-humble-rviz2 \
    ros-humble-rosdep \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    # 相机相关
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-theora-image-transport \
    ros-humble-camera-calibration \
    # GStreamer
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstrtspserver-1.0-dev \
    python3-gi \
    # 摄像头工具
    v4l-utils libv4l-0 \
    # 计算库
    libopencv-dev python3-opencv \
    libeigen3-dev \
    # 图形库
    freeglut3-dev libglew-dev \
    # 基础工具
    curl wget ca-certificates vim-tiny \
    net-tools iputils-ping iproute2 \
    # 清理
    && rm -rf /var/lib/apt/lists/* \
    && apt-get autoremove -y \
    && apt-get clean

# ============================================================================
# Layer 4: Python 科学计算包
# ============================================================================
RUN set -eux; \
    python3 -m pip install --no-cache-dir --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir \
    numpy scipy matplotlib \
    opencv-python-headless \
    PyYAML requests pyserial \
    jupyter jupyterlab && \
    rm -rf /root/.cache/pip

# ============================================================================
# Layer 5: 开发工具
# ============================================================================
RUN set -eux; \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    build-essential gcc g++ make cmake ninja-build git \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# ============================================================================
# Layer 6: rosdep 初始化
# ============================================================================
RUN set -eux; \
    rosdep init || true; \
    rosdep update

# ============================================================================
# 通用配置
# ============================================================================
WORKDIR /opt/workspace

# 配置 bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "export PYTHONPATH=/opt/workspace:\$PYTHONPATH" >> /root/.bashrc && \
    echo "alias ws='cd /opt/workspace'" >> /root/.bashrc && \
    echo "alias gs='source /opt/ros/humble/setup.bash && source /opt/workspace/install/setup.bash 2>/dev/null || true'" >> /root/.bashrc

# 入口点脚本
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
if [ -f /opt/workspace/install/setup.bash ]; then\n\
    source /opt/workspace/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# 健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD ros2 --version || exit 1

# 默认命令
CMD ["bash"]

# 暴露端口
EXPOSE 8554

# 卷挂载
VOLUME ["/opt/workspace"]

# ==============================================================================
# 目标 2: 精简模式 (slim)
# ==============================================================================
FROM base AS slim

RUN set -eux; \
    apt-get update && \
    apt-get remove -y build-essential gcc g++ make cmake ninja-build git && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ==============================================================================
# 目标 3: 运行时模式 (runtime)
# ==============================================================================
FROM base AS runtime

RUN set -eux; \
    apt-get update && \
    apt-get remove -y \
        build-essential gcc g++ make cmake ninja-build git \
        python3-pip python3-setuptools python3-wheel \
        ros-humble-rqt-common-plugins ros-humble-rviz2 \
        freeglut3-dev libglew-dev && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip uninstall -y jupyter jupyterlab && \
    rm -rf /root/.cache/pip
