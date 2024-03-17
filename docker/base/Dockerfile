# 基于 ROS Humble 的 Dockerfile
# nvcr.io/nvidia/l4t-base:r36.2.0 基于Ubuntu 22.04LTS，适用于Humble
FROM nvcr.io/nvidia/l4t-base:r36.2.0

# 禁用交互式界面
ENV DEBIAN_FRONTEND=noninteractive

# 安装Python必要开发组件
RUN apt update && apt install -y \
    python3 \
    python3-pip \
    gcc-aarch64-linux-gnu \
    v4l-utils \
    net-tools \
    vim \
    ssh \
    curl \
    git \
    freeglut3 \
    freeglut3-dev \
    libsfml-dev \
    --no-install-recommends && rm -rf /var/lib/apt/lists/*

# 安装ROS Humble
RUN locale  # check for UTF-8

RUN apt update && apt install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN locale  # verify settings

RUN apt install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt upgrade -y && apt autoremove -y

# 安装 ROS Humble相关工具
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-rqt* \
    ros-humble-cv-bridge \
    ros-humble-v4l2-camera \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-image-publisher \
    libgflags-dev \
    nlohmann-json3-dev \
    libgoogle-glog-dev \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    --no-install-recommends && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep
RUN rosdep init && \
    rosdep update

# 设置环境变量 DISPLAY
ENV DISPLAY=:0

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
