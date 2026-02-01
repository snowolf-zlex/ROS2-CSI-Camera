# ==============================================================================
# ROS2 CSI 相机节点镜像
# ==============================================================================
# 基于 l4t-humble-base 构建，添加 CSI 相机支持和工程代码
#
# 构建: docker build -f docker/init/Dockerfile -t l4t-ros2-csi-node:latest .
# ==============================================================================

ARG BASE_IMAGE=l4t-humble-base:latest
FROM ${BASE_IMAGE}

LABEL maintainer="snowolf2012" \
      description="ROS2 CSI Camera Node for NVIDIA Jetson" \
      version="1.0.0"

# 禁用交互式界面
ENV DEBIAN_FRONTEND=noninteractive

# 设置工作目录
WORKDIR /opt/workspace

# ============================================================================
# CSI 镜头颜色校准 (解决偏红问题)
# ============================================================================
RUN mkdir -p /var/nvidia/nvcam/settings/
COPY docker/init/camera_overrides.isp /var/nvidia/nvcam/settings/
RUN chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp && \
    chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

# ============================================================================
# Python 依赖
# ============================================================================
COPY docker/init/requirements.txt /opt/workspace/
RUN pip install --no-cache-dir -r /opt/workspace/requirements.txt && \
    rm -rf /root/.cache/pip

# ============================================================================
# 复制工程代码
# ============================================================================
COPY csi_cam_service/ /opt/workspace/src/

# 拷贝测试文件
COPY docker/init/multi_csi_camera.py /opt/workspace/
COPY docker/init/test.sh /opt/workspace/

# ============================================================================
# 构建 ROS2 工程
# ============================================================================
RUN cd /opt/workspace && \
    rm -rf build install && \
    colcon build --symlink-install --packages-select csi_cam_service && \
    echo "source /opt/workspace/install/setup.bash" >> ~/.bashrc && \
    chmod +x /opt/workspace/test.sh

# ============================================================================
# 启动脚本
# ============================================================================
COPY docker/init/launch_script.sh /opt/workspace/
RUN chmod +x /opt/workspace/launch_script.sh

# 入口点
ENTRYPOINT ["/entrypoint.sh"]

# 默认命令
CMD ["/opt/workspace/launch_script.sh"]

# 暴露端口
EXPOSE 8554
