# 基于 l4t版本的humble子版本构建
FROM snowolf2012/l4t-humble-base

# 禁用交互式界面
ENV DEBIAN_FRONTEND=noninteractive

# 设置工作目录
WORKDIR /opt/csi_ws

# 解决CSI镜头颜色偏红问题
RUN mkdir -p /var/nvidia/nvcam/settings/ 
COPY docker/init/camera_overrides.isp /var/nvidia/nvcam/settings/
RUN chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
RUN chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp

# 安装工程依赖包
COPY docker/init/requirements.txt /opt/csi_ws/
RUN pip install -r /opt/csi_ws/requirements.txt

# 复制整个工程目录到工作目录中
COPY csi_cam_service/ /opt/csi_ws/src/

# 拷贝测试文件
COPY docker/init/multi_csi_camera.py /opt/csi_ws/
COPY docker/init/test.sh /opt/csi_ws/

# 构建工程
RUN rm -rf build install && colcon build --symlink-install --packages-select csi_cam_service
RUN echo "source /opt/csi_ws/install/setup.bash " >> ~/.bashrc

# 设置容器启动时要执行的默认命令
COPY docker/init/launch_script.sh /opt/csi_ws/
CMD ["/opt/csi_ws/launch_script.sh"]