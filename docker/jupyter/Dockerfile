# ==============================================================================
# Jupyter Lab 镜像
# ==============================================================================
# 基于 l4t-ros2-csi-node 构建，添加 Jupyter 扩展配置
#
# 构建: docker build -f docker/jupyter/Dockerfile -t l4t-jupyter:latest .
#
# 注意: 基础镜像已包含 Jupyter，这里仅添加配置和扩展
# ==============================================================================

ARG BASE_IMAGE=l4t-ros2-csi-node:latest
FROM ${BASE_IMAGE}

LABEL maintainer="snowolf2012" \
      description="Jupyter Lab for ROS2 CSI Camera" \
      version="1.0.0"

# 禁用交互式界面
ENV DEBIAN_FRONTEND=noninteractive

# Jupyter 工作目录
WORKDIR /opt/workspace

# ============================================================================
# 安装 Jupyter 扩展组件
# ============================================================================
RUN pip install --no-cache-dir \
    jupyterlab-language-pack-zh-CN \
    qtconsole \
    ipywidgets \
    voila && \
    rm -rf /root/.cache/pip

# ============================================================================
# Jupyter 配置
# ============================================================================
RUN mkdir -p /root/.jupyter/
COPY docker/jupyter/jupyter_server_config.json /root/.jupyter/

# 拷贝 Notebook 示例
COPY notebook/multi_csi_cam.ipynb /opt/workspace/

# 入口点
ENTRYPOINT ["/entrypoint.sh"]

# 启动脚本
COPY docker/jupyter/jupyter.sh /opt/workspace/
RUN chmod +x /opt/workspace/jupyter.sh

# 暴露端口
EXPOSE 8888

# 默认命令
CMD ["/opt/workspace/jupyter.sh"]
