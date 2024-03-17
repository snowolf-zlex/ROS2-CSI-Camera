# 基于 l4t_humble_base:latest
ARG BASE_IMAGE=l4t-ros2-csi-node:latest
FROM ${BASE_IMAGE}

# 禁用交互式界面
ENV DEBIAN_FRONTEND=noninteractive

ENV JUPYTER_WORKDIR=/opt/workspace
WORKDIR /opt/workspace

RUN pip install --upgrade pip

RUN pip install -i http://mirrors.aliyun.com/pypi/simple/ --trusted-host mirrors.aliyun.com \
    jupyter \
    jupyterlab-language-pack-zh-CN \
    qtconsole \
    ipywidgets \
    voila==0.5.0a4 

RUN pip install --upgrade jupyterlab

RUN mkdir -p /root/.jupyter/
COPY docker/jupyter/jupyter_server_config.json /root/.jupyter/

# 设置环境变量 DISPLAY
ENV DISPLAY=:0

# 设置容器启动时要执行的默认命令
COPY docker/jupyter/jupyter.sh $JUPYTER_WORKDIR
COPY notebook/multi_csi_cam.ipynb $JUPYTER_WORKDIR
RUN chmod +x /opt/workspace/jupyter.sh
CMD ["/opt/workspace/jupyter.sh"]