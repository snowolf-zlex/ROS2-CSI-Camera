# Docker 模块

基于 NVIDIA Jetson 平台的 ROS2 Humble Docker 镜像构建系统。

## 镜像架构

```
┌─────────────────────────────────────────────────────────────┐
│                    nvcr.io/nvidia/l4t-base:r36.2.0          │
│                       (JetPack 5.1.2)                        │
└────────────────────────────┬────────────────────────────────┘
                             │
                    ┌────────▼─────────┐
                    │  l4t-humble-base  │  ROS2 Humble 基础环境
                    │  (standard/slim/  │  - ROS2 Desktop
                    │   runtime)        │  - GStreamer
                    └────────┬─────────┘  - OpenCV
                             │
                    ┌────────▼──────────┐
                    │ l4t-ros2-csi-node │  CSI 相机节点
                    └────────┬──────────┘  - CSI 颜色校准
                             │              - csi_cam_service
                    ┌────────▼──────────┐
                    │   l4t-jupyter     │  Jupyter 开发环境
                    └───────────────────┘  - JupyterLab
                                            - 中文支持
```

## 目录结构

```
docker/
├── base/              # ROS2 Humble 基础镜像
│   ├── Dockerfile     # 支持 3 种构建目标
│   └── build.sh       # 基础镜像构建脚本
├── init/              # CSI 相机节点镜像
│   ├── Dockerfile
│   ├── build.sh
│   ├── camera_overrides.isp   # CSI 颜色校准
│   ├── launch_script.sh       # 启动脚本
│   ├── multi_csi_camera.py    # 测试脚本
│   ├── requirements.txt
│   └── test.sh
├── jupyter/           # Jupyter 镜像
│   ├── Dockerfile
│   ├── build.sh
│   ├── jupyter_server_config.json
│   └── jupyter.sh
└── README.md
```

## 快速开始

### 方式 1: 构建所有镜像（推荐）

```bash
./build.sh
# 或
./build.sh all
```

### 方式 2: 单独构建

```bash
# 仅构建基础镜像
./build.sh base

# 仅构建 CSI 节点镜像
./build.sh init

# 仅构建 Jupyter 镜像
./build.sh jupyter
```

### 方式 3: 直接使用子目录脚本

```bash
# 基础镜像 (支持 standard/slim/runtime)
./docker/base/build.sh          # 默认 standard
./docker/base/build.sh slim     # 精简版本
YES=true ./docker/base/build.sh  # 跳过确认

# CSI 节点镜像
./docker/init/build.sh

# Jupyter 镜像
./docker/jupyter/build.sh
```

## 镜像说明

### 1. l4t-humble-base (基础镜像)

ROS2 Humble 基础环境，支持 3 种构建目标：

| 目标 | 说明 | 体积 |
|------|------|------|
| standard | 完整开发环境 (默认) | ~4GB |
| slim | 精简镜像，移除开发工具 | ~3.5GB |
| runtime | 最小运行时依赖 | ~3GB |

**构建命令**:
```bash
# 标准构建
docker build -f docker/base/Dockerfile -t l4t-humble-base:latest .

# 精简构建
docker build -f docker/base/Dockerfile --target slim -t l4t-humble-base:slim .

# 运行时构建
docker build -f docker/base/Dockerfile --target runtime -t l4t-humble-base:runtime .
```

**已安装组件**:
- ROS2 Humble Desktop
- GStreamer 1.0 + GstRtspServer
- OpenCV + Python 绑定
- numpy, scipy, matplotlib
- Jupyter/JupyterLab (standard/slim)

### 2. l4t-ros2-csi-node (CSI 相机节点)

基于基础镜像构建，添加：
- CSI 颜色校准配置 (camera_overrides.isp)
- csi_cam_service ROS2 包
- 测试脚本

**构建命令**:
```bash
docker build -f docker/init/Dockerfile -t l4t-ros2-csi-node:latest .
```

### 3. l4t-jupyter (Jupyter 开发环境)

基于 CSI 节点镜像，添加：
- JupyterLab
- 中文本地化
- 示例 Notebook

**构建命令**:
```bash
docker build -f docker/jupyter/Dockerfile -t l4t-jupyter:latest .
```

## 运行镜像

### CSI 相机节点

```bash
docker run -it --rm \
    --network host \
    --device /dev/video0 \
    --device /dev/video1 \
    l4t-ros2-csi-node:latest
```

### Jupyter 环境

```bash
docker run -it --rm \
    --network host \
    -v $(pwd)/notebook:/opt/workspace \
    l4t-jupyter:latest
```

访问: http://localhost:8888

## 测试

进入容器后测试：

```bash
# 测试 GStreamer
./test.sh

# 测试 CSI 相机
python3 multi_csi_camera.py

# 测试 ROS2 节点
ros2 launch csi_cam_service stereo_cam.launch.py
```

## 常见问题

### Q: CSI 摄像头颜色偏红？

A: 已包含 `camera_overrides.isp` 校准文件，自动修复颜色问题。

### Q: 需要特权模式？

A: CSI 摄像头可能需要：
```bash
docker run --privileged ...
```

### Q: 如何查看构建进度？

A: 构建脚本会显示每一步的输出，完成后自动显示镜像信息。

## 维护

### 更新基础镜像

```bash
docker pull nvcr.io/nvidia/l4t-base:r36.2.0
./build.sh base
```

### 清理旧镜像

```bash
docker system prune -a
```

## 许可证

MIT License
