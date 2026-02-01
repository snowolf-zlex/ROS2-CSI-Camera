# Jetson Camera Toolkit 📹

[![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)](https://github.com/snowolf-zlex/jetson-camera-toolkit)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Jetson%20Nano%20%7C%20Xavier%20NX%20%7C%20Orin-orange.svg)](https://developer.nvidia.com/embedded/jetson-platform)
[![Python](https://img.shields.io/badge/python-3.6+-blue.svg)](https://www.python.org)

基于 NVIDIA Jetson 平台的摄像头采集、RTSP 推流、相机标定一体化工具包。

## 整体用途 📹

为 NVIDIA Jetson 平台提供完整的视频处理解决方案：

- **视频流处理** - CSI/USB 摄像头采集、RTSP 推流、多路显示
- **相机标定** - 单目/双目相机标定工具
- **扩展示例** - 提供 Jupyter Notebook 和 YOLO 应用示例

### 典型应用

- 视频监控推流 📡
- 多路视频拼接显示 🖼️
- AI 目标检测示例 🎯

## 功能特性

| 功能 | 说明 |
|------|------|
| **多源视频输入** 📡 | 支持 CSI 摄像头、USB 摄像头、RTSP 流、视频文件 |
| **RTSP 推流** 📡 | 将视频流通过 RTSP 协议推流输出 |
| **多摄像头布局** 🖼️ | 支持多路摄像头同时显示和布局管理 |
| **自动设备检测** 🔍 | 自动识别 CSI/USB 摄像头类型 |
| **相机标定** 📐 | 提供单目/双目相机标定功能 |
| **示例扩展** 📚 | 提供 Jupyter Notebook 和 YOLO 等应用示例 |

## 项目结构

```
utils/
├── __init__.py           # 包初始化
├── toolkit.py            # 命令行工具入口
├── camera.py             # 摄像头核心模块
├── rtsp.py               # RTSP 推流模块
├── calibrate.py          # 相机标定模块
└── examples/             # Jupyter Notebook 示例
    ├── jupyter/          # Notebook 目录
    │   ├── camera_preview.ipynb      # 单摄像头预览
    │   ├── multi_csi_preview.ipynb   # 多 CSI 预览
    │   └── yolo_rtsp.ipynb           # YOLO + RTSP 推流
    └── README.md         # 示例说明
```

## 快速开始 🚀

### 安装依赖 📦

```bash
# 基础依赖
pip install -r requirements.txt

# GStreamer (系统包)
sudo apt-get install libgstreamer1.0-dev libgstrtspserver-1.0-dev python3-gi

# YOLO 示例依赖（可选）
pip install ultralytics
```

### 基本使用

```bash
# 摄像头推流
python utils/toolkit.py stream --source=csi://0

# 多摄像头查看
python utils/toolkit.py view --sources=0,1,2

# 单目相机标定
python utils/toolkit.py calibrate mono --images-dir=./calib_images

# 双目相机标定
python utils/toolkit.py calibrate stereo --left-dir=./left --right-dir=./right

# 生成棋盘格
python utils/toolkit.py chessboard --rows=9 --cols=6
```

## 使用方法

### 1. 摄像头推流 📡

```bash
# CSI 摄像头推流
python utils/toolkit.py stream --source=csi://0

# USB 摄像头推流
python utils/toolkit.py stream --source=usb://0

# 自定义参数
python utils/toolkit.py stream \
    --source=csi://0 \
    --width=1280 --height=720 \
    --fps=30 --port=8554
```

### 2. 多摄像头查看 👁️

```bash
# 单个摄像头
python utils/toolkit.py view --sources=0

# 多个摄像头
python utils/toolkit.py view --sources=0,1,2

# 自定义分辨率
python utils/toolkit.py view --sources=0,1 --width=640 --height=480
```

### 3. 相机标定 📐

```bash
# 单目相机标定
python utils/toolkit.py calibrate mono \
    --images-dir=./calib_images \
    --rows=6 --cols=9 --square-size=20.0

# 双目相机标定
python utils/toolkit.py calibrate stereo \
    --left-dir=./left \
    --right-dir=./right \
    --rows=6 --cols=9 --square-size=20.0
```

### 4. 生成棋盘格 ♟️

```bash
# 默认参数（9x6 内角，20mm）
python utils/toolkit.py chessboard

# A4 纸格式（居中、带文字）
python utils/toolkit.py chessboard --a4 --show

# 自定义参数
python utils/toolkit.py chessboard \
    --rows=9 --cols=6 \
    --size=20 \
    --dpi=300 \
    --output=my_chessboard.png
```

## 示例代码 📚

### Jupyter Notebook 示例

所有示例都以 Jupyter Notebook 形式提供：

```bash
# 启动 Jupyter Lab
cd /home/jetson/cam
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser

# 在浏览器中打开 utils/examples/jupyter/ 目录
```

**可用的 Notebook**:

| Notebook | 功能 | 适用场景 |
|----------|------|----------|
| `camera_preview.ipynb` | 单摄像头预览 | 摄像头测试、参数调试 |
| `multi_csi_preview.ipynb` | 多 CSI 预览 | 双目调试、视频拼接 |
| `yolo_rtsp.ipynb` | YOLO + RTSP 推流 | AI 目标检测、边缘计算 |

详细说明请查看:
- [docs/JUPYTER_EXAMPLES.md](docs/JUPYTER_EXAMPLES.md) - 使用指南
- [utils/examples/README.md](utils/examples/README.md) - 示例说明

## 视频源格式

| 格式 | 说明 | 示例 |
|------|------|------|
| `usb://N` | USB 摄像头 📷 | `usb://0` |
| `csi://N` | CSI 摄像头 📹 | `csi://0` |
| `rtsp://...` | RTSP 流地址 🌐 | `rtsp://192.168.1.100:8554/stream` |
| 文件路径 | 视频文件 📁 | `/path/to/video.mp4` |
| 数字 | 自动检测类型 🔍 | `0` |

## RTSP 客户端访问

推流启动后可通过以下方式访问：

```bash
# VLC
vlc rtsp://<IP>:8554/stream

# FFplay
ffplay rtsp://<IP>:8554/stream

# OpenCV
cv2.VideoCapture('rtsp://<IP>:8554/stream')
```

## 模块 API

### camera.py

```python
from camera import VideoSource, CameraType, detect_camera_type

# 自动检测类型
video_src = VideoSource(0, 1280, 720, 30)
video_src.open()
ret, frame = video_src.read()
video_src.release()

# 指定类型
video_src = VideoSource("csi://0", 1280, 720, 30)
video_src.open()
```

### rtsp.py

```python
from rtsp import RTSPServer, CameraLayout

# RTSP 服务器
rtsp = RTSPServer(port=8554)
rtsp.add_camera_stream(CameraType.CSI, 0, 1280, 720, 30)
rtsp.start()

# 多摄像头布局
layout = CameraLayout(num_cameras=4, cell_size=(640, 480))
canvas = layout.arrange(frames)
```

### calibrate.py

```python
from calibrate import CheckerboardConfig, MonoCalibrator, save_calibration_to_yaml

# 单目标定
config = CheckerboardConfig(rows=6, cols=9, square_size_mm=20.0)
calibrator = MonoCalibrator(config)
calibrator.add_images_from_dir("./calib_images")
result = calibrator.calibrate((640, 480))
save_calibration_to_yaml(result, "mono_calibration.yaml")
```

## 性能参考 ⚡

| 设备 | 模型 | 分辨率 | FPS |
|------|------|--------|-----|
| Jetson Nano | YOLOv8n | 640x480 | ~15 |
| Jetson Xavier NX | YOLOv8n | 640x480 | ~30 |
| Jetson Orin | YOLOv8n | 640x480 | ~60+ |

*注：使用 TensorRT 引擎可显著提升性能*

## 系统要求

- **硬件**: NVIDIA Jetson 平台 (Jetson Nano/Xavier NX/Orin) 🎮
- **系统**: JetPack 4.6 或更高版本
- **Python**: 3.6+ 🐍
- **GStreamer**: 1.0

## 许可证

Apache License 2.0
