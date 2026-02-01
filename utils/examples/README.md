# Examples 📚

本目录包含 Jupyter Notebook 示例，展示如何使用 toolkit 模块的各种功能。

## 目录结构

```
jupyter/
├── camera_preview.ipynb      # 单摄像头预览
├── multi_csi_preview.ipynb   # 多 CSI 摄像头预览
└── yolo_rtsp.ipynb           # YOLO + RTSP 推流
```

## Jupyter Notebook 示例

### 1. 单摄像头预览 (camera_preview.ipynb)

在 Jupyter 中实时预览单个 CSI/USB 摄像头的画面。

**功能:**
- 实时摄像头画面显示
- 退出按钮控制
- 使用 `camera.VideoSource` 类

**配置:**
```python
CSI_SENSOR_ID = 0  # 修改为你的 CSI 摄像头 ID
WIDTH = 640
HEIGHT = 480
FPS = 30
```

**如何使用:**
1. 启动 Jupyter: `cd /home/jetson/cam && jupyter lab`
2. 打开 `utils/examples/jupyter/camera_preview.ipynb`
3. 修改 `CSI_SENSOR_ID` 为你的摄像头 ID
4. 依次运行每个单元格（Shift + Enter）
5. 点击 "Exit" 按钮停止预览

---

### 2. 多 CSI 摄像头预览 (multi_csi_preview.ipynb)

同时预览多个 CSI 摄像头的画面，并支持分辨率切换。

**功能:**
- 双路摄像头水平拼接显示
- 实时分辨率切换 (640x480, 1280x720, 1920x1080)
- 退出按钮控制

**配置:**
```python
CSI_SENSOR_LEFT = 0   # 左摄像头 ID
CSI_SENSOR_RIGHT = 1  # 右摄像头 ID
```

**如何使用:**
1. 打开 `utils/examples/jupyter/multi_csi_preview.ipynb`
2. 确认两个摄像头的 ID 配置正确
3. 依次运行每个单元格
4. 使用下拉框切换分辨率
5. 点击 "Exit" 按钮停止预览

---

### 3. YOLO + RTSP 推流 (yolo_rtsp.ipynb)

使用 YOLO 进行目标检测，并将结果通过 RTSP 推流输出。

**功能:**
- 从 CSI/USB 摄像头获取视频
- 使用 YOLO 进行实时目标检测
- 将检测结果通过 RTSP 推流
- 显示 FPS 和访问地址

**配置:**
```python
SOURCE = "csi://0"              # 视频源
MODEL_PATH = "best.engine"     # YOLO 模型
WIDTH = 640                     # 输出宽度
HEIGHT = 480                    # 输出高度
FPS = 30                        # 帧率
INTERVAL = 1                    # 处理间隔（每N帧处理一次）
RTSP_PORT = 8554               # RTSP 端口
```

**依赖:**
```bash
pip install ultralytics
```

**如何使用:**
1. 安装 YOLO 依赖: `pip install ultralytics`
2. 下载 YOLO 模型（如需要）:
   ```bash
   # 下载 PyTorch 模型
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

   # 或导出 TensorRT 引擎（Jetson 推荐）
   yolo export model=yolov8n.pt format=engine half=True
   ```
3. 打开 `utils/examples/jupyter/yolo_rtsp.ipynb`
4. 修改配置区域的参数
5. 依次运行每个单元格
6. 使用 VLC/FFplay 访问显示的 RTSP 地址
7. 停止服务：点击 Jupyter 工具栏的 "停止" 按钮

**访问方式:**
```bash
# VLC
vlc rtsp://<IP>:8554/stream

# FFplay
ffplay rtsp://<IP>:8554/stream

# OpenCV
import cv2
cap = cv2.VideoCapture('rtsp://<IP>:8554/stream')
```

---

## 模块导入

所有 Notebook 都需要先设置路径才能导入 toolkit 模块：

```python
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath("."))))

from camera import VideoSource, CameraType, setup_logging, get_ips
from rtsp import RTSPServer
```

---

## 启动 Jupyter

```bash
cd /home/jetson/cam

# 启动 Jupyter Lab（推荐）
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser

# 或启动 Jupyter Notebook
jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser
```

然后在浏览器中访问显示的 URL（包含 token）。

---

## 为什么使用 Jupyter Notebook？

| 优势 | 说明 |
|------|------|
| **交互式开发** | 逐步测试代码，即时查看结果 |
| **可视化调试** | 实时查看摄像头画面，调整参数 |
| **快速原型** | 在部署前验证算法效果 |
| **学习演示** | 直观展示 API 使用方法 |
| **内嵌图像** | 直接在 Notebook 中查看检测结果 |

---

## 常见问题

### 无法打开摄像头

```bash
# 查看可用摄像头
ls /dev/video*

# 查看摄像头信息
v4l2-ctl --device=/dev/video0 --info
```

### Jupyter 控件不显示

```bash
# 重新安装 ipywidgets
pip install --upgrade ipywidgets
```

### 模型加载失败

```bash
# 安装 ultralytics
pip install ultralytics

# 下载 YOLO 模型
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

---

## 扩展

你可以在这些 Notebook 基础上添加更多功能：

- 添加其他目标检测算法（如 SSD、CenterNet）
- 实现图像录制功能
- 添加更多参数调节控件
- 显示详细的检测统计信息

详细的使用指南请查看: [docs/JUPYTER_EXAMPLES.md](../../docs/JUPYTER_EXAMPLES.md)
