# Jupyter Notebook 使用指南 📓

本项目提供 Jupyter Notebook 用于交互式摄像头测试、开发和调试。

## 为什么使用 Jupyter Notebook？

Jupyter Notebook 适合以下场景：

- **交互式开发** - 逐步测试代码，即时查看结果
- **可视化调试** - 实时查看摄像头画面，调整参数
- **快速原型** - 在部署前验证算法效果
- **学习演示** - 直观展示摄像头 API 的使用方法

## 前置准备

### 1. 安装 Jupyter

```bash
# 安装 Jupyter Lab 和相关依赖
pip install jupyter jupyterlab ipywidgets

# 启用 ipywidgets 扩展（用于交互控件）
jupyter nbextension enable --py widgetsnbextension
# Jupyter Lab 自动支持 ipywidgets，无需额外配置
```

### 2. 检查摄像头

在启动 Jupyter 之前，先确认摄像头工作正常：

```bash
# 查看可用的摄像头设备
ls /dev/video*

# 查看摄像头详细信息
v4l2-ctl --device=/dev/video0 --info
```

输出示例：
```
Driver Info:
 Driver name      : tegra-video
 Card type        : vi-output, imx219 10-0010  ← imx219 是 CSI 摄像头
```

### 3. 验证 GStreamer

确保 GStreamer 管道可以正常工作：

```bash
# 使用 nvgstcapture 测试 CSI 摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=0
```

## 启动 Jupyter

### 方法一：Jupyter Lab（推荐）

Jupyter Lab 提供更现代的界面和更好的 Notebook 支持。

```bash
cd /home/jetson/cam

# 启动 Jupyter Lab
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser

# 或指定工作目录
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser --notebook-dir=/home/jetson/cam/utils/examples/jupyter
```

启动后会显示类似以下信息：
```
[I 2024-02-01 10:00:00.123 ServerApp] Jupyter Lab extension loaded from /usr/local/lib/python3.8/dist-packages/jupyterlab
[I 2024-02-01 10:00:00.456 ServerApp] Serving notebooks from local directory: /home/jetson/cam
[I 2024-02-01 10:00:00.456 ServerApp] Jupyter Server 1.0.0 running at http://0.0.0.0:8888/
[I 2024-02-01 10:00:00.456 ServerApp]  or http://127.0.0.1:8888/
[I 2024-02-01 10:00:00.456 ServerApp] Use Control-C to stop this server and shut down all kernels...
```

在浏览器中访问显示的 URL（包含 token 的地址）。

### 方法二：Jupyter Notebook

```bash
cd /home/jetson/cam

# 启动 Jupyter Notebook
jupyter notebook --ip=0.0.0.0 --port=8888 --no-browser
```

### 远程访问

如果从其他设备访问 Jetson：

```bash
# 1. 在 Jetson 上启动 Jupyter
jupyter lab --ip=0.0.0.0 --port=8888 --no-browser

# 2. 在本地电脑上建立 SSH 隧道
ssh -L 8888:localhost:8888 jetson@<jetson_ip>

# 3. 在本地浏览器访问
# http://localhost:8888/
```

## Notebook 文件

### 1. 单摄像头预览 (camera.ipynb)

**用途**: 在 Jupyter 中实时预览单个 CSI/USB 摄像头的画面。

**适用场景**:
- 验证摄像头连接是否正常
- 测试不同分辨率和帧率
- 开发和调试图像处理算法

**代码结构说明**:

| 单元格 | 功能 | 说明 |
|--------|------|------|
| 导入库 | 导入必要的 Python 库 | cv2, threading, ipywidgets, IPython |
| UI 配置 | 创建界面控件 | 图像显示框、退出按钮 |
| 摄像头配置 | 设置摄像头参数 | CSI_SENSOR_ID, WIDTH, HEIGHT, FPS |
| 图像捕获 | 启动捕获线程 | 读取摄像头帧并更新到界面 |

**关键参数**:

```python
# 摄像头 ID
# 0, 1, 2... 对应 /dev/video0, /dev/video1, /dev/video2
CSI_SENSOR_ID = 0

# 输出分辨率
WIDTH = 640   # 常见值: 640, 1280, 1920
HEIGHT = 480  # 常见值: 480, 720, 1080

# 帧率
FPS = 30      # 常见值: 15, 30, 60
```

**如何使用**:

1. 打开 `camera.ipynb`
2. 修改 `CSI_SENSOR_ID` 为你的摄像头 ID
3. 依次运行每个单元格（Shift + Enter）
4. 点击 "Exit" 按钮停止预览

### 2. 多 CSI 摄像头预览 (multi_csi_cam.ipynb)

**用途**: 同时预览两个 CSI 摄像头的画面，并支持实时切换分辨率。

**适用场景**:
- 双目相机调试
- 多路视频拼接测试
- 立体视觉开发

**代码结构说明**:

| 单元格 | 功能 | 说明 |
|--------|------|------|
| 导入库 | 导入必要的库 | 添加了 numpy 用于图像拼接 |
| 摄像头配置 | 配置两个摄像头 | CSI_SENSOR_LEFT, CSI_SENSOR_RIGHT |
| 分辨率选项 | 定义支持的分辨率 | 640x480, 1280x720, 1920x1080 |
| UI 配置 | 创建界面控件 | 图像显示框、退出按钮、分辨率选择器 |
| 分辨率选择器 | 添加分辨率下拉框 | 使用 ipywidgets.interact |
| 图像捕获 | 启动双摄像头捕获 | 水平拼接两路画面 |

**关键参数**:

```python
# 左摄像头 ID
CSI_SENSOR_LEFT = 0   # 对应 /dev/video0

# 右摄像头 ID
CSI_SENSOR_RIGHT = 1  # 对应 /dev/video1

# 分辨率选项
resolution_options = {
    "640x480": (640, 480),
    "1280x720": (1280, 720),
    "1920x1080": (1920, 1080),
}
```

**如何使用**:

1. 打开 `multi_csi_cam.ipynb`
2. 确认两个摄像头的 ID 配置正确
3. 依次运行每个单元格
4. 使用下拉框切换分辨率
5. 点击 "Exit" 按钮停止预览

### 3. YOLO + RTSP 推流 (yolo_rtsp.ipynb)

在 Jupyter 中运行 YOLO 目标检测，并将结果通过 RTSP 推流输出。

**用途**:
- YOLO 目标检测 + RTSP 推流
- 实时检测和可视化
- 适合学习和测试 AI 推理性能

**适用场景**:
- 智能监控开发
- AI 模型测试和验证
- 边缘计算原型开发

**代码结构说明**:

| 单元格 | 功能 | 说明 |
|--------|------|------|
| 导入库 | 导入必要的库和 toolkit 模块 | sys, os, camera, rtsp |
| 配置参数 | 设置所有可调参数 | 视频源、模型、分辨率、端口等 |
| 初始化日志 | 设置日志级别 | setup_logging |
| 打开视频源 | 创建并打开 VideoSource | 支持自动检测摄像头类型 |
| 创建处理器 | 初始化 YOLOProcessor | 加载 YOLO 模型 |
| 创建 RTSP 服务器 | 创建并配置 RTSPServer | 设置推流参数 |
| 显示服务信息 | 打印访问地址和配置 | 显示 IP、端口、分辨率等 |
| 启动处理器 | 启动 YOLO 处理线程 | 开始捕获和检测 |
| 启动 RTSP 服务 | 阻塞运行 RTSP 服务 | 按 Ctrl+C 或停止按钮中断 |
| 清理资源 | 释放摄像头和处理器 | finally 块确保清理 |

**关键参数**:

```python
# 视频源配置
SOURCE = "csi://0"              # 可选: "csi://0", "usb://0", 0 (自动检测)

# YOLO 模型配置
MODEL_PATH = "best.engine"     # 可选: "yolov8n.pt", "best.engine"
                              # .pt - PyTorch 模型（兼容性好）
                              # .engine - TensorRT 引擎（Jetson 推荐）

# 输出配置
WIDTH = 640                     # 输出宽度
HEIGHT = 480                    # 输出高度
FPS = 30                        # 帧率

# 性能调优
INTERVAL = 1                    # 处理间隔
                              # 1 = 每帧都检测（准确）
                              # 2 = 每2帧检测一次（性能优化）

# RTSP 配置
RTSP_PORT = 8554               # RTSP 推流端口
```

**如何使用**:

1. **安装依赖**:
   ```bash
   pip install ultralytics
   ```

2. **下载 YOLO 模型** (可选):
   ```bash
   # 下载预训练模型
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

   # 或导出 TensorRT 引擎（Jetson 推荐以提升性能）
   yolo export model=yolov8n.pt format=engine half=True
   ```

3. **打开 Notebook**: 在 Jupyter Lab 中打开 `yolo_rtsp.ipynb`

4. **修改配置**: 在"配置参数"单元格中修改：
   - `SOURCE` - 视频源
   - `MODEL_PATH` - 模型路径
   - 其他参数按需调整

5. **依次运行所有单元格**: 使用 Shift + Enter 逐个执行

6. **查看服务信息**: 运行后会在输出中显示 RTSP 访问地址

7. **使用 RTSP 客户端访问**:
   ```bash
   # VLC
   vlc rtsp://<IP>:8554/stream

   # FFplay
   ffplay rtsp://<IP>:8554/stream
   ```

8. **停止服务**:
   - 点击 Jupyter 工具栏的"■"停止按钮
   - 或选择菜单 "Kernel" → "Interrupt"

**注意事项**:

- **RTSP 服务会阻塞运行** - 启动后该单元格会一直运行，直到手动停止
- **停止后需重新初始化** - 如果要再次运行，需要重新执行"打开视频源"及之后的单元格
- **模型首次加载较慢** - YOLO 模型首次加载需要几秒到几十秒，之后会快很多

**性能参考** (Jetson 设备):

| 设备 | 模型 | 分辨率 | INTERVAL | FPS |
|------|------|--------|----------|-----|
| Jetson Nano | YOLOv8n | 640x480 | 1 | ~10-15 |
| Jetson Nano | YOLOv8n | 640x480 | 2 | ~15-20 |
| Xavier NX | YOLOv8n | 640x480 | 1 | ~25-30 |
| Orin | YOLOv8n | 640x480 | 1 | ~50+ |

*使用 TensorRT 引擎 (.engine) 可获得更好性能*

## 工具模块说明

Notebook 使用以下工具模块：

### camera.py - 摄像头模块

```python
from camera import VideoSource, build_csi_pipeline

# 创建视频源
video_src = VideoSource("csi://0", 640, 480, 30)

# 打开摄像头
video_src.open()

# 读取帧
ret, frame = video_src.read()

# 释放资源
video_src.release()
```

### rtsp.py - RTSP 推流模块

```python
from rtsp import CameraLayout

# 创建布局（用于多摄像头）
layout = CameraLayout(num_cameras=2, cell_size=(640, 480))

# 排列帧
canvas = layout.arrange([frame1, frame2])
```

## 路径导入说明

由于 Notebook 位于 `utils/examples/jupyter/` 目录，需要正确设置导入路径：

```python
# 获取项目根目录的路径
import sys
import os
# 当前目录: /home/jetson/cam/utils/examples/jupyter
# 父目录: /home/jetson/cam/utils/examples
# 祖父目录: /home/jetson/cam/utils ← 这里是我们要的

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath("."))))

# 现在可以导入 utils 目录下的模块
from camera import VideoSource
from rtsp import CameraLayout
```

## 常见问题

### 问题 1: 无法打开摄像头

**症状**: 运行后显示 "无法打开摄像头" 或没有任何画面

**解决方法**:

1. 检查摄像头设备是否存在：
```bash
ls -la /dev/video*
```

2. 检查摄像头是否被其他程序占用：
```bash
# 查看哪个进程在使用摄像头
sudo lsof /dev/video0
```

3. 确认 CSI 摄像头驱动正常：
```bash
v4l2-ctl --device=/dev/video0 --all
```

### 问题 2: 画面显示黑屏或花屏

**症状**: 图像显示控件是黑色或颜色异常

**解决方法**:

1. 尝试不同的分辨率
2. 检查 GStreamer 版本兼容性
3. 使用 `nvgstcapture-1.0` 确认硬件正常

### 问题 3: 控件不显示或无法交互

**症状**: 按钮或下拉框不显示

**解决方法**:

```bash
# 重新安装 ipywidgets
pip uninstall ipywidgets widgetsnbextension
pip install ipywidgets

# Jupyter Lab 无需额外配置
# Jupyter Notebook 需要启用扩展
jupyter nbextension enable --py widgetsnbextension
```

### 问题 4: ImportError: No module named 'camera'

**症状**: 导入模块失败

**解决方法**:

确认路径设置正确，可以打印验证：

```python
import sys
import os

# 打印当前工作目录
print("当前目录:", os.getcwd())

# 打印 sys.path
print("Python 路径:", sys.path)

# 手动添加路径
sys.path.insert(0, "/home/jetson/cam/utils")
```

## 扩展开发

### 添加 FPS 显示

```python
import time

# 在捕获函数中添加
class CameraWithFPS:
    def __init__(self):
        self.frame_count = 0
        self.start_time = time.time()

    def get_fps(self):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        if elapsed > 1.0:
            fps = self.frame_count / elapsed
            self.frame_count = 0
            self.start_time = time.time()
            return fps
        return None

# 使用
fps_counter = CameraWithFPS()
# 在循环中
fps = fps_counter.get_fps()
if fps:
    print(f"FPS: {fps:.1f}")
```

### 添加图像保存

```python
import cv2
from datetime import datetime

def save_frame(frame, prefix="capture"):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    print(f"已保存: {filename}")

# 在捕获循环中
if save_button_clicked:  # 需要添加保存按钮
    save_frame(frame)
```

### 添加 YOLO 检测

```python
from ultralytics import YOLO

# 加载模型（在单独的单元格中）
model = YOLO("yolov8n.pt")

# 在捕获函数中处理帧
def process_with_yolo(frame):
    results = model(frame)
    return results[0].plot()

# 在显示前调用
frame = process_with_yolo(frame)
imgbox.value = cv2.imencode(".jpg", frame)[1].tobytes()
```

### 添加参数调节控件

```python
from ipywidgets import IntSlider, FloatSlider

# 创建滑动条
brightness_slider = IntSlider(
    value=0, min=-100, max=100, step=1,
    description='亮度:'
)

contrast_slider = FloatSlider(
    value=1.0, min=0.5, max=2.0, step=0.1,
    description='对比度:'
)

# 在图像处理中应用
def apply_adjustments(frame, brightness, contrast):
    return cv2.convertScaleAbs(frame, alpha=contrast, beta=brightness)

# 使用 interact 连接控件
def update_frame(brightness, contrast):
    # 获取新帧并应用调整
    frame = get_frame()
    adjusted = apply_adjustments(frame, brightness, contrast)
    imgbox.value = cv2.imencode(".jpg", adjusted)[1].tobytes()

interact(update_frame,
         brightness=brightness_slider,
         contrast=contrast_slider)
```

## 性能优化建议

1. **降低分辨率** - 如果不需要高分辨率，使用 640x480 可显著提高性能
2. **跳帧显示** - 每秒只更新 15-30 帧到界面，内部可以更高
3. **使用 GPU 加速** - 图像处理使用 cv2.cuda 模块（如果支持）
4. **异步处理** - 耗时操作放在单独的线程中

## 相关链接

- [Jupyter Lab 官方文档](https://jupyterlab.readthedocs.io/)
- [ipywidgets 文档](https://ipywidgets.readthedocs.io/)
- [OpenCV Python 教程](https://docs.opencv.org/master/d6/d00/tutorial_py_root.html)
- [GStreamer 文档](https://gstreamer.freedesktop.org/documentation/)
