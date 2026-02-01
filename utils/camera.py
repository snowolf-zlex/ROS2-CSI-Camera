#!/usr/bin/env python3
"""
Camera Module - 摄像头核心组件

提供可复用的摄像头相关类和函数，供 toolkit、标定工具等使用。
"""

import logging
import subprocess
import os
from enum import Enum

logger = logging.getLogger(__name__)


# =============================================================================
# 工具函数
# =============================================================================

def setup_logging(level: str = "INFO"):
    """设置日志"""
    logging.basicConfig(
        level=getattr(logging, level.upper()),
        format='[%(asctime)s] [%(levelname)s] %(message)s'
    )
    # 抑制 ultralytics 的日志
    try:
        logging.getLogger('ultralytics').setLevel(logging.WARNING)
    except:
        pass


def get_ips():
    """获取 IP 地址"""
    import socket
    import requests

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
    except Exception as e:
        logger.debug(f"获取本地 IP 失败: {e}")
        local_ip = "127.0.0.1"

    try:
        public = requests.get('https://api.ipify.org', timeout=3).text
    except Exception as e:
        logger.debug(f"获取公网 IP 失败: {e}")
        public = None

    return local_ip, public


def draw_text(img, text, pos, color=(0, 255, 0), bg_color=(0, 0, 0)):
    """绘制带背景的文字"""
    import cv2
    x, y = pos
    font = cv2.FONT_HERSHEY_SIMPLEX
    (w, h), _ = cv2.getTextSize(text, font, 0.5, 1)
    cv2.rectangle(img, (x - 3, y - h - 3), (x + w + 3, y + 3), bg_color, -1)
    cv2.putText(img, text, (x, y), font, 0.5, color, 1)
    return img


# =============================================================================
# 摄像头类型
# =============================================================================

class CameraType(Enum):
    """摄像头类型枚举"""
    CSI = "csi"
    USB = "usb"


def detect_camera_type(device_id: int):
    """
    检测摄像头类型

    Args:
        device_id: 摄像头设备 ID (如 0, 1, 2)

    Returns:
        CameraType.CSI 或 CameraType.NONE，检测失败返回 None
    """
    device_path = f"/dev/video{device_id}"
    if not os.path.exists(device_path):
        return None

    try:
        cmd = f'v4l2-ctl --device={device_path} --all | grep "Card type"'
        output = subprocess.check_output(cmd, shell=True, text=True).strip()
        # 如果输出中不包含 "USB Camera"，则认为是 CSI 摄像头
        return CameraType.CSI if "USB Camera" not in output else CameraType.USB
    except Exception as e:
        logger.debug(f"检测摄像头类型失败: {e}")
        return None


def build_csi_pipeline(sensor_id: int, width: int, height: int, fps: int) -> str:
    """
    构建 CSI 摄像头 GStreamer 管道

    Args:
        sensor_id: CSI 传感器 ID
        width: 图像宽度
        height: 图像高度
        fps: 帧率

    Returns:
        GStreamer 管道字符串
    """
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, "
        f"format=(string)NV12, framerate=(fraction){fps}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw, width=(int){width}, height=(int){height}, "
        f"format=(string)BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=(string)BGR ! "
        f"appsink"
    )


# =============================================================================
# 视频源类
# =============================================================================

class VideoSource:
    """
    统一视频源类

    支持 CSI 摄像头、USB 摄像头、RTSP 流、视频文件
    """

    def __init__(self, source, width: int = 1280, height: int = 720, fps: int = 30):
        """
        初始化视频源

        Args:
            source: 视频源，可以是:
                - str: "csi://0", "usb://0", "rtsp://...", "/path/to/video.mp4"
                - int: 0, 1, 2 (自动检测类型)
            width: 图像宽度
            height: 图像高度
            fps: 帧率
        """
        import cv2

        self.source = source
        self.width = width
        self.height = height
        self.fps = fps
        self.cap = None
        self._type = None
        self._cv2 = cv2

    def open(self) -> bool:
        """
        打开视频源

        Returns:
            bool: 成功返回 True，失败返回 False
        """
        import cv2

        if isinstance(self.source, str):
            if self.source.startswith('csi://'):
                device_id = int(self.source.split('://')[1])
                pipeline = build_csi_pipeline(device_id, self.width, self.height, self.fps)
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                self._type = CameraType.CSI

            elif self.source.startswith('usb://'):
                device_id = int(self.source.split('://')[1])
                self.cap = cv2.VideoCapture(device_id)
                self._type = CameraType.USB

            elif self.source.startswith('rtsp://'):
                self.cap = cv2.VideoCapture(self.source)

            elif self.source.lower().endswith(('.mp4', '.avi', '.mov', '.mkv')):
                self.cap = cv2.VideoCapture(self.source)

        elif isinstance(self.source, int):
            camera_type = detect_camera_type(self.source)
            if camera_type is None:
                logger.error(f"无法检测摄像头类型: {self.source}")
                return False

            if camera_type == CameraType.CSI:
                pipeline = build_csi_pipeline(self.source, self.width, self.height, self.fps)
                self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                self._type = CameraType.CSI
            else:
                self.cap = cv2.VideoCapture(self.source)
                self._type = CameraType.USB

        if self.cap is None or not self.cap.isOpened():
            logger.error(f"无法打开视频源: {self.source}")
            return False

        # CSI 摄像头通过管道设置分辨率，不需要额外设置
        if self._type != CameraType.CSI:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"视频源已打开: {self.source} -> {actual_w}x{actual_h}")
        return True

    def read(self):
        """
        读取一帧

        Returns:
            tuple: (ret, frame) - ret 表示是否成功，frame 是图像数据
        """
        return self.cap.read() if self.cap else (False, None)

    def release(self):
        """释放资源"""
        if self.cap:
            self.cap.release()
            self.cap = None

    def __enter__(self):
        """支持 with 语句"""
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """支持 with 语句"""
        self.release()
