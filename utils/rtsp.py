#!/usr/bin/env python3
"""
RTSP Module - RTSP 推流组件

提供 RTSP 推流相关的类和函数。
"""

import logging
import numpy as np

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

from camera import CameraType

Gst.init(None)

logger = logging.getLogger(__name__)


# =============================================================================
# 摄像头布局管理
# =============================================================================

class CameraLayout:
    """
    摄像头布局管理

    将多个摄像头的画面排列到一个画布上
    """

    def __init__(self, num_cameras, cell_size=(640, 480), max_width=1920):
        """
        初始化布局

        Args:
            num_cameras: 摄像头数量
            cell_size: 每个格子的尺寸 (width, height)
            max_width: 画布最大宽度
        """
        self.cell_w, self.cell_h = cell_size
        max_cols = max(1, max_width // self.cell_w)
        self.rows = (num_cameras + max_cols - 1) // max_cols
        self.cols = min(num_cameras, max_cols)

    def arrange(self, frames):
        """
        排列帧到画布

        Args:
            frames: 帧列表

        Returns:
            np.ndarray: 排列后的画布
        """
        canvas = np.zeros((self.cell_h * self.rows, self.cell_w * self.cols, 3), dtype=np.uint8)
        for i, frame in enumerate(frames[:self.rows * self.cols]):
            r, c = i // self.cols, i % self.cols
            frame = self._resize_frame(frame, (self.cell_w, self.cell_h))
            y1, y2 = r * self.cell_h, (r + 1) * self.cell_h
            x1, x2 = c * self.cell_w, (c + 1) * self.cell_w
            canvas[y1:y2, x1:x2] = frame
        return canvas

    @staticmethod
    def _resize_frame(frame, size):
        """调整帧大小"""
        import cv2
        if frame is None:
            return np.zeros((size[1], size[0], 3), dtype=np.uint8)
        return cv2.resize(frame, size)


# =============================================================================
# RTSP 服务器
# =============================================================================

class RTSPServer(GstRtspServer.RTSPServer):
    """
    RTSP 服务器

    支持两种推流方式:
    1. 直接摄像头推流 (add_camera_stream)
    2. 应用程序处理后推流 (add_app_stream)
    """

    def __init__(self, port: int = 8554):
        """
        初始化 RTSP 服务器

        Args:
            port: RTSP 服务端口
        """
        super().__init__()
        self.port = port
        self.set_service(str(port))

    def add_camera_stream(self, camera_type: CameraType, source: int,
                          width: int, height: int, fps: int, path: str = "/stream"):
        """
        添加摄像头直接推流

        Args:
            camera_type: 摄像头类型 (CameraType.CSI 或 CameraType.USB)
            source: 摄像头设备 ID
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            path: RTSP 路径
        """
        if camera_type == CameraType.CSI:
            launch = (
                f'nvarguscamerasrc sensor-id={source} ! '
                f'video/x-raw(memory:NVMM), width={width}, height={height}, '
                f'framerate={fps}/1 ! nvv4l2h264enc ! rtph264pay name=pay0 pt=96'
            )
        else:
            launch = (
                f'v4l2src device=/dev/video{source} ! '
                f'videoconvert ! video/x-raw, format=I420 ! '
                f'x264enc tune=zerolatency ! rtph264pay name=pay0 pt=96'
            )

        factory = GstRtspServer.RTSPMediaFactory()
        factory.set_launch(launch)
        factory.set_shared(True)
        self.get_mount_points().add_factory(path, factory)

    def add_app_stream(self, frame_provider, width: int, height: int, fps: int, path: str = "/stream"):
        """
        添加应用程序处理后推流

        用于 YOLO 等处理后的视频推流

        Args:
            frame_provider: 帧提供函数，返回 numpy.ndarray 格式的图像
            width: 图像宽度
            height: 图像高度
            fps: 帧率
            path: RTSP 路径
        """
        factory = self._AppFactory(frame_provider, width, height, fps)
        factory.set_shared(True)
        self.get_mount_points().add_factory(path, factory)

    def start(self):
        """启动 RTSP 服务（阻塞运行）"""
        self.attach(None)
        logger.info(f"RTSP 服务已启动: 端口={self.port}")
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            logger.info("RTSP 服务已停止")
            loop.quit()

    class _AppFactory(GstRtspServer.RTSPMediaFactory):
        """应用程序推流工厂（内部类）"""

        def __init__(self, frame_provider, width, height, fps):
            super().__init__()
            self.provider = frame_provider
            self.width, self.height = width, height
            self.fps = fps
            self.duration = int(1e9 / fps)
            self.frame_count = 0

        def do_create_element(self, url):
            launch = (
                'appsrc name=src is-live=true block=true format=TIME '
                f'caps=video/x-raw,format=BGR,width={self.width},height={self.height},framerate={self.fps}/1 '
                '! videoconvert ! video/x-raw,format=I420 '
                '! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 '
                '! nvv4l2h264enc insert-sps-pps=true bitrate=4000000 '
                '! rtph264pay config-interval=1 pt=96 name=pay0'
            )
            return Gst.parse_launch(launch)

        def do_configure(self, media):
            appsrc = media.get_element().get_child_by_name("src")
            appsrc.connect("need-data", self._on_need_data)

        def _on_need_data(self, src, length):
            frame = self.provider()
            if frame is not None:
                data = frame.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.duration = self.duration
                timestamp = self.frame_count * self.duration
                buf.pts = buf.dts = timestamp
                buf.offset = timestamp
                self.frame_count += 1
                src.emit("push-buffer", buf)
