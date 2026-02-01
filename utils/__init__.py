"""
Jetson Camera Toolkit - 工具包

提供摄像头采集、RTSP 推流、相机标定等功能。
"""

__version__ = "1.0.0"

from .camera import (
    VideoSource,
    CameraType,
    detect_camera_type,
    setup_logging,
    get_ips,
    draw_text,
)

from .rtsp import (
    RTSPServer,
    CameraLayout,
)

from .calibrate import (
    CheckerboardConfig,
    MonoCalibrator,
    StereoCalibrator,
    save_calibration_to_yaml,
    load_calibration_from_yaml,
)

__all__ = [
    # Camera
    "VideoSource",
    "CameraType",
    "detect_camera_type",
    "setup_logging",
    "get_ips",
    "draw_text",
    # RTSP
    "RTSPServer",
    "CameraLayout",
    # Calibration
    "CheckerboardConfig",
    "MonoCalibrator",
    "StereoCalibrator",
    "save_calibration_to_yaml",
    "load_calibration_from_yaml",
]
