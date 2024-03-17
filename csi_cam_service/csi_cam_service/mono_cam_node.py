#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CSI单目摄像头图像处理模块
默认参数：
    video_device_id:=0
    image_size:=[640,480]
    fps:=30
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
import cv2
import numpy as np

from ruamel.yaml import YAML


class MonocularCameraNode(Node):
    def __init__(self):
        super().__init__("sigle_csi_cam_node")
        self.declare_parameter("video_device_id", 0)  # 声明和初始化参数
        self.declare_parameter("image_size", [640, 480])
        self.declare_parameter("fps", 30)
        self.declare_parameter("calibration_file_path", None)  # 相机校准数据文件

        self.video_device_id = self.get_parameter("video_device_id").value
        self.width, self.height = self.get_parameter("image_size").value
        self.fps = self.get_parameter("fps").value

        self.calibration_file_path = self.get_parameter("calibration_file_path").value
        self.calibration_data = self.load_camera_info()

        self.get_logger().info(f"Video Device ID: {self.video_device_id}")
        self.get_logger().info(f"Video Width: {self.width}")
        self.get_logger().info(f"Video Height: {self.height}")
        self.get_logger().info(f"Video FPS: {self.fps}")
        self.get_logger().info(f"Calibration File Path: {self.calibration_file_path}")

        self.publisher_raw = self.create_publisher(
            Image, "/mono_cam/image_raw", 10
        )  # 创建原始图像发布者
        self.publisher_compressed = self.create_publisher(
            Image, "/mono_cam/image_compressed", 10
        )  # 创建压缩图像发布者

        self.publisher_camera_info = self.create_publisher(
            CameraInfo, "/mono_cam/camera_info", 10
        )

        self.bridge = CvBridge()  # 初始化CvBridge
        self.capture = self.init_csi_caputre(
            self.video_device_id, self.width, self.height, self.fps
        )  # 初始化视频捕获对象
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open video device")
            raise RuntimeError("Failed to open video device")

    def capture_image(self):
        ret, frame = self.capture.read()  # 从指定视频设备捕获帧
        if ret:
            # 相机校准
            frame = self.calibrate_camera(frame, self.calibration_data)
            image_msg = self.bridge.cv2_to_imgmsg(
                frame, encoding="bgr8"
            )  # 将OpenCV图像转换为ROS图像消息
            self.publisher_raw.publish(image_msg)  # 发布原始图像消息

            # 压缩原始图像
            compressed_frame = self.compress_image(frame)
            compressed_image_msg = self.bridge.cv2_to_imgmsg(
                compressed_frame, encoding="bgr8"
            )  # 将OpenCV图像转换为ROS图像消息
            self.publisher_compressed.publish(compressed_image_msg)  # 发布压缩图像消息

            self.get_logger().debug("Image captured and published")  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture image")  # 输出错误日志信息

    def load_camera_info(self):
        camera_info_msg = None
        if self.calibration_file_path is not None:
            yaml_data = self.load_camera_calibration_from_yaml(
                self.calibration_file_path
            )
            camera_info_msg = CameraInfo()
            camera_info_msg.height = yaml_data["image_height"]
            camera_info_msg.width = yaml_data["image_width"]
            camera_info_msg.header.frame_id = yaml_data["camera_name"]
            camera_info_msg.distortion_model = yaml_data["distortion_model"]
            camera_info_msg.d = yaml_data["distortion_coefficients"]["data"]  # 畸变系数
            camera_info_msg.k = yaml_data["camera_matrix"]["data"]  # 内参矩阵
            camera_info_msg.r = yaml_data["rectification_matrix"]["data"]  # 矫正矩阵
            camera_info_msg.p = yaml_data["projection_matrix"]["data"]  # 投影矩阵

        return camera_info_msg

    def publish_camera_calibration(self):
        if self.calibration_data:

            # Publish camera calibration parameters
            self.publisher_camera_info.publish(self.calibration_data)

    def build_pipeline(self, sensor_id, width=640, height=480, framerate=30):
        """
        构建GStreamer管道以从指定的传感器ID捕获图像。

        参数：
        - sensor_id：传感器ID
        - width：图像宽度，默认为640
        - height：图像高度，默认为480
        - framerate：帧率，默认为30

        返回：
        - pipeline：GStreamer管道字符串
        """
        pipeline = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
            "nvvidconv ! "
            "video/x-raw, format=(string)BGRx ! "
            "videoconvert ! "
            "appsink"
        )
        return pipeline

    def init_csi_caputre(self, video_device_id, width=640, height=480, framerate=30):
        """
        初始化CSI单目摄像头

        参数：
        - video_device_id：摄像头设备ID
        - width：图像宽度，默认为640
        - height：图像高度，默认为480
        - framerate：帧率，默认为30

        返回：
        - cap：视频捕获对象
        """
        return cv2.VideoCapture(
            self.build_pipeline(video_device_id, width, height, framerate),
            cv2.CAP_GSTREAMER,
        )

    def compress_image(self, img, quality=90):
        """
        压缩图像。

        参数：
        - img：要压缩的图像
        - quality：压缩质量（0-100），默认为90

        返回：
        - compressed_img：压缩后的图像
        """
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, compressed_img = cv2.imencode(".jpg", img, encode_param)
        return cv2.imdecode(compressed_img, 1)

    def calibrate_camera(self, frame, calibration_data):

        if calibration_data:  # 相机校准
            frame = cv2.undistort(
                frame,
                np.array(calibration_data.k).reshape(3, 3),
                np.array(calibration_data.d),
            )
        return frame

    def load_camera_calibration_from_yaml(self, input_file):
        """
        从 YAML 文件中加载相机标定数据。

        Args:
            input_file (str): 要加载数据的 YAML 文件路径。

        Returns:
            dict: 包含相机标定数据的字典。
        """
        # 创建一个 YAML 对象
        yaml = YAML()

        # 从 YAML 文件中加载数据
        with open(input_file, "r") as f:
            data = yaml.load(f)

        return data


def main():
    rclpy.init()  # 初始化ROS2节点
    node = MonocularCameraNode()  # 创建CSI摄像头节点
    try:
        while rclpy.ok():
            node.capture_image()  # 捕获并发布图像
            node.publish_camera_calibration()
    except KeyboardInterrupt:
        pass
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2节点


if __name__ == "__main__":
    main()
