#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CSI双目摄像头图像处理模块
默认参数：
    video_device_id:=[0,1]
    image_size:=[640,480]
    fps:=30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import cv2
import os

import cv2
import numpy as np

from ruamel.yaml import YAML


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_cam_node")

        # 声明和初始化参数
        self.declare_parameter("video_device_id", [0, 1])
        self.declare_parameter("image_size", [640, 480])
        self.declare_parameter("fps", 30)
        self.declare_parameter("calibration_file_path", None)  # 相机校准数据文件
        self.declare_parameter("algorithm_type", "bm")  # 深度算法类型, bm 或 sgbm

        self.left_video_device_id, self.right_video_device_id = self.get_parameter(
            "video_device_id"
        ).value
        self.width, self.height = self.get_parameter("image_size").value
        self.fps = self.get_parameter("fps").value

        self.calibration_file_path = self.get_parameter("calibration_file_path").value
        self.calibration_data_left = None
        self.calibration_data_right = None
        if self.calibration_file_path:
            self.calibration_data_left = self.load_camera_info(
                self.calibration_file_path[0]
            )
            self.calibration_data_right = self.load_camera_info(
                self.calibration_file_path[1]
            )

        self.algorithm_type = self.get_parameter("algorithm_type").value

        self.get_logger().info(f"Left Video Device ID: {self.left_video_device_id}")
        self.get_logger().info(f"Right Video Device ID: {self.right_video_device_id}")
        self.get_logger().info(f"Video Width: {self.width}")
        self.get_logger().info(f"Video Height: {self.height}")
        self.get_logger().info(f"Video FPS: {self.fps}")

        self.get_logger().info(f"Calibration File Path: {self.calibration_file_path}")

        self.get_logger().info(f"Deep Algorithm: {self.algorithm_type}")

        self.publisher_left_raw = self.create_publisher(
            Image, "/stereo_cam/image_left_raw", 10
        )  # 创建左侧原始图像发布者
        self.publisher_right_raw = self.create_publisher(
            Image, "/stereo_cam/image_right_raw", 10
        )  # 创建右侧原始图像发布者

        self.publisher_combine_raw = self.create_publisher(
            Image, "/stereo_cam/image_combine_raw", 10
        )  # 创建整合原始图像发布者

        self.publisher_compressed = self.create_publisher(
            Image, "/stereo_cam/image_compressed", 10
        )  # 创建压缩深度图像发布者

        self.publisher_compressed_depth = self.create_publisher(
            Image, "/stereo_cam/image_compressed_depth", 10
        )  # 创建压缩深度图像发布者

        self.publisher_camera_info = self.create_publisher(
            CameraInfo, "/stereo_cam/camera_info", 10
        )  # 创建镜头信息发布者

        self.bridge = CvBridge()  # 初始化CvBridge
        self.capture_left, self.capture_right = self.init_csi_caputres(
            [self.left_video_device_id, self.right_video_device_id],
            self.width,
            self.height,
            self.fps,
        )  # 初始化视频捕获对象

        if not self.capture_left.isOpened():
            self.get_logger().error("Failed to open left video device")
            raise RuntimeError("Failed to open left video device")
        if not self.capture_right.isOpened():
            self.get_logger().error("Failed to open right video device")
            raise RuntimeError("Failed to open right video device")

    def process_depth_map(self, left_img, right_img):
        # 深度算法选择
        depth_map = None
        if self.algorithm_type == "bm":
            depth_map = self.get_depth_map_bm(left_img, right_img)
        elif self.algorithm_type == "sgbm":
            depth_map = self.get_depth_map_sgbm(left_img, right_img)
        else:
            raise ValueError("Invalid algorithm type")
        return depth_map

    def capture_image(self):
        ret_left, frame_left = self.capture_left.read()  # 从指定视频设备捕获帧

        if ret_left:

            # 相机校准
            frame_left = self.calibrate_camera(frame_left, self.calibration_data_left)

            image_msg_left = self.bridge.cv2_to_imgmsg(frame_left, encoding="bgr8")
            self.publisher_left_raw.publish(image_msg_left)  # 发布图像消息

            self.get_logger().debug("Left image captured and published")  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture Left image")  # 输出错误日志信息

        ret_right, frame_right = self.capture_right.read()  # 从指定视频设备捕获帧
        if ret_right:

            # 相机校准
            frame_right = self.calibrate_camera(
                frame_right, self.calibration_data_right
            )

            image_msg_right = self.bridge.cv2_to_imgmsg(frame_right, encoding="bgr8")
            self.publisher_right_raw.publish(image_msg_right)  # 发布原始图像消息

            self.get_logger().debug(
                "Right image captured and published"
            )  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture right image")  # 输出错误日志信息

        if ret_left and ret_right:
            combined_image = cv2.hconcat([frame_left, frame_right])
            image_msg_combined = self.bridge.cv2_to_imgmsg(
                combined_image, encoding="bgr8"
            )
            self.publisher_combine_raw.publish(
                image_msg_combined
            )  # 发布原始整合图像消息

            compressed_frame_left = self.compress_image(frame_left)  # 压缩左侧原始图像
            compressed_frame_right = self.compress_image(
                frame_right
            )  # 压缩右侧原始图像

            compressed_normalized = cv2.normalize(
                compressed_frame_left, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )

            # 压缩左侧图像
            image_msg_compressed = self.bridge.cv2_to_imgmsg(
                compressed_normalized, encoding="bgr8"
            )
            self.publisher_compressed.publish(
                image_msg_compressed
            )  # 发布压缩图像消息（左侧）

            depth_map = self.process_depth_map(
                compressed_frame_left, compressed_frame_right
            )

            depth_map_normalized = cv2.normalize(
                depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )

            # 使用cv2_to_imgmsg函数将uint8类型的深度图像编码为图像消息
            image_msg_depth = self.bridge.cv2_to_imgmsg(
                depth_map_normalized, encoding="mono8"
            )

            self.publisher_compressed_depth.publish(image_msg_depth)  # 发布深度图像消息

            self.get_logger().debug(
                "Combined image captured and published"
            )  # 输出日志信息
        else:
            self.get_logger().error(
                "Failed to capture left and right images"
            )  # 输出错误日志信息

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

    def init_csi_caputres(
        self, video_device=[0, 1], width=640, height=480, framerate=30
    ):
        """
        初始化CSI双目摄像头

        参数：
        - video_device：摄像头设备ID列表，默认为[0, 1]
        - width：图像宽度，默认为640
        - height：图像高度，默认为480
        - framerate：帧率，默认为30

        返回：
        - cap_left：左侧摄像头视频捕获对象
        - cap_right：右侧摄像头视频捕获对象
        """
        return (
            self.init_csi_caputre(video_device[0], width, height, framerate),
            self.init_csi_caputre(video_device[1], width, height, framerate),
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

    def get_depth_map_bm(self, left_img, right_img, num_disparities=16, block_size=15):
        """
        使用BM算法获取深度图像。

        参数：
        - left_img：左侧摄像头捕获的图像
        - right_img：右侧摄像头捕获的图像
        - num_disparities：视差搜索范围，默认为16
        - block_size：匹配块大小，默认为15

        返回：
        - disparity_normalized：归一化的深度图像
        """
        # 转换为灰度图像
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # 立体匹配（BM算法）
        stereo = cv2.StereoBM_create(
            numDisparities=num_disparities, blockSize=block_size
        )
        disparity = stereo.compute(left_gray, right_gray)

        # 归一化深度图像
        min_disp = disparity.min()
        max_disp = disparity.max()
        disparity_normalized = (disparity - min_disp) / (max_disp - min_disp)

        return disparity_normalized

    def get_depth_map_sgbm(
        self, left_img, right_img, num_disparities=16, block_size=15
    ):
        """
        使用SGBM算法获取深度图像。

        参数：
        - left_img：左侧摄像头捕获的图像
        - right_img：右侧摄像头捕获的图像
        - num_disparities：视差搜索范围，默认为16
        - block_size：匹配块大小，默认为15

        返回：
        - disparity_normalized：归一化的深度图像
        """
        # 转换为灰度图像
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # 立体匹配（SGBM算法）
        stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
        )
        disparity = stereo.compute(left_gray, right_gray)

        # 归一化深度图像
        min_disp = disparity.min()
        max_disp = disparity.max()
        disparity_normalized = (disparity - min_disp) / (max_disp - min_disp)

        return disparity_normalized

    def load_camera_calibration_from_yaml(self, input_file):
        """
        从 YAML 文件中加载相机标定数据。

        Args:
            input_file (str): 要加载数据的 YAML 文件路径。

        Returns:
            dict: 包含相机标定数据的字典。
        """
        if not input_file:
            return None

        # 创建一个 YAML 对象
        yaml = YAML()

        # 从 YAML 文件中加载数据
        with open(input_file, "r") as f:
            data = yaml.load(f)

        return data

    def load_camera_info(self, calibration_file_path):
        camera_info_msg = None
        if calibration_file_path is not None:
            yaml_data = self.load_camera_calibration_from_yaml(calibration_file_path)
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

    def calibrate_camera(self, frame, calibration_data):

        if calibration_data:  # 相机校准
            frame = cv2.undistort(
                frame,
                np.array(calibration_data.k).reshape(3, 3),
                np.array(calibration_data.d),
            )
        return frame

    def publish_camera_calibration(self):
        if self.calibration_data_left:

            # Publish camera calibration parameters
            self.publisher_camera_info.publish(self.calibration_data_left)


def main():
    rclpy.init()  # 初始化ROS2节点
    node = StereoCameraNode()  # 创建CSI摄像头节点
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
