#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CSI单目摄像头图像处理模块
默认参数：
    video_device_id=0
    width=640
    height=480
    fps=30
"""


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class SingleCSICameraNode(Node):
    def __init__(self):
        super().__init__("sigle_csi_cam_node")
        self.declare_parameter("video_device_id", 0)  # 声明和初始化参数
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        self.video_device_id = self.get_parameter("video_device_id").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value

        self.get_logger().info(f"Video Device ID: {self.video_device_id}")
        self.get_logger().info(f"Video Width: {self.width}")
        self.get_logger().info(f"Video Height: {self.height}")
        self.get_logger().info(f"Video FPS: {self.fps}")

        self.publisher_ = self.create_publisher(
            Image, "sigle_csi_cam/image", 10
        )  # 创建图像发布者
        self.bridge = CvBridge()  # 初始化CvBridge
        self.capture = cv2.VideoCapture(
            self.build_pipeline(self.video_device_id), cv2.CAP_GSTREAMER
        )  # 初始化视频捕获对象
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open video device")
            raise RuntimeError("Failed to open video device")

    def build_pipeline(self, sensor_id):
        width = self.width  # 图像宽度
        height = self.height  # 图像高度
        framerate = self.fps  # 帧率
        pipeline = (
            f"nvarguscamerasrc sensor-id={sensor_id} ! "
            f"video/x-raw(memory:NVMM), width={width}, height={height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
            "nvvidconv ! "
            "video/x-raw, format=(string)BGRx ! "
            "videoconvert ! "
            "appsink"
        )  # 构建GStreamer管道
        return pipeline

    def capture_image(self):
        ret, frame = self.capture.read()  # 从指定视频设备捕获帧
        if ret:
            image_msg = self.bridge.cv2_to_imgmsg(
                frame, encoding="bgr8"
            )  # 将OpenCV图像转换为ROS图像消息
            self.publisher_.publish(image_msg)  # 发布图像消息
            self.get_logger().info("Image captured and published")  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture image")  # 输出错误日志信息


def main():
    rclpy.init()  # 初始化ROS2节点
    node = SingleCSICameraNode()  # 创建CSI摄像头节点
    try:
        while rclpy.ok():
            node.capture_image()  # 捕获并发布图像
    except KeyboardInterrupt:
        pass
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2节点


if __name__ == "__main__":
    main()
