#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
CSI双目摄像头图像处理模块
默认参数：
    left_video_device_id=0
    right_video_device_id=1
    width=640
    height=480
    fps=30
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DualCSICameraNode(Node):
    def __init__(self):
        super().__init__("dual_csi_cam_node")
        self.declare_parameter("left_video_device_id", 0)  # 声明和初始化参数
        self.declare_parameter("right_video_device_id", 1)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)

        left_video_device_id = self.get_parameter("left_video_device_id").value
        right_video_device_id = self.get_parameter("right_video_device_id").value
        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        fps = self.get_parameter("fps").value

        self.get_logger().info(f"Left Video Device ID: {left_video_device_id}")
        self.get_logger().info(f"Right Video Device ID: {right_video_device_id}")
        self.get_logger().info(f"Video Width: {width}")
        self.get_logger().info(f"Video Height: {height}")
        self.get_logger().info(f"Video FPS: {fps}")
        self.width = width
        self.height = height
        self.fps = fps
        self.publisher_left = self.create_publisher(
            Image, "left_csi_cam/image", 10
        )  # 创建图像发布者
        self.publisher_right = self.create_publisher(
            Image, "right_csi_cam/image", 10
        )  # 创建图像发布者
        self.publisher = self.create_publisher(
            Image, "dual_csi_cam/image", 10
        )  # 创建图像发布者

        self.bridge = CvBridge()  # 初始化CvBridge
        self.capture_left = cv2.VideoCapture(
            self.build_pipeline(left_video_device_id), cv2.CAP_GSTREAMER
        )  # 初始化视频捕获对象
        self.capture_right = cv2.VideoCapture(
            self.build_pipeline(right_video_device_id), cv2.CAP_GSTREAMER
        )  # 初始化视频捕获对象

        if not self.capture_left.isOpened():
            self.get_logger().error("Failed to open left video device")
            raise RuntimeError("Failed to open left video device")
        if not self.capture_right.isOpened():
            self.get_logger().error("Failed to open right video device")
            raise RuntimeError("Failed to open right video device")

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
        ret_left, frame_left = self.capture_left.read()  # 从指定视频设备捕获帧
        if ret_left:
            image_msg_left = self.bridge.cv2_to_imgmsg(frame_left, encoding="bgr8")
            self.publisher_left.publish(image_msg_left)  # 发布图像消息
            self.get_logger().info("Left image captured and published")  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture Left image")  # 输出错误日志信息

        ret_right, frame_right = self.capture_right.read()  # 从指定视频设备捕获帧
        if ret_right:
            image_msg_right = self.bridge.cv2_to_imgmsg(frame_right, encoding="bgr8")
            self.publisher_right.publish(image_msg_right)  # 发布图像消息
            self.get_logger().info("Right image captured and published")  # 输出日志信息
        else:
            self.get_logger().error("Failed to capture right image")  # 输出错误日志信息

        if ret_left and ret_right:
            combined_image = cv2.hconcat([frame_left, frame_right])
            image_msg_combined = self.bridge.cv2_to_imgmsg(
                combined_image, encoding="bgr8"
            )
            self.publisher.publish(image_msg_combined)  # 发布图像消息
            self.get_logger().info(
                "Combined image captured and published"
            )  # 输出日志信息
        else:
            self.get_logger().error(
                "Failed to capture left and right images"
            )  # 输出错误日志信息


def main():
    rclpy.init()  # 初始化ROS2节点
    node = DualCSICameraNode()  # 创建CSI摄像头节点
    try:
        while rclpy.ok():
            node.capture_image()  # 捕获并发布图像
    except KeyboardInterrupt:
        pass
    node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS2节点


if __name__ == "__main__":
    main()
