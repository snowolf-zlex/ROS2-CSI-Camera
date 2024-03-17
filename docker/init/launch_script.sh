#!/bin/bash

# # 启动 Python 脚本
# python3 your_script.py

# 加载工程环境
# source /opt/csi_ros2/install/setup.bash 

# 启动 ROS2 launch 文件
# ros2 launch csi_cam_service stereo_cam.launch.py
python3 /opt/csi_ws/multi_csi_camera.py
