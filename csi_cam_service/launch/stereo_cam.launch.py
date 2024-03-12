import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取左右相机参数文件的路径
    left = os.path.join(
        get_package_share_directory("csi_cam_service"), "config", "left.yaml"
    )
    right = os.path.join(
        get_package_share_directory("csi_cam_service"), "config", "right.yaml"
    )

    return LaunchDescription([
        Node(
            package="csi_cam_service",
            executable="stereo_cam_node",
            namespace="csi_cam_service",
            name="scn",  # 重新命名节点为"scn(stereo_cam_node)"
            parameters=[{"calibration_file_path": [left, right]}],  # 设置相机标定文件路径
        )
    ])
