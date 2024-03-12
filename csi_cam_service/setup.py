from setuptools import setup
import os
from glob import glob

package_name = "csi_cam_service"

setup(
    name=package_name,
    version="1.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Snowolf",
    maintainer_email="zlex.dongliang@gmail.com",
    description="CSI单目/多目摄像头ROS2模块，提供单目/双目图像话题，包括原始图像、压缩图像、深度图像和镜头信息等。",
    license="Apache-2.0 license",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mono_cam_node = csi_cam_service.mono_cam_node:main",
            "stereo_cam_node = csi_cam_service.stereo_cam_node:main",
        ],
    },
)
