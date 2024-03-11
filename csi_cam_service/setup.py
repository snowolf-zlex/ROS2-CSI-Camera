from setuptools import setup

package_name = "csi_cam_service"

setup(
    name=package_name,
    version="1.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "single_csi_cam_node = csi_cam_service.single_csi_cam_node:main",
            "dual_csi_cam_node = csi_cam_service.dual_csi_cam_node:main",
        ],
    },
)
