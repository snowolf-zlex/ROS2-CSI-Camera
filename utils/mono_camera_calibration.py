#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
单目摄像头CSI相机标定模块

标定步骤：
1. 准备标定板：选择具有易于检测特征的标定板，如黑白相间的方格或圆点。
2. 拍摄标定图像：从CSI摄像头捕获一系列标定图像。
3. 检测特征点：使用图像处理技术检测标定板上的特征点。
4. 标定相机：计算相机的内部参数（如相机矩阵和畸变系数）。
5. 保存标定结果：将标定结果存储为YAML文件，以便后续使用。

"""

import cv2
import numpy as np
import glob
from ruamel.yaml import YAML
import os

def build_pipeline(sensor_id, width=640, height=480, framerate=30):
    """
    构建GStreamer管道，用于从CSI摄像头获取图像数据。

    Args:
        sensor_id (int): 摄像头传感器ID。

    Returns:
        str: GStreamer管道字符串。
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


def save_camera_calibration_to_yaml(calibration_result, output_file, path_prefix="./config/"):
    """
    将相机标定结果存储为YAML文件。

    Args:
        calibration_result (dict): 包含相机标定结果的字典。
        output_file (str): 输出的YAML文件路径。
        path_prefix (str, optional): 存储文件的路径前缀。默认为"./config/"。

    Returns:
        None
    """
    yaml = YAML()
    data = {
        "image_width": calibration_result["image_width"],
        "image_height": calibration_result["image_height"],
        "camera_name": "single_csi_camera",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": list(calibration_result["camera_matrix"].flatten().tolist()),
        },
        "distortion_model": calibration_result["distortion_model"],
        "distortion_coefficients": {
            "rows": 1,
            "cols": len(calibration_result["distortion_coefficients"]),
            "data": calibration_result["distortion_coefficients"].flatten().tolist(),
        },
    }

    create_directory_if_not_exists(path_prefix)
    with open(output_file, "w") as f:
        yaml.dump(data, f)


def load_camera_calibration_from_yaml(input_file):
    """
    从 YAML 文件中加载相机标定数据。

    Args:
        input_file (str): 要加载数据的 YAML 文件路径。

    Returns:
        dict: 包含相机标定数据的字典。
    """
    yaml = YAML()
    with open(input_file, "r") as f:
        data = yaml.load(f)
    return data


def load_calibration_data(calibration_file):
    """
    加载相机标定参数。

    Args:
        calibration_file (str): 相机标定文件路径。

    Returns:
        tuple: 相机矩阵和畸变系数。
    """
    calibration_data = load_camera_calibration_from_yaml(calibration_file)
    camera_matrix = np.array(calibration_data["camera_matrix"]["data"]).reshape(3, 3)
    dist_coeffs = np.array(calibration_data["distortion_coefficients"]["data"])
    return camera_matrix, dist_coeffs


def create_directory_if_not_exists(directory):
    """
    如果指定目录不存在，则创建目录。

    Args:
        directory (str): 目录路径。

    Returns:
        bool: 创建成功返回True，否则返回False。
    """
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
            print(f"目录 '{directory}' 已成功创建。")
            return True
        except OSError as e:
            print(f"创建目录 '{directory}' 失败：{e}")
            return False
    else:
        print(f"目录 '{directory}' 已存在。")
        return True


def capture_calibration_images(
    sensor_id, output_directory, num_images=20, width=640, height=480, framerate=30
):
    """
    从CSI摄像头捕获用于标定的图像。

    Args:
        sensor_id (int): 摄像头传感器ID。
        output_directory (str): 存储捕获图像的目录路径。
        num_images (int, optional): 要捕获的图像数量。默认为20。
        width (int, optional): 图像宽度。默认为640。
        height (int, optional): 图像高度。默认为480。
        framerate (int, optional): 视频帧率。默认为30。

    Returns:
        list: 捕获的图像文件路径列表。
    """
    pipeline = build_pipeline(sensor_id, width, height, framerate)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise ValueError(f"Error: Unable to open CSI camera with sensor ID {sensor_id}")

    create_directory_if_not_exists(output_directory)

    image_paths = []
    try:
        for i in range(num_images):
            ret, frame = cap.read()
            if not ret:
                break

            image_path = os.path.join(output_directory, f"calibration_image_{i}.jpg")
            cv2.imwrite(image_path, frame)

            image_paths.append(image_path)

            print(f"Captured image {i + 1}/{num_images}")

    except KeyboardInterrupt:
        print("\nImage capture interrupted.")

    cap.release()
    cv2.destroyAllWindows()

    return image_paths


def calibrate_camera(images_directory):
    """
    对单目CSI摄像头进行标定。

    Args:
        images_directory (str): 存储标定图像的目录路径。

    Returns:
        dict: 包含标定结果的字典，包括 camera_matrix, distortion_coefficients, distortion_model。
    """
    CHECKERBOARD_SIZE = (6, 9)  # 棋盘格尺寸
    SQUARE_SIZE = 20  # 棋盘格尺寸（毫米）

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []
    objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    image_paths = glob.glob(os.path.join(images_directory, "*.jpg"))

    for image_path in image_paths:
        img = cv2.imread(image_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD_SIZE, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

    h, w = gray.shape
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )

    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

    return {
        "image_width": w,
        "image_height": h,
        "camera_matrix": mtx,
        "distortion_coefficients": dist,
        "distortion_model": "plumb_bob",
    }


def main():
    sensor_id = 0  # CSI摄像头传感器ID
    output_directory = "./calibration_images/"  # 存储标定图像的目录路径

    print("1. 捕获标定图像...")
    capture_calibration_images(sensor_id, output_directory)

    print("2. 标定摄像头...")
    calibration_result = calibrate_camera(output_directory)

    print("3. 保存标定结果...")
    output_file = "./camera_calibration.yaml"
    save_camera_calibration_to_yaml(calibration_result, output_file)

    print(f"相机标定结果已保存到 '{output_file}'.")


if __name__ == "__main__":
    main()
