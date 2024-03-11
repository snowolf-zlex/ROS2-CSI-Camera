#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
双目摄像头立体相机标定模块

一般标定步骤：
1. 收集数据：获取一组双目图像对，确保它们具有不同的角度和深度。最好使用具有已知尺寸的标定板或标定目标。
2. 选择标定板：标定板应该具有易于检测的特征，例如黑白相间的方格或圆点。
3. 摆放标定板：将标定板放置在摄像头视野内，确保在不同的角度和位置都能够被捕捉到。
4. 检测特征点：使用图像处理技术检测标定板上的特征点，并存储为Yaml文件。这些特征点应该在两个摄像头的图像中都能够准确地检测到。
5. 计算内参：使用已知的摄像头参数（例如焦距、主点等）来计算内参矩阵。这可以通过使用标定板的已知尺寸以及检测到的特征点来实现。
6. 计算外参：通过比较两个摄像头的图像，以及它们相对于标定板的位置和角度，计算两个摄像头之间的外部参数，例如旋转和平移。
7. 程序优化：通常需要进行迭代优化，以确保内参和外参的准确性。
8. 结果验证：使用标定结果来验证双目摄像头的视觉测量和深度感知能力。
"""

import cv2
import time
import os
import sys
import numpy as np
import glob
from ruamel.yaml import YAML
from tqdm import tqdm


def build_pipeline(sensor_id, width=640, height=480, framerate=30):
    """
    构建GStreamer管道，用于从CSI摄像头获取图像数据。

    Args:
        sensor_id (int): 摄像头传感器ID。

    Returns:
        str: GStreamer管道字符串。

    """

    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "  # 使用nvarguscamerasrc捕获图像
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "  # 设置图像格式和分辨率
        "nvvidconv ! "  # 进行格式转换
        "video/x-raw, format=(string)BGRx ! "  # 设置图像格式
        "videoconvert ! "  # 进行格式转换
        "appsink"  # 将图像传递给appsink元素
    )
    return pipeline


def save_camera_calibration_to_yaml(
    calibration_result, output_file, path_prefix="./config/"
):
    """
    将相机标定结果存储为YAML文件。

    Args:
        calibration_result (dict): 包含相机标定结果的字典。
        output_file (str): 输出的YAML文件路径。
        path_prefix (str, optional): 存储文件的路径前缀。默认为"./config/"。

    Returns:
        None
    """
    # 创建一个 YAML 对象
    yaml = YAML()

    # 将标定结果整理成一个字典
    data = {
        "image_width": calibration_result["image_width"],
        "image_height": calibration_result["image_height"],
        "camera_name": "narrow_stereo",
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
        "rectification_matrix": {
            "rows": 3,
            "cols": 3,
            "data": calibration_result["rectification_matrix"].flatten().tolist(),
        },
        "projection_matrix": {
            "rows": 3,
            "cols": 4,
            "data": calibration_result["projection_matrix"].flatten().tolist(),
        },
    }

    create_directory_if_not_exists(path_prefix)

    # 将整理好的数据写入 YAML 文件
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
    # 创建一个 YAML 对象
    yaml = YAML()

    # 从 YAML 文件中加载数据
    with open(input_file, "r") as f:
        data = yaml.load(f)

    return data


def load_calibration_data(calibration_file):
    """
    加载相机标定参数
    Args:
        calibration_file (str): 相机标定文件路径
    Returns:
        tuple: 相机矩阵和畸变系数
    """
    calibration_data = load_camera_calibration_from_yaml(calibration_file)

    camera_matrix = np.array(calibration_data["camera_matrix"]["data"]).reshape(3, 3)
    dist_coeffs = np.array(calibration_data["distortion_coefficients"]["data"])

    return camera_matrix, dist_coeffs


def undistort_video(devices_id, calibration_file):
    """
    校正通过GStreamer捕获的视频
    Args:
        sensor_id (int): 相机传感器ID
        calibration_file (str): 相机标定文件路径
    """
    left_camera_matrix, left_dist_coeffs = load_calibration_data(calibration_file[0])
    right_camera_matrix, right_dist_coeffs = load_calibration_data(calibration_file[1])

    capture_left = cv2.VideoCapture(build_pipeline(devices_id[0]), cv2.CAP_GSTREAMER)
    capture_right = cv2.VideoCapture(build_pipeline(devices_id[1]), cv2.CAP_GSTREAMER)

    while True:
        ret_left, frame_left = capture_left.read()
        ret_right, frame_right = capture_right.read()  # 从指定视频设备捕获帧
        if not ret_left or not ret_right:
            break

        oringnal_image = cv2.hconcat([frame_left, frame_right])  # 原始图像横向整合

        undistorted_image_left = cv2.undistort(
            frame_left, left_camera_matrix, left_dist_coeffs
        )
        undistorted_image_right = cv2.undistort(
            frame_right, right_camera_matrix, right_dist_coeffs
        )

        undistorted_image = cv2.hconcat(
            [undistorted_image_left, undistorted_image_right]
        )  # 校准图像横向整合

        combined_image = cv2.vconcat(
            [oringnal_image, undistorted_image]
        )  # 图像纵向整合

        cv2.imshow("Undistorted Video", combined_image)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

    capture_left.release()
    capture_right.release()
    cv2.destroyAllWindows()


def pretty_print_yaml(data):
    """
    以原始格式打印 YAML 数据。

    Args:
        data (dict): 要打印的 YAML 数据。
    """
    # 创建一个 YAML 对象
    yaml = YAML()

    # 打印 YAML 数据
    yaml.dump(data, sys.stdout)


def clear_directory(directory):
    """
    清空指定目录中的文件和子目录。

    Args:
        directory (str): 目录路径。

    Returns:
        None

    """
    if not os.path.exists(directory):
        print(f"目录 '{directory}' 不存在")
        return

    files = os.listdir(directory)  # 获取目录中的所有文件和子目录
    for file in files:
        file_path = os.path.join(directory, file)
        if os.path.isfile(file_path):
            os.remove(file_path)  # 如果是文件，则删除
            print(f"删除文件: {file_path}")
        elif os.path.isdir(file_path):
            clear_directory(file_path)  # 如果是子目录，则递归调用清空目录函数

    os.rmdir(directory)  # 删除完所有文件后，删除目录本身
    print(f"删除目录: {directory}")


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


def get_photos_path(path_prefix="./images/"):
    """
    构建用于保存图片的路径。
    Args:
        prefix_path (str): 路径前缀。
    Returns:
        tuple: 保存图片的两个路径。

    """
    path_camera_1 = path_prefix + "1/"
    path_camera_2 = path_prefix + "2/"

    return path_camera_1, path_camera_2


def calibrate_camera(
    images_path, images_size=[640, 480], CHECKERBOARD=(6, 9), SQUARE_SIZE=20
):
    """
    对相机图片进行标定。

    Args:
        images_path (str): 存储拍摄图片的路径。
        CHECKERBOARD (tuple, optional): 棋盘格子的行列数。默认为(6, 9)。
        SQUARE_SIZE (int, optional): 棋盘格子的尺寸（毫米）。默认为20。

    Returns:
        dict: 包含标定结果的字典，包括 camera_matrix, distortion_coefficients, distortion_model, rectification_matrix, projection_matrix。
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objpoints = []
    imgpoints = []
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    images = glob.glob(images_path)

    if not images:
        raise ValueError("No images were found!")

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(
            gray,
            CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH
            + cv2.CALIB_CB_FAST_CHECK
            + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if ret == True:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow("Calibrate Camera", img)
        cv2.waitKey(0)
    cv2.destroyAllWindows()

    h, w = gray.shape
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, (w, h), None, None
    )

    # 计算其他标定参数
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    # mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, new_mtx, (w, h), 5) # 畸变校正
    rectification_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    projection_matrix = np.array(
        [
            [new_mtx[0, 0], 0, new_mtx[0, 2], 0],
            [0, new_mtx[1, 1], new_mtx[1, 2], 0],
            [0, 0, 1, 0],
        ]
    )

    return {
        "image_width": images_size[0],
        "image_height": images_size[1],
        "camera_matrix": mtx,
        "distortion_coefficients": dist,
        "distortion_model": "plumb_bob",
        "rectification_matrix": rectification_matrix,
        "projection_matrix": projection_matrix,
    }


def capture_chessboard_images_with_prompt(
    videos=[0, 1],
    photos_num=20,
    CHECKERBOARD=(6, 9),
    SQUARE_SIZE=20,
    path_prefix="./images/",
):
    """
    使用双目摄像头拍摄包含棋盘格的图片，并给出内角提示

    Args:
        videos (list of int, optional): 摄像头设备ID列表，分别对应左右摄像头，默认为[0, 1]。
        photos_num (int, optional): 拍摄照片数量，默认为20。
        CHECKERBOARD (tuple, optional): 棋盘格子的行列数，默认为(6, 9)。
        SQUARE_SIZE (int, optional): 棋盘格子的尺寸（毫米），默认为20。
        path_prefix (str, optional): 图片存储路径的前缀，默认为"./images/"。
    """
    cap1 = cv2.VideoCapture(build_pipeline(videos[0]), cv2.CAP_GSTREAMER)
    cap2 = cv2.VideoCapture(build_pipeline(videos[1]), cv2.CAP_GSTREAMER)

    if not cap1.isOpened() or not cap2.isOpened():
        raise ValueError("Error: Cannot open cameras.")

    # 获取图像的宽度和高度
    image_size1 = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
        cap1.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )
    image_size2 = int(cap2.get(cv2.CAP_PROP_FRAME_WIDTH)), int(
        cap2.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    # 清理并创建图片存储目录
    path_camera_1, path_camera_2 = get_photos_path(path_prefix)
    clear_directory(path_camera_1)
    clear_directory(path_camera_2)
    create_directory_if_not_exists(path_camera_1)
    create_directory_if_not_exists(path_camera_2)

    objpoints1 = []  # 左摄像头物体点列表
    objpoints2 = []  # 右摄像头物体点列表
    imgpoints1 = []
    imgpoints2 = []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    try:
        photos_taken = 0

        with tqdm(
            total=photos_num, desc="Processing", colour="yellow", ncols=100
        ) as pbar:

            while photos_taken < photos_num:
                ret1, frame1 = cap1.read()
                ret2, frame2 = cap2.read()
                if (not ret1) or (not ret2):
                    print("无法从 nvarguscamerasrc 读取帧")
                    break

                gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

                cret1, corners1 = cv2.findChessboardCorners(
                    gray1,
                    CHECKERBOARD,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK
                    + cv2.CALIB_CB_NORMALIZE_IMAGE,
                )
                cret2, corners2 = cv2.findChessboardCorners(
                    gray2,
                    CHECKERBOARD,
                    cv2.CALIB_CB_ADAPTIVE_THRESH
                    + cv2.CALIB_CB_FAST_CHECK
                    + cv2.CALIB_CB_NORMALIZE_IMAGE,
                )

                if cret1 and cret2:
                    objpoints1.append(
                        np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
                    )
                    objpoints1[-1][:, :2] = np.mgrid[
                        0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]
                    ].T.reshape(-1, 2)
                    objpoints1[-1] *= SQUARE_SIZE

                    objpoints2.append(
                        np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
                    )
                    objpoints2[-1][:, :2] = np.mgrid[
                        0 : CHECKERBOARD[0], 0 : CHECKERBOARD[1]
                    ].T.reshape(-1, 2)
                    objpoints2[-1] *= SQUARE_SIZE

                    cv2.cornerSubPix(gray1, corners1, (11, 11), (-1, -1), criteria)
                    imgpoints1.append(corners1)
                    cv2.cornerSubPix(gray2, corners2, (11, 11), (-1, -1), criteria)
                    imgpoints2.append(corners2)

                    frame1 = cv2.drawChessboardCorners(
                        frame1, CHECKERBOARD, corners1, ret1
                    )
                    frame2 = cv2.drawChessboardCorners(
                        frame2, CHECKERBOARD, corners2, ret2
                    )

                combined_image = cv2.hconcat([frame1, frame2])

                cv2.imshow("Calibrating Camera", combined_image)

                key = cv2.waitKey(1) & 0xFF
                if key == ord("q") or key == 27:
                    break

                if key == ord("s"):
                    file_name = time.strftime("%Y-%m-%d_%H-%M-%S") + ".jpg"
                    cv2.imwrite(path_camera_1 + file_name, gray1)
                    cv2.imwrite(path_camera_2 + file_name, gray2)

                    photos_taken += 1
                    pbar.update(1)

        print("数据收集已完成！")

        cv2.destroyAllWindows()

        # 打印存储路径提示
        print("图片存储路径：")
        print(f"左摄像头：{path_camera_1}")
        print(f"右摄像头：{path_camera_2}")

        return image_size1, image_size2

    except KeyboardInterrupt:
        cap1.release()
        cap2.release()


def main():
    print("1. 数据采集：使用双目摄像头拍摄包含棋盘格的图片，给出内角提示确认后存储图片")
    capture_chessboard_images_with_prompt()

    print("2. 相机标定：使用图片做相机标定，并输出标定结果")
    images_path1 = "./images/1/*.jpg"
    images_path2 = "./images/2/*.jpg"

    calibration_result1 = calibrate_camera(images_path1)
    # 输出标定结果
    print("Camera 1:")
    print("Camera Image Width:")
    print(calibration_result1["image_width"])
    print("Camera Image Height:")
    print(calibration_result1["image_height"])
    print("Camera Matrix:")
    print(calibration_result1["camera_matrix"])
    print("Camera Matrix:")
    print(calibration_result1["camera_matrix"])
    print("\nDistortion Coefficients:")
    print(calibration_result1["distortion_coefficients"])
    print("\nDistortion Model:")
    print(calibration_result1["distortion_model"])
    print("\nRectification Matrix:")
    print(calibration_result1["rectification_matrix"])
    print("\nProjection Matrix:")
    print(calibration_result1["projection_matrix"])

    calibration_result2 = calibrate_camera(images_path2)
    # 输出标定结果
    print("Camera 1:")
    print("Camera Image Width:")
    print(calibration_result2["image_width"])
    print("Camera Image Height:")
    print(calibration_result2["image_height"])
    print("Camera Matrix:")
    print(calibration_result2["camera_matrix"])
    print("\nDistortion Coefficients:")
    print(calibration_result2["distortion_coefficients"])
    print("\nDistortion Model:")
    print(calibration_result2["distortion_model"])
    print("\nRectification Matrix:")
    print(calibration_result2["rectification_matrix"])
    print("\nProjection Matrix:")
    print(calibration_result2["projection_matrix"])

    print("3. 数据存储：将数据转为Yaml格式存储")
    camera_calibration_file1 = "./config/video0.yaml"
    camera_calibration_file2 = "./config/video1.yaml"

    save_camera_calibration_to_yaml(calibration_result1, camera_calibration_file1)
    save_camera_calibration_to_yaml(calibration_result2, camera_calibration_file2)

    print("4. 数据查看：提取相机标定结果，查看相机标定参数")
    # 加载相机标定参数
    calibration_data1 = load_camera_calibration_from_yaml(camera_calibration_file1)
    calibration_data2 = load_camera_calibration_from_yaml(camera_calibration_file2)

    # 打印加载的数据
    pretty_print_yaml(calibration_data1)
    pretty_print_yaml(calibration_data2)

    print("5. 相机校准：使用相机标定数据，对比校准结果")
    undistort_video([0, 1], [camera_calibration_file1, camera_calibration_file2])


if __name__ == "__main__":
    main()
