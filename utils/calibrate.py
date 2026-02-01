#!/usr/bin/env python3
"""
Calibration Module - 相机标定功能

提供单目和双目相机标定功能。
"""

import os
import logging
import cv2
import numpy as np
import glob

logger = logging.getLogger(__name__)


# =============================================================================
# 标定配置
# =============================================================================

class CheckerboardConfig:
    """棋盘格配置"""

    # 常用棋盘格尺寸 (内角点)
    SIZE_6x9 = (6, 9)   # 7x10 格子
    SIZE_9x6 = (9, 6)   # 10x7 格子

    def __init__(self, rows: int, cols: int, square_size_mm: float = 20.0):
        """
        初始化棋盘格配置

        Args:
            rows: 内角点行数
            cols: 内角点列数
            square_size_mm: 方格大小（毫米）
        """
        self.rows = rows
        self.cols = cols
        self.square_size_mm = square_size_mm

    @property
    def size(self):
        """返回 (rows, cols) 格式"""
        return (self.rows, self.cols)


# =============================================================================
# 单目标定
# =============================================================================

class MonoCalibrator:
    """单目相机标定器"""

    def __init__(self, checkerboard: CheckerboardConfig):
        """
        初始化单目标定器

        Args:
            checkerboard: 棋盘格配置
        """
        self.checkerboard = checkerboard
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 准备物体点（3D）
        self.objp = np.zeros((checkerboard.rows * checkerboard.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard.rows, 0:checkerboard.cols].T.reshape(-1, 2)
        self.objp *= checkerboard.square_size_mm

        self.obj_points = []  # 3D 点
        self.img_points = []  # 2D 点

    def add_image(self, image_path: str) -> bool:
        """
        添加标定图像

        Args:
            image_path: 图像路径

        Returns:
            bool: 成功检测到角点返回 True
        """
        img = cv2.imread(image_path)
        if img is None:
            logger.warning(f"无法读取图像: {image_path}")
            return False

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 寻找棋盘格角点
        ret, corners = cv2.findChessboardCorners(
            gray,
            self.checkerboard.size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret:
            self.obj_points.append(self.objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
            self.img_points.append(corners2)
            logger.debug(f"成功处理: {image_path}")
            return True

        logger.warning(f"未检测到角点: {image_path}")
        return False

    def add_images_from_dir(self, directory: str, pattern: str = "*.jpg") -> int:
        """
        从目录添加所有标定图像

        Args:
            directory: 图像目录
            pattern: 文件匹配模式

        Returns:
            int: 成功处理的图像数量
        """
        image_paths = glob.glob(os.path.join(directory, pattern))
        count = 0
        for path in image_paths:
            if self.add_image(path):
                count += 1
        return count

    def calibrate(self, image_size: tuple) -> dict:
        """
        执行标定

        Args:
            image_size: 图像尺寸 (width, height)

        Returns:
            dict: 标定结果
        """
        if len(self.obj_points) == 0:
            raise ValueError("没有有效的标定图像，请先添加图像")

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )

        if not ret:
            raise ValueError("标定失败")

        # 计算最优相机矩阵
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, image_size, 1, image_size)

        return {
            "image_width": image_size[0],
            "image_height": image_size[1],
            "camera_matrix": mtx,
            "new_camera_matrix": new_mtx,
            "distortion_coefficients": dist,
            "distortion_model": "plumb_bob",
            "roi": roi,
            "rvecs": rvecs,
            "tvecs": tvecs,
            "reprojection_error": ret,
        }


# =============================================================================
# 双目标定
# =============================================================================

class StereoCalibrator:
    """双目相机标定器"""

    def __init__(self, checkerboard: CheckerboardConfig):
        """
        初始化双目标定器

        Args:
            checkerboard: 棋盘格配置
        """
        self.checkerboard = checkerboard
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 准备物体点（3D）
        self.objp = np.zeros((checkerboard.rows * checkerboard.cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:checkerboard.rows, 0:checkerboard.cols].T.reshape(-1, 2)
        self.objp *= checkerboard.square_size_mm

        self.obj_points = []  # 3D 点
        self.img_points_left = []  # 左相机 2D 点
        self.img_points_right = []  # 右相机 2D 点

    def add_stereo_pair(self, left_image_path: str, right_image_path: str) -> bool:
        """
        添加双目图像对

        Args:
            left_image_path: 左相机图像路径
            right_image_path: 右相机图像路径

        Returns:
            bool: 两张图都成功检测到角点返回 True
        """
        img_left = cv2.imread(left_image_path)
        img_right = cv2.imread(right_image_path)

        if img_left is None or img_right is None:
            logger.warning(f"无法读取图像对: {left_image_path}, {right_image_path}")
            return False

        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        # 寻找棋盘格角点
        ret_left, corners_left = cv2.findChessboardCorners(
            gray_left, self.checkerboard.size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        ret_right, corners_right = cv2.findChessboardCorners(
            gray_right, self.checkerboard.size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if ret_left and ret_right:
            self.obj_points.append(self.objp)

            corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), self.criteria)
            corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), self.criteria)

            self.img_points_left.append(corners_left)
            self.img_points_right.append(corners_right)
            logger.debug(f"成功处理图像对: {left_image_path}, {right_image_path}")
            return True

        logger.warning(f"未检测到角点: {left_image_path}, {right_image_path}")
        return False

    def add_stereo_pairs_from_dirs(self, left_dir: str, right_dir: str, pattern: str = "*.jpg") -> int:
        """
        从目录添加所有双目图像对

        Args:
            left_dir: 左相机图像目录
            right_dir: 右相机图像目录
            pattern: 文件匹配模式

        Returns:
            int: 成功处理的图像对数量
        """
        left_paths = sorted(glob.glob(os.path.join(left_dir, pattern)))
        right_paths = sorted(glob.glob(os.path.join(right_dir, pattern)))

        count = 0
        for left_path, right_path in zip(left_paths, right_paths):
            if self.add_stereo_pair(left_path, right_path):
                count += 1
        return count

    def calibrate(self, image_size: tuple) -> dict:
        """
        执行双目标定

        Args:
            image_size: 图像尺寸 (width, height)

        Returns:
            dict: 标定结果
        """
        if len(self.obj_points) == 0:
            raise ValueError("没有有效的标定图像对，请先添加图像")

        # 先分别标定两个相机
        flags = cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5

        ret_left, mtx_left, dist_left, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points_left, image_size, None, None, flags=flags
        )

        ret_right, mtx_right, dist_right, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points_right, image_size, None, None, flags=flags
        )

        # 双目标定
        ret_stereo, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = cv2.stereoCalibrate(
            self.obj_points,
            self.img_points_left,
            self.img_points_right,
            mtx_left, dist_left,
            mtx_right, dist_right,
            image_size,
            flags=cv2.CALIB_FIX_INTRINSIC
        )

        if not ret_stereo:
            raise ValueError("双目标定失败")

        # 计算立体校正
        R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(
            mtx_left, dist_left,
            mtx_right, dist_right,
            image_size, R, T,
            alpha=0
        )

        return {
            "image_width": image_size[0],
            "image_height": image_size[1],
            "left": {
                "camera_matrix": mtx_left,
                "distortion_coefficients": dist_left,
                "rectification_matrix": R1,
                "projection_matrix": P1,
                "roi": roi_left,
                "reprojection_error": ret_left,
            },
            "right": {
                "camera_matrix": mtx_right,
                "distortion_coefficients": dist_right,
                "rectification_matrix": R2,
                "projection_matrix": P2,
                "roi": roi_right,
                "reprojection_error": ret_right,
            },
            "rotation_matrix": R,
            "translation_matrix": T,
            "essential_matrix": E,
            "fundamental_matrix": F,
            "disparity_to_depth": Q,
            "stereo_reprojection_error": ret_stereo,
        }


# =============================================================================
# YAML 保存/加载
# =============================================================================

def save_calibration_to_yaml(calibration_result: dict, output_file: str):
    """
    保存标定结果到 YAML 文件

    Args:
        calibration_result: 标定结果字典
        output_file: 输出文件路径
    """
    try:
        from ruamel.yaml import YAML
    except ImportError:
        try:
            from yaml import dump
            # 使用 PyYAML
            class YAML:
                def dump(self, data, f):
                    f.write(dump(data, default_flow_style=False))
        except ImportError:
            raise ImportError("请安装 ruamel.yaml 或 pyyaml: pip install ruamel.yaml")

    yaml = YAML()

    # 单目标定格式
    if "left" not in calibration_result:
        data = {
            "image_width": calibration_result["image_width"],
            "image_height": calibration_result["image_height"],
            "camera_name": "mono_camera",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": calibration_result["camera_matrix"].flatten().tolist(),
            },
            "distortion_model": calibration_result["distortion_model"],
            "distortion_coefficients": {
                "rows": 1,
                "cols": len(calibration_result["distortion_coefficients"]),
                "data": calibration_result["distortion_coefficients"].flatten().tolist(),
            },
        }
    # 双目标定格式
    else:
        data = {
            "image_width": calibration_result["image_width"],
            "image_height": calibration_result["image_height"],
            "camera_name": "stereo_camera",
            "left": {
                "camera_matrix": {
                    "rows": 3, "cols": 3,
                    "data": calibration_result["left"]["camera_matrix"].flatten().tolist(),
                },
                "distortion_coefficients": {
                    "rows": 1,
                    "cols": len(calibration_result["left"]["distortion_coefficients"]),
                    "data": calibration_result["left"]["distortion_coefficients"].flatten().tolist(),
                },
                "rectification_matrix": {
                    "rows": 3, "cols": 3,
                    "data": calibration_result["left"]["rectification_matrix"].flatten().tolist(),
                },
                "projection_matrix": {
                    "rows": 3, "cols": 4,
                    "data": calibration_result["left"]["projection_matrix"].flatten().tolist(),
                },
            },
            "right": {
                "camera_matrix": {
                    "rows": 3, "cols": 3,
                    "data": calibration_result["right"]["camera_matrix"].flatten().tolist(),
                },
                "distortion_coefficients": {
                    "rows": 1,
                    "cols": len(calibration_result["right"]["distortion_coefficients"]),
                    "data": calibration_result["right"]["distortion_coefficients"].flatten().tolist(),
                },
                "rectification_matrix": {
                    "rows": 3, "cols": 3,
                    "data": calibration_result["right"]["rectification_matrix"].flatten().tolist(),
                },
                "projection_matrix": {
                    "rows": 3, "cols": 4,
                    "data": calibration_result["right"]["projection_matrix"].flatten().tolist(),
                },
            },
        }

    os.makedirs(os.path.dirname(output_file) or '.', exist_ok=True)
    with open(output_file, "w") as f:
        yaml.dump(data, f)

    logger.info(f"标定结果已保存到: {output_file}")


def load_calibration_from_yaml(input_file: str) -> dict:
    """
    从 YAML 文件加载标定结果

    Args:
        input_file: 输入文件路径

    Returns:
        dict: 标定结果
    """
    try:
        from ruamel.yaml import YAML
    except ImportError:
        try:
            from yaml import safe_load as load
            class YAML:
                def load(self, f):
                    return load(f)
        except ImportError:
            raise ImportError("请安装 ruamel.yaml 或 pyyaml: pip install ruamel.yaml")

    yaml = YAML()
    with open(input_file, "r") as f:
        data = yaml.load(f)

    # 转换 numpy 数组
    def to_numpy(matrix_data):
        return np.array(matrix_data["data"]).reshape(
            matrix_data["rows"], matrix_data.get("cols", 1)
        )

    if "left" in data:
        # 双目标定
        return {
            "image_width": data["image_width"],
            "image_height": data["image_height"],
            "left": {
                "camera_matrix": to_numpy(data["left"]["camera_matrix"]),
                "distortion_coefficients": to_numpy(data["left"]["distortion_coefficients"]),
                "rectification_matrix": to_numpy(data["left"]["rectification_matrix"]),
                "projection_matrix": to_numpy(data["left"]["projection_matrix"]),
            },
            "right": {
                "camera_matrix": to_numpy(data["right"]["camera_matrix"]),
                "distortion_coefficients": to_numpy(data["right"]["distortion_coefficients"]),
                "rectification_matrix": to_numpy(data["right"]["rectification_matrix"]),
                "projection_matrix": to_numpy(data["right"]["projection_matrix"]),
            },
        }
    else:
        # 单目标定
        return {
            "image_width": data["image_width"],
            "image_height": data["image_height"],
            "camera_matrix": to_numpy(data["camera_matrix"]),
            "distortion_coefficients": to_numpy(data["distortion_coefficients"]),
            "distortion_model": data.get("distortion_model", "plumb_bob"),
        }
