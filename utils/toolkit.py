#!/usr/bin/env python3
"""
Jetson Camera Toolkit - 统一工具入口

基于 NVIDIA Jetson 平台的摄像头采集、RTSP 推流、相机标定一体化工具包。

用法:
    # 摄像头推流
    python toolkit.py stream --source=csi://0

    # 多摄像头查看
    python toolkit.py view --sources=0,1,2

    # 相机标定
    python toolkit.py calibrate mono --images-dir=./calib_images
    python toolkit.py calibrate stereo --left-dir=./left --right-dir=./right

    # 生成棋盘格
    python toolkit.py chessboard
"""

import sys
import os
import argparse
import logging
import cv2
import numpy as np

# 导入模块
from camera import (
    VideoSource, CameraType, detect_camera_type,
    setup_logging, get_ips, draw_text
)
from rtsp import RTSPServer, CameraLayout
from calibrate import (
    CheckerboardConfig, MonoCalibrator, StereoCalibrator,
    save_calibration_to_yaml, load_calibration_from_yaml
)

logger = logging.getLogger(__name__)


# =============================================================================
# 命令: stream (摄像头推流)
# =============================================================================

def cmd_stream(args):
    """摄像头直接推流"""
    source = args.source
    camera_type = None

    if isinstance(source, str):
        if source.startswith('csi://'):
            camera_type = CameraType.CSI
            source = int(source.split('://')[1])
        elif source.startswith('usb://'):
            camera_type = CameraType.USB
            source = int(source.split('://')[1])
        else:
            logger.error(f"不支持的源格式: {source}")
            return 1
    elif isinstance(source, int):
        camera_type = detect_camera_type(source)
        if camera_type is None:
            logger.error(f"无法检测摄像头类型: {source}")
            return 1

    rtsp = RTSPServer(port=args.port)
    rtsp.add_camera_stream(camera_type, source, args.width, args.height, args.fps)

    local_ip, _ = get_ips()
    type_name = "CSI" if camera_type == CameraType.CSI else "USB"

    logger.info("=" * 50)
    logger.info(f"{type_name} 摄像头推流服务已启动")
    logger.info(f"设备: {source}")
    logger.info(f"分辨率: {args.width}x{args.height} @ {args.fps}fps")
    logger.info(f"访问地址: rtsp://{local_ip}:{args.port}/stream")
    logger.info("=" * 50)

    try:
        rtsp.start()
    except KeyboardInterrupt:
        logger.info("服务已停止")
    return 0


# =============================================================================
# 命令: view (多摄像头查看)
# =============================================================================

def cmd_view(args):
    """多摄像头布局显示"""
    indices = [int(x) for x in args.sources.split(',')]

    layout = CameraLayout(
        num_cameras=len(indices),
        cell_size=(args.width, args.height)
    )

    caps = []
    for idx in indices:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            caps.append((cap, idx))
            print(f"已打开: /dev/video{idx}")
        else:
            print(f"无法打开: /dev/video{idx}")

    if not caps:
        print("没有可用摄像头")
        return 1

    print(f"布局: {layout.rows}x{layout.cols}, 按 Q 退出")

    try:
        while True:
            frames = []
            for cap, idx in caps:
                ret, frame = cap.read()
                if ret:
                    frame = draw_text(frame, f"/dev/video{idx}",
                                      (10, frame.shape[0] - 10),
                                      color=(0, 255, 255))
                    frames.append(frame)
                else:
                    placeholder = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                    frames.append(placeholder)

            canvas = layout.arrange(frames)
            info = f"Layout: {layout.rows}x{layout.cols} | Cameras: {len(caps)} | Press Q"
            canvas = draw_text(canvas, info, (10, 30), color=(0, 255, 0))

            cv2.imshow("Multi-Camera View", canvas)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        for cap, _ in caps:
            cap.release()
        cv2.destroyAllWindows()

    return 0


# =============================================================================
# 命令: calibrate (相机标定)
# =============================================================================

def cmd_calibrate(args):
    """相机标定"""
    checkerboard = CheckerboardConfig(args.rows, args.cols, args.square_size)

    if args.mode == "mono":
        logger.info(f"单目相机标定 - 图像目录: {args.images_dir}")

        calibrator = MonoCalibrator(checkerboard)
        count = calibrator.add_images_from_dir(args.images_dir)

        if count == 0:
            logger.error("没有有效的标定图像")
            return 1

        logger.info(f"成功处理 {count} 张图像")

        # 获取图像尺寸
        import glob
        img_path = glob.glob(os.path.join(args.images_dir, "*.jpg"))[0]
        img = cv2.imread(img_path)
        h, w = img.shape[:2]

        result = calibrator.calibrate((w, h))
        logger.info(f"标定完成，重投影误差: {result['reprojection_error']:.4f}")

        output_file = args.output or "mono_calibration.yaml"
        save_calibration_to_yaml(result, output_file)
        logger.info(f"结果已保存到: {output_file}")

    elif args.mode == "stereo":
        logger.info(f"双目相机标定 - 左图像目录: {args.left_dir}, 右图像目录: {args.right_dir}")

        calibrator = StereoCalibrator(checkerboard)
        count = calibrator.add_stereo_pairs_from_dirs(args.left_dir, args.right_dir)

        if count == 0:
            logger.error("没有有效的标定图像对")
            return 1

        logger.info(f"成功处理 {count} 对图像")

        # 获取图像尺寸
        import glob
        img_path = glob.glob(os.path.join(args.left_dir, "*.jpg"))[0]
        img = cv2.imread(img_path)
        h, w = img.shape[:2]

        result = calibrator.calibrate((w, h))
        logger.info(f"标定完成，重投影误差: {result['stereo_reprojection_error']:.4f}")

        output_file = args.output or "stereo_calibration.yaml"
        save_calibration_to_yaml(result, output_file)
        logger.info(f"结果已保存到: {output_file}")

    return 0


# =============================================================================
# 命令: chessboard (生成棋盘格)
# =============================================================================

def cmd_chessboard(args):
    """生成棋盘格标定盘"""
    # 参数
    rows = args.rows or 9
    cols = args.cols or 6
    square_size = args.size or 20  # mm

    # 计算实际尺寸
    width_mm = cols * square_size
    height_mm = rows * square_size

    # A4 纸背景
    if args.a4:
        a4_width_mm = 210
        a4_height_mm = 297
        dpi = args.dpi or 150
        width_px = int(a4_width_mm * dpi / 25.4)
        height_px = int(a4_height_mm * dpi / 25.4)

        # 白色背景
        img = np.ones((height_px, width_px, 3), dtype=np.uint8) * 255

        # 居中计算
        square_size_px = int(square_size * dpi / 25.4)
        start_x = (width_px - cols * square_size_px) // 2
        start_y = (height_px - rows * square_size_px) // 2
    else:
        dpi = args.dpi or 300
        width_px = int(width_mm * dpi / 25.4)
        height_px = int(height_mm * dpi / 25.4)

        img = np.ones((height_px, width_px, 3), dtype=np.uint8) * 255
        square_size_px = int(square_size * dpi / 25.4)
        start_x = 0
        start_y = 0

    # 绘制棋盘格
    for row in range(rows - 1):
        for col in range(cols - 1):
            if (row + col) % 2 != 0:
                x1 = start_x + col * square_size_px
                y1 = start_y + row * square_size_px
                x2 = start_x + (col + 1) * square_size_px
                y2 = start_y + (row + 1) * square_size_px
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), -1)

    # 添加说明文字
    if args.a4:
        text = f"{cols}x{rows} | {square_size}mm"
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, 0.8, 2)
        text_x = width_px - tw - 50
        text_y = height_px - th - 50
        cv2.putText(img, text, (text_x, text_y), font, 0.8, (128, 128, 128), 2)

    # 保存
    output_path = args.output or "chessboard.png"
    cv2.imwrite(output_path, img)
    print(f"棋盘格已生成: {output_path}")
    print(f"  尺寸: {cols}x{rows} 格子, 每格 {square_size}mm")
    print(f"  实际尺寸: {width_mm}x{height_mm}mm")
    print(f"  图像: {width_px}x{height_px}px @ {dpi}DPI")

    # 显示
    if args.show:
        cv2.imshow("Chessboard", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0


# =============================================================================
# 主函数
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Jetson Camera Toolkit - 统一工具集",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 摄像头推流
  python toolkit.py stream --source=csi://0

  # 多摄像头查看
  python toolkit.py view --sources=0,1,2

  # 单目相机标定
  python toolkit.py calibrate mono --images-dir=./calib_images

  # 双目相机标定
  python toolkit.py calibrate stereo --left-dir=./left --right-dir=./right

  # 生成棋盘格
  python toolkit.py chessboard --rows=9 --cols=6

更多示例请参考 examples/ 目录
        """
    )

    parser.add_argument('--log', default='INFO',
                       choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       help='日志级别')

    subparsers = parser.add_subparsers(dest='command', help='命令')

    # stream 命令
    stream_parser = subparsers.add_parser('stream', help='摄像头直接推流')
    stream_parser.add_argument('--source', default='csi://0', help='视频源 (csi://0, usb://0)')
    stream_parser.add_argument('--width', type=int, default=1280, help='宽度')
    stream_parser.add_argument('--height', type=int, default=720, help='高度')
    stream_parser.add_argument('--fps', type=int, default=30, help='帧率')
    stream_parser.add_argument('--port', type=int, default=8554, help='RTSP 端口')

    # view 命令
    view_parser = subparsers.add_parser('view', help='多摄像头查看')
    view_parser.add_argument('--sources', default='0', help='摄像头索引，逗号分隔')
    view_parser.add_argument('--width', type=int, default=640, help='宽度')
    view_parser.add_argument('--height', type=int, default=480, help='高度')

    # calibrate 命令
    calib_parser = subparsers.add_parser('calibrate', help='相机标定')
    calib_parser.add_argument('mode', choices=['mono', 'stereo'], help='标定模式')
    calib_parser.add_argument('--rows', type=int, default=6, help='棋盘格行数（内角）')
    calib_parser.add_argument('--cols', type=int, default=9, help='棋盘格列数（内角）')
    calib_parser.add_argument('--square-size', type=float, default=20.0, help='方格大小 (mm)')
    # 单目参数
    calib_parser.add_argument('--images-dir', default='./calib_images', help='单目标定图像目录')
    # 双目参数
    calib_parser.add_argument('--left-dir', default='./left', help='双目左相机图像目录')
    calib_parser.add_argument('--right-dir', default='./right', help='双目右相机图像目录')
    calib_parser.add_argument('--output', help='输出文件路径')

    # chessboard 命令
    chess_parser = subparsers.add_parser('chessboard', help='生成棋盘格标定盘')
    chess_parser.add_argument('--rows', type=int, default=9, help='行数（内角）')
    chess_parser.add_argument('--cols', type=int, default=6, help='列数（内角）')
    chess_parser.add_argument('--size', type=int, default=20, help='方格大小 (mm)')
    chess_parser.add_argument('--dpi', type=int, default=300, help='DPI')
    chess_parser.add_argument('--a4', action='store_true', help='输出 A4 纸格式（居中、带文字）')
    chess_parser.add_argument('--show', action='store_true', help='显示图像')
    chess_parser.add_argument('--output', default='chessboard.png', help='输出文件路径')

    args = parser.parse_args()

    # 设置日志
    setup_logging(args.log)

    if args.command == 'stream':
        return cmd_stream(args)
    elif args.command == 'view':
        return cmd_view(args)
    elif args.command == 'calibrate':
        return cmd_calibrate(args)
    elif args.command == 'chessboard':
        return cmd_chessboard(args)
    else:
        parser.print_help()
        return 0


if __name__ == "__main__":
    sys.exit(main())
