#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
相机标定棋盘板生成模块
因设备像素有不同，最终显示、打印效果会存在部分偏差
"""

import cv2
import numpy as np

def generate_chessboard_image(rows, cols, square_size_mm, printing_dpi=150):
    # A4纸尺寸
    a4_width_mm = 210
    a4_height_mm = 297

    # 计算棋盘格子的像素尺寸
    square_size_px = int(square_size_mm / 25.4 * printing_dpi)  # 每英寸像素数

    # 计算A4纸的像素尺寸
    a4_width_px = int(a4_width_mm / 25.4 * printing_dpi)  # 每英寸像素数
    a4_height_px = int(a4_height_mm / 25.4 * printing_dpi)  # 每英寸像素数

    # 创建A4纸图像
    a4_paper = np.ones((a4_height_px, a4_width_px, 3), dtype=np.uint8) * 255  # 白色背景

    # 计算棋盘格子的起始坐标
    start_x = (a4_width_px - cols * square_size_px) // 2
    start_y = (a4_height_px - rows * square_size_px) // 2

    # 绘制棋盘格子
    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 != 0:  # 如果不是偶数行偶数列
                color = (0, 0, 0)  # 黑色
                cv2.rectangle(
                    a4_paper,
                    (start_x + j * square_size_px, start_y + i * square_size_px),
                    (
                        start_x + (j + 1) * square_size_px,
                        start_y + (i + 1) * square_size_px,
                    ),
                    color,
                    -1,
                )

    # 在图像下方添加说明
    text = f"{cols} x {rows} | {square_size_mm}mm"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.8
    font_thickness = 2
    text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
    text_x = a4_width_px - text_size[0] - 150  # 右对齐
    text_y = a4_height_px - text_size[1] - 120  # 留出一些空白
    cv2.putText(a4_paper, text, (text_x, text_y), font, font_scale, (128, 128, 128), font_thickness)  # 灰色字体

    # 生成文件名
    file_name = f"chessboard_a4_{rows}x{cols}_{square_size_mm}mm_white_bg.png"

    # 保存A4纸图像
    cv2.imwrite(file_name, a4_paper)

    # 显示A4纸图像
    cv2.imshow("A4 Paper with Chessboard", a4_paper)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# 调用函数生成棋盘格图像
generate_chessboard_image(rows=10, cols=7, square_size_mm=20)
