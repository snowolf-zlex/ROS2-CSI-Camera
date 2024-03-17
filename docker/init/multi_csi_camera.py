import cv2

"""
# 开启第一摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=0 &
# 开启第二摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=1 &
"""

# 构建通道字符串
def build_pipeline(sensor_id):
    width = 640
    height = 480
    framerate = 30
    pipeline = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=(string)BGRx ! "
        "videoconvert ! "
        "appsink"
    )
    return pipeline

# 打开GStreamer管道
cap1 = cv2.VideoCapture(build_pipeline(0), cv2.CAP_GSTREAMER)
cap2 = cv2.VideoCapture(build_pipeline(1), cv2.CAP_GSTREAMER)

while True:
    # 读取帧
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()

    if (not ret1) and (not ret2):
        print("无法从nvarguscamerasrc读取帧")
        break

    # 水平拼接两个图像
    combined_image = cv2.hconcat([frame1, frame2])
    # 垂直拼接两个图像
    # combined_image = cv2.vconcat([frame1, frame2])

    # 显示拼接后的图像
    cv2.imshow("Combined Cameras", combined_image)

    # 检查是否按下了 ESC 键
    if cv2.waitKey(1) == 27:
        break

# 释放资源
cap1.release()
cap2.release()
cv2.destroyAllWindows()
