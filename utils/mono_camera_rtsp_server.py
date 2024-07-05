from gi.repository import Gst, GstRtspServer, GLib
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')


class MonoCameraRTSPServer:
    # 静态变量，摄像头参数
    CAMERA_PARAMS = {
        'launch_string': '( nvarguscamerasrc sensor_id={sensor_id} ! video/x-raw(memory:NVMM), width={width}, height={height}, format=(string)NV12, framerate=(fraction){framerate}/1 ! nvvidconv ! video/x-raw, format=I420 ! omxh264enc ! rtph264pay name=pay0 pt=96 )'
    }

    def __init__(self):
        """
        初始化函数，设置GStreamer和RTSP服务器，配置单个摄像头并启动服务。
        """
        Gst.init(None)
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service('8554')

        # 设置单目摄像头
        self._setup_camera(sensor_id=0, mount_point='/camera')

        # 将服务器连接到主循环并启动
        self.server.attach(None)
        print("RTSP流已准备就绪:")
        print(f"rtsp://localhost:8554/camera")

    def _setup_camera(self, sensor_id, mount_point):
        """
        配置摄像头的RTSP工厂，设置摄像头参数和挂载点。

        Args:
        - sensor_id: 摄像头的传感器ID。
        - mount_point: 摄像头在RTSP服务器上的挂载点路径。
        """
        factory = GstRtspServer.RTSPMediaFactory()
        launch_string = self._build_launch_string(
            sensor_id=sensor_id, width=1280, height=720, framerate=30)
        factory.set_launch(launch_string)
        factory.set_shared(True)
        mounts = self.server.get_mount_points()
        mounts.add_factory(mount_point, factory)

    def _build_launch_string(self, sensor_id, width, height, framerate):
        """
        构建摄像头的GStreamer启动字符串。

        Args:
        - sensor_id: 摄像头的传感器ID。
        - width: 视频帧宽度。
        - height: 视频帧高度。
        - framerate: 视频帧率。

        Returns:
        - 合成的GStreamer启动字符串。
        """
        launch_string = self.CAMERA_PARAMS['launch_string'].format(
            sensor_id=sensor_id,
            width=width,
            height=height,
            framerate=framerate
        )
        return launch_string


if __name__ == "__main__":
    server = MonoCameraRTSPServer()
    loop = GLib.MainLoop()
    loop.run()
