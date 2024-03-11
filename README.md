# ROS2-CSI-Camera

CSI单目及双目摄像头ROS2模块，适用于Raspberry Pi和Jetson等ARM平台。主要用于ROS2中图像节点发布，使其能够像使用USB_CAM一样方便，如使用该工具做CSI摄像头相机标定。

> [!NOTE]  
> 当前实现参考了[v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)。
>
> ``` shell
> ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" -p image_size:=[1280,720]
> ```

## 1. 准备工作

### 1.1 设备查看

在使用CSI摄像头前，请确认CSI摄像头已连接好，且图像显示正常。

``` shell
# 查看当前摄像头设备
ll /dev/video*
```

如下所示，说明系统中已经连接好了两个摄像头。

``` txt
crw-rw----+ 1 root video 81, 0 Nov 22 05:10 /dev/video0
crw-rw----+ 1 root video 81, 4 Nov 22 05:10 /dev/video1
```

使用`v4l2-ctl`命令查看摄像头设备信息，如查看`/dev/video0`

``` shell
# 查看简要信息
v4l2-ctl --device /dev/video0 --info

# 查看全部信息
v4l2-ctl --device /dev/video0 --all
```

以下是使用`v4l2-ctl --device /dev/video0 --info`命令查看的设备简要信息，说明这是一个`imx219`芯片的CSI摄像头设备。

``` txt
Driver Info:
 Driver name      : tegra-video
 Card type        : vi-output, imx219 10-0010
 Bus info         : platform:tegra-capture-vi:2
 Driver version   : 5.10.104
 Capabilities     : 0x84200001
  Video Capture
  Streaming
  Extended Pix Format
  Device Capabilities
 Device Caps      : 0x04200001
  Video Capture
  Streaming
  Extended Pix Format
Media Driver Info:
 Driver name      : tegra-camrtc-ca
 Model            : NVIDIA Tegra Video Input Device
 Serial           : 
 Bus info         : 
 Media version    : 5.10.104
 Hardware revision: 0x00000003 (3)
 Driver version   : 5.10.104
Interface Info:
 ID               : 0x0300001b
 Type             : V4L Video
Entity Info:
 ID               : 0x00000019 (25)
 Name             : vi-output, imx219 10-0010
 Function         : V4L2 I/O
 Pad 0x0100001a   : 0: Sink
   Link 0x0200001f: from remote pad 0x1000006 of entity '13e40000.host1x:nvcsi@15a00000-': Data, Enabled
```

> [!TIP]
> 如果没有该工具，可通过以下命令安装：
>
> ```shell
> sudo apt install v4l-utils 
> ```

### 1.2 设备检测

完成上述设备检测后，可以使用`nvgstcapture-1.0`命令调取摄像头图像。

```shell
# 开启第一摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=0 &
# 开启第二摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=1 &
```

### 1.3 偏色修正

部分CSI摄像头存在偏色问题，以下是`imx219`芯片CSI摄像头解决办法。

```shell
wget http://www.waveshare.net/w/upload/e/eb/Camera_overrides.tar.gz
tar zxvf Camera_overrides.tar.gz 
sudo cp camera_overrides.isp /var/nvidia/nvcam/settings/

sudo chmod 664 /var/nvidia/nvcam/settings/camera_overrides.isp
sudo chown root:root /var/nvidia/nvcam/settings/camera_overrides.isp
```

## 2. 服务部署

### 2.1 准备工作

请确保在进行以下工作前已经安装好了ROS2以及相应的包，这里使用了ROS2的Foxy版本。

创建ros2工程目录，如`~/ros_ws`，用户可自行指定。

``` shell
mkdir -p ~/ros_ws/src
```

将工程下载到`~/ros_ws/src`目录下，并进行编译。

``` shell
cd ~/ros_ws/src
git clone https://github.com/snowolf-zlex/CSI-Camera.git .

cd ~/ros_ws
rm -rf build install && colcon build --packages-select csi_cam_service
```

如果编译出现异常情况，可以试试如下命令。

``` shell
cd ~/ros_ws
rm -rf build install && colcon build --symlink-install --packages-select csi_cam_service
```

### 2.2 启动服务

加载项目环境：

``` shell
cd ~/ros_ws
source install/setup.bash 
```

> [!TIP]
> 也可以修改`~/.bashrc`文件：
>
> ``` shell
> echo "source ~/ros_ws/install/setup.bash“ >> ~/.bashrc
> ```

启动CSI单目摄像头，默认参数：

- video_device_id:=0
- image_size:=[640,480]
- fps:=30

``` shell
ros2 run csi_cam_service single_csi_cam_node
```

通过`video_device_id`参数指定单目摄像头设备ID，如使用`/dev/video1`。

``` shell
ros2 run csi_cam_service single_csi_cam_node --ros-args -p video_device_id:=1
```

这时就可以通过节点`single_csi_cam/image`访问到该摄像头数据，可以通过rviz2来查看图像。

这里有1个图像节点：

- /single_csi_cam/image

启动CSI双目摄像头，启动CSI单目摄像头，默认参数：

- video_device_id:=[0,1]
- image_size:=[640,480]
- fps:=30

``` shell
ros2 run csi_cam_service dual_csi_cam_node 
```

通过`video_device_id`参数指定双目摄像头设备ID，如，使用`/dev/video1`作为`dual_csi_cam/image_left`，使用`/dev/video0`作为`dual_csi_cam/image_right`。

> [!CAUTION]
> `video_device_id`必须以成对数组出现，且不能有空格。

``` shell
ros2 run csi_cam_service dual_csi_cam_node --ros-args -p video_device_id:=[1,0]
```

这里有3个图像节点：

- /dual_csi_cam/image_left：默认`/dev/video0`设备图像
- /dual_csi_cam/image_right：默认`/dev/video1`设备图像
- /dual_csi_cam/image_combine：横向整合两个图像

通过以下命令查看图像：

``` shell
ros2 run rqt_image_view rqt_image_view
```

### 2.3 常见错误

由于CSI使用的是`GStreamer`，会有内存分配问题，如下所示。

> [!WARNING]  
> (python3:6344): GStreamer-WARNING **: 11:52:06.411: Failed to load plugin '/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvarguscamerasrc.so': /lib/aarch64-linux-gnu/libGLdispatch.so.0: cannot allocate memory in static TLS block
>
> (python3:6344): GStreamer-WARNING **: 11:52:06.415: Failed to load plugin '/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvvidconv.so': /lib/aarch64-linux-gnu/libGLdispatch.so.0: cannot allocate memory in static TLS block
>

修改`~/.bashrc`文件，使其预加载：

``` shell
# 提前将库加载到内存
export LD_PRELOAD=/lib/aarch64-linux-gnu/libGLdispatch.so.0
```

### 2.4 相机标定

完成上述工作后，可以使用`camera_calibration`来做相机标定。

> [!IMPORTANT]
> 这里使用了7x10 20mm的棋盘格标定盘。
>
> 1. 实际上使用的是棋盘格内角，也就是6x9，这里很容易写错参数。
> 2. 实际打印出来的棋盘格，由于打印机设备、纸张材料等因素干扰，实际上可能不是标定的尺寸，需要根据实际情况微调参数。

执行`camera_calibration`命令，来做单目相机标定。

``` shell
# --approximate 0.1: 指定标定节点的时间间隔（以秒为单位），用于控制相邻帧之间的时间间隔。在这个命令中，标定节点将会尝试以0.1秒的间隔处理图像帧。
# --size 6x9: 棋盘格的大小。在这个命令中，棋盘格的大小为6行x9列。
# --square 0.020: 棋盘格每个方格的大小（以米为单位）。在这个命令中，每个方格的大小为0.020米。
# image:=/single_csi_cam/image ：这里使用了单目相机图像节点
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 6x9 --square 0.020 \
--ros-args --remap image:=/single_csi_cam/image \
--ros-args --remap camera:=/custom_camera
```

执行`camera_calibration`命令，来做单目相机标定。

``` shell
# 这里使用了双目相机图像节点
# left:=/dual_csi_cam/image_left 
# right:=/dual_csi_cam/image_right
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 6x9 --square 0.020 \
--ros-args --remap left:=/dual_csi_cam/image_left \
--ros-args --remap right:=/dual_csi_cam/image_right \
--ros-args --remap left_camera:=/custom_camera/image_left \
--ros-args --remap right_camera:=/custom_camera/image_right 
```

## 附件

棋盘格图像来自OpenCV。
![chessboard](https://github.com/snowolf-zlex/ROS2-CSI-Camera/assets/3873394/6fb26cc8-7664-4e47-b451-ab47405e4b72)
