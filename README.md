# ROS2-CSI-Camera

CSI单目及双目摄像头ROS2模块，适用于Raspberry Pi和Jetson等ARM平台。主要用于ROS2中图像节点发布，使其能够像使用USB_CAM一样方便，提供单双目原始图像、压缩图像和深度图像(BM、SGBM算法），支持相机标定和校准。

> [!NOTE]  
> 当前实现参考了[v4l2_camera](https://github.com/tier4/ros2_v4l2_camera)
>
> ``` shell
> ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video0" -p image_size:=[1280,720]
> ```

**似乎`v4l2_camera`对于CSI设备的支持不是很友好，为了能借助ROS camera_calibration 工具来实现相机标定，于是我写了这个项目。**

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

> [!NOTE]  
> CSI摄像头连接不是很稳定，即便没有任何物理碰触，也有可能重开机后找不到设备。
> 如果发现程序突然打不开摄像头，可以先看看是不是系统里找不到摄像头了。

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
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=0 
```

```shell
# 开启第二摄像头
DISPLAY=:0.0 nvgstcapture-1.0 --sensor-id=1 
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

#### 2.2.1 单目摄像头(Monocular Camera)

CSI单目摄像头节点共有3个图像话题：

- /mono_cam/image_raw: 原始图像
- /mono_cam/image_compressed: 压缩图像
- /mono_cam/camera_info: 摄像头设备信息

启动CSI单目摄像头，默认参数：

- video_device_id:=0
- image_size:=[640,480]
- fps:=30
- calibration_file_path:=? 支持通过外部相机标定文件校准镜头

``` shell
ros2 run csi_cam_service mono_cam_node
```

通过`video_device_id`参数指定单目摄像头设备ID，如使用`/dev/video1`。

``` shell
ros2 run csi_cam_service mono_cam_node --ros-args -p video_device_id:=1
```

这时就可以通过`/mono_cam/image_raw`访问到该摄像头数据，可以通过rviz2来查看图像。

#### 2.2.2 双目摄像头(Stereo Camera)

CSI双目摄像头节点，共有3个图像话题：

- /stereo_cam/image_left_raw：默认`/dev/video0`设备图像
- /stereo_cam/image_right_raw：默认`/dev/video1`设备图像
- /stereo_cam/image_combine_raw：横向整合两个图像
- /stereo_cam/image_compressed：压缩图像（左侧摄像头）
- /stereo_cam/image_compressed_depth：压缩及深度图像
- /stereo_cam/camera_info：设备信息（左侧摄像头）

默认参数：

- video_device_id:=[0,1]
- image_size:=[640,480]
- fps:=30
- calibration_file_path:=[,] 支持通过外部相机标定文件校准镜头
- algorithm_type:=bm 提供深度算法支持，默认BM算法，还可以选择SGBM

``` shell
ros2 run csi_cam_service stereo_cam_node 
```

通过`video_device_id`参数指定双目摄像头设备ID，如，使用`/dev/video1`作为`/stereo_cam/image_left`，使用`/dev/video0`作为`/stereo_cam/image_right`。

> [!CAUTION]
> `video_device_id`必须以成对数组出现，且不能有空格。

``` shell
ros2 run csi_cam_service stereo_cam_node --ros-args -p video_device_id:=[1,0]
```

这里提供了launch运行方式，将使用内置的相机标定文件（config中）进行校准，以便实现深度图像。

``` shell
ros2 launch csi_cam_service stereo_cam.launch.py 
```

通过以下命令可查看图像：

``` shell
ros2 run rqt_image_view rqt_image_view
```

使用BM算法默认参数获得深度图像（比耶），参数有待优化：

![2024-03-12 11-28-49 的屏幕截图](https://github.com/snowolf-zlex/ROS2-CSI-Camera/assets/3873394/dd92553c-a415-4e29-8ef6-bd1880b6a532)

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
# image:=/mono_cam/image_raw: 这里使用了单目相机节点和图像话题
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 6x9 --square 0.020 \
--ros-args --remap image:=/mono_cam/image_raw \
--ros-args --remap camera:=/custom_camera
```

单目相机标定过程：

![2024-03-10 10-25-26 的屏幕截图](https://github.com/snowolf-zlex/ROS2-CSI-Camera/assets/3873394/8747eba1-1f38-4a70-a579-a6d621e8d833)

下面执行`camera_calibration`命令做双目相机标定。

``` shell
# 这里使用了双目相机节点和图像话题
# left:=/stereo_cam/image_left_raw
# right:=/stereo_cam/image_right_raw
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 6x9 --square 0.020 \
--ros-args --remap left:=/stereo_cam/image_left_raw \
--ros-args --remap right:=/stereo_cam/image_right_raw \
--ros-args --remap left_camera:=/custom_camera/image_left \
--ros-args --remap right_camera:=/custom_camera/image_right 
```

双目摄像头标定:

![2024-03-12 13-12-27 的屏幕截图](https://github.com/snowolf-zlex/ROS2-CSI-Camera/assets/3873394/8fd74376-16a4-4f5e-876a-eb59de12017a)


完成标定后，提取标定文件，并移动到需要的位置。

``` shell
# 提取标定参数文件
mkdir -p ~/calibration
cd ~/calibration
mv /tmp/calibrationdata.tar.gz .

# 解压缩
tar vxzf calibrationdata.tar.gz

#这里能找到left.yaml 和 right.yaml两个文件，将其移动到需要位置即可
ll *.yaml
```

## 附件

棋盘格图像来自OpenCV。
![chessboard](https://github.com/snowolf-zlex/ROS2-CSI-Camera/assets/3873394/6fb26cc8-7664-4e47-b451-ab47405e4b72)
