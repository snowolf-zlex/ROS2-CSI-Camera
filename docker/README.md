# Docker模块

基于nvcr.io/nvidia/l4t-base:r36.2.0的docker镜像，构建了包含Humbled ROS2环境，并在此基础上构建了CSI双目摄像头实现。

在根目录下，使用以下命令完成镜像构建：

``` shell
./build.sh
```

在`src/docker/init`目录中，有完整的镜像构建过程。使用以下命令可以运行该镜像：

``` shell
./run.sh
```

通过以下命令可以测试GSreamer时是否正常：

``` shell
sh ./test.sh
```

``` shell
python3 multi_csi_camera.py
```

也可以直接运行ROS2 Node：

``` shell
ros2 launch csi_cam_service stereo_cam.launch.py
```
