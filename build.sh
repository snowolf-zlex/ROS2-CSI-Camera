cd ./docker/base/ && ./build.sh && cd ../../

docker build -t l4t_ros2_csi_node:latest -f ./docker/init/Dockerfile .