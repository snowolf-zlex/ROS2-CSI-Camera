#!/bin/bash
# ==============================================================================
# ROS2 Humble 基础镜像构建脚本
# ==============================================================================
# 用法: ./build.sh [目标] [标签]
#
# 构建目标:
#   standard - 完整开发环境 (默认)
#   slim     - 精简镜像
#   runtime  - 最小运行时
# ==============================================================================

set -e

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

TARGET=${1:-standard}
TAG=${2:-latest}
IMAGE_NAME="l4t-humble-base"

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
}

# 显示配置
print_header "构建 ROS2 Humble 基础镜像"
print_info "目标: ${TARGET}"
print_info "镜像: ${IMAGE_NAME}:${TAG}"
echo ""

# 确认 (非交互模式: YES=true 或 CI 环境变量存在时跳过)
if [ "${YES}" != "true" ] && [ -z "$CI" ]; then
    read -p "是否继续构建? [Y/n] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]] && [[ ! -z $REPLY ]]; then
        echo "构建已取消"
        exit 0
    fi
fi

# 构建
print_header "开始构建"
CMD="docker build"
if [ "$TARGET" != "standard" ]; then
    CMD="$CMD --target $TARGET"
fi

$CMD \
    -f "${SCRIPT_DIR}/Dockerfile" \
    -t "${IMAGE_NAME}:${TAG}" \
    --build-arg TZ=Asia/Shanghai \
    --build-arg UBUNTU_CODENAME=jammy \
    "${PROJECT_ROOT}"

if [ $? -eq 0 ]; then
    print_header "构建完成!"
    docker images "${IMAGE_NAME}:${TAG}"
else
    echo -e "${RED}构建失败!${NC}"
    exit 1
fi
