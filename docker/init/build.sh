#!/bin/bash
# ==============================================================================
# CSI 相机节点镜像构建脚本
# ==============================================================================

set -e

GREEN='\033[0;32m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo ""
echo "========================================"
echo "构建 CSI 相机节点镜像"
echo "========================================"
echo ""

# 构建
docker build \
    -f "${SCRIPT_DIR}/Dockerfile" \
    -t l4t-ros2-csi-node:latest \
    "${PROJECT_ROOT}"

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}构建完成!${NC}"
    echo ""
    docker images l4t-ros2-csi-node:latest
else
    echo "构建失败!"
    exit 1
fi
