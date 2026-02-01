#!/bin/bash
# ==============================================================================
# Docker 镜像统一构建脚本
# ==============================================================================
# 用法: ./build.sh [选项]
#
# 选项:
#   all     - 构建所有镜像 (默认)
#   base    - 仅构建基础镜像
#   init    - 仅构建 CSI 相机节点镜像
#   jupyter - 仅构建 Jupyter 镜像
#
# 示例:
#   ./build.sh          # 构建所有镜像
#   ./build.sh base     # 仅构建基础镜像
#   ./build.sh init     # 仅构建 CSI 节点镜像
#   ./build.sh jupyter  # 仅构建 Jupyter 镜像
# ==============================================================================

set -e

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

TARGET=${1:-all}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo ""
    echo "========================================"
    echo "$1"
    echo "========================================"
}

# 显示帮助
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  all     - 构建所有镜像 (默认)"
    echo "  base    - 仅构建基础镜像 (l4t-humble-base)"
    echo "  init    - 仅构建 CSI 相机节点镜像 (l4t-ros2-csi-node)"
    echo "  jupyter - 仅构建 Jupyter 镜像 (l4t-jupyter)"
    echo ""
    echo "镜像构建顺序: base -> init -> jupyter"
    echo ""
}

# 构建基础镜像
build_base() {
    print_header "构建基础镜像 (l4t-humble-base)"
    bash "${SCRIPT_DIR}/docker/base/build.sh"
}

# 构建初始化镜像
build_init() {
    print_header "构建 CSI 相机节点镜像 (l4t-ros2-csi-node)"
    bash "${SCRIPT_DIR}/docker/init/build.sh"
}

# 构建 Jupyter 镜像
build_jupyter() {
    print_header "构建 Jupyter 镜像 (l4t-jupyter)"
    bash "${SCRIPT_DIR}/docker/jupyter/build.sh"
}

# 主逻辑
case $TARGET in
    all)
        print_header "Docker 镜像构建"
        print_info "构建顺序: base -> init -> jupyter"
        echo ""
        build_base
        build_init
        build_jupyter
        print_header "所有镜像构建完成!"
        echo ""
        print_info "构建的镜像:"
        docker images | grep -E "l4t-humble-base|l4t-ros2-csi-node|l4t-jupyter" || true
        ;;
    base)
        build_base
        ;;
    init)
        build_init
        ;;
    jupyter)
        build_jupyter
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "未知选项: ${TARGET}"
        echo ""
        show_help
        exit 1
        ;;
esac
