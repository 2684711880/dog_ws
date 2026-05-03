#!/bin/bash
# ============================================
# 日志清理脚本
# 清理 log 目录
# ============================================

set -e

YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${YELLOW}清理构建日志...${NC}"

if [ -d "/home/a/dog/dog_ws/log" ]; then
    sudo rm -rf /home/a/dog/dog_ws/log
    echo -e "${GREEN}✓ 已清理${NC}"
else
    echo "log 目录不存在"
fi
