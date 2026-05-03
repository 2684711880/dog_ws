#!/bin/bash
# ============================================
# 四足机器人 CAN 接口启动脚本
# 初始化 can0, can1, can2, can3
# ============================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

CAN_BITRATE=1000000
CAN_SAMPLE_POINT=0.75
CAN_INTERFACES=(can0 can1 can2 can3)

echo -e "${YELLOW}============================================${NC}"
echo -e "${YELLOW}  四足机器人 CAN 接口初始化${NC}"
echo -e "${YELLOW}============================================${NC}"

for can_if in "${CAN_INTERFACES[@]}"; do
    echo -n "正在配置 ${can_if} ... "
    
    if sudo ip link set "${can_if}" type can bitrate "${CAN_BITRATE}" sample-point "${CAN_SAMPLE_POINT}" 2>/dev/null; then
        sudo ip link set "${can_if}" up 2>/dev/null
        echo -e "${GREEN}✓ 成功${NC} (bitrate=${CAN_BITRATE}, sample-point=${CAN_SAMPLE_POINT})"
    else
        echo -e "${RED}✗ 失败${NC}"
        exit 1
    fi
done

echo ""
echo -e "${GREEN}所有 CAN 接口初始化完成！${NC}"
echo ""

echo -e "${YELLOW}当前 CAN 接口状态:${NC}"
ip -details link show | grep -A 4 "^[0-9]*: can"
