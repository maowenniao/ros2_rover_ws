#!/usr/bin/env bash
# setup_can.sh - 配置 SocketCAN 接口（CANable / USB-CAN 等）
# 用法：
#   ./setup_can.sh can0 250000

set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${2:-250000}"

echo "[setup_can] 配置 ${IFACE} @ ${BITRATE} bps ..."

if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[setup_can] 错误：接口 ${IFACE} 不存在。请检查 USB-CAN 适配器是否插入。"
    echo "          可用接口：$(ls /sys/class/net | grep -E '^can' || echo '(none)')"
    exit 1
fi

# 关闭接口
sudo ip link set "$IFACE" down 2>/dev/null || true

# 设置 bitrate
sudo ip link set "$IFACE" type can bitrate "$BITRATE"
sudo ip link set "$IFACE" up

echo "[setup_can] 接口 ${IFACE} 已 UP。详细信息："
ip -details link show "$IFACE"
echo ""
echo "    现在可以运行："
echo "      candump ${IFACE}   # 监听总线"
echo "      cansend ${IFACE} 123#DEADBEEF  # 发送测试帧"