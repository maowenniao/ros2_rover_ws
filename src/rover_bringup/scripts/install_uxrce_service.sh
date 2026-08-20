#!/usr/bin/env bash
# install_uxrce_service.sh - 安装 Micro XRCE-DDS Agent 为 systemd 服务（USB Serial 模式）
#
# Pixhawk 6C 通过 USB 连接 Jetson 后，设备路径为 /dev/pixhawk（udev 规则）
# 默认波特率 921600 8N1
#
# 用法：
#   sudo ./install_uxrce_service.sh

set -euo pipefail

if [ "$(id -u)" -ne 0 ]; then
    echo "[install_uxrce_service] 请用 sudo 运行本脚本"
    exit 1
fi

AGENT_BIN="$(command -v MicroXRCEAgent || true)"
if [ -z "$AGENT_BIN" ] && [ -x /usr/local/bin/MicroXRCEAgent ]; then
    AGENT_BIN="/usr/local/bin/MicroXRCEAgent"
fi
if [ -z "$AGENT_BIN" ]; then
    echo "[install_uxrce_service] 未找到 MicroXRCEAgent，请先编译安装。"
    exit 1
fi

# 自动检测设备
DEVICE="/dev/pixhawk"
if [ ! -e "$DEVICE" ]; then
    DEVICE="/dev/ttyACM0"
fi
if [ ! -e "$DEVICE" ]; then
    echo "[install_uxrce_service] 警告：未找到 ${DEVICE}，请先运行 setup_pixhawk_usb.sh"
    DEVICE="/dev/ttyACM0"
fi

BAUDRATE=921600

cat > /etc/systemd/system/uxrce_agent.service <<EOF
[Unit]
Description=Micro XRCE-DDS Agent (Jetson <-> Pixhawk via USB Serial)
After=network-online.target dev-pixhawk.device
Wants=network-online.target

[Service]
Type=simple
ExecStart=${AGENT_BIN} serial -D ${DEVICE} -b ${BAUDRATE} -v 6
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable uxrce_agent.service
systemctl start uxrce_agent.service

echo "[install_uxrce_service] 已安装并启动 uxrce_agent.service"
echo "    设备: ${DEVICE}"
echo "    波特率: ${BAUDRATE}"
echo ""
systemctl status uxrce_agent.service --no-pager