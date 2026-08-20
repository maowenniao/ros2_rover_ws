#!/usr/bin/env bash
# setup_pixhawk_usb.sh - 检查 Jetson 与 Pixhawk 6C 的 USB 直连
#
# Pixhawk 6C 通过 USB 连接到 Jetson 时，会在 Jetson 上呈现
#   - /dev/ttyACM0  - MAVLink / uXRCE-DDS 数据通道
#   - /dev/ttyACM1  - 调试 NSH 串口（可选）
#
# 用法：
#   ./setup_pixhawk_usb.sh
#
# 输出：
#   - 检查 /dev/ttyACM* 是否存在
#   - 配置串口波特率（921600）
#   - 提示如何将 Pixhawk 加入 udev 规则（固定设备名）

set -euo pipefail

echo "[setup_pixhawk_usb] 检查 Pixhawk 6C USB 设备..."

# 1) 查找设备
if ! ls /dev/ttyACM* >/dev/null 2>&1; then
    echo "[setup_pixhawk_usb] ❌ 未找到 /dev/ttyACM* 设备！"
    echo "         请检查："
    echo "         1) Pixhawk 6C USB 是否已插入 Jetson"
    echo "         2) Jetson USB 是否启用（USB 3.0 口更稳定）"
    echo "         3) 试运行：lsusb | grep -i 'px4\|holyb'"
    exit 1
fi

echo "[setup_pixhawk_usb] 找到以下设备："
ls -la /dev/ttyACM* 2>/dev/null

# 2) 配置 udev 规则，固定设备名（推荐）
echo ""
echo "[setup_pixhawk_usb] 安装 udev 规则以固定设备名..."

# Pixhawk 6C 默认 USB 设备：VID=0x26AC PID=0x0011（或 0x0010）
# 也可能为 3D Robotics (0x26AC)
sudo tee /etc/udev/rules.d/99-pixhawk.rules >/dev/null <<'EOF'
# Pixhawk 6C / FMUv6 飞控
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", \
    SYMLINK+="pixhawk", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0010", \
    SYMLINK+="pixhawk", MODE="0666"
# 兼容老版 FMUv5
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0001", \
    SYMLINK+="pixhawk", MODE="0666"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger

# 3) 验证
sleep 1
if [ -e /dev/pixhawk ]; then
    echo "[setup_pixhawk_usb] ✅ /dev/pixhawk 软链接已创建"
else
    echo "[setup_pixhawk_usb] ⚠️ /dev/pixhawk 软链接未创建，但 /dev/ttyACM* 仍可使用"
fi

# 4) 设置波特率（通过 stty，uXRCE Agent 会自行配置）
for dev in /dev/ttyACM*; do
    if [ -e "$dev" ]; then
        sudo stty -F "$dev" 921600 cs8 -cstopb -parenb -echo -ixon \
            -ixoff -crtscts 2>/dev/null || \
        echo "[setup_pixhawk_usb] 提示：uXRCE Agent 会自行配置波特率"
    fi
done

echo ""
echo "[setup_pixhawk_usb] 现在可以启动 XRCE-DDS Agent（Serial 模式）："
echo "    MicroXRCEAgent serial -D /dev/pixhawk -b 921600 -v 6"
echo "    或：MicroXRCEAgent serial -D /dev/ttyACM0 -b 921600 -v 6"