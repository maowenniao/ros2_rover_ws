# ODrive CAN 总线测试指南

本文档介绍如何在 Jetson 上进行 ODrive CAN 通讯测试。

## 目录

1. [环境准备](#环境准备)
2. [CAN 总线配置](#can-总线配置)
3. [使用测试工具](#使用测试工具)
4. [使用 ROS2 节点测试](#使用-ros2-节点测试)
5. [Web 远程控制](#web-远程控制)
6. [常见问题](#常见问题)

---

## 环境准备

### 1. 安装依赖

```bash
# 安装 python-can
pip install python-can

# 安装 ROS2 依赖
sudo apt install python3-rclpy python3-std-msgs python3-sensor-msgs \
                 python3-nav-msgs python3-geometry-msgs python3-diagnostic-msgs
```

### 2. 安装 SocketCAN 工具

```bash
sudo apt install can-utils
```

### 3. 配置 CAN 网卡

#### 方式 A：USB 转 CAN 适配器（SocketCAN）

**常见支持的型号：**

| 品牌型号 | 芯片/驱动 | SocketCAN 支持 |
|----------|-----------|----------------|
| CANable-0 / CANable-II | CH340/FTDI (slcan) | ✅ |
| 飞奥伦蒂 FA-USB-CAN | FTDI (slcan) | ✅ |
| 致远电子 CAN-USB | 专有 | ✅ |
| Peak-PCAN-USB | PEAK (pcan) | ✅ |
| 树莓派 CANable | CH340 (slcan) | ✅ |
|  seeed-stickCAN | FTDI (slcan) | ✅ |
| 其他 FTDI/CH340 基于 | slcan | ✅ |

**步骤 1：确认设备识别**

```bash
# 插入 USB CAN 适配器后，查看设备节点
dmesg | tail -20
ls /dev/ttyUSB* /dev/ttyACM*

# 查看 USB 设备信息
lsusb
# 常见识别：
# - FTDI 设备: FTDI Technology...
# - CH340 设备: NCH340...
# - DROFT (CANable): DROFT Technologies...
```

**步骤 2：配置 CAN 接口（slcan 模式 - 最通用）**

```bash
# 安装 slcan 工具
sudo apt install slcan can-utils

# 加载内核模块
sudo modprobe can
sudo modprobe slcan

# 方式 1：设备为 /dev/ttyUSB0（常见于 CH340/FTDI 模块）
sudo slcand -s5 /dev/ttyUSB0 can0

# 方式 2：设备为 /dev/ttyACM0（常见于 CANable）
sudo slcand -s5 /dev/ttyACM0 can0

# 方式 3：如果设备名不同，替换为实际设备名
sudo slcand -s5 /dev/ttyUSBx can0

# 启动 CAN 接口（波特率 250kbps）
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up

# 检查状态
ip link show can0
```

**步骤 2 备选：如果 slcan 模式不成功，尝试 can-usb 驱动**

```bash
# 某些设备需要 can-usb 驱动
sudo modprobe can
sudo modprobe can-usb

# 接口自动创建为 can0
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
```

**步骤 3：验证 CAN 通信**

```bash
# 监听 CAN 消息（ODrive 会周期性发送心跳）
candump can0

# 应该能看到类似输出：
# can0  0x01   [1]  01
# can0  0x81   [1]  01

# 如果没有任何输出，检查设备节点权限
sudo chmod 666 /dev/ttyUSB0  # 或 /dev/ttyACM0
```

**设置 udev 规则（可选，避免每次用 sudo）：**

```bash
# 添加当前用户到 dialout 组（允许访问串口设备）
sudo usermod -a -G dialout $USER

# 创建 udev 规则（根据不同设备调整）
# FTDI 设备：
sudo tee /etc/udev/rules.d/99-usb-can.rules << EOF
SUBSYSTEM=="tty", ATTRS{"idVendor"}=="0403", ATTRS{"idProduct"}=="6001", MODE="0666"
EOF

# CH340 设备：
sudo tee /etc/udev/rules.d/99-usb-can.rules << EOF
SUBSYSTEM=="tty", ATTRS{"idVendor"}=="1a86", ATTRS{"idProduct"}=="7523", MODE="0666"
EOF

sudo udevadm trigger
sudo udevadm control --reload-rules

# 重新插拔 USB 设备后生效
```

**设置 udev 规则（可选，避免每次用 sudo）：**

```bash
# 添加当前用户到 dialout 组（允许访问 /dev/ttyACM*）
sudo usermod -a -G dialout $USER

# 或者创建 udev 规则
sudo tee /etc/udev/rules.d/99-canable.rules << EOF
SUBSYSTEM=="tty", ATTRS{"idVendor"}=="1a86", ATTRS{"idProduct"}=="7523", MODE="0666"
EOF
sudo udevadm trigger
sudo udevadm control --reload-rules

# 重新插拔 USB 设备后生效
```

**步骤 2b：使用特定驱动（如 Peak-PCAN）**

```bash
# 安装 PEAK-PCAN 驱动
# 从 https://www.peak-system.com/filedownload/peak-linux-driver.9.12.0.zip 下载
unzip peak-linux-driver.9.12.0.zip
cd peak-linux-driver.9.12.0
make
sudo make install
sudo modprobe can-peak

# 启动接口
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
```

**步骤 2c：使用 can-usb 驱动（如 can-usb 板）**

```bash
sudo modprobe can
sudo modprobe can-usb

# 接口自动创建为 can0
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up
```

#### 方式 B：MCP2515 SPI CAN 接口（树莓派/Jetson 板载 SPI）

```bash
# 加载内核模块
sudo modprobe can
sudo modprobe can-raw
sudo modprobe can-bcm
sudo modprobe mcp251x

# 启动 CAN 接口（波特率 250kbps）
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up

# 检查状态
ip link show can0
```

### 4. 验证 CAN 接口

```bash
# 监听 CAN 消息（ODrive 会周期性发送心跳）
candump can0

# 应该能看到类似输出：
# can0  0x01   [1]  01
```

---

## Web 远程控制

### 1. 安装依赖

```bash
pip install flask python-can
```

### 2. 启动 Web 服务器

```bash
# ROS2 方式（需要先 source 工作空间）
ros2 run odrive_can_bridge web_controller.py

# 或直接运行
cd src/odrive_can_bridge
python3 odrive_can_bridge/web_controller.py
```

### 3. 访问控制界面

```bash
# 本地访问
# 浏览器打开: http://localhost:5000

# Jetson 远程访问
# 浏览器打开: http://<Jetson-IP>:5000
```

### 4. 控制功能

| 功能 | 说明 |
|------|------|
| 方向控制 | 前进/后退/左转/右转/停止 |
| 速度调节 | 0-100% 滑块调节 |
| 速度模式 | 方向控制 + 速度调节 |
| 位置模式 | 输入目标位置（圈数）直接控制 |
| 急停按钮 | 一键急停所有电机 |
| 设备状态 | 实时显示 ODrive 状态 |

### 5. 键盘快捷键

| 按键 | 功能 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| 空格 | 停止 |
| E | 急停 |

### 6. 预览模式（无需硬件）

```bash
# 直接在浏览器打开预览页面
# 位置: src/odrive_can_bridge/preview_control_panel.html
# 或使用本地服务器
python3 -m http.server 8080 --directory src/odrive_can_bridge/
# 浏览器打开: http://localhost:8080/preview_control_panel.html
```

---

## 使用测试工具

### 方式一：交互式命令行

```bash
# ROS2 运行方式
ros2 run odrive_can_bridge can_bus_test.py

# 或直接运行
python3 odrive_can_bridge/can_bus_test.py
```

**可用命令：**

| 命令 | 说明 |
|------|------|
| `scan` | 扫描 CAN 总线上的 ODrive 设备 |
| `status [nid]` | 显示设备状态（可指定 nid） |
| `status_all` | 显示所有设备状态 |
| `clear_error nid` | 清除指定设备的错误 |
| `setup nid` | 完整设置设备（清错→设模式→进闭环） |
| `vel_test nid v s` | 速度测试 (v=速度 turn/s, s=秒) |
| `full_test [nids]` | 完整测试序列 |
| `rx` | 开始接收 CAN 消息 |
| `quit` | 退出 |

**示例交互：**

```
odrive-test> scan

============================================================
扫描 CAN 总线上的 ODrive 设备...
============================================================

监听心跳消息 (5 秒)...
  发现 ODrive nid=0
  发现 ODrive nid=2

发现 2 个 ODrive 设备: [0, 2]

odrive-test> status_all

============================================================
ODrive 设备状态总览
============================================================

==================================================
ODrive Node ID: 0
==================================================
  在线状态: [在线]
  轴状态: CLOSED_LOOP_CONTROL
  运行时间: 12.3s
  心跳计数: 61

  编码器:
    位置: 1.2345 turns
    速度: 0.0012 turn/s

  电源:
    总线电压: 16.80 V
    总线电流: 0.15 A

odrive-test> setup 0
  已设置 nid=0 为速度闭环控制

odrive-test> vel_test 0 0.5 3
测试速度命令: nid=0, vel=0.5 turn/s, 持续 3.0s
  已发送进入闭环控制命令
  已发送停止命令
```

### 方式二：命令行模式

```bash
# 扫描模式
ros2 run odrive_can_bridge can_bus_test.py --mode scan

# 测试模式
ros2 run odrive_can_bridge can_bus_test.py --mode test

# 指定 CAN 通道和波特率
ros2 run odrive_can_bridge can_bus_test.py -c can0 -b 250000 -n 0 1 2 3
```

---

## 使用 ROS2 节点测试

### 1. 启动完整系统

```bash
# 启动 ODrive CAN 桥接 + 里程计 + 速度控制
ros2 launch odrive_can_bridge odrive_velocity_only.launch.py
```

### 2. 检查话题

```bash
# 查看所有发布的话题
ros2 topic list

# 应该能看到：
# /wheel_cmd
# /wheel_state
# /wheel_odom
# /odrive_diagnostic
# /odrive_online
# /odrive_error_info
# /cmd_vel
```

### 3. 发送速度命令

```bash
# 方式一：使用 ros2 topic pub
# 前进 1 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  --rate 10

# 原地旋转 1 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}' \
  --rate 10

# 方式二：使用键盘控制（需要 teleop 包）
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. 查看 ODrive 状态

```bash
# 查看诊断信息
ros2 topic echo /odrive_diagnostic

# 查看在线状态
ros2 topic echo /odrive_online

# 查看轮速状态
ros2 topic echo /wheel_state

# 查看里程计
ros2 topic echo /wheel_odom
```

### 5. 查看 TF 树

```bash
# 查看坐标变换树
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

---

## 使用 can-utils 工具

### candump - 监听 CAN 消息

```bash
candump can0
```

### cansend - 发送 CAN 消息

```bash
# ODrive CAN ID 计算公式：arb_id = (cmd_id << 6) | node_id

# 发送心跳请求到 nid=0 (cmd_id=0x01)
# arb_id = (0x01 << 6) | 0 = 0x040
cansend can0 040#0102030405060708

# 发送清错命令到 nid=0 (cmd_id=0x1E)
# arb_id = (0x1E << 6) | 0 = 0x780
cansend can0 780

# 发送进入闭环命令到 nid=0 (cmd_id=0x06, state=0x08)
# arb_id = (0x06 << 6) | 0 = 0x180
cansend can0 18008000000
```

### candump 过滤特定节点

```bash
# 只查看 nid=0 的消息
candump can0,0x3FF:0x7FF

# 只查看 nid=0 和 nid=1 的心跳
candump can0,0x040:0x07F,0x0C0:0x0FF
```

---

## 常见问题

### 1. 无法连接 CAN 接口

```bash
# 检查模块是否加载
lsmod | grep can

# 检查接口是否存在
ip link show

# 重新加载驱动
sudo modprobe mcp251x
```

### 2. ODrive 无响应

```bash
# 检查 ODrive 是否通电（LED 应该亮起）
# 检查 CAN 接线（CAN_H 和 CAN_L）
# 检查终端电阻（CAN 总线两端应有 120Ω 电阻）

# 使用 canbus-check 工具
sudo pip install can-isotp
```

### 3. ODrive 报错

```bash
# 查看错误信息
ros2 topic echo /odrive_error_info

# 常见错误：
# - Phase Resistance OOR: 电机未连接或相电阻超限
# - Index Not Found: 编码器索引搜索失败
# - Drivers Over Temp: 驱动器过热
```

### 4. 轮子不转

```bash
# 检查 ODrive 是否进入闭环状态
ros2 topic echo /odrive_diagnostic

# 手动设置闭环
ros2 run odrive_can_bridge can_bus_test.py
odrive-test> setup 0

# 检查 /wheel_cmd 是否有发布
ros2 topic echo /wheel_cmd
```

### 5. 里程计漂移

```bash
# 检查轮径参数是否正确
ros2 param get /wheel_odometry_node wheel_radius

# 调整协方差
# 编辑 config/odrive_can_params.yaml 中的协方差参数
```

---

## 完整测试流程

```bash
# 1. 配置 CAN 接口
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up

# 2. 启动 ODrive 系统
ros2 launch odrive_can_bridge odrive_velocity_only.launch.py &

# 3. 在另一个终端运行测试工具
ros2 run odrive_can_bridge can_bus_test.py

# 4. 扫描设备
odrive-test> scan

# 5. 查看状态
odrive-test> status_all

# 6. 设置设备
odrive-test> setup 0
odrive-test> setup 1
odrive-test> setup 2
odrive-test> setup 3

# 7. 测试速度控制
odrive-test> vel_test 0 0.5 2

# 8. 在另一个终端发送 cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {x: 0.0, y: 0.0, z: 0.0}}' \
  --rate 10 -1

# 9. 查看里程计
ros2 topic echo /wheel_odom

# 10. 停止
odrive-test> vel_test 0 0.0 0.1
```

---

## 附录：ODrive CAN 协议速查

| 命令 | cmd_id | CAN ID (nid=0) | 说明 |
|------|--------|----------------|------|
| Heartbeat | 0x01 | 0x040 | 心跳（ODrive 主动发送）|
| Estop | 0x02 | 0x080 | 急停 |
| Set_Axis_State | 0x06 | 0x180 | 设置轴状态 |
| Get_Encoder_Est | 0x07 | 0x1C0 | 读取编码器 |
| Set_Controller_Mode | 0x09 | 0x280 | 设置控制模式 |
| Set_Input_Vel | 0x0C | 0x380 | 设置速度 |
| Clear_Errors | 0x1E | 0x780 | 清除错误 |
| Get_Bus_V_I | 0x1D | 0x740 | 读取总线电压/电流 |

**轴状态：**
- `0x01` = IDLE
- `0x08` = CLOSED_LOOP_CONTROL

**控制模式：**
- `0x00` = VOLTAGE
- `0x01` = TORQUE
- `0x02` = VELOCITY
- `0x03` = POSITION