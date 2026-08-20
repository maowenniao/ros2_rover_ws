# ODrive CAN Bridge 模块 - 文件结构说明

## 项目根目录

```
odrive_can_bridge/
├── package.xml                    # ROS2 包描述文件
├── setup.py                       # Python 包安装配置
├── setup.cfg                      # 设置配置
├── preview_control_panel.html     # 独立预览页面（可在本地浏览器直接打开）
├── README_TESTING.md              # 测试指南文档
├── resource/                      # 包资源目录
│   └── odrive_can_bridge          # ros2 run 入口标记
```

## 配置文件

```
config/
└── odrive_can_params.yaml         # ODrive CAN 参数配置
   ├── CAN 总线设置 (bitrate: 250kbps)
   ├── 节点 ID 映射 (0-3)
   ├── 电机参数 (极对数、齿数、总减速比)
   ├── 轮径参数 (0.085m)
   └── PID 参数 (位置/速度环)
```

## Launch 文件

```
launch/
├── odrive_can.launch.py           # 完整启动文件
│   └── 启动 odrive_can_node + wheel_odometry_node
└── odrive_velocity_only.launch.py # 简化速度控制启动文件
    └── 仅启动 odrive_velocity_control_node
```

## Python 源代码

```
odrive_can_bridge/
├── __init__.py                    # Python 包初始化文件
├── odrive_can_node.py             # CAN 通信核心节点
│   ├── 订阅 /odrive_commands 自定义消息
│   ├── 通过 CAN 总线发送 ODrive 指令
│   └── 接收编码器反馈并发布到 /odom
├── wheel_odometry_node.py         # 轮式里程计节点
│   ├── 订阅 CAN 编码器数据
│   └── 发布 nav_msgs/msg/Odometry
├── odrive_velocity_control_node.py # 简化速度控制节点
│   ├── 订阅 geometry_msgs/msg/Twist
│   └── 直接转换为 ODrive CAN 指令
├── web_controller.py              # Web 控制服务器 (Flask)
│   ├── HTTP API 接口
│   ├── 实时状态推送 (SSE)
│   └── 方向/速度/位置控制指令
├── can_bus_test.py                # CAN 总线测试工具
│   ├── 扫描可用 ODrive 设备
│   ├── 发送测试指令
│   └── 验证 CAN 通信
└── templates/
    ├── index.html                 # Web 控制前端主页面
    └── preview_control_panel.html # 独立预览页面副本
```

## 文件功能概览

| 文件 | 功能 | 使用场景 |
|------|------|----------|
| **odrive_can_node.py** | CAN 通信核心 | 完整系统，需自定义消息类型 |
| **odrive_velocity_control_node.py** | 速度控制 | 简化模式，订阅标准 Twist 消息 |
| **wheel_odometry_node.py** | 里程计计算 | 与 CAN 节点配合，提供位姿估计 |
| **web_controller.py** | Web 控制服务器 | 远程网页控制小车 |
| **can_bus_test.py** | CAN 测试工具 | 调试和验证 CAN 硬件 |
| **templates/index.html** | Web 前端 | 连接硬件时的控制界面 |
| **preview_control_panel.html | 预览页面 | 本地查看 UI 效果，无需硬件 |
| **odrive_can_params.yaml** | 参数配置 | 修改电机、轮径、CAN 参数 |

## 快速启动

### 1. 预览 Web 界面（无需硬件）
```bash
# 在浏览器中直接打开
start preview_control_panel.html
```

### 2. 启动速度控制（需 CAN 硬件）
```bash
# 配置 CAN 接口
sudo ip link set can0 up type can bitrate 250000
sudo ifconfig can0 up

# 启动速度控制节点
ros2 run odrive_can_bridge odrive_velocity_control_node

# 另开终端发送测试指令
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}' --once
```

### 3. 启动 Web 控制服务器
```bash
python3 src/odrive_can_bridge/odrive_can_bridge/web_controller.py
# 浏览器访问 http://localhost:5000
```

## 依赖关系

```
web_controller.py ──┐
                    │
                    ├─→ odrive_velocity_control_node.py ←── /cmd_vel (Twist)
                    │                                  │
                    └─→ wheel_odometry_node.py ←── 编码器数据 (odom)
                                       │
                               odrive_can_node.py ←── CAN 总线 (250kbps)
                                               │
                                            ODrive x4
```

## 注意事项

1. **目录结构**: 项目实际路径为 `ros2_rover_ws-main/ros2_rover_ws-main/...`（嵌套目录）
2. **CAN 硬件**: 需要 Jetson 配备 CAN 控制器或 USB-CAN 适配器
3. **ODrive 配置**: 默认使用 CAN 2.0 协议，波特率 250kbps
4. **Web 控制**: 需要安装 `flask` 和 `python-can` 库
5. **预览页面**: `preview_control_panel.html` 可在本地浏览器直接打开，无需连接硬件
