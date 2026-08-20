# 四轴差动无人小车 ROS 2 工作空间（V3.1）

> Jetson Orin Nano + Pixhawk 6C（**USB 直连**）+ 2× ODrive V3.6（**USB-CAN**）+ Intel D435i  
> **Ubuntu 24.04 Noble + ROS 2 Jazzy Jalisco**（LTS）  
> PX4 `SYS_AUTOSTART = 50000` = **Generic Rover Differential**（PX4 原生 4 轮差速）

---

## 📦 工作空间结构（V3.1）

```
ros2_rover_ws/
├── src/
│   ├── diffdrive_rover/         # 差速车控制：cmd_vel → PX4 TrajectorySetpoint
│   ├── odrive_can_bridge/       # ODrive CAN 总线桥接（python-can）
│   ├── d435i_launch/            # D435i 相机启动封装
│   ├── rover_description/       # ⭐ URDF 3D 模型 + 关节桥接
│   ├── rover_bringup/           # ⭐ 系统总启动 + 诊断脚本 + ARM/Estop
│   └── (外部依赖)
│       ├── px4_msgs/            # PX4 消息定义（git clone）
│       └── px4_ros_com/         # PX4 ROS 2 桥接示例（git clone）
```

---

## 🚀 快速开始

### 1. 一次性环境准备（Ubuntu 24.04 + ROS 2 Jazzy）

```bash
# 系统依赖
sudo apt update
sudo apt install -y git curl wget vim net-tools iputils-ping \
                    iproute2 can-utils udev python3-pip \
                    software-properties-common
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions \
                    python3-rosdep \
                    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                    ros-jazzy-joint-state-publisher-gui \
                    ros-jazzy-realsense2-camera ros-jazzy-realsense2-description \
                    ros-jazzy-rviz2 ros-jazzy-rmw-cyclonedds-cpp
pip3 install python-can pyulog empy==3.3.4

# 自动 source
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init && rosdep update

# 编译 Micro XRCE-DDS Agent（Serial 模式）
mkdir -p ~/tools && cd ~/tools
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && git submodule update --init --recursive
mkdir build && cd build && cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig
```

### 2. 准备源代码

```bash
mkdir -p ~/ros2_rover_ws/src
cd ~/ros2_rover_ws/src

# 本项目的 5 个包直接拷贝到此目录
# 然后克隆 PX4 上游：
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
git clone https://github.com/PX4/px4_ros_com.git -b release/1.14
```

### 3. 构建

```bash
cd ~/ros2_rover_ws
rosdep install --from-paths src --ignore-src -y --rosdistro=jazzy
colcon build --symlink-install
source install/setup.bash
```

### 4. 硬件配置

```bash
# 4.1 USB 直连 Pixhawk 6C（创建 /dev/pixhawk 软链接）
sudo ./src/rover_bringup/scripts/setup_pixhawk_usb.sh

# 4.2 USB-CAN 转接板 → can0
sudo ./src/rover_bringup/scripts/setup_can.sh can0 250000

# 4.3 （可选）安装 XRCE Agent 为 systemd 服务
sudo ./src/rover_bringup/scripts/install_uxrce_service.sh
```

### 5. 启动系统

```bash
# 一键启动
ros2 launch rover_bringup full_system.launch.py

# 包含 RViz（看 3D 模型 + 传感器）
ros2 launch rover_bringup full_system.launch.py enable_rviz:=true

# 自定义 Pixhawk 设备
ros2 launch rover_bringup full_system.launch.py pixhawk_device:=/dev/ttyACM1
```

启动顺序：
1. **T+0.0s**：SocketCAN + 检查 `/dev/pixhawk`
2. **T+0.5s**：Micro XRCE-DDS Agent (Serial @ 921600)
3. **T+1.0s**：URDF robot_state_publisher + joint_state_bridge
4. **T+1.5s**：D435i 相机
5. **T+2.0s**：差速车控制 + PX4 里程计桥接
6. **T+2.5s**：ODrive CAN 桥接 + 轮速里程计
7. **T+3.5s**：RViz

### 6. 单独调试各模块

```bash
ros2 launch rover_description rover_description.launch.py   # 仅 URDF
ros2 launch diffdrive_rover diffdrive.launch.py              # 仅差速车控制
ros2 launch odrive_can_bridge odrive_can.launch.py            # 仅 ODrive CAN
ros2 launch d435i_launch d435i.launch.py                    # 仅 D435i
```

---

## 🛠 调试与诊断

| 工具 | 命令 | 用途 |
|------|------|------|
| USB Pixhawk | `sudo ./scripts/setup_pixhawk_usb.sh` | 检查 /dev/pixhawk |
| CAN 总线监听 | `python3 scripts/can_bus_test.py can0` | 验证 ODrive 心跳 |
| PX4 链路检查 | `python3 scripts/px4_ros2_check.py` | 验证 uXRCE-DDS |
| 命令观察 | `python3 scripts/cmdvel_teleop_test.py` | 观察 cmd_vel 与 setpoint |
| 解锁 | `ros2 run rover_bringup arm_rover` | 解锁 PX4 |
| 急停 | `ros2 run rover_bringup estop_rover` | Disarm + 0 速度 |

---

## 📋 关键话题（V3.1）

| 话题 | 方向 | 类型 |
|------|------|------|
| `/cmd_vel` | 输入 | geometry_msgs/Twist |
| `/fmu/out/vehicle_odometry` | PX4→ROS | px4_msgs/VehicleOdometry |
| `/fmu/out/vehicle_status` | PX4→ROS | px4_msgs/VehicleStatus |
| `/fmu/in/trajectory_setpoint` | ROS→PX4 | px4_msgs/TrajectorySetpoint |
| `/fmu/in/vehicle_command` | ROS→PX4 | px4_msgs/VehicleCommand |
| `/wheel_cmd` | ROS 内部 | std_msgs/Float32MultiArray |
| `/wheel_state` | ROS 内部 | sensor_msgs/JointState |
| `/joint_states` | ROS 内部 | sensor_msgs/JointState（URDF） |
| `/robot_description` | ROS 内部 | std_msgs/String（URDF 内容） |
| `/odom` | ROS 内部 | nav_msgs/Odometry |
| `/wheel_odom` | ROS 内部 | nav_msgs/Odometry |

---

## ⚙️ 参数调节

- **物理参数**：编辑 `src/diffdrive_rover/config/diffdrive_params.yaml`
- **CAN 配置**：编辑 `src/odrive_can_bridge/config/odrive_can_params.yaml`
- **URDF 尺寸**：编辑 `src/rover_description/urdf/rover.urdf.xacro`

---

## 🚨 安全提示

1. **ODrive 首次上电：电机必须悬空！**
2. **Pixhawk 必须先处于 Offboard 模式才会响应 setpoint**
3. **PX4 机架必须选 Generic Rover Differential（SYS_AUTOSTART=50000）**
4. 紧急情况：`ros2 run rover_bringup estop_rover`

---

## 🔄 V1 → V2 → V3.1 变更摘要

| 项 | V1 | V2 | **V3.1** |
|----|----|----|----|
| Ubuntu | 22.04 | 22.04 | **24.04 Noble** |
| ROS 2 | Humble | Humble | **Jazzy Jalisco** |
| Jetson↔Pixhawk | 以太网 | USB 直连 | USB 直连 |
| Jetson↔ODrive | SocketCAN | USB-CAN | USB-CAN |
| PX4 机架 | 1010 | 10002 | **50000（Generic Rover Differential）** |
| 3D 模型 | 无 | URDF | URDF |
| launch 节点 | 5 | 7 | 7 |

---

更多信息见 `../Jetson_Pixhawk6C_通信与ROS2配置指南.md`（V3.1）。