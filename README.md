# 四轮差速车 ROS 2 工作空间

Jetson Orin Nano + Pixhawk 6C（USB，仅解锁 / 急停 / 状态）+ 两块 ODrive V3.6（USB-CAN，四轴）+ Intel D435i。

Ubuntu 24.04 + ROS 2 Jazzy。电机由 Jetson 做差速解算后直接走 CAN，不把 `/cmd_vel` 交给 PX4 的 rover 控制器。

## 控制链

```
/cmd_vel_nav ──可选 safety_cmdvel_node──► /cmd_vel
                                            │
                                            ▼
                              odrive_velocity_control   ← 唯一的 /wheel_cmd 发布者
                                            │
                                            ▼
                              odrive_can_node ──CAN──► ODrive
                                            │
                                            ▼
                              /wheel_state ──► wheel_odometry ──► /wheel_odom
                                            │                    odom → base_link
                                            ▼
                              joint_state_bridge ──► /joint_states
```

`/odrive_enable` 为 true 之前，驱动不会进入闭环。`/wheel_cmd` 超过 0.2 s 没有更新时，CAN 节点改发 0，避免 ODrive 锁存上一次速度。

急停（网页、`estop_rover` 或 `/estop`）会同时：

1. 把 `/cmd_vel` 清零
2. 让 ODrive 收到 Estop 并退出闭环
3. 对 PX4 发强制 disarm

PX4 的 `TrajectorySetpoint` 节点还在 `diffdrive_rover` 里，但总启动不会加载它，它也不再发布 `/wheel_cmd`。

## 包

| 包 | 作用 |
|---|---|
| `rover_description` | 几何参数 `config/rover_geometry.yaml`、URDF、关节桥 |
| `odrive_can_bridge` | CAN 协议、差速、轮速里程计、本机网页遥控 |
| `diffdrive_rover` | 可选的速度安全层；可选的 PX4 设定点（不驱动轮子） |
| `rover_bringup` | `full_system`、解锁、急停 |
| `d435i_launch` | RealSense 启动 |
| `rover_localization` / `rover_mapping` / `rover_navigation` / `rover_mission` | 后续定位和导航的草稿，这条控制链不用它们 |

几何只改这一份：`src/rover_description/config/rover_geometry.yaml`。带 `MEASURE` 的是占位值。车体 yaw 上限是 `max_angular_vel`（rad/s），不要把轮子角速度填进去。

## 构建

```bash
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions \
    python3-rosdep python3-can python3-flask python3-yaml \
    ros-jazzy-xacro ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher-gui ros-jazzy-rviz2
source /opt/ros/jazzy/setup.bash
cd ~/ros2_rover_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
# px4_ros_com 没有 release/1.14 这个分支名，1.14 行是 release/v1.14。
# 本仓库的控制链不依赖它。要对齐差速机架 50000，固件和消息请改用 PX4 1.16。
git clone https://github.com/PX4/px4_ros_com.git -b release/v1.14
cd ~/ros2_rover_ws
rosdep install --from-paths src --ignore-src -y --rosdistro=jazzy
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select odrive_can_bridge diffdrive_rover rover_description
colcon test-result --verbose
```

`python3-can` 没有对应的 rosdep 键，所以用 apt 安装，不要写进 `package.xml`。

## 启动

```bash
# 轮子先离地。
ros2 launch rover_bringup full_system.launch.py enable_d435i:=false enable_agent:=true

# 另一终端：允许闭环并解锁 PX4
ros2 run rover_bringup arm_rover

# 低速前进（会持续发，Ctrl-C 后 0.5 s 内停车）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" -r 10

# 急停
ros2 run rover_bringup estop_rover
```

可选参数：

- `enable_safety:=true`：遥控改发 `/cmd_vel_nav`，安全层输出 `/cmd_vel`。没有 `/scan` 时它会停车。
- `enable_web:=true`：网页只听 `127.0.0.1:5000`，向 `/cmd_vel` 发布，不占用 CAN。要对局域网开放必须自己改 `--host`，并限制网络。
- `enable_rviz:=true`

单独调底盘：`ros2 launch odrive_can_bridge odrive_can.launch.py`

## 上车前核对

轮子离地、车体垫稳，电流先用 yaml 里的小值。

1. 改 `rover_geometry.yaml`：用尺子量轮半径、左右轮距、前后轴距。
2. `wheel_directions` 先保持 `[1, 1, 1, 1]`。
3. `candump can0` 能看到心跳。`ros2 topic echo /odrive_diagnostic` 里四轴先是未闭环。
4. `ros2 run rover_bringup arm_rover` 之后四轴应变为 state `8`（闭环）。此时轮子不应转。
5. 每次只让一个轮子转。把 `wheel_directions` 里对应那一项临时改成只给该轮非零指令不方便，可以临时把 `/wheel_cmd` 的发布停掉，用下面的方式看反馈方向：给很小的 `linear.x`（例如 0.05），看哪几个轮子往“车头方向”转。反了的轴把 `wheel_directions` 改成 `-1`，改完重启 launch。
6. 四个轮子都朝车头转之后，再给 `angular.z: 0.2`。从上往下看，左转时左轮应往后、右轮应往前。
7. 确认松手或杀掉 `topic pub` 后 0.5 s 内轮子停下，`estop_rover` 后轴退出闭环且 PX4 上锁。
8. 轮子着地前把 `current_lim` 和 `max_wheel_speed` 改成电机铭牌允许的值。建议在 ODrive 上把 `axis.config.watchdog_timeout` 设成大约 0.5 s，这样 CAN 节点崩溃时驱动器自己退出闭环。

不要同时跑旧的 `cmdvel_to_setpoint` 来驱动轮子。Nav2、EKF、`ros2_control` 还没接到这条链上。
