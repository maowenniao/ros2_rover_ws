# 四轮差速车 ROS 2 工作空间

Jetson Orin Nano + Pixhawk 6C（USB，仅解锁 / 急停 / 状态）+ 两块 ODrive V3.6（USB-CAN，四轴）+ Intel D435i。

Ubuntu 24.04 + ROS 2 Jazzy。电机由 Jetson 做差速解算后直接走 CAN，不把 `/cmd_vel` 交给 PX4 的 rover 控制器。

## 控制链

```
/cmd_vel_nav ── safety_cmdvel_node ──► /cmd_vel
导航时 Nav2 经速度平滑写到 /cmd_vel_nav。只开底盘时可以直接发 /cmd_vel。
                                            │
                                            ▼
                              odrive_velocity_control   ← 唯一的 /wheel_cmd 发布者
                                            │
                                            ▼
                              odrive_can_node ──CAN──► ODrive
                                            │
                                            ▼
                              /wheel_state ──► /wheel_odom ──► EKF + D435i IMU ──► /odom
建图和导航时 EKF 是 odom → base_link 的唯一发布者。单独开底盘时轮速里程计自己发这份 TF。
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
| `d435i_launch` | D435i 驱动参数和光学坐标系静态 TF |
| `rover_localization` | EKF，发布 `/odom` 与 `odom → base_link` |
| `rover_mapping` | 深度转 `/scan`，slam_toolbox 发布 `map → odom` |
| `rover_navigation` | Nav2 + AMCL。导航时不要再开 SLAM |
| `rover_mission` | 航点动作客户端 |

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

不要同时跑旧的 `cmdvel_to_setpoint` 来驱动轮子。`ros2_control` 还没接上。

## 相机、定位和导航

D435i 话题在 `/d435i/d435i/` 下（命名空间和相机名都是 `d435i`）：

| 话题 | 用途 |
|---|---|
| `/d435i/d435i/color/image_raw` | 彩色 |
| `/d435i/d435i/depth/image_rect_raw` | 深度，转成 `/scan` |
| `/d435i/d435i/aligned_depth_to_color/image_raw` | 对齐到彩色的深度 |
| `/d435i/d435i/depth/color/points` | 局部代价地图的点云 |
| `/d435i/d435i/imu` | `sensor_msgs/Imu`，进 EKF |

光学坐标系由静态 TF 从 URDF 的 `d435i_link` 接到 `d435i_depth_optical_frame`、`d435i_color_optical_frame` 和 `d435i_imu_optical_frame`。驱动自己的 TF 关掉了，避免两份外参。这些是手册上的名义值，不是这台相机的标定。

`/scan` 的 `frame_id` 是 `d435i_depth_optical_frame`。深度图转扫描的射线就在这个光学帧里，不能改成 `base_link`。

```bash
# 建图：EKF 发 odom，slam_toolbox 发 map。没有 AMCL。
ros2 launch rover_bringup mapping.launch.py

# 边开边看。遥控发到 /cmd_vel_nav，安全层才会看 /scan。
ros2 topic pub /cmd_vel_nav geometry_msgs/msg/Twist "{linear: {x: 0.2}}" -r 10

# 保存地图。会生成 pgm 和 yaml。
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/rover_map"

# 把这两个文件放到 src/rover_navigation/maps/，或启动时指定 map:=。
ros2 launch rover_bringup navigation.launch.py map:=$HOME/maps/rover_map.yaml
```

导航链路是 `/cmd_vel_raw` → 速度平滑 → `/cmd_vel_nav` → 安全层 → `/cmd_vel`。速度上限从 `rover_geometry.yaml` 填进 Nav2，不会超过底盘限幅。建图和导航不要同时开：一个用 slam_toolbox，一个用 AMCL，都会发 `map → odom`。

仓库里的 `maps/rover_warehouse.pgm` 只是一张 1 米见方的空图，用来确认 map_server 能读文件。真正跑导航之前换成上面保存的地图。

### 上车核对

轮子可以着地，但第一次先留出空地，旁边不要站人。

1. `ros2 launch rover_bringup mapping.launch.py enable_rviz:=true`。RViz 里 Fixed Frame 用 `map`。应能看到彩色、深度，以及 `d435i_link` 到三个 optical 帧的 TF。`ros2 run tf2_tools view_frames` 里 `odom → base_link` 只有 EKF，没有第二份。
2. `ros2 topic hz /d435i/d435i/depth/image_rect_raw` 和 `ros2 topic hz /scan` 都有数据。`/scan` 的 frame 是 `d435i_depth_optical_frame`。
3. 慢速遥控走一圈。RViz 的 Map 应跟着铺开。人对着相机站在 1 m 内时，发 `/cmd_vel_nav` 前进应被安全层停住；倒车指令仍应能离开障碍。
4. `map_saver_cli` 存图后，停掉建图，用 `navigation.launch.py` 加载这张图。RViz 里粒子或激光应贴在墙上，而不是再起一份 SLAM。
5. 在空地上用 RViz 的 Nav2 Goal 点一个近目标。车应沿路径走，速度不超过几何文件里的线速度和角速度。
6. 把障碍放到路径上，车应在安全距离停下，而不是顶上去。确认后再试倒车离开。急停仍用 `ros2 run rover_bringup estop_rover`。
