# rover_navigation

Nav2 在 Jetson 上的启动与配置。包含：

- `config/nav2_params.yaml` 控制器、规划器、行为树、代价地图参数
- `launch/navigation.launch.py` Nav2 全栈启动
- `maps/rover_warehouse.yaml` 地图占位（用 map_saver_cli 替换）
- `maps/.gitkeep` 保证目录进入版本

## 启动

```bash
ros2 launch rover_navigation navigation.launch.py
```

Nav2 默认输出 `/cmd_vel_nav`；差速底盘的安全层（`safety_cmdvel_node`）会订阅此话题并限速输出 `/cmd_vel`，最终交由 `cmdvel_to_setpoint_node` 送入 Pixhawk。
