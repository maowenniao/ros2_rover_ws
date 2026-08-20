# rover_mission

任务与安全层：

- `rover_mission/waypoint_mission_node.py` 把 `config/waypoints.yaml` 中的航点通过 Nav2 `FollowWaypoints` action 依次执行；可循环。
- `rover_mission/safety_monitor_node.py` 监测 `/cmd_vel_nav` 与 `/scan`，发现异常（超速或前向距离过近）发布 `/estop`。

启动：

```bash
ros2 launch rover_mission mission.launch.py
```
