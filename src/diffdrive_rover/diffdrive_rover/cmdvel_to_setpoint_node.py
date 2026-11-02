#!/usr/bin/env python3
"""
cmdvel_to_setpoint_node
=======================

功能：
    订阅 ROS 2 标准话题 ``/cmd_vel`` (geometry_msgs/Twist)，
    转换为 PX4 的 ``TrajectorySetpoint`` 并发布到 ``/fmu/in/trajectory_setpoint``，
    并根据 Offboard 模式要求周期性发布 ``OffboardControlMode``。

本节点不发布 /wheel_cmd。电机由 odrive_velocity_control 直驱。
/cmd_vel 超时后向 PX4 发零速度。默认系统启动不会加载本节点。

依赖：
    px4_msgs >= 1.14
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from rover_description.geometry import declare_geometry
from px4_msgs.msg import (
    TrajectorySetpoint,
    OffboardControlMode,
    VehicleCommand,
)


class CmdVelToSetpointNode(Node):
    """差速车 cmd_vel → PX4 TrajectorySetpoint 桥接节点。"""

    def __init__(self) -> None:
        super().__init__('cmdvel_to_setpoint_node')

        # ============== 参数 ==============
        declare_geometry(self)
        self.declare_parameter('cmdvel_timeout', 0.5)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('velocity_mode', True)
        self.declare_parameter('enable_safety_clamp', True)

        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_angular_vel = float(self.get_parameter('max_angular_vel').value)
        self.cmdvel_timeout = float(self.get_parameter('cmdvel_timeout').value)
        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(self.get_parameter('target_component').value)
        self.velocity_mode = bool(self.get_parameter('velocity_mode').value)
        self.enable_safety_clamp = bool(
            self.get_parameter('enable_safety_clamp').value
        )
        rate_hz = float(self.get_parameter('setpoint_rate_hz').value)

        # ============== QoS ==============
        # PX4 uXRCE-DDS 默认使用 BEST_EFFORT
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ============== 订阅 ==============
        self.cmdvel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmdvel_cb,
            qos_reliable,
        )

        # ============== 发布 ==============
        self.traj_sp_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_px4,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_px4,
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_px4,
        )

        # ============== 内部状态 ==============
        self._last_cmdvel_time = self.get_clock().now()
        self._current_vx = 0.0  # 机体坐标系下前进方向速度 (m/s)
        self._current_vy = 0.0
        self._current_wz = 0.0  # 角速度 (rad/s)
        self._yaw = 0.0         # 累积航向 (rad)，仅在位置模式用

        # ============== 定时器 ==============
        period = 1.0 / rate_hz
        self._timer = self.create_timer(period, self._publish_timer_cb)

        self.get_logger().info(
            f'cmdvel_to_setpoint_node 已启动：'
            f'rate={rate_hz}Hz, velocity_mode={self.velocity_mode}（不驱动电机）'
        )

    # ------------------------------------------------------------------
    # /cmd_vel 回调
    # ------------------------------------------------------------------
    def _cmdvel_cb(self, msg: Twist) -> None:
        """保存最新的速度指令。"""
        self._current_vx = float(msg.linear.x)
        self._current_vy = float(msg.linear.y)
        self._current_wz = float(msg.angular.z)
        self._last_cmdvel_time = self.get_clock().now()
        self.get_logger().debug(
            f'cmdvel: vx={self._current_vx:.2f}, '
            f'vy={self._current_vy:.2f}, wz={self._current_wz:.2f}',
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # 周期性发布
    # ------------------------------------------------------------------
    def _publish_timer_cb(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_cmdvel_time).nanoseconds / 1e9

        # 失联保护：超时则清零
        if dt > self.cmdvel_timeout:
            self._current_vx = 0.0
            self._current_vy = 0.0
            self._current_wz = 0.0

        # 安全限幅
        if self.enable_safety_clamp:
            self._current_vx = self._clamp(
                self._current_vx, -self.max_linear_vel, self.max_linear_vel
            )
            self._current_vy = self._clamp(
                self._current_vy, -self.max_linear_vel, self.max_linear_vel
            )
            self._current_wz = self._clamp(
                self._current_wz, -self.max_angular_vel, self.max_angular_vel
            )

        # 累积 yaw（位置模式需要）
        if not self.velocity_mode:
            self._yaw += self._current_wz * (1.0 / 20.0)
            # wrap 到 [-pi, pi]
            if self._yaw > math.pi:
                self._yaw -= 2.0 * math.pi
            elif self._yaw < -math.pi:
                self._yaw += 2.0 * math.pi

        # === 发布 OffboardControlMode ===
        # Offboard 模式要求持续收到该消息；至少一个字段必须为 true
        ocm = OffboardControlMode()
        ocm.timestamp = int(now.nanoseconds / 1000)  # us
        if self.velocity_mode:
            ocm.velocity = True
            ocm.position = False
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
        else:
            ocm.velocity = True  # 同时也要 true，否则 offboard 检查失败
            ocm.position = True
            ocm.acceleration = False
            ocm.attitude = False
            ocm.body_rate = False
        self.offboard_mode_pub.publish(ocm)

        # === 发布 TrajectorySetpoint ===
        sp = TrajectorySetpoint()
        sp.timestamp = int(now.nanoseconds / 1000)
        if self.velocity_mode:
            # PX4 的 velocity 字段是 NED 系，但 PX4 Rover 中车辆以 x 轴为前向
            # 设前进方向为 +x (机体坐标系)，PX4 内部会做 transform
            sp.velocity[0] = self._current_vx
            sp.velocity[1] = self._current_vy
            sp.velocity[2] = 0.0
            # 位置给 NaN 表示不参与位置控制
            sp.position[0] = float('nan')
            sp.position[1] = float('nan')
            sp.position[2] = float('nan')
        else:
            sp.position[0] = 0.0  # Rover 一般不控制绝对位置
            sp.position[1] = 0.0
            sp.position[2] = 0.0
            sp.velocity[0] = self._current_vx
            sp.velocity[1] = self._current_vy
            sp.velocity[2] = 0.0
        sp.acceleration[0] = 0.0
        sp.acceleration[1] = 0.0
        sp.acceleration[2] = 0.0
        sp.jerk[0] = 0.0
        sp.jerk[1] = 0.0
        sp.jerk[2] = 0.0
        sp.yaw = self._yaw if not self.velocity_mode else float('nan')
        sp.yawspeed = self._current_wz
        self.traj_sp_pub.publish(sp)

    # ------------------------------------------------------------------
    # 工具
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))

    # ------------------------------------------------------------------
    # 对外命令（可由其他节点调用）
    # ------------------------------------------------------------------
    def publish_vehicle_command(self, command: int, **params) -> None:
        """通用 PX4 VehicleCommand 发布工具。
        用法：publish_vehicle_command(176, param1=1.0, param2=6.0)
        """
        now = self.get_clock().now()
        msg = VehicleCommand()
        msg.timestamp = int(now.nanoseconds / 1000)
        msg.command = command
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = 255
        msg.source_component = 0
        msg.from_external = True
        self.vehicle_cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()