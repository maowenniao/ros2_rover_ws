#!/usr/bin/env python3
"""
odrive_can_node
===============

功能：
    ODrive V3.6 CAN 总线 <-> ROS 2 桥接节点。

    - 订阅 ``/wheel_cmd`` (Float32MultiArray, 4 个轮子的角速度 rad/s)
      -> 转换为 ODrive CAN 帧（Set_Input_Vel 0x00C）下发。
    - 监听 ODrive 心跳 (Heartbeat 0x001) 并发布 ``/wheel_state``
      (sensor_msgs/JointState 包含每轴 position/velocity/effort)。
    - 周期性触发 Set_Axis_State 让节点进入闭环控制。
    - 解析 ODrive 错误码并发布健康诊断信息。
    - 检测 ODrive 离线状态并发布诊断警告。

ODrive CAN 协议关键命令：

ODrive v3.6 的 CAN 帧 ID 格式：
    arb_id = (cmd_id << 6) | node_id

+-------+-------------------+----------+
| cmd_id| 名称              | 数据长度 |
+=======+===================+==========+
| 0x01  | Heartbeat         | 8 bytes  |
| 0x02  | Estop             | 0 bytes  |
| 0x06  | Set_Axis_State    | 4 bytes  |
| 0x07  | Get_Encoder_Est.  | 0 bytes  |
| 0x09  | Set_Controller_M. | 8 bytes  |
| 0x0C  | Set_Input_Vel     | 8 bytes  |
| 0x0D  | Set_Input_Pos     | 8 bytes  |
| 0x0E  | Set_Input_Torque  | 4 bytes  |
| 0x10  | Set_Limits        | 8 bytes  |
| 0x18  | Get_Iq            | 0 bytes  |
| 0x1C  | Reboot            | 0 bytes  |
| 0x1D  | Get_Bus_V_I       | 0 bytes  |
| 0x1E  | Clear_Errors      | 0 bytes  |
+-------+-------------------+----------+

参见: https://docs.odriverobotics.com/v0.5.6/can-protocol.html
"""
import struct
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray, String, Bool
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


# ======================================================================
# ODrive CAN 命令 ID 定义
# ======================================================================
CMD_HEARTBEAT = 0x01
CMD_ESTOP = 0x02
CMD_SET_AXIS_STATE = 0x06
CMD_GET_ENCODER_ESTIMATES = 0x07
CMD_SET_CONTROLLER_MODE = 0x09
CMD_SET_INPUT_VEL = 0x0C
CMD_SET_INPUT_POS = 0x0D
CMD_SET_INPUT_TORQUE = 0x0E
CMD_SET_LIMITS = 0x10
CMD_GET_IQ = 0x18
CMD_REBOOT = 0x1C
CMD_GET_BUS_VOLTAGE_CURRENT = 0x1D
CMD_CLEAR_ERRORS = 0x1E

# Axis State 值
AXIS_STATE_IDLE = 0x01
AXIS_STATE_STARTUP_SEQUENCE = 0x02
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 0x03
AXIS_STATE_MOTOR_CALIBRATION = 0x04
AXIS_STATE_ENCODER_INDEX_SEARCH = 0x06
AXIS_STATE_ENCODER_CALIBRATION = 0x07
AXIS_STATE_CLOSED_LOOP_CONTROL = 0x08
AXIS_STATE_LOCK_IN_SPINOUT_DETECT = 0x0B

# 轴状态名称映射
AXIS_STATE_NAMES = {
    AXIS_STATE_IDLE: "IDLE",
    AXIS_STATE_STARTUP_SEQUENCE: "STARTUP_SEQUENCE",
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE: "FULL_CALIBRATION",
    AXIS_STATE_MOTOR_CALIBRATION: "MOTOR_CALIBRATION",
    AXIS_STATE_ENCODER_INDEX_SEARCH: "ENCODER_INDEX_SEARCH",
    AXIS_STATE_ENCODER_CALIBRATION: "ENCODER_CALIBRATION",
    AXIS_STATE_CLOSED_LOOP_CONTROL: "CLOSED_LOOP_CONTROL",
    AXIS_STATE_LOCK_IN_SPINOUT_DETECT: "LOCK_IN_SPINOUT",
}

# Control Mode
CONTROL_MODE_VOLTAGE = 0x00
CONTROL_MODE_TORQUE = 0x01
CONTROL_MODE_VELOCITY = 0x02
CONTROL_MODE_POSITION = 0x03

# Input Mode
INPUT_MODE_PASSTHROUGH = 0x01
INPUT_MODE_VEL_RAMP = 0x02
INPUT_MODE_TORQUE_RAMP = 0x03
INPUT_MODE_MIRROR = 0x04
INPUT_MODE_TORQUE = 0x05
INPUT_MODE_POS_FILTER = 0x06

# CAN ID 计算
NODE_ID_MASK = 0x3F
CMD_ID_SHIFT = 6


def make_can_id(node_id: int, cmd_id: int) -> int:
    """根据 ODrive 协议组装 CAN 帧 ID."""
    return ((cmd_id & 0x3F) << CMD_ID_SHIFT) | (node_id & NODE_ID_MASK)


def parse_can_id(arb_id: int):
    """解析 arb_id 返回 (cmd_id, node_id)."""
    node_id = arb_id & NODE_ID_MASK
    cmd_id = (arb_id >> CMD_ID_SHIFT) & 0x3F
    return cmd_id, node_id


# ======================================================================
# ODrive 错误码定义
# ======================================================================
# Axis error flags
AXIS_ERROR_PHASE_RESISTANCE_ADC_FAIL = 0x00000001
AXIS_ERROR_BOOTTIME_ADC_NON_ZERO = 0x00000002
AXIS_ERROR_SYSTEM_LEVEL = 0x00000004
AXIS_ERROR_MOTOR_WRONG_HW = 0x00000008
AXIS_ERROR_WRONG_ID = 0x00000010
AXIS_ERROR_INVALID_BEMF_CONV_METHOD = 0x00000020
AXIS_ERROR_CTRL_TICK_NOT_RUNNING = 0x00000040
AXIS_ERROR_STOPPED_WITHOUT_ESTIMATE = 0x00000080
AXIS_ERROR_PERIODIC_CALLBACK_NOT_RUNNING = 0x00000100
AXIS_ERROR_MISSING_ESTIMATE_HERE = 0x00000200
AXIS_ERROR_OVRD_CURRENT_SETPOINT_NOT_UPDATING = 0x00000400
AXIS_ERROR_MODULATION_D_ZERO = 0x00000800
AXIS_ERROR_MOTOR_CALIBRATION_FAILED = 0x00001000
AXIS_ERROR_GPIO_GPIO10_IS_PIN5 = 0x00002000
AXIS_ERROR_GPIO_AHRSWN_DATA = 0x00004000
AXIS_ERROR_SAMPLING_ADC_BAD_CALIBRATION = 0x00010000
AXIS_ERROR_TIMER_OVERRUN_WARNING = 0x00020000
AXIS_ERROR_CONTROLLER_TICK_NOT_RUNNING = 0x00040000
AXIS_ERROR_ADC_CONVERSION_NOT_RUNNING = 0x00080000
AXIS_ERROR_CONTROL_SETPOINT_NOT_UPDATING = 0x00100000
AXIS_ERROR_MODULATION_D_ZERO_MORE = 0x00200000
AXIS_ERROR_VECTOR_MAGNITUDE_OVERFLOW = 0x00400000
AXIS_ERROR_PERIODIC_INPUT_NOT_UPDATING = 0x00800000

# Motor error flags
MOTOR_ERROR_BOOTTIME_SENSORS_NOT_SETTLED = 0x00000001
MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x00000002
MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x00000004
MOTOR_ERROR_DRIVERS_OVER_TEMP = 0x00000008
MOTOR_ERROR_FAILED_TO_SETUP_HW = 0x00000010
MOTOR_ERROR_POWER_GEAR_CHANGED = 0x00000020
MOTOR_ERROR_CURRENT_MEASUREMENTS_UNRELIABLE = 0x00000040
MOTOR_ERROR_INTERLOCK_FAILED = 0x00000080
MOTOR_ERROR_DC_BUS_SOLID_STATE_OUTPUT_SHORT = 0x00000100
MOTOR_ERROR_DC_SOLID_STATE_OUTPUT_STUCK_CLOSED = 0x00000200
MOTOR_ERROR_SOLID_STATE_OUTPUT_WONT_TURN_ON = 0x00000400
MOTOR_ERROR_SOLID_STATE_OUTPUT_WONT_TURN_OFF = 0x00000800
MOTOR_ERROR_POWER_STAGE_MISMATCH = 0x00001000
MOTOR_ERROR_SYS_VOLTAGE_OUT_OF_RANGE = 0x00002000
MOTOR_ERROR_INVALID_TORQUE_CONSTANT_KV = 0x00004000
MOTOR_ERROR_CURRENT_SENSOR_1_FAIL = 0x00008000
MOTOR_ERROR_CURRENT_SENSOR_2_FAIL = 0x00010000
MOTOR_ERROR_CURRENT_SAMPLES_MISMATCH = 0x00020000
MOTOR_ERROR_MCU_OVER_TEMP = 0x00040000
MOTOR_ERROR_DC_BUS_CURRENT_OUT_OF_RANGE = 0x00080000
MOTOR_ERROR_CURRENT_LIMIT_MISMATCH = 0x00100000

# Encoder error flags
ENCODER_ERROR_BOOTTIME_NOT_SETTLED = 0x00000001
ENCODER_ERROR_INDEX_NOT_FOUND = 0x00000002
ENCODER_ERROR_NO_HALL_NOT_RUNNING = 0x00000004
ENCODER_ERROR_CALIBRATION_RATIO_OOR = 0x00000008
ENCODER_ERROR_NO_RASPBERRY_PI = 0x00000010
ENCODER_ERROR_POLLING_TOO_EARLY = 0x00000020
ENCODER_ERROR_COMM_OVERFLOW = 0x00000040
ENCODER_ERROR_MISSING_ESTIMATES = 0x00000080
ENCODER_ERROR_MISSING_GLOBAL_SET_POINTS = 0x00000100
ENCODER_ERROR_MISSING_LOCAL_SET_POINTS = 0x00000200
ENCODER_ERROR_BAD_TIMING_ESTIMATE = 0x00000400
ENCODER_ERROR_MISSING_HALL_SIGNALS = 0x00000800
ENCODER_ERROR_HALL_SLOPE_TOO_LOW = 0x00001000
ENCODER_ERROR_HALL_NOT_CLEAN_COMMS = 0x00002000
ENCODER_ERROR_NOT_READY = 0x00004000
ENCODER_ERROR_BAD_M_AXIS = 0x00008000

# Controller error flags
CONTROLLER_ERROR_OVERSPEED = 0x00000001
CONTROLLER_ERROR_TRAJ_BAD_INPUT_POS = 0x00000002
CONTROLLER_ERROR_TRAJ_BAD_INPUT_VEL = 0x00000004
CONTROLLER_ERROR_TRAJ_BAD_X_TOL = 0x00000008
CONTROLLER_ERROR_TRAJ_BAD_V_TOL = 0x00000010
CONTROLLER_ERROR_TRAJ_BAD_VEL_SCALE = 0x00000020
CONTROLLER_ERROR_TRAJ_ABORT = 0x00000040
CONTROLLER_ERROR_TRAJ_POS_ARC_OVERFLOW = 0x00000080
CONTROLLER_ERROR_TRAJ_GAIN_SCHED_BAD_MODE = 0x00000100
CONTROLLER_ERROR_ICLAMP_NOT_UPDATING = 0x00000200
CONTROLLER_ERROR_BAD_INPUT_TRANSFORM = 0x00000400
CONTROLLER_ERROR_BAD_IFF_TRANSFORM = 0x00000800
CONTROLLER_ERROR_WATCHDOG_REJECTED_CMD = 0x00001000

# 错误码名称映射
AXIS_ERROR_NAMES = {
    AXIS_ERROR_PHASE_RESISTANCE_ADC_FAIL: "Phase Resistance ADC Fail",
    AXIS_ERROR_BOOTTIME_ADC_NON_ZERO: "Boottime ADC Non-Zero",
    AXIS_ERROR_SYSTEM_LEVEL: "System Level",
    AXIS_ERROR_MOTOR_WRONG_HW: "Motor Wrong HW",
    AXIS_ERROR_WRONG_ID: "Wrong Axis ID",
    AXIS_ERROR_INVALID_BEMF_CONV_METHOD: "Invalid BEMF Conv Method",
    AXIS_ERROR_CTRL_TICK_NOT_RUNNING: "Ctrl Tick Not Running",
    AXIS_ERROR_STOPPED_WITHOUT_ESTIMATE: "Stopped Without Estimate",
    AXIS_ERROR_PERIODIC_CALLBACK_NOT_RUNNING: "Periodic Callback Not Running",
    AXIS_ERROR_MISSING_ESTIMATE_HERE: "Missing Estimate",
    AXIS_ERROR_OVRD_CURRENT_SETPOINT_NOT_UPDATING: "Override Current Setpoint",
    AXIS_ERROR_MODULATION_D_ZERO: "Modulation D Zero",
    AXIS_ERROR_MOTOR_CALIBRATION_FAILED: "Motor Calibration Failed",
    AXIS_ERROR_GPIO_GPIO10_IS_PIN5: "GPIO10 is PIN5",
    AXIS_ERROR_GPIO_AHRSWN_DATA: "GPIO AHRS Warnings",
    AXIS_ERROR_SAMPLING_ADC_BAD_CALIBRATION: "Sampling ADC Bad Calib",
    AXIS_ERROR_TIMER_OVERRUN_WARNING: "Timer Overrun Warning",
    AXIS_ERROR_CONTROLLER_TICK_NOT_RUNNING: "Controller Tick Not Running",
    AXIS_ERROR_ADC_CONVERSION_NOT_RUNNING: "ADC Conversion Not Running",
    AXIS_ERROR_CONTROL_SETPOINT_NOT_UPDATING: "Control Setpoint Not Upd",
    AXIS_ERROR_MODULATION_D_ZERO_MORE: "Modulation D Zero More",
    AXIS_ERROR_VECTOR_MAGNITUDE_OVERFLOW: "Vector Magnitude Overflow",
    AXIS_ERROR_PERIODIC_INPUT_NOT_UPDATING: "Periodic Input Not Upd",
}

MOTOR_ERROR_NAMES = {
    MOTOR_ERROR_BOOTTIME_SENSORS_NOT_SETTLED: "Boottime Sensors",
    MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE: "Phase Resistance OOR",
    MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE: "Phase Inductance OOR",
    MOTOR_ERROR_DRIVERS_OVER_TEMP: "Drivers Over Temp",
    MOTOR_ERROR_FAILED_TO_SETUP_HW: "Failed Setup HW",
    MOTOR_ERROR_POWER_GEAR_CHANGED: "Power Gear Changed",
    MOTOR_ERROR_CURRENT_MEASUREMENTS_UNRELIABLE: "Current Unreliable",
    MOTOR_ERROR_INTERLOCK_FAILED: "Interlock Failed",
    MOTOR_ERROR_DC_BUS_SOLID_STATE_OUTPUT_SHORT: "DC Bus SS Output Short",
    MOTOR_ERROR_DC_SOLID_STATE_OUTPUT_STUCK_CLOSED: "DC SS Stuck Closed",
    MOTOR_ERROR_SOLID_STATE_OUTPUT_WONT_TURN_ON: "SS Output Won't Turn On",
    MOTOR_ERROR_SOLID_STATE_OUTPUT_WONT_TURN_OFF: "SS Output Won't Off",
    MOTOR_ERROR_POWER_STAGE_MISMATCH: "Power Stage Mismatch",
    MOTOR_ERROR_SYS_VOLTAGE_OUT_OF_RANGE: "SYS Voltage OOR",
    MOTOR_ERROR_INVALID_TORQUE_CONSTANT_KV: "Invalid Torque Const KV",
    MOTOR_ERROR_CURRENT_SENSOR_1_FAIL: "Current Sensor 1 Fail",
    MOTOR_ERROR_CURRENT_SENSOR_2_FAIL: "Current Sensor 2 Fail",
    MOTOR_ERROR_CURRENT_SAMPLES_MISMATCH: "Current Samples Mismatch",
    MOTOR_ERROR_MCU_OVER_TEMP: "MCU Over Temp",
    MOTOR_ERROR_DC_BUS_CURRENT_OUT_OF_RANGE: "DC Bus Current OOR",
    MOTOR_ERROR_CURRENT_LIMIT_MISMATCH: "Current Limit Mismatch",
}

ENCODER_ERROR_NAMES = {
    ENCODER_ERROR_BOOTTIME_NOT_SETTLED: "Boottime Not Settled",
    ENCODER_ERROR_INDEX_NOT_FOUND: "Index Not Found",
    ENCODER_ERROR_NO_HALL_NOT_RUNNING: "No Hall Not Running",
    ENCODER_ERROR_CALIBRATION_RATIO_OOR: "Calibration Ratio OOR",
    ENCODER_ERROR_NO_RASPBERRY_PI: "No Raspberry Pi",
    ENCODER_ERROR_POLLING_TOO_EARLY: "Polling Too Early",
    ENCODER_ERROR_COMM_OVERFLOW: "Comm Overflow",
    ENCODER_ERROR_MISSING_ESTIMATES: "Missing Estimates",
    ENCODER_ERROR_MISSING_GLOBAL_SET_POINTS: "Missing Global Set Pts",
    ENCODER_ERROR_MISSING_LOCAL_SET_POINTS: "Missing Local Set Pts",
    ENCODER_ERROR_BAD_TIMING_ESTIMATE: "Bad Timing Estimate",
    ENCODER_ERROR_MISSING_HALL_SIGNALS: "Missing Hall Signals",
    ENCODER_ERROR_HALL_SLOPE_TOO_LOW: "Hall Slope Too Low",
    ENCODER_ERROR_HALL_NOT_CLEAN_COMMS: "Hall Not Clean Comms",
    ENCODER_ERROR_NOT_READY: "Not Ready",
    ENCODER_ERROR_BAD_M_AXIS: "Bad M Axis",
}

CONTROLLER_ERROR_NAMES = {
    CONTROLLER_ERROR_OVERSPEED: "Overspeed",
    CONTROLLER_ERROR_TRAJ_BAD_INPUT_POS: "Traj Bad Input Pos",
    CONTROLLER_ERROR_TRAJ_BAD_INPUT_VEL: "Traj Bad Input Vel",
    CONTROLLER_ERROR_TRAJ_BAD_X_TOL: "Traj Bad X Tol",
    CONTROLLER_ERROR_TRAJ_BAD_V_TOL: "Traj Bad V Tol",
    CONTROLLER_ERROR_TRAJ_BAD_VEL_SCALE: "Traj Bad Vel Scale",
    CONTROLLER_ERROR_TRAJ_ABORT: "Traj Abort",
    CONTROLLER_ERROR_TRAJ_POS_ARC_OVERFLOW: "Traj Pos Arc Overflow",
    CONTROLLER_ERROR_TRAJ_GAIN_SCHED_BAD_MODE: "Traj Gain Sched Bad",
    CONTROLLER_ERROR_ICLAMP_NOT_UPDATING: "Iclamp Not Updating",
    CONTROLLER_ERROR_BAD_INPUT_TRANSFORM: "Bad Input Transform",
    CONTROLLER_ERROR_BAD_IFF_TRANSFORM: "Bad IFF Transform",
    CONTROLLER_ERROR_WATCHDOG_REJECTED_CMD: "Watchdog Rejected Cmd",
}


def decode_errors(error_flag: int, error_names: dict) -> List[str]:
    """解析错误标志，返回错误信息列表."""
    errors = []
    for flag, name in error_names.items():
        if error_flag & flag:
            errors.append(name)
    return errors if errors else ["None (0x{:08X})".format(error_flag)]


# ======================================================================
# 数据结构
# ======================================================================
@dataclass
class ODriveState:
    """单个 ODrive 轴的状态."""
    node_id: int = 0
    last_heartbeat_time: float = 0.0
    axis_state: int = 0
    axis_error: int = 0
    motor_error: int = 0
    encoder_error: int = 0
    controller_error: int = 0
    pos_estimate: float = 0.0  # turns
    vel_estimate: float = 0.0  # turn/s
    iq_setpoint: float = 0.0   # A
    iq_measured: float = 0.0   # A
    bus_voltage: float = 0.0   # V
    bus_current: float = 0.0   # A
    is_online: bool = False

    # 统计信息
    heartbeat_count: int = 0
    error_count: int = 0
    last_error_time: float = 0.0

    @property
    def is_in_error(self) -> bool:
        """检查是否处于错误状态."""
        return (self.axis_error != 0 or self.motor_error != 0 or
                self.encoder_error != 0 or self.controller_error != 0)

    @property
    def is_closed_loop(self) -> bool:
        """检查是否处于闭环控制状态."""
        return self.axis_state == AXIS_STATE_CLOSED_LOOP_CONTROL

    @property
    def uptime_seconds(self) -> float:
        """获取从上次心跳以来的时间（秒）."""
        return time.time() - self.last_heartbeat_time if self.last_heartbeat_time > 0 else 0.0


# ======================================================================
# ROS 节点
# ======================================================================
class ODriveCanNode(Node):
    """ODrive CAN 总线 <-> ROS 2 桥接节点."""

    def __init__(self) -> None:
        super().__init__('odrive_can_node')

        # ============== 参数 ==============
        self.declare_parameter('can_interface', 'socketcan')
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('can_bitrate', 250000)
        self.declare_parameter('node_ids', [0, 1, 2, 3])
        self.declare_parameter('wheel_cmd_order', [0, 1, 2, 3])
        self.declare_parameter('control_mode', 1)  # 1=VELOCITY
        self.declare_parameter('auto_enter_closed_loop', True)
        self.declare_parameter('auto_set_interval_sec', 1.0)
        self.declare_parameter('cmd_send_rate_hz', 50.0)
        self.declare_parameter('current_lim', 20.0)
        self.declare_parameter('vel_limit', 10.0)
        self.declare_parameter('heartbeat_timeout_sec', 1.0)
        self.declare_parameter('enable_diagnostic', True)
        self.declare_parameter('diagnostic_rate_hz', 1.0)
        self.declare_parameter('require_closed_loop_for_cmd', True)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_channel = str(self.get_parameter('can_channel').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.node_ids = list(self.get_parameter('node_ids').value)
        self.wheel_cmd_order = list(self.get_parameter('wheel_cmd_order').value)
        self.control_mode = int(self.get_parameter('control_mode').value)
        self.auto_enter_closed_loop = bool(
            self.get_parameter('auto_enter_closed_loop').value)
        self.auto_set_interval_sec = float(
            self.get_parameter('auto_set_interval_sec').value)
        self.cmd_send_rate_hz = float(
            self.get_parameter('cmd_send_rate_hz').value)
        self.current_lim = float(self.get_parameter('current_lim').value)
        self.vel_limit = float(self.get_parameter('vel_limit').value)
        self.heartbeat_timeout_sec = float(
            self.get_parameter('heartbeat_timeout_sec').value)
        self.enable_diagnostic = bool(
            self.get_parameter('enable_diagnostic').value)
        self.diagnostic_rate_hz = float(
            self.get_parameter('diagnostic_rate_hz').value)
        self.require_closed_loop_for_cmd = bool(
            self.get_parameter('require_closed_loop_for_cmd').value)

        # ============== 状态 ==============
        self._states: Dict[int, ODriveState] = {
            nid: ODriveState(node_id=nid) for nid in self.node_ids
        }
        self._latest_wheel_cmd: Optional[List[float]] = None

        # ============== CAN 总线连接 ==============
        try:
            import can  # python-can
        except ImportError:
            self.get_logger().fatal(
                '缺少 python-can 库，请安装: pip install python-can')
            raise

        try:
            self._bus = can.interface.Bus(
                interface=self.can_interface,
                channel=self.can_channel,
                bitrate=self.can_bitrate,
            )
        except Exception as e:
            self.get_logger().fatal(
                f'无法连接 CAN 总线 {self.can_interface}/{self.can_channel}: {e}')
            raise

        self.get_logger().info(
            f'CAN 总线已连接: {self.can_interface}/{self.can_channel} '
            f'@{self.can_bitrate} bps, node_ids={self.node_ids}')

        # ============== ROS 2 接口 ==============
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.wheel_cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_cmd',
            self._wheel_cmd_cb,
            qos_reliable,
        )
        self.wheel_state_pub = self.create_publisher(
            JointState, '/wheel_state', qos_reliable)

        # 诊断发布
        self.diagnostic_pub = self.create_publisher(
            DiagnosticStatus, '/odrive_diagnostic', qos_best_effort)

        # 在线状态发布
        self.online_pub = self.create_publisher(
            Bool, '/odrive_online', qos_best_effort)

        # 错误信息发布
        self.error_pub = self.create_publisher(
            String, '/odrive_error_info', qos_best_effort)

        # ============== 定时器 ==============
        # 速度指令发送
        cmd_period = 1.0 / self.cmd_send_rate_hz
        self._cmd_timer = self.create_timer(cmd_period, self._send_wheel_cmd)

        # 自动 Set_Axis_State (首次/重连时让节点进闭环)
        self._setup_timer = None
        if self.auto_enter_closed_loop:
            self._setup_timer = self.create_timer(
                self.auto_set_interval_sec, self._auto_setup_axes)

        # 心跳接收 + 状态发布
        self._state_timer = self.create_timer(0.05, self._poll_can_and_publish)

        # 诊断定时器
        if self.enable_diagnostic:
            self._diag_timer = self.create_timer(
                1.0 / self.diagnostic_rate_hz, self._publish_diagnostic)

        self.get_logger().info('odrive_can_node 已启动')

    # ------------------------------------------------------------------
    # /wheel_cmd 回调
    # ------------------------------------------------------------------
    def _wheel_cmd_cb(self, msg: Float32MultiArray) -> None:
        """/wheel_cmd.data 顺序 [左前, 左后, 右前, 右后]，单位 rad/s."""
        if len(msg.data) < len(self.wheel_cmd_order):
            self.get_logger().warn(
                f'/wheel_cmd 长度 {len(msg.data)} < 期望 {len(self.wheel_cmd_order)}')
            return

        # 检查是否所有需要的 ODrive 都在闭环状态
        if self.require_closed_loop_for_cmd:
            for nid in self.node_ids:
                if not self._states[nid].is_closed_loop:
                    state = self._states[nid]
                    self.get_logger().warn(
                        f'拒绝速度指令: ODrive nid={nid} 未进入闭环 '
                        f'(当前状态: {AXIS_STATE_NAMES.get(state.axis_state, state.axis_state)})')
                    return

        self._latest_wheel_cmd = list(msg.data)

    # ------------------------------------------------------------------
    # 周期性下发速度指令
    # ------------------------------------------------------------------
    def _send_wheel_cmd(self) -> None:
        if self._latest_wheel_cmd is None:
            return
        cmd = self._latest_wheel_cmd
        # wheel_cmd.data[i] 对应 wheel_cmd_order[i] 指定的 node_id
        for idx, node_id in enumerate(self.wheel_cmd_order):
            if idx >= len(cmd):
                continue
            # ODrive 中 vel 单位是 turn/s；输入 rad/s 需要 / (2*pi)
            rad_per_s = float(cmd[idx])
            turns_per_s = rad_per_s / (2.0 * 3.141592653589793)
            self._send_set_input_vel(node_id, turns_per_s)

    def _send_set_input_vel(self, node_id: int, vel_turns: float,
                            torque_ff: float = 0.0) -> None:
        """ODrive cmd 0x0C: Set_Input_Vel(float32 vel, float32 torque_ff)."""
        try:
            data = struct.pack('<ff', vel_turns, torque_ff)
            self._send_can(node_id, CMD_SET_INPUT_VEL, data)
        except Exception as e:
            self.get_logger().warn(f'Set_Input_Vel 失败 nid={node_id}: {e}')

    def _send_set_axis_state(self, node_id: int, state: int) -> None:
        """ODrive cmd 0x06: Set_Axis_State(uint32 state)."""
        try:
            data = struct.pack('<I', state)
            self._send_can(node_id, CMD_SET_AXIS_STATE, data)
        except Exception as e:
            self.get_logger().warn(f'Set_Axis_State 失败 nid={node_id}: {e}')

    def _send_set_controller_mode(self, node_id: int,
                                  control_mode: int,
                                  input_mode: int) -> None:
        """ODrive cmd 0x09: Set_Controller_Mode(uint32 control_mode, uint32 input_mode)."""
        try:
            data = struct.pack('<II', control_mode, input_mode)
            self._send_can(node_id, CMD_SET_CONTROLLER_MODE, data)
        except Exception as e:
            self.get_logger().warn(f'Set_Controller_Mode 失败 nid={node_id}: {e}')

    def _send_set_limits(self, node_id: int,
                         vel_limit: float, current_lim: float) -> None:
        """ODrive cmd 0x10: Set_Limits(float32 vel_limit, float32 current_lim)."""
        try:
            data = struct.pack('<ff', vel_limit, current_lim)
            self._send_can(node_id, CMD_SET_LIMITS, data)
        except Exception as e:
            self.get_logger().warn(f'Set_Limits 失败 nid={node_id}: {e}')

    def _send_can(self, node_id: int, cmd: int, data: bytes) -> None:
        import can
        msg = can.Message(
            arbitration_id=make_can_id(node_id, cmd),
            data=data,
            is_extended_id=False,
        )
        self._bus.send(msg)

    # ------------------------------------------------------------------
    # 自动设置（首次/重连后让 ODrive 进入闭环）
    # ------------------------------------------------------------------
    def _auto_setup_axes(self) -> None:
        """周期性检查并让 ODrive 进入闭环控制状态."""
        any_not_closed_loop = False
        for nid in self.node_ids:
            state = self._states[nid]
            # 检查是否离线
            if not state.is_online:
                self.get_logger().debug(f'ODrive nid={nid} 离线，跳过设置')
                continue

            # 如果不在闭环状态，尝试进入
            if not state.is_closed_loop:
                any_not_closed_loop = True
                # 1) 清错
                self._send_can(nid, CMD_CLEAR_ERRORS, b'')
                # 2) 设置控制模式
                if self.control_mode == 1:
                    self._send_set_controller_mode(
                        nid, CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH)
                elif self.control_mode == 2:
                    self._send_set_controller_mode(
                        nid, CONTROL_MODE_POSITION, INPUT_MODE_PASSTHROUGH)
                # 3) 设置限制
                self._send_set_limits(nid, self.vel_limit, self.current_lim)
                # 4) 进入闭环
                self._send_set_axis_state(nid, AXIS_STATE_CLOSED_LOOP_CONTROL)
                self.get_logger().info(
                    f'已下发 Set_Axis_State=CLOSED_LOOP 到 ODrive nid={nid}')

        if any_not_closed_loop:
            self.get_logger().debug('部分 ODrive 未进入闭环，持续尝试...')
        elif self._setup_timer is not None:
            # 所有轴都进入闭环后，停止周期性设置
            self.get_logger().info('所有 ODrive 已进入闭环控制，停止周期性设置')
            self._setup_timer.cancel()
            self._setup_timer = None

    # ------------------------------------------------------------------
    # 读取 CAN 消息 + 更新 + 发布状态
    # ------------------------------------------------------------------
    def _poll_can_and_publish(self) -> None:
        """非阻塞读取 CAN 消息，解析 Heartbeat / Get_Encoder_Estimates 等."""
        try:
            while True:
                msg = self._bus.recv(timeout=0.0)
                if msg is None:
                    break
                self._parse_can_msg(msg)
        except Exception as e:
            self.get_logger().warn(f'CAN 读取异常: {e}')
            return

        # 检查离线状态
        self._check_heartbeat_timeout()

        # 发布 wheel_state
        self._publish_wheel_state()

    def _parse_can_msg(self, msg) -> None:
        """解析 ODrive CAN 消息."""
        cmd, node_id = parse_can_id(msg.arbitration_id)
        if node_id not in self._states:
            return
        state = self._states[node_id]
        now = time.time()

        if cmd == CMD_HEARTBEAT:
            # 8 bytes: u32 axis_state, u32 axis_error_flags
            # error_flags 格式: u16 axis_error | u16 motor_error
            if len(msg.data) >= 4:
                state.axis_state = struct.unpack('<I', msg.data[0:4])[0]
            if len(msg.data) >= 8:
                err_flags = struct.unpack('<I', msg.data[4:8])[0]
                # 低16位是 motor_error，高16位是 axis_error
                state.motor_error = err_flags & 0xFFFF
                state.axis_error = (err_flags >> 16) & 0xFFFF
            state.last_heartbeat_time = now
            state.is_online = True
            state.heartbeat_count += 1

            # 检测新错误
            if state.is_in_error:
                if not hasattr(state, '_was_in_error') or not state._was_in_error:
                    state.error_count += 1
                    state.last_error_time = now
                    self._log_error_details(node_id, state)
            state._was_in_error = state.is_in_error

        elif cmd == CMD_GET_ENCODER_ESTIMATES:
            # 8 bytes: float32 pos_estimate, float32 vel_estimate
            if len(msg.data) >= 8:
                state.pos_estimate, state.vel_estimate = struct.unpack(
                    '<ff', msg.data[0:8])

        elif cmd == CMD_GET_BUS_VOLTAGE_CURRENT:
            # 8 bytes: float32 bus_voltage, float32 bus_current
            if len(msg.data) >= 8:
                state.bus_voltage, state.bus_current = struct.unpack(
                    '<ff', msg.data[0:8])

        elif cmd == CMD_GET_IQ:
            # 8 bytes: float32 iq_setpoint, float32 iq_measured
            if len(msg.data) >= 8:
                state.iq_setpoint, state.iq_measured = struct.unpack(
                    '<ff', msg.data[0:8])

    def _check_heartbeat_timeout(self) -> None:
        """检查心跳超时，标记离线状态."""
        any_offline = False
        for nid in self.node_ids:
            state = self._states[nid]
            if state.last_heartbeat_time > 0:
                elapsed = time.time() - state.last_heartbeat_time
                if elapsed > self.heartbeat_timeout_sec:
                    if state.is_online:
                        state.is_online = False
                        any_offline = True
                        self.get_logger().warn(
                            f'ODrive nid={nid} 心跳超时 ({elapsed:.2f}s)，标记为离线')
                else:
                    if not state.is_online:
                        state.is_online = True
                        any_offline = False
                        self.get_logger().info(f'ODrive nid={nid} 已重新上线')

        # 发布在线状态
        online_msg = Bool()
        online_msg.data = not any_offline
        self.online_pub.publish(online_msg)

    def _log_error_details(self, node_id: int, state: ODriveState) -> None:
        """记录详细的错误信息."""
        axis_errors = decode_errors(state.axis_error, AXIS_ERROR_NAMES)
        motor_errors = decode_errors(state.motor_error, MOTOR_ERROR_NAMES)
        encoder_errors = decode_errors(state.encoder_error, ENCODER_ERROR_NAMES)
        controller_errors = decode_errors(
            state.controller_error, CONTROLLER_ERROR_NAMES)

        error_msg = f'ODrive nid={node_id} 错误:\n'
        error_msg += f'  轴错误: {", ".join(axis_errors)}\n'
        error_msg += f'  电机错误: {", ".join(motor_errors)}\n'
        error_msg += f'  编码器错误: {", ".join(encoder_errors)}\n'
        error_msg += f'  控制器错误: {", ".join(controller_errors)}'

        self.get_logger().error(error_msg)

        # 发布错误信息
        error_pub_msg = String()
        error_pub_msg.data = error_msg
        self.error_pub.publish(error_pub_msg)

    def _publish_wheel_state(self) -> None:
        now = self.get_clock().now()
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [f'wheel_{nid}' for nid in self.node_ids]
        js.position = [
            self._states[nid].pos_estimate * 2.0 * 3.141592653589793
            for nid in self.node_ids
        ]  # turns -> rad
        js.velocity = [
            self._states[nid].vel_estimate * 2.0 * 3.141592653589793
            for nid in self.node_ids
        ]
        js.effort = [
            self._states[nid].iq_measured for nid in self.node_ids
        ]
        self.wheel_state_pub.publish(js)

    # ------------------------------------------------------------------
    # 诊断信息发布
    # ------------------------------------------------------------------
    def _publish_diagnostic(self) -> None:
        """发布 ODrive 诊断信息."""
        if not self.enable_diagnostic:
            return

        now = self.get_clock().now()

        for nid in self.node_ids:
            state = self._states[nid]
            diag = DiagnosticStatus()
            diag.name = f'ODrive Node {nid}'
            diag.message = self._get_node_message(state)
            diag.level = self._get_diagnostic_level(state)
            diag.stamp = now.to_msg()

            # 添加键值对信息
            diag.values.append(KeyValue(
                key='state',
                value=AXIS_STATE_NAMES.get(
                    state.axis_state, str(state.axis_state))))
            diag.values.append(KeyValue(
                key='online',
                value='Yes' if state.is_online else 'No'))
            diag.values.append(KeyValue(
                key='heartbeat_count',
                value=str(state.heartbeat_count)))

            if state.is_online:
                diag.values.append(KeyValue(
                    key='uptime',
                    value=f'{state.uptime_seconds:.1f}s'))
                diag.values.append(KeyValue(
                    key='velocity',
                    value=f'{state.vel_estimate * 2 * 3.14159:.2f} rad/s'))
                diag.values.append(KeyValue(
                    key='position',
                    value=f'{state.pos_estimate * 2 * 3.14159:.2f} rad'))

                if state.bus_voltage > 0:
                    diag.values.append(KeyValue(
                        key='bus_voltage',
                        value=f'{state.bus_voltage:.1f} V'))
                if abs(state.bus_current) > 0:
                    diag.values.append(KeyValue(
                        key='bus_current',
                        value=f'{state.bus_current:.1f} A'))

            if state.is_in_error:
                diag.values.append(KeyValue(
                    key='errors',
                    value=f'AX:0x{state.axis_error:X} '
                    f'MOT:0x{state.motor_error:X} '
                    f'ENC:0x{state.encoder_error:X} '
                    f'CTR:0x{state.controller_error:X}'))
                diag.values.append(KeyValue(
                    key='error_count',
                    value=str(state.error_count)))

            self.diagnostic_pub.publish(diag)

    def _get_node_message(self, state: ODriveState) -> str:
        """获取节点状态消息."""
        if not state.is_online:
            return 'OFFLINE'
        if state.is_in_error:
            return f'ERROR - {AXIS_STATE_NAMES.get(state.axis_state, "UNKNOWN")}'
        if state.is_closed_loop:
            return f'CLOSED LOOP - {AXIS_STATE_NAMES.get(state.axis_state, "UNKNOWN")}'
        return AXIS_STATE_NAMES.get(state.axis_state, 'UNKNOWN')

    def _get_diagnostic_level(self, state: ODriveState) -> int:
        """获取级别 (0=OK, 1=Warning, 2=Error)."""
        if not state.is_online:
            return DiagnosticStatus.ERROR
        if state.is_in_error:
            return DiagnosticStatus.ERROR
        if not state.is_closed_loop:
            return DiagnosticStatus.WARN
        return DiagnosticStatus.OK


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ODriveCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 安全停机：发送 Estop + IDLE
        try:
            for nid in node.node_ids:
                node._send_can(nid, CMD_ESTOP, b'')
                node._send_set_axis_state(nid, AXIS_STATE_IDLE)
        except Exception:
            pass
        try:
            if node._bus:
                node._bus.shutdown()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
