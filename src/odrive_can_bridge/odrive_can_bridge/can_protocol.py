"""ODrive V3.6 CANSimple 帧编解码（固件 0.5.x）。

仲裁 ID 与命令号对齐
https://github.com/odriverobotics/ODrive/blob/fw-v0.5.6/Firmware/communication/can/can_simple.hpp
11 位标准帧：高 6 位是 node id，低 5 位是命令号。
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# 命令号来自 CANSimple 枚举，从 0 连续递增。
CMD_HEARTBEAT = 0x01
CMD_ESTOP = 0x02
CMD_GET_MOTOR_ERROR = 0x03
CMD_GET_ENCODER_ERROR = 0x04
CMD_SET_AXIS_NODE_ID = 0x06
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C
CMD_SET_INPUT_VEL = 0x0D
CMD_SET_INPUT_TORQUE = 0x0E
CMD_SET_LIMITS = 0x0F
CMD_GET_IQ = 0x14
CMD_REBOOT = 0x16
CMD_GET_BUS_VOLTAGE_CURRENT = 0x17
CMD_CLEAR_ERRORS = 0x18

AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8

CONTROL_MODE_VOLTAGE = 0
CONTROL_MODE_TORQUE = 1
CONTROL_MODE_VELOCITY = 2
CONTROL_MODE_POSITION = 3
INPUT_MODE_PASSTHROUGH = 1

_CMD_ID_MASK = 0x1F


def make_can_id(node_id: int, cmd_id: int) -> int:
    """组装 11 位 CAN 标识符：node_id 占高 6 位，cmd_id 占低 5 位。"""
    return ((int(node_id) & 0x3F) << 5) | (int(cmd_id) & _CMD_ID_MASK)


def parse_can_id(arbitration_id: int) -> Tuple[int, int]:
    """返回 (cmd_id, node_id)。"""
    cmd_id = int(arbitration_id) & _CMD_ID_MASK
    node_id = (int(arbitration_id) >> 5) & 0x3F
    return cmd_id, node_id


def rad_s_to_turns(rad_s: float) -> float:
    """车轮角速度 rad/s 转为 ODrive 的 turn/s。"""
    return float(rad_s) / (2.0 * math.pi)


def turns_to_rad(turns: float) -> float:
    """ODrive 的圈数或 turn/s 转为弧度或 rad/s。"""
    return float(turns) * 2.0 * math.pi


def encode_set_input_vel(node_id: int, turns_per_s: float, torque_ff: float = 0.0) -> Tuple[int, bytes]:
    """cmd 0x0D：float32 速度 (turn/s) + float32 力矩前馈。"""
    return make_can_id(node_id, CMD_SET_INPUT_VEL), struct.pack('<ff', float(turns_per_s), float(torque_ff))


def encode_set_input_torque(node_id: int, torque_nm: float) -> Tuple[int, bytes]:
    """cmd 0x0E：float32 力矩 (N·m)。"""
    return make_can_id(node_id, CMD_SET_INPUT_TORQUE), struct.pack('<f', float(torque_nm))


def encode_set_axis_state(node_id: int, state: int) -> Tuple[int, bytes]:
    """cmd 0x07：int32 目标轴状态。"""
    return make_can_id(node_id, CMD_SET_AXIS_STATE), struct.pack('<i', int(state))


def encode_set_controller_mode(node_id: int, control_mode: int, input_mode: int) -> Tuple[int, bytes]:
    """cmd 0x0B：int32 控制模式 + int32 输入模式。"""
    return make_can_id(node_id, CMD_SET_CONTROLLER_MODE), struct.pack('<ii', int(control_mode), int(input_mode))


def encode_set_limits(node_id: int, vel_limit_turns: float, current_lim: float) -> Tuple[int, bytes]:
    """cmd 0x0F：float32 速度上限 (turn/s) + float32 电流上限 (A)。"""
    return make_can_id(node_id, CMD_SET_LIMITS), struct.pack('<ff', float(vel_limit_turns), float(current_lim))


def encode_empty(node_id: int, cmd_id: int) -> Tuple[int, bytes]:
    """无数据的请求或命令（读编码器、清错、急停）。"""
    return make_can_id(node_id, cmd_id), b''


@dataclass(frozen=True)
class Heartbeat:
    axis_error: int
    axis_state: int
    motor_error: bool
    encoder_error: bool
    controller_error: bool
    trajectory_done: bool

    @property
    def closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATE_CLOSED_LOOP

    @property
    def faulted(self) -> bool:
        return self.axis_error != 0 or self.motor_error or self.encoder_error or self.controller_error


def decode_heartbeat(data: bytes) -> Heartbeat:
    """解析 8 字节心跳。

    字节 0-3：Axis_Error（uint32）
    字节 4：Axis_Current_State（uint8）
    字节 5：电机错误标志
    字节 6：编码器错误标志
    字节 7：bit0 控制器错误，bit7 轨迹完成
    """
    if len(data) < 8:
        raise ValueError(f'heartbeat 需要 8 字节，收到 {len(data)}')
    axis_error, axis_state, motor_flags, encoder_flags, controller_flags = struct.unpack_from('<IBBBB', data)
    return Heartbeat(
        axis_error=int(axis_error),
        axis_state=int(axis_state),
        motor_error=bool(motor_flags & 0x01),
        encoder_error=bool(encoder_flags & 0x01),
        controller_error=bool(controller_flags & 0x01),
        trajectory_done=bool(controller_flags & 0x80),
    )


def decode_encoder_estimates(data: bytes) -> Tuple[float, float]:
    """cmd 0x09 的回复：位置 (turn)、速度 (turn/s)。"""
    if len(data) < 8:
        raise ValueError(f'encoder estimates 需要 8 字节，收到 {len(data)}')
    pos_turns, vel_turns = struct.unpack_from('<ff', data)
    return float(pos_turns), float(vel_turns)


def decode_bus_voltage_current(data: bytes) -> Tuple[float, float]:
    """cmd 0x17 的回复：母线电压 (V)、母线电流 (A)。"""
    if len(data) < 8:
        raise ValueError(f'bus voltage 需要 8 字节，收到 {len(data)}')
    voltage, current = struct.unpack_from('<ff', data)
    return float(voltage), float(current)


def decode_iq(data: bytes) -> Tuple[float, float]:
    """cmd 0x14 的广播：Iq 设定值、Iq 测量值 (A)。"""
    if len(data) < 8:
        raise ValueError(f'Iq 需要 8 字节，收到 {len(data)}')
    setpoint, measured = struct.unpack_from('<ff', data)
    return float(setpoint), float(measured)


def select_velocity_command(
    latest: Optional[Sequence[float]],
    age_sec: Optional[float],
    timeout_sec: float,
    enabled: bool,
    estopped: bool,
    closed_loop: bool,
    wheel_count: int,
) -> Optional[List[float]]:
    """决定本周期要发给 ODrive 的轮速 (rad/s)。

    未使能或已急停时返回 None，调用方不得再发 Set_Input_Vel。
    尚未进入闭环、还没有指令或指令超时，则发全 0，避免 ODrive 锁存旧速度。
    """
    if estopped or not enabled:
        return None
    stale = latest is None or age_sec is None or age_sec > timeout_sec or len(latest) < wheel_count
    if not closed_loop or stale:
        return [0.0] * wheel_count
    return [float(latest[i]) for i in range(wheel_count)]
