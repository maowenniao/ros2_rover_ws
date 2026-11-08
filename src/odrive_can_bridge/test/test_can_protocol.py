"""ODrive 0.5.6 CANSimple 帧测试。不需要硬件。"""
import struct

from odrive_can_bridge.can_protocol import (
    AXIS_STATE_CLOSED_LOOP,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_HEARTBEAT,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_INPUT_VEL,
    decode_encoder_estimates,
    decode_heartbeat,
    decode_iq,
    encode_set_input_torque,
    encode_set_input_vel,
    make_can_id,
    parse_can_id,
    rad_s_to_turns,
    select_velocity_command,
)


def test_arbitration_id_matches_firmware_layout():
    assert make_can_id(0, CMD_HEARTBEAT) == 0x001
    # 文档示例：node 1、命令 0x0C → 0x2C。
    assert make_can_id(1, 0x0C) == 0x2C
    assert parse_can_id(0x2C) == (0x0C, 1)
    assert parse_can_id(make_can_id(3, CMD_SET_INPUT_VEL)) == (CMD_SET_INPUT_VEL, 3)


def test_set_input_torque_is_command_0x0e_float():
    arb, data = encode_set_input_torque(1, -0.75)
    assert arb == (1 << 5) | CMD_SET_INPUT_TORQUE
    assert struct.unpack('<f', data)[0] == -0.75
    setpoint, measured = decode_iq(struct.pack('<ff', 0.2, 0.15))
    assert abs(setpoint - 0.2) < 1e-6
    assert abs(measured - 0.15) < 1e-6


def test_set_input_vel_is_command_0x0d_little_endian():
    arb, data = encode_set_input_vel(2, 1.5, 0.25)
    assert arb == (2 << 5) | 0x0D
    vel, torque = struct.unpack('<ff', data)
    assert vel == 1.5
    assert torque == 0.25


def test_heartbeat_state_is_not_the_first_uint32():
    payload = struct.pack('<IBBBB', 0, AXIS_STATE_CLOSED_LOOP, 0, 0, 0)
    hb = decode_heartbeat(payload)
    assert hb.axis_error == 0
    assert hb.axis_state == AXIS_STATE_CLOSED_LOOP
    assert hb.closed_loop
    assert not hb.faulted

    faulted = struct.pack('<IBBBB', 0x20, 1, 1, 0, 0x80)
    hb = decode_heartbeat(faulted)
    assert hb.axis_error == 0x20
    assert hb.axis_state == 1
    assert hb.motor_error
    assert hb.trajectory_done
    assert not hb.closed_loop


def test_encoder_estimates_are_turns():
    payload = struct.pack('<ff', 1.25, -0.5)
    pos, vel = decode_encoder_estimates(payload)
    assert pos == 1.25
    assert vel == -0.5
    assert abs(rad_s_to_turns(2 * 3.141592653589793) - 1.0) < 1e-6


def test_stale_or_disarmed_command_does_not_repeat_old_speed():
    moving = [1.0, 1.0, 1.0, 1.0]
    assert select_velocity_command(moving, 0.05, 0.2, True, False, True, 4) == moving
    assert select_velocity_command(moving, 0.25, 0.2, True, False, True, 4) == [0.0] * 4
    assert select_velocity_command(moving, 0.01, 0.2, False, False, True, 4) is None
    assert select_velocity_command(moving, 0.01, 0.2, True, True, True, 4) is None
    assert select_velocity_command(moving, 0.01, 0.2, True, False, False, 4) == [0.0] * 4
    assert select_velocity_command(None, None, 0.2, True, False, True, 4) == [0.0] * 4
