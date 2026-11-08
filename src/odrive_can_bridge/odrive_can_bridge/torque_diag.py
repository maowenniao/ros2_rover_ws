"""单轴力矩/电流梯度诊断，以及四轴工具复用的安全原语。

本模块硬性拒绝多轴。四轴同时力矩只允许走 ``odrive_quad_torque_diag``。
Phase0 不授权实跑：没有完整旗标、USB 只读门控和文件锁时，不开 CAN。
不发送清错，不保存配置，不写速度/电流上限。
"""
from __future__ import annotations

import argparse
import fcntl
import json
import os
import struct
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Iterable, Optional, Sequence

from odrive_can_bridge.can_protocol import (
    AXIS_STATE_CLOSED_LOOP,
    AXIS_STATE_IDLE,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_GET_MOTOR_ERROR,
    CMD_HEARTBEAT,
    CMD_REBOOT,
    CMD_SET_AXIS_NODE_ID,
    CMD_SET_AXIS_STATE,
    CMD_SET_CONTROLLER_MODE,
    CMD_SET_INPUT_POS,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_INPUT_VEL,
    CMD_SET_LIMITS,
    CMD_CLEAR_ERRORS,
    CMD_ESTOP,
    CONTROL_MODE_TORQUE,
    CONTROL_MODE_VELOCITY,
    INPUT_MODE_PASSTHROUGH,
    decode_bus_voltage_current,
    decode_encoder_estimates,
    decode_heartbeat,
    decode_iq,
    encode_set_axis_state,
    encode_set_controller_mode,
    encode_set_input_torque,
    make_can_id,
    parse_can_id,
    rad_s_to_turns,
    turns_to_rad,
)

TOOL_SINGLE = "odrive_torque_diag"
TOOL_QUAD = "odrive_quad_torque_diag"

NODES = (0, 1, 2, 3)
LEFT_SERIAL = "3580387c3335"
RIGHT_SERIAL = "358038733335"
LEFT_NODES = (0, 1)
RIGHT_NODES = (2, 3)

CURRENT_MIN_A = 0.1
CURRENT_MAX_A = 5.0
CURRENT_STEP_A = 0.1
CURRENT_SLEW_A_PER_S = 0.5
HOLD_SEC = 1.0
TICK_SEC = 0.020
STALE_SEC = 0.5
GAP_ABORT_SEC = 0.10
SOFT_SPEED_STOP_RAD_S = 3.5
SOFT_SPEED_STOP_TICKS = 3
SINGLE_BUDGET_FLOOR_SEC = 160.0
QUAD_BUDGET_FLOOR_SEC = 180.0
VBUS_MARGIN_V = 2.0
USB_BASELINE_MAX_AGE_SEC = 300.0
NOMINAL_KT = 0.744
NOMINAL_VEL_GAIN = 0.670
NOMINAL_VEL_LIMIT_TURNS = 3.183

MULTI_AXIS_REJECTION = "必须用 --node 指定单个节点 0..3。不做四轮同时力矩。"
FOUR_WHEEL_BAN = "不要四轮同时给力矩。"

# 诊断进程允许发出的命令。清错、写上限、寻址空查询都不在此列。
ALLOWED_TX_CMDS = frozenset({
    CMD_SET_INPUT_TORQUE,
    CMD_SET_AXIS_STATE,
    CMD_SET_CONTROLLER_MODE,
})
FORBIDDEN_CMDS = frozenset({
    CMD_CLEAR_ERRORS,
    CMD_SET_LIMITS,
    CMD_ESTOP,
    CMD_GET_IQ,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_MOTOR_ERROR,
    CMD_GET_ENCODER_ERROR,
    CMD_GET_BUS_VOLTAGE_CURRENT,
    CMD_REBOOT,
    CMD_SET_AXIS_NODE_ID,
    CMD_SET_INPUT_POS,
    CMD_SET_INPUT_VEL,
})

_DEVICE_BROADCASTS = frozenset({
    CMD_HEARTBEAT,
    CMD_GET_ENCODER_ESTIMATES,
    CMD_GET_IQ,
    CMD_GET_BUS_VOLTAGE_CURRENT,
})

_OTHER_HOST_PATTERNS = (
    "odrive_can_node",
    "odrive_velocity_control",
    "web_controller",
    "odrive_ramp_diag",
    "ramp_diag",
    "odrive_torque_diag",
    "odrive_quad_torque_diag",
)

AUTHORIZATION_NOTE = "实跑需用户当次授权。未出现明确授权时不得对电机执行 --execute。"


class UsbReadError(RuntimeError):
    """USB 只读失败。调用方不得因此改去写 CAN。"""


class DiagLockError(RuntimeError):
    """诊断文件锁已被占用。"""


class CanOpenError(RuntimeError):
    """CAN 传输层未能打开。"""


@dataclass(frozen=True)
class Segment:
    kind: str
    t0: float
    t1: float
    i0: float
    i1: float


@dataclass(frozen=True)
class UsbAxisBaseline:
    serial: str
    node_id: int
    control_mode: int
    input_mode: int
    axis_state: int
    axis_error: int
    motor_error: bool
    encoder_error: bool
    controller_error: bool
    torque_constant: float
    vel_gain: float
    vel_limit: float
    current_lim: float
    iq_rate_ms: float
    heartbeat_rate_ms: float
    enable_watchdog: bool
    enable_torque_mode_vel_limit: bool
    armed: bool
    read_at_unix: float


@dataclass
class BoardSnapshot:
    serial: str
    read_at_unix: float
    axes: dict[int, UsbAxisBaseline]

    def axis(self, node: int) -> UsbAxisBaseline:
        if node not in self.axes:
            raise UsbReadError(f"序列号 {self.serial} 上没有 node {node}")
        return self.axes[node]


@dataclass(frozen=True)
class BoardSpec:
    name: str
    serial: str
    nodes: tuple[int, ...]


BOARDS = (
    BoardSpec("左板", LEFT_SERIAL, LEFT_NODES),
    BoardSpec("右板", RIGHT_SERIAL, RIGHT_NODES),
)


@dataclass
class Frame:
    arb_id: int
    data: bytes
    is_rtr: bool = False


@dataclass
class DiagResult:
    exit_code: int = 0
    measurement: str = "dry-run"
    execute_opened_can: bool = False
    usb_write_attempted: bool = False
    ok: bool = False
    fail_closed: bool = False
    stop_reason: Optional[str] = None
    reasons: list[str] = field(default_factory=list)
    samples: list[dict] = field(default_factory=list)
    tx: list[tuple[int, int, bytes]] = field(default_factory=list)
    summary: dict = field(default_factory=dict)
    plan_text: str = ""


@dataclass
class SessionRequest:
    tool: str
    mode: str
    nodes: tuple[int, ...]
    profile: str = "sync"
    current_min_a: float = CURRENT_MIN_A
    current_max_a: float = CURRENT_MAX_A
    step_a: float = CURRENT_STEP_A
    slew_a_per_s: float = CURRENT_SLEW_A_PER_S
    hold_sec: float = HOLD_SEC
    soft_speed_stop: bool = True
    budget_sec: Optional[float] = None
    baselines: Optional[dict[int, UsbAxisBaseline]] = None
    vbus_margin_v: float = VBUS_MARGIN_V


def guard_tx(cmd_id: int) -> None:
    """诊断发送路径的最后一道闸。禁止清错、写上限和寻址查询。"""
    if cmd_id in FORBIDDEN_CMDS or cmd_id not in ALLOWED_TX_CMDS:
        raise RuntimeError(f"禁止发送的 CAN 命令 0x{cmd_id:02x}")


def is_host_request(cmd_id: int, data: bytes, is_rtr: bool = False) -> bool:
    """设备自己的 8 字节周期广播不是主机请求。空帧和设定命令是。"""
    if is_rtr or len(data) == 0:
        return True
    if cmd_id in _DEVICE_BROADCASTS and len(data) >= 8:
        return False
    return True


def _is_foreign_host(cmd_id: int, data: bytes, is_rtr: bool = False, echoed: bool = False) -> bool:
    if echoed:
        return False
    return is_host_request(cmd_id, data, is_rtr)


def hardware_execute_blockers(*, node: object = None, nodes: object = None) -> list[str]:
    """单轴工具的多轴硬拒绝。合法单节点返回空列表。"""
    if nodes not in (None, "", (), []):
        return [MULTI_AXIS_REJECTION]
    if isinstance(node, str):
        text = node.strip()
        if not text or "," in text or " " in text:
            return [MULTI_AXIS_REJECTION]
        try:
            node = int(text)
        except ValueError:
            return [MULTI_AXIS_REJECTION]
    if node not in NODES:
        return [MULTI_AXIS_REJECTION]
    return []


def limit_torque_like_fw056(
    torque_nm: float,
    vel_turn_s: float,
    vel_limit_turn_s: float,
    vel_gain: float,
    enable_torque_mode_vel_limit: bool = True,
) -> float:
    """固件 0.5.6 转矩模式速度窗：Tmax=(vel_limit-vel)*vel_gain。"""
    if not enable_torque_mode_vel_limit:
        return float(torque_nm)
    upper = (float(vel_limit_turn_s) - float(vel_turn_s)) * float(vel_gain)
    lower = (-float(vel_limit_turn_s) - float(vel_turn_s)) * float(vel_gain)
    return min(upper, max(lower, float(torque_nm)))


class Plant:
    """Phase0 仿真对象。阻尼后的平衡速度低于默认软停，用来走完指令剖面。

    它不是空载轮子的动力学孪生；超速软停由测试注入速度单独验收。
    """

    def __init__(
        self,
        torque_constant: float,
        vel_gain: float,
        vel_limit_turn_s: float,
        damping: float = 1.5,
    ) -> None:
        self.torque_constant = float(torque_constant)
        self.vel_gain = float(vel_gain)
        self.vel_limit_turn_s = float(vel_limit_turn_s)
        self.damping = float(damping)
        self.vel_rad_s = 0.0
        self.iq_measured = 0.0
        self.torque_limited = 0.0

    def step(self, torque_cmd_nm: float, dt: float) -> None:
        vel_turns = rad_s_to_turns(self.vel_rad_s)
        limited = limit_torque_like_fw056(
            torque_cmd_nm,
            vel_turns,
            self.vel_limit_turn_s,
            self.vel_gain,
            True,
        )
        self.torque_limited = limited
        if self.torque_constant > 0:
            self.iq_measured = limited / self.torque_constant
        else:
            self.iq_measured = 0.0
        target = 0.0 if self.damping <= 0 else limited / self.damping
        if dt > 0:
            tau = 0.05
            alpha = dt / (tau + dt)
            self.vel_rad_s += (target - self.vel_rad_s) * alpha


def _interp(segment: Segment, t: float) -> float:
    if segment.t1 <= segment.t0:
        return segment.i1
    alpha = (t - segment.t0) / (segment.t1 - segment.t0)
    return segment.i0 + (segment.i1 - segment.i0) * alpha


def build_profile_segments(
    current_min: float,
    current_max: float,
    step: float,
    slew: float,
    hold: float,
) -> list[Segment]:
    """0 → 各级保持 → 对称降回 0。斜率恒定，不跳变。"""
    if step <= 0 or slew <= 0:
        raise ValueError("步长和斜率必须为正")
    if current_max + 1e-9 < current_min or current_min < 0:
        raise ValueError("电流范围无效")
    n = int(round((current_max - current_min) / step))
    levels = [current_min + i * step for i in range(max(n, 0) + 1)]
    if levels:
        levels[-1] = float(current_max)
    cleaned = []
    for value in levels:
        if not cleaned or abs(value - cleaned[-1]) > 1e-9:
            cleaned.append(float(value))
    targets = list(cleaned)
    for value in reversed(cleaned[:-1]):
        targets.append(value)
    targets.append(0.0)

    segments: list[Segment] = []
    t = 0.0
    current = 0.0
    for target in targets:
        if abs(target - current) > 1e-12:
            dur = abs(target - current) / slew
            segments.append(Segment("ramp", t, t + dur, current, target))
            t += dur
            current = target
        if hold > 0 and target > 1e-12:
            segments.append(Segment("hold", t, t + hold, target, target))
            t += hold
    if not segments:
        segments.append(Segment("hold", 0.0, 0.0, 0.0, 0.0))
    return segments


def profile_duration(segments: Sequence[Segment]) -> float:
    if not segments:
        return 0.0
    return float(segments[-1].t1)


def current_at(t: float, segments: Sequence[Segment]) -> float:
    if not segments:
        return 0.0
    if t <= 0:
        return _interp(segments[0], segments[0].t0) if segments[0].t0 <= 0 else 0.0
    for segment in segments:
        if segment.t0 <= t < segment.t1:
            return _interp(segment, t)
    last = segments[-1]
    if t <= last.t1 + 1e-9:
        return last.i1
    return 0.0


def phase_offset(node: int, nodes: Sequence[int], mode: str, duration: float) -> float:
    if mode == "sync":
        return 0.0
    if mode != "staggered":
        raise ValueError(f"未知剖面 {mode}")
    ordered = list(nodes)
    index = ordered.index(node)
    return index * (duration / 4.0)


def node_current(
    t: float,
    node: int,
    nodes: Sequence[int],
    mode: str,
    segments: Sequence[Segment],
) -> float:
    local = t - phase_offset(node, nodes, mode, profile_duration(segments))
    if local < 0:
        return 0.0
    return current_at(local, segments)


def schedule_duration(nodes: Sequence[int], mode: str, segments: Sequence[Segment]) -> float:
    duration = profile_duration(segments)
    if mode != "staggered" or len(nodes) <= 1:
        return duration
    return duration + max(phase_offset(node, nodes, mode, duration) for node in nodes)


def slew_limit(previous_a: float, target_a: float, dt: float, slew_a_per_s: float) -> float:
    """正常节拍内限制电流变化。紧急置零不走这里。"""
    if dt <= 0:
        return previous_a
    step = abs(slew_a_per_s) * dt
    if target_a > previous_a + step:
        return previous_a + step
    if target_a < previous_a - step:
        return previous_a - step
    return target_a


def make_axis(serial: str, node: int, **overrides: object) -> UsbAxisBaseline:
    raw_now = overrides.pop("read_at_unix", time.time())
    now = float(raw_now)
    data: dict[str, object] = dict(
        serial=str(serial).lower(),
        node_id=node,
        control_mode=CONTROL_MODE_VELOCITY,
        input_mode=INPUT_MODE_PASSTHROUGH,
        axis_state=AXIS_STATE_IDLE,
        axis_error=0,
        motor_error=False,
        encoder_error=False,
        controller_error=False,
        torque_constant=NOMINAL_KT,
        vel_gain=NOMINAL_VEL_GAIN,
        vel_limit=NOMINAL_VEL_LIMIT_TURNS,
        current_lim=20.0,
        iq_rate_ms=20.0,
        heartbeat_rate_ms=10.0,
        enable_watchdog=False,
        enable_torque_mode_vel_limit=True,
        armed=False,
        read_at_unix=now,
    )
    data.update(overrides)
    data["serial"] = str(data["serial"]).lower()
    data["read_at_unix"] = float(data["read_at_unix"])
    return UsbAxisBaseline(**data)  # type: ignore[arg-type]


def make_board(serial: str, nodes: Sequence[int], **axis_overrides: object) -> BoardSnapshot:
    read_at = float(axis_overrides.get("read_at_unix", time.time()))
    axes = {int(node): make_axis(serial, int(node), **axis_overrides) for node in nodes}
    return BoardSnapshot(serial=str(serial).lower(), read_at_unix=read_at, axes=axes)


def evaluate_usb_baseline(
    baseline: UsbAxisBaseline,
    expected_serial: str,
    expected_node: int,
) -> list[str]:
    """只读门控。任一条失败，调用方不得打开 CAN，也不得清错。"""
    reasons = []
    serial = str(baseline.serial).lower()
    expected = str(expected_serial).lower()
    if serial != expected:
        reasons.append(f"node {expected_node} 序列号 {serial} != {expected}")
    if int(baseline.node_id) != int(expected_node):
        reasons.append(f"node_id {baseline.node_id} != {expected_node}")
    if int(baseline.control_mode) != CONTROL_MODE_VELOCITY:
        reasons.append(f"node {expected_node} control_mode 不是 VELOCITY")
    if int(baseline.input_mode) != INPUT_MODE_PASSTHROUGH:
        reasons.append(f"node {expected_node} input_mode 不是 PASSTHROUGH")
    if int(baseline.axis_state) != AXIS_STATE_IDLE:
        reasons.append(f"node {expected_node} 不是 IDLE")
    if baseline.armed:
        reasons.append(f"node {expected_node} 已武装")
    if (
        int(baseline.axis_error) != 0
        or baseline.motor_error
        or baseline.encoder_error
        or baseline.controller_error
    ):
        reasons.append(f"node {expected_node} 存在锁存错误，不清错，中止")
    if not (
        float(baseline.torque_constant) > 0
        and float(baseline.vel_gain) > 0
        and float(baseline.vel_limit) > 0
    ):
        reasons.append(f"node {expected_node} Kt/vel_gain/vel_limit 必须为正")
    if float(baseline.current_lim) < 5.0:
        reasons.append(f"node {expected_node} current_lim < 5")
    if not (float(baseline.iq_rate_ms) > 0 and float(baseline.heartbeat_rate_ms) > 0):
        reasons.append(f"node {expected_node} iq/heartbeat 广播周期无效")
    if baseline.enable_watchdog:
        reasons.append(f"node {expected_node} enable_watchdog 必须为 False")
    if not baseline.enable_torque_mode_vel_limit:
        reasons.append(f"node {expected_node} 转矩速度保护开关必须保持开启")
    return reasons


def baseline_age_reasons(snapshot: BoardSnapshot, now: float) -> list[str]:
    age = float(now) - float(snapshot.read_at_unix)
    if age > USB_BASELINE_MAX_AGE_SEC:
        return [f"USB baseline 已过期 {age:.0f}s，超过 5 分钟，不开 CAN"]
    if age < -5:
        return ["USB baseline 时间戳在未来，不开 CAN"]
    return []


def boards_for_nodes(nodes: Iterable[int]) -> tuple[BoardSpec, ...]:
    selected = set(int(node) for node in nodes)
    return tuple(board for board in BOARDS if any(node in selected for node in board.nodes))


def board_from_json(payload: dict) -> BoardSnapshot:
    serial = str(payload["serial"]).lower()
    read_at = float(payload["read_at_unix"])
    axes: dict[int, UsbAxisBaseline] = {}
    for item in payload["axes"]:
        raw = dict(item)
        raw.setdefault("serial", serial)
        raw.setdefault("read_at_unix", read_at)
        axis = UsbAxisBaseline(
            serial=str(raw["serial"]).lower(),
            node_id=int(raw["node_id"]),
            control_mode=int(raw["control_mode"]),
            input_mode=int(raw["input_mode"]),
            axis_state=int(raw["axis_state"]),
            axis_error=int(raw["axis_error"]),
            motor_error=bool(raw["motor_error"]),
            encoder_error=bool(raw["encoder_error"]),
            controller_error=bool(raw["controller_error"]),
            torque_constant=float(raw["torque_constant"]),
            vel_gain=float(raw["vel_gain"]),
            vel_limit=float(raw["vel_limit"]),
            current_lim=float(raw["current_lim"]),
            iq_rate_ms=float(raw["iq_rate_ms"]),
            heartbeat_rate_ms=float(raw["heartbeat_rate_ms"]),
            enable_watchdog=bool(raw["enable_watchdog"]),
            enable_torque_mode_vel_limit=bool(raw["enable_torque_mode_vel_limit"]),
            armed=bool(raw["armed"]),
            read_at_unix=float(raw["read_at_unix"]),
        )
        axes[axis.node_id] = axis
    return BoardSnapshot(serial=serial, read_at_unix=read_at, axes=axes)


def board_to_json(snapshot: BoardSnapshot) -> dict:
    return {
        "serial": snapshot.serial,
        "read_at_unix": snapshot.read_at_unix,
        "axes": [asdict(snapshot.axes[node]) for node in sorted(snapshot.axes)],
    }


class LiveUsbReader:
    """现场 USB 只读入口。没有保存、清错或写配置的方法。"""

    writes = 0

    def read_board(self, serial: str) -> BoardSnapshot:
        try:
            import odrive  # type: ignore
        except ImportError as exc:
            raise UsbReadError("未安装 odrive Python 包，无法只读 USB") from exc
        finder = getattr(odrive, "find_any", None)
        if finder is None:
            raise UsbReadError("odrive.find_any 不可用")
        try:
            try:
                device = finder(serial_number=serial, timeout=3)
            except TypeError:
                device = finder(timeout=3)
        except Exception as exc:
            raise UsbReadError(f"USB 只读失败: {exc}") from exc
        if device is None:
            raise UsbReadError(f"没有找到序列号 {serial}")
        return _snapshot_from_device(device, serial)


def _attr(obj: object, path: str, default: object = None) -> object:
    current = obj
    for part in path.split("."):
        if not hasattr(current, part):
            return default
        current = getattr(current, part)
    return current


def _format_serial(value: object, fallback: str) -> str:
    if isinstance(value, int):
        return format(value, "x").zfill(12)
    if value is None:
        return fallback.lower()
    return str(value).lower().removeprefix("0x")


def _snapshot_from_device(device: object, requested_serial: str) -> BoardSnapshot:
    serial = _format_serial(_attr(device, "serial_number"), requested_serial)
    axes: dict[int, UsbAxisBaseline] = {}
    now = time.time()
    for axis_name in ("axis0", "axis1"):
        axis = _attr(device, axis_name)
        if axis is None:
            continue
        node_id = _attr(axis, "config.can.node_id")
        if node_id is None:
            continue
        iq_rate = _attr(axis, "config.can.iq_rate_ms")
        heartbeat = _attr(axis, "config.can.heartbeat_rate_ms", 0)
        record = UsbAxisBaseline(
            serial=serial,
            node_id=int(node_id),
            control_mode=int(_attr(axis, "controller.config.control_mode", -1)),
            input_mode=int(_attr(axis, "controller.config.input_mode", -1)),
            axis_state=int(_attr(axis, "current_state", -1)),
            axis_error=int(_attr(axis, "error", 1) or 0),
            motor_error=bool(int(_attr(axis, "motor.error", 0) or 0)),
            encoder_error=bool(int(_attr(axis, "encoder.error", 0) or 0)),
            controller_error=bool(int(_attr(axis, "controller.error", 0) or 0)),
            torque_constant=float(_attr(axis, "motor.config.torque_constant", 0.0) or 0.0),
            vel_gain=float(_attr(axis, "controller.config.vel_gain", 0.0) or 0.0),
            vel_limit=float(_attr(axis, "controller.config.vel_limit", 0.0) or 0.0),
            current_lim=float(_attr(axis, "motor.config.current_lim", 0.0) or 0.0),
            iq_rate_ms=float(iq_rate) if iq_rate is not None else 0.0,
            heartbeat_rate_ms=float(heartbeat or 0.0),
            enable_watchdog=bool(_attr(axis, "config.enable_watchdog", True)),
            enable_torque_mode_vel_limit=bool(
                _attr(axis, "controller.config.enable_torque_mode_vel_limit", False)
            ),
            armed=int(_attr(axis, "current_state", AXIS_STATE_IDLE) or 0) != AXIS_STATE_IDLE,
            read_at_unix=now,
        )
        axes[record.node_id] = record
    return BoardSnapshot(serial=serial, read_at_unix=now, axes=axes)


def collect_baselines(
    nodes: Sequence[int],
    reader: object,
    now: float,
) -> tuple[dict[int, UsbAxisBaseline], list[str]]:
    reasons: list[str] = []
    prechecked = getattr(reader, "reasons", None)
    if prechecked:
        reasons.extend(list(prechecked))
    baselines: dict[int, UsbAxisBaseline] = {}
    if reasons:
        return baselines, reasons
    for board in boards_for_nodes(nodes):
        try:
            snapshot = reader.read_board(board.serial)  # type: ignore[attr-defined]
        except UsbReadError as exc:
            reasons.append(f"{board.name} USB 只读失败: {exc}")
            continue
        reasons.extend(baseline_age_reasons(snapshot, now))
        if str(snapshot.serial).lower() != board.serial.lower():
            reasons.append(
                f"{board.name} 序列号不匹配: 期望 {board.serial} 实际 {snapshot.serial}，不开 CAN"
            )
            continue
        for node in board.nodes:
            if node not in nodes:
                continue
            try:
                axis = snapshot.axis(node)
            except UsbReadError as exc:
                reasons.append(str(exc))
                continue
            reasons.extend(evaluate_usb_baseline(axis, board.serial, node))
            baselines[node] = axis
    return baselines, reasons


def default_lock_path() -> str:
    preferred_dir = "/run/lock"
    if os.path.isdir(preferred_dir) and os.access(preferred_dir, os.W_OK):
        return os.path.join(preferred_dir, "odrive_can_diag.lock")
    cache = os.path.join(os.path.expanduser("~"), ".cache")
    os.makedirs(cache, exist_ok=True)
    return os.path.join(cache, "odrive_can_diag.lock")


class DiagFileLock:
    """fcntl.flock 独占。抢不到则调用方 exit 2，且不得写 CAN。"""

    def __init__(self, path: str) -> None:
        self.path = path
        self._fh = None

    def acquire(self) -> None:
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._fh = open(self.path, "a+")
        try:
            fcntl.flock(self._fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._fh.close()
            self._fh = None
            raise DiagLockError("已有诊断进程持有锁，拒绝启动，不写 CAN。") from exc

    def release(self) -> None:
        if self._fh is None:
            return
        try:
            fcntl.flock(self._fh.fileno(), fcntl.LOCK_UN)
        finally:
            self._fh.close()
            self._fh = None


class VirtualClock:
    def __init__(self, start: float = 0.0) -> None:
        self.t = float(start)

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        self.t += float(dt)


class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, dt: float) -> None:
        if dt > 0:
            time.sleep(dt)


def find_other_can_hosts(proc_lines: Optional[Sequence[str]] = None) -> Optional[list[str]]:
    """返回其它可能占用 CAN 的进程行。无法列举时返回 None，由执行门控拒绝。"""
    if proc_lines is None:
        try:
            completed = subprocess.run(
                ["ps", "-eo", "pid,args"],
                capture_output=True,
                text=True,
                check=False,
            )
        except OSError:
            return None
        if completed.returncode != 0:
            return None
        proc_lines = completed.stdout.splitlines()
    own = os.getpid()
    hits = []
    for line in proc_lines:
        parts = line.strip().split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid == own:
            continue
        args = parts[1]
        if any(pattern in args for pattern in _OTHER_HOST_PATTERNS):
            hits.append(line.strip())
    return hits


def _all_idle_clean(heartbeats: dict[int, object], stamps: dict[int, float], nodes: Sequence[int], now: float) -> bool:
    for node in nodes:
        hb = heartbeats.get(node)
        stamp = stamps.get(node)
        if hb is None or stamp is None or now - stamp > STALE_SEC:
            return False
        if hb.axis_state != AXIS_STATE_IDLE or hb.faulted:  # type: ignore[attr-defined]
            return False
    return True


def _kt_for(node: int, baselines: Optional[dict[int, UsbAxisBaseline]]) -> float:
    if baselines and node in baselines:
        return float(baselines[node].torque_constant)
    return NOMINAL_KT


def _plant_params(node: int, baselines: Optional[dict[int, UsbAxisBaseline]]) -> tuple[float, float, float]:
    if baselines and node in baselines:
        axis = baselines[node]
        return float(axis.torque_constant), float(axis.vel_gain), float(axis.vel_limit)
    return NOMINAL_KT, NOMINAL_VEL_GAIN, NOMINAL_VEL_LIMIT_TURNS


class SimulatedBus:
    """不接触 SocketCAN 的四轴/单轴对象。测试用它注入故障、外来帧和丢 Iq。"""

    def __init__(
        self,
        nodes: Sequence[int],
        baselines: Optional[dict[int, UsbAxisBaseline]] = None,
        *,
        foreign_mode: Optional[str] = None,
        fault_node: Optional[int] = None,
        fault_after_nonzero: bool = False,
        drop_iq_node: Optional[int] = None,
        drop_iq_after_nonzero: bool = False,
        force_vel: Optional[dict[int, float]] = None,
        vbus_after_nonzero: Optional[float] = None,
    ) -> None:
        self.nodes = tuple(int(node) for node in nodes)
        self.opened = False
        self.tx: list[tuple[int, bytes]] = []
        self.foreign_mode = foreign_mode
        self.foreign_sent = False
        self.fault_node = fault_node
        self.fault_after_nonzero = fault_after_nonzero
        self.drop_iq_node = drop_iq_node
        self.drop_iq_after_nonzero = drop_iq_after_nonzero
        self.force_vel = dict(force_vel or {})
        self.vbus_v = 24.0
        self.ibus_a = 0.1
        self.vbus_after_nonzero = vbus_after_nonzero
        self.saw_nonzero = False
        self.axes = {
            node: {
                "state": AXIS_STATE_IDLE,
                "torque": 0.0,
                "control_mode": CONTROL_MODE_VELOCITY,
                "input_mode": INPUT_MODE_PASSTHROUGH,
            }
            for node in self.nodes
        }
        self.plants = {}
        for node in self.nodes:
            kt, gain, limit = _plant_params(node, baselines)
            self.plants[node] = Plant(kt, gain, limit)

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    def step(self, dt: float) -> None:
        for node in self.nodes:
            axis = self.axes[node]
            torque = axis["torque"] if axis["state"] == AXIS_STATE_CLOSED_LOOP else 0.0
            self.plants[node].step(float(torque), dt)

    def send(self, arb_id: int, data: bytes) -> None:
        cmd, node = parse_can_id(arb_id)
        guard_tx(cmd)
        payload = bytes(data)
        self.tx.append((arb_id, payload))
        if node not in self.axes:
            return
        axis = self.axes[node]
        if cmd == CMD_SET_INPUT_TORQUE:
            torque = struct.unpack("<f", payload[:4])[0]
            axis["torque"] = torque
            if abs(torque) > 1e-9:
                self.saw_nonzero = True
                if self.vbus_after_nonzero is not None:
                    self.vbus_v = float(self.vbus_after_nonzero)
        elif cmd == CMD_SET_AXIS_STATE:
            axis["state"] = struct.unpack("<i", payload[:4])[0]
        elif cmd == CMD_SET_CONTROLLER_MODE:
            mode, input_mode = struct.unpack("<ii", payload[:8])
            axis["control_mode"] = mode
            axis["input_mode"] = input_mode

    def recv(self) -> list[Frame]:
        frames: list[Frame] = []
        if self.foreign_mode == "precheck" and not self.foreign_sent:
            frames.append(Frame(make_can_id(0, CMD_GET_IQ), b""))
            self.foreign_sent = True
        if self.foreign_mode == "motion" and self.saw_nonzero and not self.foreign_sent:
            frames.append(Frame(make_can_id(0, CMD_SET_INPUT_VEL), struct.pack("<ff", 1.0, 0.0)))
            self.foreign_sent = True
        for node in self.nodes:
            axis = self.axes[node]
            axis_error = 0
            if (
                self.fault_node == node
                and self.fault_after_nonzero
                and self.saw_nonzero
            ):
                axis_error = 0x20
            hb = struct.pack("<IBBBB", axis_error, int(axis["state"]), 0, 0, 0)
            frames.append(Frame(make_can_id(node, CMD_HEARTBEAT), hb))
            if node in self.force_vel:
                vel_rad = float(self.force_vel[node])
            else:
                vel_rad = self.plants[node].vel_rad_s
            frames.append(Frame(
                make_can_id(node, CMD_GET_ENCODER_ESTIMATES),
                struct.pack("<ff", 0.0, rad_s_to_turns(vel_rad)),
            ))
            drop = (
                self.drop_iq_node == node
                and self.drop_iq_after_nonzero
                and self.saw_nonzero
            )
            if not drop:
                iq = float(self.plants[node].iq_measured)
                frames.append(Frame(
                    make_can_id(node, CMD_GET_IQ),
                    struct.pack("<ff", iq, iq),
                ))
            frames.append(Frame(
                make_can_id(node, CMD_GET_BUS_VOLTAGE_CURRENT),
                struct.pack("<ff", self.vbus_v, self.ibus_a),
            ))
        return frames


class SocketCanBus:
    """真实 SocketCAN。只有执行门控全部通过后才会被构造。"""

    def __init__(self, channel: str = "can0") -> None:
        self.channel = channel
        self.opened = False
        self._bus = None
        self._can = None

    def open(self) -> None:
        if self.opened and self._bus is not None:
            return
        import can

        self._can = can
        self._bus = can.interface.Bus(interface="socketcan", channel=self.channel)
        self.opened = True

    def close(self) -> None:
        if self._bus is not None:
            shutdown = getattr(self._bus, "shutdown", None)
            if shutdown:
                shutdown()
            self._bus = None
        self.opened = False

    def step(self, dt: float) -> None:
        return None

    def send(self, arb_id: int, data: bytes) -> None:
        cmd, _node = parse_can_id(arb_id)
        guard_tx(cmd)
        if self._bus is None or self._can is None:
            raise CanOpenError("CAN 尚未打开")
        self._bus.send(self._can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=False,
        ))

    def recv(self) -> list[Frame]:
        frames: list[Frame] = []
        if self._bus is None:
            return frames
        while True:
            msg = self._bus.recv(timeout=0.0)
            if msg is None:
                break
            frames.append(Frame(
                arb_id=int(msg.arbitration_id),
                data=bytes(msg.data),
                is_rtr=bool(getattr(msg, "is_remote_frame", False)),
            ))
        return frames


def open_socketcan(channel: str = "can0") -> SocketCanBus:
    bus = SocketCanBus(channel)
    bus.open()
    return bus


def _rejected(reason: str, extra: Optional[Sequence[str]] = None, measurement: str = "rejected") -> DiagResult:
    reasons = [reason]
    if extra:
        reasons.extend(extra)
    text = "\n".join(reasons + ["未打开 CAN。execute_opened_can=false。", AUTHORIZATION_NOTE])
    result = DiagResult(
        exit_code=2,
        measurement=measurement,
        execute_opened_can=False,
        usb_write_attempted=False,
        ok=False,
        fail_closed=True,
        stop_reason=reason,
        reasons=reasons,
        plan_text=text,
    )
    result.summary = summarize(result, None)
    return result


def summarize(result: DiagResult, request: Optional[SessionRequest]) -> dict:
    per_node: dict[str, dict] = {}
    nodes = request.nodes if request is not None else sorted({node for _cmd, node, _data in result.tx})
    torques: dict[int, list[float]] = {int(node): [] for node in nodes}
    idle_nodes = set()
    for cmd, node, data in result.tx:
        if cmd == CMD_SET_INPUT_TORQUE and node in torques and len(data) >= 4:
            torques[node].append(float(struct.unpack("<f", data[:4])[0]))
        if cmd == CMD_SET_AXIS_STATE and len(data) >= 4:
            state = struct.unpack("<i", data[:4])[0]
            if state == AXIS_STATE_IDLE:
                idle_nodes.add(node)
    for node in nodes:
        series = torques.get(int(node), [])
        per_node[str(node)] = {
            "nonzero_torque_commands": any(abs(value) > 1e-9 for value in series),
            "peak_abs_torque_nm": max((abs(value) for value in series), default=0.0),
            "final_torque_nm": series[-1] if series else None,
            "entered_idle_stop": int(node) in idle_nodes,
        }
    forbidden = sorted({
        f"0x{cmd:02x}"
        for cmd, _node, _data in result.tx
        if cmd in FORBIDDEN_CMDS or cmd not in ALLOWED_TX_CMDS
    })
    summary = {
        "tool": None if request is None else request.tool,
        "measurement": result.measurement,
        "execute_opened_can": bool(result.execute_opened_can),
        "usb_write_attempted": bool(result.usb_write_attempted),
        "usb_writes": 1 if result.usb_write_attempted else 0,
        "profile": None if request is None else request.profile,
        "soft_speed_stop": None if request is None else bool(request.soft_speed_stop),
        "nodes": [int(node) for node in nodes],
        "cleared_errors": False,
        "forbidden_cmds_sent": forbidden,
        "fail_closed": bool(result.fail_closed),
        "ok": bool(result.ok),
        "stop_reason": result.stop_reason,
        "reasons": list(result.reasons),
        "per_node": per_node,
        "sample_count": len(result.samples),
        "phase0_only": True,
        "authorization_note": AUTHORIZATION_NOTE,
    }
    return summary


class ProfileSession:
    """单进程力矩剖面。单轴与四轴共用，但入口各自限制节点数。"""

    def __init__(self, request: SessionRequest, bus: object, clock: object, usb_reader: object = None) -> None:
        self.request = request
        self.nodes = tuple(int(node) for node in request.nodes)
        self.mode = request.mode
        self.bus = bus
        self.clock = clock
        self.usb_reader = usb_reader
        self.result = DiagResult(measurement="simulated" if request.mode == "simulate" else "can")
        self.tx: list[tuple[int, int, bytes]] = []
        self.samples: list[dict] = []
        self._recent_tx: list[tuple[int, bytes]] = []
        self.hb: dict[int, object] = {}
        self.hb_time: dict[int, float] = {}
        self.iq_time: dict[int, float] = {}
        self.iq_this_tick: dict[int, tuple[float, float]] = {}
        self.vel: dict[int, float] = {}
        self.overspeed: dict[int, int] = {node: 0 for node in self.nodes}
        self.vbus_v: Optional[float] = None
        self.ibus_a: Optional[float] = None
        self.vbus_time: Optional[float] = None
        self.vbus_base: Optional[float] = None
        self.foreign = False
        self.armed = False
        self.motion_t0 = 0.0
        self._finalized = False

    def run(self) -> DiagResult:
        try:
            self._run()
        except AssertionError:
            raise
        except Exception as exc:
            if self.armed and not self.result.fail_closed:
                self._fail_closed(f"内部错误，fail closed，不清错: {exc}")
            elif self.result.exit_code == 0:
                self.result.exit_code = 2
                self.result.ok = False
                self.result.fail_closed = True
                self.result.stop_reason = f"诊断中断: {exc}"
        finally:
            if self.bus is not None and getattr(self.bus, "opened", False):
                closer = getattr(self.bus, "close", None)
                if closer:
                    try:
                        closer()
                    except Exception:
                        pass
            self._finalize()
        return self.result

    def _run(self) -> None:
        if self.request.tool == TOOL_SINGLE and len(self.nodes) != 1:
            self._reject_before_open(MULTI_AXIS_REJECTION)
            return
        if self.request.tool == TOOL_QUAD and len(self.nodes) < 2:
            self._reject_before_open("单轴请用 odrive_torque_diag。")
            return
        if self.mode == "execute":
            missing = [node for node in self.nodes if not self.request.baselines or node not in self.request.baselines]
            if missing:
                self._reject_before_open(f"缺少 node {missing} 的 USB Kt，不开 CAN")
                return
        try:
            self.bus.open()
        except Exception as exc:
            self.result.execute_opened_can = False
            self.result.exit_code = 2
            self.result.fail_closed = True
            self.result.stop_reason = f"CAN 打开失败: {exc}"
            self.result.reasons.append(self.result.stop_reason)
            return
        if self.mode == "execute":
            self.result.execute_opened_can = True
        precheck = self._precheck()
        if precheck == "foreign":
            self.result.exit_code = 2
            self.result.fail_closed = True
            self.result.ok = False
            self.result.stop_reason = "检测到其它主机帧，不进入闭环"
            self.result.reasons.append(self.result.stop_reason)
            return
        if precheck:
            self.result.exit_code = 2
            self.result.fail_closed = True
            self.result.ok = False
            self.result.stop_reason = precheck
            self.result.reasons.append(precheck)
            return
        self.armed = True
        for node in self.nodes:
            self._tx_mode(node, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH)
            self._tx_state(node, AXIS_STATE_CLOSED_LOOP)
        self.bus.step(0.0)
        self._ingest(self.bus.recv())
        problem = self._motion_problem(self.clock.now(), motion_started=False)
        if problem:
            self._fail_closed(problem)
            return
        segments = build_profile_segments(
            self.request.current_min_a,
            self.request.current_max_a,
            self.request.step_a,
            self.request.slew_a_per_s,
            self.request.hold_sec,
        )
        horizon = schedule_duration(self.nodes, self.request.profile, segments)
        floor = QUAD_BUDGET_FLOOR_SEC if len(self.nodes) > 1 else SINGLE_BUDGET_FLOOR_SEC
        budget = self.request.budget_sec if self.request.budget_sec is not None else max(floor, horizon + 5.0)
        self.motion_t0 = self.clock.now()
        self.overspeed = {node: 0 for node in self.nodes}
        last_tick: Optional[float] = None
        previous = {node: 0.0 for node in self.nodes}
        limit = max(10, int((max(horizon, budget) + 5.0) / TICK_SEC) + 100)
        finished_clean = False
        for _ in range(limit):
            now = self.clock.now()
            if last_tick is not None and now - last_tick > GAP_ABORT_SEC:
                self._fail_closed("调度间隔超过 0.10 s，不追力矩")
                return
            elapsed = now - self.motion_t0
            if elapsed > budget:
                self._fail_closed("总时长超过预算")
                return
            if elapsed >= horizon - 1e-9:
                finished_clean = True
                break
            last_tick = now
            self.bus.step(TICK_SEC)
            self._ingest(self.bus.recv())
            problem = self._motion_problem(now, motion_started=True)
            if problem:
                self._fail_closed(problem)
                return
            for node in self.nodes:
                target = node_current(elapsed, node, self.nodes, self.request.profile, segments)
                command_a = slew_limit(previous[node], target, TICK_SEC, self.request.slew_a_per_s)
                previous[node] = command_a
                kt = _kt_for(node, self.request.baselines)
                torque = command_a * kt
                self._tx_torque(node, torque)
                iq = self.iq_this_tick.get(node)
                self.samples.append({
                    "t_sec": elapsed,
                    "node": node,
                    "current_cmd_a": command_a,
                    "torque_cmd_nm": torque,
                    "iq_setpoint_a": None if iq is None else iq[0],
                    "iq_measured_a": None if iq is None else iq[1],
                    "vel_rad_s": self.vel.get(node),
                    "vbus_v": self.vbus_v,
                    "ibus_a": self.ibus_a,
                })
            self.clock.sleep(TICK_SEC)
        else:
            self._fail_closed("内部迭代超限，全停，不清错")
            return
        if finished_clean:
            self._clean_finish()

    def _reject_before_open(self, reason: str) -> None:
        self.result.exit_code = 2
        self.result.fail_closed = True
        self.result.ok = False
        self.result.execute_opened_can = False
        self.result.measurement = "rejected"
        self.result.stop_reason = reason
        self.result.reasons.append(reason)

    def _precheck(self) -> Optional[str]:
        deadline = self.clock.now() + STALE_SEC
        while True:
            self.bus.step(0.0)
            self._ingest(self.bus.recv())
            if self.foreign:
                return "foreign"
            if _all_idle_clean(self.hb, self.hb_time, self.nodes, self.clock.now()):
                return None
            if self.clock.now() >= deadline:
                return "预检未全部健康 IDLE，不写力矩模式"
            self.clock.sleep(TICK_SEC)

    def _ingest(self, frames: Sequence[Frame]) -> None:
        now = self.clock.now()
        self.iq_this_tick = {}
        for frame in frames:
            cmd, node = parse_can_id(frame.arb_id)
            echoed = any(arb == frame.arb_id and data == frame.data for arb, data in self._recent_tx)
            if _is_foreign_host(cmd, frame.data, frame.is_rtr, echoed):
                self.foreign = True
                continue
            if cmd == CMD_HEARTBEAT and len(frame.data) >= 8:
                try:
                    self.hb[node] = decode_heartbeat(frame.data)
                    self.hb_time[node] = now
                except ValueError:
                    continue
            elif cmd == CMD_GET_ENCODER_ESTIMATES and len(frame.data) >= 8:
                try:
                    _pos, vel_turns = decode_encoder_estimates(frame.data)
                except ValueError:
                    continue
                vel_rad = turns_to_rad(vel_turns)
                self.vel[node] = vel_rad
                if abs(vel_rad) > SOFT_SPEED_STOP_RAD_S:
                    self.overspeed[node] = self.overspeed.get(node, 0) + 1
                else:
                    self.overspeed[node] = 0
            elif cmd == CMD_GET_IQ and len(frame.data) >= 8:
                try:
                    setpoint, measured = decode_iq(frame.data)
                except ValueError:
                    continue
                self.iq_time[node] = now
                self.iq_this_tick[node] = (setpoint, measured)
            elif cmd == CMD_GET_BUS_VOLTAGE_CURRENT and len(frame.data) >= 8:
                try:
                    voltage, current = decode_bus_voltage_current(frame.data)
                except ValueError:
                    continue
                self.vbus_v = voltage
                self.ibus_a = current
                self.vbus_time = now
                if self.vbus_base is None:
                    self.vbus_base = voltage

    def _motion_problem(self, now: float, motion_started: bool) -> Optional[str]:
        if self.foreign:
            return "检测到其它主机帧，全停，不清错"
        for node in self.nodes:
            hb = self.hb.get(node)
            stamp = self.hb_time.get(node)
            if hb is None or stamp is None or now - stamp > STALE_SEC:
                return "心跳过期。零力矩并 IDLE，保留故障，不清错。"
            if hb.faulted:  # type: ignore[attr-defined]
                return "轴错误。零力矩并 IDLE，保留故障，不清错。"
        if not motion_started:
            return None
        if now - self.motion_t0 > STALE_SEC:
            for node in self.nodes:
                stamp = self.iq_time.get(node)
                if stamp is None or now - stamp > STALE_SEC:
                    return "Iq 广播过期。零力矩并 IDLE，不把缺失记成 0 A，不清错。"
            if self.vbus_time is None or now - self.vbus_time > STALE_SEC:
                return "母线广播缺失或过期，全停，不清错。"
        if (
            self.vbus_v is not None
            and self.vbus_base is not None
            and self.vbus_v > self.vbus_base + self.request.vbus_margin_v
        ):
            return "母线电压超过基线 +2.0 V，全停，不清错。"
        if self.request.soft_speed_stop:
            for node in self.nodes:
                if self.overspeed.get(node, 0) >= SOFT_SPEED_STOP_TICKS:
                    return "软停：估计速度连续超过 3.5 rad/s，全停，不清错。"
        return None

    def _fail_closed(self, reason: str) -> None:
        self._command_zero_and_idle()
        self.result.fail_closed = True
        self.result.ok = False
        self.result.exit_code = 2
        self.result.stop_reason = reason
        self.result.reasons.append(reason)
        if self.mode == "execute" and self.usb_reader is not None:
            tx_len = len(self.tx)
            try:
                _baselines, reasons = collect_baselines(self.nodes, self.usb_reader, time.time())
            except Exception as exc:
                reasons = [f"结束后 USB 只读失败: {exc}"]
            if len(self.tx) != tx_len:
                raise RuntimeError("fail closed 之后又写了 CAN")
            self.result.reasons.extend(reasons)

    def _clean_finish(self) -> None:
        self._command_zero_and_idle()
        for node in self.nodes:
            self._tx_mode(node, CONTROL_MODE_VELOCITY, INPUT_MODE_PASSTHROUGH)
        if self.mode != "execute" or self.usb_reader is None:
            self.result.ok = True
            self.result.exit_code = 0
            self.result.stop_reason = None
            return
        tx_len = len(self.tx)
        try:
            _baselines, reasons = collect_baselines(self.nodes, self.usb_reader, time.time())
        except Exception as exc:
            reasons = [str(exc)]
        if len(self.tx) != tx_len:
            raise RuntimeError("USB 核验期间写了 CAN")
        if reasons:
            self.result.fail_closed = True
            self.result.ok = False
            self.result.exit_code = 2
            self.result.stop_reason = "结束后 USB 双板核验失败，fail closed，不再写，不清错。"
            self.result.reasons.extend(reasons)
            return
        self.result.ok = True
        self.result.exit_code = 0

    def _command_zero_and_idle(self) -> None:
        for node in self.nodes:
            self._tx_torque(node, 0.0)
        for node in self.nodes:
            self._tx_state(node, AXIS_STATE_IDLE)

    def _tx_torque(self, node: int, torque_nm: float) -> None:
        arb, data = encode_set_input_torque(node, torque_nm)
        self._transmit(CMD_SET_INPUT_TORQUE, node, arb, data)

    def _tx_state(self, node: int, state: int) -> None:
        arb, data = encode_set_axis_state(node, state)
        self._transmit(CMD_SET_AXIS_STATE, node, arb, data)

    def _tx_mode(self, node: int, control_mode: int, input_mode: int) -> None:
        arb, data = encode_set_controller_mode(node, control_mode, input_mode)
        self._transmit(CMD_SET_CONTROLLER_MODE, node, arb, data)

    def _transmit(self, cmd: int, node: int, arb: int, data: bytes) -> None:
        guard_tx(cmd)
        self.bus.send(arb, data)
        self.tx.append((cmd, node, bytes(data)))
        self._recent_tx.append((arb, bytes(data)))
        if len(self._recent_tx) > 80:
            del self._recent_tx[0]

    def _finalize(self) -> None:
        if self._finalized:
            return
        self._finalized = True
        writes = int(getattr(self.usb_reader, "writes", 0) or 0)
        self.result.usb_write_attempted = writes > 0
        self.result.tx = list(self.tx)
        self.result.samples = list(self.samples)
        self.result.summary = summarize(self.result, self.request)
        lines = [
            f"{self.request.tool} measurement={self.result.measurement}",
            f"execute_opened_can={'true' if self.result.execute_opened_can else 'false'}",
            AUTHORIZATION_NOTE,
        ]
        if self.result.stop_reason:
            lines.append(self.result.stop_reason)
        if self.result.reasons:
            lines.extend(self.result.reasons)
        self.result.plan_text = "\n".join(lines)


def run_session_with_lock(
    request: SessionRequest,
    bus: object,
    clock: object,
    usb_reader: object = None,
    lock_path: Optional[str] = None,
) -> DiagResult:
    lock = DiagFileLock(lock_path or default_lock_path())
    try:
        lock.acquire()
    except DiagLockError as exc:
        return _rejected(str(exc))
    try:
        return ProfileSession(request, bus, clock, usb_reader).run()
    finally:
        lock.release()


def single_plan(node: int) -> str:
    return "\n".join([
        f"odrive_torque_diag 单轴 node {node}",
        FOUR_WHEEL_BAN,
        "多轴请用 odrive_quad_torque_diag，本工具仍禁止多轴。",
        MULTI_AXIS_REJECTION.replace("必须用 --node 指定单个节点 0..3。", "本进程只接受一个 --node。"),
        "未打开 CAN。execute_opened_can=false。",
        AUTHORIZATION_NOTE,
    ])


def _multi_invocation(argv: Sequence[str]) -> bool:
    if any(arg == "--nodes" or arg.startswith("--nodes=") for arg in argv):
        return True
    if sum(1 for arg in argv if arg == "--node") > 1:
        return True
    for index, arg in enumerate(argv):
        if arg == "--node" and index + 1 < len(argv):
            value = argv[index + 1]
            if "," in value or " " in value.strip():
                return True
        if arg.startswith("--node="):
            value = arg.split("=", 1)[1]
            if "," in value or " " in value.strip():
                return True
    return False


def _parse_single_node(text: Optional[str]) -> Optional[int]:
    reasons = hardware_execute_blockers(node=text)
    if reasons:
        return None
    return int(text)  # type: ignore[arg-type]


def _single_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="odrive_torque_diag")
    parser.add_argument("--node", default=None)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--power-attended", action="store_true")
    parser.add_argument("--wheels-raised", action="store_true")
    parser.add_argument("--authorize-ram-torque-mode", action="store_true")
    parser.add_argument("--acknowledge-kt-unverified", action="store_true")
    parser.add_argument("--disable-soft-speed-stop", action="store_true")
    parser.add_argument("--current-max-a", type=float, default=CURRENT_MAX_A)
    parser.add_argument("--current-min-a", type=float, default=CURRENT_MIN_A)
    parser.add_argument("--hold-sec", type=float, default=HOLD_SEC)
    parser.add_argument("--lock-path", default=None)
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--usb-baseline", default=None)
    parser.add_argument("--acknowledge-stale-usb-baseline", action="store_true")
    return parser


def single_execute_blockers(args: argparse.Namespace) -> list[str]:
    reasons = []
    required = (
        ("power_attended", "--power-attended"),
        ("wheels_raised", "--wheels-raised"),
        ("authorize_ram_torque_mode", "--authorize-ram-torque-mode"),
        ("acknowledge_kt_unverified", "--acknowledge-kt-unverified"),
    )
    for attr, flag in required:
        if not getattr(args, attr):
            reasons.append(f"缺少 {flag}")
    if args.usb_baseline and not args.acknowledge_stale_usb_baseline:
        reasons.append("使用缓存 USB baseline 必须同时提供 --acknowledge-stale-usb-baseline")
    return reasons


def _run_single(args: argparse.Namespace, node: int) -> DiagResult:
    soft = not args.disable_soft_speed_stop
    request = SessionRequest(
        tool=TOOL_SINGLE,
        mode="execute" if args.execute else "simulate",
        nodes=(node,),
        profile="sync",
        current_min_a=args.current_min_a,
        current_max_a=args.current_max_a,
        hold_sec=args.hold_sec,
        soft_speed_stop=soft,
    )
    reader = None
    if args.execute:
        reasons = single_execute_blockers(args)
        if reasons:
            return _rejected("；".join(reasons))
        if args.usb_baseline:
            if not args.acknowledge_stale_usb_baseline:
                return _rejected("使用缓存 USB baseline 必须同时提供 --acknowledge-stale-usb-baseline")
            payload = json.loads(open(args.usb_baseline, "r", encoding="utf-8").read())
            snapshot = board_from_json(payload)
            reader = _MapReader({snapshot.serial: snapshot})
        else:
            reader = LiveUsbReader()
        baselines, usb_reasons = collect_baselines((node,), reader, time.time())
        if usb_reasons:
            return _rejected("单轴 USB 门控未通过，不开 CAN。", usb_reasons)
        hosts = find_other_can_hosts()
        if hosts is None:
            return _rejected("无法确认其它 CAN 主机，拒绝执行，不开 CAN。")
        if hosts:
            return _rejected("检测到其它 CAN 主机，拒绝执行，不开 CAN。", hosts)
        request.baselines = baselines
        lock = DiagFileLock(args.lock_path or default_lock_path())
        try:
            lock.acquire()
        except DiagLockError as exc:
            return _rejected(str(exc))
        try:
            try:
                bus = open_socketcan(args.can_channel)
            except Exception as exc:
                if isinstance(exc, AssertionError):
                    raise
                return _rejected(f"CAN 打开失败: {exc}。不开 CAN。")
            result = ProfileSession(request, bus, RealClock(), reader).run()
        finally:
            lock.release()
        result.plan_text = single_plan(node) + "\n" + result.plan_text
        return result
    bus = SimulatedBus((node,))
    clock = VirtualClock()
    result = run_session_with_lock(request, bus, clock, reader, args.lock_path)
    if result.measurement != "rejected":
        result.plan_text = single_plan(node) + "\n" + result.plan_text
    return result


class _MapReader:
    writes = 0

    def __init__(self, boards: dict[str, BoardSnapshot]) -> None:
        self._boards = boards

    def read_board(self, serial: str) -> BoardSnapshot:
        if serial not in self._boards:
            raise UsbReadError(f"没有 {serial} 的 baseline")
        return self._boards[serial]


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv_list = list(sys.argv[1:] if argv is None else argv)
    if _multi_invocation(argv_list):
        print(MULTI_AXIS_REJECTION)
        print(FOUR_WHEEL_BAN)
        print("execute_opened_can=false")
        return 2
    args = _single_parser().parse_args(argv_list)
    node = _parse_single_node(args.node)
    if node is None:
        print(MULTI_AXIS_REJECTION)
        print(FOUR_WHEEL_BAN)
        print("execute_opened_can=false")
        return 2
    if args.execute and args.simulate:
        print("不能同时 --execute 和 --simulate。execute_opened_can=false")
        return 2
    if not args.execute and not args.simulate:
        print(single_plan(node))
        return 0
    if args.execute:
        reasons = single_execute_blockers(args)
        if reasons:
            print("\n".join(reasons))
            print("未打开 CAN。execute_opened_can=false。")
            print(AUTHORIZATION_NOTE)
            return 2
    result = _run_single(args, node)
    print(result.plan_text)
    print(json.dumps(result.summary, ensure_ascii=False))
    return int(result.exit_code)


if __name__ == "__main__":
    sys.exit(main())
