"""四轴同时力矩/电流梯度诊断。Phase0：仿真、dry-run 与执行门控。

实跑（--execute）需用户当次授权。缺旗标、双板 USB 只读失败或抢不到文件锁时，
不开 CAN。一轴故障则四轴力矩 0 并 IDLE，不清错，不保存配置，不写上限。
单轴工具 ``odrive_torque_diag`` 仍拒绝多轴。
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

from odrive_can_bridge.torque_diag import (
    AUTHORIZATION_NOTE,
    CURRENT_MAX_A,
    CURRENT_MIN_A,
    CURRENT_SLEW_A_PER_S,
    CURRENT_STEP_A,
    HOLD_SEC,
    LEFT_SERIAL,
    QUAD_BUDGET_FLOOR_SEC,
    RIGHT_SERIAL,
    TOOL_QUAD,
    BoardSnapshot,
    DiagFileLock,
    DiagLockError,
    DiagResult,
    LiveUsbReader,
    RealClock,
    SessionRequest,
    SimulatedBus,
    UsbReadError,
    VirtualClock,
    _rejected,
    board_from_json,
    boards_for_nodes,
    collect_baselines,
    default_lock_path,
    find_other_can_hosts,
    open_socketcan,
)

ALLOWED_NODE_SETS = {
    (0, 1),
    (2, 3),
    (0, 1, 2, 3),
}

_REQUIRED_EXECUTE_FLAGS = (
    ("power_attended", "--power-attended"),
    ("wheels_raised", "--wheels-raised"),
    ("authorize_ram_torque_mode", "--authorize-ram-torque-mode"),
    ("acknowledge_kt_unverified", "--acknowledge-kt-unverified"),
    ("authorize_simultaneous_multi_axis_torque", "--authorize-simultaneous-multi-axis-torque"),
    ("acknowledge_bus_regen_risk", "--acknowledge-bus-regen-risk"),
)


@dataclass
class QuadConfig:
    execute: bool = False
    simulate: bool = False
    nodes: tuple[int, ...] = (0, 1, 2, 3)
    profile: str = "staggered"
    current_min_a: float = CURRENT_MIN_A
    current_max_a: float = CURRENT_MAX_A
    step_a: float = CURRENT_STEP_A
    slew_a_per_s: float = CURRENT_SLEW_A_PER_S
    hold_sec: float = HOLD_SEC
    disable_soft_speed_stop: bool = False
    authorize_no_soft_speed_stop_multi_axis: bool = False
    power_attended: bool = False
    wheels_raised: bool = False
    authorize_ram_torque_mode: bool = False
    acknowledge_kt_unverified: bool = False
    authorize_simultaneous_multi_axis_torque: bool = False
    acknowledge_bus_regen_risk: bool = False
    no_other_can_host_confirmed: bool = False
    acknowledge_stale_usb_baseline: bool = False
    usb_baseline_left: Optional[str] = None
    usb_baseline_right: Optional[str] = None
    lock_path: Optional[str] = None
    json_out: Optional[str] = None
    can_channel: str = "can0"
    budget_sec: Optional[float] = None

    @property
    def mode(self) -> str:
        if self.execute and self.simulate:
            return "invalid"
        if self.execute:
            return "execute"
        if self.simulate:
            return "simulate"
        return "dry-run"


def effective_soft_stop(cfg: QuadConfig) -> bool:
    return not (
        cfg.disable_soft_speed_stop and cfg.authorize_no_soft_speed_stop_multi_axis
    )


def parse_nodes(text: str) -> tuple[int, ...]:
    parts = [part.strip() for part in str(text).split(",") if part.strip()]
    if not parts:
        raise ValueError("节点列表为空")
    return tuple(sorted({int(part) for part in parts}))


def config_blockers(cfg: QuadConfig) -> list[str]:
    reasons = []
    if cfg.disable_soft_speed_stop and not cfg.authorize_no_soft_speed_stop_multi_axis:
        reasons.append("关闭软停需要同时提供 --authorize-no-soft-speed-stop-multi-axis")
    if cfg.nodes not in ALLOWED_NODE_SETS:
        reasons.append(
            "节点组合只允许同板 0,1 或 2,3，或四轴 0,1,2,3。单轴请用 odrive_torque_diag。"
        )
    if cfg.profile not in ("staggered", "sync"):
        reasons.append("profile 必须是 staggered 或 sync")
    if not (0 < cfg.current_min_a <= cfg.current_max_a <= CURRENT_MAX_A):
        reasons.append("电流范围必须在 (0, 5.0] A 内，且 min<=max")
    if cfg.step_a <= 0 or cfg.slew_a_per_s <= 0 or cfg.hold_sec < 0:
        reasons.append("步长、斜率必须为正，hold 不能为负")
    return reasons


def hardware_execute_blockers(cfg: QuadConfig) -> list[str]:
    """缺任一强制旗标则拒绝执行，且不得打开 USB/CAN。"""
    reasons = list(config_blockers(cfg))
    if not cfg.execute:
        return reasons
    for attr, flag in _REQUIRED_EXECUTE_FLAGS:
        if not getattr(cfg, attr):
            reasons.append(f"缺少 {flag}")
    if (cfg.usb_baseline_left or cfg.usb_baseline_right) and not cfg.acknowledge_stale_usb_baseline:
        reasons.append("使用缓存 USB baseline 必须同时提供 --acknowledge-stale-usb-baseline")
    return reasons


def plan_text(cfg: QuadConfig) -> str:
    soft = "开" if effective_soft_stop(cfg) else "关"
    return "\n".join([
        "odrive_quad_torque_diag Phase0",
        f"节点: {','.join(str(node) for node in cfg.nodes)}",
        f"剖面: {cfg.profile}（默认 staggered，错相 90°）",
        (
            f"电流: {cfg.current_min_a:g}→{cfg.current_max_a:g} A，"
            f"斜率 {cfg.slew_a_per_s:g} A/s，hold {cfg.hold_sec:g} s"
        ),
        f"软停: {soft}（默认开，3.5 rad/s × 3 tick）",
        f"左板 {LEFT_SERIAL} 轴 0,1；右板 {RIGHT_SERIAL} 轴 2,3。USB 只读，不写配置。",
        "禁止清错、禁止保存配置、禁止写 vel_limit/current_lim。",
        "一轴故障则全部轴力矩 0 并 IDLE，不清错。",
        "单轴工具 odrive_torque_diag 仍然禁止四轮同时力矩。",
        AUTHORIZATION_NOTE,
        "未打开 CAN。execute_opened_can=false。",
        f"建议预算不低于 {QUAD_BUDGET_FLOOR_SEC:.0f} s；错相全剖面会按实际时长放宽。",
    ])


class JsonFileUsbReader:
    """缓存 baseline 的只读入口。不写设备。"""

    writes = 0

    def __init__(self, paths: dict[str, Optional[str]]) -> None:
        self.reasons: list[str] = []
        self._boards: dict[str, BoardSnapshot] = {}
        for serial, path in paths.items():
            if not path:
                self.reasons.append(f"缺少 {serial} 的 USB baseline 文件，不开 CAN")
                continue
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
            self._boards[serial] = board_from_json(payload)

    def read_board(self, serial: str) -> BoardSnapshot:
        if serial not in self._boards:
            raise UsbReadError(f"没有 {serial} 的 baseline 文件")
        return self._boards[serial]


def _baseline_paths(cfg: QuadConfig) -> dict[str, Optional[str]]:
    paths: dict[str, Optional[str]] = {}
    needed = {board.serial for board in boards_for_nodes(cfg.nodes)}
    if LEFT_SERIAL in needed:
        paths[LEFT_SERIAL] = cfg.usb_baseline_left
    if RIGHT_SERIAL in needed:
        paths[RIGHT_SERIAL] = cfg.usb_baseline_right
    return paths


def _request(cfg: QuadConfig, baselines) -> SessionRequest:
    return SessionRequest(
        tool=TOOL_QUAD,
        mode="execute" if cfg.mode == "execute" else "simulate",
        nodes=tuple(cfg.nodes),
        profile=cfg.profile,
        current_min_a=cfg.current_min_a,
        current_max_a=cfg.current_max_a,
        step_a=cfg.step_a,
        slew_a_per_s=cfg.slew_a_per_s,
        hold_sec=cfg.hold_sec,
        soft_speed_stop=effective_soft_stop(cfg),
        budget_sec=cfg.budget_sec,
        baselines=baselines,
    )


def _dry_result(cfg: QuadConfig) -> DiagResult:
    result = DiagResult(
        exit_code=0,
        measurement="dry-run",
        execute_opened_can=False,
        usb_write_attempted=False,
        ok=True,
        fail_closed=False,
        plan_text=plan_text(cfg),
    )
    result.summary = {
        "tool": TOOL_QUAD,
        "measurement": "dry-run",
        "execute_opened_can": False,
        "usb_write_attempted": False,
        "usb_writes": 0,
        "profile": cfg.profile,
        "soft_speed_stop": effective_soft_stop(cfg),
        "nodes": list(cfg.nodes),
        "cleared_errors": False,
        "forbidden_cmds_sent": [],
        "fail_closed": False,
        "ok": True,
        "stop_reason": None,
        "reasons": [],
        "phase0_only": True,
        "authorization_note": AUTHORIZATION_NOTE,
    }
    return result


def run_quad(
    cfg: QuadConfig,
    *,
    usb_reader: object = None,
    bus: object = None,
    clock: object = None,
    host_probe=None,
    can_factory=None,
    now: Optional[float] = None,
) -> DiagResult:
    """执行门控之后才可能打开 CAN。仿真不打开 SocketCAN。"""
    if cfg.mode == "invalid":
        return _rejected("不能同时 --execute 和 --simulate。")
    if cfg.mode == "dry-run":
        blockers = config_blockers(cfg)
        if blockers:
            return _rejected("；".join(blockers))
        return _dry_result(cfg)

    blockers = hardware_execute_blockers(cfg) if cfg.mode == "execute" else config_blockers(cfg)
    if blockers:
        return _rejected("；".join(blockers))

    moment = time.time() if now is None else float(now)
    lock = DiagFileLock(cfg.lock_path or default_lock_path())
    try:
        lock.acquire()
    except DiagLockError as exc:
        return _rejected(str(exc))

    try:
        reader = None
        baselines = None
        if cfg.mode == "execute":
            if usb_reader is not None:
                reader = usb_reader
            elif cfg.usb_baseline_left or cfg.usb_baseline_right:
                reader = JsonFileUsbReader(_baseline_paths(cfg))
            else:
                reader = LiveUsbReader()
            baselines, reasons = collect_baselines(cfg.nodes, reader, moment)
            writes = int(getattr(reader, "writes", 0) or 0)
            if writes:
                refused = _rejected("USB 入口出现写入，拒绝执行，不开 CAN。")
                refused.usb_write_attempted = True
                return refused
            if reasons:
                return _rejected("双板 USB 门控未通过，不开 CAN。", reasons)
            if host_probe is None:
                hosts = find_other_can_hosts()
            else:
                hosts = host_probe()
            if hosts is None and not cfg.no_other_can_host_confirmed:
                return _rejected("无法确认其它 CAN 主机，拒绝执行，不开 CAN。")
            if hosts and not cfg.no_other_can_host_confirmed:
                return _rejected("检测到其它 CAN 主机，拒绝执行，不开 CAN。", list(hosts))
            if bus is None:
                try:
                    if can_factory is not None:
                        bus = can_factory(cfg)
                    else:
                        bus = open_socketcan(cfg.can_channel)
                except Exception as exc:
                    if isinstance(exc, AssertionError):
                        raise
                    return _rejected(f"CAN 打开失败: {exc}。不开 CAN。")
            if clock is None:
                clock = RealClock()
        else:
            if bus is None:
                bus = SimulatedBus(cfg.nodes)
            if clock is None:
                clock = VirtualClock()
        return _run_profile(cfg, baselines, bus, clock, reader)
    finally:
        lock.release()


def _run_profile(cfg, baselines, bus, clock, reader) -> DiagResult:
    from odrive_can_bridge.torque_diag import ProfileSession

    return ProfileSession(_request(cfg, baselines), bus, clock, reader).run()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="odrive_quad_torque_diag")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--nodes", default="0,1,2,3")
    parser.add_argument("--profile", choices=("staggered", "sync"), default="staggered")
    parser.add_argument("--current-max-a", type=float, default=CURRENT_MAX_A)
    parser.add_argument("--current-min-a", type=float, default=CURRENT_MIN_A)
    parser.add_argument("--step-a", type=float, default=CURRENT_STEP_A)
    parser.add_argument("--slew-a-per-s", type=float, default=CURRENT_SLEW_A_PER_S)
    parser.add_argument("--hold-sec", type=float, default=HOLD_SEC)
    parser.add_argument("--power-attended", action="store_true")
    parser.add_argument("--wheels-raised", action="store_true")
    parser.add_argument("--authorize-ram-torque-mode", action="store_true")
    parser.add_argument("--acknowledge-kt-unverified", action="store_true")
    parser.add_argument("--authorize-simultaneous-multi-axis-torque", action="store_true")
    parser.add_argument("--acknowledge-bus-regen-risk", action="store_true")
    parser.add_argument("--disable-soft-speed-stop", action="store_true")
    parser.add_argument("--authorize-no-soft-speed-stop-multi-axis", action="store_true")
    parser.add_argument("--no-other-can-host-confirmed", action="store_true")
    parser.add_argument("--usb-baseline-left", default=None)
    parser.add_argument("--usb-baseline-right", default=None)
    parser.add_argument("--acknowledge-stale-usb-baseline", action="store_true")
    parser.add_argument("--lock-path", default=None)
    parser.add_argument("--json-out", default=None)
    parser.add_argument("--can-channel", default="can0")
    parser.add_argument("--budget-sec", type=float, default=None)
    return parser


def config_from_args(args: argparse.Namespace) -> QuadConfig:
    return QuadConfig(
        execute=bool(args.execute),
        simulate=bool(args.simulate),
        nodes=parse_nodes(args.nodes),
        profile=args.profile,
        current_min_a=args.current_min_a,
        current_max_a=args.current_max_a,
        step_a=args.step_a,
        slew_a_per_s=args.slew_a_per_s,
        hold_sec=args.hold_sec,
        disable_soft_speed_stop=bool(args.disable_soft_speed_stop),
        authorize_no_soft_speed_stop_multi_axis=bool(args.authorize_no_soft_speed_stop_multi_axis),
        power_attended=bool(args.power_attended),
        wheels_raised=bool(args.wheels_raised),
        authorize_ram_torque_mode=bool(args.authorize_ram_torque_mode),
        acknowledge_kt_unverified=bool(args.acknowledge_kt_unverified),
        authorize_simultaneous_multi_axis_torque=bool(args.authorize_simultaneous_multi_axis_torque),
        acknowledge_bus_regen_risk=bool(args.acknowledge_bus_regen_risk),
        no_other_can_host_confirmed=bool(args.no_other_can_host_confirmed),
        acknowledge_stale_usb_baseline=bool(args.acknowledge_stale_usb_baseline),
        usb_baseline_left=args.usb_baseline_left,
        usb_baseline_right=args.usb_baseline_right,
        lock_path=args.lock_path,
        json_out=args.json_out,
        can_channel=args.can_channel,
        budget_sec=args.budget_sec,
    )


def _emit(result: DiagResult, json_out: Optional[str]) -> None:
    print(result.plan_text)
    encoded = json.dumps(result.summary, ensure_ascii=False)
    print(encoded)
    if json_out:
        Path(json_out).write_text(encoded + "\n", encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    argv_list = list(sys.argv[1:] if argv is None else argv)
    try:
        args = _parser().parse_args(argv_list)
        cfg = config_from_args(args)
    except (ValueError, argparse.ArgumentError) as exc:
        print(f"参数无效: {exc}")
        print("未打开 CAN。execute_opened_can=false。")
        return 2
    result = run_quad(cfg)
    _emit(result, cfg.json_out)
    return int(result.exit_code)


if __name__ == "__main__":
    sys.exit(main())
