"""四轴力矩诊断 Phase0。只走仿真和门控，不打开 SocketCAN。"""
import json
from pathlib import Path

import pytest

from odrive_can_bridge.can_protocol import (
    AXIS_STATE_CLOSED_LOOP,
    AXIS_STATE_IDLE,
    CMD_CLEAR_ERRORS,
    CMD_SET_AXIS_STATE,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_LIMITS,
)
from odrive_can_bridge.quad_torque_diag import (
    QuadConfig,
    config_blockers,
    hardware_execute_blockers,
    main,
    run_quad,
)
from odrive_can_bridge.torque_diag import (
    CURRENT_MAX_A,
    CURRENT_MIN_A,
    CURRENT_SLEW_A_PER_S,
    HOLD_SEC,
    LEFT_SERIAL,
    RIGHT_SERIAL,
    TICK_SEC,
    UsbReadError,
    VirtualClock,
    board_to_json,
    build_profile_segments,
    current_at,
    is_host_request,
    make_board,
    node_current,
    profile_duration,
    SimulatedBus,
    DiagFileLock,
)
from odrive_can_bridge.can_protocol import CMD_GET_IQ, CMD_HEARTBEAT

ROOT = Path(__file__).resolve().parents[1]
NODES = (0, 1, 2, 3)
REQUIRED = (
    "--power-attended",
    "--wheels-raised",
    "--authorize-ram-torque-mode",
    "--acknowledge-kt-unverified",
    "--authorize-simultaneous-multi-axis-torque",
    "--acknowledge-bus-regen-risk",
)


class MapReader:
    writes = 0

    def __init__(self, boards):
        self.boards = boards
        self.calls = []

    def read_board(self, serial):
        self.calls.append(serial)
        if serial not in self.boards:
            raise UsbReadError(serial)
        return self.boards[serial]


def fast_cfg(tmp_path, **overrides) -> QuadConfig:
    cfg = QuadConfig(
        simulate=True,
        nodes=NODES,
        profile="staggered",
        current_min_a=0.1,
        current_max_a=0.2,
        hold_sec=0.0,
        lock_path=str(tmp_path / "diag.lock"),
    )
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


def execute_cfg(tmp_path, **overrides) -> QuadConfig:
    cfg = fast_cfg(tmp_path, simulate=False, execute=True)
    cfg.power_attended = True
    cfg.wheels_raised = True
    cfg.authorize_ram_torque_mode = True
    cfg.acknowledge_kt_unverified = True
    cfg.authorize_simultaneous_multi_axis_torque = True
    cfg.acknowledge_bus_regen_risk = True
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


def healthy_reader(now=None):
    moment = 1_700_000_000.0 if now is None else now
    return MapReader({
        LEFT_SERIAL: make_board(LEFT_SERIAL, (0, 1), read_at_unix=moment),
        RIGHT_SERIAL: make_board(RIGHT_SERIAL, (2, 3), read_at_unix=moment),
    }), moment


def assert_closed(result):
    cmds = [cmd for cmd, _node, _data in result.tx]
    assert CMD_CLEAR_ERRORS not in cmds
    assert CMD_SET_LIMITS not in cmds
    assert result.summary["forbidden_cmds_sent"] == []
    assert result.summary["cleared_errors"] is False
    assert result.usb_write_attempted is False
    assert result.execute_opened_can is False or result.measurement == "can"


def zero_and_idle(result):
    idle = {node: False for node in NODES}
    zero = {node: False for node in NODES}
    for cmd, node, data in result.tx:
        if node not in idle:
            continue
        if cmd == CMD_SET_INPUT_TORQUE:
            torque = __import__("struct").unpack("<f", data[:4])[0]
            if torque == 0.0:
                zero[node] = True
        if cmd == CMD_SET_AXIS_STATE:
            state = __import__("struct").unpack("<i", data[:4])[0]
            if state == AXIS_STATE_IDLE:
                idle[node] = True
    assert all(idle.values())
    assert all(zero.values())


def test_dry_run_does_not_open_can(capsys):
    code = main(["--nodes", "0,1,2,3"])
    out = capsys.readouterr().out
    assert code == 0
    assert "未打开 CAN" in out
    assert "execute_opened_can=false" in out
    assert "当次授权" in out
    assert "staggered" in out
    payload = json.loads(out.strip().splitlines()[-1])
    assert payload["execute_opened_can"] is False
    assert payload["measurement"] == "dry-run"
    assert payload["soft_speed_stop"] is True
    assert payload["profile"] == "staggered"


@pytest.mark.parametrize("missing", REQUIRED)
def test_execute_missing_flag_opens_nothing(missing, capsys):
    argv = ["--execute", "--nodes", "0,1,2,3", *REQUIRED]
    argv.remove(missing)
    code = main(argv)
    out = capsys.readouterr().out
    assert code == 2
    assert missing in out
    assert "execute_opened_can=false" in out
    assert "未打开 CAN" in out


def test_default_profile_and_soft_stop():
    cfg = QuadConfig()
    assert cfg.profile == "staggered"
    assert cfg.current_min_a == CURRENT_MIN_A
    assert cfg.current_max_a == CURRENT_MAX_A
    assert cfg.hold_sec == HOLD_SEC
    assert hardware_execute_blockers(cfg) == []
    blocked = QuadConfig(execute=True, disable_soft_speed_stop=True)
    reasons = " ".join(hardware_execute_blockers(blocked))
    assert "--authorize-no-soft-speed-stop-multi-axis" in reasons
    assert config_blockers(QuadConfig(disable_soft_speed_stop=True))


def test_profile_timeline_sync_and_staggered():
    segments = build_profile_segments(0.1, 5.0, 0.1, 0.5, 1.0)
    duration = profile_duration(segments)
    assert duration == pytest.approx(119.0, abs=1e-6)
    peak = max(current_at(index * 0.5, segments) for index in range(int(duration * 2) + 3))
    assert peak == pytest.approx(5.0)
    for node in NODES:
        assert node_current(30.0, node, NODES, "sync", segments) == pytest.approx(current_at(30.0, segments))
    for moment in (0.0, 10.0, 59.0, 80.0, 140.0):
        assert node_current(moment, 1, NODES, "staggered", segments) == pytest.approx(
            node_current(moment - duration / 4.0, 0, NODES, "staggered", segments)
        )
    assert node_current(1.0, 0, NODES, "staggered", segments) > 0
    assert node_current(1.0, 1, NODES, "staggered", segments) == 0
    assert node_current(1.0 + 0.75 * duration, 3, NODES, "staggered", segments) == pytest.approx(
        node_current(1.0, 0, NODES, "staggered", segments)
    )


def test_simulate_four_axes_have_commands(tmp_path):
    result = run_quad(fast_cfg(tmp_path))
    assert result.exit_code == 0
    assert result.measurement == "simulated"
    assert result.execute_opened_can is False
    assert result.ok is True
    assert result.summary["soft_speed_stop"] is True
    for node in NODES:
        assert result.summary["per_node"][str(node)]["nonzero_torque_commands"] is True
    assert_closed(result)


def test_flock_second_instance_does_not_open(tmp_path):
    lock_path = tmp_path / "diag.lock"
    held = DiagFileLock(str(lock_path))
    held.acquire()
    try:
        bus = SimulatedBus(NODES)
        result = run_quad(fast_cfg(tmp_path), bus=bus)
        assert result.exit_code == 2
        assert result.execute_opened_can is False
        assert bus.opened is False
        assert "不写 CAN" in result.plan_text
    finally:
        held.release()


def test_foreign_precheck_does_not_enter_closed_loop(tmp_path):
    bus = SimulatedBus(NODES, foreign_mode="precheck")
    result = run_quad(fast_cfg(tmp_path), bus=bus)
    assert result.exit_code == 2
    assert result.execute_opened_can is False
    assert "不进入闭环" in result.stop_reason
    states = [
        __import__("struct").unpack("<i", data[:4])[0]
        for cmd, _node, data in result.tx
        if cmd == CMD_SET_AXIS_STATE
    ]
    assert AXIS_STATE_CLOSED_LOOP not in states
    assert CMD_SET_INPUT_TORQUE not in [cmd for cmd, _n, _d in result.tx]
    assert_closed(result)


def test_foreign_during_motion_stops_all(tmp_path):
    bus = SimulatedBus(NODES, foreign_mode="motion")
    result = run_quad(fast_cfg(tmp_path), bus=bus)
    assert result.exit_code == 2
    assert "主机" in result.stop_reason
    assert any(
        cmd == CMD_SET_AXIS_STATE and __import__("struct").unpack("<i", data[:4])[0] == AXIS_STATE_CLOSED_LOOP
        for cmd, _node, data in result.tx
    )
    zero_and_idle(result)
    assert_closed(result)


def test_one_axis_fault_stops_all_without_clear(tmp_path):
    bus = SimulatedBus(NODES, fault_node=1, fault_after_nonzero=True)
    result = run_quad(fast_cfg(tmp_path), bus=bus)
    assert result.exit_code == 2
    assert result.fail_closed is True
    assert "轴错误" in result.stop_reason
    assert "不清错" in result.stop_reason
    zero_and_idle(result)
    assert_closed(result)
    assert CMD_CLEAR_ERRORS not in [cmd for cmd, _n, _d in result.tx]


def test_bad_serial_does_not_open_can(tmp_path):
    moment = 1_700_000_000.0
    reader = MapReader({
        LEFT_SERIAL: make_board("deadbeef0000", (0, 1), read_at_unix=moment),
        RIGHT_SERIAL: make_board(RIGHT_SERIAL, (2, 3), read_at_unix=moment),
    })
    opened = []

    def can_factory(_cfg):
        opened.append(True)
        raise AssertionError("CAN opened")

    result = run_quad(
        execute_cfg(tmp_path),
        usb_reader=reader,
        can_factory=can_factory,
        host_probe=lambda: [],
        now=moment,
    )
    assert opened == []
    assert result.exit_code == 2
    assert result.execute_opened_can is False
    assert reader.writes == 0
    assert any("序列号" in reason and "不开 CAN" in reason for reason in result.reasons)
    assert result.tx == []


def test_stale_baseline_file_does_not_open_can(tmp_path):
    moment = 50_000.0
    left = tmp_path / "left.json"
    right = tmp_path / "right.json"
    left.write_text(json.dumps(board_to_json(make_board(LEFT_SERIAL, (0, 1), read_at_unix=0.0))), encoding="utf-8")
    right.write_text(
        json.dumps(board_to_json(make_board(RIGHT_SERIAL, (2, 3), read_at_unix=moment))),
        encoding="utf-8",
    )
    opened = []
    cfg = execute_cfg(
        tmp_path,
        usb_baseline_left=str(left),
        usb_baseline_right=str(right),
        acknowledge_stale_usb_baseline=True,
    )
    result = run_quad(cfg, can_factory=lambda _cfg: opened.append(True), host_probe=lambda: [], now=moment)
    assert opened == []
    assert result.execute_opened_can is False
    assert result.exit_code == 2
    assert any("5 分钟" in reason or "过期" in reason for reason in result.reasons)


def test_cached_baseline_without_acknowledge_is_rejected():
    cfg = execute_cfg(
        Path("/tmp"),
        usb_baseline_left="left.json",
        usb_baseline_right="right.json",
        acknowledge_stale_usb_baseline=False,
        lock_path="/tmp/ignored-quad-lock",
    )
    reasons = " ".join(hardware_execute_blockers(cfg))
    assert "--acknowledge-stale-usb-baseline" in reasons


def test_soft_stop_trips_every_axis(tmp_path):
    bus = SimulatedBus(NODES, force_vel={0: 4.0})
    result = run_quad(fast_cfg(tmp_path), bus=bus)
    assert result.exit_code == 2
    assert "软停" in result.stop_reason
    zero_and_idle(result)
    assert_closed(result)


def test_disabling_soft_stop_requires_second_flag(tmp_path):
    bus = SimulatedBus(NODES, force_vel={0: 10.0})
    refused = run_quad(fast_cfg(tmp_path, disable_soft_speed_stop=True), bus=bus)
    assert refused.exit_code == 2
    assert bus.opened is False
    allowed = run_quad(
        fast_cfg(
            tmp_path,
            disable_soft_speed_stop=True,
            authorize_no_soft_speed_stop_multi_axis=True,
            lock_path=str(tmp_path / "other.lock"),
        ),
        bus=SimulatedBus(NODES, force_vel={0: 10.0}),
    )
    assert allowed.exit_code == 0
    assert allowed.summary["soft_speed_stop"] is False
    assert "软停" not in (allowed.stop_reason or "")


def test_iq_gap_is_not_stored_as_zero(tmp_path):
    bus = SimulatedBus(NODES, drop_iq_node=0, drop_iq_after_nonzero=True)
    result = run_quad(fast_cfg(tmp_path, profile="sync"), bus=bus)
    assert result.exit_code == 2
    assert "Iq" in result.stop_reason
    assert "0 A" in result.stop_reason
    node_samples = [sample for sample in result.samples if sample["node"] == 0]
    assert any(sample["iq_measured_a"] is None for sample in node_samples)
    seen_null = False
    for sample in node_samples:
        if sample["iq_measured_a"] is None:
            seen_null = True
        elif seen_null:
            raise AssertionError(sample)
    assert_closed(result)


def test_gap_does_not_chase_torque(tmp_path):
    class JumpClock(VirtualClock):
        def __init__(self):
            super().__init__()
            self.sleeps = 0

        def sleep(self, dt):
            self.sleeps += 1
            self.t += 0.25 if self.sleeps == 4 else dt

    result = run_quad(fast_cfg(tmp_path), clock=JumpClock())
    assert result.exit_code == 2
    assert "0.10" in result.stop_reason
    assert "不追" in result.stop_reason
    currents = [sample["current_cmd_a"] for sample in result.samples if sample["node"] == 0]
    assert currents
    for earlier, later in zip(currents, currents[1:]):
        assert later - earlier <= CURRENT_SLEW_A_PER_S * TICK_SEC + 1e-6
    zero_and_idle(result)


def test_vbus_jump_stops_all(tmp_path):
    bus = SimulatedBus(NODES, vbus_after_nonzero=30.0)
    result = run_quad(fast_cfg(tmp_path), bus=bus)
    assert result.exit_code == 2
    assert "母线" in result.stop_reason
    zero_and_idle(result)
    assert_closed(result)


def test_post_usb_failure_does_not_write_again(tmp_path):
    moment = 1_700_000_000.0
    good = {
        LEFT_SERIAL: make_board(LEFT_SERIAL, (0, 1), read_at_unix=moment),
        RIGHT_SERIAL: make_board(RIGHT_SERIAL, (2, 3), read_at_unix=moment),
    }
    bad = {
        LEFT_SERIAL: make_board(LEFT_SERIAL, (0, 1), read_at_unix=moment, axis_error=1),
        RIGHT_SERIAL: good[RIGHT_SERIAL],
    }
    bus = SimulatedBus(NODES)

    class TwoPass:
        writes = 0

        def __init__(self):
            self.phase = 0
            self.seen = set()
            self.tx_at_second = None

        def read_board(self, serial):
            if serial in self.seen:
                self.phase = 1
                self.seen = set()
                self.tx_at_second = len(bus.tx)
            self.seen.add(serial)
            table = good if self.phase == 0 else bad
            return table[serial]

    reader = TwoPass()
    result = run_quad(
        execute_cfg(tmp_path),
        usb_reader=reader,
        bus=bus,
        clock=VirtualClock(),
        host_probe=lambda: [],
        now=moment,
    )
    assert result.exit_code == 2
    assert result.fail_closed is True
    assert result.measurement == "can"
    assert "不再写" in result.stop_reason
    assert reader.tx_at_second is not None
    assert len(bus.tx) == reader.tx_at_second
    assert reader.writes == 0
    assert CMD_CLEAR_ERRORS not in [cmd for cmd, _n, _d in result.tx]


def test_other_host_blocks_before_can(tmp_path):
    reader, moment = healthy_reader()
    opened = []
    result = run_quad(
        execute_cfg(tmp_path),
        usb_reader=reader,
        can_factory=lambda _cfg: opened.append(True),
        host_probe=lambda: ["123 odrive_can_node"],
        now=moment,
    )
    assert opened == []
    assert result.execute_opened_can is False
    assert "其它 CAN 主机" in result.stop_reason


def test_eight_byte_iq_broadcast_is_not_a_host_request():
    assert is_host_request(CMD_GET_IQ, b"\x00" * 8) is False
    assert is_host_request(CMD_HEARTBEAT, b"\x00" * 8) is False
    assert is_host_request(CMD_GET_IQ, b"") is True


def test_source_has_no_clear_save_or_limit_write():
    quad = (ROOT / "odrive_can_bridge" / "quad_torque_diag.py").read_text(encoding="utf-8")
    single = (ROOT / "odrive_can_bridge" / "torque_diag.py").read_text(encoding="utf-8")
    for banned in ("save_configuration", "erase_configuration", "write_configuration", "encode_set_limits"):
        assert banned not in quad
        assert banned not in single
    assert "CMD_CLEAR_ERRORS" not in quad
    assert "CMD_SET_LIMITS" not in quad
    assert "CMD_CLEAR_ERRORS" in single
    assert "FORBIDDEN_CMDS" in single
    doc = (ROOT / "docs" / "QUAD_TORQUE_DIAG.md").read_text(encoding="utf-8")
    assert "实跑需用户当次授权" in doc
    assert LEFT_SERIAL in doc and RIGHT_SERIAL in doc


def test_cli_simulate_reports_simulated(tmp_path, capsys):
    code = main([
        "--simulate",
        "--nodes", "0,1,2,3",
        "--current-max-a", "0.2",
        "--hold-sec", "0",
        "--lock-path", str(tmp_path / "cli.lock"),
    ])
    out = capsys.readouterr().out
    assert code == 0
    payload = json.loads(out.strip().splitlines()[-1])
    assert payload["measurement"] == "simulated"
    assert payload["execute_opened_can"] is False
    assert all(payload["per_node"][str(node)]["nonzero_torque_commands"] for node in NODES)


def test_budget_abort_stops_all(tmp_path):
    result = run_quad(fast_cfg(tmp_path, budget_sec=0.05))
    assert result.exit_code == 2
    assert "预算" in result.stop_reason
    zero_and_idle(result)
    assert_closed(result)
