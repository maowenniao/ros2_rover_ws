"""单轴力矩诊断仍然拒绝多轴。不需要硬件。"""
import odrive_can_bridge.torque_diag as torque_diag
from odrive_can_bridge.can_protocol import CMD_CLEAR_ERRORS, CMD_SET_LIMITS
from odrive_can_bridge.torque_diag import (
    FOUR_WHEEL_BAN,
    MULTI_AXIS_REJECTION,
    NOMINAL_KT,
    NOMINAL_VEL_GAIN,
    NOMINAL_VEL_LIMIT_TURNS,
    guard_tx,
    hardware_execute_blockers,
    limit_torque_like_fw056,
    main,
)


def test_missing_node_is_rejected():
    assert main([]) == 2
    assert hardware_execute_blockers(node=None) == [MULTI_AXIS_REJECTION]


def test_comma_separated_node_is_rejected(capsys):
    assert main(["--node", "0,1"]) == 2
    assert MULTI_AXIS_REJECTION in capsys.readouterr().out


def test_nodes_flag_is_rejected(capsys):
    assert main(["--nodes", "0,1,2,3"]) == 2
    assert "不做四轮同时力矩" in capsys.readouterr().out


def test_repeated_node_flag_is_rejected(capsys):
    assert main(["--node", "0", "--node", "1"]) == 2
    assert MULTI_AXIS_REJECTION in capsys.readouterr().out


def test_out_of_range_node_is_rejected():
    assert hardware_execute_blockers(node=4) == [MULTI_AXIS_REJECTION]
    assert hardware_execute_blockers(node=-1) == [MULTI_AXIS_REJECTION]
    assert main(["--node", "4"]) == 2


def test_single_node_is_not_a_multi_axis_block():
    assert hardware_execute_blockers(node=0) == []
    assert hardware_execute_blockers(node="2") == []
    assert hardware_execute_blockers(nodes="0,1") == [MULTI_AXIS_REJECTION]


def test_plan_text_forbids_four_wheels_at_once(capsys):
    assert main(["--node", "1"]) == 0
    out = capsys.readouterr().out
    assert FOUR_WHEEL_BAN in out
    assert "odrive_quad_torque_diag" in out
    assert "execute_opened_can=false" in out
    assert "当次授权" in out


def test_execute_without_flags_does_not_open_can(capsys):
    code = main(["--node", "0", "--execute"])
    out = capsys.readouterr().out
    assert code == 2
    assert "execute_opened_can=false" in out
    assert "--power-attended" in out


def test_simulate_one_axis_stays_off_can(capsys, tmp_path):
    code = main([
        "--node", "0",
        "--simulate",
        "--current-max-a", "0.2",
        "--hold-sec", "0",
        "--lock-path", str(tmp_path / "one.lock"),
    ])
    out = capsys.readouterr().out
    assert code == 0
    assert '"measurement": "simulated"' in out
    assert '"execute_opened_can": false' in out


def test_zero_speed_clamp_matches_fw056_estimate():
    limited = limit_torque_like_fw056(10.0, 0.0, NOMINAL_VEL_LIMIT_TURNS, NOMINAL_VEL_GAIN, True)
    clamp_a = limited / NOMINAL_KT
    assert clamp_a == (NOMINAL_VEL_GAIN * NOMINAL_VEL_LIMIT_TURNS / NOMINAL_KT)
    assert abs(clamp_a - 2.87) < 0.02
    assert limit_torque_like_fw056(1.0, NOMINAL_VEL_LIMIT_TURNS, NOMINAL_VEL_LIMIT_TURNS, NOMINAL_VEL_GAIN) == 0
    negative = limit_torque_like_fw056(-10.0, 0.0, NOMINAL_VEL_LIMIT_TURNS, NOMINAL_VEL_GAIN)
    assert negative == -(NOMINAL_VEL_LIMIT_TURNS * NOMINAL_VEL_GAIN)


def test_guard_rejects_clear_and_limit_writes():
    for cmd in (CMD_CLEAR_ERRORS, CMD_SET_LIMITS):
        try:
            guard_tx(cmd)
        except RuntimeError as exc:
            assert "禁止" in str(exc)
        else:
            raise AssertionError(f"命令 {cmd:#x} 不应被放行")


def test_session_refuses_multi_axis_before_open():
    from odrive_can_bridge.torque_diag import (
        TOOL_SINGLE,
        ProfileSession,
        SessionRequest,
        SimulatedBus,
        VirtualClock,
    )

    bus = SimulatedBus((0, 1, 2, 3))
    request = SessionRequest(tool=TOOL_SINGLE, mode="simulate", nodes=(0, 1, 2, 3))
    result = ProfileSession(request, bus, VirtualClock()).run()
    assert result.exit_code == 2
    assert result.execute_opened_can is False
    assert bus.opened is False
    assert MULTI_AXIS_REJECTION in result.stop_reason
    assert result.tx == []
