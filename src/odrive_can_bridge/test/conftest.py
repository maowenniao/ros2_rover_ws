"""让 `pytest test/...` 能导入包，并禁止测试碰到真实 CAN。"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import pytest

import odrive_can_bridge.quad_torque_diag as quad_torque_diag
import odrive_can_bridge.torque_diag as torque_diag


@pytest.fixture(autouse=True)
def _phase0_blocks_hardware(monkeypatch):
    def _blocked(*_args, **_kwargs):
        raise AssertionError("Phase0 测试不允许打开 SocketCAN 或现场 USB")

    monkeypatch.setattr(torque_diag, "open_socketcan", _blocked)
    monkeypatch.setattr(quad_torque_diag, "open_socketcan", _blocked)
    monkeypatch.setattr(torque_diag.LiveUsbReader, "read_board", _blocked)
