def test_launch_has_no_hardcoded_home():
    from pathlib import Path
    text = (Path(__file__).resolve().parents[1] / 'launch' / 'full_system.launch.py').read_text(encoding='utf-8')
    assert '/home/ai' not in text
    assert 'MicroXRCEAgent' in text
    assert 'LaunchConfiguration object' not in text
    assert 'odrive_can.launch.py' in text
    assert 'cmdvel_to_setpoint' not in text
