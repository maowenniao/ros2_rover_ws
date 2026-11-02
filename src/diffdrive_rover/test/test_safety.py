from geometry_msgs.msg import Twist

from diffdrive_rover.safety import limit_twist


def _cmd(vx=0.4, wz=0.2):
    msg = Twist()
    msg.linear.x = vx
    msg.angular.z = wz
    return msg


def test_command_timeout_publishes_zero():
    out = limit_twist(_cmd(), None, 0.5, 0.0, 0.5, False, 10.0, 0.45, 1.2)
    assert out.linear.x == 0.0 and out.angular.z == 0.0
    out = limit_twist(_cmd(), 1.0, 0.5, 0.0, 0.5, False, 10.0, 0.45, 1.2)
    assert out.linear.x == 0.0


def test_missing_scan_stops_when_required():
    out = limit_twist(_cmd(), 0.1, 0.5, None, 0.5, True, 10.0, 0.45, 1.2)
    assert out.linear.x == 0.0
    out = limit_twist(_cmd(), 0.1, 0.5, None, 0.5, False, 10.0, 0.45, 1.2)
    assert abs(out.linear.x - 0.4) < 1e-9


def test_obstacle_stops_forward_but_not_reverse():
    blocked = limit_twist(_cmd(0.4, 0.2), 0.1, 0.5, 0.1, 0.5, True, 0.2, 0.45, 1.2)
    assert blocked.linear.x == 0.0
    assert abs(blocked.angular.z - 0.05) < 1e-9
    reverse = limit_twist(_cmd(-0.3, 0.2), 0.1, 0.5, 0.1, 0.5, True, 0.2, 0.45, 1.2)
    assert abs(reverse.linear.x + 0.3) < 1e-9
    assert abs(reverse.angular.z - 0.2) < 1e-9
