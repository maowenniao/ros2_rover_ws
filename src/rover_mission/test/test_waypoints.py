from geometry_msgs.msg import PoseStamped

from rover_mission.waypoint_mission_node import as_bool, filter_close_waypoints


def _pose(x, y):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def test_string_false_is_not_truthy():
    assert as_bool('false') is False
    assert as_bool('true') is True
    assert as_bool(False) is False


def test_close_waypoints_are_dropped():
    poses = [_pose(0, 0), _pose(0.1, 0), _pose(1.0, 0)]
    kept = filter_close_waypoints(poses, 0.3)
    assert len(kept) == 2
    assert kept[1].pose.position.x == 1.0
