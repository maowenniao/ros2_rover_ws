import math

import pytest

from rover_navigation.gps_waypoints import (
    bearing_enu,
    parse_gps_waypoints,
    should_send,
    yaw_to_quat,
)


def test_bearing_sign_matches_enu():
    north = bearing_enu(31.0, 121.0, 31.001, 121.0)
    east = bearing_enu(31.0, 121.0, 31.0, 121.001)
    south = bearing_enu(31.0, 121.0, 30.999, 121.0)
    west = bearing_enu(31.0, 121.0, 31.0, 120.999)
    assert abs(north - math.pi / 2.0) < 1e-6
    assert abs(east) < 1e-6
    assert abs(south + math.pi / 2.0) < 1e-6
    assert abs(abs(west) - math.pi) < 1e-3


def test_missing_yaw_faces_the_next_point_and_explicit_yaw_is_kept():
    points = parse_gps_waypoints({'waypoints': [
        {'latitude': 31.0, 'longitude': 121.0},
        {'latitude': 31.001, 'longitude': 121.0, 'yaw': 0.3, 'altitude': 4.0},
    ]})
    assert abs(points[0].yaw - math.pi / 2.0) < 1e-6
    assert points[0].altitude == 0.0
    assert abs(points[1].yaw - 0.3) < 1e-9
    assert points[1].altitude == 4.0
    assert points[1].latitude == 31.001
    quat = yaw_to_quat(points[0].yaw)
    assert abs(quat[2] - math.sin(math.pi / 4.0)) < 1e-6
    assert quat[0] == 0.0


def test_south_and_west_coordinates_stay_negative():
    points = parse_gps_waypoints({'waypoints': [
        {'latitude': -33.8, 'longitude': -151.2, 'yaw': 0.0},
    ]})
    assert points[0].latitude == -33.8
    assert points[0].longitude == -151.2


def test_bad_coordinates_are_rejected():
    with pytest.raises(ValueError):
        parse_gps_waypoints({'waypoints': [{'latitude': 95.0, 'longitude': 0.0}]})
    with pytest.raises(ValueError):
        parse_gps_waypoints({'waypoints': [{'latitude': 1.0, 'longitude': 200.0}]})


def test_waypoints_wait_for_a_fix():
    assert should_send(True, False, False, 30.0, 8.0) is False
    assert should_send(True, False, True, 1.0, 8.0) is False
    assert should_send(False, False, True, 30.0, 8.0) is False
    assert should_send(True, True, True, 30.0, 8.0) is False
    assert should_send(True, False, True, None, 8.0) is False
    assert should_send(True, False, True, 8.0, 8.0) is True
