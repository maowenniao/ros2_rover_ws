import math
from pathlib import Path

import yaml

from diffdrive_rover.gps_logic import (
    COVARIANCE_TYPE_APPROXIMATED,
    COVARIANCE_TYPE_DIAGONAL_KNOWN,
    DIAG_OK,
    DIAG_STALE,
    DIAG_WARN,
    FIX_TYPE_2D,
    FIX_TYPE_3D,
    FIX_TYPE_EXTRAPOLATED,
    FIX_TYPE_NONE,
    FIX_TYPE_RTCM,
    FIX_TYPE_RTK_FIXED,
    FIX_TYPE_RTK_FLOAT,
    POSE_FRAME_NED,
    STATUS_FIX,
    STATUS_GBAS_FIX,
    STATUS_SBAS_FIX,
    enu_orientation_from_odometry,
    frd_ned_quat_to_flu_enu,
    gps_diagnostic,
    sensor_gps_to_navsat,
    yaw_from_ros_quat,
)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _ned_yaw(yaw: float):
    """NED 正航向是从上往下看顺时针，四元数 wxyz。"""
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def test_no_fix_is_not_published():
    for fix_type in (0, FIX_TYPE_NONE, FIX_TYPE_EXTRAPOLATED, 7):
        assert sensor_gps_to_navsat(31.2, 121.5, 12.0, fix_type, 1.5, 2.5) is None
    assert sensor_gps_to_navsat(math.nan, 121.5, 12.0, FIX_TYPE_3D, 1.5, 2.5) is None
    assert sensor_gps_to_navsat(91.0, 121.5, 12.0, FIX_TYPE_3D, 1.5, 2.5) is None
    assert sensor_gps_to_navsat(31.2, 181.0, 12.0, FIX_TYPE_3D, 1.5, 2.5) is None


def test_lat_lon_altitude_keep_sign_and_use_ellipsoid():
    # 南纬、西经保持为负。椭球高向上为正，向下为负，不改符号，也不换成海平面高度。
    south = sensor_gps_to_navsat(-33.8, -151.2, 48.5, FIX_TYPE_3D, 1.2, 2.0)
    assert south is not None
    assert south.latitude == -33.8
    assert south.longitude == -151.2
    assert south.altitude == 48.5
    assert south.frame_id == 'gps_link'
    below = sensor_gps_to_navsat(10.0, 20.0, -3.5, FIX_TYPE_3D, 1.0, 1.0)
    assert below is not None
    assert below.altitude == -3.5


def test_fix_status_and_covariance_from_eph_epv():
    fix = sensor_gps_to_navsat(31.2, 121.5, 8.0, FIX_TYPE_3D, 2.0, 3.0)
    assert fix is not None
    assert fix.status == STATUS_FIX
    assert fix.covariance_type == COVARIANCE_TYPE_DIAGONAL_KNOWN
    assert fix.covariance[0] == 4.0
    assert fix.covariance[4] == 4.0
    assert fix.covariance[8] == 9.0
    assert fix.covariance[1] == 0.0
    assert fix.covariance[2] == 0.0

    dgps = sensor_gps_to_navsat(31.2, 121.5, 8.0, FIX_TYPE_RTCM, 0.8, 1.5)
    rtk_float = sensor_gps_to_navsat(31.2, 121.5, 8.0, FIX_TYPE_RTK_FLOAT, 0.2, 0.4)
    rtk_fixed = sensor_gps_to_navsat(31.2, 121.5, 8.0, FIX_TYPE_RTK_FIXED, 0.02, 0.04)
    assert dgps is not None and dgps.status == STATUS_SBAS_FIX
    assert rtk_float is not None and rtk_float.status == STATUS_GBAS_FIX
    assert rtk_fixed is not None and rtk_fixed.status == STATUS_GBAS_FIX
    assert rtk_fixed.covariance[0] == 0.02 * 0.02
    assert rtk_fixed.covariance[8] == 0.04 * 0.04

    missing = sensor_gps_to_navsat(31.2, 121.5, 8.0, FIX_TYPE_2D, 0.0, math.nan)
    assert missing is not None
    assert missing.status == STATUS_FIX
    assert missing.covariance_type == COVARIANCE_TYPE_APPROXIMATED
    assert missing.covariance[0] > 1.0
    assert missing.covariance[8] > 1.0


def test_heading_frame_and_yaw_sign():
    # 机头朝北：NED 单位四元数。ROS 航向 0 朝东，朝北是 +90°。
    north = frd_ned_quat_to_flu_enu((1.0, 0.0, 0.0, 0.0))
    assert north is not None
    assert abs(_wrap(yaw_from_ros_quat(north) - math.pi / 2.0)) < 1e-6

    # NED 正航向把机头从北转向东，是从上往下看的顺时针。ROS 航向因此变小。
    clockwise = frd_ned_quat_to_flu_enu(_ned_yaw(0.4))
    assert clockwise is not None
    assert abs(_wrap(yaw_from_ros_quat(clockwise) - (math.pi / 2.0 - 0.4))) < 1e-6

    east = frd_ned_quat_to_flu_enu(_ned_yaw(math.pi / 2.0))
    assert east is not None
    assert abs(_wrap(yaw_from_ros_quat(east))) < 1e-6

    assert frd_ned_quat_to_flu_enu((math.nan, 0.0, 0.0, 0.0)) is None
    assert enu_orientation_from_odometry(POSE_FRAME_NED, (1.0, 0.0, 0.0, 0.0)) is not None
    assert enu_orientation_from_odometry(2, (1.0, 0.0, 0.0, 0.0)) is None
    assert enu_orientation_from_odometry(0, (1.0, 0.0, 0.0, 0.0)) is None


def test_diagnostics_report_fix_satellites_and_hdop():
    stale = gps_diagnostic(False, None, 2.0)
    assert stale.level == DIAG_STALE
    assert dict(stale.values)['satellites'] == '0'

    no_fix = gps_diagnostic(True, 0.2, 2.0, fix_type=FIX_TYPE_NONE, satellites=4, hdop=3.5, eph=9.0, epv=12.0)
    assert no_fix.level == DIAG_WARN
    values = dict(no_fix.values)
    assert values['fix'] == 'none'
    assert values['satellites'] == '4'
    assert values['hdop'] == '3.50'

    good = gps_diagnostic(True, 0.2, 2.0, fix_type=FIX_TYPE_3D, satellites=12, hdop=0.8)
    assert good.level == DIAG_OK
    assert dict(good.values)['fix'] == '3D'
    assert dict(good.values)['hdop'] == '0.80'

    lost = gps_diagnostic(True, 5.0, 2.0, fix_type=FIX_TYPE_3D, satellites=12, hdop=0.8)
    assert lost.level == DIAG_STALE


def test_dds_topics_publish_raw_gps_at_a_modest_rate():
    path = Path(__file__).resolve().parents[1] / 'config' / 'dds_topics.yaml'
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    pubs = data['publications']
    topics = {item['topic']: item for item in pubs}
    gps = topics['/fmu/out/vehicle_gps_position']
    assert gps['type'] == 'px4_msgs::msg::SensorGps'
    assert float(gps['rate_limit']) == 5.0
    assert '/fmu/out/sensor_combined' not in topics
    assert '/fmu/out/vehicle_global_position' not in topics
    assert '/fmu/out/vehicle_odometry' in topics
    assert '/fmu/in/vehicle_command' in {item['topic'] for item in data['subscriptions']}


def test_state_node_uses_ellipsoid_altitude():
    text = (Path(__file__).resolve().parents[1] / 'diffdrive_rover' / 'px4_state_node.py').read_text(encoding='utf-8')
    assert 'altitude_ellipsoid_m' in text
    assert 'altitude_msl_m' not in text
    assert "frame_id = 'gps_link'" in text or "gps_frame" in text
