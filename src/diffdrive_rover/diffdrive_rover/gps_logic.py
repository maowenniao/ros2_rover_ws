"""PX4 GPS 与航向换算。不依赖 rclpy，方便单测。

坐标系：
- 经纬高是 WGS84。纬度北正南负，经度东正西负，高度用椭球高、向上为正。
- 飞控姿态是机体系 FRD 到地面 NED 的四元数，w 在前。ROS 要的是机体系 FLU 到 ENU，
  航向 0 朝东，逆时针为正。NED 里正航向是从上往下看顺时针，转到 ROS 后要变号。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

# 与 sensor_msgs/NavSatStatus、NavSatFix 的常量一致。
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2
SERVICE_GPS = 1
COVARIANCE_TYPE_UNKNOWN = 0
COVARIANCE_TYPE_APPROXIMATED = 1
COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

# 与 diagnostic_msgs/DiagnosticStatus 的 level 一致。
DIAG_OK = 0
DIAG_WARN = 1
DIAG_ERROR = 2
DIAG_STALE = 3

# 与 px4_msgs/SensorGps 的 fix_type 一致。0 和 1 都是无定位。
FIX_TYPE_NONE = 1
FIX_TYPE_2D = 2
FIX_TYPE_3D = 3
FIX_TYPE_RTCM = 4
FIX_TYPE_RTK_FLOAT = 5
FIX_TYPE_RTK_FIXED = 6
FIX_TYPE_EXTRAPOLATED = 8

# 与 px4_msgs/VehicleOdometry 的 pose_frame 一致。
POSE_FRAME_NED = 1

# eph/epv 无效时用 10 m 的近似标准差，避免协方差 0 被当成完美定位。
FALLBACK_VARIANCE_M2 = 100.0

_PUBLISHABLE = {
    FIX_TYPE_2D,
    FIX_TYPE_3D,
    FIX_TYPE_RTCM,
    FIX_TYPE_RTK_FLOAT,
    FIX_TYPE_RTK_FIXED,
}

# 180° about the body X axis: FRD -> FLU.
_Q_FRD_TO_FLU = (0.0, 1.0, 0.0, 0.0)
# 180° about the north-east axis: NED -> ENU.
_Q_NED_TO_ENU = (0.0, math.sqrt(0.5), math.sqrt(0.5), 0.0)

RosQuat = Tuple[float, float, float, float]
Wxyz = Tuple[float, float, float, float]


def has_gnss_fix(fix_type: int) -> bool:
    """2D/3D/差分/RTK 才算有定位。外推和无定位都不发。"""
    return int(fix_type) in _PUBLISHABLE


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _positive(value: float) -> bool:
    return _finite(value) and float(value) > 0.0


def _diag(xx: float, yy: float, zz: float) -> Tuple[float, ...]:
    return (
        float(xx), 0.0, 0.0,
        0.0, float(yy), 0.0,
        0.0, 0.0, float(zz),
    )


def position_covariance(eph: float, epv: float) -> Tuple[Tuple[float, ...], int]:
    """eph、epv 是米，按 1σ 精度。协方差用它们的平方，东、北各用 eph，天向用 epv。"""
    if _positive(eph) and _positive(epv):
        horizontal = float(eph) * float(eph)
        vertical = float(epv) * float(epv)
        return _diag(horizontal, horizontal, vertical), COVARIANCE_TYPE_DIAGONAL_KNOWN
    return _diag(FALLBACK_VARIANCE_M2, FALLBACK_VARIANCE_M2, FALLBACK_VARIANCE_M2), COVARIANCE_TYPE_APPROXIMATED


def navsat_status(fix_type: int) -> int:
    if int(fix_type) == FIX_TYPE_RTCM:
        return STATUS_SBAS_FIX
    if int(fix_type) in (FIX_TYPE_RTK_FLOAT, FIX_TYPE_RTK_FIXED):
        return STATUS_GBAS_FIX
    return STATUS_FIX


@dataclass(frozen=True)
class NavSatFixFields:
    latitude: float
    longitude: float
    altitude: float
    status: int
    service: int
    covariance: Tuple[float, ...]
    covariance_type: int
    frame_id: str


def sensor_gps_to_navsat(
    latitude_deg: float,
    longitude_deg: float,
    altitude_ellipsoid_m: float,
    fix_type: int,
    eph: float,
    epv: float,
    frame_id: str = 'gps_link',
) -> Optional[NavSatFixFields]:
    """无定位时返回 None。高度只用椭球高，正方向朝上，不取平均海平面，也不取反。"""
    if not has_gnss_fix(fix_type):
        return None
    if not _finite(latitude_deg) or not _finite(longitude_deg) or not _finite(altitude_ellipsoid_m):
        return None
    latitude = float(latitude_deg)
    longitude = float(longitude_deg)
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        return None
    covariance, covariance_type = position_covariance(eph, epv)
    return NavSatFixFields(
        latitude=latitude,
        longitude=longitude,
        altitude=float(altitude_ellipsoid_m),
        status=navsat_status(fix_type),
        service=SERVICE_GPS,
        covariance=covariance,
        covariance_type=covariance_type,
        frame_id=frame_id or 'gps_link',
    )


def _fix_name(fix_type: int) -> str:
    return {
        0: 'none',
        FIX_TYPE_NONE: 'none',
        FIX_TYPE_2D: '2D',
        FIX_TYPE_3D: '3D',
        FIX_TYPE_RTCM: 'DGPS',
        FIX_TYPE_RTK_FLOAT: 'RTK float',
        FIX_TYPE_RTK_FIXED: 'RTK fixed',
        FIX_TYPE_EXTRAPOLATED: 'extrapolated',
    }.get(int(fix_type), str(int(fix_type)))


def _fmt(value: float, digits: int) -> str:
    if not _finite(value):
        return 'nan'
    return f'{float(value):.{digits}f}'


@dataclass(frozen=True)
class GpsDiagnostic:
    level: int
    message: str
    values: Tuple[Tuple[str, str], ...]


def gps_diagnostic(
    ever_received: bool,
    age_sec: Optional[float],
    stale_sec: float,
    fix_type: int = 0,
    satellites: int = 0,
    hdop: float = math.nan,
    eph: float = math.nan,
    epv: float = math.nan,
) -> GpsDiagnostic:
    """定位质量、卫星数和 HDOP。从没收到或中断时标成过期，不把它当成急停故障。"""
    quality = _fix_name(fix_type)
    values = (
        ('fix', quality),
        ('satellites', str(int(satellites))),
        ('hdop', _fmt(hdop, 2)),
        ('eph', _fmt(eph, 2)),
        ('epv', _fmt(epv, 2)),
    )
    if not ever_received:
        return GpsDiagnostic(DIAG_STALE, '没有 GPS 数据', values)
    if age_sec is None or age_sec > stale_sec:
        return GpsDiagnostic(DIAG_STALE, 'GPS 数据中断', values)
    if not has_gnss_fix(fix_type):
        return GpsDiagnostic(DIAG_WARN, '无定位', values)
    if int(fix_type) == FIX_TYPE_2D:
        return GpsDiagnostic(DIAG_WARN, '2D 定位', values)
    if int(fix_type) == FIX_TYPE_RTK_FIXED:
        return GpsDiagnostic(DIAG_OK, 'RTK 固定', values)
    if int(fix_type) == FIX_TYPE_RTK_FLOAT:
        return GpsDiagnostic(DIAG_OK, 'RTK 浮点', values)
    if int(fix_type) == FIX_TYPE_RTCM:
        return GpsDiagnostic(DIAG_OK, '差分定位', values)
    return GpsDiagnostic(DIAG_OK, '3D 定位', values)


def _quat_mul(left: Wxyz, right: Wxyz) -> Wxyz:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _normalize(quat: Wxyz) -> Optional[Wxyz]:
    w, x, y, z = quat
    if not all(math.isfinite(v) for v in (w, x, y, z)):
        return None
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-6:
        return None
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    if w < 0.0:
        w, x, y, z = -w, -x, -y, -z
    return (w, x, y, z)


def frd_ned_quat_to_flu_enu(q_wxyz: Sequence[float]) -> Optional[RosQuat]:
    """FRD→NED 的 wxyz 四元数，转成 ROS 的 xyzw（FLU→ENU）。无效时返回 None。"""
    if len(q_wxyz) < 4:
        return None
    ned = _normalize((float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])))
    if ned is None:
        return None
    enu = _normalize(_quat_mul(_Q_NED_TO_ENU, _quat_mul(ned, _Q_FRD_TO_FLU)))
    if enu is None:
        return None
    w, x, y, z = enu
    return (x, y, z, w)


def enu_orientation_from_odometry(pose_frame: int, q_wxyz: Sequence[float]) -> Optional[RosQuat]:
    """只有 NED 参考系的姿态才能当成东西北的航向。FRD 世界系的航向没有北向。"""
    if int(pose_frame) != POSE_FRAME_NED:
        return None
    return frd_ned_quat_to_flu_enu(q_wxyz)


def orientation_covariance(valid: bool, variance: Optional[Sequence[float]] = None) -> Tuple[float, ...]:
    """无效姿态把第一项设成 -1，融合节点会丢掉它。"""
    if not valid:
        return (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    rx, ry, rz = 0.05, 0.05, 0.05
    if variance is not None and len(variance) >= 3:
        vals = [float(v) for v in list(variance)[:3]]
        if all(_positive(v) for v in vals):
            rx, ry, rz = vals
    return _diag(rx, ry, rz)


def yaw_from_ros_quat(quat_xyzw: Sequence[float]) -> float:
    x, y, z, w = (float(quat_xyzw[0]), float(quat_xyzw[1]), float(quat_xyzw[2]), float(quat_xyzw[3]))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
