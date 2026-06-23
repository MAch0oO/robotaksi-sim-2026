"""ROS mesajı ⇄ çekirdek dataclass dönüşümleri (rclpy'siz, test edilebilir).

Bu modül bilinçli olarak ``rclpy``'den ve somut mesaj sınıflarından bağımsızdır:
fonksiyonlar mesaj nesnelerinin **alanları** üzerinde çalışır (duck-typing). Böylece
ROS kurulu olmayan bir ortamda bile sahte (SimpleNamespace) nesnelerle birim test edilebilir.

Düğüm (``planning_node``) bu fonksiyonları çağırır; mesaj nesnesi oluşturma/yayınlama
sorumluluğu düğümde kalır.

Arayüz Sözleşmesi notları:
  * Tüm geometrik veriler ``map`` frame'inde varsayılır.
  * emergency = (/safety_state == 2) OR (FB_OMUX_to_AUTONOMOUS.FB_EMERGENCY == 1).
  * Çıkış yörüngesi MultiDOFJointTrajectory'ye eşlenir; geri vites işaretli hızla belirtilir.
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Sequence, Tuple

from ..common.enums import LightState, ObstacleClass, SignType
from ..common.types import (
    Constraints,
    LaneInfo,
    Obstacle,
    Pose2D,
    TrafficLight,
    TrafficSign,
    Trajectory,
    VehicleState,
)

# Trafik ışığı tamsayı kodları (sözleşme: /perception/traffic_light_state, std_msgs/Int8)
_INT_TO_LIGHT = {0: LightState.NONE, 1: LightState.RED, 2: LightState.YELLOW, 3: LightState.GREEN}

# Güvenlik durumu kodları (sözleşme: /safety_state, std_msgs/Int8)
SAFETY_GO = 0
SAFETY_SLOW = 1
SAFETY_EMERGENCY = 2

# Vites istekleri (sözleşme: /planning/gear_request, std_msgs/Int8) — araç RC_SelectionGear
GEAR_NEUTRAL = 0
GEAR_DRIVE = 1
GEAR_REVERSE = 2


# ---------------------------------------------------------------------- #
# Quaternion <-> yaw (düzlemsel)
# ---------------------------------------------------------------------- #
def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion'dan düzlemsel kafa açısını (yaw, rad) çıkarır."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Düzlemsel yaw'dan (qx, qy, qz, qw) quaternion üretir."""
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


# ---------------------------------------------------------------------- #
# Girdi dönüşümleri (ROS -> çekirdek)
# ---------------------------------------------------------------------- #
def pose2d_from_pose(pose) -> Pose2D:
    """geometry_msgs/Pose -> Pose2D (position + orientation→yaw)."""
    p, o = pose.position, pose.orientation
    return Pose2D(p.x, p.y, yaw_from_quaternion(o.x, o.y, o.z, o.w))


def vehicle_state_from_odom(odom) -> VehicleState:
    """nav_msgs/Odometry -> VehicleState.

    pose.pose → Pose2D; twist.twist.linear.x → v; twist.twist.angular.z → ω.
    """
    pose = pose2d_from_pose(odom.pose.pose)
    tw = odom.twist.twist
    stamp = _stamp_to_sec(getattr(odom.header, "stamp", None))
    frame = getattr(odom.header, "frame_id", "map")
    return VehicleState(pose=pose, v=tw.linear.x, omega=tw.angular.z, frame=frame, stamp=stamp)


def obstacles_from_object_array(msg, dynamic_speed_thr: float = 0.3) -> List[Obstacle]:
    """derived_object_msgs/ObjectArray -> List[Obstacle].

    Her Object için: pose.position → merkez; polygon/shape → yarıçap; twist.linear → hız;
    hız eşiği üstündeyse DYNAMIC, değilse STATIC.
    """
    out: List[Obstacle] = []
    for obj in getattr(msg, "objects", []):
        cx, cy = obj.pose.position.x, obj.pose.position.y
        vx = obj.twist.linear.x if hasattr(obj, "twist") else 0.0
        vy = obj.twist.linear.y if hasattr(obj, "twist") else 0.0
        polygon = _polygon_points(obj)
        radius = _object_radius(obj, cx, cy, polygon)
        is_dyn = math.hypot(vx, vy) > dynamic_speed_thr
        out.append(Obstacle(
            id=int(getattr(obj, "id", len(out))),
            obstacle_class=ObstacleClass.DYNAMIC if is_dyn else ObstacleClass.STATIC,
            center=(cx, cy),
            radius=radius,
            polygon=polygon,
            velocity=(vx, vy),
        ))
    return out


def lane_from_path(path_msg, half_width: float) -> Optional[LaneInfo]:
    """nav_msgs/Path + yarı-genişlik -> LaneInfo. Boş path → None."""
    pts = [(ps.pose.position.x, ps.pose.position.y) for ps in getattr(path_msg, "poses", [])]
    if len(pts) < 2:
        return None
    return LaneInfo(centerline=pts, left_offset=half_width, right_offset=half_width)


def traffic_light_from_state(value: int, position: Optional[Tuple[float, float]] = None) -> TrafficLight:
    """std_msgs/Int8 durum kodu (+ opsiyonel konum) -> TrafficLight."""
    return TrafficLight(state=_INT_TO_LIGHT.get(int(value), LightState.UNKNOWN), position=position)


# Algı ekibi levha etiketi -> SignType (referans karar düğümü etiketleriyle uyumlu)
_SIGN_MAP = {
    "LEVHA_YAYA_GECIDI": SignType.PEDESTRIAN_CROSSING,
    "LEVHA_GIRILMEZ": SignType.NO_ENTRY,
    "LEVHA_SAGA_DONULMEZ": SignType.NO_RIGHT_TURN,
    "LEVHA_SOLA_DONULMEZ": SignType.NO_LEFT_TURN,
    "LEVHA_MECBURI_SAG": SignType.MANDATORY_RIGHT,
    "LEVHA_MECBURI_SOL": SignType.MANDATORY_LEFT,
    "LEVHA_MECBURI_ILERI": SignType.MANDATORY_AHEAD,
    "LEVHA_MECBURI_ILERIDEN_SAG": SignType.AHEAD_RIGHT,
    "LEVHA_MECBURI_ILERIDEN_SOL": SignType.AHEAD_LEFT,
    "LEVHA_SAGDAN_GIDINIZ": SignType.KEEP_RIGHT,
    "LEVHA_SOLDAN_GIDINIZ": SignType.KEEP_LEFT,
    "LEVHA_PARK_YASAKTIR": SignType.NO_PARKING,
    "LEVHA_IKI_YONLU_YOL": SignType.TWO_WAY,
    "LEVHA_TUNEL": SignType.TUNNEL,
    "LEVHA_SERIT_DUZENLEME_SOLDAN_SAGA": SignType.LANE_MERGE_RIGHT,
    "LEVHA_SERIT_DUZENLEME_SAGDAN_SOLA": SignType.LANE_MERGE_LEFT,
}


def traffic_sign_from_label(label: str, distance: float) -> Optional[TrafficSign]:
    """Algı etiketi (ör. 'LEVHA_TUNEL') + mesafe -> TrafficSign. Tanınmazsa None."""
    sign_type = _SIGN_MAP.get((label or "").strip().upper())
    if sign_type is None:
        return None
    return TrafficSign(sign_type=sign_type, distance=distance)


def emergency_from_signals(safety_state: int, fb_emergency: int = 0) -> bool:
    """Acil durum: lidar güvenlik düğümü VEYA araç donanım e-stop geri beslemesi.

    emergency = (/safety_state == 2) OR (FB_OMUX_to_AUTONOMOUS.FB_EMERGENCY == 1)
    """
    return int(safety_state) == SAFETY_EMERGENCY or int(fb_emergency) == 1


def speed_cap_from_safety(safety_state: int, normal_cap: float, slow_cap: float) -> float:
    """safety_state == 1 (Yavaşla) ise hız üst sınırını düşürür."""
    return slow_cap if int(safety_state) == SAFETY_SLOW else normal_cap


# ---------------------------------------------------------------------- #
# Çıktı dönüşümleri (çekirdek -> ROS'a hazır ara form)
# ---------------------------------------------------------------------- #
def trajectory_to_points(traj: Trajectory) -> List[dict]:
    """Çekirdek Trajectory -> MultiDOFJointTrajectory noktaları için ara form.

    Her eleman: {x, y, yaw, v, t}. Düğüm bunları transform + twist + time_from_start'a çevirir.
    """
    return [
        {"x": tp.pose.x, "y": tp.pose.y, "yaw": tp.pose.yaw, "v": tp.v, "t": tp.t}
        for tp in traj.points
    ]


def constraints_to_dict(c: Constraints, state_name: str, target_v: float) -> dict:
    """Kısıtları /planner/constraint JSON'una çevirir (Rota Ekibi okur)."""
    return {
        "durum": state_name,
        "hedef_hiz_ms": target_v,
        "girilmez": c.no_entry,
        "saga_donulmez": c.no_right_turn,
        "sola_donulmez": c.no_left_turn,
        "mecburi_yon": list(c.mandatory_directions),
        "delayed_turn": c.delayed_turn,
        "park_yasak": c.no_parking,
        "iki_yonlu_yol": c.two_way,
        "tunel_aktif": c.tunnel_active,
        "serit_birlesme": c.lane_merge,
        "timestamp": time.time(),
    }


def gear_request_from_trajectory(traj: Trajectory) -> int:
    """Yörünge yön sinyalinden vites isteği üretir (1=Drive, 2=Reverse, 0=Neutral).

    İşaretli hız konvansiyonu: ilk anlamlı hız negatifse geri vites istenir.
    NOT: TEB şu an ileri (pozitif) hız üretir; geri park segmentlerinin işaretli hızla
    işaretlenmesi küçük bir çekirdek iyileştirmesi olarak planlandı (bkz. yol haritası #4).
    Bu fonksiyon o iyileştirme geldiğinde otomatik doğru çalışır; şimdilik Drive döner.
    """
    for p in traj.points:
        if abs(p.v) > 1e-3:
            return GEAR_REVERSE if p.v < 0 else GEAR_DRIVE
    return GEAR_NEUTRAL


# ---------------------------------------------------------------------- #
# Yardımcılar
# ---------------------------------------------------------------------- #
def _polygon_points(obj) -> List[Tuple[float, float]]:
    poly = getattr(obj, "polygon", None)
    if poly is None or not getattr(poly, "points", None):
        return []
    return [(pt.x, pt.y) for pt in poly.points]


def _object_radius(obj, cx: float, cy: float, polygon: Sequence[Tuple[float, float]]) -> float:
    if polygon:
        return max(math.hypot(px - cx, py - cy) for px, py in polygon)
    shape = getattr(obj, "shape", None)
    dims = getattr(shape, "dimensions", None) if shape is not None else None
    if dims:
        return 0.5 * max(dims)
    return 0.3  # bilinmeyen geometri için küçük varsayılan


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9
