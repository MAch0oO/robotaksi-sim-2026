"""Veri sözleşmeleri (dataclass'lar).

Bu dosya modülün "dili"dir: HFSM, planlayıcılar, testler ve diğer ekiplerle
arayüz hep bu tipleri kullanır. Tipler bilinçli olarak sade ve dış-bağımsızdır
(numpy/ROS yok) ki birim testlerde kolayca üretilebilsin.

Geometrik primitifler burada; bunları paketleyen girdi/çıktı kapsayıcıları
(``PlanningInput`` / ``PlanningOutput``) ``io`` paketindedir.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .enums import LightState, MissionType, ObstacleClass, SignType


@dataclass
class Pose2D:
    """Düzlemsel poz: konum (m) + kafa açısı (rad). Yerel ENU frame'inde."""

    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        """Bu poz ile diğeri arasındaki Öklid mesafesi (m)."""
        return math.hypot(other.x - self.x, other.y - self.y)


@dataclass
class VehicleState:
    """Lokalizasyon ekibinden gelen anlık araç durumu (EKF çıktısı).

    Bizim için salt-okunur girdidir; planlama bu pozu başlangıç olarak alır.
    """

    pose: Pose2D
    v: float = 0.0           # boylamsal hız (m/s)
    omega: float = 0.0       # dönme hızı (rad/s)
    frame: str = "map"       # poz hangi frame'de tanımlı
    stamp: float = 0.0       # zaman damgası (s)


@dataclass
class Obstacle:
    """Algı ekibinden gelen tek bir engel.

    Geometri iki biçimden biriyle verilebilir:
      * ``polygon``: köşe noktaları [(x, y), ...] — bariyer/koni dizilimi.
      * ``center`` + ``radius``: dairesel yaklaşım — hızlı testler / yaya.
    En az biri dolu olmalıdır.
    """

    id: int
    obstacle_class: ObstacleClass
    center: Optional[Tuple[float, float]] = None
    radius: float = 0.0
    polygon: List[Tuple[float, float]] = field(default_factory=list)
    velocity: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) m/s; dinamik engel için

    @property
    def is_dynamic(self) -> bool:
        """Engel hareketli mi (DYNAMIC sınıfı veya hız vektörü sıfırdan farklı)."""
        moving = math.hypot(*self.velocity) > 1e-3
        return moving or self.obstacle_class is ObstacleClass.DYNAMIC


@dataclass
class TrafficLight:
    """Algı ekibinden gelen trafik ışığı bilgisi."""

    state: LightState = LightState.NONE
    position: Optional[Tuple[float, float]] = None  # ışığın/dur çizgisinin konumu


@dataclass
class LaneInfo:
    """Algı ekibinden gelen şerit bilgisi.

    ``centerline`` boyunca araç ortalanmaya çalışır; ofsetler costmap'te
    sürülebilir koridoru sınırlamak için kullanılır.
    """

    centerline: List[Tuple[float, float]] = field(default_factory=list)
    left_offset: float = 0.0    # merkez çizgiden sol şerit kenarına (m)
    right_offset: float = 0.0   # merkez çizgiden sağ şerit kenarına (m)


@dataclass
class TrafficSign:
    """Algı ekibinden gelen trafik levhası (tip + araca uzaklık)."""

    sign_type: SignType = SignType.NONE
    distance: float = float("inf")  # levhaya mesafe (m)


@dataclass
class Constraints:
    """Levha/kural kaynaklı planlama kısıtları (karar katmanı üretir).

    Bir kısmı planlayıcıda fiilen UYGULANIR (no_entry->dur, no_parking->park kilidi,
    hız sınırı); yön/şerit topolojisi gerektirenler (no_*_turn, mandatory, lane_merge,
    tunnel, two_way) bu iskelette **bildirim** olarak taşınır (telemetri + sinyal),
    tam geometrik uygulama şerit topolojisi gelince yapılır. Referans karar düğümünün
    'bildirim seviyesi' felsefesiyle uyumludur.
    """

    no_entry: bool = False
    no_right_turn: bool = False
    no_left_turn: bool = False
    mandatory_directions: List[str] = field(default_factory=list)  # "SAG"/"SOL"/"ILERI"
    delayed_turn: Optional[str] = None       # ileride dönülecek yön ("SAG"/"SOL")
    no_parking: bool = False
    two_way: bool = False
    tunnel_active: bool = False
    lane_merge: Optional[str] = None         # "SAGA" / "SOLA"


@dataclass
class Waypoint:
    """GEOJSON'dan türetilen tek görev noktası."""

    id: int
    pose: Pose2D                  # hedef konum + istenen kafa açısı
    mission_type: MissionType
    tolerance: float = 1.0        # "vardı" kabul edilen yarıçap (m)
    completed: bool = False


@dataclass
class TrajectoryPoint:
    """Zaman-parametreli tek yörünge noktası (TEB çıktısının atomik birimi)."""

    pose: Pose2D
    v: float = 0.0     # bu noktadaki hedef hız (m/s)
    t: float = 0.0     # yörünge başından itibaren geçen süre (s)


@dataclass
class Trajectory:
    """Sıralı yörünge noktaları. TEB üretir, kontrol ekibi takip eder."""

    points: List[TrajectoryPoint] = field(default_factory=list)

    def __len__(self) -> int:
        return len(self.points)

    @property
    def is_empty(self) -> bool:
        return len(self.points) == 0
