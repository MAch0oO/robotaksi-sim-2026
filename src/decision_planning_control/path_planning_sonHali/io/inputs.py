"""Planlama hattının her döngüde tükettiği girdi paketi.

Algı + lokalizasyon + görev verisi tek bir ``PlanningInput`` nesnesinde toplanır.
ROS2 sarmalayıcı (ileride) ilgili topic'leri bu nesneye doldurmaktan sorumludur;
çekirdek (HFSM/planlayıcılar) yalnızca bu sözleşmeyi görür.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from ..common.types import (
    LaneInfo,
    Obstacle,
    Pose2D,
    TrafficLight,
    TrafficSign,
    VehicleState,
    Waypoint,
)


@dataclass
class PlanningInput:
    """Tek kontrol döngüsü için tüm dış girdiler.

    Attributes:
        vehicle_state: lokalizasyondan anlık araç durumu (zorunlu).
        obstacles: algıdan o anki engel listesi (statik + dinamik).
        traffic_light: en yakın/ilgili trafik ışığı durumu.
        lane: mevcut şerit geometrisi (None ise şerit bilgisi yok).
        waypoints: göreve ait sıralı waypoint listesi (GEOJSON'dan, sabit).
        park_target: algı ekibinin seçtiği nihai park pozu (slot). PARKING durumunda
            Hybrid A* hedefi bu olur. Henüz gelmediyse None (araç HOLD'da bekler).
            (ROS sarmalayıcısında /selected_slot_pose topic'inden doldurulacak.)
        emergency: donanım/dış kaynaklı acil durdurma bayrağı.
        stamp: bu girdi paketinin zaman damgası (s).
    """

    vehicle_state: VehicleState
    obstacles: List[Obstacle] = field(default_factory=list)
    traffic_light: TrafficLight = field(default_factory=TrafficLight)
    traffic_sign: Optional[TrafficSign] = None
    lane: Optional[LaneInfo] = None
    waypoints: List[Waypoint] = field(default_factory=list)
    park_target: Optional[Pose2D] = None
    emergency: bool = False
    stamp: float = 0.0
