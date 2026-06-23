"""GEOJSON görev dosyasını ``Waypoint`` listesine çevirir.

Yarışma günü paylaşılan GEOJSON (start / gorev_N / park_giris noktaları) okunur,
GPS koordinatları ``start`` noktası origin alınarak yerel ENU'ya dönüştürülür.

Önemli: GEOJSON'daki koordinat aracın ön ucunu ifade eder ve istenen kafa açısı
ayrı bir özellik (``heading``/``yon``) olarak gelebilir. Örnek şablonda kafa açısı
yoksa ``yaw=0`` atanır; tur şemasında verilirse okunur.
"""

from __future__ import annotations

import json
from typing import List, Tuple

from ..common.enums import MissionType
from ..common.geometry import enu_from_gps
from ..common.types import Pose2D, Waypoint

# GEOJSON 'name' önekini görev tipine eşler. Sıra önemli: en özgül önce.
_NAME_TO_TYPE = (
    ("start", MissionType.START),
    ("park", MissionType.PARK_ENTRANCE),
    ("indir", MissionType.PASSENGER_DROPOFF),
    ("dropoff", MissionType.PASSENGER_DROPOFF),
    ("bindir", MissionType.PASSENGER_PICKUP),
    ("pickup", MissionType.PASSENGER_PICKUP),
    ("gorev", MissionType.GOAL),  # genel hedef; tur semantiği mission_manager'da netleşir
)


def _classify(name: str) -> MissionType:
    """GEOJSON özellik adından görev tipini çıkarır."""
    lowered = name.lower()
    for prefix, mtype in _NAME_TO_TYPE:
        if prefix in lowered:
            return mtype
    return MissionType.GOAL


def _extract_heading(properties: dict) -> float:
    """Özelliklerden kafa açısını (derece -> rad) çeker; yoksa 0.0."""
    import math

    for key in ("heading", "yon", "yaw", "bearing"):
        if key in properties and properties[key] is not None:
            return math.radians(float(properties[key]))
    return 0.0


def load_waypoints(
    geojson_path: str,
    default_tolerance: float = 1.0,
) -> Tuple[List[Waypoint], Tuple[float, float]]:
    """GEOJSON dosyasını okuyup waypoint listesi + ENU origin döndürür.

    Args:
        geojson_path: GEOJSON dosya yolu.
        default_tolerance: "vardı" kabul yarıçapı (m), özel değer yoksa.

    Returns:
        (waypoints, origin) — waypoints yerel ENU'da sıralı liste;
        origin, dönüşümde kullanılan (ref_lat, ref_lon).

    Raises:
        ValueError: dosyada 'start' noktası yoksa (origin tanımlanamaz).
    """
    with open(geojson_path, "r", encoding="utf-8") as fh:
        data = json.load(fh)

    features = data.get("features", [])

    # 1) Origin = 'start' noktası. Önce onu bulmalıyız.
    ref = None
    for feat in features:
        name = feat.get("properties", {}).get("name", "")
        if _classify(name) is MissionType.START:
            lon, lat = feat["geometry"]["coordinates"][:2]
            ref = (lat, lon)
            break
    if ref is None:
        raise ValueError("GEOJSON içinde 'start' noktası bulunamadı; origin tanımlanamaz.")

    ref_lat, ref_lon = ref

    # 2) Tüm noktaları yerel ENU'ya çevirip waypoint üret.
    waypoints: List[Waypoint] = []
    for idx, feat in enumerate(features):
        if feat.get("geometry", {}).get("type") != "Point":
            continue
        props = feat.get("properties", {})
        name = props.get("name", f"wp_{idx}")
        lon, lat = feat["geometry"]["coordinates"][:2]
        x, y = enu_from_gps(lat, lon, ref_lat, ref_lon)
        pose = Pose2D(x=x, y=y, yaw=_extract_heading(props))
        waypoints.append(
            Waypoint(
                id=idx,
                pose=pose,
                mission_type=_classify(name),
                tolerance=float(props.get("tolerance", default_tolerance)),
            )
        )

    return waypoints, ref
