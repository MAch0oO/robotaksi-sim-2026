"""Geometri yardımcıları: koordinat dönüşümü, açı işlemleri, Ackermann adımı.

Bu modül dışa bağımlılık içermez (saf Python + math). Planlayıcılar ve HFSM
mesafe/açı hesaplarını ve GPS->yerel dönüşümünü buradan çağırır.

Frame notu:
    GPS (WGS84, derece) --enu_from_gps--> yerel ENU (metre, doğu=x, kuzey=y)
    Yerel ENU bizim planlama düzlemimizdir; tüm planlayıcılar burada çalışır.
"""

from __future__ import annotations

import math
from typing import Tuple

# WGS84 referans elipsoidi
_EARTH_RADIUS = 6_378_137.0  # ekvator yarıçapı (m)


def normalize_angle(angle: float) -> float:
    """Bir açıyı (-pi, pi] aralığına indirger.

    Açı farkı / kafa açısı hatası hesaplarında tutarlılık için zorunludur.
    """
    return math.atan2(math.sin(angle), math.cos(angle))


def angle_diff(target: float, source: float) -> float:
    """``target - source`` farkını (-pi, pi] aralığında döndürür."""
    return normalize_angle(target - source)


def euclidean(ax: float, ay: float, bx: float, by: float) -> float:
    """İki nokta arası Öklid mesafesi (m)."""
    return math.hypot(bx - ax, by - ay)


def enu_from_gps(
    lat: float,
    lon: float,
    ref_lat: float,
    ref_lon: float,
) -> Tuple[float, float]:
    """GPS (derece) koordinatını referans noktaya göre yerel ENU'ya çevirir.

    Yarışma pisti küçük olduğundan (birkaç yüz metre) düz-dünya (equirectangular)
    yaklaşımı yeterli doğruluktadır ve UTM kütüphanesi bağımlılığı gerektirmez.

    Args:
        lat, lon: dönüştürülecek nokta (derece).
        ref_lat, ref_lon: yerel başlangıç (origin) noktası (genelde GEOJSON 'start').

    Returns:
        (x_east, y_north) metre cinsinden, referans noktası (0, 0) kabul edilir.
    """
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    mean_lat = math.radians((lat + ref_lat) * 0.5)
    x_east = d_lon * math.cos(mean_lat) * _EARTH_RADIUS
    y_north = d_lat * _EARTH_RADIUS
    return x_east, y_north


def ackermann_step(
    x: float,
    y: float,
    yaw: float,
    steer: float,
    distance: float,
    wheelbase: float,
) -> Tuple[float, float, float]:
    """Tek-iz (bicycle) Ackermann modeliyle aracı bir adım ileri taşır.

    Hybrid A* düğüm genişletmesinde sürülebilir (kinematik olarak geçerli)
    komşu pozları üretmek için kullanılır.

    Args:
        x, y, yaw: mevcut poz (m, m, rad). Referans: arka aks (base_link).
        steer: direksiyon (ön teker) açısı (rad). Pozitif = sola.
        distance: bu adımda kat edilen yay uzunluğu (m). Negatif = geri.
        wheelbase: dingil mesafesi (m).

    Returns:
        (x_new, y_new, yaw_new) — yeni poz.
    """
    if abs(steer) < 1e-6:
        # Düz hareket — sayısal kararlılık için ayrı ele alınır.
        x_new = x + distance * math.cos(yaw)
        y_new = y + distance * math.sin(yaw)
        return x_new, y_new, yaw
    # Eğrisel hareket: dönüş yarıçapı R = L / tan(steer)
    turning_radius = wheelbase / math.tan(steer)
    yaw_new = normalize_angle(yaw + distance / turning_radius)
    x_new = x + turning_radius * (math.sin(yaw_new) - math.sin(yaw))
    y_new = y - turning_radius * (math.cos(yaw_new) - math.cos(yaw))
    return x_new, y_new, yaw_new
