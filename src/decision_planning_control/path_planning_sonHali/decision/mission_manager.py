"""Waypoint tabanlı görev takibi.

GEOJSON'dan gelen sıralı waypoint listesini yönetir: hangi noktadayız, hedefe
varıldı mı, sıradakine geç. HFSM bu sınıfı sorgulayarak PARKING / PASSENGER_OPS
gibi görev-tetikli durumlara karar verir.

Tur semantiği (yolcu alma/bırakma) waypoint ``mission_type`` üzerinden okunur;
genel ``GOAL`` noktaları yalnızca geçiş noktasıdır (özel servis gerektirmez).
"""

from __future__ import annotations

from typing import List, Optional

from ..common.enums import MissionType
from ..common.types import Pose2D, Waypoint


class MissionManager:
    """Sıralı waypoint ilerlemesini izleyen basit görev yöneticisi."""

    def __init__(self, waypoints: List[Waypoint]):
        # START noktası bir hedef değildir; varsa atlanır.
        self._waypoints = [w for w in waypoints if w.mission_type is not MissionType.START]
        self._index = 0

    # --- Sorgular ---
    @property
    def is_finished(self) -> bool:
        """Tüm waypointler tamamlandı mı?"""
        return self._index >= len(self._waypoints)

    def current(self) -> Optional[Waypoint]:
        """Aktif hedef waypoint; görev bittiyse None."""
        if self.is_finished:
            return None
        return self._waypoints[self._index]

    def distance_from(self, pose: Pose2D) -> float:
        """Araçtan aktif hedefe Öklid mesafesi (m); hedef yoksa sonsuz."""
        wp = self.current()
        if wp is None:
            return float("inf")
        return pose.distance_to(wp.pose)

    def arrived(self, pose: Pose2D) -> bool:
        """Araç aktif hedefin tolerans yarıçapına girdi mi?"""
        wp = self.current()
        if wp is None:
            return False
        return self.distance_from(pose) <= wp.tolerance

    def current_type(self) -> Optional[MissionType]:
        """Aktif hedefin görev tipi; hedef yoksa None."""
        wp = self.current()
        return wp.mission_type if wp else None

    # --- İlerleme ---
    def complete_current(self) -> None:
        """Aktif waypoint'i tamamlandı işaretle ve sıradakine geç."""
        wp = self.current()
        if wp is not None:
            wp.completed = True
            self._index += 1

    @property
    def progress(self) -> str:
        """İnsan-okur ilerleme bilgisi (log/teşhis)."""
        return f"{self._index}/{len(self._waypoints)}"
