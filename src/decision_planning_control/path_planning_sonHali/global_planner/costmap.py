"""Occupancy costmap — Hybrid A* için sürülebilir alan haritası.

Yerel ENU düzleminde sabit çözünürlüklü bir ızgara tutar. İki kaynaktan beslenir:
  * Şerit geometrisi (GEOJSON/algı): koridor dışı hücreler kapatılır (şerit ihlali
    cezası nedeniyle araç şeritte kalmalı).
  * Canlı engeller (algı): araç ayak izi kadar şişirilerek (inflation) işaretlenir.

Çarpışma modeli (Tur C): engeller aracın çevresel yarıçapı kadar şişirildiğinden,
plan sırasında aracı bir nokta gibi kontrol etmek yeterlidir (temkinli/disk modeli).
TODO(ileride): iki-daire (ön/arka) modeliyle daha az temkinli kontrol.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

from ..common.types import Obstacle, Pose2D
from ..common.vehicle_params import BEE1, VehicleParams

FREE = np.uint8(0)
OCCUPIED = np.uint8(1)


class Costmap:
    """Yerel ENU düzleminde 2B doluluk ızgarası."""

    def __init__(
        self,
        resolution: float,
        width_m: float,
        height_m: float,
        origin: Tuple[float, float] = (0.0, 0.0),
        vehicle: VehicleParams = BEE1,
    ):
        """
        Args:
            resolution: hücre boyutu (m).
            width_m, height_m: harita boyutları (m).
            origin: ızgaranın (0,0) hücresinin dünya koordinatı (sol-alt köşe).
            vehicle: şişirme yarıçapı için araç parametreleri.
        """
        self.resolution = resolution
        self.origin = origin
        self.vehicle = vehicle
        self.cols = int(math.ceil(width_m / resolution))
        self.rows = int(math.ceil(height_m / resolution))
        # grid[row(y), col(x)] = FREE/OCCUPIED
        self.grid = np.zeros((self.rows, self.cols), dtype=np.uint8)
        # Mesafe alanı (Tur E): compute_distance_field() ile doldurulur.
        self._dist: Optional[np.ndarray] = None   # her hücrenin en yakın OCCUPIED'a uzaklığı (m)
        self._grad_x: Optional[np.ndarray] = None  # mesafe alanının x gradyanı (boşluğa doğru)
        self._grad_y: Optional[np.ndarray] = None

    # ------------------------------------------------------------------ #
    # Koordinat dönüşümleri
    # ------------------------------------------------------------------ #
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Dünya (m) -> ızgara (col, row) indeksi."""
        i = int((x - self.origin[0]) / self.resolution)
        j = int((y - self.origin[1]) / self.resolution)
        return i, j

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        """Izgara hücre merkezi -> dünya (m)."""
        x = self.origin[0] + (i + 0.5) * self.resolution
        y = self.origin[1] + (j + 0.5) * self.resolution
        return x, y

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.cols and 0 <= j < self.rows

    def is_free(self, x: float, y: float) -> bool:
        """Dünya konumu harita içinde ve boş mu?"""
        i, j = self.world_to_grid(x, y)
        if not self.in_bounds(i, j):
            return False
        return self.grid[j, i] == FREE

    # ------------------------------------------------------------------ #
    # Harita doldurma
    # ------------------------------------------------------------------ #
    def add_obstacle(self, obs: Obstacle, extra_margin: float = 0.0) -> None:
        """Bir engeli GERÇEK boyutuyla işaretler (araç boyutu iki-daire kontrolünde).

        İki-daire çarpışma modeline geçişle birlikte engeller artık araç ayak iziyle
        şişirilmez; araç gövdesi ``is_footprint_free`` içinde dikkate alınır. ``extra_margin``
        yalnızca ek bir güvenlik tamponu istenirse kullanılır.
        """
        cx, cy, r = _center_radius(obs)
        self._mark_disc(cx, cy, r + extra_margin)

    def add_obstacles(self, obstacles: Sequence[Obstacle], extra_margin: float = 0.0) -> None:
        for obs in obstacles:
            self.add_obstacle(obs, extra_margin)

    def restrict_to_corridor(
        self,
        centerline: Sequence[Tuple[float, float]],
        half_width: float,
    ) -> None:
        """Şerit merkez çizgisinden ``half_width``'ten uzak hücreleri kapatır.

        Böylece Hybrid A* aracı şeridin içinde tutar (şerit ihlali önlenir).
        Merkez çizgi en az iki nokta içermelidir.
        """
        if len(centerline) < 2:
            return
        # Her hücre merkezinin polyline'a uzaklığını ölç, uzaksa kapat.
        for j in range(self.rows):
            for i in range(self.cols):
                x, y = self.grid_to_world(i, j)
                if _dist_to_polyline(x, y, centerline) > half_width:
                    self.grid[j, i] = OCCUPIED

    # ------------------------------------------------------------------ #
    # Mesafe alanı (Tur E — TEB sınır kuvveti için)
    # ------------------------------------------------------------------ #
    def compute_distance_field(self) -> None:
        """Her boş hücrenin en yakın OCCUPIED hücreye uzaklığını ve gradyanını üretir.

        ``distance_transform_edt`` boş (FREE) maskesi üzerinde çalışır: sonuç, en
        yakın dolu hücreye olan Öklid uzaklığıdır (metre). Gradyan boşluğa (engelden
        uzağa) doğru yönü verir; TEB band noktalarını bu yönde iter.

        Costmap her güncellendiğinde (engel/şerit değişince) yeniden çağrılmalıdır.
        """
        free_mask = self.grid == FREE
        self._dist = distance_transform_edt(free_mask) * self.resolution
        # np.gradient: [d/d_row (y ekseni), d/d_col (x ekseni)]
        gy, gx = np.gradient(self._dist)
        self._grad_x = gx / self.resolution
        self._grad_y = gy / self.resolution

    def is_footprint_free(self, pose: Pose2D, vehicle: Optional[VehicleParams] = None) -> bool:
        """Araç ayak izi (iki-daire) verilen pozda çarpışmasız mı? (yön-duyarlı).

        Her daire merkezi araç kafa açısına göre döndürülür; mesafe alanı (EDT) ile
        ``distance_at(merkez) >= yarıçap`` koşulu kontrol edilir. Mesafe alanı
        hesaplanmamışsa tembel olarak üretilir. Harita dışına taşan daire = çarpışma.
        """
        if self._dist is None:
            self.compute_distance_field()
        veh = vehicle or self.vehicle
        cos_t, sin_t = math.cos(pose.yaw), math.sin(pose.yaw)
        for offset, radius in veh.footprint_circles():
            cx = pose.x + offset * cos_t
            cy = pose.y + offset * sin_t
            i, j = self.world_to_grid(cx, cy)
            if not self.in_bounds(i, j):
                return False
            if self._dist[j, i] < radius:
                return False
        return True

    def distance_at(self, x: float, y: float) -> float:
        """Dünya konumunun en yakın engele uzaklığı (m). Alan yoksa/dışıysa 0."""
        if self._dist is None:
            return 0.0
        i, j = self.world_to_grid(x, y)
        if not self.in_bounds(i, j):
            return 0.0
        return float(self._dist[j, i])

    def gradient_at(self, x: float, y: float) -> Tuple[float, float]:
        """Mesafe alanının gradyanı (boşluğa doğru birim-benzeri vektör). Yoksa (0,0)."""
        if self._grad_x is None:
            return 0.0, 0.0
        i, j = self.world_to_grid(x, y)
        if not self.in_bounds(i, j):
            return 0.0, 0.0
        return float(self._grad_x[j, i]), float(self._grad_y[j, i])

    def _mark_disc(self, cx: float, cy: float, radius: float) -> None:
        """Merkez (cx,cy) yarıçap ``radius`` daireyi OCCUPIED yapar."""
        i0, j0 = self.world_to_grid(cx - radius, cy - radius)
        i1, j1 = self.world_to_grid(cx + radius, cy + radius)
        r2 = radius * radius
        for j in range(max(0, j0), min(self.rows, j1 + 1)):
            for i in range(max(0, i0), min(self.cols, i1 + 1)):
                x, y = self.grid_to_world(i, j)
                if (x - cx) ** 2 + (y - cy) ** 2 <= r2:
                    self.grid[j, i] = OCCUPIED


# ---------------------------------------------------------------------- #
# Yardımcılar
# ---------------------------------------------------------------------- #
def _center_radius(obs: Obstacle) -> Tuple[float, float, float]:
    """Engeli (merkez_x, merkez_y, yarıçap) dairesine indirger."""
    if obs.center is not None:
        return obs.center[0], obs.center[1], max(obs.radius, 0.0)
    if obs.polygon:
        cx = sum(p[0] for p in obs.polygon) / len(obs.polygon)
        cy = sum(p[1] for p in obs.polygon) / len(obs.polygon)
        r = max(math.hypot(p[0] - cx, p[1] - cy) for p in obs.polygon)
        return cx, cy, r
    return 0.0, 0.0, 0.0


def _dist_to_polyline(px: float, py: float, pts: Sequence[Tuple[float, float]]) -> float:
    """Bir noktanın bir polyline'a en kısa uzaklığı (m)."""
    best = float("inf")
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        best = min(best, _dist_to_segment(px, py, ax, ay, bx, by))
    return best


def _dist_to_segment(px, py, ax, ay, bx, by) -> float:
    """Nokta-doğru parçası uzaklığı."""
    dx, dy = bx - ax, by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
    proj_x, proj_y = ax + t * dx, ay + t * dy
    return math.hypot(px - proj_x, py - proj_y)
