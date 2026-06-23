"""Hybrid A* — Ackermann kısıtlı küresel planlayıcı.

Klasik A*'tan farkı: arama düğümleri sürekli (x, y, yaw) pozlarıdır ve komşular
ızgara komşusu değil, aracın gerçekten gidebileceği yay hareketleridir
(``ackermann_step``). Böylece üretilen rota baştan kinematik olarak sürülebilir.

Closed-set için süreklilik (i_hücre, j_hücre, yaw_bin) olarak ayrıklaştırılır.
Çıktı: ``List[Pose2D]`` (başlangıçtan hedefe) veya bulunamazsa None.

Hız profili / zaman parametresi YOKtur — o TEB'in (Tur D) işidir. Burada yalnızca
geometrik, sürülebilir bir rota üretilir.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from ..common.geometry import ackermann_step, angle_diff, normalize_angle
from ..common.types import Pose2D
from ..common.vehicle_params import BEE1, VehicleParams
from .costmap import Costmap


@dataclass(frozen=True)
class HybridAStarConfig:
    """Hybrid A* arama parametreleri."""

    step_size: float = 1.0          # her hareket primitifinin yay uzunluğu (m)
    num_steer: int = 5              # [-max_steer, +max_steer] arası direksiyon örneği
    yaw_bins: int = 72              # closed-set için kafa açısı ayrıklaştırması (5° adım)
    goal_pos_tol: float = 1.0       # hedef konum toleransı (m)
    goal_yaw_tol: float = math.radians(15.0)  # hedef kafa açısı toleransı (rad)
    allow_reverse: bool = False     # geri vites (park için True yapılır)
    reverse_penalty: float = 2.0    # geri hareket maliyet çarpanı
    steer_penalty: float = 0.5      # direksiyon büyüklüğü cezası (düz rotayı teşvik)
    steer_change_penalty: float = 0.3  # direksiyon değişimi cezası (yumuşak rota)
    max_iterations: int = 100_000   # güvenlik üst sınırı


@dataclass(order=True)
class _Node:
    """Öncelik kuyruğu düğümü. ``f`` ile sıralanır."""

    f: float
    g: float = field(compare=False)
    x: float = field(compare=False)
    y: float = field(compare=False)
    yaw: float = field(compare=False)
    steer: float = field(compare=False)
    parent: Optional["_Node"] = field(compare=False, default=None)


class HybridAStar:
    """Ackermann kısıtlı küresel rota planlayıcı."""

    def __init__(
        self,
        costmap: Costmap,
        config: Optional[HybridAStarConfig] = None,
        vehicle: VehicleParams = BEE1,
    ):
        self.costmap = costmap
        self.config = config or HybridAStarConfig()
        self.vehicle = vehicle
        self._steers = self._build_steer_set()
        self.iterations = 0  # son aramadaki düğüm genişletme sayısı (teşhis)

    def _build_steer_set(self) -> List[float]:
        """[-max_steer, +max_steer] aralığını eşit aralıklı örnekler."""
        n = self.config.num_steer
        m = self.vehicle.max_steer_angle
        if n <= 1:
            return [0.0]
        return [(-m + 2 * m * k / (n - 1)) for k in range(n)]

    # ------------------------------------------------------------------ #
    # Ana arama
    # ------------------------------------------------------------------ #
    def plan(self, start: Pose2D, goal: Pose2D) -> Optional[List[Pose2D]]:
        """Başlangıçtan hedefe sürülebilir bir rota arar.

        Returns:
            Pose2D listesi (start -> goal) veya çözüm yoksa None.
        """
        self.iterations = 0
        cfg = self.config

        if not self.costmap.is_footprint_free(start, self.vehicle):
            return None  # başlangıç engelde (araç ayak izi çarpışıyor)

        start_node = _Node(
            f=self._heuristic(start.x, start.y, goal),
            g=0.0, x=start.x, y=start.y, yaw=start.yaw, steer=0.0,
        )
        open_heap: List[_Node] = [start_node]
        # closed-set: ayrıklaştırılmış poz -> ulaşılan en iyi g
        best_g: Dict[Tuple[int, int, int], float] = {}

        directions = [1.0, -1.0] if cfg.allow_reverse else [1.0]

        while open_heap and self.iterations < cfg.max_iterations:
            self.iterations += 1
            node = heapq.heappop(open_heap)

            if self._reached_goal(node, goal):
                return self._reconstruct(node)

            key = self._discretize(node.x, node.y, node.yaw)
            if key in best_g and best_g[key] <= node.g:
                continue  # daha iyisine zaten ulaşıldı
            best_g[key] = node.g

            # Komşuları üret: her direksiyon x her yön
            for steer in self._steers:
                for direction in directions:
                    nx, ny, nyaw = ackermann_step(
                        node.x, node.y, node.yaw, steer,
                        direction * cfg.step_size, self.vehicle.wheelbase,
                    )
                    if not self.costmap.is_footprint_free(Pose2D(nx, ny, nyaw), self.vehicle):
                        continue  # çarpışma (iki-daire, yön-duyarlı)
                    g_new = node.g + self._move_cost(steer, node.steer, direction)
                    nkey = self._discretize(nx, ny, nyaw)
                    if nkey in best_g and best_g[nkey] <= g_new:
                        continue
                    h = self._heuristic(nx, ny, goal)
                    heapq.heappush(open_heap, _Node(
                        f=g_new + h, g=g_new, x=nx, y=ny, yaw=nyaw,
                        steer=steer, parent=node,
                    ))

        return None  # çözüm bulunamadı / iterasyon limiti

    # ------------------------------------------------------------------ #
    # Maliyet, sezgisel, hedef, ayrıklaştırma
    # ------------------------------------------------------------------ #
    def _move_cost(self, steer: float, prev_steer: float, direction: float) -> float:
        """Bir hareket primitifinin maliyeti (mesafe + cezalar)."""
        cfg = self.config
        cost = cfg.step_size
        if direction < 0:
            cost *= cfg.reverse_penalty
        cost += cfg.steer_penalty * abs(steer)
        cost += cfg.steer_change_penalty * abs(steer - prev_steer)
        return cost

    def _heuristic(self, x: float, y: float, goal: Pose2D) -> float:
        """Öklid mesafesi (kabul edilebilir alt-sınır)."""
        return math.hypot(goal.x - x, goal.y - y)

    def _reached_goal(self, node: _Node, goal: Pose2D) -> bool:
        if math.hypot(goal.x - node.x, goal.y - node.y) > self.config.goal_pos_tol:
            return False
        return abs(angle_diff(goal.yaw, node.yaw)) <= self.config.goal_yaw_tol

    def _discretize(self, x: float, y: float, yaw: float) -> Tuple[int, int, int]:
        """Sürekli pozu closed-set anahtarına çevirir."""
        i, j = self.costmap.world_to_grid(x, y)
        yaw_bin = int(normalize_angle(yaw) / (2 * math.pi) * self.config.yaw_bins) % self.config.yaw_bins
        return i, j, yaw_bin

    def _reconstruct(self, node: Optional[_Node]) -> List[Pose2D]:
        """Hedef düğümünden parent zinciriyle rotayı geri kurar."""
        path: List[Pose2D] = []
        while node is not None:
            path.append(Pose2D(node.x, node.y, node.yaw))
            node = node.parent
        path.reverse()
        return path
