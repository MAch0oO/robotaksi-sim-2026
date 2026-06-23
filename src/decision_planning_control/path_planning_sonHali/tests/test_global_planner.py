"""Hybrid A* + Costmap testi.

Ortada bir engel olan haritada başlangıçtan hedefe sürülebilir, çarpışmasız ve
kinematik olarak geçerli (eğrilik sınırına uyan) bir rota üretildiğini doğrular.
``pytest`` ile veya doğrudan ``python ...`` ile çalışır.
"""

from __future__ import annotations

import math

from path_planning.common.enums import ObstacleClass
from path_planning.common.types import Obstacle, Pose2D
from path_planning.common.vehicle_params import BEE1
from path_planning.global_planner.costmap import Costmap
from path_planning.global_planner.hybrid_astar import HybridAStar, HybridAStarConfig


def build_costmap() -> Costmap:
    cm = Costmap(resolution=0.5, width_m=30.0, height_m=30.0, origin=(0.0, 0.0))
    # Doğrudan hedefe giden çizgiyi tıkayan bir engel
    cm.add_obstacle(Obstacle(1, ObstacleClass.STATIC, center=(15.0, 5.0), radius=1.0))
    return cm


def run(verbose: bool = False):
    cm = build_costmap()
    planner = HybridAStar(cm, HybridAStarConfig(step_size=1.0, num_steer=7))
    start = Pose2D(5.0, 5.0, 0.0)
    goal = Pose2D(25.0, 5.0, 0.0)

    path = planner.plan(start, goal)
    assert path is not None, "Rota bulunamadi"

    # 1) Tüm rota pozları çarpışmasız (iki-daire, yön-duyarlı)
    for p in path:
        assert cm.is_footprint_free(p, BEE1), f"Carpisma: ({p.x:.1f},{p.y:.1f})"

    # 2) Hedefe varış toleransta
    last = path[-1]
    assert math.hypot(goal.x - last.x, goal.y - last.y) <= planner.config.goal_pos_tol

    # 3) Kinematik geçerlilik: adım başına eğrilik <= max eğrilik (+ küçük tolerans)
    max_kappa = BEE1.max_curvature()
    for a, b in zip(path, path[1:]):
        ds = math.hypot(b.x - a.x, b.y - a.y)
        if ds < 1e-6:
            continue
        dyaw = abs(math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw)))
        kappa = dyaw / ds
        assert kappa <= max_kappa + 1e-3, f"Egrilik asimi: {kappa:.3f} > {max_kappa:.3f}"

    if verbose:
        print(f"Rota bulundu: {len(path)} nokta, {planner.iterations} dugum genisletildi.")
        print(f"  start=({start.x},{start.y})  goal=({goal.x},{goal.y})")
        max_lat = max(abs(p.y - 5.0) for p in path)
        print(f"  engelden kacmak icin max yanal sapma: {max_lat:.2f} m")
        for p in path[::max(1, len(path)//12)]:
            print(f"    x={p.x:5.2f} y={p.y:5.2f} yaw={math.degrees(p.yaw):6.1f} deg")

    return path


def test_plan_around_obstacle():
    path = run(verbose=False)
    assert len(path) >= 2


def test_two_circle_passes_narrow_gap():
    """İki-daire modeli, disk modelinin 'sığamadığı' dar geçitten geçebilmeli."""
    cm = Costmap(resolution=0.25, width_m=20.0, height_m=10.0, origin=(0.0, -2.0))
    # 2.4 m'lik bir boşluk bırakan iki engel (y=0 ve y=4.4, yarıçap 1.0)
    cm.add_obstacle(Obstacle(1, ObstacleClass.STATIC, center=(10.0, 0.0), radius=1.0))
    cm.add_obstacle(Obstacle(2, ObstacleClass.STATIC, center=(10.0, 4.4), radius=1.0))
    cm.compute_distance_field()

    gap_center = Pose2D(10.0, 2.2, 0.0)
    clearance = cm.distance_at(gap_center.x, gap_center.y)

    # Disk modeli (yarıçap ~1.67) bu boşluğu çarpışma sayardı; iki-daire (~1.07) saymaz.
    assert clearance < BEE1.footprint_radius, "Boşluk disk modeli için yeterince dar değil"
    assert cm.is_footprint_free(gap_center, BEE1), "İki-daire geçişi reddetti"

    # Uçtan uca: planlayıcı boşluktan geçen bir rota bulmalı
    planner = HybridAStar(cm, HybridAStarConfig(step_size=0.5, num_steer=7))
    path = planner.plan(Pose2D(2.0, 2.2, 0.0), Pose2D(18.0, 2.2, 0.0))
    assert path is not None, "İki-daire ile dar geçitten rota bulunamadı"


if __name__ == "__main__":
    run(verbose=True)
    print("\nTUM TESTLER GECTI.")
