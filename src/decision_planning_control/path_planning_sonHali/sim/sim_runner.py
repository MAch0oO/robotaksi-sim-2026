"""Uçtan uca pist simülasyonu (ROS'suz).

GEOJSON görevini yükler, ``PlannerPipeline``'ı gerçek pist ölçeğinde çalıştırır ve
aracı üretilen yörüngeyi kusursuz takip eden idealize bir kinematik modelle ilerletir.
Amaç: tüm zincirin (GEOJSON→HFSM→Hybrid A*→TEB→çıktı→ilerleme→waypoint→park) uçtan uca
ve gerçek ölçekte çalıştığını + kabaca hesap maliyetini görmek.

Senaryoya statik engel ve geçici kırmızı ışık enjekte edilerek sakınma ve HOLD da uyarılır.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..common.enums import LightState, ObstacleClass, TopState
from ..common.types import (
    LaneInfo,
    Obstacle,
    Pose2D,
    TrafficLight,
    Trajectory,
    VehicleState,
)
from ..decision.mission_manager import MissionManager
from ..global_planner.hybrid_astar import HybridAStarConfig
from ..io.geojson_loader import load_waypoints
from ..io.inputs import PlanningInput
from ..planner_pipeline import MapConfig, PlannerPipeline


@dataclass
class SimConfig:
    dt: float = 0.1                 # simülasyon adımı (s)
    max_time: float = 200.0         # güvenlik üst sınırı (s)
    arrival_radius: float = 1.0     # GOAL waypoint varış toleransı (m)


def _bounds(points: List[Tuple[float, float]], margin: float = 15.0) -> MapConfig:
    """Waypoint kümesini kapsayan (kaba çözünürlüklü, hız için) costmap sınırları."""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ox, oy = min(xs) - margin, min(ys) - margin
    w = (max(xs) - min(xs)) + 2 * margin
    h = (max(ys) - min(ys)) + 2 * margin
    return MapConfig(resolution=1.0, width_m=w, height_m=h, origin=(ox, oy))


def _advance(pose: Pose2D, traj: Trajectory, speed: float, dt: float) -> Pose2D:
    """Aracı yörünge boyunca |speed|*dt kadar ilerletir (idealize takip)."""
    pts = traj.points
    if len(pts) < 2 or speed <= 1e-6:
        return pose
    remaining = speed * dt
    cx, cy, cyaw = pts[0].pose.x, pts[0].pose.y, pts[0].pose.yaw
    for a, b in zip(pts, pts[1:]):
        seg = a.pose.distance_to(b.pose)
        if seg < 1e-9:
            continue
        if seg <= remaining:
            remaining -= seg
            cx, cy, cyaw = b.pose.x, b.pose.y, b.pose.yaw
        else:
            t = remaining / seg
            cx = a.pose.x + t * (b.pose.x - a.pose.x)
            cy = a.pose.y + t * (b.pose.y - a.pose.y)
            cyaw = b.pose.yaw
            break
    return Pose2D(cx, cy, cyaw)


def run(geojson_path: str, verbose: bool = True) -> dict:
    """Görevi uçtan uca koşturur; özet sözlük döndürür."""
    waypoints, _ = load_waypoints(geojson_path, default_tolerance=1.5)
    drive_pts = [(w.pose.x, w.pose.y) for w in waypoints]

    mission = MissionManager(waypoints)
    pipe = PlannerPipeline(
        mission,
        map_config=_bounds(drive_pts),
        astar_config=HybridAStarConfig(step_size=2.0, num_steer=5, max_iterations=200_000),
    )

    # Park hedefi: park giriş noktasının yaklaşma yönünde 2 m ilerisi (kısa, ulaşılabilir manevra)
    park_idx = next((i for i, w in enumerate(waypoints)
                     if w.mission_type.name == "PARK_ENTRANCE"), None)
    park_target = None
    if park_idx is not None:
        park_wp = waypoints[park_idx]
        prev = waypoints[park_idx - 1].pose if park_idx > 0 else Pose2D(0, 0, 0)
        approach = math.atan2(park_wp.pose.y - prev.y, park_wp.pose.x - prev.x)
        park_target = Pose2D(park_wp.pose.x + 2.0 * math.cos(approach),
                             park_wp.pose.y + 2.0 * math.sin(approach), approach)

    # Senaryo enjeksiyonu: ilk segmentte statik engel + 4-7 s arası kırmızı ışık
    start = waypoints[0].pose if waypoints else Pose2D(0, 0, 0)
    first = waypoints[0].pose
    obstacle = Obstacle(99, ObstacleClass.STATIC,
                        center=(0.5 * first.x, 0.5 * first.y + 1.5), radius=0.5)

    cfg = SimConfig()
    pose = Pose2D(0.0, 0.0, math.atan2(first.y, first.x))  # start, ilk hedefe bakar
    v = 0.0
    t = 0.0
    wall0 = time.perf_counter()
    replans = 0
    last_state = None
    timeline: List[str] = []

    while t < cfg.max_time:
        light = TrafficLight(LightState.RED, position=(pose.x + 5 * math.cos(pose.yaw),
                                                       pose.y + 5 * math.sin(pose.yaw))) \
            if 4.0 <= t <= 7.0 else TrafficLight(LightState.GREEN)

        inp = PlanningInput(
            vehicle_state=VehicleState(pose, v=v, stamp=t),
            obstacles=[obstacle],
            traffic_light=light,
            lane=None,
            park_target=park_target,
            stamp=t,
        )
        n_before = pipe._planned_goal_key
        out = pipe.update(inp)
        if pipe._planned_goal_key != n_before:
            replans += 1

        state = out.behavior_state.name if out.behavior_state else pipe.hfsm.top_state.name
        if state != last_state:
            timeline.append(f"[t={t:6.1f}s] {state:<18} prog={mission.progress} "
                            f"pos=({pose.x:6.1f},{pose.y:6.1f}) {out.info}")
            last_state = state

        if pipe.hfsm.top_state is TopState.MISSION_COMPLETE:
            break

        v = out.target_v
        pose = _advance(pose, out.trajectory, v, cfg.dt)
        t += cfg.dt

    wall = time.perf_counter() - wall0
    result = {
        "completed": pipe.hfsm.top_state is TopState.MISSION_COMPLETE,
        "sim_time": t,
        "wall_time": wall,
        "replans": replans,
        "progress": mission.progress,
        "timeline": timeline,
    }
    if verbose:
        print(f"GEOJSON: {geojson_path}")
        print(f"Waypoint sayisi: {len(waypoints)}  | Harita: "
              f"{pipe.map_cfg.width_m:.0f}x{pipe.map_cfg.height_m:.0f} m @ {pipe.map_cfg.resolution} m")
        print("--- Durum zaman cizelgesi ---")
        for line in timeline:
            print(" ", line)
        print(f"\nTamamlandi : {result['completed']}")
        print(f"Sim suresi : {t:.1f} s   Gercek hesap suresi: {wall:.2f} s   Replan: {replans}")
    return result


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    run(os.path.join(here, "config", "sample_mission.geojson"))
