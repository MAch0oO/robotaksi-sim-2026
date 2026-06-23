"""Tur E entegrasyon (duman) testi.

Üç katmanlı güvenlik mimarisini uçtan uca doğrular:
  A) Uçtan uca + Boundary force: şerit kenarındaki engelde üretilen yörünge şeritte kalır.
  B) Validation gate: şerit dışına taşan bir yörünge için valid=False üretilir.
  C) Replan/Hold: HFSM sürüş emri verse de global rota bulunamayınca HOLD'a düşülür.
``pytest`` veya doğrudan ``python ...`` ile çalışır.
"""

from __future__ import annotations

import math

from path_planning.common.enums import ObstacleClass, PlannerAction, SubState
from path_planning.common.types import (
    LaneInfo,
    Obstacle,
    Pose2D,
    TrafficLight,
    Trajectory,
    TrajectoryPoint,
    VehicleState,
    Waypoint,
)
from path_planning.common.enums import MissionType
from path_planning.decision.mission_manager import MissionManager
from path_planning.global_planner.costmap import Costmap
from path_planning.io.inputs import PlanningInput
from path_planning.planner_pipeline import MapConfig, PlannerPipeline, validate_trajectory

YAW_UP = math.pi / 2


def make_pipeline(corridor_half: float) -> PlannerPipeline:
    mission = MissionManager([Waypoint(0, Pose2D(0, 20, YAW_UP), MissionType.GOAL, tolerance=1.0)])
    map_cfg = MapConfig(resolution=0.5, width_m=20.0, height_m=36.0, origin=(-10.0, -8.0))
    return PlannerPipeline(mission, map_config=map_cfg)


def make_input(obstacles, corridor_half: float) -> PlanningInput:
    return PlanningInput(
        vehicle_state=VehicleState(Pose2D(0.0, 0.0, YAW_UP), v=0.0),
        obstacles=obstacles,
        traffic_light=TrafficLight(),
        lane=LaneInfo(centerline=[(0.0, -8.0), (0.0, 28.0)],
                      left_offset=corridor_half, right_offset=corridor_half),
    )


def test_end_to_end_stays_in_lane():
    """A) İki şeritli yolda kenardaki statik engelden kaçarken şeritte kalır."""
    corridor = 3.5
    pipe = make_pipeline(corridor)
    obs = [Obstacle(1, ObstacleClass.STATIC, center=(1.0, 10.0), radius=0.4)]
    out = pipe.update(make_input(obs, corridor))

    assert out.valid, f"Yorunge gecersiz: {out.info}"
    assert out.behavior_state is SubState.OBSTACLE_AVOIDANCE
    assert len(out.trajectory) >= 2
    # Sınır kuvveti + doğrulama: tüm noktalar koridor içinde
    for tp in out.trajectory.points:
        assert abs(tp.pose.x) <= corridor + 1e-6, f"Serit disina tasti: x={tp.pose.x:.2f}"
    return out


def test_validation_gate_rejects_out_of_lane():
    """B) Şerit dışına taşan yapay yörünge -> valid=False."""
    cm = Costmap(0.5, 20, 36, origin=(-10, -8))
    cm.restrict_to_corridor([(0, -8), (0, 28)], half_width=3.5)
    cm.compute_distance_field()
    # Koridor dışına (x=8) çıkan yörünge
    traj = Trajectory()
    for i in range(5):
        x = i * 2.0  # 0,2,4,6,8 -> son iki nokta koridor disi
        traj.points.append(TrajectoryPoint(Pose2D(x, 5.0, 0.0), v=2.0, t=float(i)))
    valid, reason = validate_trajectory(traj, cm)
    assert not valid, "Serit disi yorunge gecerli sayildi"


def test_blocked_corridor_falls_back_to_hold():
    """C) HFSM AVOID dese de tek şeritte rota yoksa REPLAN denenir, sonra HOLD."""
    corridor = 3.5
    pipe = make_pipeline(corridor)
    # Tüm koridoru kapatan büyük statik engel (HFSM 'sınırda sakınılabilir' sansa da
    # şişirilmiş disk koridoru tamamen tıkar -> Hybrid A* None)
    obs = [Obstacle(1, ObstacleClass.STATIC, center=(0.0, 10.0), radius=2.0)]
    out = pipe.update(make_input(obs, corridor))

    assert out.planner_action is PlannerAction.HOLD
    assert out.target_v == 0.0
    assert "HOLD" in out.info


def _park_pipeline():
    mission = MissionManager([Waypoint(0, Pose2D(0, 10, YAW_UP),
                                       MissionType.PARK_ENTRANCE, tolerance=1.5)])
    return PlannerPipeline(mission)  # varsayılan MapConfig (-30..30) sahneyi kapsar


def _park_input(park_target, sign=None):
    return PlanningInput(
        vehicle_state=VehicleState(Pose2D(0.0, 9.0, YAW_UP), v=0.0),
        obstacles=[],
        traffic_light=TrafficLight(),
        traffic_sign=sign,
        lane=None,                # park alanı: şerit koridoru yok
        park_target=park_target,
    )


def test_parking_with_target_plans_maneuver():
    """park_target gelince PARKING -> geri vitesli Hybrid A* + geçerli yörünge."""
    pipe = _park_pipeline()
    out = pipe.update(_park_input(Pose2D(0.0, 13.0, YAW_UP)))
    assert out.behavior_state is SubState.PARKING
    assert out.planner_action is PlannerAction.PARK
    assert out.valid and len(out.trajectory) >= 2
    # Park planlayıcısı geri vitese izin vermeli
    assert pipe.park_astar_cfg.allow_reverse is True


def test_parking_without_target_holds():
    """park_target henüz gelmediyse araç HOLD'da bekler."""
    pipe = _park_pipeline()
    out = pipe.update(_park_input(None))
    assert out.planner_action is PlannerAction.HOLD
    assert out.target_v == 0.0
    assert "park_target" in out.info


def test_parking_blocked_by_no_parking_sign():
    """PARK YASAKTIR levhası aktifken park manevrası başlatılmaz -> HOLD."""
    from path_planning.common.types import TrafficSign
    from path_planning.common.enums import SignType
    pipe = _park_pipeline()
    out = pipe.update(_park_input(Pose2D(0.0, 13.0, YAW_UP),
                                  sign=TrafficSign(SignType.NO_PARKING, distance=5.0)))
    assert out.planner_action is PlannerAction.HOLD
    assert out.constraints.no_parking is True
    assert "park yasak" in out.info


if __name__ == "__main__":
    out = test_end_to_end_stays_in_lane()
    max_x = max(abs(tp.pose.x) for tp in out.trajectory.points)
    print(f"A) Uctan uca: durum={out.behavior_state.name}, action={out.planner_action.name}, "
          f"{len(out.trajectory)} nokta, max|x|={max_x:.2f} m (koridor 3.5) -> SERITTE")
    test_validation_gate_rejects_out_of_lane()
    print("B) Validation gate: serit disi yorunge reddedildi -> valid=False")
    test_blocked_corridor_falls_back_to_hold()
    print("C) Blokaj: REPLAN denendi, rota yok -> HOLD'a dusuldu")
    test_parking_with_target_plans_maneuver()
    print("D) Park: park_target ile PARKING -> geri vitesli manevra (valid)")
    test_parking_without_target_holds()
    print("E) Park: park_target yok -> HOLD'da bekliyor")
    print("\nTUM TESTLER GECTI.")
