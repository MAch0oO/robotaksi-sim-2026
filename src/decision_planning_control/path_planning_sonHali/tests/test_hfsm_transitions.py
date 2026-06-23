"""HFSM geçiş senaryosu testi.

Tek bir aracı INIT'ten MISSION_COMPLETE'e sürerek tüm kritik geçişleri uyarır:
ışık, statik/dinamik engel, yolcu operasyonu (süre bazlı), park ve acil durum.
Hem ``pytest`` ile hem de doğrudan ``python ...`` ile çalışır (senaryo dökümü basar).
"""

from __future__ import annotations

import math

from path_planning.common.enums import (
    LightState,
    MissionType,
    ObstacleClass,
    SubState,
    TopState,
    TurnSignal,
)
from path_planning.decision.transitions import DecisionConfig
from path_planning.common.types import (
    LaneInfo,
    Obstacle,
    Pose2D,
    TrafficLight,
    TrafficSign,
    VehicleState,
    Waypoint,
)
from path_planning.common.enums import SignType
from path_planning.decision.hfsm import HFSM
from path_planning.decision.mission_manager import MissionManager
from path_planning.io.inputs import PlanningInput

YAW_UP = math.pi / 2  # +y yönüne bakan araç: ileri = +y, sol = -x


def make_mission() -> MissionManager:
    wps = [
        Waypoint(0, Pose2D(0, 10, YAW_UP), MissionType.GOAL, tolerance=1.0),
        Waypoint(1, Pose2D(0, 20, YAW_UP), MissionType.PASSENGER_PICKUP, tolerance=1.0),
        Waypoint(2, Pose2D(0, 30, YAW_UP), MissionType.PARK_ENTRANCE, tolerance=1.0),
    ]
    return MissionManager(wps)


def make_input(
    x: float,
    y: float,
    stamp: float,
    *,
    obstacles=None,
    light=LightState.NONE,
    light_pos=None,
    emergency=False,
    park_target=None,
) -> PlanningInput:
    return PlanningInput(
        vehicle_state=VehicleState(Pose2D(x, y, YAW_UP), v=5.0, stamp=stamp),
        obstacles=obstacles or [],
        traffic_light=TrafficLight(state=light, position=light_pos),
        lane=LaneInfo(left_offset=3.0, right_offset=3.0),  # yandan geçişe yer olan yol
        park_target=park_target,
        emergency=emergency,
        stamp=stamp,
    )


def run_scenario(verbose: bool = False):
    # Test için kısa süreler (dwell 3s, kalkış 2s)
    hfsm = HFSM(make_mission(), DecisionConfig(passenger_dwell_time=3.0,
                                               departure_signal_time=2.0))
    seen = []

    def step(label, inp):
        d = hfsm.update(inp)
        seen.append((label, d))
        if verbose:
            sub = d.sub_state.name if d.sub_state else "-"
            print(f"[{inp.stamp:5.1f}s] {label:<22} TOP={d.top_state.name:<17} "
                  f"SUB={sub:<18} ACT={d.planner_action.name:<14} "
                  f"v={d.target_v:4.1f} prog={d.mission_progress}  ({d.info})")
        return d

    # 1) İlk döngü: INIT -> READY -> MISSION_EXECUTION, serbest yol
    d = step("baslangic", make_input(0, 0, 0.0))
    assert d.top_state is TopState.MISSION_EXECUTION
    assert d.sub_state is SubState.LANE_FOLLOWING

    # 2) Önde kırmızı ışık -> dur
    d = step("kirmizi isik", make_input(0, 0, 1.0, light=LightState.RED, light_pos=(0, 5)))
    assert d.sub_state is SubState.STOP_AND_WAIT and d.target_v == 0.0

    # 3) Yeşil -> tekrar seyir
    d = step("yesil isik", make_input(0, 0, 2.0, light=LightState.GREEN, light_pos=(0, 5)))
    assert d.sub_state is SubState.LANE_FOLLOWING

    # 4) Önde statik engel (şeritte yer var) -> sakınma
    stat = Obstacle(1, ObstacleClass.STATIC, center=(0.0, 8.0), radius=0.4)
    d = step("statik engel", make_input(0, 0, 3.0, obstacles=[stat]))
    assert d.sub_state is SubState.OBSTACLE_AVOIDANCE

    # 5) Çok yakın dinamik engel -> dur
    dyn = Obstacle(2, ObstacleClass.DYNAMIC, center=(0.0, 3.0), radius=0.4, velocity=(0.0, -1.0))
    d = step("yakin dinamik", make_input(0, 0, 4.0, obstacles=[dyn]))
    assert d.sub_state is SubState.STOP_AND_WAIT

    # 6) GOAL waypoint'e varış -> otomatik tamam, seyir devam (prog 1/3)
    d = step("goal varis", make_input(0, 10, 5.0))
    assert d.sub_state is SubState.LANE_FOLLOWING
    assert hfsm.mission.progress == "1/3"

    # 7) Yolcu noktasına varış -> PASSENGER_OPS (kapı açık, bekleme başlar)
    d = step("yolcu varis", make_input(0, 20, 6.0))
    assert d.sub_state is SubState.PASSENGER_OPS and d.target_v == 0.0
    assert d.door_open is True, "Durakta kapi acik bildirimi bekleniyordu"

    # 8) Bekleme (3s) dolunca -> PASSENGER_DEPARTURE (kapı kapalı, sol sinyal)
    d = step("yolcu kalkis", make_input(0, 20, 9.5))
    assert d.sub_state is SubState.PASSENGER_DEPARTURE
    assert d.door_open is False and d.turn_signal is TurnSignal.LEFT

    # 9) Kalkış sinyali (2s) dolunca -> görev tamam, sürüşe dön
    d = step("kalkis bitti", make_input(0, 20, 12.0))
    assert hfsm.mission.progress == "2/3"

    # 10) Park giriş noktasına varış -> PARKING (park_target slot: (0,31))
    park_slot = Pose2D(0, 31, YAW_UP)
    d = step("park varis", make_input(0, 30, 13.0, park_target=park_slot))
    assert d.sub_state is SubState.PARKING and d.planner_action.name == "PARK"
    assert d.turn_signal is TurnSignal.HAZARD, "Parkta dortlu flasor bekleniyordu"

    # 11) Slot'a (park_target) ulaşıldı -> bu döngüde park tamamlanır
    d = step("park tamam", make_input(0, 31, 14.0, park_target=park_slot))
    assert d.sub_state is SubState.PARKING and hfsm.mission.progress == "3/3"

    # 12) Görev bitti -> MISSION_COMPLETE
    d = step("park sonrasi", make_input(0, 31, 15.0, park_target=park_slot))
    assert d.top_state is TopState.MISSION_COMPLETE

    # 13) Acil durum her durumu ezer (dörtlü flaşör)
    d = step("acil durum", make_input(0, 31, 16.0, emergency=True))
    assert d.top_state is TopState.EMERGENCY_STOP and d.target_v == 0.0
    assert d.turn_signal is TurnSignal.HAZARD

    return hfsm


def _sign_input(sign, stamp):
    return PlanningInput(
        vehicle_state=VehicleState(Pose2D(0, 0, YAW_UP), v=5.0, stamp=stamp),
        lane=LaneInfo(left_offset=3.0, right_offset=3.0),
        traffic_sign=sign,
        stamp=stamp,
    )


def test_sign_no_entry_stops():
    """GİRİLMEZ levhası -> STOP_AND_WAIT + no_entry kısıtı."""
    hfsm = HFSM(MissionManager([Waypoint(0, Pose2D(0, 50, YAW_UP), MissionType.GOAL, 1.0)]))
    hfsm.update(_sign_input(None, 0.0))  # MISSION_EXECUTION'a ısın
    d = hfsm.update(_sign_input(TrafficSign(SignType.NO_ENTRY, distance=5.0), 0.1))
    assert d.sub_state is SubState.STOP_AND_WAIT
    assert d.constraints.no_entry is True


def test_sign_speed_cap():
    """İKİ YÖNLÜ YOL levhası -> seyir sürer ama hız yavaşlama sınırına iner."""
    hfsm = HFSM(MissionManager([Waypoint(0, Pose2D(0, 50, YAW_UP), MissionType.GOAL, 1.0)]))
    hfsm.update(_sign_input(None, 0.0))
    d = hfsm.update(_sign_input(TrafficSign(SignType.TWO_WAY, distance=5.0), 0.1))
    assert d.sub_state is SubState.LANE_FOLLOWING
    assert abs(d.target_v - 20.0 / 3.6) < 0.05   # slow_speed
    assert d.constraints.two_way is True


def test_full_scenario():
    hfsm = run_scenario(verbose=False)
    # Determinizm: ikinci koşum aynı geçiş logunu üretmeli
    again = run_scenario(verbose=False)
    log1 = [(r.level, r.from_state, r.to_state) for r in hfsm.transition_log]
    log2 = [(r.level, r.from_state, r.to_state) for r in again.transition_log]
    assert log1 == log2


if __name__ == "__main__":
    hfsm = run_scenario(verbose=True)
    print("\n--- Gecis Logu ---")
    for r in hfsm.transition_log:
        print(f"  [{r.stamp:5.1f}s] {r.level:<3} {r.from_state:<17} -> {r.to_state:<17} ({r.reason})")
    print("\nTUM TESTLER GECTI." if True else "")
