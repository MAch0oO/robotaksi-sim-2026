"""Davranış durumu sınıfları (State pattern).

Her alt durum, ``execute`` çağrısında o döngü için iki şey üretir:
  * ``planner_action``: planlama hattına verilecek yüksek seviyeli emir.
  * ``target_v``: bu durumda izin verilen üst hız (m/s).
``on_enter`` / ``on_exit`` ileride sıfırlama/log için kanca sağlar.

Durumlar planlamayı YAPMAZ; yalnızca planlayıcıya ne yapacağını söyler.
Gerçek yörünge üretimi Hybrid A* / TEB (Tur C-D) işidir.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from ..common.enums import PlannerAction, SubState
from ..common.vehicle_params import VehicleParams
from .transitions import WorldView


@dataclass
class StateOutput:
    """Bir durumun tek döngülük kararı."""

    planner_action: PlannerAction
    target_v: float
    info: str = ""


@dataclass
class StateContext:
    """Durumların paylaştığı salt-okunur bağlam."""

    vehicle: VehicleParams
    avoid_speed_factor: float = 0.5   # sakınma sırasında max hızın oranı
    parking_speed: float = 1.0        # park manevrası hızı (m/s)


class BehaviorState(ABC):
    """Tüm alt davranış durumlarının soyut tabanı."""

    substate: SubState

    def on_enter(self, ctx: StateContext) -> None:  # noqa: D401 - kanca
        """Duruma girişte bir kez çağrılır (varsayılan: işlem yok)."""

    def on_exit(self, ctx: StateContext) -> None:
        """Durumdan çıkışta bir kez çağrılır (varsayılan: işlem yok)."""

    @abstractmethod
    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        """Bu döngünün planlayıcı emrini ve hedef hızını üretir."""


class LaneFollowing(BehaviorState):
    """Normal seyir: TEB global rotayı serbest hızda takip eder."""

    substate = SubState.LANE_FOLLOWING

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        return StateOutput(PlannerAction.FOLLOW_GLOBAL, ctx.vehicle.max_speed, "serit takibi")


class ObstacleAvoidance(BehaviorState):
    """Sakınma: TEB yörüngeyi engelden kaçacak şekilde deforme eder, hız düşer."""

    substate = SubState.OBSTACLE_AVOIDANCE

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        target = ctx.vehicle.max_speed * ctx.avoid_speed_factor
        return StateOutput(PlannerAction.AVOID, target, "engelden sakinma")


class StopAndWait(BehaviorState):
    """Dur ve bekle: kırmızı ışık / yaya / sakınılamaz engel. Hedef hız 0."""

    substate = SubState.STOP_AND_WAIT

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        if world.red_light_ahead:
            reason = "kirmizi isik"
        elif world.dynamic_block_close:
            reason = "yakin dinamik engel"
        else:
            reason = "yol kapali (statik)"
        return StateOutput(PlannerAction.HOLD, 0.0, f"dur-bekle: {reason}")


class PassengerOps(BehaviorState):
    """Durak: kapı açık, yolcu al/bırak beklemesi. Araç durur."""

    substate = SubState.PASSENGER_OPS

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        return StateOutput(PlannerAction.HOLD, 0.0, "durak: yolcu bekleme")


class PassengerDeparture(BehaviorState):
    """Duraktan kalkış: kapı kapalı, sol sinyal süresi. Araç hâlâ durur."""

    substate = SubState.PASSENGER_DEPARTURE

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        return StateOutput(PlannerAction.HOLD, 0.0, "durak: kalkis (sol sinyal)")


class Parking(BehaviorState):
    """Park: Hybrid A* park moduna geçer, düşük hızda dik park manevrası."""

    substate = SubState.PARKING

    def execute(self, world: WorldView, ctx: StateContext) -> StateOutput:
        return StateOutput(PlannerAction.PARK, ctx.parking_speed, "park manevrasi")


def build_state_table() -> dict:
    """Tüm alt durumların {SubState: BehaviorState} eşlemesini üretir."""
    states = [
        LaneFollowing(),
        ObstacleAvoidance(),
        StopAndWait(),
        PassengerOps(),
        PassengerDeparture(),
        Parking(),
    ]
    return {s.substate: s for s in states}
