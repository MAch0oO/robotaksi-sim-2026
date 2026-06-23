"""Hiyerarşik Sonlu Durum Makinesi (HFSM) motoru.

Her kontrol döngüsünde ``update(PlanningInput)`` çağrılır ve bir
``BehaviorDecision`` üretir: aktif üst/alt durum + planlayıcı emri + hedef hız.

Hiyerarşi:
    Üst seviye:  INIT -> READY -> MISSION_EXECUTION -> MISSION_COMPLETE
                 EMERGENCY_STOP (her durumdan, en yüksek öncelik)
    Alt seviye:  yalnızca MISSION_EXECUTION içinde (transitions.select_substate)

Tasarım ilkeleri:
  * Deterministik: aynı girdi dizisi -> aynı durum dizisi.
  * Her an tek aktif alt durum.
  * Geçişler hem olay (engel, ışık, varış) hem süre (yolcu beklemesi) bazlı.
  * Tüm geçişler loglanır.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from ..common.enums import (
    MissionType,
    PlannerAction,
    SignType,
    SubState,
    TopState,
    TurnSignal,
)
from ..common.types import Constraints
from ..common.vehicle_params import BEE1, VehicleParams
from ..io.inputs import PlanningInput
from .mission_manager import MissionManager
from .states import StateContext, build_state_table
from .transitions import (
    DecisionConfig,
    apply_sign,
    build_world_view,
    select_substate,
    signal_from_constraints,
)


@dataclass
class BehaviorDecision:
    """HFSM'in tek döngülük çıktısı (planlama hattının girdisi olur)."""

    top_state: TopState
    sub_state: Optional[SubState]
    planner_action: PlannerAction
    target_v: float
    mission_progress: str = ""
    info: str = ""
    door_open: bool = False
    turn_signal: TurnSignal = TurnSignal.NONE
    constraints: Constraints = field(default_factory=Constraints)


@dataclass
class TransitionRecord:
    """Loglanan tek geçiş (determinizm doğrulaması ve teşhis için)."""

    stamp: float
    level: str          # "TOP" veya "SUB"
    from_state: str
    to_state: str
    reason: str


class HFSM:
    """Robotaksi davranış karar motoru."""

    def __init__(
        self,
        mission: MissionManager,
        config: Optional[DecisionConfig] = None,
        vehicle: VehicleParams = BEE1,
    ):
        self.mission = mission
        self.config = config or DecisionConfig()
        self._ctx = StateContext(vehicle=vehicle)
        self._states = build_state_table()

        self.top_state: TopState = TopState.INIT
        self.sub_state: Optional[SubState] = None

        self._parked: bool = False
        self._parking_latched: bool = False        # park_giris'e varıldı, manevra sürüyor
        # Durak (yolcu transferi) faz takibi: None / "WAIT" / "DEPART"
        self._passenger_phase: Optional[str] = None
        self._phase_start: float = 0.0
        # Levha (sign) durumu
        self._constraints = Constraints()
        self._active_sign: SignType = SignType.NONE
        self._sign_time: float = 0.0
        self._sign_speed_cap: float = float("inf")
        self._sign_force_stop: bool = False
        self.transition_log: List[TransitionRecord] = []

    # ------------------------------------------------------------------ #
    # Ana döngü
    # ------------------------------------------------------------------ #
    def update(self, inp: PlanningInput) -> BehaviorDecision:
        """Bir kontrol döngüsünü işler ve davranış kararını döndürür."""
        # 0) Acil durum her şeyi ezer.
        if inp.emergency:
            self._set_top(TopState.EMERGENCY_STOP, inp.stamp, "acil durdurma sinyali")
            self._leave_substate(inp.stamp)
            return self._decide(SubState.STOP_AND_WAIT, PlannerAction.IDLE, 0.0,
                                "ACIL DURDURMA", turn_signal=TurnSignal.HAZARD)

        # Acil durum kalktıysa göreve dön.
        if self.top_state is TopState.EMERGENCY_STOP and not inp.emergency:
            self._set_top(TopState.MISSION_EXECUTION, inp.stamp, "acil durum kalkti")

        # 1) Üst seviye ilerleme.
        if self.top_state is TopState.INIT:
            if inp.waypoints or self.mission.current() is not None:
                self._set_top(TopState.READY, inp.stamp, "gorev yuklendi")
        if self.top_state is TopState.READY:
            self._set_top(TopState.MISSION_EXECUTION, inp.stamp, "harekete gec")

        if self.top_state is TopState.MISSION_EXECUTION and self.mission.is_finished:
            self._set_top(TopState.MISSION_COMPLETE, inp.stamp, "tum gorevler tamam")

        # 2) Hareketsiz üst durumlar.
        if self.top_state in (TopState.INIT, TopState.READY, TopState.MISSION_COMPLETE):
            self._leave_substate(inp.stamp)
            label = self.top_state.name.lower()
            return self._decide(None, PlannerAction.IDLE, 0.0, label)

        # 3) Levhaları değerlendir (kısıt + hız sınırı + dur).
        self._evaluate_signs(inp)

        # 4) MISSION_EXECUTION: görev bayraklarını güncelle, durum seç.
        at_park, at_passenger = self._update_mission_flags(inp)
        world = build_world_view(
            inp,
            at_park=at_park,
            at_passenger_stop=at_passenger,
            mission_finished=self.mission.is_finished,
            config=self.config,
        )
        chosen = select_substate(world)

        # Levha kaynaklı zorunlu duruş (girilmez) — acil dışında en güçlü.
        if self._sign_force_stop and chosen is not SubState.STOP_AND_WAIT:
            chosen = SubState.STOP_AND_WAIT

        # 5) Faz/zaman bazlı durum (yolcu transferi, park tamamlanması).
        effective = self._resolve_phase(chosen, inp)
        self._enter_substate(effective, inp.stamp, world)

        out = self._states[effective].execute(world, self._ctx)
        # Levha hız sınırını uygula.
        target_v = min(out.target_v, self._sign_speed_cap)
        door, signal = self._notifications(effective)
        return self._decide(effective, out.planner_action, target_v, out.info,
                            door_open=door, turn_signal=signal)

    # ------------------------------------------------------------------ #
    # Görev bayrakları
    # ------------------------------------------------------------------ #
    def _update_mission_flags(self, inp: PlanningInput) -> Tuple[bool, bool]:
        """Aktif waypoint'e göre (at_park, at_passenger) bayraklarını üretir.

        GOAL noktaları varışta otomatik tamamlanır (geçiş noktası). Yolcu ve
        park noktaları durumu tetikler; tamamlanmaları zamanlayıcıya bağlıdır.
        """
        pose = inp.vehicle_state.pose
        mtype = self.mission.current_type()

        # Park: park_giris'e varış PARKING'i kilitler (latch). Araç slota ilerlerken
        # park_giris toleransından çıksa bile durum düşmesin diye latch korunur;
        # latch yalnızca park_target'a varışta (_handle_timers) temizlenir.
        if mtype is MissionType.PARK_ENTRANCE:
            if self.mission.arrived(pose):
                self._parking_latched = True
            return self._parking_latched, False

        if not self.mission.arrived(pose):
            return False, False
        if mtype is MissionType.GOAL:
            self.mission.complete_current()  # geçiş noktası: anında tamam
            return False, False
        if mtype in (MissionType.PASSENGER_PICKUP, MissionType.PASSENGER_DROPOFF):
            return False, True
        return False, False

    # ------------------------------------------------------------------ #
    # Levha değerlendirme
    # ------------------------------------------------------------------ #
    def _evaluate_signs(self, inp: PlanningInput) -> None:
        """Görüş mesafesindeki levhayı kısıt + hız sınırı + dur'a çevirir.

        Aynı levha görüldükçe etki tazelenir; görüş kaybından ``sign_effect_time``
        sonra kısıtlar temizlenir (referans LEVHA_ETKI mantığı).
        """
        now = inp.stamp
        sign = inp.traffic_sign
        if (sign is not None and sign.sign_type is not SignType.NONE
                and sign.distance <= self.config.sign_read_distance):
            if sign.sign_type is not self._active_sign:
                self._constraints = Constraints()
                self._sign_speed_cap, self._sign_force_stop = apply_sign(
                    sign.sign_type, self._constraints, self.config)
                self._active_sign = sign.sign_type
            self._sign_time = now  # görüldükçe tazele

        # Etki süresi dolduysa temizle
        if (self._active_sign is not SignType.NONE
                and now - self._sign_time > self.config.sign_effect_time):
            self._active_sign = SignType.NONE
            self._constraints = Constraints()
            self._sign_speed_cap = float("inf")
            self._sign_force_stop = False

    # ------------------------------------------------------------------ #
    # Faz çözümleme: yolcu transferi + park tamamlanması
    # ------------------------------------------------------------------ #
    def _resolve_phase(self, chosen: SubState, inp: PlanningInput) -> SubState:
        """Seçilen alt durumu, zaman bazlı fazlara göre etkin alt duruma çevirir."""
        stamp = inp.stamp

        # Yolcu transferi: WAIT (kapı açık) -> DEPART (sol sinyal) -> tamam
        if chosen is SubState.PASSENGER_OPS:
            if self._passenger_phase is None:
                self._passenger_phase = "WAIT"
                self._phase_start = stamp
            if self._passenger_phase == "WAIT":
                if stamp - self._phase_start >= self.config.passenger_dwell_time:
                    self._passenger_phase = "DEPART"
                    self._phase_start = stamp
                else:
                    return SubState.PASSENGER_OPS
            if self._passenger_phase == "DEPART":
                if stamp - self._phase_start >= self.config.departure_signal_time:
                    self.mission.complete_current()  # durak tamam -> sıradakine
                    self._passenger_phase = None
                    return SubState.LANE_FOLLOWING
                return SubState.PASSENGER_DEPARTURE
        else:
            self._passenger_phase = None

        # Park tamamlanması: araç park_target'a (algı slot'u) vardı mı?
        if chosen is SubState.PARKING and not self._parked and inp.park_target is not None:
            if inp.vehicle_state.pose.distance_to(inp.park_target) <= self.config.park_tolerance:
                self._parked = True
                self._parking_latched = False
                self.mission.complete_current()  # park_giris tamam -> görev biter

        return chosen

    # ------------------------------------------------------------------ #
    # Donanım bildirimleri (kapı + sinyal)
    # ------------------------------------------------------------------ #
    def _notifications(self, sub: SubState) -> Tuple[bool, TurnSignal]:
        """Etkin alt duruma göre (door_open, turn_signal) üretir."""
        if sub is SubState.PASSENGER_OPS:
            return True, TurnSignal.NONE              # kapı açık, bekleme
        if sub is SubState.PASSENGER_DEPARTURE:
            return False, TurnSignal.LEFT             # kalkış: sol sinyal
        if sub is SubState.PARKING:
            return False, TurnSignal.HAZARD           # park: dörtlü
        return False, signal_from_constraints(self._constraints)

    # ------------------------------------------------------------------ #
    # Durum geçiş yardımcıları + loglama
    # ------------------------------------------------------------------ #
    def _set_top(self, new: TopState, stamp: float, reason: str) -> None:
        if new is not self.top_state:
            self.transition_log.append(
                TransitionRecord(stamp, "TOP", self.top_state.name, new.name, reason)
            )
            self.top_state = new

    def _enter_substate(self, new: SubState, stamp: float, world) -> None:
        if new is self.sub_state:
            return
        if self.sub_state is not None:
            self._states[self.sub_state].on_exit(self._ctx)
        self.transition_log.append(
            TransitionRecord(
                stamp, "SUB",
                self.sub_state.name if self.sub_state else "None",
                new.name, world_reason(world),
            )
        )
        self.sub_state = new
        self._states[new].on_enter(self._ctx)

    def _leave_substate(self, stamp: float) -> None:
        if self.sub_state is not None:
            self._states[self.sub_state].on_exit(self._ctx)
            self.transition_log.append(
                TransitionRecord(stamp, "SUB", self.sub_state.name, "None", "ust durum degisti")
            )
            self.sub_state = None
        self._passenger_phase = None

    def _decide(
        self,
        sub: Optional[SubState],
        action: PlannerAction,
        target_v: float,
        info: str,
        *,
        door_open: bool = False,
        turn_signal: TurnSignal = TurnSignal.NONE,
    ) -> BehaviorDecision:
        return BehaviorDecision(
            top_state=self.top_state,
            sub_state=sub,
            planner_action=action,
            target_v=target_v,
            mission_progress=self.mission.progress,
            info=info,
            door_open=door_open,
            turn_signal=turn_signal,
            constraints=self._constraints,
        )


def world_reason(world) -> str:
    """Bir alt durum geçişinin insan-okur gerekçesini üretir (log için)."""
    if world.must_stop:
        if world.red_light_ahead:
            return "kirmizi isik"
        if world.dynamic_block_close:
            return "yakin dinamik engel"
        return "sakinilmaz statik engel"
    if world.obstacle_avoidable:
        return "sakinilabilir engel"
    if world.at_park:
        return "park girisine varildi"
    if world.at_passenger_stop:
        return "yolcu noktasina varildi"
    return "serbest yol"
