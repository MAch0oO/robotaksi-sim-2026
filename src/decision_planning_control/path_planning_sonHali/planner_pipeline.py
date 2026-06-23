"""Uçtan uca planlama hattı (orkestratör).

Her kontrol döngüsünde "Tek Dünya" + derinlemesine savunma akışını işletir:

    PlanningInput
        │
        ▼  HFSM.update  → davranış kararı (PlannerAction + target_v)
        ▼  Costmap kur  → şerit + statik + dinamik (anlık konum) rasterize + EDT
        ▼  Hybrid A*    → (gerekirse) sert kısıtlı global rota
        ▼  TEB          → costmap gradyanıyla band + zaman-parametreli Trajectory
        ▼  Validate     → costmap.is_free örnekleme  (güvenlik kapısı)
        ▼  Politika     → valid? ver : REPLAN_GLOBAL → HOLD
    PlanningOutput

HFSM kararı planlamayı yönetir; planlayıcılar yalnızca emir aldıklarında çalışır.
Dinamik engel tahmini (hız vektörüyle gelecek konum) bu iskelette YOKtur — engeller
anlık konumlarıyla costmap'e işlenir (bilinçli sadeleştirme, optimizasyon aşamasına bırakıldı).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import List, Optional, Tuple

from .common.enums import PlannerAction
from .common.types import Pose2D, Trajectory, TrajectoryPoint
from .common.vehicle_params import BEE1, VehicleParams
from .decision.hfsm import HFSM, BehaviorDecision
from .decision.mission_manager import MissionManager
from .decision.transitions import DecisionConfig
from .global_planner.costmap import Costmap
from .global_planner.hybrid_astar import HybridAStar, HybridAStarConfig
from .io.inputs import PlanningInput
from .io.outputs import PlanningOutput
from .local_planner.teb import TEB, TEBConfig


@dataclass
class MapConfig:
    """Her döngü kurulan costmap'in boyut/çözünürlük ayarları."""

    resolution: float = 0.5
    width_m: float = 60.0
    height_m: float = 60.0
    origin: Tuple[float, float] = (-30.0, -30.0)
    lane_half_width: float = 1.75   # şerit bilgisi yoksa varsayılan koridor yarı genişliği


def validate_trajectory(
    traj: Trajectory,
    costmap: Costmap,
    vehicle: VehicleParams = BEE1,
    sample_ds: float = 0.25,
) -> Tuple[bool, str]:
    """Yörüngeyi costmap'e karşı denetler (güvenlik kapısı / Katman 2).

    Her band noktasını ve aralarındaki ara örnekleri **iki-daire ayak izi**
    (`is_footprint_free`, yön-duyarlı) ile kontrol eder. İlk ihlalde ``(False, gerekçe)``.
    """
    pts = traj.points
    if not pts:
        return False, "bos yorunge"
    for k, tp in enumerate(pts):
        if not costmap.is_footprint_free(tp.pose, vehicle):
            return False, f"nokta {k} carpisma ({tp.pose.x:.1f},{tp.pose.y:.1f})"
        if k > 0:  # ara örnekleme (segment yönü kafa açısı olarak kullanılır)
            a, b = pts[k - 1].pose, tp.pose
            seg = a.distance_to(b)
            n = int(seg / sample_ds)
            heading = math.atan2(b.y - a.y, b.x - a.x)
            for s in range(1, n):
                t = s / n
                mid = Pose2D(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), heading)
                if not costmap.is_footprint_free(mid, vehicle):
                    return False, f"segment {k-1}-{k} carpisma"
    return True, "ok"


class PlannerPipeline:
    """HFSM + Hybrid A* + TEB'i tek döngüde birleştiren orkestratör."""

    def __init__(
        self,
        mission: MissionManager,
        *,
        map_config: Optional[MapConfig] = None,
        decision_config: Optional[DecisionConfig] = None,
        astar_config: Optional[HybridAStarConfig] = None,
        teb_config: Optional[TEBConfig] = None,
        vehicle: VehicleParams = BEE1,
    ):
        self.mission = mission
        self.hfsm = HFSM(mission, decision_config, vehicle)
        self.vehicle = vehicle
        self.map_cfg = map_config or MapConfig()
        self.astar_cfg = astar_config or HybridAStarConfig()
        # Park için aynı planlayıcı; geri vites aktif ve kafa açısı toleransı gevşek
        # (iskelet park hassasiyeti — dar manevra alanında ulaşılabilirliği artırır).
        self.park_astar_cfg = replace(
            self.astar_cfg, allow_reverse=True, goal_yaw_tol=math.radians(40), goal_pos_tol=0.8)
        self.teb = TEB(teb_config, vehicle)
        self._global_path: Optional[List[Pose2D]] = None
        self._planned_goal_key: Optional[Tuple[float, float, float]] = None

    # ------------------------------------------------------------------ #
    # Ana döngü
    # ------------------------------------------------------------------ #
    def update(self, inp: PlanningInput) -> PlanningOutput:
        decision = self.hfsm.update(inp)
        action = decision.planner_action

        # Hareketsiz durumlar (INIT/READY/COMPLETE/E-STOP)
        if action is PlannerAction.IDLE:
            return self._passive(decision, "hareketsiz durum")

        # HOLD: araç olduğu yerde durur (kırmızı ışık / yolcu / sakınılamaz engel)
        if action is PlannerAction.HOLD:
            return self._hold(decision, inp, "HOLD")

        # Hedef pozunu ve planlayıcı yapılandırmasını davranışa göre seç.
        if action is PlannerAction.PARK:
            # Park Yasaktır levhası aktifse park manevrası başlatılmaz.
            if decision.constraints.no_parking:
                return self._hold(decision, inp, "park yasak levhasi -> HOLD")
            # Park hedefi algı ekibinden gelir; henüz yoksa bekle.
            if inp.park_target is None:
                return self._hold(decision, inp, "park_target bekleniyor -> HOLD")
            goal_pose = inp.park_target
            astar_cfg = self.park_astar_cfg          # geri vites aktif
        else:
            goal = self.mission.current()
            if goal is None:
                return self._passive(decision, "hedef yok")
            goal_pose = goal.pose
            astar_cfg = self.astar_cfg

        # Sürüş/park: Tek Dünya costmap + global rota + TEB + doğrulama
        costmap = self._build_costmap(inp)
        force_replan = action is PlannerAction.REPLAN_GLOBAL
        traj, valid, reason = self._plan_and_validate(
            inp, goal_pose, astar_cfg, costmap, decision.target_v, force_replan)

        if not valid:
            # Politika: tek seferlik global yeniden planlama dene, sonra HOLD
            traj, valid, reason = self._plan_and_validate(
                inp, goal_pose, astar_cfg, costmap, decision.target_v, force_replan=True)
            if not valid:
                return self._hold(decision, inp, f"gecersiz -> HOLD ({reason})")

        return PlanningOutput(
            trajectory=traj,
            target_v=decision.target_v,
            behavior_state=decision.sub_state,
            planner_action=action,
            valid=True,
            door_open=decision.door_open,
            turn_signal=decision.turn_signal,
            constraints=decision.constraints,
            info=decision.info,
        )

    # ------------------------------------------------------------------ #
    # Planlama + doğrulama
    # ------------------------------------------------------------------ #
    def _plan_and_validate(self, inp, goal_pose, astar_cfg, costmap, target_v, force_replan):
        """Global rotayı (gerekirse) planla, TEB üret, doğrula."""
        key = (round(goal_pose.x, 2), round(goal_pose.y, 2), round(goal_pose.yaw, 3))
        need_replan = (
            force_replan
            or self._global_path is None
            or self._planned_goal_key != key
        )
        if need_replan:
            planner = HybridAStar(costmap, astar_cfg, self.vehicle)
            self._global_path = planner.plan(inp.vehicle_state.pose, goal_pose)
            self._planned_goal_key = key

        if self._global_path is None:
            return Trajectory(), False, "global rota yok"

        traj = self.teb.plan(self._global_path, inp.vehicle_state, inp.obstacles,
                             target_v, costmap=costmap)
        valid, reason = validate_trajectory(traj, costmap, self.vehicle)
        return traj, valid, reason

    # ------------------------------------------------------------------ #
    # Costmap kurulumu (Tek Dünya)
    # ------------------------------------------------------------------ #
    def _build_costmap(self, inp: PlanningInput) -> Costmap:
        cm = Costmap(
            self.map_cfg.resolution, self.map_cfg.width_m, self.map_cfg.height_m,
            self.map_cfg.origin, self.vehicle,
        )
        # Şerit koridoru (varsa) — şerit dışını kapat
        if inp.lane is not None and len(inp.lane.centerline) >= 2:
            half = min(inp.lane.left_offset, inp.lane.right_offset)
            if half <= 0:
                half = self.map_cfg.lane_half_width
            cm.restrict_to_corridor(inp.lane.centerline, half)
        # Statik + dinamik engeller (anlık konum) aynı ızgaraya
        cm.add_obstacles(inp.obstacles)
        cm.compute_distance_field()
        return cm

    # ------------------------------------------------------------------ #
    # Çıktı yardımcıları
    # ------------------------------------------------------------------ #
    def _passive(self, decision: BehaviorDecision, info: str) -> PlanningOutput:
        return PlanningOutput(
            trajectory=Trajectory(), target_v=0.0,
            behavior_state=decision.sub_state, planner_action=decision.planner_action,
            valid=True, door_open=decision.door_open, turn_signal=decision.turn_signal,
            constraints=decision.constraints, info=info,
        )

    def _hold(self, decision: BehaviorDecision, inp: PlanningInput, info: str) -> PlanningOutput:
        # Tek noktalı duruş yörüngesi (araç olduğu yerde durur)
        traj = Trajectory()
        traj.points.append(TrajectoryPoint(pose=inp.vehicle_state.pose, v=0.0, t=0.0))
        return PlanningOutput(
            trajectory=traj, target_v=0.0,
            behavior_state=decision.sub_state, planner_action=PlannerAction.HOLD,
            valid=True, door_open=decision.door_open, turn_signal=decision.turn_signal,
            constraints=decision.constraints, info=info,
        )
