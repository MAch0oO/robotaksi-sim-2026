"""Planlama hattının ürettiği çıktı paketi (kontrol ekibine teslim edilir).

Asıl ürün ``trajectory``'dir. ``behavior_state`` ve ``planner_action`` alanları
loglama / hata ayıklama / kontrol ekibinin bağlam farkındalığı içindir.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from ..common.enums import PlannerAction, SubState, TurnSignal
from ..common.types import Constraints, Trajectory


@dataclass
class PlanningOutput:
    """Bir kontrol döngüsünün planlama sonucu.

    Attributes:
        trajectory: takip edilecek zaman-parametreli yörünge (boş olabilir).
        target_v: bu döngüdeki üst hız sınırı (m/s); HOLD durumunda 0.
        behavior_state: o an aktif HFSM alt durumu (bağlam/log).
        planner_action: HFSM'in verdiği yüksek seviyeli planlama emri.
        valid: True ise yörünge kullanılabilir; False ise kontrol güvenli
            duruşa geçmeli (planlama başarısız / yol kapalı).
        door_open: durakta kapı açma bildirimi (OMUX autonomous_door_open).
        turn_signal: sinyal komutu (OMUX rc_signalstatus: 0/1/2/3).
        constraints: levha/kural kaynaklı kısıtlar (telemetri + downstream).
        info: insan-okur açıklama (log/teşhis).
    """

    trajectory: Trajectory = field(default_factory=Trajectory)
    target_v: float = 0.0
    behavior_state: Optional[SubState] = None
    planner_action: PlannerAction = PlannerAction.IDLE
    valid: bool = False
    door_open: bool = False
    turn_signal: TurnSignal = TurnSignal.NONE
    constraints: Constraints = field(default_factory=Constraints)
    info: str = ""
