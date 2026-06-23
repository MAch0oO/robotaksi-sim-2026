"""Dünya okuma + öncelikli alt durum seçimi (HFSM geçiş mantığı).

İki sorumluluk:
  1. ``build_world_view``: ham ``PlanningInput`` + görev ilerlemesini, karar için
     anlamlı boolean "olgulara" indirger (kırmızı ışık önde mi, engel koridorda mı,
     waypoint'e varıldı mı ...). Geometri burada bir kez yapılır.
  2. ``select_substate``: bu olgulardan, sabit bir öncelik merdiveniyle aktif alt
     durumu seçer. Memoryless ve deterministik — aynı olgular hep aynı durumu verir.

Engeller yalnızca STATIC / DYNAMIC olarak ele alınır (algı ekibi sözleşmesi).

Not: Engelin "sakınılabilir" olup olmadığı tam olarak Tur C'de costmap + Hybrid A*
ile belirlenecektir. Burada şerit bilgisi varsa yanal boşluğa, yoksa yapılandırılabilir
bir varsayıma göre pragmatik bir ön karar verilir (TODO: costmap entegrasyonu).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ..common.enums import LightState, MissionType, ObstacleClass, SignType, SubState, TurnSignal
from ..common.types import Constraints, Obstacle, Pose2D
from ..io.inputs import PlanningInput


@dataclass(frozen=True)
class DecisionConfig:
    """HFSM karar eşikleri. Tümü ayarlanabilir (config/params.yaml ile beslenebilir)."""

    lookahead_distance: float = 15.0     # önde engel taradığımız boylamsal pencere (m)
    corridor_half_width: float = 1.5     # araç koridorunun yarı genişliği (m)
    stop_distance: float = 5.0           # bu mesafeden yakın dinamik engel -> dur (m)
    light_trigger_distance: float = 12.0 # kırmızı ışığa bu mesafede tepki ver (m)
    passenger_dwell_time: float = 18.0   # durakta kapı açık bekleme süresi (s) — şartname 15-20s
    departure_signal_time: float = 5.0   # kalkışta sol sinyal süresi (s)
    assume_static_avoidable: bool = True # şerit bilgisi yoksa statik engel sakınılabilir say
    park_tolerance: float = 0.5          # park_target'a varış toleransı (m) — park tamam sayılır
    # --- Levha (sign) parametreleri ---
    sign_read_distance: float = 10.0     # levha bu mesafeden yakınsa işlenir (m)
    sign_effect_time: float = 5.0        # levha etkisi görüş kaybından sonra bu süre korunur (s)
    slow_speed: float = 20.0 / 3.6       # genel yavaşlama hızı (m/s)
    pedestrian_speed: float = 15.0 / 3.6 # yaya geçidi yaklaşım hızı (m/s)
    tunnel_speed: float = 15.0 / 3.6     # tünel içi hız (m/s)


@dataclass
class WorldView:
    """Bir kontrol döngüsü için karar-ilişkili olgular (tüm geometri çözülmüş)."""

    emergency: bool = False
    red_light_ahead: bool = False
    static_block_unavoidable: bool = False  # önde sakınılamayan statik engel
    obstacle_avoidable: bool = False        # önde sakınılabilir engel (statik/dinamik-orta)
    dynamic_block_close: bool = False        # çok yakın dinamik engel -> dur
    at_park: bool = False                    # park giriş bölgesine varıldı
    at_passenger_stop: bool = False          # yolcu noktasına varıldı, servis bekliyor
    mission_finished: bool = False

    @property
    def must_stop(self) -> bool:
        """Aracın tamamen durması gereken koşullar (en yüksek davranış önceliği)."""
        return self.red_light_ahead or self.dynamic_block_close or self.static_block_unavoidable


def _obstacle_center_radius(obs: Obstacle) -> Tuple[float, float, float]:
    """Engeli (merkez_x, merkez_y, yarıçap) dairesel yaklaşımına indirger.

    Poligon verilmişse centroid + en uzak köşe mesafesi kullanılır.
    """
    if obs.center is not None:
        return obs.center[0], obs.center[1], max(obs.radius, 0.0)
    if obs.polygon:
        cx = sum(p[0] for p in obs.polygon) / len(obs.polygon)
        cy = sum(p[1] for p in obs.polygon) / len(obs.polygon)
        r = max(math.hypot(p[0] - cx, p[1] - cy) for p in obs.polygon)
        return cx, cy, r
    # Geometri yoksa nokta engel kabul edilir.
    return 0.0, 0.0, 0.0


def _to_vehicle_frame(px: float, py: float, ego: Pose2D) -> Tuple[float, float]:
    """Bir noktayı araç gövde frame'ine taşır -> (boylamsal_ileri, yanal_sol)."""
    dx, dy = px - ego.x, py - ego.y
    longitudinal = dx * math.cos(ego.yaw) + dy * math.sin(ego.yaw)
    lateral = -dx * math.sin(ego.yaw) + dy * math.cos(ego.yaw)
    return longitudinal, lateral


def build_world_view(
    inp: PlanningInput,
    *,
    at_park: bool,
    at_passenger_stop: bool,
    mission_finished: bool,
    config: DecisionConfig,
) -> WorldView:
    """Ham girdi + görev durumundan karar olgularını hesaplar.

    Görev ilerlemesine dayalı bayraklar (``at_park`` vb.) HFSM/MissionManager
    tarafından sağlanır; çevresel bayraklar (ışık, engel) burada hesaplanır.
    """
    wv = WorldView(
        emergency=inp.emergency,
        at_park=at_park,
        at_passenger_stop=at_passenger_stop,
        mission_finished=mission_finished,
    )

    ego = inp.vehicle_state.pose

    # --- Trafik ışığı: önde kırmızı mı? ---
    light = inp.traffic_light
    if light.state is LightState.RED:
        if light.position is None:
            wv.red_light_ahead = True  # konum yoksa temkinli davran
        else:
            lon, lat = _to_vehicle_frame(light.position[0], light.position[1], ego)
            wv.red_light_ahead = 0.0 <= lon <= config.light_trigger_distance

    # --- Engeller: koridordaki en kritik durumu belirle ---
    for obs in inp.obstacles:
        cx, cy, r = _obstacle_center_radius(obs)
        lon, lat = _to_vehicle_frame(cx, cy, ego)
        # Sadece önümüzde ve koridor genişliği içinde olanlar ilgilendirir.
        in_front = 0.0 < lon <= config.lookahead_distance
        in_corridor = abs(lat) <= (config.corridor_half_width + r)
        if not (in_front and in_corridor):
            continue

        if obs.is_dynamic:
            if lon <= config.stop_distance:
                wv.dynamic_block_close = True   # çok yakın -> dur ve bekle
            else:
                wv.obstacle_avoidable = True    # orta mesafe -> sakınma dene
        else:  # STATIC
            if _static_avoidable(lat, r, inp, config):
                wv.obstacle_avoidable = True
            else:
                wv.static_block_unavoidable = True

    return wv


def _static_avoidable(lateral: float, radius: float, inp: PlanningInput, config: DecisionConfig) -> bool:
    """Statik engelin yanından geçilebilir mi (basit ön karar).

    Şerit bilgisi varsa engeli atlatmak için gereken yanal kayma şerit içinde
    kalıyor mu bakılır; yoksa yapılandırılabilir varsayıma düşülür.
    TODO(Tur C): costmap + Hybrid A* ile gerçek sürülebilirlik kontrolü.
    """
    needed_clearance = abs(lateral) - radius  # engel kenarına yanal boşluk
    if inp.lane is not None:
        # Engelin solundan/sağından geçecek alan var mı?
        room_left = inp.lane.left_offset - (lateral + radius)
        room_right = (lateral - radius) - (-inp.lane.right_offset)
        return room_left >= config.corridor_half_width or room_right >= config.corridor_half_width
    if needed_clearance >= config.corridor_half_width:
        return True  # engel zaten koridor kenarında, hafif kayma yeter
    return config.assume_static_avoidable


def select_substate(world: WorldView) -> SubState:
    """Öncelik merdiveni: olgulardan tek aktif alt durumu seçer.

    Öncelik (yüksek -> düşük):
        STOP_AND_WAIT > OBSTACLE_AVOIDANCE > PARKING > PASSENGER_OPS > LANE_FOLLOWING
    (EMERGENCY üst seviye durumdur, burada değil HFSM'de ele alınır.)
    """
    if world.must_stop:
        return SubState.STOP_AND_WAIT
    if world.obstacle_avoidable:
        return SubState.OBSTACLE_AVOIDANCE
    if world.at_park:
        return SubState.PARKING
    if world.at_passenger_stop:
        return SubState.PASSENGER_OPS
    return SubState.LANE_FOLLOWING


# ---------------------------------------------------------------------- #
# Levha (trafik işareti) -> kısıt + hız + dur eşlemesi
# ---------------------------------------------------------------------- #
def apply_sign(sign_type: SignType, c: Constraints, config: DecisionConfig) -> Tuple[float, bool]:
    """Bir levhayı ``Constraints`` üzerine işler; (hız_sınırı, dur_zorla) döndürür.

    Uygulanabilen kısıtlar (no_entry->dur, no_parking, hız) gerçek etkiye dönüşür;
    yön/şerit kısıtları (no_*_turn, mandatory, lane_merge, tunnel, two_way) bildirim
    olarak ``c`` üzerine yazılır (telemetri + sinyal). Şerit topolojisi gelince tam
    geometrik uygulama eklenir.
    """
    inf = float("inf")
    st = SignType
    if sign_type is st.PEDESTRIAN_CROSSING:
        return config.pedestrian_speed, False
    if sign_type is st.NO_ENTRY:
        c.no_entry = True
        return inf, True
    if sign_type is st.NO_RIGHT_TURN:
        c.no_right_turn = True
        return config.slow_speed, False
    if sign_type is st.NO_LEFT_TURN:
        c.no_left_turn = True
        return config.slow_speed, False
    if sign_type is st.MANDATORY_RIGHT:
        c.mandatory_directions = ["SAG"]
        return config.slow_speed, False
    if sign_type is st.MANDATORY_LEFT:
        c.mandatory_directions = ["SOL"]
        return config.slow_speed, False
    if sign_type is st.MANDATORY_AHEAD:
        c.mandatory_directions = ["ILERI"]
        return inf, False
    if sign_type is st.AHEAD_RIGHT:
        c.delayed_turn = "SAG"
        return inf, False
    if sign_type is st.AHEAD_LEFT:
        c.delayed_turn = "SOL"
        return inf, False
    if sign_type is st.KEEP_RIGHT:
        c.no_left_turn = True
        c.mandatory_directions = ["SAG"]
        return config.slow_speed, False
    if sign_type is st.KEEP_LEFT:
        c.no_right_turn = True
        c.mandatory_directions = ["SOL"]
        return config.slow_speed, False
    if sign_type is st.NO_PARKING:
        c.no_parking = True
        return config.slow_speed, False
    if sign_type is st.TWO_WAY:
        c.two_way = True
        return config.slow_speed, False
    if sign_type is st.TUNNEL:
        c.tunnel_active = True
        return config.tunnel_speed, False
    if sign_type is st.LANE_MERGE_RIGHT:
        c.lane_merge = "SAGA"
        return config.slow_speed, False
    if sign_type is st.LANE_MERGE_LEFT:
        c.lane_merge = "SOLA"
        return config.slow_speed, False
    return inf, False


def signal_from_constraints(c: Constraints) -> TurnSignal:
    """Kısıtlardan donanım sinyal komutu türetir (mecburi yön / şerit birleşme / delayed)."""
    if c.lane_merge == "SAGA":
        return TurnSignal.RIGHT
    if c.lane_merge == "SOLA":
        return TurnSignal.LEFT
    if "SAG" in c.mandatory_directions or c.delayed_turn == "SAG":
        return TurnSignal.RIGHT
    if "SOL" in c.mandatory_directions or c.delayed_turn == "SOL":
        return TurnSignal.LEFT
    return TurnSignal.NONE
