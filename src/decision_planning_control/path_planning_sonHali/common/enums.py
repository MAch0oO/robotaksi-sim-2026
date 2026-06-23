"""Path planning modülünde kullanılan sabit enum tanımları.

Tüm durumlar, görev tipleri ve sensör-kaynaklı kategoriler burada tek noktadan
tanımlanır. Böylece HFSM, planlayıcılar ve testler aynı sözlüğü paylaşır.
"""

from __future__ import annotations

from enum import Enum, IntEnum, auto


class TopState(Enum):
    """HFSM üst seviye operasyon modları.

    Sistemin genel modunu temsil eder. Akış:
        INIT -> READY -> MISSION_EXECUTION -> MISSION_COMPLETE
    EMERGENCY_STOP her durumdan tetiklenebilir (en yüksek öncelik).
    """

    INIT = auto()              # parametreler / görev henüz yüklenmedi
    READY = auto()             # görev yüklendi, başlangıç pozu doğrulandı, harekete hazır
    MISSION_EXECUTION = auto() # asıl sürüş; alt durumlar (SubState) aktiftir
    MISSION_COMPLETE = auto()  # tüm waypointler + park tamamlandı
    EMERGENCY_STOP = auto()    # e-stop / watchdog / çarpışma riski


class SubState(Enum):
    """MISSION_EXECUTION içindeki davranış (alt) durumları.

    Öncelik sıralaması (yüksekten düşüğe) ``transitions`` modülünde uygulanır:
        STOP_AND_WAIT > OBSTACLE_AVOIDANCE > PARKING / PASSENGER_OPS > LANE_FOLLOWING
    Aynı anda yalnızca bir alt durum aktiftir.
    """

    LANE_FOLLOWING = auto()       # normal seyir: TEB global rotayı takip eder
    OBSTACLE_AVOIDANCE = auto()   # engelden sakınma: TEB yörüngeyi deforme eder
    STOP_AND_WAIT = auto()        # kırmızı ışık / yaya / sakınılamaz engel -> hedef hız 0
    PASSENGER_OPS = auto()        # durak: kapı açık, yolcu al/bırak bekleme (süre bazlı)
    PASSENGER_DEPARTURE = auto()  # duraktan kalkış: kapı kapalı, sol sinyal (süre bazlı)
    PARKING = auto()              # park_giris sonrası dik park manevrası


class MissionType(Enum):
    """GEOJSON'dan gelen her görev noktasının tipi."""

    START = auto()          # araç başlangıç noktası
    PASSENGER_PICKUP = auto()   # yolcu alma (bindirme)
    PASSENGER_DROPOFF = auto()  # yolcu bırakma (indirme)
    GOAL = auto()           # ara/son hedef noktası (özel işlem yok)
    PARK_ENTRANCE = auto()  # otopark giriş bölgesi (gerçek park yeri değil)


class LightState(Enum):
    """Algı ekibinden gelen trafik ışığı durumu."""

    UNKNOWN = auto()  # ışık görülemiyor / belirsiz
    RED = auto()
    YELLOW = auto()
    GREEN = auto()
    NONE = auto()     # bu senaryoda/turda ışık yok (ör. Tur 1)


class ObstacleClass(Enum):
    """Algı ekibinden gelen engelin kategorisi.

    Algı ekibi engelleri iki sınıfa ayırır (yaya ayrı bir sınıf DEĞİLDİR;
    hareketli ise DYNAMIC olarak gelir):
        STATIC: yol/şeridi kapatan sabit engel (koni, bariyer, kapalı yol).
        DYNAMIC: hareketli engel (araç, yaya vb.); hız vektörü anlamlıdır.
    """

    STATIC = auto()
    DYNAMIC = auto()


class SignType(Enum):
    """Algı ekibinden gelen trafik levhası tipleri (şartname levha listesi).

    Davranışa etkilerine göre gruplanır; eşleme ``decision`` katmanında yapılır.
    """

    NONE = auto()
    PEDESTRIAN_CROSSING = auto()  # yaya geçidi -> yavaşla/dur
    NO_ENTRY = auto()             # girilmez -> dur
    NO_RIGHT_TURN = auto()        # sağa dönülmez
    NO_LEFT_TURN = auto()         # sola dönülmez
    MANDATORY_RIGHT = auto()      # mecburi sağ
    MANDATORY_LEFT = auto()       # mecburi sol
    MANDATORY_AHEAD = auto()      # mecburi ileri
    AHEAD_RIGHT = auto()          # ileriden sağ (delayed turn)
    AHEAD_LEFT = auto()           # ileriden sol (delayed turn)
    KEEP_RIGHT = auto()           # sağdan gidiniz
    KEEP_LEFT = auto()            # soldan gidiniz
    NO_PARKING = auto()           # park yasaktır
    TWO_WAY = auto()              # iki yönlü yol
    TUNNEL = auto()               # tünel
    LANE_MERGE_RIGHT = auto()     # B-50h: sola kapanıyor, sağa birleş
    LANE_MERGE_LEFT = auto()      # B-50ı: sağa kapanıyor, sola birleş


class TurnSignal(IntEnum):
    """Donanım sinyal komutu — araç RC_SignalStatus ile birebir."""

    NONE = 0
    RIGHT = 1
    LEFT = 2
    HAZARD = 3


class PlannerAction(IntEnum):
    """HFSM'in alt planlayıcılara verdiği yüksek seviyeli emir.

    Bu, HFSM (amir) ile planlama hattı arasındaki sözleşmedir. Davranış
    durumu seçildikten sonra hangi planlama eyleminin yapılacağını söyler.
    """

    FOLLOW_GLOBAL = auto()  # TEB aktif, mevcut global rotayı takip et
    AVOID = auto()          # TEB engelden kaçacak şekilde deforme et
    HOLD = auto()           # dur ve bekle (hedef hız 0), planlama dondurulur
    REPLAN_GLOBAL = auto()  # Hybrid A* yeniden çağrılsın (yol tıkalı / park)
    PARK = auto()           # Hybrid A* park moduna geçsin (dik park hedefi)
    IDLE = auto()           # hareket yok (INIT / COMPLETE / E-STOP)
