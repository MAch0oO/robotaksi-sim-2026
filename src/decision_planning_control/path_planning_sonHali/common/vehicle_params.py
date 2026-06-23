"""BEE1 araç parametreleri ve hareket kısıtları.

Değerler resmi "Hazır Araç Bilgilendirme Dökümanı"ndan alınmıştır. Hybrid A*
(Ackermann adım üretimi, ayak izi çarpışma kontrolü) ve TEB (hız/ivme kısıtları)
bu sabitleri ortak kullanır. Tüm birimler SI (metre, saniye, radyan).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass(frozen=True)
class VehicleParams:
    """BEE1'in fiziksel ölçüleri ve kinematik/dinamik kısıtları.

    ``frozen=True``: parametreler çalışma anında değişmez (deterministik plan).
    """

    # --- Boyutlar (m) ---
    wheelbase: float = 1.860        # dingil mesafesi (L) — Ackermann modelinin temeli
    length: float = 2.740           # toplam uzunluk
    width: float = 1.060            # toplam genişlik
    front_overhang: float = 0.410   # ön aks merkezinden ön uca
    rear_overhang: float = 0.470    # arka uzantı

    # --- Direksiyon / dönüş kısıtları ---
    max_steer_angle: float = math.radians(30.0)  # dış teker max 30° (kısıtlayıcı olan)
    # Not: iç teker 32.5°, dış teker 30°. Tek-iz (bicycle) modelde
    # daha güvenli/kısıtlayıcı değer olan 30° kullanılır.
    min_turning_radius: float = 4.10  # duvardan duvara dönüş yarıçapı (m)

    # --- Hız / ivme kısıtları ---
    max_speed: float = 30.0 / 3.6     # 30 km/h limiti -> m/s (~8.33)
    max_accel: float = 2.5            # maksimum hızlanma ivmesi (m/s^2)
    max_decel: float = 6.5            # maksimum yavaşlama ivmesi (m/s^2, pozitif değer)
    max_jerk: float = 2.0             # sarsıntı sınırı (m/s^3) — yumuşak kalkış/duruş

    # --- Çarpışma kontrolü için ayak izi tamponu ---
    safety_margin: float = 0.20       # ayak izine eklenen güvenlik payı (m)

    @property
    def rear_to_front(self) -> float:
        """Arka aks (base_link) referansından aracın ön ucuna mesafe (m)."""
        return self.wheelbase + self.front_overhang

    @property
    def footprint_radius(self) -> float:
        """Aracı çevreleyen kaba dairesel yarıçap (hızlı çarpışma ön elemesi).

        Köşegenin yarısı + güvenlik payı. Hassas kontrol için poligon ayak izi
        ayrıca kullanılır; bu yalnızca ucuz bir ön filtredir.
        """
        return 0.5 * math.hypot(self.length, self.width) + self.safety_margin

    def max_curvature(self) -> float:
        """Minimum dönüş yarıçapına karşılık gelen maksimum eğrilik (1/m)."""
        return 1.0 / self.min_turning_radius

    def footprint_circles(self, num_circles: int = 2) -> List[Tuple[float, float]]:
        """Aracı kaplayan daire dizisi: (boylamsal_ofset, yarıçap) listesi.

        Referans arka aks (planlama/base_link). Gövde, ön uç (wheelbase+front_overhang)
        ile arka uç (-rear_overhang) arasında uzanır; ``num_circles`` daire bu eksen
        boyunca eşit aralıkla yerleştirilir. Her daire, kapladığı alt-dikdörtgeni
        (boy = L/n, en = W) çevreler: yarıçap = √((L/n / 2)² + (W/2)²) + güvenlik payı.

        Daire sayısı arttıkça yarıçap küçülür (dar boşluklarda daha az temkinli);
        varsayılan iki-daire, disk modelinden belirgin daha az false-positive verir.
        """
        front_end = self.wheelbase + self.front_overhang
        rear_end = -self.rear_overhang
        body_center = 0.5 * (front_end + rear_end)
        spacing = self.length / num_circles
        radius = math.hypot(spacing / 2.0, self.width / 2.0) + self.safety_margin
        offsets = [
            body_center + (k - (num_circles - 1) / 2.0) * spacing
            for k in range(num_circles)
        ]
        return [(off, radius) for off in offsets]


# Modül genelinde paylaşılan tekil örnek (gerekirse override edilebilir).
BEE1 = VehicleParams()
