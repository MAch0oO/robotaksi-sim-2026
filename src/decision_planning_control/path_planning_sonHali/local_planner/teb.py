"""Timed-Elastic-Band (TEB) — yerel planlayıcı.

Hybrid A*'tan gelen geometrik küresel rotayı girdi alır ve iki aşamada
kontrol ekibine teslim edilecek zaman-parametreli bir ``Trajectory`` üretir:

  1. Elastik band deformasyonu: rota üzerindeki ara noktalar, dinamik/statik
     engellerden iten ve komşularına doğru çeken (düzgünleştiren) kuvvetlerle
     iteratif olarak kaydırılır. Uçlar (araç pozu, hedef) sabittir.
  2. Hız profili: eğrilik, hız ve ivme/yavaşlama kısıtlarına uyan v(s) profili
     ileri/geri geçişle hesaplanır; ardından zaman damgaları çıkarılır.

NOT (sadeleştirme): Gerçek TEB, poz + zaman aralıklarını bir hiper-graf üzerinde
ortak optimize eder (g2o). Buradaki sürüm kuvvet-tabanlı band deformasyonu +
analitik hız profilidir — TEB'in davranışını verir, tam optimizasyon değildir.
TODO(ileride): zaman aralıklarını da değişken yapan en-küçük-kareler optimizasyonu.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from ..common.geometry import angle_diff, normalize_angle
from ..common.types import Obstacle, Pose2D, Trajectory, TrajectoryPoint, VehicleState
from ..common.vehicle_params import BEE1, VehicleParams


@dataclass(frozen=True)
class TEBConfig:
    """TEB parametreleri."""

    resample_spacing: float = 0.5      # band noktaları arası mesafe (m)
    local_horizon: float = 15.0        # araçtan ileri bakılan band uzunluğu (m)
    iterations: int = 30               # band deformasyon iterasyon sayısı
    obstacle_influence: float = 3.0    # engelin band'i ittiği etki yarıçapı (m)
    weight_obstacle: float = 0.6       # engel itme kuvveti adım katsayısı (costmap yoksa)
    weight_smooth: float = 0.2         # düzgünleştirme (elastiklik) katsayısı
    max_lateral_accel: float = 2.0     # eğrilik hız sınırı için yanal ivme (m/s^2)
    stop_at_end: bool = False          # band sonunda hız 0'a insin mi (park/dur)
    # --- Tur E: costmap sınır kuvveti + clamp ---
    weight_boundary: float = 0.8       # costmap gradyan (sınır/engel) itme katsayısı
    d_safe: float = 1.2                # bu mesafeden yakın OCCUPIED için itme devreye girer (m)
    max_step: float = 0.3              # iterasyon başına nokta yer değiştirme tavanı (m) — clamp


class TEB:
    """Zaman-parametreli elastik band yerel planlayıcı."""

    def __init__(self, config: Optional[TEBConfig] = None, vehicle: VehicleParams = BEE1):
        self.config = config or TEBConfig()
        self.vehicle = vehicle

    # ------------------------------------------------------------------ #
    # Ana giriş
    # ------------------------------------------------------------------ #
    def plan(
        self,
        global_path: Sequence[Pose2D],
        vehicle_state: VehicleState,
        obstacles: Sequence[Obstacle],
        target_v: float,
        costmap=None,
    ) -> Trajectory:
        """Global rotayı yerel, engelden kaçan, zamanlı yörüngeye çevirir.

        Args:
            global_path: Hybrid A* çıktısı (sürülebilir geometrik rota).
            vehicle_state: anlık araç durumu (band başlangıcı + başlangıç hızı).
            obstacles: o anki engeller (statik + dinamik).
            target_v: HFSM'in izin verdiği üst hız (m/s); 0 ise araç durur.
            costmap: verilirse band, costmap mesafe alanının gradyanıyla şerit/duvar
                içinde tutulur ("Tek Dünya" sınır kuvveti). None ise engel-listesi
                itmesine düşülür (Tur D davranışı, geriye uyum).

        Returns:
            ``Trajectory`` — boş olabilir (geçerli band kurulamazsa).
        """
        band = self._build_band(global_path, vehicle_state.pose)
        if len(band) < 2:
            return Trajectory()

        # Hedef hız sıfırsa (HOLD): sadece duruş yörüngesi döndür.
        v_cap = min(target_v, self.vehicle.max_speed)
        if v_cap <= 1e-3:
            return self._standstill(band[0], vehicle_state.v)

        # Kaynak yönelimler (aracın gerçek kafa açısı) — yön tespiti için deformasyondan önce sakla.
        src_yaw = [p.yaw for p in band]

        self._deform(band, obstacles, costmap)
        self._recompute_headings(band)  # band[i].yaw = hareket yönü (travel direction)

        # Geri-vites tespiti: hareket yönü ile aracın gerçek yönelimi ~180° terssa geri.
        directions = self._infer_directions(band, src_yaw)
        for i, d in enumerate(directions):
            if d < 0:
                # Yayınlanan poz, aracın GERÇEK yönelimini göstermeli (hareket yönü değil).
                band[i].yaw = normalize_angle(band[i].yaw + math.pi)

        speeds = self._velocity_profile(band, abs(vehicle_state.v), v_cap)
        signed = [s * d for s, d in zip(speeds, directions)]  # geri segmentler negatif hız
        return self._timestamp(band, signed)

    @staticmethod
    def _infer_directions(band: List[Pose2D], src_yaw: List[float]) -> List[float]:
        """Her nokta için sürüş yönü: +1 ileri, -1 geri.

        Hareket yönü (recompute sonrası band[i].yaw) ile noktanın kaynak yönelimi
        (aracın gerçek kafa açısı) arasındaki açı 90°'den büyükse araç o segmentte
        geri gidiyordur. Uçlar komşularını takip eder.
        """
        n = len(band)
        dirs = [1.0] * n
        for i in range(n):
            travel = band[i].yaw
            if abs(angle_diff(travel, src_yaw[i])) > math.pi / 2:
                dirs[i] = -1.0
        return dirs

    # ------------------------------------------------------------------ #
    # 1) Band oluşturma
    # ------------------------------------------------------------------ #
    def _build_band(self, path: Sequence[Pose2D], ego: Pose2D) -> List[Pose2D]:
        """Araca en yakın noktadan başlayıp ufka kadar rotayı eşit aralıkla örnekler."""
        if len(path) < 2:
            return list(path)
        start_idx = self._nearest_index(path, ego)
        # Ufuk içindeki ham noktaları topla
        raw = [ego]
        acc = 0.0
        prev = path[start_idx]
        for p in path[start_idx + 1:]:
            acc += math.hypot(p.x - prev.x, p.y - prev.y)
            raw.append(p)
            prev = p
            if acc >= self.config.local_horizon:
                break
        return self._resample(raw, self.config.resample_spacing)

    @staticmethod
    def _nearest_index(path: Sequence[Pose2D], ego: Pose2D) -> int:
        best_i, best_d = 0, float("inf")
        for i, p in enumerate(path):
            d = (p.x - ego.x) ** 2 + (p.y - ego.y) ** 2
            if d < best_d:
                best_i, best_d = i, d
        return best_i

    @staticmethod
    def _resample(pts: Sequence[Pose2D], spacing: float) -> List[Pose2D]:
        """Poz dizisini sabit ``spacing`` aralığında yeniden örnekler.

        Ara noktalara kaynak segmentin kafa açısı (aracın GERÇEK yönelimi) taşınır;
        bu, geri-vites segmentlerinde yön (işaret) tespiti için gereklidir.
        """
        if len(pts) < 2:
            return list(pts)
        out = [Pose2D(pts[0].x, pts[0].y, pts[0].yaw)]
        carry = 0.0
        for a, b in zip(pts, pts[1:]):
            seg = math.hypot(b.x - a.x, b.y - a.y)
            if seg < 1e-9:
                continue
            dirx, diry = (b.x - a.x) / seg, (b.y - a.y) / seg
            dist = carry
            while dist + spacing <= seg:
                dist += spacing
                out.append(Pose2D(a.x + dirx * dist, a.y + diry * dist, a.yaw))
            carry = (dist + spacing) - seg
        # Son noktayı koru (hedef sabit kalmalı)
        last = pts[-1]
        if math.hypot(out[-1].x - last.x, out[-1].y - last.y) > 1e-6:
            out.append(Pose2D(last.x, last.y, last.yaw))
        return out

    # ------------------------------------------------------------------ #
    # 2) Band deformasyonu (elastik band + engel itmesi)
    # ------------------------------------------------------------------ #
    def _deform(self, band: List[Pose2D], obstacles: Sequence[Obstacle], costmap=None) -> None:
        """Ara noktaları engel/sınırdan iter ve düzgünleştirir (uçlar sabit).

        Costmap verilirse engel itmesi **mesafe alanı gradyanına** devredilir
        (sınır + engel tek mekanizma — "Tek Dünya"). Her noktanın iterasyon başına
        yer değiştirmesi ``max_step`` ile clamp'lenir (self-intersection / aşırı
        deformasyon önlenir).
        """
        cfg = self.config
        circles = [_as_circle(o) for o in obstacles] if costmap is None else []
        for _ in range(cfg.iterations):
            for i in range(1, len(band) - 1):  # uçlar sabit
                p = band[i]
                # a) Düzgünleştirme: komşuların orta noktasına çekim
                mid_x = 0.5 * (band[i - 1].x + band[i + 1].x)
                mid_y = 0.5 * (band[i - 1].y + band[i + 1].y)
                fx = cfg.weight_smooth * (mid_x - p.x)
                fy = cfg.weight_smooth * (mid_y - p.y)

                if costmap is not None:
                    # b1) Sınır/engel kuvveti: costmap mesafe alanı gradyanı (boşluğa doğru)
                    d = costmap.distance_at(p.x, p.y)
                    if d < cfg.d_safe:
                        gx, gy = costmap.gradient_at(p.x, p.y)
                        gnorm = math.hypot(gx, gy)
                        if gnorm > 1e-6:
                            push = cfg.weight_boundary * (cfg.d_safe - d) / cfg.d_safe
                            fx += push * gx / gnorm
                            fy += push * gy / gnorm
                else:
                    # b2) Costmap yoksa: engel-listesi itmesi (Tur D davranışı)
                    for cx, cy, r in circles:
                        dx, dy = p.x - cx, p.y - cy
                        dist = math.hypot(dx, dy)
                        influence = r + cfg.obstacle_influence
                        if 1e-6 < dist < influence:
                            push = cfg.weight_obstacle * (influence - dist) / influence
                            fx += push * dx / dist
                            fy += push * dy / dist

                # c) Clamp: iterasyon başına yer değiştirmeyi sınırla
                step = math.hypot(fx, fy)
                if step > cfg.max_step:
                    scale = cfg.max_step / step
                    fx *= scale
                    fy *= scale
                band[i] = Pose2D(p.x + fx, p.y + fy, 0.0)

    @staticmethod
    def _recompute_headings(band: List[Pose2D]) -> None:
        """Deformasyon sonrası her noktanın kafa açısını komşulardan günceller."""
        n = len(band)
        for i in range(n):
            if i < n - 1:
                ax, ay = band[i].x, band[i].y
                bx, by = band[i + 1].x, band[i + 1].y
            else:
                ax, ay = band[i - 1].x, band[i - 1].y
                bx, by = band[i].x, band[i].y
            band[i].yaw = math.atan2(by - ay, bx - ax)

    # ------------------------------------------------------------------ #
    # 3) Hız profili + zaman damgası
    # ------------------------------------------------------------------ #
    def _velocity_profile(self, band: List[Pose2D], v_start: float, v_cap: float) -> List[float]:
        """Eğrilik + ivme/yavaşlama kısıtlı v(s) profili (ileri/geri geçiş)."""
        n = len(band)
        veh = self.vehicle
        # a) Eğrilik tabanlı üst sınır
        v_limit = [v_cap] * n
        for i in range(1, n - 1):
            kappa = self._curvature(band[i - 1], band[i], band[i + 1])
            if kappa > 1e-4:
                v_curv = math.sqrt(self.config.max_lateral_accel / kappa)
                v_limit[i] = min(v_limit[i], v_curv)

        v = list(v_limit)
        v[0] = min(v[0], max(v_start, 0.0))
        if self.config.stop_at_end:
            v[-1] = 0.0

        # b) İleri geçiş: hızlanma sınırı  v_i^2 <= v_{i-1}^2 + 2*a*ds
        for i in range(1, n):
            ds = band[i - 1].distance_to(band[i])
            v[i] = min(v[i], math.sqrt(v[i - 1] ** 2 + 2 * veh.max_accel * ds))
        # c) Geri geçiş: yavaşlama sınırı  v_i^2 <= v_{i+1}^2 + 2*decel*ds
        for i in range(n - 2, -1, -1):
            ds = band[i].distance_to(band[i + 1])
            v[i] = min(v[i], math.sqrt(v[i + 1] ** 2 + 2 * veh.max_decel * ds))
        return v

    @staticmethod
    def _curvature(a: Pose2D, b: Pose2D, c: Pose2D) -> float:
        """Üç noktadan Menger eğriliği (1/m)."""
        ab = math.hypot(b.x - a.x, b.y - a.y)
        bc = math.hypot(c.x - b.x, c.y - b.y)
        ca = math.hypot(a.x - c.x, a.y - c.y)
        if ab < 1e-6 or bc < 1e-6 or ca < 1e-6:
            return 0.0
        # Üçgen alanının iki katı (işaretsiz)
        area2 = abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
        return 2.0 * area2 / (ab * bc * ca)

    @staticmethod
    def _timestamp(band: List[Pose2D], speeds: List[float]) -> Trajectory:
        """Hız profilinden kümülatif zaman damgalarını üretip Trajectory kurar."""
        traj = Trajectory()
        t = 0.0
        for i, p in enumerate(band):
            if i > 0:
                ds = band[i - 1].distance_to(p)
                # Hız işaretli olabilir (geri vites); zaman için mutlak ortalama kullanılır.
                v_avg = max(0.5 * (abs(speeds[i - 1]) + abs(speeds[i])), 1e-3)
                t += ds / v_avg
            traj.points.append(TrajectoryPoint(pose=p, v=speeds[i], t=t))
        return traj

    @staticmethod
    def _standstill(pose: Pose2D, v_now: float) -> Trajectory:
        """HOLD durumu: tek noktalı, hızı sıfır yörünge (araç olduğu yerde durur)."""
        traj = Trajectory()
        traj.points.append(TrajectoryPoint(pose=pose, v=0.0, t=0.0))
        return traj


# ---------------------------------------------------------------------- #
# Yardımcı
# ---------------------------------------------------------------------- #
def _as_circle(obs: Obstacle) -> Tuple[float, float, float]:
    """Engeli (merkez_x, merkez_y, yarıçap) dairesine indirger."""
    if obs.center is not None:
        return obs.center[0], obs.center[1], max(obs.radius, 0.0)
    if obs.polygon:
        cx = sum(p[0] for p in obs.polygon) / len(obs.polygon)
        cy = sum(p[1] for p in obs.polygon) / len(obs.polygon)
        r = max(math.hypot(p[0] - cx, p[1] - cy) for p in obs.polygon)
        return cx, cy, r
    return 0.0, 0.0, 0.0
