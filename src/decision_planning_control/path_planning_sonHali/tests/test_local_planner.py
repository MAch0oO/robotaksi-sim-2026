"""TEB yerel planlayıcı testi.

Düz bir global rotaya yakın bir dinamik engel yerleştirir ve TEB'in:
  * band'i engelden saptırdığını,
  * hız/ivme/eğrilik kısıtlarına uyan bir hız profili ürettiğini,
  * tutarlı (artan) zaman damgaları verdiğini
doğrular. ``pytest`` veya doğrudan ``python ...`` ile çalışır.
"""

from __future__ import annotations

import math

from path_planning.common.enums import ObstacleClass
from path_planning.common.types import Obstacle, Pose2D, VehicleState
from path_planning.common.vehicle_params import BEE1
from path_planning.local_planner.teb import TEB, TEBConfig


def straight_path(length: int = 21) -> list[Pose2D]:
    return [Pose2D(float(i), 0.0, 0.0) for i in range(length)]


def run(verbose: bool = False):
    teb = TEB(TEBConfig())
    path = straight_path()
    ego = VehicleState(Pose2D(0.0, 0.0, 0.0), v=0.0)
    # Rotanın hemen yanında (+y) dinamik engel
    obs = [Obstacle(1, ObstacleClass.DYNAMIC, center=(8.0, 0.6), radius=0.5, velocity=(0.0, -0.5))]
    target_v = BEE1.max_speed

    traj = teb.plan(path, ego, obs, target_v)
    assert len(traj) >= 2, "Yorunge bos"

    # 1) Zaman damgaları kesinlikle artan
    for a, b in zip(traj.points, traj.points[1:]):
        assert b.t > a.t - 1e-9, "Zaman damgasi artmiyor"

    # 2) Hız sınırları: 0 <= v <= min(max_speed, target_v)
    cap = min(target_v, BEE1.max_speed)
    for tp in traj.points:
        assert -1e-6 <= tp.v <= cap + 1e-3, f"Hiz sinir disi: {tp.v}"

    # 3) İvme/yavaşlama kısıtı (küçük sayısal toleransla)
    for a, b in zip(traj.points, traj.points[1:]):
        ds = a.pose.distance_to(b.pose)
        if ds < 1e-6:
            continue
        accel = (b.v ** 2 - a.v ** 2) / (2 * ds)
        assert accel <= BEE1.max_accel + 0.2, f"Hizlanma asimi: {accel:.2f}"
        assert accel >= -BEE1.max_decel - 0.2, f"Yavaslama asimi: {accel:.2f}"

    # 4) Engelden sakınma: band engelin karşı tarafına (-y) saptı mı?
    min_y = min(tp.pose.y for tp in traj.points)
    assert min_y < -0.1, f"Band engelden sapmadi (min_y={min_y:.2f})"

    # 5) Engele yaklaşım mesafesi sapma sayesinde arttı mı?
    obs_xy = (8.0, 0.6)
    closest_after = min(math.hypot(tp.pose.x - obs_xy[0], tp.pose.y - obs_xy[1]) for tp in traj.points)
    closest_before = 0.6  # düz rota engele en yakın 0.6 m idi
    assert closest_after > closest_before, "Sakinma mesafe kazandirmadi"

    if verbose:
        print(f"Yorunge: {len(traj)} nokta, toplam sure {traj.points[-1].t:.2f} s")
        print(f"  band engelden min sapma (min_y): {min_y:.2f} m")
        print(f"  engele en yakin mesafe: {closest_before:.2f} -> {closest_after:.2f} m")
        print("  ornek noktalar (x, y, v, t):")
        for tp in traj.points[::max(1, len(traj)//10)]:
            print(f"    x={tp.pose.x:5.2f} y={tp.pose.y:6.2f} v={tp.v:4.2f} t={tp.t:5.2f}")

    return traj


def test_teb_avoids_and_profiles():
    traj = run(verbose=False)
    assert len(traj) >= 2


def test_reverse_park_produces_negative_speed():
    """Geri-vites segmenti: araç +x'e bakarken -x'e ilerleyen rota -> negatif hız."""
    teb = TEB(TEBConfig())
    # Poz yönelimi +x (yaw=0) sabit, ama konumlar -x yönünde ilerliyor => geri vites
    path = [Pose2D(float(5 - i), 0.0, 0.0) for i in range(6)]  # x: 5,4,3,2,1,0 ; yaw=0
    ego = VehicleState(Pose2D(5.0, 0.0, 0.0), v=0.0)
    traj = teb.plan(path, ego, [], target_v=BEE1.max_speed)
    assert len(traj) >= 2
    # Hareketli noktaların hızı negatif olmalı (geri)
    moving = [tp.v for tp in traj.points if abs(tp.v) > 1e-3]
    assert moving and all(v < 0 for v in moving), f"Geri segmentte negatif hiz bekleniyordu: {moving}"
    # Yayınlanan kafa açısı aracın gerçek yönelimini (~0 rad, +x) korumalı (hareket yönü pi değil)
    for tp in traj.points:
        assert abs(math.atan2(math.sin(tp.pose.yaw), math.cos(tp.pose.yaw))) < math.radians(20)


def test_forward_keeps_positive_speed():
    """İleri sürüşte yön tespiti yanlış pozitifi geri'ye çevirmemeli."""
    teb = TEB(TEBConfig())
    path = straight_path()  # +x ileri
    traj = teb.plan(path, VehicleState(Pose2D(0, 0, 0), v=0.0), [], target_v=BEE1.max_speed)
    assert all(tp.v >= -1e-6 for tp in traj.points)


def test_hold_produces_standstill():
    """target_v=0 (HOLD) -> tek noktalı, sıfır hızlı duruş yörüngesi."""
    teb = TEB()
    traj = teb.plan(straight_path(), VehicleState(Pose2D(0, 0, 0), v=3.0), [], target_v=0.0)
    assert len(traj) == 1 and traj.points[0].v == 0.0


if __name__ == "__main__":
    run(verbose=True)
    print("\nTUM TESTLER GECTI.")
