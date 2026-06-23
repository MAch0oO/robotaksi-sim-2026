"""Uçtan uca pist testi: GEOJSON görevinin tam zinciri tamamladığını doğrular.

Hybrid A* gerçek ölçekte koştuğu için bu test diğerlerinden yavaştır (~birkaç saniye).
"""

from __future__ import annotations

import os

from path_planning.sim.sim_runner import run

_GEOJSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "sample_mission.geojson",
)


def test_track_mission_completes():
    res = run(_GEOJSON, verbose=False)
    assert res["completed"], "Gorev MISSION_COMPLETE'e ulasmadi"
    assert res["progress"] == "4/4", f"Tum waypointler tamamlanmadi: {res['progress']}"


if __name__ == "__main__":
    run(_GEOJSON, verbose=True)
