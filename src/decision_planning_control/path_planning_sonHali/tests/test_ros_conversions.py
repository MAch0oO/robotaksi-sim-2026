"""ROS dönüşümleri testi (rclpy gerektirmez).

Mesaj nesneleri SimpleNamespace ile taklit edilir; conversions duck-typing kullandığı
için bu yeterlidir. Böylece ROS kurulu olmayan ortamda da arayüz mantığı doğrulanır.
"""

from __future__ import annotations

import math
from types import SimpleNamespace as NS

from path_planning.common.enums import LightState, ObstacleClass
from path_planning.common.types import Pose2D, Trajectory, TrajectoryPoint
from path_planning.ros2_adapter import conversions as cv


def _pose(x, y, yaw):
    qx, qy, qz, qw = cv.quaternion_from_yaw(yaw)
    return NS(position=NS(x=x, y=y, z=0.0), orientation=NS(x=qx, y=qy, z=qz, w=qw))


def test_quaternion_yaw_roundtrip():
    for yaw in (-3.0, -1.2, 0.0, 0.7, 2.5):
        qx, qy, qz, qw = cv.quaternion_from_yaw(yaw)
        assert abs(cv.yaw_from_quaternion(qx, qy, qz, qw) - yaw) < 1e-9


def test_vehicle_state_from_odom():
    odom = NS(
        pose=NS(pose=_pose(3.0, -2.0, math.pi / 2)),
        twist=NS(twist=NS(linear=NS(x=4.5, y=0.0, z=0.0), angular=NS(x=0, y=0, z=0.3))),
        header=NS(stamp=NS(sec=10, nanosec=500_000_000), frame_id="map"),
    )
    vs = cv.vehicle_state_from_odom(odom)
    assert (round(vs.pose.x, 3), round(vs.pose.y, 3)) == (3.0, -2.0)
    assert abs(vs.pose.yaw - math.pi / 2) < 1e-9
    assert vs.v == 4.5 and vs.omega == 0.3
    assert abs(vs.stamp - 10.5) < 1e-9 and vs.frame == "map"


def test_obstacles_from_object_array():
    moving = NS(id=1, pose=NS(position=NS(x=5.0, y=0.0)),
                twist=NS(linear=NS(x=1.0, y=0.0)), polygon=None, shape=NS(dimensions=[2.0, 1.0, 1.0]))
    static = NS(id=2, pose=NS(position=NS(x=8.0, y=1.0)),
                twist=NS(linear=NS(x=0.0, y=0.0)),
                polygon=NS(points=[NS(x=7.5, y=0.5), NS(x=8.5, y=0.5), NS(x=8.5, y=1.5)]), shape=None)
    obs = cv.obstacles_from_object_array(NS(objects=[moving, static]))
    assert obs[0].obstacle_class is ObstacleClass.DYNAMIC and obs[0].is_dynamic
    assert obs[1].obstacle_class is ObstacleClass.STATIC
    assert obs[0].center == (5.0, 0.0)
    assert obs[1].polygon  # poligon korundu


def test_lane_from_path():
    path = NS(poses=[NS(pose=_pose(0, 0, 0)), NS(pose=_pose(10, 0, 0))])
    lane = cv.lane_from_path(path, half_width=1.75)
    assert lane is not None and len(lane.centerline) == 2
    assert lane.left_offset == 1.75 and lane.right_offset == 1.75
    assert cv.lane_from_path(NS(poses=[]), 1.75) is None  # boş path -> None


def test_traffic_light_and_emergency():
    assert cv.traffic_light_from_state(1).state is LightState.RED
    assert cv.traffic_light_from_state(3).state is LightState.GREEN
    # emergency: lidar VEYA arac e-stop
    assert cv.emergency_from_signals(0, 0) is False
    assert cv.emergency_from_signals(2, 0) is True   # lidar acil
    assert cv.emergency_from_signals(0, 1) is True   # arac e-stop feedback


def test_speed_cap_and_gear():
    assert cv.speed_cap_from_safety(0, 8.33, 2.0) == 8.33
    assert cv.speed_cap_from_safety(1, 8.33, 2.0) == 2.0   # Yavasla
    # gear: isaretli hiz konvansiyonu
    fwd = Trajectory([TrajectoryPoint(Pose2D(0, 0, 0), v=3.0, t=0.0)])
    rev = Trajectory([TrajectoryPoint(Pose2D(0, 0, 0), v=-1.0, t=0.0)])
    assert cv.gear_request_from_trajectory(fwd) == cv.GEAR_DRIVE
    assert cv.gear_request_from_trajectory(rev) == cv.GEAR_REVERSE
    assert cv.gear_request_from_trajectory(Trajectory()) == cv.GEAR_NEUTRAL


def test_traffic_sign_from_label():
    from path_planning.common.enums import SignType
    s = cv.traffic_sign_from_label("LEVHA_TUNEL", 8.0)
    assert s is not None and s.sign_type is SignType.TUNNEL and s.distance == 8.0
    assert cv.traffic_sign_from_label("ARAC", 5.0) is None         # levha değil
    assert cv.traffic_sign_from_label("", float("inf")) is None


def test_constraints_to_dict():
    from path_planning.common.types import Constraints
    c = Constraints(no_entry=True, no_parking=True, lane_merge="SAGA",
                    mandatory_directions=["SAG"])
    d = cv.constraints_to_dict(c, "LANE_FOLLOWING", 5.0)
    assert d["girilmez"] is True and d["park_yasak"] is True
    assert d["serit_birlesme"] == "SAGA" and d["mecburi_yon"] == ["SAG"]
    assert d["durum"] == "LANE_FOLLOWING"


def test_trajectory_to_points():
    traj = Trajectory([
        TrajectoryPoint(Pose2D(1, 2, 0.5), v=3.0, t=0.0),
        TrajectoryPoint(Pose2D(2, 2, 0.0), v=2.0, t=1.0),
    ])
    pts = cv.trajectory_to_points(traj)
    assert pts[0] == {"x": 1, "y": 2, "yaw": 0.5, "v": 3.0, "t": 0.0}
    assert len(pts) == 2


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"GECTI: {name}")
    print("\nTUM TESTLER GECTI.")
