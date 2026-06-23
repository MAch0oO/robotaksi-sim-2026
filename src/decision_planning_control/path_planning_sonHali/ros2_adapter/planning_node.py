"""ROS2 (Foxy) planlama düğümü — çekirdek hattının ince sarmalayıcısı.

Sorumluluğu YALNIZCA: (1) sözleşmedeki topic'lere abone olup son mesajları tutmak,
(2) sabit hızda çekirdek ``PlannerPipeline.update`` çağırmak, (3) çıktıyı sözleşmedeki
topic'lere yayınlamak. İş mantığı çekirdektedir; burada karar/algoritma yoktur.

Bu modül ``rclpy`` ve ROS mesaj paketleri gerektirir → yalnızca araç ortamında çalışır.
Dönüşüm mantığı ``conversions`` modülündedir (rclpy'siz, ayrıca test edilir).

Çalıştırma (araçta):  ros2 run <paket> planning_node  --ros-args -p geojson_path:=...
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Standart Foxy mesajları
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PointStamped, Transform, Twist, Quaternion
from std_msgs.msg import Int8, Float32, Bool, String
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from builtin_interfaces.msg import Duration
# Algı engel listesi (entegrasyon dokunma noktası — ekibin paketine göre değişebilir)
from derived_object_msgs.msg import ObjectArray

from . import conversions as cv
from ..decision.mission_manager import MissionManager
from ..io.geojson_loader import load_waypoints
from ..io.inputs import PlanningInput
from ..planner_pipeline import MapConfig, PlannerPipeline


class PlanningNode(Node):
    """Path Planning ROS2 sarmalayıcı düğümü."""

    def __init__(self) -> None:
        super().__init__("path_planning_node")

        # --- Parametreler ---
        self.declare_parameter("geojson_path", "")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("lane_half_width", 1.75)
        self.declare_parameter("normal_speed_cap", 8.33)   # 30 km/h
        self.declare_parameter("slow_speed_cap", 2.0)      # safety_state == 1 (Yavaşla)

        geojson_path = self.get_parameter("geojson_path").value
        rate = float(self.get_parameter("control_rate_hz").value)
        self._lane_half = float(self.get_parameter("lane_half_width").value)
        self._normal_cap = float(self.get_parameter("normal_speed_cap").value)
        self._slow_cap = float(self.get_parameter("slow_speed_cap").value)

        # --- Çekirdek hattı kur ---
        waypoints, _origin = load_waypoints(geojson_path) if geojson_path else ([], None)
        self.pipeline = PlannerPipeline(MissionManager(waypoints), map_config=MapConfig())

        # --- Son mesaj önbellekleri ---
        self._odom: Odometry | None = None
        self._objects: ObjectArray | None = None
        self._lane: Path | None = None
        self._light_state: int = 0
        self._light_pos = None
        self._sign_label: str = ""
        self._sign_distance: float = float("inf")
        self._park_target = None
        self._safety_state: int = cv.SAFETY_GO
        self._fb_emergency: int = 0

        # --- QoS profilleri ---
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        latched_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST, depth=1)

        # --- Aboneler (girdiler) ---
        self.create_subscription(Odometry, "/localization/odometry", self._on_odom, sensor_qos)
        self.create_subscription(ObjectArray, "/perception/obstacles", self._on_objects, sensor_qos)
        self.create_subscription(Path, "/perception/lane_centerline", self._on_lane, sensor_qos)
        self.create_subscription(Int8, "/perception/traffic_light_state", self._on_light, sensor_qos)
        self.create_subscription(PointStamped, "/perception/traffic_light_pose", self._on_light_pose, sensor_qos)
        self.create_subscription(String, "/perception/object_type", self._on_sign_label, sensor_qos)
        self.create_subscription(Float32, "/perception/object_distance", self._on_sign_distance, sensor_qos)
        self.create_subscription(PoseStamped, "/selected_slot_pose", self._on_park_target, latched_qos)
        self.create_subscription(Int8, "/safety_state", self._on_safety, latched_qos)
        self._try_subscribe_vehicle_estop(sensor_qos)

        # --- Yayıncılar (çıktılar) ---
        self.pub_traj = self.create_publisher(MultiDOFJointTrajectory, "/planning/trajectory", 10)
        self.pub_valid = self.create_publisher(Bool, "/planning/trajectory_valid", 10)
        self.pub_state = self.create_publisher(String, "/planning/state", 10)
        self.pub_target_v = self.create_publisher(Float32, "/planning/target_speed", 10)
        self.pub_gear = self.create_publisher(Int8, "/planning/gear_request", 10)
        self.pub_constraint = self.create_publisher(String, "/planner/constraint", 10)
        # OMUX (kapı/sinyal/vites/acil) — araç mesaj paketi varsa yayınla.
        self.pub_omux = None
        try:
            from smart_can_msgs.msg import rc_unittoOmux  # type: ignore
            self._rc_unittoOmux = rc_unittoOmux
            self.pub_omux = self.create_publisher(rc_unittoOmux, "/beemobs/rc_unittoOmux", 10)
        except Exception:
            self.get_logger().warn("rc_unittoOmux tipi yok; OMUX (kapi/sinyal) yayinlanmayacak.")

        # --- Kontrol döngüsü ---
        self.create_timer(1.0 / rate, self._control_loop)
        self.get_logger().info(f"path_planning_node hazır ({rate:.0f} Hz, {len(waypoints)} waypoint).")

    # ------------------------------------------------------------------ #
    # Abone geri çağrıları (yalnızca son mesajı sakla)
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg): self._odom = msg
    def _on_objects(self, msg): self._objects = msg
    def _on_lane(self, msg): self._lane = msg
    def _on_light(self, msg): self._light_state = msg.data
    def _on_light_pose(self, msg): self._light_pos = (msg.point.x, msg.point.y)
    def _on_sign_label(self, msg): self._sign_label = msg.data
    def _on_sign_distance(self, msg): self._sign_distance = float(msg.data)
    def _on_park_target(self, msg): self._park_target = cv.pose2d_from_pose(msg.pose)
    def _on_safety(self, msg): self._safety_state = msg.data

    def _try_subscribe_vehicle_estop(self, qos) -> None:
        """Araç donanım e-stop geri beslemesine (/beemobs/FB_OMUX_to_AUTONOMOUS) abone olur.

        Mesaj tipi araç paketine özgüdür (smart_can_*); kurulu değilse atlanır ve yalnızca
        /safety_state kullanılır. ENTEGRASYON: ekip aşağıdaki tip adını araçtaki paketle eşler.
        """
        def _read_fb_emergency(m):
            # Alan adı paket tanımına göre 'FB_EMERGENCY' veya 'fb_emergency' olabilir.
            val = getattr(m, "FB_EMERGENCY", None)
            if val is None:
                val = getattr(m, "fb_emergency", 0)
            self._fb_emergency = int(val)

        try:
            from smart_can_msgs.msg import FB_OMUX_to_AUTONOMOUS  # type: ignore
            self.create_subscription(
                FB_OMUX_to_AUTONOMOUS, "/beemobs/FB_OMUX_to_AUTONOMOUS", _read_fb_emergency, qos)
            self.get_logger().info("Arac e-stop geri beslemesi baglandi.")
        except Exception:
            self.get_logger().warn("Arac e-stop mesaj tipi bulunamadi; yalnizca /safety_state kullanilacak.")

    # ------------------------------------------------------------------ #
    # Ana kontrol döngüsü
    # ------------------------------------------------------------------ #
    def _control_loop(self) -> None:
        if self._odom is None:
            return  # lokalizasyon gelmeden planlama yapma

        inp = PlanningInput(
            vehicle_state=cv.vehicle_state_from_odom(self._odom),
            obstacles=cv.obstacles_from_object_array(self._objects) if self._objects else [],
            traffic_light=cv.traffic_light_from_state(self._light_state, self._light_pos),
            traffic_sign=cv.traffic_sign_from_label(self._sign_label, self._sign_distance),
            lane=cv.lane_from_path(self._lane, self._lane_half) if self._lane else None,
            park_target=self._park_target,
            emergency=cv.emergency_from_signals(self._safety_state, self._fb_emergency),
            stamp=self.get_clock().now().nanoseconds * 1e-9,
        )

        out = self.pipeline.update(inp)

        # safety_state == 1 (Yavaşla): yayınlanan hızı üst sınırla (çekirdekte alan yok — wrapper kapar)
        cap = cv.speed_cap_from_safety(self._safety_state, self._normal_cap, self._slow_cap)
        self._publish(out, cap, inp.emergency)

    # ------------------------------------------------------------------ #
    # Yayınlama
    # ------------------------------------------------------------------ #
    def _publish(self, out, speed_cap: float, emergency: bool) -> None:
        now = self.get_clock().now().to_msg()

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = now
        traj_msg.header.frame_id = "map"
        traj_msg.joint_names = ["base_link"]
        for pt in cv.trajectory_to_points(out.trajectory):
            p = MultiDOFJointTrajectoryPoint()
            tf = Transform()
            tf.translation.x, tf.translation.y, tf.translation.z = pt["x"], pt["y"], 0.0
            qx, qy, qz, qw = cv.quaternion_from_yaw(pt["yaw"])
            tf.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            p.transforms.append(tf)
            tw = Twist()
            tw.linear.x = max(-speed_cap, min(speed_cap, pt["v"]))  # yavaşla sınırı
            p.velocities.append(tw)
            p.time_from_start = Duration(sec=int(pt["t"]), nanosec=int((pt["t"] % 1.0) * 1e9))
            traj_msg.points.append(p)
        self.pub_traj.publish(traj_msg)

        self.pub_valid.publish(Bool(data=bool(out.valid)))
        state_name = out.behavior_state.name if out.behavior_state else out.planner_action.name
        self.pub_state.publish(String(data=state_name))
        self.pub_target_v.publish(Float32(data=float(min(out.target_v, speed_cap))))
        gear = cv.gear_request_from_trajectory(out.trajectory)
        self.pub_gear.publish(Int8(data=gear))

        # /planner/constraint JSON (Rota Ekibi okur)
        import json
        self.pub_constraint.publish(String(data=json.dumps(
            cv.constraints_to_dict(out.constraints, state_name, out.target_v),
            ensure_ascii=False)))

        # OMUX: kapı / sinyal / vites / acil (araç mesaj paketi varsa)
        if self.pub_omux is not None:
            m = self._rc_unittoOmux()
            m.rc_ignition = 1
            m.rc_drl = 1
            m.rc_selectiongear = int(gear)               # 1=Drive, 2=Reverse
            m.autonomous_door_open = 1 if out.door_open else 0
            m.rc_signalstatus = int(out.turn_signal)     # 0/1/2/3
            m.autonomous_emergency = 1 if emergency else 0
            self.pub_omux.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
