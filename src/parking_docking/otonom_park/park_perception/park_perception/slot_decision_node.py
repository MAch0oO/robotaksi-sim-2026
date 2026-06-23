import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PolygonStamped, PoseStamped, PointStamped
from std_msgs.msg import Bool, String
import tf2_ros
import tf2_geometry_msgs  # PointStamped TF dönüşümü için (import şart)


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SlotDecisionNode(Node):
    def __init__(self):
        super().__init__('slot_decision_node')
        self.declare_parameter('sign_match_threshold', 3.0)   # tabela-slot eşleşme (m)
        self.declare_parameter('confidence_threshold', 0.5)   # min güven skoru
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('sign_frame', 'camera_frame')  # tabela hangi frame'de

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_corners = None
        self.corners_frame = None
        self.latest_occupied = None
        self.signs = []   # [{x,y,z,conf,stamp}]

        self.create_subscription(PolygonStamped, '/slot_corners', self.corners_cb, 10)
        self.create_subscription(Bool, '/slot_occupancy', self.occ_cb, 10)
        self.create_subscription(String, '/ai/tabela_3d_konum', self.sign_cb, 10)

        self.pub = self.create_publisher(PoseStamped, '/selected_slot_pose', 10)
        self.timer = self.create_timer(0.5, self.decide)   # periyodik karar
        self.get_logger().info('slot_decision_node basladi.')

    def corners_cb(self, msg):
        self.latest_corners = [(p.x, p.y) for p in msg.polygon.points]
        self.corners_frame = msg.header.frame_id or 'lidar_frame'

    def occ_cb(self, msg):
        self.latest_occupied = msg.data

    def sign_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get('class_name') != 'park_edilemez':   # SADECE park edilemez
            return
        pos = data.get('position_3d', {})
        self.signs.append({
            'x': pos.get('x', 0.0), 'y': pos.get('y', 0.0), 'z': pos.get('z', 0.0),
            'conf': data.get('confidence', 0.0),
            'stamp': self.get_clock().now(),
        })

    def transform_point(self, x, y, z, src_frame, target_frame):
        ps = PointStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = rclpy.time.Time().to_msg()   # en güncel TF
        ps.point.x = float(x); ps.point.y = float(y); ps.point.z = float(z)
        out = self.tf_buffer.transform(ps, target_frame, timeout=Duration(seconds=0.5))
        return np.array([out.point.x, out.point.y])

    def decide(self):
        if self.latest_corners is None or len(self.latest_corners) < 4:
            return
        if self.latest_occupied is None:
            return

        target = self.get_parameter('target_frame').value

        # 1) Slot köşelerini map'e çevir
        try:
            corners = np.array([
                self.transform_point(x, y, 0.0, self.corners_frame, target)
                for (x, y) in self.latest_corners
            ])
        except Exception as e:
            self.get_logger().warn(f'TF (slot) bekleniyor: {e}')
            return
        center = corners.mean(axis=0)

        # 2) Doluluk kontrolü
        if self.latest_occupied:
            self.get_logger().info('Slot DOLU -> yayinlanmiyor')
            return

        # 3) Tabela eşleştirme (taze tabelalar + güven filtresi + mesafe)
        now = self.get_clock().now()
        self.signs = [s for s in self.signs if (now - s['stamp']) < Duration(seconds=2.0)]
        conf_th = self.get_parameter('confidence_threshold').value
        match_th = self.get_parameter('sign_match_threshold').value
        sign_frame = self.get_parameter('sign_frame').value

        sign_near = False
        for s in self.signs:
            if s['conf'] < conf_th:
                continue
            try:
                sp = self.transform_point(s['x'], s['y'], s['z'], sign_frame, target)
            except Exception as e:
                self.get_logger().warn(f'TF (tabela) bekleniyor: {e}')
                continue
            if np.linalg.norm(sp - center) <= match_th:
                sign_near = True
                break

        if sign_near:
            self.get_logger().info('Slot BOS ama TABELA var -> YASAK, yayinlanmiyor')
            return

        # 4) PARK EDILEBILIR -> hedef poz hesapla
        back_center = (corners[0] + corners[1]) / 2.0    # arka_sol + arka_sag
        front_center = (corners[2] + corners[3]) / 2.0   # on_sag + on_sol
        direction = back_center - front_center           # ön -> arka (içeri burun)
        theta = math.atan2(direction[1], direction[0])

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = target
        out.pose.position.x = float(center[0])
        out.pose.position.y = float(center[1])
        out.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(theta)
        out.pose.orientation.x = qx
        out.pose.orientation.y = qy
        out.pose.orientation.z = qz
        out.pose.orientation.w = qw
        self.pub.publish(out)
        self.get_logger().info(
            f'PARK EDILEBILIR -> /selected_slot_pose '
            f'({center[0]:.2f}, {center[1]:.2f}, {math.degrees(theta):.0f} deg)')


def main(args=None):
    rclpy.init(args=args)
    node = SlotDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()