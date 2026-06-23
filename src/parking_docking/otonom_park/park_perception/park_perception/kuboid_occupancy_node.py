import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PolygonStamped
from std_msgs.msg import Bool

_DATATYPES = {
    PointField.INT8: np.int8,     PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,   PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,   PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


def pointcloud2_to_xyz(cloud):
    np_dtype = np.dtype({
        'names':   [f.name for f in cloud.fields],
        'formats': [_DATATYPES[f.datatype] for f in cloud.fields],
        'offsets': [f.offset for f in cloud.fields],
        'itemsize': cloud.point_step,
    })
    arr = np.frombuffer(cloud.data, dtype=np_dtype)
    return np.column_stack((arr['x'], arr['y'], arr['z'])).astype(np.float32)


# ---- Senin kuboid mantığın (birebir) ----
def build_cuboid(corners_xy, z_min, z_max):
    bl = np.array([corners_xy[0][0], corners_xy[0][1], z_min])   # köşe[0] = origin
    br = np.array([corners_xy[1][0], corners_xy[1][1], z_min])   # köşe[1] = komşu
    tl = np.array([corners_xy[3][0], corners_xy[3][1], z_min])   # köşe[3] = komşu
    O = bl
    u = br - bl
    v = tl - bl
    w = np.array([0.0, 0.0, z_max - z_min])
    return O, u, v, w


def points_in_cuboid(points, O, u, v, w):
    d = points - O
    pu = d @ u
    pv = d @ v
    pw = d @ w
    inside = (
        (pu >= 0) & (pu <= u @ u) &
        (pv >= 0) & (pv <= v @ v) &
        (pw >= 0) & (pw <= w @ w)
    )
    return inside


class KuboidOccupancyNode(Node):
    def __init__(self):
        super().__init__('kuboid_occupancy_node')
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 2.0)
        self.declare_parameter('n_esik', 50)

        self.latest_corners = None

        self.sub_corners = self.create_subscription(
            PolygonStamped, '/slot_corners', self.corners_cb, 10)
        self.sub_cloud = self.create_subscription(
            PointCloud2, '/obstacle_points', self.cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Bool, '/slot_occupancy', 10)

        self.get_logger().info('kuboid_occupancy_node basladi.')

    def corners_cb(self, msg):
        self.latest_corners = [(p.x, p.y) for p in msg.polygon.points]

    def cloud_cb(self, msg):
        if self.latest_corners is None or len(self.latest_corners) < 4:
            return

        points = pointcloud2_to_xyz(msg)
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        n_esik = self.get_parameter('n_esik').value

        O, u, v, w = build_cuboid(self.latest_corners, z_min, z_max)
        inside = points_in_cuboid(points, O, u, v, w)
        N = int(inside.sum())
        occupied = N > n_esik

        # N'i logla (N_esik kalibrasyonu için terminalde görürsün)
        self.get_logger().info(f'N={N} -> {"DOLU" if occupied else "BOS"} (esik={n_esik})')

        out = Bool()
        out.data = bool(occupied)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = KuboidOccupancyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()