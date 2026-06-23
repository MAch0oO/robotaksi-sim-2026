import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Int8

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


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')
        self.declare_parameter('corridor_half_width', 1.0)  # yan koridor yarı genişlik (m)
        self.declare_parameter('z_min', 0.1)                # zemin gürültüsünü ele
        self.declare_parameter('z_max', 2.0)
        self.declare_parameter('threshold_slow', 1.0)       # bu altında YAVAŞLA (m)
        self.declare_parameter('threshold_stop', 0.4)       # bu altında DUR (m)

        self.sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(Int8, '/safety_state', 10)
        self.get_logger().info('safety_monitor_node basladi. /lidar/points dinleniyor.')

    def cb(self, msg):
        pts = pointcloud2_to_xyz(msg)
        if pts.shape[0] == 0:
            return

        hw = self.get_parameter('corridor_half_width').value
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value

        # İleri koridor: x>0 (ileri), |y|<yarı genişlik, zemin dışı
        mask = (
            (pts[:, 0] > 0.30) &
            (np.abs(pts[:, 1]) < hw) &
            (pts[:, 2] > z_min) & (pts[:, 2] < z_max)
        )
        forward = pts[mask]
        dist = float(np.min(forward[:, 0])) if forward.shape[0] > 0 else float('inf')

        slow = self.get_parameter('threshold_slow').value
        stop = self.get_parameter('threshold_stop').value

        if dist <= stop:
            state = 2
        elif dist <= slow:
            state = 1
        else:
            state = 0

        out = Int8(); out.data = state
        self.pub.publish(out)
        self.get_logger().info(
            f'ileri mesafe={dist:.2f}m -> {["DEVAM","YAVASLA","DUR"][state]}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()