import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {
    PointField.INT8: np.int8,     PointField.UINT8: np.uint8,
    PointField.INT16: np.int16,   PointField.UINT16: np.uint16,
    PointField.INT32: np.int32,   PointField.UINT32: np.uint32,
    PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64,
}


def pointcloud2_to_xyzi(cloud):
    np_dtype = np.dtype({
        'names':   [f.name for f in cloud.fields],
        'formats': [_DATATYPES[f.datatype] for f in cloud.fields],
        'offsets': [f.offset for f in cloud.fields],
        'itemsize': cloud.point_step,
    })
    arr = np.frombuffer(cloud.data, dtype=np_dtype)
    x = arr['x'].astype(np.float32)
    y = arr['y'].astype(np.float32)
    z = arr['z'].astype(np.float32)
    if 'intensity' in arr.dtype.names:
        i = arr['intensity'].astype(np.float32)
    else:
        i = np.zeros_like(x)
    return np.column_stack((x, y, z, i))


def xyzi_to_pointcloud2(points_xyzi, header):
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points_xyzi.shape[0]
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * points_xyzi.shape[0]
    msg.is_dense = True
    msg.data = points_xyzi.astype(np.float32).tobytes()
    return msg


def apply_roi_filter(points, x_min, x_max, y_min, y_max, z_min, z_max):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    inside = (
        (x >= x_min) & (x <= x_max) &
        (y >= y_min) & (y <= y_max) &
        (z >= z_min) & (z <= z_max)
    )
    return points[inside]


class RoiFilterNode(Node):
    def __init__(self):
        super().__init__('roi_filter_node')
        self.declare_parameter('x_min', -2.0)
        self.declare_parameter('x_max', 6.0)
        self.declare_parameter('y_min', -2.5)
        self.declare_parameter('y_max', 2.5)
        self.declare_parameter('z_min', -0.3)
        self.declare_parameter('z_max', 2.0)

        self.sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, '/roi_points', qos_profile_sensor_data)
        self.get_logger().info('roi_filter_node basladi. /lidar/points dinleniyor.')

    def cloud_cb(self, msg):
        points = pointcloud2_to_xyzi(msg)
        if points.shape[0] == 0:
            return
        p = self.get_parameter
        filtered = apply_roi_filter(
            points,
            p('x_min').value, p('x_max').value,
            p('y_min').value, p('y_max').value,
            p('z_min').value, p('z_max').value,
        )
        out = xyzi_to_pointcloud2(filtered, msg.header)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RoiFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()