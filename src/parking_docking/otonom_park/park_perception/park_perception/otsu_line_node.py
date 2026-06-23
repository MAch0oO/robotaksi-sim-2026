import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from skimage.filters import threshold_otsu

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


class OtsuLineNode(Node):
    def __init__(self):
        super().__init__('otsu_line_node')
        self.sub = self.create_subscription(
            PointCloud2, '/ground_points', self.cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, '/line_candidate_points', qos_profile_sensor_data)
        self.get_logger().info('otsu_line_node basladi. /ground_points dinleniyor.')

    def cloud_cb(self, msg):
        points = pointcloud2_to_xyzi(msg)        # (N,4)
        if points.shape[0] == 0:
            return

        intensity = points[:, 3]

        # Guard: Otsu en az 2 farkli deger ister
        if np.unique(intensity).shape[0] < 2:
            self.get_logger().warn('Intensity tek degerli, Otsu atlandi.')
            return

        otsu_th = threshold_otsu(intensity)       # senin mantigin: otomatik esik
        line_mask = intensity > otsu_th           # esik ustu = park cizgisi
        line_candidates = points[line_mask]

        out = xyzi_to_pointcloud2(line_candidates, msg.header)  # frame_id korunur
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OtsuLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()