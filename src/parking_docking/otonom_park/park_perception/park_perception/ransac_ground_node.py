import numpy as np
import open3d as o3d
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


class RansacGroundNode(Node):
    def __init__(self):
        super().__init__('ransac_ground_node')
        # Senin ransac.py'ndeki parametreler
        self.declare_parameter('distance_threshold', 0.15)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('num_iterations', 1000)

        self.sub = self.create_subscription(
            PointCloud2, '/voxel_points', self.cloud_cb, qos_profile_sensor_data)
        self.pub_ground = self.create_publisher(
            PointCloud2, '/ground_points', qos_profile_sensor_data)
        self.pub_obstacle = self.create_publisher(
            PointCloud2, '/obstacle_points', qos_profile_sensor_data)
        self.get_logger().info('ransac_ground_node basladi. /voxel_points dinleniyor.')

    def cloud_cb(self, msg):
        points = pointcloud2_to_xyzi(msg)        # (N,4)
        ransac_n = self.get_parameter('ransac_n').value
        if points.shape[0] < ransac_n:
            return

        # xyz -> Open3D (RANSAC sadece geometriyle ilgilenir)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

        # RANSAC ile zemin duzlemi (senin segment_plane mantigin)
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=self.get_parameter('distance_threshold').value,
            ransac_n=ransac_n,
            num_iterations=self.get_parameter('num_iterations').value)

        # inlier indekslerini ORIJINAL xyzi dizisine uygula -> intensity korunur
        mask = np.zeros(points.shape[0], dtype=bool)
        mask[inliers] = True
        ground = points[mask]      # zemin (intensity dahil)
        obstacle = points[~mask]   # engeller

        # Iki ayri topic yayinla (frame_id korunur)
        self.pub_ground.publish(xyzi_to_pointcloud2(ground, msg.header))
        self.pub_obstacle.publish(xyzi_to_pointcloud2(obstacle, msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = RansacGroundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()