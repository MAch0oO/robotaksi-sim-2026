import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from sklearn.cluster import DBSCAN
import tf2_ros

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


def transform_matrix(tr):
    """TransformStamped.transform -> 4x4 matris."""
    qx, qy, qz, qw = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]
    return M


class ObstacleExtractorNode(Node):
    def __init__(self):
        super().__init__('obstacle_extractor_node')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('cluster_eps', 0.5)          # DBSCAN komşuluk (m)
        self.declare_parameter('cluster_min_samples', 5)    # min nokta/küme
        self.declare_parameter('min_z', 0.1)                # zemin gürültüsü ele
        self.declare_parameter('max_z', 2.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(
            PointCloud2, '/obstacle_points', self.cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(ObjectArray, '/perception/obstacles', 10)
        self.get_logger().info('obstacle_extractor_node basladi. /obstacle_points -> /perception/obstacles')

    def cb(self, msg):
        pts = pointcloud2_to_xyz(msg)
        if pts.shape[0] == 0:
            return

        mnz = self.get_parameter('min_z').value
        mxz = self.get_parameter('max_z').value
        pts = pts[(pts[:, 2] > mnz) & (pts[:, 2] < mxz)]

        ms = self.get_parameter('cluster_min_samples').value
        if pts.shape[0] < ms:
            self.pub.publish(ObjectArray())     # engel yok
            return

        # lidar -> map dönüşümü
        target = self.get_parameter('target_frame').value
        src = msg.header.frame_id or 'lidar_frame'
        try:
            tf = self.tf_buffer.lookup_transform(
                target, src, rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f'TF bekleniyor: {e}', throttle_duration_sec=2.0)
            return
        M = transform_matrix(tf.transform)
        homo = np.hstack([pts, np.ones((pts.shape[0], 1))])
        pts_map = (M @ homo.T).T[:, :3]

        # DBSCAN kümeleme (x,y)
        eps = self.get_parameter('cluster_eps').value
        labels = DBSCAN(eps=eps, min_samples=ms).fit_predict(pts_map[:, :2])

        arr = ObjectArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = target
        oid = 0
        for lbl in set(labels):
            if lbl == -1:                       # gürültü
                continue
            c = pts_map[labels == lbl]
            obj = Object()
            obj.header = arr.header
            obj.id = oid
            obj.pose.position.x = float(c[:, 0].mean())
            obj.pose.position.y = float(c[:, 1].mean())
            obj.pose.position.z = 0.0
            obj.pose.orientation.w = 1.0
            obj.shape.type = SolidPrimitive.BOX
            obj.shape.dimensions = [
                max(float(c[:, 0].max() - c[:, 0].min()), 0.1),
                max(float(c[:, 1].max() - c[:, 1].min()), 0.1),
                max(float(c[:, 2].max() - c[:, 2].min()), 0.1),
            ]
            arr.objects.append(obj)
            oid += 1

        self.pub.publish(arr)
        self.get_logger().info(f'{len(arr.objects)} engel yayinlandi', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleExtractorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()