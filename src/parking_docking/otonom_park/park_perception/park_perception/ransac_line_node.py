import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PolygonStamped, Point32
from sklearn.linear_model import RANSACRegressor

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


# ---- Senin çizgi fonksiyonların (birebir) ----
def normalize_line(line):
    a, b, c = line
    n = np.hypot(a, b)
    return (a / n, b / n, c / n)


def fit_line_ransac(points, residual_threshold=0.25):
    X = points[:, 0]
    Y = points[:, 1]

    rA = RANSACRegressor(residual_threshold=residual_threshold, min_samples=2)
    rA.fit(X.reshape(-1, 1), Y)
    mA = rA.estimator_.coef_[0]
    cA = rA.estimator_.intercept_
    inA = rA.inlier_mask_
    lineA = (mA, -1.0, cA)

    rB = RANSACRegressor(residual_threshold=residual_threshold, min_samples=2)
    rB.fit(Y.reshape(-1, 1), X)
    mB = rB.estimator_.coef_[0]
    cB = rB.estimator_.intercept_
    inB = rB.inlier_mask_
    lineB = (1.0, -mB, -cB)

    if inA.sum() >= inB.sum():
        line, inliers = lineA, inA
    else:
        line, inliers = lineB, inB
    return normalize_line(line), inliers


def line_intersection(l1, l2):
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    x = (-c1 * b2 + b1 * c2) / det
    y = (-a1 * c2 + c1 * a2) / det
    return np.array([x, y])


class RansacLineNode(Node):
    def __init__(self):
        super().__init__('ransac_line_node')
        self.declare_parameter('residual_threshold', 0.25)
        self.sub = self.create_subscription(
            PointCloud2, '/line_candidate_points', self.cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PolygonStamped, '/slot_corners', 10)
        self.get_logger().info('ransac_line_node basladi. /line_candidate_points dinleniyor.')

    def cloud_cb(self, msg):
        points = pointcloud2_to_xyzi(msg)
        xy = points[:, :2]
        if xy.shape[0] < 10:        # 3 çizgi için yeterli nokta yok
            return

        rt = self.get_parameter('residual_threshold').value

        # --- Multi-RANSAC: sırayla 3 çizgi ---
        remaining = xy.copy()
        lines, line_points = [], []
        for _ in range(3):
            if remaining.shape[0] < 2:
                return
            line, inliers = fit_line_ransac(remaining, rt)
            lines.append(line)
            line_points.append(remaining[inliers])
            remaining = remaining[~inliers]

        # --- Arka duvar = en büyük |b| ---
        b_values = [abs(l[1]) for l in lines]
        back_idx = int(np.argmax(b_values))
        back_line = lines[back_idx]
        side_indices = [k for k in range(3) if k != back_idx]

        # --- Yan çizgileri sol/sağ ayır ---
        side_a, side_b = lines[side_indices[0]], lines[side_indices[1]]
        mean_x_a = line_points[side_indices[0]][:, 0].mean()
        mean_x_b = line_points[side_indices[1]][:, 0].mean()
        if mean_x_a <= mean_x_b:
            left_line, right_line = side_a, side_b
        else:
            left_line, right_line = side_b, side_a

        # --- Sanal ön çizgi (arka duvara paralel) ---
        ab, bb, cb = back_line
        side_pts = np.vstack((line_points[side_indices[0]], line_points[side_indices[1]]))
        dists = ab * side_pts[:, 0] + bb * side_pts[:, 1] + cb
        sign = np.sign(np.mean(dists))
        d_front = sign * np.percentile(np.abs(dists), 95)
        front_line = (ab, bb, cb - d_front)

        # --- 4 köşe = kesişimler ---
        arka_sol = line_intersection(back_line, left_line)
        arka_sag = line_intersection(back_line, right_line)
        on_sag = line_intersection(front_line, right_line)
        on_sol = line_intersection(front_line, left_line)
        corners = [arka_sol, arka_sag, on_sag, on_sol]
        if any(c is None for c in corners):
            return

        # --- PolygonStamped yayınla ---
        out = PolygonStamped()
        out.header = msg.header              # frame_id korunur (lidar_frame)
        for c in corners:
            p = Point32()
            p.x = float(c[0]); p.y = float(c[1]); p.z = 0.0
            out.polygon.points.append(p)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = RansacLineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()