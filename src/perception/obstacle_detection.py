#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2

import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

from cv_bridge import CvBridge
import cv2


class Track:
    _next_id = 0

    def __init__(self, centroid, stamp):
        self.id = Track._next_id
        Track._next_id += 1
        self.centroid = np.asarray(centroid, dtype=float)
        self.velocity = np.zeros(2, dtype=float)
        self.speed = 0.0
        self.last_stamp = stamp
        self.last_update = stamp
        self.hits = 1
        self.is_dynamic = False
        self.dynamic_timer = 0
        self.stale = False

    def update(self, centroid, stamp,
               enter_threshold=0.04, exit_threshold=0.02,
               timer_frames=15):
        centroid = np.asarray(centroid, dtype=float)
        dt = stamp - self.last_stamp
        if dt > 1e-3:
            inst_vel = (centroid[:2] - self.centroid[:2]) / dt
            alpha = 0.2
            self.velocity = alpha * inst_vel + (1.0 - alpha) * self.velocity
            self.speed = float(np.linalg.norm(self.velocity))

        self.centroid = centroid
        self.last_stamp = stamp
        self.last_update = stamp
        self.hits += 1
        self.stale = False

        if self.speed > enter_threshold:
            self.is_dynamic = True
            self.dynamic_timer = timer_frames
        elif self.speed < exit_threshold:
            if self.dynamic_timer > 0:
                self.dynamic_timer -= 1
            else:
                self.is_dynamic = False

    def mark_stale(self):
        self.stale = True


def euclidean_cluster(points, tolerance, min_size, max_size):
    n = len(points)
    if n == 0:
        return []
    visited = np.zeros(n, dtype=bool)
    clusters = []
    inv = 1.0 / tolerance
    keys = np.floor(points * inv).astype(np.int64)
    voxel_map = {}
    for i in range(n):
        key = (keys[i, 0], keys[i, 1], keys[i, 2])
        voxel_map.setdefault(key, []).append(i)

    def neighbors(idx):
        cx, cy, cz = keys[idx]
        res = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    cand = voxel_map.get((cx + dx, cy + dy, cz + dz))
                    if cand:
                        for j in cand:
                            if not visited[j] and np.linalg.norm(points[idx] - points[j]) <= tolerance:
                                res.append(j)
        return res

    for i in range(n):
        if visited[i]:
            continue
        queue = [i]
        visited[i] = True
        cluster_idx = []
        while queue:
            cur = queue.pop()
            cluster_idx.append(cur)
            for nb in neighbors(cur):
                if not visited[nb]:
                    visited[nb] = True
                    queue.append(nb)
        if min_size <= len(cluster_idx) <= max_size:
            clusters.append(np.array(cluster_idx))
    return clusters


class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection_system')

        self.lidar_topic = '/velodyne_points'
        self.rgb_topic = '/zed2/image_raw'
        self.camera_info_topic = '/zed2/camera_info'

        self.lidar_frame = 'velodyne'
        self.camera_frame = 'zed2_left_camera_frame'
        self.tracking_frame = 'odom'
        self.output_frame = 'base_link'

        self.cluster_tolerance = 0.5
        self.min_cluster_size = 2
        self.max_cluster_size = 5000
        self.ground_z_threshold = -0.25
        self.roi_x_min = 0.0
        self.roi_x_max = 20.0
        self.roi_y_min = -10.0
        self.roi_y_max = 10.0

        self.dynamic_enter_threshold = 0.08
        self.dynamic_exit_threshold = 0.02
        self.dynamic_timer_frames = 15

        self.track_max_distance = 2.0
        self.track_timeout = 1.0
        self.min_track_hits = 3

        self.stale_track_timeout = 1.0
        self.tf_timeout = Duration(seconds=0.1)

        self.tracks = {}
        self.active_tracks = []
        self.fx = self.fy = self.cx = self.cy = None
        self.bridge = CvBridge()
        self.tracking_tf_ok = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.lidar_frame_count = 0
        self.camera_info_received = False

        self.get_logger().info(
            f'Parametreler Yuklendi -> Lidar Konusu: {self.lidar_topic}, Kamera Konusu: {self.rgb_topic}, '
            f'Ground Z Siniri: {self.ground_z_threshold:.2f}, Tolerans: {self.cluster_tolerance:.2f}'
        )

        self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)

        cv2.namedWindow("Otonom Surus Kamerasi", cv2.WINDOW_NORMAL)
        self.get_logger().info("Duzeltilmis Engel Tanima Node'u Basladi! Bekleniyor...")

    def lookup_transform(self, target_frame, source_frame, stamp):
        if target_frame == source_frame:
            return None

        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=self.tf_timeout)
        except TransformException:
            pass

        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, Time())
        except TransformException as e:
            self.get_logger().error(
                f"TF Hatasi ({source_frame} -> {target_frame}): {e}",
                throttle_duration_sec=2.0)
            return None

    @staticmethod
    def transform_to_matrix(tf: TransformStamped):
        t = tf.transform.translation
        q = tf.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
            [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)]
        ])

        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = [t.x, t.y, t.z]
        return M

    def transform_point(self, x, y, z, source_frame, target_frame, stamp=None):
        if source_frame == target_frame:
            return np.array([float(x), float(y), float(z)])

        if stamp is None:
            stamp = Time()

        tf = self.lookup_transform(target_frame, source_frame, stamp)
        if tf is None:
            return None

        M = self.transform_to_matrix(tf)
        p = np.array([x, y, z, 1.0])
        out = M @ p
        return out[:3]

    def lidar_callback(self, msg: PointCloud2):
        self.lidar_frame_count += 1
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        raw = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if len(raw) == 0:
            return
        points = np.array([[p[0], p[1], p[2]] for p in raw], dtype=np.float32)

        mask_ground = points[:, 2] > self.ground_z_threshold
        mask_roi = ((points[:, 0] > self.roi_x_min) & (points[:, 0] < self.roi_x_max) &
                    (points[:, 1] > self.roi_y_min) & (points[:, 1] < self.roi_y_max))
        filtered = points[mask_ground & mask_roi]

        if filtered.shape[0] < self.min_cluster_size:
            self.active_tracks = []
            return

        clusters = euclidean_cluster(filtered, self.cluster_tolerance, self.min_cluster_size, self.max_cluster_size)

        tf_lidar_to_track = self.lookup_transform(self.tracking_frame, self.lidar_frame,
                                                  msg.header.stamp)
        self.tracking_tf_ok = tf_lidar_to_track is not None

        if not self.tracking_tf_ok:
            self.get_logger().warn(
                f'Lidar-Takip TF donusumu alinamadi ({self.lidar_frame} -> {self.tracking_frame})! '
                'Takip sifirlaniyor.', throttle_duration_sec=5.0
            )
            for tr in self.tracks.values():
                tr.mark_stale()
            self._purge_stale_tracks(stamp_sec)
            return

        M_lidar_to_track = self.transform_to_matrix(tf_lidar_to_track)

        detections = []
        for idx_arr in clusters:
            pts = filtered[idx_arr]
            c_local = pts.mean(axis=0)
            size = pts.max(axis=0) - pts.min(axis=0)

            p_h = np.array([c_local[0], c_local[1], c_local[2], 1.0])
            c_track = (M_lidar_to_track @ p_h)[:3]

            c_base = self.transform_point(c_local[0], c_local[1], c_local[2],
                                          self.lidar_frame, self.output_frame,
                                          stamp=msg.header.stamp)

            if c_base is None:
                continue

            detections.append({
                'centroid_track': c_track,
                'centroid_base': c_base,
                'size': size,
                'local': c_local,
            })

        candidates = []
        for di, det in enumerate(detections):
            c = det['centroid_track']
            for tid, tr in self.tracks.items():
                dist = np.linalg.norm(c[:2] - tr.centroid[:2])
                if dist < self.track_max_distance:
                    candidates.append((dist, di, tid))

        candidates.sort(key=lambda x: x[0])

        matched_det = set()
        matched_track = set()
        for dist, di, tid in candidates:
            if di in matched_det or tid in matched_track:
                continue
            matched_det.add(di)
            matched_track.add(tid)
            det = detections[di]
            tr = self.tracks[tid]
            tr.update(det['centroid_track'], stamp_sec,
                      enter_threshold=self.dynamic_enter_threshold,
                      exit_threshold=self.dynamic_exit_threshold,
                      timer_frames=self.dynamic_timer_frames)
            det['speed'] = tr.speed
            det['is_dynamic'] = (tr.is_dynamic and tr.hits >= self.min_track_hits)
            det['track_id'] = tid

        for di, det in enumerate(detections):
            if di in matched_det:
                continue
            nt = Track(det['centroid_track'], stamp_sec)
            self.tracks[nt.id] = nt
            det['speed'] = 0.0
            det['is_dynamic'] = False
            det['track_id'] = nt.id

        for tid in [t for t, tr in self.tracks.items() if (stamp_sec - tr.last_update) > self.track_timeout]:
            del self.tracks[tid]

        self.active_tracks = detections

        if self.lidar_frame_count % 30 == 0:
            self.get_logger().info(
                f'Lidar F: {self.lidar_frame_count} | Kume Sayisi: {len(clusters)} | Aktif Takip Sayisi: {len(self.tracks)}'
            )

    def _purge_stale_tracks(self, stamp_sec):
        for tid in [t for t, tr in self.tracks.items()
                    if (stamp_sec - tr.last_update) > self.stale_track_timeout]:
            del self.tracks[tid]

    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        if not self.camera_info_received:
            self.get_logger().info(
                f'Kamera Kalibrasyon Parametreleri Alindi -> fx: {self.fx:.2f}, fy: {self.fy:.2f}, '
                f'cx: {self.cx:.2f}, cy: {self.cy:.2f}'
            )
            self.camera_info_received = True

    def rgb_callback(self, msg: Image):
        if self.fx is None:
            self.get_logger().warn('Kamera kalibrasyon parametreleri bekleniyor...', throttle_duration_sec=5.0)
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Goruntu alinamadi: {e}")
            return

        h_img, w_img = img.shape[:2]

        current_tracks = list(self.active_tracks)
        if not current_tracks:
            cv2.imshow("Otonom Surus Kamerasi", img)
            cv2.waitKey(1)
            return

        tf_base_to_cam = self.lookup_transform(self.camera_frame, self.output_frame, msg.header.stamp)
        if tf_base_to_cam is None:
            self.get_logger().warn(
                f'Projeksiyon icin TF bulunamadi ({self.output_frame} -> {self.camera_frame})!',
                throttle_duration_sec=5.0
            )
            cv2.imshow("Otonom Surus Kamerasi", img)
            cv2.waitKey(1)
            return
        M_base_to_cam = self.transform_to_matrix(tf_base_to_cam)

        for det in current_tracks:
            c = det['centroid_base']
            s = det['size']

            corners_3d = []
            for dx in [-1, 1]:
                for dy in [-1, 1]:
                    for dz in [-1, 1]:
                        corners_3d.append([
                            c[0] + dx * s[0] / 2.0,
                            c[1] + dy * s[1] / 2.0,
                            c[2] + dz * s[2] / 2.0,
                            1.0
                        ])
            corners_3d = np.array(corners_3d)

            corners_cam = (M_base_to_cam @ corners_3d.T).T[:, :3]

            pixels = []
            for p_cam in corners_cam:
                if p_cam[0] > 0.1:
                    u = int((-p_cam[1] / p_cam[0]) * self.fx + self.cx)
                    v = int((-p_cam[2] / p_cam[0]) * self.fy + self.cy)
                    pixels.append((u, v))

            for p in pixels:
                cv2.circle(img, p, 3, (0, 255, 255), -1)

            if len(pixels) >= 2:
                min_u = max(0, min(p[0] for p in pixels))
                max_u = min(w_img, max(p[0] for p in pixels))
                min_v = max(0, min(p[1] for p in pixels))
                max_v = min(h_img, max(p[1] for p in pixels))

                if max_u - min_u > 2 and max_v - min_v > 2 and min_u < w_img and max_u > 0:

                    if det.get('is_dynamic', False):
                        color = (0, 0, 255)
                        label = f"DINAMIK {det['speed']:.1f} m/s"
                    else:
                        color = (0, 255, 255)
                        label = "STATIK"

                    cv2.rectangle(img, (min_u, min_v), (max_u, max_v), color, 3)

                    (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(img, (min_u, min_v - lh - 10), (min_u + lw, min_v), color, -1)
                    cv2.putText(img, label, (min_u, min_v - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

        cv2.imshow("Otonom Surus Kamerasi", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
