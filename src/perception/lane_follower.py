#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # Zed Kamera sol resim aboneliği
        self.subscription = self.create_subscription(
            Image,
            '/zed_cam/left/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # Kararlı viraj dönüşleri için hassaslaştırılmış PID katsayıları
        self.declare_parameter('kp', 0.0050)
        self.declare_parameter('ki', 0.00003)
        self.declare_parameter('kd', 0.0090)
        self.declare_parameter('max_speed', 1.8)
        self.declare_parameter('min_speed', 0.45)
        self.declare_parameter('debug', False)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.debug_mode = self.get_parameter('debug').value

        self.get_logger().info(
            f'Parametreler Yuklendi -> Kp: {self.kp:.5f}, Ki: {self.ki:.5f}, Kd: {self.kd:.5f}, '
            f'Max Speed: {self.max_speed:.2f} m/s, Min Speed: {self.min_speed:.2f} m/s, Debug: {self.debug_mode}'
        )

        self.prev_error = 0.0
        self.integral = 0.0
        self.current_steering = 0.0

        self.max_steer = 0.6458
        self.max_steer_step = 0.045

        self.last_left_x = {'near': None, 'mid': None, 'far': None}
        self.last_right_x = {'near': None, 'mid': None, 'far': None}

        self.roi_top_ratio = 0.55
        self.roi_bottom_ratio = 0.95

        self.frame_counter = 0
        self.get_logger().info('Gelistirilmis Sag Serit Kilitlemeli Kontrolcu Dugumu baslatildi.')

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Hatasi: {e}')
            return

        self.frame_counter += 1
        height, width, _ = cv_image.shape
        img_center_x = width / 2.0

        roi_start_y = int(height * self.roi_top_ratio)
        roi_end_y = int(height * self.roi_bottom_ratio)
        roi = cv_image[roi_start_y:roi_end_y, :]
        roi_h = roi_end_y - roi_start_y

        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 40, 40])
        upper_yellow = np.array([36, 255, 255])
        mask_yellow = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)

        lower_white = np.array([0, 0, 180])
        upper_white = np.array([180, 50, 255])
        mask_white = cv2.inRange(hsv_roi, lower_white, upper_white)

        mask_combined = cv2.bitwise_or(mask_yellow, mask_white)
        mask_combined = cv2.GaussianBlur(mask_combined, (5, 5), 0)

        y_near = int(roi_h * 0.85)
        y_mid = int(roi_h * 0.60)
        y_far = int(roi_h * 0.30)

        center_near = self.process_slice(mask_yellow, mask_white, mask_combined, y_near, img_center_x, 'near')
        center_mid = self.process_slice(mask_yellow, mask_white, mask_combined, y_mid, img_center_x, 'mid')
        center_far = self.process_slice(mask_yellow, mask_white, mask_combined, y_far, img_center_x, 'far')

        steer_ratio = abs(self.current_steering) / self.max_steer
        w_far = 0.15 + 0.15 * steer_ratio
        w_mid = 0.35
        w_near = 1.0 - w_far - w_mid

        valid_centers = []
        weights = []

        if center_near is not None:
            valid_centers.append(center_near)
            weights.append(w_near)
        if center_mid is not None:
            valid_centers.append(center_mid)
            weights.append(w_mid)
        if center_far is not None:
            valid_centers.append(center_far)
            weights.append(w_far)

        if len(valid_centers) > 0:
            weights = np.array(weights) / np.sum(weights)
            target_x = sum(c * w for c, w in zip(valid_centers, weights))
            error = img_center_x - target_x
        else:
            error = self.prev_error
            self.get_logger().warn('Seritler tamamen kayboldu!')

        self.integral += error
        self.integral = max(min(self.integral, 40.0), -40.0)

        derivative = error - self.prev_error
        self.prev_error = error

        target_steering = self.kp * error + self.ki * self.integral + self.kd * derivative
        target_steering = max(min(target_steering, self.max_steer), -self.max_steer)

        steer_diff = target_steering - self.current_steering
        steer_diff = max(min(steer_diff, self.max_steer_step), -self.max_steer_step)
        self.current_steering += steer_diff

        steer_factor = abs(self.current_steering) / self.max_steer
        speed = self.max_speed * (1.0 - 0.72 * steer_factor)
        speed = max(speed, self.min_speed)

        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = self.current_steering
        self.publisher.publish(twist)

        if self.frame_counter % 50 == 0:
            self.get_logger().info(
                f'F: {self.frame_counter} | Err: {error:.1f} | Steer: {self.current_steering:.3f} rad | Speed: {speed:.2f} m/s'
            )
            self.get_logger().info(
                f'Dilim Durumlari -> Near: {"Bulundu" if center_near is not None else "Kayip"}, '
                f'Mid: {"Bulundu" if center_mid is not None else "Kayip"}, '
                f'Far: {"Bulundu" if center_far is not None else "Kayip"}'
            )

            if self.debug_mode:
                debug_img = roi.copy()
                cv2.line(debug_img, (0, y_near), (width, y_near), (255, 0, 0), 1)
                cv2.line(debug_img, (0, y_mid), (width, y_mid), (0, 255, 0), 1)
                cv2.line(debug_img, (0, y_far), (width, y_far), (0, 0, 255), 1)
                if center_near is not None: cv2.circle(debug_img, (int(center_near), y_near), 5, (255, 0, 0), -1)
                if center_mid is not None: cv2.circle(debug_img, (int(center_mid), y_mid), 5, (0, 255, 0), -1)
                if center_far is not None: cv2.circle(debug_img, (int(center_far), y_far), 5, (0, 0, 255), -1)
                cv2.imwrite(f'/tmp/lane_debug_{self.frame_counter}.jpg', debug_img)
                self.get_logger().info(f'Debug gorseli kaydedildi: /tmp/lane_debug_{self.frame_counter}.jpg')

    def process_slice(self, mask_yellow, mask_white, mask_combined, y_row, img_center_x, level):
        roi_h = mask_combined.shape[0]
        slice_ratio = y_row / float(roi_h)
        estimated_half_width = 165.0 * slice_ratio + 35.0

        yellow_pixels = np.where(mask_yellow[y_row, :] > 0)[0]
        white_pixels = np.where(mask_white[y_row, :] > 0)[0]

        left_x = None
        right_x = None

        yellow_clusters = []
        if len(yellow_pixels) > 0:
            diffs_y = np.diff(yellow_pixels)
            gaps_y = np.where(diffs_y > 45)[0]
            if len(gaps_y) == 0:
                yellow_clusters.append(np.mean(yellow_pixels))
            else:
                start_y = 0
                for gap in gaps_y:
                    end_y = gap + 1
                    yellow_clusters.append(np.mean(yellow_pixels[start_y:end_y]))
                    start_y = end_y
                yellow_clusters.append(np.mean(yellow_pixels[start_y:]))

        white_clusters = []
        if len(white_pixels) > 0:
            diffs_w = np.diff(white_pixels)
            gaps_w = np.where(diffs_w > 45)[0]
            if len(gaps_w) == 0:
                white_clusters.append(np.mean(white_pixels))
            else:
                start_w = 0
                for gap in gaps_w:
                    end_w = gap + 1
                    white_clusters.append(np.mean(white_pixels[start_w:end_w]))
                    start_w = end_w
                white_clusters.append(np.mean(white_pixels[start_w:]))

        if len(yellow_clusters) > 0:
            left_x = yellow_clusters[0]

            right_candidates = [w for w in white_clusters if w > left_x]
            if len(right_candidates) > 0:
                right_x = min(right_candidates)
            else:
                last_r = self.last_right_x[level]
                if last_r is not None and len(white_clusters) > 0:
                    right_x = min(white_clusters, key=lambda w: abs(w - last_r))

        else:
            all_clusters = sorted(white_clusters)
            if len(all_clusters) == 0:
                return None

            last_l = self.last_left_x[level]
            last_r = self.last_right_x[level]

            if last_l is not None and last_r is not None:
                best_l_dist = float('inf')
                best_r_dist = float('inf')

                for c in all_clusters:
                    dist_l = abs(c - last_l)
                    dist_r = abs(c - last_r)

                    if dist_l < best_l_dist and dist_l < 150.0:
                        best_l_dist = dist_l
                        left_x = c
                    if dist_r < best_r_dist and dist_r < 150.0:
                        best_r_dist = dist_r
                        right_x = c

                if left_x == right_x and left_x is not None:
                    if best_l_dist < best_r_dist:
                        right_x = None
                    else:
                        left_x = None
            else:
                if len(all_clusters) == 1:
                    c = all_clusters[0]
                    if c < img_center_x:
                        left_x = c
                    else:
                        right_x = c
                elif len(all_clusters) == 2:
                    left_x = all_clusters[0]
                    right_x = all_clusters[1]
                else:
                    left_candidates = [x for x in all_clusters if x < img_center_x]
                    right_candidates = [x for x in all_clusters if x >= img_center_x]
                    if len(left_candidates) > 0:
                        left_x = max(left_candidates)
                    if len(right_candidates) > 0:
                        right_x = min(right_candidates)

        if left_x is not None:
            self.last_left_x[level] = left_x
        if right_x is not None:
            self.last_right_x[level] = right_x

        if left_x is not None and right_x is not None:
            return (left_x + right_x) / 2.0
        elif left_x is not None:
            return left_x + estimated_half_width
        elif right_x is not None:
            return right_x - estimated_half_width

        return None

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
