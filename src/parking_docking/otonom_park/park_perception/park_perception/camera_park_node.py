"""Kamera tabanli park dugumu - sadece goruntuyle park (odometry/LiDAR GEREKMEZ)."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraParkNode(Node):
    def __init__(self):
        super().__init__('camera_park_node')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('safety_topic', '/safety_state')

        # Cep rengi (HSV). Varsayilan MAVI; cep farkli renkse degistir.
        self.declare_parameter('hsv_lower', [100, 80, 40])
        self.declare_parameter('hsv_upper', [130, 255, 255])

        self.declare_parameter('detect_area_ratio', 0.02)
        self.declare_parameter('park_area_ratio', 0.32)
        self.declare_parameter('center_tol', 0.10)
        self.declare_parameter('stop_frames', 5)
        self.declare_parameter('lost_frames_limit', 15)

        self.declare_parameter('search_speed', 0.45)
        self.declare_parameter('approach_speed', 0.60)
        self.declare_parameter('creep_speed', 0.35)
        self.declare_parameter('steer_gain', 1.0)
        self.declare_parameter('max_steer', 1.0)

        gp = self.get_parameter
        self.lower = np.array(gp('hsv_lower').value, dtype=np.uint8)
        self.upper = np.array(gp('hsv_upper').value, dtype=np.uint8)
        self.detect_area = float(gp('detect_area_ratio').value)
        self.park_area = float(gp('park_area_ratio').value)
        self.center_tol = float(gp('center_tol').value)
        self.stop_frames = int(gp('stop_frames').value)
        self.lost_limit = int(gp('lost_frames_limit').value)
        self.search_speed = float(gp('search_speed').value)
        self.approach_speed = float(gp('approach_speed').value)
        self.creep_speed = float(gp('creep_speed').value)
        self.steer_gain = float(gp('steer_gain').value)
        self.max_steer = float(gp('max_steer').value)

        self.bridge = CvBridge()
        self.safety_state = 0
        self.state = 'SEARCH'
        self.parked = False
        self.stop_counter = 0
        self.lost_counter = 0

        self.create_subscription(Image, gp('image_topic').value, self.image_cb, 10)
        self.create_subscription(Int8, gp('safety_topic').value, self.safety_cb, 10)
        self.pub = self.create_publisher(Twist, gp('cmd_topic').value, 10)
        self.park_active_pub = self.create_publisher(Bool, '/park_active', 10)
        self.get_logger().info(
            f"camera_park_node basladi. Goruntu={gp('image_topic').value} -> {gp('cmd_topic').value}"
        )

    def safety_cb(self, msg):
        self.safety_state = msg.data

    def detect_slot(self, img):
        h, w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 0.0, 0.0

        largest = max(contours, key=cv2.contourArea)
        area_ratio = cv2.contourArea(largest) / float(h * w)
        if area_ratio < self.detect_area:
            return False, 0.0, area_ratio

        M = cv2.moments(largest)
        cx = M['m10'] / M['m00'] if M['m00'] > 0 else w / 2.0
        error = (cx - w / 2.0) / (w / 2.0)
        return True, error, area_ratio

    def image_cb(self, msg):
        if self.parked:
            self.park_active_pub.publish(Bool(data=True))
            self.pub.publish(Twist())
            return
        if self.safety_state == 2:
            self.park_active_pub.publish(Bool(data=True))
            self.pub.publish(Twist())
            self.get_logger().warn('GUVENLIK DUR!', throttle_duration_sec=1.0)
            return

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        seen, error, area = self.detect_slot(img)

        cmd = Twist()

        if self.state == 'SEARCH':
            if seen:
                self.state = 'APPROACH'
                self.lost_counter = 0
                self.get_logger().info('Cep bulundu -> APPROACH')
                self.park_active_pub.publish(Bool(data=True))
            else:
                self.park_active_pub.publish(Bool(data=False))
                return

        elif self.state == 'APPROACH':
            self.park_active_pub.publish(Bool(data=True))
            if not seen:
                self.lost_counter += 1
                cmd.linear.x = max(self.creep_speed * 0.5, 0.35)
                if self.lost_counter > self.lost_limit:
                    self.state = 'SEARCH'
                    self.stop_counter = 0
                    self.get_logger().warn('Cep kayboldu -> SEARCH')
                self.pub.publish(cmd)
                return

            self.lost_counter = 0
            aligned = abs(error) <= self.center_tol
            close = area >= self.park_area

            if close and aligned:
                self.stop_counter += 1
                cmd.linear.x = self.creep_speed
                cmd.angular.z = float(np.clip(-error * self.steer_gain,
                                              -self.max_steer, self.max_steer))
                if self.stop_counter >= self.stop_frames:
                    self.parked = True
                    self.state = 'PARKED'
                    self.pub.publish(Twist())
                    self.get_logger().info('PARK TAMAMLANDI -> DUR')
                    return
            else:
                self.stop_counter = 0
                cmd.angular.z = float(np.clip(-error * self.steer_gain,
                                              -self.max_steer, self.max_steer))
                dist_factor = max(0.0, 1.0 - area / self.park_area)
                speed = self.creep_speed + (self.approach_speed - self.creep_speed) * dist_factor
                speed *= max(0.4, 1.0 - abs(error))
                cmd.linear.x = speed

        if self.safety_state == 1:
            cmd.linear.x *= 0.5

        # Gazebo fizik motoru sürtünmesini aşmak için min hız limiti
        if abs(cmd.linear.x) > 0.01:
            cmd.linear.x = max(cmd.linear.x, 0.38)

        self.pub.publish(cmd)

        if seen:
            self.get_logger().info(
                f'{self.state} | sapma={error:+.2f} | alan={area:.2f} | hiz={cmd.linear.x:.2f}',
                throttle_duration_sec=0.5,
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraParkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
