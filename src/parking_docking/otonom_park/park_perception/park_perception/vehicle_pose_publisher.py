import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
import tf2_ros


class VehiclePosePublisher(Node):
    def __init__(self):
        super().__init__('vehicle_pose_publisher')
        # Frame isimleri (sizin sisteme göre)
        self.declare_parameter('target_frame', 'map')      # referans (harita)
        self.declare_parameter('vehicle_frame', 'chassis') # araç gövdesi

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, '/vehicle_pose', 10)
        self.timer = self.create_timer(0.1, self.publish_pose)  # 10 Hz (sürekli)

        self.get_logger().info('vehicle_pose_publisher basladi. map->chassis dinleniyor.')

    def publish_pose(self):
        target = self.get_parameter('target_frame').value
        vehicle = self.get_parameter('vehicle_frame').value
        try:
            t = self.tf_buffer.lookup_transform(
                target, vehicle, rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'TF (map->chassis) bekleniyor: {e}', throttle_duration_sec=2.0)
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = target
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = 0.0
        msg.pose.orientation = t.transform.rotation   # quaternion direkt
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VehiclePosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()