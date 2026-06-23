import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectory
from std_msgs.msg import Int8


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class ParkControllerNode(Node):
    def __init__(self):
        super().__init__('park_controller_node')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('reverse_steer_sign', 1.0)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.traj = None
        self.pose = None
        self.safety_state = 0

        self.create_subscription(MultiDOFJointTrajectory, '/planning/trajectory', self.traj_cb, 10)
        self.create_subscription(Odometry, '/localization/odometry', self.odom_cb, qos_profile_sensor_data)
        self.create_subscription(Int8, '/safety_state', self.safety_cb, 10)

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f'park_controller_node basladi. Komut -> {cmd_topic}')

    def traj_cb(self, msg):
        pts = []
        for p in msg.points:
            if not p.transforms:
                continue
            tr = p.transforms[0].translation
            v = p.velocities[0].linear.x if p.velocities else 0.0
            pts.append((tr.x, tr.y, v))
        self.traj = pts

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def safety_cb(self, msg):
        self.safety_state = msg.data

    def stop(self):
        self.pub.publish(Twist())

    def control_loop(self):
        if self.safety_state == 2:
            self.stop(); return
        if self.traj is None or self.pose is None or len(self.traj) < 1:
            self.stop(); return

        x = self.pose.position.x
        y = self.pose.position.y
        yaw = quat_to_yaw(self.pose.orientation)
        pts = np.array([(p[0], p[1]) for p in self.traj])
        vels = np.array([p[2] for p in self.traj])

        goal = pts[-1]
        if math.hypot(goal[0] - x, goal[1] - y) < self.get_parameter('goal_tolerance').value:
            self.stop()
            self.get_logger().info('Hedefe varildi -> DUR', throttle_duration_sec=2.0)
            return

        i = int(np.argmin(np.hypot(pts[:, 0] - x, pts[:, 1] - y)))

        v_plan = float(vels[i])
        if abs(v_plan) < 1e-3:
            self.stop(); return          # HOLD (planlayıcı 0 hız verdi)
        direction = 1.0 if v_plan > 0 else -1.0   # YÖN: planlayıcı hız işaretinden

        Ld = self.get_parameter('lookahead').value
        target = goal
        for k in range(i, len(pts)):
            if math.hypot(pts[k][0] - x, pts[k][1] - y) >= Ld:
                target = pts[k]; break

        dx, dy = target[0] - x, target[1] - y
        y_l = -math.sin(yaw) * dx + math.cos(yaw) * dy
        Ld_eff = math.hypot(dx, dy)
        if Ld_eff < 1e-3:
            self.stop(); return
        kappa = 2.0 * y_l / (Ld_eff * Ld_eff)

        speed = min(abs(v_plan), self.get_parameter('max_speed').value)
        if self.safety_state == 1:
            speed *= 0.5
        speed = max(speed, 0.05)

        v_signed = direction * speed
        omega = v_signed * kappa
        if direction < 0:
            omega *= self.get_parameter('reverse_steer_sign').value

        cmd = Twist()
        cmd.linear.x = float(v_signed)
        cmd.angular.z = float(omega)
        self.pub.publish(cmd)
        self.get_logger().info(
            f'{"ILERI" if direction > 0 else "GERI"} v={v_signed:.2f} w={omega:.2f}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ParkControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()