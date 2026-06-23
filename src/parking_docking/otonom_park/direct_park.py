import rclpy, math
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int8
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def norm(a): return math.atan2(math.sin(a),math.cos(a))
def med(v):
    s=sorted(v); n=len(s); return s[n//2] if n%2 else 0.5*(s[n//2-1]+s[n//2])
class D(Node):
    def __init__(self):
        super().__init__("direct_park")
        self.odom=None; self.slot=None; self.safe=0; self.done=False
        self.buf=[]; self.no=0; self.ns=0
        rl=QoSProfile(depth=10)
        self.create_subscription(Odometry,"/odom",self.o,rl)
        self.create_subscription(PoseStamped,"/selected_slot_pose",self.s,rl)
        self.create_subscription(Int8,"/safety_state",self.sf,rl)
        self.pub=self.create_publisher(Twist,"/cmd_vel",10)
        self.create_timer(0.1,self.loop)
        self.create_timer(2.0,self.diag)
        self.get_logger().info("direct_park basladi")
    def diag(self): self.get_logger().info(f"SAYAC odom={self.no} slot={self.ns} safe={self.safe} kilit={self.slot}")
    def o(self,m): self.no+=1; self.odom=m
    def s(self,m):
        self.ns+=1
        if self.slot is not None: return
        self.buf.append((m.pose.position.x,m.pose.position.y))
        if len(self.buf)>=5:
            self.slot=(med([b[0] for b in self.buf]),med([b[1] for b in self.buf]))
            self.get_logger().info(f"CEP KILITLENDI: ({self.slot[0]:.2f},{self.slot[1]:.2f})")
    def sf(self,m): self.safe=m.data
    def loop(self):
        if self.odom is None or self.slot is None: return
        cmd=Twist()
        if self.done or self.safe==2:
            self.done=True
            self.get_logger().info("GUVENLIK/PARK DUR",throttle_duration_sec=2.0)
            self.pub.publish(cmd); return
        cx=self.odom.pose.pose.position.x; cy=self.odom.pose.pose.position.y
        cyaw=yaw(self.odom.pose.pose.orientation); sx,sy=self.slot
        dx=sx-cx; dy=sy-cy; d=math.hypot(dx,dy)
        if d<0.4:
            self.done=True; self.get_logger().info(f"PARK TAMAM mesafe={d:.2f}m -> DUR")
        else:
            e=norm(math.atan2(dy,dx)-cyaw)
            cmd.linear.x=0.3; cmd.angular.z=max(-0.6,min(0.6,1.5*e))
            self.get_logger().info(f"mesafe={d:.2f}m err={math.degrees(e):.0f} arac=({cx:.1f},{cy:.1f})",throttle_duration_sec=0.5)
        self.pub.publish(cmd)
rclpy.init(); rclpy.spin(D())
