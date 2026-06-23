import numpy as np, rclpy, cv2
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
def pc2_to_xyz(c):
    dt=np.dtype({'names':[f.name for f in c.fields],'formats':[
        {1:np.int8,2:np.uint8,3:np.int16,4:np.uint16,5:np.int32,6:np.uint32,7:np.float32,8:np.float64}[f.datatype]
        for f in c.fields],'offsets':[f.offset for f in c.fields],'itemsize':c.point_step})
    a=np.frombuffer(c.data,dtype=dt);return np.column_stack((a['x'],a['y'],a['z'])).astype(np.float32)
def xyzi(p,h):
    m=PointCloud2();m.header=h;m.height=1;m.width=p.shape[0]
    m.fields=[PointField(name=n,offset=o,datatype=7,count=1) for n,o in [('x',0),('y',4),('z',8),('intensity',12)]]
    m.is_bigendian=False;m.point_step=16;m.row_step=16*p.shape[0];m.is_dense=True
    m.data=p.astype(np.float32).tobytes();return m
class N(Node):
    def __init__(s):
        super().__init__('intensity_generator_node')
        s.fx=448.13386;s.fy=448.13386;s.cx=640.5;s.cy=360.5
        s.declare_parameter('dy',0.12);s.declare_parameter('dz',-0.1)
        s.declare_parameter('white_v',215);s.declare_parameter('white_s',50);s.declare_parameter('dilate',9)
        s.dy=float(s.get_parameter('dy').value);s.dz=float(s.get_parameter('dz').value)
        s.wv=int(s.get_parameter('white_v').value);s.ws=int(s.get_parameter('white_s').value)
        s.dl=int(s.get_parameter('dilate').value)
        s.br=CvBridge();s.img=None
        s.create_subscription(Image,'/camera/image_raw',s.icb,qos_profile_sensor_data)
        s.create_subscription(PointCloud2,'/lidar/points',s.ccb,qos_profile_sensor_data)
        s.pub=s.create_publisher(PointCloud2,'/lidar/points_with_intensity',qos_profile_sensor_data)
        s.dbg=s.create_publisher(Image,'/fusion_debug',qos_profile_sensor_data)
        s.get_logger().info('fuzyon v2 (parlak+dilate) basladi.')
    def icb(s,m):
        try:s.img=s.br.imgmsg_to_cv2(m,'bgr8')
        except Exception as e:s.get_logger().warn(str(e))
    def ccb(s,m):
        pts=pc2_to_xyz(m);Np=pts.shape[0];out=np.zeros((Np,4),np.float32);out[:,:3]=pts
        if s.img is None or Np==0:s.pub.publish(xyzi(out,m.header));return
        H,W=s.img.shape[:2];z=pts[:,0];xo=-(pts[:,1]+s.dy);yo=-(pts[:,2]+s.dz)
        val=z>0.3;u=np.full(Np,-1.0);v=np.full(Np,-1.0)
        u[val]=s.fx*xo[val]/z[val]+s.cx;v[val]=s.fy*yo[val]/z[val]+s.cy
        ui=u.astype(np.int32);vi=v.astype(np.int32)
        inb=val&(ui>=0)&(ui<W)&(vi>=0)&(vi<H)
        hsv=cv2.cvtColor(s.img,cv2.COLOR_BGR2HSV)
        white=((hsv[:,:,2]>s.wv)&(hsv[:,:,1]<s.ws)).astype(np.uint8)
        if s.dl>1:white=cv2.dilate(white,np.ones((s.dl,s.dl),np.uint8))
        white=white.astype(bool)
        idx=np.where(inb)[0];hit=white[vi[idx],ui[idx]]
        out[idx[hit],3]=255.0
        s.get_logger().info(f'z_s:[{pts[:,2].min():.2f},{pts[:,2].max():.2f}] in-img:{int(inb.sum())} marked:{int(hit.sum())}',throttle_duration_sec=1.0)
        d=s.img.copy();d[vi[idx],ui[idx]]=(0,0,255)
        if hit.any():d[vi[idx[hit]],ui[idx[hit]]]=(0,255,0)
        s.dbg.publish(s.br.cv2_to_imgmsg(d,'bgr8'));s.pub.publish(xyzi(out,m.header))
def main(a=None):
    rclpy.init(args=a);n=N()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
