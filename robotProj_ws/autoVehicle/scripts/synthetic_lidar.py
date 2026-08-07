#!/usr/bin/env python3
"""
Integrated Synthetic LiDAR + Point Cloud Mapper — single node.
No DDS between scanner and mapper — everything in-process.

Publishes:
  /scan          (LaserScan)      — live wall scan
  /pointcloud_map (PointCloud2)   — accumulated 3D map
  /lidar_points  (PointCloud2)    — live floor surface points

Service: /save_map (Trigger) → saves .pcd
"""
import math, struct, os, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException

HW=0.15; RAMP_X0=3.2; RAMP_X1=4.3314; RAMP_Z0=0.0; RAMP_Z1=1.1314
CORR_END=5.78; JUNC_X=5.9314; JUNC_L=5.78; JUNC_R=6.08
JUNC_GAP=0.2; JUNC_MAX_Y=1.6; JUNC_END_Y=1.65
RNG_MIN=0.05; RNG_MAX=30.0

def floor_z(wx):
    if wx<RAMP_X0: return 0.0
    if wx<RAMP_X1: return RAMP_Z0+(wx-RAMP_X0)/(RAMP_X1-RAMP_X0)*(RAMP_Z1-RAMP_Z0)
    return RAMP_Z1

def raycast(ox,oy,oz,angle):
    dx=math.cos(angle); dy=math.sin(angle); best=RNG_MAX
    if abs(dy)>1e-10:
        for wy_val in (-HW,HW):
            t=(wy_val-oy)/dy
            if RNG_MIN<t<best:
                ix=ox+t*dx
                if 0<=ix<=CORR_END+0.01: best=t
    if abs(dx)>1e-10:
        t=(JUNC_L-ox)/dx
        if RNG_MIN<t<best:
            iy=oy+t*dy
            if JUNC_GAP<=abs(iy)<=JUNC_MAX_Y+0.01: best=t
    if abs(dx)>1e-10:
        t=(JUNC_R-ox)/dx
        if RNG_MIN<t<best:
            iy=oy+t*dy
            if abs(iy)<=JUNC_MAX_Y+0.01: best=t
    if abs(dy)>1e-10:
        for ey in (-JUNC_END_Y,JUNC_END_Y):
            t=(ey-oy)/dy
            if RNG_MIN<t<best:
                ix=ox+t*dx
                if JUNC_L<=ix<=JUNC_R+0.01: best=t
    # ── Ramp surface (45°) intersection ──────────────────────────
    # Horizontal ray at height oz hits ramp when floor_z(wx) = oz.
    # Ramp: floor_z(x) = x - RAMP_X0  for x in [RAMP_X0, RAMP_X1]
    # Intersection at x = RAMP_X0 + oz, valid if oz in [0, RAMP_Z1]
    if RAMP_Z0 <= oz <= RAMP_Z1:
        rx = RAMP_X0 + oz
        if abs(dx) > 1e-10:
            t = (rx - ox) / dx
            if RNG_MIN < t < best:
                iy = oy + t * dy
                if abs(iy) <= HW + 0.01:  # within corridor width
                    best = t
    return float(best)

class IntegratedMapper(Node):
    def __init__(self):
        super().__init__('integrated_mapper')
        self.tf_b=Buffer(); self.tf_l=TransformListener(self.tf_b,self)
        self.sp=self.create_publisher(LaserScan,'/scan',10)
        self.fp=self.create_publisher(PointCloud2,'/lidar_points',10)
        self.mp=self.create_publisher(PointCloud2,'/pointcloud_map',10)
        self.save_dir=os.path.expanduser('~/pointcloud_maps')
        os.makedirs(self.save_dir,exist_ok=True)
        self.save_srv=self.create_service(Trigger,'/save_map',self.save_cb)
        self.map_pts=[]
        self._cnt=0
        self._sw=0
        self.get_logger().info('IntegratedMapper | scan+floor+map @10Hz')
        self.timer=self.create_timer(0.1,self.tick)
        self.map_timer=self.create_timer(0.5,self.publish_map)

    def tick(self):
        try:
            tf=self.tf_b.lookup_transform('odom','radar_link',
                rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.3))
        except TransformException: return
        lx=tf.transform.translation.x; ly=tf.transform.translation.y
        lz=tf.transform.translation.z; fz=floor_z(lx)
        # Rotation matrix
        qx=tf.transform.rotation.x; qy=tf.transform.rotation.y
        qz=tf.transform.rotation.z; qw=tf.transform.rotation.w
        xx,zz=qx*qx,qz*qz; yy=qy*qy
        xy,xz,yz=qx*qy,qx*qz,qy*qz
        wx,wy,wz=qw*qx,qw*qy,qw*qz
        r00=1.0-2.0*(yy+zz); r01=2.0*(xy-wz); r02=2.0*(xz+wy)
        r10=2.0*(xy+wz); r11=1.0-2.0*(xx+zz); r12=2.0*(yz-wx)
        r20=2.0*(xz-wy); r21=2.0*(yz+wx); r22=1.0-2.0*(xx+yy)

        # ── Wall scan ──────────────────────────────────────────────
        s=LaserScan(); s.header.stamp=self.get_clock().now().to_msg()
        s.header.frame_id='radar_link'
        s.angle_min=-math.pi; s.angle_max=math.pi
        s.angle_increment=2*math.pi/360; s.range_min=RNG_MIN; s.range_max=RNG_MAX
        rng=[]; ang=s.angle_min; wpts=[]
        for _ in range(360):
            # Transform ray direction from radar_link to world frame
            dx_local=math.cos(ang); dy_local=math.sin(ang)
            dx_world=r00*dx_local+r01*dy_local
            dy_world=r10*dx_local+r11*dy_local
            world_ang=math.atan2(dy_world,dx_world)
            r=raycast(lx,ly,lz,world_ang)
            if r<RNG_MAX:
                rng.append(r)
                px=r*math.cos(ang); py=r*math.sin(ang); pz=0.0
                wpts.append((r00*px+r01*py+r02*pz+lx, r10*px+r11*py+r12*pz+ly, r20*px+r21*py+r22*pz+lz, 0.4))
            else:
                rng.append(float('nan'))
            ang+=s.angle_increment
        s.ranges=rng; self.sp.publish(s)
        self.map_pts.extend(wpts)

        # ── Floor scan ─────────────────────────────────────────────
        fp_pts=[]
        fp_world=[]
        for ai in range(24):
            az=ai*2*math.pi/24; dx=math.cos(az); dy=math.sin(az)
            for ri in range(1,9):
                dist=ri*0.5; wx=lx+dx*dist; wy=ly+dy*dist; wz=floor_z(wx)
                ok=True
                if wx<CORR_END:
                    if abs(wy)>HW: ok=False
                else:
                    if not(JUNC_L<=wx<=JUNC_R): ok=False
                    if abs(wy)>JUNC_MAX_Y: ok=False
                if ok:
                    fp_pts.append((wx-lx,wy-ly,wz-lz,0.7))
                    fp_world.append((wx,wy,wz,0.7))
        pc=PointCloud2(); pc.header.stamp=self.get_clock().now().to_msg()
        pc.header.frame_id='radar_link'; pc.height=1
        pc.width=len(fp_pts)
        pc.fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
                   PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                   PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
                   PointField(name='intensity',offset=12,datatype=PointField.FLOAT32,count=1)]
        pc.is_bigendian=False; pc.point_step=16; pc.row_step=16*len(fp_pts)
        pc.is_dense=True
        pc.data=b''.join(struct.pack('<ffff',x,y,z,i) for(x,y,z,i) in fp_pts)
        self.fp.publish(pc)
        self.map_pts.extend(fp_world)

        self._cnt+=1
        if self._cnt<=3:
            hits=sum(1 for r in rng if not math.isnan(r))
            self.get_logger().info(
                f'[SCAN #{self._cnt}] ({lx:.2f},{ly:.2f},{lz:.2f}) '
                f'wall_hits={hits}/360 floor={len(fp_world)} map={len(self.map_pts)}')
        elif self._cnt%50==0:
            self._sw+=1
            self.get_logger().info(
                f'[MAP] {self._cnt}scans {len(self.map_pts)}pts '
                f'(saved {self._sw}x) @({lx:.2f},{ly:.2f},{lz:.2f})')

    def publish_map(self):
        if len(self.map_pts)<10: return
        pts=self.map_pts
        if len(pts)>200000: pts=pts[::max(1,len(pts)//200000)]
        h=Header(); h.stamp=self.get_clock().now().to_msg(); h.frame_id='odom'
        pc=PointCloud2(); pc.header=h; pc.height=1; pc.width=len(pts)
        pc.fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
                   PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                   PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
                   PointField(name='intensity',offset=12,datatype=PointField.FLOAT32,count=1)]
        pc.is_bigendian=False; pc.point_step=16; pc.row_step=16*len(pts)
        pc.is_dense=True
        pc.data=b''.join(struct.pack('<ffff',x,y,z,i) for(x,y,z,i) in pts)
        self.mp.publish(pc)

    def save_cb(self,req,rsp):
        if not self.map_pts: rsp.success=False; rsp.message='empty'; return rsp
        ts=time.strftime('%Y%m%d_%H%M%S')
        fn=os.path.join(self.save_dir,f'map_{ts}.pcd')
        try:
            with open(fn,'w') as f:
                f.write('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n')
                f.write('SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n')
                f.write(f'WIDTH {len(self.map_pts)}\nHEIGHT 1\n')
                f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
                f.write(f'POINTS {len(self.map_pts)}\nDATA ascii\n')
                for x,y,z,i in self.map_pts:
                    f.write(f'{x:.6f} {y:.6f} {z:.6f} {i:.4f}\n')
            self._sw+=1
            rsp.success=True; rsp.message=f'{len(self.map_pts)}pts -> {fn}'
        except Exception as e:
            rsp.success=False; rsp.message=str(e)
        return rsp

def main():
    rclpy.init(); rclpy.spin(IntegratedMapper())
if __name__=='__main__': main()
