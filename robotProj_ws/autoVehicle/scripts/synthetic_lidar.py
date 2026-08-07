#!/usr/bin/env python3
"""
Synthetic LiDAR — accurate course_test geometry.

- /scan (LaserScan):     horizontal wall scan (360 rays)
- /lidar_points (PointCloud2): dense floor surface grid (ring pattern, ~200 pts)

The floor scanner projects points onto the ground plane at the LiDAR's height.
As the robot moves up the 45° ramp, floor points trace the 3D surface.
"""
import math, struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from tf2_ros import Buffer, TransformListener, TransformException

HW = 0.15; RAMP_X0=3.2; RAMP_X1=4.3314; RAMP_Z0=0.0; RAMP_Z1=1.1314
CORR_END=5.78; JUNC_X=5.9314; JUNC_L=5.78; JUNC_R=6.08
JUNC_GAP=0.2; JUNC_MAX_Y=1.6; JUNC_END_Y=1.65
RNG_MIN=0.05; RNG_MAX=30.0

def floor_z(wx):
    if wx<RAMP_X0: return 0.0
    if wx<RAMP_X1: return RAMP_Z0+(wx-RAMP_X0)/(RAMP_X1-RAMP_X0)*(RAMP_Z1-RAMP_Z0)
    return RAMP_Z1

def raycast(ox, oy, angle):
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
        for end_y in (-JUNC_END_Y,JUNC_END_Y):
            t=(end_y-oy)/dy
            if RNG_MIN<t<best:
                ix=ox+t*dx
                if JUNC_L<=ix<=JUNC_R+0.01: best=t
    return float(best), float(ox+best*dx), float(oy+best*dy)

class SyntheticLidar(Node):
    def __init__(self):
        super().__init__('synthetic_lidar')
        self.tf_b=Buffer(); self.tf_l=TransformListener(self.tf_b,self)
        self.sp=self.create_publisher(LaserScan,'/scan',10)
        self.fp=self.create_publisher(PointCloud2,'/lidar_points',10)
        self._cnt=0
        self.get_logger().info('SyntheticLidar | 360 wall rays + floor surface @10Hz')
        self.timer=self.create_timer(0.1,self.tick)

    def tick(self):
        try:
            tf=self.tf_b.lookup_transform('odom','radar_link',
                rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=0.3))
        except TransformException: return
        lx=tf.transform.translation.x; ly=tf.transform.translation.y
        lz=tf.transform.translation.z; fz=floor_z(lx)

        # ── 1. Wall scan ─────────────────────────────────────────────
        s=LaserScan(); s.header.stamp=self.get_clock().now().to_msg()
        s.header.frame_id='radar_link'
        s.angle_min=-math.pi; s.angle_max=math.pi
        s.angle_increment=2*math.pi/360; s.range_min=RNG_MIN; s.range_max=RNG_MAX
        rng=[]; ang=s.angle_min
        for _ in range(360):
            r,_,_=raycast(lx,ly,ang)
            rng.append(r if r<RNG_MAX else float('nan'))
            ang+=s.angle_increment
        s.ranges=rng; self.sp.publish(s)

        # ── 2. Floor surface scan (dense ring pattern) ───────────────
        # Cast rays at various azimuths + downward angles to hit the floor.
        floor_pts=[]
        n_azimuth=24   # 24 directions around the robot
        n_range=8      # 8 distance steps per direction
        for ai in range(n_azimuth):
            az=ai*2*math.pi/n_azimuth
            dx=math.cos(az); dy=math.sin(az)
            for ri in range(1,n_range+1):
                dist=ri*0.5  # 0.5m, 1.0m, ..., 4.0m
                wx=lx+dx*dist; wy=ly+dy*dist; wz=floor_z(wx)
                # Validate: point must be within corridor/junction
                ok=True
                if wx<CORR_END:
                    if abs(wy)>HW: ok=False
                else:
                    if not(JUNC_L<=wx<=JUNC_R): ok=False
                    if abs(wy)>JUNC_MAX_Y: ok=False
                if ok:
                    floor_pts.append((wx-lx,wy-ly,wz-lz,0.7))

        pc=PointCloud2(); pc.header.stamp=self.get_clock().now().to_msg()
        pc.header.frame_id='radar_link'; pc.height=1; pc.width=len(floor_pts)
        pc.fields=[PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
                   PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
                   PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
                   PointField(name='intensity',offset=12,datatype=PointField.FLOAT32,count=1)]
        pc.is_bigendian=False; pc.point_step=16; pc.row_step=16*len(floor_pts)
        pc.is_dense=True
        pc.data=b''.join(struct.pack('<ffff',x,y,z,i) for(x,y,z,i) in floor_pts)
        self.fp.publish(pc)

        self._cnt+=1
        if self._cnt<=3:
            hits=sum(1 for r in rng if not math.isnan(r))
            self.get_logger().info(
                f'[SYNTH #{self._cnt}] ({lx:.2f},{ly:.2f},{lz:.2f}) '
                f'wall_hits={hits}/360 floor_pts={len(floor_pts)}')
        elif self._cnt%50==0:
            self.get_logger().info(
                f'[SYNTH] {self._cnt} | ({lx:.2f},{ly:.2f},{lz:.2f}) fl={len(floor_pts)}')

def main():
    rclpy.init(); rclpy.spin(SyntheticLidar())
if __name__=='__main__': main()
