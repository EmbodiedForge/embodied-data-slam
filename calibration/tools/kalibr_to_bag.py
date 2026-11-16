#!/usr/bin/env python3
"""Convert the current raw UMI capture to a calibration bag without intrinsics."""
import argparse
import csv
from pathlib import Path
import sys

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu

# Installed toolkit layout: calibration/tools/this.py, software/umi_slam/.
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'software/umi_slam'))
from pipeline import decode_legacy_imu, integer_ns, strictly_increasing


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stride',type=int,default=5); p.add_argument('--images-only',action='store_true')
    p.add_argument('--imu-hz',type=float,default=250)
    p.add_argument('--accel-range-g',type=float,default=4); p.add_argument('--gyro-range-dps',type=float,default=2000)
    p.add_argument('--max-frames',type=int,default=0,help='Diagnostic limit, 0 means full sequence')
    a=p.parse_args()
    if a.stride<1 or a.max_frames<0: p.error('Invalid stride or frame limit')
    if a.output.exists(): p.error('Output already exists')
    with (a.input/'umi_video_timestamps.csv').open() as f: times=[integer_ns(r['driver_ts_ns']) for r in csv.DictReader(f)]
    strictly_increasing(times,'Camera')
    chosen=list(range(0,len(times),a.stride))
    if a.max_frames: chosen=chosen[:a.max_frames]
    if len(chosen)<2: p.error('Need at least two image pairs')
    it,iv=([],[]) if a.images_only else decode_legacy_imu(a.input/'umi_imu_timestamps.bin',a.imu_hz,a.accel_range_g,a.gyro_range_dps)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    temporary=a.output.with_name(a.output.name+'.partial')
    if temporary.exists(): p.error('Partial output already exists; choose a new output name')
    cap=cv2.VideoCapture(str(a.input/'stereo.mkv')); bridge=CvBridge(); count=0; imu_count=0
    def stamp(ns): return rospy.Time(ns//1000000000,ns%1000000000)
    try:
        with rosbag.Bag(str(temporary),'w') as bag:
            for t,v in zip(it,iv):
                if not times[chosen[0]]-100000000 <= t <= times[chosen[-1]]+100000000: continue
                msg=Imu(); msg.header.stamp=stamp(t); msg.header.frame_id='imu0'
                msg.orientation_covariance[0]=-1
                msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z=v[:3]
                msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z=v[3:]
                bag.write('/imu0',msg,msg.header.stamp); imu_count+=1
            selected=set(chosen); shape=None
            for i in range(chosen[-1]+1):
                ok,image=cap.read()
                if not ok: raise ValueError('Video shorter than timestamp list')
                if shape is None: shape=image.shape[:2]
                if image.shape[:2]!=shape or image.shape[1]%2: raise ValueError('Invalid stereo image dimensions')
                if i not in selected: continue
                half=image.shape[1]//2
                for c,im in enumerate((image[:,:half],image[:,half:])):
                    msg=bridge.cv2_to_imgmsg(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),encoding='mono8')
                    msg.header.stamp=stamp(times[i]); msg.header.frame_id=f'cam{c}'
                    bag.write(f'/cam{c}/image_raw',msg,msg.header.stamp)
                count+=1
        if not a.images_only and imu_count<2: raise ValueError('IMU does not overlap the video')
        temporary.rename(a.output)
    finally: cap.release()
    print(f'Wrote {count} stereo pairs and {imu_count} IMU samples: {a.output}')
    if not a.images_only: print('IMU times use legacy UART uniform reconstruction; timing accuracy is not independently verified.')


if __name__=='__main__': main()
