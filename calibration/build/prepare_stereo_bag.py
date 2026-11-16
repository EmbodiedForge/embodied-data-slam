#!/usr/bin/env python3
import argparse, csv
from pathlib import Path
import cv2
import rosbag, rospy
from cv_bridge import CvBridge

ap = argparse.ArgumentParser()
ap.add_argument('--dataset', type=Path, required=True)
ap.add_argument('--output', type=Path, required=True)
ap.add_argument('--stride', type=int, default=10)
args = ap.parse_args()
csv_path = args.dataset / 'umi_video_timestamps.csv'
rows = list(csv.DictReader(csv_path.open(newline='')))
cap = cv2.VideoCapture(str(args.dataset / 'stereo.mkv'))
if not cap.isOpened(): raise RuntimeError('cannot open stereo.mkv')
args.output.parent.mkdir(parents=True, exist_ok=True)
bridge = CvBridge(); count = 0; used = 0
with rosbag.Bag(str(args.output), 'w') as bag:
    while True:
        ok, frame = cap.read()
        if not ok or count >= len(rows): break
        row = rows[count]; count += 1
        if (count - 1) % args.stride: continue
        if frame.shape[1] != 2560 or frame.shape[0] != 720:
            raise RuntimeError(f'unexpected frame shape {frame.shape}')
        stamp = rospy.Time.from_sec(float(row['driver_ts_ns']) / 1e9)
        for topic, image, fid in [('/cam0/image_raw', frame[:, :1280], 'cam0'),
                                  ('/cam1/image_raw', frame[:, 1280:], 'cam1')]:
            msg = bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), encoding='mono8')
            msg.header.stamp = stamp; msg.header.frame_id = fid
            bag.write(topic, msg, stamp)
        used += 1
cap.release()
print(f'frames_read={count} pairs_written={used} output={args.output}')
