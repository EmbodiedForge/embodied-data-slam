#!/usr/bin/env python3
"""Convert Kalibr pinhole-radtan stereo-IMU calibration to this ORB-SLAM3 build."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml


def rigid(value):
    m=np.asarray(value,dtype=float)
    if m.shape!=(4,4) or not np.isfinite(m).all() or not np.allclose(m[3],[0,0,0,1]): raise ValueError('Invalid 4x4 transform')
    if not np.allclose(m[:3,:3].T@m[:3,:3],np.eye(3),atol=1e-4) or not np.isclose(np.linalg.det(m[:3,:3]),1,atol=1e-4): raise ValueError('Invalid rotation')
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cam',type=Path,required=True); p.add_argument('--imu',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True); p.add_argument('--fps',type=int,default=25)
    p.add_argument('--capture-profile',type=Path,help='Optional raw device protocol/units profile to preserve')
    a=p.parse_args()
    if a.fps<1: p.error('fps must be positive')
    cam=yaml.safe_load(a.cam.read_text()); imu=yaml.safe_load(a.imu.read_text()); imu=imu.get('imu0',imu)
    if any(c not in cam for c in ('cam0','cam1')): p.error('Stereo cam0/cam1 required')
    lines=['%YAML:1.0','File.version: "1.0"','Camera.type: "PinHole"']
    for index in range(2):
        c=cam[f'cam{index}']
        if c['camera_model']!='pinhole' or c['distortion_model']!='radtan': p.error('Only pinhole-radtan is supported by this converter')
        values=c['intrinsics']+c['distortion_coeffs']
        if len(values)!=8 or not np.isfinite(values).all(): p.error('Invalid camera coefficients')
        for key,value in zip(('fx','fy','cx','cy','k1','k2','p1','p2'),values): lines.append(f'Camera{index+1}.{key}: {float(value):.14g}')
    if cam['cam0']['resolution']!=cam['cam1']['resolution']: p.error('Stereo resolutions differ')
    width,height=cam['cam0']['resolution']
    if width<=0 or height<=0: p.error('Invalid resolution')
    stereo=np.linalg.inv(rigid(cam['cam1']['T_cn_cnm1']))
    extrinsic=np.linalg.inv(rigid(cam['cam0']['T_cam_imu']))
    baseline=float(np.linalg.norm(stereo[:3,3]))
    if baseline<=0: p.error('Zero stereo baseline')
    lines.extend([f'Camera.width: {int(width)}',f'Camera.height: {int(height)}',f'Camera.fps: {a.fps}',
                  'Camera.RGB: 0','Stereo.ThDepth: 40.0',f'Stereo.b: {baseline:.14g}'])
    for name,m in [('Stereo.T_c1_c2',stereo),('IMU.T_b_c1',extrinsic)]:
        lines.extend([name+': !!opencv-matrix',' rows: 4',' cols: 4',' dt: f',' data: ['+', '.join(f'{x:.14g}' for x in m.flat)+']'])
    for key,name in [('NoiseGyro','gyroscope_noise_density'),('NoiseAcc','accelerometer_noise_density'),
                     ('GyroWalk','gyroscope_random_walk'),('AccWalk','accelerometer_random_walk'),('Frequency','update_rate')]:
        value=float(imu[name])
        if not np.isfinite(value) or value<=0: p.error(f'Invalid IMU parameter: {name}')
        lines.append(f'IMU.{key}: {value:.14g}')
    lines.extend(['IMU.fastInit: 0','ORBextractor.nFeatures: 2000','ORBextractor.scaleFactor: 1.2',
                  'ORBextractor.nLevels: 8','ORBextractor.iniThFAST: 15','ORBextractor.minThFAST: 5',
                  'Viewer.KeyFrameSize: 0.05','Viewer.KeyFrameLineWidth: 1.0','Viewer.GraphLineWidth: 0.9',
                  'Viewer.PointSize: 2.0','Viewer.CameraSize: 0.08','Viewer.CameraLineWidth: 3.0',
                  'Viewer.ViewpointX: 0.0','Viewer.ViewpointY: -0.7','Viewer.ViewpointZ: -3.5','Viewer.ViewpointF: 500.0'])
    shift=float(cam['cam0'].get('timeshift_cam_imu',0))
    if not np.isfinite(shift): p.error('Invalid camera-IMU time shift')
    profile=json.loads(a.capture_profile.read_text()) if a.capture_profile else {'adapter':'euroc'}
    profile['camera_to_imu_offset_ns']=round(shift*1e9)
    profile['calibration_source']=a.cam.name
    a.output_dir.mkdir(parents=True,exist_ok=False)
    (a.output_dir/'orbslam.yaml').write_text('\n'.join(lines)+'\n')
    (a.output_dir/'device_profile.json').write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n')
    print('Created orbslam.yaml and device_profile.json:',a.output_dir)
    print('Frame convention: T_A_B maps B to A. Positive timeshift means t_imu=t_camera+shift.')
    print('Use BOTH files in the SLAM application. IMU noise values must match the actual sensor.')


if __name__=='__main__': main()
