#!/usr/bin/env python3
"""Offline UMI stereo + IMU processing. Output pose is T_world_imu, metres, xyzw."""
import argparse
import csv
from decimal import Decimal
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent


def save_json(path, value):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def strictly_increasing(values, label):
    if len(values) < 2 or any(b <= a for a, b in zip(values, values[1:])):
        raise ValueError(f'{label}: need at least two strictly increasing timestamps')


def integer_ns(value):
    n = Decimal(str(value))
    if not n.is_finite() or n != n.to_integral_value() or n < 0:
        raise ValueError(f'Invalid nanosecond timestamp: {value}')
    return int(n)


def decode_legacy_imu(path, hz, accel_g, gyro_dps):
    """MS901M type 03, UART blocks <uint64 ns, uint32 size, bytes>.

    Arrival stamps are NOT sample timestamps. Uniform reconstruction is specific
    to this legacy device and assumes no missing sensor packets.
    """
    if not all(math.isfinite(x) and x > 0 for x in (hz, accel_g, gyro_dps)):
        raise ValueError('IMU frequency and ranges must be positive finite numbers')
    raw = bytearray(); blocks = []
    with path.open('rb') as f:
        while header := f.read(12):
            if len(header) != 12: raise ValueError('Truncated IMU block header')
            ts, size = struct.unpack('<QI', header)
            if size > 16 * 1024 * 1024: raise ValueError('Invalid IMU block length')
            payload = f.read(size)
            if len(payload) != size: raise ValueError('Truncated IMU block payload')
            raw.extend(payload); blocks.append((ts, len(raw)))
    if not blocks: raise ValueError('Empty IMU stream')
    samples = []; ends = []; invalid = 0; i = 0
    while i + 5 <= len(raw):
        if raw[i:i+2] != b'\x55\x55': i += 1; continue
        end = i + 5 + raw[i+3]
        if end > len(raw): break
        packet = raw[i:end]
        if (sum(packet[:-1]) & 255) != packet[-1]: invalid += 1; i += 1; continue
        if packet[2] == 3 and packet[3] == 12:
            samples.append(struct.unpack('<6h', packet[4:-1])); ends.append(end)
        i = end
    if len(samples) < 2: raise ValueError('No usable MS901M samples')
    if invalid: raise ValueError(f'{invalid} corrupt IMU packets; cannot safely reconstruct a uniform time axis')
    # Anchor to the first block containing a complete type-03 sample. Correctly
    # handle UART fragments and other packet types before the first sample.
    first = next((ts, end) for ts, end in blocks if end >= ends[0])
    n = sum(end <= first[1] for end in ends)
    t0 = first[0] - round((n-1) * 1e9 / hz)
    values = np.asarray(samples, dtype=float)
    acc = values[:, :3] * (accel_g * 9.80665 / 32768)
    gyro = values[:, 3:] * (gyro_dps * math.pi / 180 / 32768)
    times = [t0 + round(k * 1e9 / hz) for k in range(len(samples))]
    return times, np.column_stack([gyro, acc])


def read_imu_csv(path):
    times = []; values = []
    with path.open() as f:
        for row in csv.reader(f):
            if not row or row[0].startswith('#'): continue
            if len(row) != 7: raise ValueError('IMU CSV needs timestamp_ns,gx,gy,gz,ax,ay,az')
            times.append(integer_ns(row[0])); values.append([float(v) for v in row[1:]])
    return times, np.asarray(values)


def settings_info(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened(): raise ValueError('Cannot read ORB-SLAM3 settings')
    try:
        required = ['Camera.width', 'Camera.height', 'Camera.fps', 'IMU.Frequency',
                    'IMU.T_b_c1', 'Stereo.T_c1_c2', 'IMU.NoiseGyro', 'IMU.NoiseAcc']
        for key in required:
            if fs.getNode(key).empty(): raise ValueError(f'Missing settings field: {key}')
        shape = (int(fs.getNode('Camera.height').real()), int(fs.getNode('Camera.width').real()))
        for name in ['IMU.T_b_c1', 'Stereo.T_c1_c2']:
            matrix = fs.getNode(name).mat()
            if matrix.shape != (4,4) or not np.isfinite(matrix).all() or not np.allclose(matrix[3], [0,0,0,1]):
                raise ValueError(f'Invalid transform: {name}')
            R = matrix[:3,:3]
            if not np.allclose(R.T@R, np.eye(3), atol=1e-3) or not np.isclose(np.linalg.det(R), 1, atol=1e-3):
                raise ValueError(f'Invalid rotation: {name}')
        return {'shape': shape, 'fps': fs.getNode('Camera.fps').real(),
                'imu_hz': fs.getNode('IMU.Frequency').real(),
                'fast_init': bool(fs.getNode('IMU.fastInit').real())}
    finally:
        fs.release()


def prepare(source, dest, profile, settings, stride, limit):
    info = settings_info(settings)
    warning = []
    if info['fast_init']:
        warning.append('IMU.fastInit enabled: experimental slow-motion preset; not an accuracy guarantee.')
    if stride < 1 or limit < 0: raise ValueError('stride must be >=1 and max-frames >=0')
    legacy = (source/'stereo.mkv').is_file()
    if legacy:
        if profile.get('adapter') != 'umi_ms901m_uniform':
            raise ValueError('This raw capture needs an explicit umi_ms901m_uniform device profile')
        with (source/'umi_video_timestamps.csv').open() as f:
            times = [integer_ns(r['driver_ts_ns']) for r in csv.DictReader(f)]
        hz = float(profile['imu_hz'])
        if hz <= 0 or not math.isfinite(hz): raise ValueError('Invalid IMU frequency')
        it, iv = decode_legacy_imu(source/'umi_imu_timestamps.bin', hz,
                                   float(profile['accel_range_g']), float(profile['gyro_range_dps']))
        warning.append('IMU sample times reconstructed at fixed frequency from UART arrival blocks; clock drift/dropped samples not independently verified.')
    else:
        path = source/'timestamps.txt'
        if path.is_file():
            times = [integer_ns(x.strip()) for x in path.read_text().splitlines() if x.strip() and not x.startswith('#')]
            left = [f'{t}.jpg' if (source/f'mav0/cam0/data/{t}.jpg').is_file() else f'{t}.png' for t in times]
            right = [f'{t}.jpg' if (source/f'mav0/cam1/data/{t}.jpg').is_file() else f'{t}.png' for t in times]
        else:
            def camera_csv(path):
                with path.open() as f:
                    return [(integer_ns(r[0]), r[1]) for r in csv.reader(f) if r and not r[0].startswith('#')]
            l = camera_csv(source/'mav0/cam0/data.csv'); r = camera_csv(source/'mav0/cam1/data.csv')
            if [x[0] for x in l] != [x[0] for x in r]: raise ValueError('Stereo timestamps must match; explicit synchronization required')
            times = [x[0] for x in l]; left = [x[1] for x in l]; right = [x[1] for x in r]
        it, iv = read_imu_csv(source/'mav0/imu0/data.csv')
    strictly_increasing(times, 'Camera'); strictly_increasing(it, 'IMU')
    if iv.shape != (len(it), 6) or not np.isfinite(iv).all(): raise ValueError('Invalid IMU measurements')
    hz = 1e9 / np.median(np.diff(np.asarray(it, dtype=np.int64)))
    if abs(hz/info['imu_hz']-1) > .1: raise ValueError('IMU rate does not match calibration')
    max_imu_gap = float(np.max(np.diff(np.asarray(it, dtype=np.int64))) / 1e9)
    if max_imu_gap > 5 / info['imu_hz']:
        raise ValueError(f'IMU gap {max_imu_gap:.6f}s exceeds five sampling intervals')
    accel_median = float(np.median(np.linalg.norm(iv[:,3:], axis=1)))
    if not 5 < accel_median < 15: warning.append(f'Acceleration median {accel_median:.2f} m/s²: check units/range or unusually dynamic motion.')
    # Positive offset means t_imu = t_camera + offset (Kalibr convention).
    offset = integer_ns(abs(profile.get('camera_to_imu_offset_ns', 0)))
    if profile.get('camera_to_imu_offset_ns', 0) < 0: offset = -offset
    camera_imu_times = [t + offset for t in times]
    chosen = [i for i in range(0, len(times), stride) if it[0] <= camera_imu_times[i] <= it[-1]]
    trimmed = len(range(0, len(times), stride)) - len(chosen)
    if trimmed: warning.append(f'{trimmed} image pairs outside IMU coverage were excluded.')
    if limit: chosen = chosen[:limit]
    if len(chosen) < 2: raise ValueError('Insufficient image pairs inside IMU coverage')
    period = float(np.median(np.diff(np.array([times[i] for i in chosen], dtype=np.int64))) / 1e9)
    # Record effective frame rate in a copied settings file, never mutate calibration.
    import re
    text = settings.read_text()
    text, n = re.subn(r'^Camera\.fps:.*$', f'Camera.fps: {max(1, round(1/period))}', text, flags=re.M)
    if n != 1: raise ValueError('Expected one Camera.fps setting')
    (dest/'settings.yaml').write_text(text)
    (dest/'left').mkdir(); (dest/'right').mkdir()
    cap = cv2.VideoCapture(str(source/'stereo.mkv')) if legacy else None
    wanted = set(chosen); written = 0
    try:
        with (dest/'frames.csv').open('w') as output:
            writer = csv.writer(output, lineterminator='\n')
            for i in range(chosen[-1]+1):
                if legacy:
                    ok, im = cap.read()
                    if not ok: raise ValueError(f'Video ends before timestamp row {i+1}')
                    if i not in wanted: continue
                    if im.shape[:2] != (info['shape'][0], 2*info['shape'][1]):
                        raise ValueError(f'Video size {im.shape[:2]} does not match stereo calibration')
                    mid = info['shape'][1]; pair = [im[:,:mid], im[:,mid:]]
                else:
                    if i not in wanted: continue
                    pair = []
                    for c, name in enumerate([left[i], right[i]]):
                        file = (source/f'mav0/cam{c}/data'/name).resolve()
                        if not file.is_relative_to((source/f'mav0/cam{c}/data').resolve()): raise ValueError('Image filename leaves dataset')
                        image = cv2.imread(str(file), cv2.IMREAD_GRAYSCALE)
                        if image is None or image.shape != info['shape']: raise ValueError(f'Invalid image or size: {file}')
                        pair.append(image)
                names = [f'left/{times[i]}.png', f'right/{times[i]}.png']
                for name, im in zip(names, pair):
                    if im.ndim == 3: im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
                    if not cv2.imwrite(str(dest/name), im): raise OSError('Failed to write prepared image')
                writer.writerow([times[i], *names]); written += 1
                if written % 100 == 0: print(f'PREPARE {written}/{len(chosen)}', flush=True)
    finally:
        if cap is not None: cap.release()
    # Shift IMU to camera clock so exported timestamps stay identical to image stamps.
    with (dest/'imu.csv').open('w') as f:
        w = csv.writer(f)
        for t, v in zip(it, iv): w.writerow([t-offset, *v])
    result = {'adapter': 'umi_ms901m_uniform' if legacy else 'euroc', 'source_frames': len(times),
              'selected_frames': written, 'imu_samples': len(it), 'stride': stride,
              'camera_to_imu_offset_ns': offset, 'median_frame_period_s': period,
              'imu_hz_observed': float(hz), 'accel_norm_median_m_s2': accel_median,
              'max_imu_gap_s': max_imu_gap,
              'first_timestamp_ns': times[chosen[0]], 'last_timestamp_ns': times[chosen[-1]],
              'warnings': warning, 'calibration_sha256': hashlib.sha256(settings.read_bytes()).hexdigest()}
    save_json(dest/'input_report.json', result)
    return result


def export_result(out, report):
    rows = []
    with (out/'imu_euroc.txt').open() as f:
        for line in f:
            v = line.split()
            if not v: continue
            if len(v) != 8: raise ValueError('Malformed trajectory output')
            # ORB-SLAM3 writes (double seconds * 1e9) with decimal digits,
            # e.g. 106493924019000.015625. Round only this export representation;
            # input sensor stamps remain strictly integral. Match to the exact
            # original image timestamp below before publishing a pose.
            timestamp=Decimal(v[0])
            if not timestamp.is_finite() or timestamp < 0: raise ValueError('Invalid exported timestamp')
            rows.append((int(timestamp.to_integral_value()), np.array([float(x) for x in v[1:]])))
    if not rows: raise ValueError('Backend exported no trajectory')
    strictly_increasing([t for t, _ in rows], 'Trajectory')
    with (out/'tracking.csv').open() as f:
        states = {int(r['timestamp_ns']): r for r in csv.DictReader(f)}
    # C++ double->ns conversion can round by a few ns. Match within 1 us, never
    # interpolate missing poses or concatenate separately exported maps.
    stamps = np.array(sorted(states), dtype=np.int64)
    matched = []
    for t, pose in rows:
        if not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:])-1) > .01:
            raise ValueError('Invalid pose/quaternion')
        j = int(np.searchsorted(stamps, t)); candidates = [x for x in (j-1,j) if 0 <= x < len(stamps)]
        nearest = min(candidates, key=lambda x: abs(int(stamps[x])-t))
        stamp = int(stamps[nearest])
        if abs(stamp-t) > 1000: raise ValueError('Exported pose has no matching image timestamp')
        state = states[stamp]
        if state['imu_initialized'] == '1' and state['state'] == '2': matched.append((stamp, pose, state['map_id']))
    if len(matched) < 2: raise ValueError('No usable initialized IMU trajectory')
    strictly_increasing([x[0] for x in matched], 'Matched trajectory')
    start = report['first_timestamp_ns']; previous = None; segment = 0
    with (out/'imu_trajectory.csv').open('w') as f, (out/'imu_trajectory.tum').open('w') as tum:
        w = csv.writer(f)
        w.writerow(['timestamp_ns','time_from_input_start_s','x_m','y_m','z_m','qx','qy','qz','qw','segment_id'])
        for stamp, pose, map_id in matched:
            if previous and (stamp-previous[0] > report['median_frame_period_s']*1e9*2.5 or map_id != previous[1]): segment += 1
            w.writerow([stamp, (stamp-start)/1e9, *pose, segment])
            tum.write(f'{stamp//1000000000}.{stamp%1000000000:09d} ' + ' '.join(f'{v:.9f}' for v in pose) + '\n')
            previous = (stamp, map_id)
    quality = {'status': 'completed', 'pose': 'T_world_imu', 'position_unit': 'm',
               'quaternion_order': 'qx,qy,qz,qw', 'timestamp_unit': 'ns',
               'time_basis': 'original camera clock (not Unix time)',
               'world_frame': 'ORB-SLAM3 export frame: first keyframe IMU frame of selected map; not robot base',
               'source_export': 'ORB-SLAM3 largest map, finalized after shutdown',
               'output_frames': len(matched), 'selected_frames': report['selected_frames'],
               'coverage_fraction': len(matched)/report['selected_frames'], 'segments': segment+1,
               'first_output_timestamp_ns': matched[0][0], 'last_output_timestamp_ns': matched[-1][0],
               'accuracy': 'not measured; no independent reference supplied',
               'warnings': list(report['warnings'])}
    if quality['coverage_fraction'] < .99: quality['warnings'].append('Trajectory does not cover every selected frame; see tracking.csv for initialization/loss.')
    if len({r['map_id'] for r in states.values()}) > 1: quality['warnings'].append('SLAM changed maps. Export contains only the map selected by ORB-SLAM3, not all motion.')
    save_json(out/'quality.json', quality)
    # Offline HTML plot: no external services, no network dependencies.
    import html
    positions = np.array([p[:3] for _,p,_ in matched])
    projections = []
    for x,y,label in [(0,1,'XY'),(0,2,'XZ'),(1,2,'YZ')]:
        p = positions[:,[x,y]]; lo=p.min(0); span=np.ptp(p,axis=0); scale=340/max(float(span.max()),1e-6)
        screen=(p-(lo+span/2))*scale+200
        # Draw gaps separately, preserving the same aspect ratio on both axes.
        groups=[]; group=[]; prev=None
        for k,(stamp,pose,map_id) in enumerate(matched):
            if prev and (stamp-prev[0] > report['median_frame_period_s']*1e9*2.5 or map_id != prev[1]):
                groups.append(group); group=[]
            group.append(f'{screen[k,0]:.2f},{400-screen[k,1]:.2f}'); prev=(stamp,map_id)
        groups.append(group)
        lines=''.join('<polyline fill="none" stroke="#15803d" stroke-width="1.5" points="'+' '.join(g)+'"/>' for g in groups)
        projections.append(f'<section><h2>{label} · metres</h2><svg viewBox="0 0 400 400">{lines}</svg><p>Range: {span[0]:.4f} × {span[1]:.4f} m</p></section>')
    (out/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>UMI IMU trajectory</title><style>body{font:16px sans-serif;margin:32px;background:#f6f8fa}main{display:flex}section{width:32%;background:white;margin:8px;padding:12px}pre{white-space:pre-wrap}svg{width:100%}</style><h1>UMI · IMU 原点轨迹</h1><p>位置单位：米；精度尚未通过独立真值验证。断线表示输出间断或地图切换。</p><main>'+''.join(projections)+'</main><pre>'+html.escape(json.dumps(quality,ensure_ascii=False,indent=2))+'</pre>')
    return quality


def execute(args):
    source = args.input.resolve(); output = args.output.resolve(); settings = args.settings.resolve()
    if output == source or source.is_relative_to(output) or output.is_relative_to(source):
        raise ValueError('Input and output directories must be separate, not nested')
    if not source.is_dir(): raise ValueError('Input directory does not exist')
    profile = json.loads(args.profile.read_text()) if args.profile else {}
    backend = json.loads(args.backend.read_text()) if not args.prepare_only else None
    output.mkdir(parents=True, exist_ok=False)
    state = {'status':'preparing', 'input':str(source), 'settings':str(settings), 'profile':profile,
             'backend':backend, 'started_unix_s':time.time()}
    save_json(output/'job.json', state)
    child = None
    try:
        if args.prepared_input:
            prepared=source
            report=json.loads((prepared/'input_report.json').read_text())
            if hashlib.sha256(settings.read_bytes()).hexdigest() != report['calibration_sha256']:
                raise ValueError('Prepared input was produced with different calibration')
            with (prepared/'frames.csv').open() as f:
                frame_rows=list(csv.reader(f))
            frame_times=[integer_ns(row[0]) for row in frame_rows]
            strictly_increasing(frame_times,'Prepared camera')
            if len(frame_rows)!=report['selected_frames']: raise ValueError('Prepared frame count changed')
            for row in frame_rows:
                if len(row)!=3: raise ValueError('Invalid prepared frame manifest')
                for name in row[1:]:
                    file=(prepared/name).resolve()
                    if not file.is_relative_to(prepared) or not file.is_file(): raise ValueError('Invalid prepared image path')
            it,iv=read_imu_csv(prepared/'imu.csv'); strictly_increasing(it,'Prepared IMU')
            if iv.shape != (len(it),6) or not np.isfinite(iv).all(): raise ValueError('Invalid prepared IMU')
            settings_info(prepared/'settings.yaml')
            state['prepared_input']=str(prepared)
        else:
            prepared=output/'prepared'; prepared.mkdir()
            report=prepare(source, prepared, profile, settings, args.stride, args.max_frames)
        if args.prepare_only:
            state['status']='prepared'; return
        state['status']='running'; save_json(output/'job.json',state)
        env=os.environ.copy(); env['LD_LIBRARY_PATH']=':'.join(backend['library_paths']+[env.get('LD_LIBRARY_PATH','')])
        command=[backend['binary'],backend['vocabulary'],str(prepared/'settings.yaml'),str(prepared),str(output)]
        print('Running stereo-inertial SLAM; output:', output, flush=True)
        with (output/'slam.log').open('w') as log:
            child=subprocess.Popen(command,cwd=output,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            timeout=args.timeout or max(180, (report['last_timestamp_ns']-report['first_timestamp_ns'])/1e9*5+120)
            end=time.monotonic()+timeout
            while child.poll() is None:
                if time.monotonic()>end: raise TimeoutError(f'SLAM exceeded {timeout:.0f}s; see slam.log')
                time.sleep(.2)
            state['backend_exit_code']=child.returncode
            if child.returncode: raise RuntimeError(f'SLAM exited with code {child.returncode}; see slam.log')
        quality=export_result(output,report)
        state['status']='completed'; state['coverage_fraction']=quality['coverage_fraction']
        print(f"Completed: {quality['output_frames']} poses; coverage {quality['coverage_fraction']:.1%}", flush=True)
    except BaseException as e:
        state['status']='cancelled' if isinstance(e,KeyboardInterrupt) else 'failed'
        state['error']=str(e) or type(e).__name__
        raise
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGTERM)
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: os.killpg(child.pid,signal.SIGKILL); child.wait()
        state['finished_unix_s']=time.time(); save_json(output/'job.json',state)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True,type=Path)
    p.add_argument('--output',required=True,type=Path,help='New directory; existing output is never overwritten')
    p.add_argument('--settings',required=True,type=Path,help='ORB-SLAM3 stereo-inertial YAML matching the device')
    p.add_argument('--profile',type=Path,help='Required for legacy UMI raw capture; optional for EuRoC')
    p.add_argument('--backend',type=Path,default=HERE/'.build/backend.json')
    p.add_argument('--stride',type=int,default=5)
    p.add_argument('--max-frames',type=int,default=0,help='0 = all; positive value for diagnostic runs')
    p.add_argument('--timeout',type=float,default=0)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--prepared-input',action='store_true',help='Input is a previous prepared/ directory; uses its frame selection and timing')
    args=p.parse_args()
    if args.timeout < 0: p.error('timeout must be >=0')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try: execute(args)
    except KeyboardInterrupt: print('Cancelled',file=sys.stderr); return 130
    except Exception as e: print(f'ERROR: {e}',file=sys.stderr); return 1
    return 0


if __name__=='__main__':
    sys.exit(main())
