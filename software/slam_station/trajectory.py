"""Optional rigid target-frame export; never changes the original IMU trajectory."""
import csv
import json
import math
from pathlib import Path
from devices import validate_transform


def quaternion_matrix(q):
    n = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(n) or abs(n-1) > .01: raise ValueError('无效轨迹四元数')
    x,y,z,w = [v/n for v in q]
    return [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]


def matrix_quaternion(r):
    # Choose the largest component to remain stable near 180 degrees.
    terms = [1+r[0][0]-r[1][1]-r[2][2], 1-r[0][0]+r[1][1]-r[2][2],
             1-r[0][0]-r[1][1]+r[2][2], 1+r[0][0]+r[1][1]+r[2][2]]
    i = max(range(4), key=terms.__getitem__); s = 2*math.sqrt(max(terms[i],0))
    if i == 3: q = [(r[2][1]-r[1][2])/s,(r[0][2]-r[2][0])/s,(r[1][0]-r[0][1])/s,s/4]
    elif i == 0: q = [s/4,(r[0][1]+r[1][0])/s,(r[0][2]+r[2][0])/s,(r[2][1]-r[1][2])/s]
    elif i == 1: q = [(r[0][1]+r[1][0])/s,s/4,(r[1][2]+r[2][1])/s,(r[0][2]-r[2][0])/s]
    else: q = [(r[0][2]+r[2][0])/s,(r[1][2]+r[2][1])/s,s/4,(r[1][0]-r[0][1])/s]
    return q


def export_target(result, device):
    matrix = device.get('T_imu_target')
    if matrix is None: return None
    validate_transform(matrix)
    result = Path(result); rows = []
    with (result/'imu_trajectory.csv').open(encoding='utf-8', newline='') as f:
        reader = csv.DictReader(f); fields = reader.fieldnames
        for row in reader:
            p = [float(row[k]) for k in ('x_m','y_m','z_m')]
            q = [float(row[k]) for k in ('qx','qy','qz','qw')]
            if not all(math.isfinite(v) for v in p+q): raise ValueError('轨迹包含非有限数值')
            r = quaternion_matrix(q)
            position = [p[i]+sum(r[i][j]*matrix[j][3] for j in range(3)) for i in range(3)]
            rotation = [[sum(r[i][k]*matrix[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
            row.update(zip(('x_m','y_m','z_m','qx','qy','qz','qw'), position+matrix_quaternion(rotation)))
            rows.append(row)
    if not rows: raise ValueError('没有可导出的轨迹')
    # Fixed filenames; arbitrary device names are metadata, never filesystem paths.
    with (result/'target_trajectory.csv').open('x', encoding='utf-8', newline='') as f:
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    with (result/'target_frame.json').open('x',encoding='utf-8') as f:
        json.dump({'target_frame':device['target_frame'], 'T_imu_target':matrix,
                   'composition':'T_world_target = T_world_imu @ T_imu_target',
                   'position_unit':'m', 'quaternion_order':'xyzw',
                   'timestamp_clock':device['timestamp_clock'], 'source':'imu_trajectory.csv',
                   'note':'Same SLAM world frame, timestamps and segment IDs; no interpolation.'},f,ensure_ascii=False,indent=2)
    return result/'target_trajectory.csv'
