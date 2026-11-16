"""Versioned device contract. Raw decoders normalize to EuRoC before submission."""
import json
import math
from pathlib import Path
from typing import Protocol


class DatasetAdapter(Protocol):
    """Future device-specific decoder, invoked explicitly by an integration."""
    def normalize(self, source: Path, destination: Path, device: dict) -> Path:
        """Write a self-contained synchronized EuRoC dataset; return its root."""
        ...


def validate_transform(matrix):
    if not isinstance(matrix, list) or len(matrix) != 4 or any(not isinstance(r, list) or len(r) != 4 for r in matrix):
        raise ValueError('T_imu_target 必须是 4×4 矩阵')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for row in matrix for v in row):
        raise ValueError('外参必须是有限数值')
    if any(abs(a-b) > 1e-8 for a,b in zip(matrix[3], [0,0,0,1])):
        raise ValueError('外参最后一行必须为 [0,0,0,1]')
    r = [row[:3] for row in matrix[:3]]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[k][i]*r[k][j] for k in range(3)) - int(i == j)) > 1e-5:
                raise ValueError('外参旋转矩阵必须正交')
    det = sum(r[0][i]*(r[1][(i+1)%3]*r[2][(i+2)%3]-r[1][(i+2)%3]*r[2][(i+1)%3]) for i in range(3))
    if abs(det-1) > 1e-5: raise ValueError('外参旋转行列式必须为 +1')
    return matrix


def load_device(path):
    d = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if d.get('schema_version') != 1: raise ValueError('不支持的设备配置版本')
    for key in ('id', 'name', 'family', 'adapter', 'timestamp_clock'):
        if not isinstance(d.get(key), str) or not d[key].strip(): raise ValueError(f'缺少设备字段 {key}')
    if d['family'] not in ('UMI', 'EGO', 'generic'): raise ValueError('family 应为 UMI / EGO / generic')
    if d['adapter'] not in ('euroc', 'umi_ms901m_uniform'):
        raise ValueError('尚未实现该原始协议；请通过 DatasetAdapter 转为 EuRoC 后接入')
    expected = {'timestamp_unit':'ns', 'gyro_unit':'rad/s', 'accel_unit':'m/s^2', 'stereo_sync':'exact'}
    for k,v in expected.items():
        if d.get(k) != v: raise ValueError(f'{k} 必须为 {v}；请先显式转换数据')
    offset = d.get('camera_to_imu_offset_ns')
    if isinstance(offset, bool) or not isinstance(offset, int): raise ValueError('时间偏移必须为整数纳秒')
    if d['adapter'] == 'umi_ms901m_uniform':
        for k in ('imu_hz', 'accel_range_g', 'gyro_range_dps'):
            value = d.get(k)
            if isinstance(value, bool) or not isinstance(value, (int,float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'{k} 必须为正数')
    target = d.get('target_frame')
    if not isinstance(target, str) or not target.strip(): raise ValueError('需要 target_frame 名称')
    matrix = d.get('T_imu_target')
    if matrix is not None:
        validate_transform(matrix)
        if target == 'imu': raise ValueError('输出新坐标外参时请选择不同于 imu 的目标名称')
    for stream in d.get('auxiliary_streams', []):
        if not isinstance(stream, dict) or not all(isinstance(stream.get(k), str) and stream[k] for k in ('name','kind','path','timestamp_clock')):
            raise ValueError('辅助流需要 name/kind/path/timestamp_clock')
        rel = Path(stream['path'])
        if rel.is_absolute() or '..' in rel.parts or ':' in str(rel) or '\\' in str(rel):
            raise ValueError('辅助流 path 必须为采集目录内的相对路径，使用 /')
    return d


def pipeline_profile(device):
    keys = ('adapter','imu_hz','accel_range_g','gyro_range_dps','camera_to_imu_offset_ns')
    return {k:device[k] for k in keys if k in device}


def check_dataset(source, device):
    source = Path(source).resolve()
    if not source.is_dir(): raise ValueError('采集目录不存在')
    if device['adapter'] == 'umi_ms901m_uniform':
        required = ['stereo.mkv','umi_video_timestamps.csv','umi_imu_timestamps.bin']
    else:
        if (source/'stereo.mkv').exists(): raise ValueError('原始 UMI 视频不能选择 EuRoC 适配器')
        required = ['mav0/imu0/data.csv']
        if not (source/'timestamps.txt').is_file(): required += ['mav0/cam0/data.csv','mav0/cam1/data.csv']
        for cam in ('cam0','cam1'):
            if not (source/f'mav0/{cam}/data').is_dir(): raise ValueError(f'缺少 {cam} 图像目录')
    for f in required:
        if not (source/f).is_file(): raise ValueError(f'缺少输入文件：{f}')
    for stream in device.get('auxiliary_streams', []):
        path = (source/stream['path']).resolve()
        if not path.is_relative_to(source) or not path.is_file(): raise ValueError('辅助流文件缺失或超出采集目录')
