#!/usr/bin/env python3
"""Assemble the portable folder; image exports and final checksums are separate."""
from pathlib import Path
import shutil
import tarfile
import yaml

ROOT=Path(__file__).resolve().parents[1]
DEST=Path('/media/jt/PortableSSD/模思工程/Docker&Software/UMI_Toolkit_20260921')


def copy(src,dst):
    dst.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(src,dst)


def eligible(path,base):
    relative=path.relative_to(base)
    if any(p in ('build','.git','__pycache__','Examples','Examples_old') or p.startswith('.build') for p in relative.parts): return False
    if path.name=='ORBvoc.txt' or '.so' in path.suffixes or path.suffix in ('.o','.a','.pyc'): return False
    return path.is_file() and not path.is_symlink()


def main():
    DEST.mkdir(parents=True,exist_ok=True)
    app=ROOT/'umi_slam'
    for src in app.rglob('*'):
        if eligible(src,app): copy(src,DEST/'software/umi_slam'/src.relative_to(app))
    for name in ('README_先读我.md','load_images.sh','launch_software.sh'):
        copy(ROOT/'portable_toolkit'/name,DEST/name)
    copy(ROOT/'portable_toolkit/kalibr.sh',DEST/'calibration/kalibr.sh')
    for name in ('kalibr_to_bag.py','kalibr_to_orb.py'):
        copy(ROOT/'portable_toolkit'/name,DEST/'calibration/tools'/name)
    for src in (ROOT/'docker/kalibr').iterdir():
        if src.is_file(): copy(src,DEST/'calibration/build'/src.name)
    copy(ROOT/'data/新硬件20260817/aprilgrid_6x6_40mm_A3.pdf',DEST/'calibration/targets/aprilgrid_6x6_nominal40mm_A3.pdf')
    copy(ROOT/'data/新硬件20260817/calibration_20260817/kalibr/aprilgrid_6x6_34p5mm.yaml',DEST/'calibration/targets/aprilgrid_6x6_measured34p5mm.yaml')
    for src in (ROOT/'20260908_142951').iterdir():
        if src.suffix in ('.yaml','.pdf','.txt'):
            copy(src,DEST/'calibration/reference_20260908'/src.name)
    copy(ROOT/'20260907_151037/kalibr_stereo-camchain.yaml',DEST/'calibration/reference_20260908/stereo-camchain.yaml')
    imu=yaml.safe_load((ROOT/'20260908_142951/stereo_imu-imu.yaml').read_text())['imu0']
    template={k:imu[k] for k in ('rostopic','update_rate','accelerometer_noise_density','accelerometer_random_walk','gyroscope_noise_density','gyroscope_random_walk')}
    (DEST/'calibration/templates').mkdir(exist_ok=True)
    (DEST/'calibration/templates/imu_ms901m_example.yaml').write_text('# Historical noise values: verify for the actual IMU; not a new measurement.\n'+yaml.safe_dump(template,sort_keys=False))
    (DEST/'calibration/targets/aprilgrid_6x6_nominal40mm.example.yaml').write_text("# Example only: measure the printed target first.\ntarget_type: aprilgrid\ntagCols: 6\ntagRows: 6\ntagSize: 0.04\ntagSpacing: 0.3\n")
    print('Software and calibration files copied',flush=True)
    session=ROOT/'data/handeye_validation_20260915_160344/20260915_160354'
    for name in ('stereo.mkv','umi_video_timestamps.csv','umi_imu_timestamps.bin','capture_metadata.json'):
        copy(session/name,DEST/'examples/lissajous_20260915/input'/name)
    for src in (ROOT/'outputs/docker_umi/validation_final_20260921').iterdir():
        if src.is_file(): copy(src,DEST/'examples/lissajous_20260915/reference_result'/src.name)
    copy(ROOT/'20260908_142951/stereo_imu.bag',DEST/'examples/calibration_20260908/stereo_imu.bag')
    print('Example data copied',flush=True)
    archive_path=DEST/'source/SLAM_source.tar.gz'; archive_path.parent.mkdir(exist_ok=True)
    with tarfile.open(archive_path,'w:gz',compresslevel=1) as archive:
        for directory in (app,ROOT/'stereo_imu_trajectory/ORB_SLAM3',ROOT/'stereo_imu_trajectory/Pangolin',ROOT/'docker/kalibr',ROOT/'portable_toolkit'):
            for src in sorted(directory.rglob('*')):
                if eligible(src,directory): archive.add(src,arcname=str(Path('SLAM_Source_20260921')/src.relative_to(ROOT)),recursive=False)
    print('Source archive created',flush=True)


if __name__=='__main__': main()
