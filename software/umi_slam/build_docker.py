#!/usr/bin/env python3
"""Send only source files to Docker, including with the legacy Docker builder."""
import argparse
from pathlib import Path
import subprocess
import tarfile
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def include(path):
    relative=path.relative_to(ROOT)
    if any(p in ('build','.git','__pycache__','Examples','Examples_old') or p.startswith('.build') for p in relative.parts): return False
    if path.name=='ORBvoc.txt' or '.so' in path.suffixes or path.suffix in ('.a','.o','.pyc'): return False
    return path.is_file() and not path.is_symlink()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--tag',default='umi-slam:0.1'); p.add_argument('--jobs',type=int,default=2)
    a=p.parse_args()
    with tempfile.TemporaryFile() as context:
        with tarfile.open(fileobj=context,mode='w:gz') as archive:
            for directory in [HERE,ROOT/'stereo_imu_trajectory/ORB_SLAM3',ROOT/'stereo_imu_trajectory/Pangolin']:
                for file in sorted(directory.rglob('*')):
                    if include(file): archive.add(file,arcname=str(file.relative_to(ROOT)),recursive=False)
        print(f'Build context: {context.tell()/1024/1024:.1f} MiB',flush=True)
        context.seek(0)
        subprocess.run(['docker','build','-f','umi_slam/Dockerfile','-t',a.tag,
                        '--build-arg',f'BUILD_JOBS={a.jobs}','-'],stdin=context,check=True)


if __name__=='__main__': main()
