#!/usr/bin/env python3
"""Package the tested native Ubuntu 22.04/x86_64 build using a cached base image.

This is an offline alternative to the source Dockerfile, not a cross compiler.
"""
import argparse
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import tempfile

import cv2
import numpy

HERE=Path(__file__).resolve().parent
BASE='chicheng/orb_slam3@sha256:fbf562a6707a0fe9bafb04375c4cfb7b051156c14bc1e820f3f238b9aa17744f'


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',type=Path,default=HERE/'.build/backend.json')
    p.add_argument('--tag',default='umi-slam:0.1')
    a=p.parse_args()
    if platform.machine()!='x86_64' or 'VERSION_ID="22.04"' not in Path('/etc/os-release').read_text():
        p.error('Offline runtime packaging requires an Ubuntu 22.04 x86_64 host')
    subprocess.run(['docker','image','inspect',BASE],check=True,stdout=subprocess.DEVNULL)
    backend=json.loads(a.backend.read_text())
    build=Path(backend['binary']).parents[1]
    env=os.environ.copy(); env['LD_LIBRARY_PATH']=':'.join(backend['library_paths'])
    with tempfile.TemporaryDirectory(prefix='umi-runtime-') as directory:
        context=Path(directory); app=context/'umi_slam'; libs=context/'lib'; py=context/'python'
        app.mkdir(); libs.mkdir(); py.mkdir()
        for file in HERE.glob('*.py'): shutil.copy2(file,app/file.name)
        shutil.copytree(HERE/'profiles',app/'profiles')
        shutil.copy2(HERE/'README.md',app/'README.md')
        shutil.copytree(HERE/'backend',app/'backend')
        shutil.copytree(HERE/'tests',app/'tests',ignore=shutil.ignore_patterns('__pycache__'))
        shutil.copy2(backend['binary'],context/'umi_slam_runner')
        shutil.copy2(backend['vocabulary'],context/'ORBvoc.txt')
        shutil.copy2(cv2.__file__,py/Path(cv2.__file__).name)
        shutil.copytree(Path(numpy.__file__).parent,py/'numpy',ignore=shutil.ignore_patterns('__pycache__'))
        # Include linked dependencies, except glibc's loader/runtime which come
        # from the same Ubuntu release in the pinned base image.
        exclude=re.compile(r'^(ld-linux|lib(c|m|pthread|dl|rt|util|resolv|anl)\.so)')
        files=[Path(backend['binary']),Path(cv2.__file__),*Path(numpy.__file__).parent.rglob('*.so')]
        dependencies={}
        for file in files:
            output=subprocess.check_output(['ldd',str(file)],env=env,text=True)
            if 'not found' in output: raise RuntimeError(f'Unresolved native dependency: {file}\n{output}')
            for line in output.splitlines():
                match=re.search(r'^\s*(\S+) => (/\S+)',line)
                if match and not exclude.match(match[1]): dependencies[match[1]]=Path(match[2])
        for name,file in dependencies.items(): shutil.copy2(file,libs/name)
        # Ship the exact patched ORB/Pangolin source and their original notices.
        ignore=shutil.ignore_patterns('build','.git','*.so*','*.a','*.o','ORBvoc.txt','__pycache__')
        for name in ('ORB_SLAM3','Pangolin'): shutil.copytree(build/name,context/'source'/name,ignore=ignore)
        config={**backend,'binary':'/opt/umi/umi_slam_runner','vocabulary':'/opt/umi/ORBvoc.txt','library_paths':['/opt/umi/lib']}
        (app/'.build').mkdir(); (app/'.build/backend.json').write_text(json.dumps(config,indent=2))
        (context/'Dockerfile').write_text(f'''FROM {BASE}
COPY . /opt/umi
ENV PYTHONPATH=/opt/umi/python LD_LIBRARY_PATH=/opt/umi/lib PYTHONUNBUFFERED=1
RUN python3 /opt/umi/umi_slam/pipeline.py --help && python3 -m unittest discover -s /opt/umi/umi_slam/tests -v
ENTRYPOINT ["python3", "/opt/umi/umi_slam/pipeline.py"]
CMD ["--help"]
''')
        subprocess.run(['docker','build','-t',a.tag,str(context)],check=True)


if __name__=='__main__': main()
