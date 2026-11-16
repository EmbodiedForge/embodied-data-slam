#!/usr/bin/env python3
import argparse
import hashlib
import json
from pathlib import Path
import signal
import sys
from jobs import Job
from runtime import diagnose, docker_call


def load_image(bundle):
    archive=bundle/'images/umi-slam_0.1.tar.gz'
    expected=None
    for line in (bundle/'SHA256SUMS').read_text(encoding='utf-8').splitlines():
        parts=line.split(maxsplit=1)
        if len(parts)==2 and parts[1].lstrip('*') == 'images/umi-slam_0.1.tar.gz': expected=parts[0]
    if expected is None: raise ValueError('SHA256SUMS 中缺少 SLAM 镜像记录')
    print('校验离线镜像 SHA256…',flush=True)
    digest=hashlib.sha256()
    with archive.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): digest.update(block)
    if digest.hexdigest() != expected: raise ValueError('镜像校验失败，停止导入')
    # Only an explicit import command changes local Docker images.
    result=docker_call(['image','load','--input',str(archive)],check=True)
    print(json.dumps(diagnose(),ensure_ascii=False,indent=2))
    return result.returncode


def main():
    p=argparse.ArgumentParser(description='SLAM Station 本地 Docker 上位机命令行')
    sub=p.add_subparsers(dest='action',required=True)
    sub.add_parser('diagnose')
    load=sub.add_parser('load-image'); load.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[2])
    run=sub.add_parser('run')
    for key in ('source','parent','settings','device'): run.add_argument('--'+key,required=True,type=Path)
    run.add_argument('--stride',type=int,default=5); run.add_argument('--max-frames',type=int,default=0)
    run.add_argument('--timeout',type=float,default=0); run.add_argument('--prepare-only',action='store_true')
    run.add_argument('--image',default='umi-slam:0.1')
    args=p.parse_args()
    try:
        if args.action == 'diagnose': print(json.dumps(diagnose(),ensure_ascii=False,indent=2)); return 0
        if args.action == 'load-image': return load_image(args.bundle.resolve())
        kwargs=vars(args); kwargs.pop('action'); job=Job(**kwargs)
        signal.signal(signal.SIGINT,lambda *_:job.stop())
        signal.signal(signal.SIGTERM,lambda *_:job.stop())
        print('任务目录：'+str(job.folder),flush=True); job.run()
        print(json.dumps(job.state,ensure_ascii=False,indent=2))
        return 0 if job.state['status'] in ('prepared','completed') else 130 if job.state['status']=='cancelled' else 1
    except Exception as e: print('ERROR: '+str(e),file=sys.stderr); return 1


if __name__=='__main__': sys.exit(main())
