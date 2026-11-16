#!/usr/bin/env python3
"""Relocatable host-side Docker launcher; only Python's standard library is used."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid


def command(args, name):
    source=args.input.expanduser().resolve(); output=args.output.expanduser().resolve()
    settings=args.settings.expanduser().resolve()
    if not source.is_dir() or not settings.is_file(): raise ValueError('Input directory or calibration file does not exist')
    if source==output or source.is_relative_to(output) or output.is_relative_to(source):
        raise ValueError('Input and output must be separate, not nested')
    if output.exists(): raise ValueError('Output directory already exists')
    if args.stride < 1: raise ValueError('stride must be >=1')
    output.parent.mkdir(parents=True,exist_ok=True)
    mounts=[(source,'/input:ro'),(output.parent,'/output'),(settings,'/config/settings.yaml:ro')]
    if args.profile:
        profile=args.profile.expanduser().resolve()
        if not profile.is_file(): raise ValueError('Device profile does not exist')
        mounts.append((profile,'/config/profile.json:ro'))
    cmd=['docker','run','--rm','--init','--network','none','--name',name,'--user',f'{os.getuid()}:{os.getgid()}']
    for host,target in mounts:
        if ':' in str(host): raise ValueError('Docker bind paths cannot contain a colon')
        cmd.extend(['-v',f'{host}:{target}'])
    cmd.extend([args.image,'--input','/input','--output','/output/'+output.name,'--settings','/config/settings.yaml',
                '--stride',str(args.stride),'--max-frames',str(args.max_frames),'--timeout',str(args.timeout)])
    if args.profile: cmd.extend(['--profile','/config/profile.json'])
    if args.prepare_only: cmd.append('--prepare-only')
    if args.prepared_input: cmd.append('--prepared-input')
    return cmd


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    p.add_argument('--settings',type=Path,required=True); p.add_argument('--profile',type=Path)
    p.add_argument('--image',default='umi-slam:0.1'); p.add_argument('--stride',type=int,default=5)
    p.add_argument('--max-frames',type=int,default=0); p.add_argument('--timeout',type=float,default=0)
    p.add_argument('--prepare-only',action='store_true'); p.add_argument('--prepared-input',action='store_true')
    args=p.parse_args(); name='umi-slam-'+uuid.uuid4().hex[:12]; process=None
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        # Do not silently pull images: workstation operation must be offline.
        subprocess.run(['docker','image','inspect',args.image],check=True,stdout=subprocess.DEVNULL)
        process=subprocess.Popen(command(args,name))
        return process.wait()
    except KeyboardInterrupt:
        # docker stop targets exactly this job; the container's pipeline writes
        # cancelled status and terminates its own SLAM process group.
        subprocess.run(['docker','stop','--time','10',name],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        if process is not None:
            try: process.wait(timeout=15)
            except subprocess.TimeoutExpired: process.terminate(); process.wait(timeout=5)
        return 130
    except Exception as e:
        print(f'ERROR: {e}',file=sys.stderr); return 1


if __name__=='__main__': sys.exit(main())
