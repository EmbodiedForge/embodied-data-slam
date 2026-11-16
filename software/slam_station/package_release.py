#!/usr/bin/env python3
"""Build a relocatable Windows Python release, optionally with offline image/data."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[2]


def main():
    p=argparse.ArgumentParser(); p.add_argument('--offline',action='store_true'); args=p.parse_args()
    rels=['launch_windows.cmd','load_images_windows.cmd','launch_station.sh','WINDOWS_上位机说明.md','SHA256SUMS']
    paths=[ROOT/f for f in rels]
    for sub in ('software/slam_station','software/umi_slam/profiles'):
        paths += [f for f in (ROOT/sub).rglob('*') if f.is_file() and not any(x in f.parts for x in ('__pycache__','.venv-build','build','dist'))]
    if args.offline:
        paths += [ROOT/'images/umi-slam_0.1.tar.gz']
        paths += [f for f in (ROOT/'examples/lissajous_20260915').rglob('*') if f.is_file()]
    name='SLAMStation_Windows_0.2_'+('Offline' if args.offline else 'Source')
    dest=ROOT/'releases'; dest.mkdir(exist_ok=True); out=dest/(name+'.zip')
    manifest={'version':'0.2','platform':'Windows x64 + Python >=3.10 + Docker Desktop Linux containers',
              'native_windows_validated':False,'contains_exe':False,'offline_image':args.offline,'files':{}}
    with zipfile.ZipFile(out,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as archive:
        for path in sorted(paths):
            rel=path.relative_to(ROOT).as_posix()
            digest=hashlib.sha256()
            with path.open('rb') as f:
                for block in iter(lambda:f.read(8*1024*1024),b''): digest.update(block)
            manifest['files'][rel]=digest.hexdigest()
            compression=zipfile.ZIP_STORED if path.suffix in ('.gz','.mkv','.png','.jpg','.bin') else zipfile.ZIP_DEFLATED
            archive.write(path,name+'/'+rel,compress_type=compression)
        archive.writestr(name+'/STATION_MANIFEST.json',json.dumps(manifest,ensure_ascii=False,indent=2))
    digest=hashlib.sha256()
    with out.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''): digest.update(block)
    out.with_suffix('.zip.sha256').write_text(digest.hexdigest()+'  '+out.name+'\n')
    print(out)


if __name__=='__main__': main()
