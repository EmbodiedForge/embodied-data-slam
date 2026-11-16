#!/usr/bin/env python3
"""Build an isolated backend; never patch the experiment's original source tree."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

HERE = Path(__file__).resolve().parent


def patch_orb(root):
    cmake = root / 'CMakeLists.txt'
    cmake.write_text(cmake.read_text().split('# Build examples')[0].replace('-march=native', ''))
    # Also remove CPU-specific flags from bundled libraries for portability.
    for p in (root / 'Thirdparty').rglob('CMakeLists.txt'):
        p.write_text(p.read_text().replace('-march=native', ''))
    header = root / 'include/System.h'
    header.write_text(header.read_text().replace('    int GetTrackingState();',
        '    int GetTrackingState();\n    bool OfflineImuInitialized();\n    unsigned long OfflineMapId();'))
    source = root / 'src/System.cc'
    text = source.read_text()
    text = text.replace('Map* pBiggerMap;', 'Map* pBiggerMap = nullptr;')
    start = text.index('    /*if(mpViewer)', text.index('void System::Shutdown()'))
    end = text.index('    if(!mStrSaveAtlasToFile.empty())', start)
    text = text[:start] + '''    if(mpViewer) mpViewer->RequestFinish();
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
          mpLoopCloser->isRunningGBA() || (mpViewer && !mpViewer->isFinished()))
        usleep(5000);

''' + text[end:]
    # The original exporter dereferences null for an empty atlas.
    needle = '    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();'
    text = text.replace(needle, '    if(!pBiggerMap || pBiggerMap->KeyFramesInMap() == 0) return;\n' + needle)
    marker = 'int System::GetTrackingState()'
    assert marker in text, 'Unsupported ORB-SLAM3 source version'
    text = text.replace(marker, '''bool System::OfflineImuInitialized() { return mpAtlas->isImuInitialized(); }
unsigned long System::OfflineMapId() { return mpAtlas->GetCurrentMap()->GetId(); }

''' + marker)
    source.write_text(text)


def run(*args):
    print('+', ' '.join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), check=True)


def source_digest(source_root):
    digest = hashlib.sha256(Path(__file__).read_bytes())
    for name in ['ORB_SLAM3', 'Pangolin']:
        original = source_root / name
        for file in sorted(original.rglob('*')):
            if (file.suffix in ('.cc', '.cpp', '.c', '.h', '.hpp', '.cmake') or file.name == 'CMakeLists.txt') and not any(x in file.parts for x in ('build', '.git')):
                digest.update(str(file.relative_to(original)).encode()); digest.update(file.read_bytes())
    return digest.hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-root', type=Path, default=HERE.parent / 'stereo_imu_trajectory')
    p.add_argument('--build-dir', type=Path, default=HERE / '.build')
    p.add_argument('--jobs', type=int, default=2)
    a = p.parse_args()
    dest = a.build_dir.resolve(); dest.mkdir(parents=True, exist_ok=True)
    orb = dest / 'ORB_SLAM3'; pang = dest / 'Pangolin'; prefix = dest / 'install'
    # Stamp ensures changed source or patches cannot silently reuse an old build.
    digest = source_digest(a.source_root)
    ignored = shutil.ignore_patterns('build', '.git', '__pycache__', '*.so*', '*.o',
                                     '*.a', 'ORBvoc.txt', 'Examples', 'Examples_old', '.build')
    for name in ['ORB_SLAM3', 'Pangolin']:
        original = a.source_root / name
        if not original.is_dir(): p.error(f'Missing source: {original}')
    stamp = dest / 'source.sha256'
    if stamp.exists() and stamp.read_text() != digest:
        raise RuntimeError('Source changed: choose a new --build-dir to preserve the old build')
    if not stamp.exists():
        if orb.exists() or pang.exists():
            raise RuntimeError('Incomplete source staging: choose a new --build-dir')
        shutil.copytree(a.source_root / 'ORB_SLAM3', orb, ignore=ignored)
        shutil.copytree(a.source_root / 'Pangolin', pang, ignore=ignored)
        patch_orb(orb)
        with tarfile.open(orb / 'Vocabulary/ORBvoc.txt.tar.gz') as archive:
            member = archive.getmember('ORBvoc.txt')
            with archive.extractfile(member) as src, (orb / 'Vocabulary/ORBvoc.txt').open('wb') as dst:
                shutil.copyfileobj(src, dst)
        stamp.write_text(digest)
    run('cmake', '-S', pang, '-B', pang/'build', '-DCMAKE_BUILD_TYPE=Release',
        f'-DCMAKE_INSTALL_PREFIX={prefix}', '-DBUILD_EXAMPLES=OFF', '-DBUILD_TOOLS=OFF',
        '-DBUILD_PANGOLIN_PYTHON=OFF')
    run('cmake', '--build', pang/'build', '-j', a.jobs)
    run('cmake', '--install', pang/'build')
    db = orb/'Thirdparty/DBoW2'
    run('cmake', '-S', db, '-B', db/'build', '-DCMAKE_BUILD_TYPE=Release')
    run('cmake', '--build', db/'build', '-j', a.jobs)
    run('cmake', '-S', orb, '-B', orb/'build', '-DCMAKE_BUILD_TYPE=Release', f'-DCMAKE_PREFIX_PATH={prefix}')
    run('cmake', '--build', orb/'build', '--target', 'g2o', '-j', a.jobs)
    run('cmake', '--build', orb/'build', '--target', 'ORB_SLAM3', '-j', a.jobs)
    run('cmake', '-S', HERE/'backend', '-B', dest/'runner', f'-DORB_ROOT={orb}',
        f'-DCMAKE_PREFIX_PATH={prefix}', '-DCMAKE_BUILD_TYPE=Release')
    run('cmake', '--build', dest/'runner', '-j', a.jobs)
    config = {'binary': str(dest/'runner/umi_slam_runner'),
              'vocabulary': str(orb/'Vocabulary/ORBvoc.txt'),
              'library_paths': [str(orb/'lib'), str(orb/'Thirdparty/DBoW2/lib'),
                                str(orb/'Thirdparty/g2o/lib'), str(prefix/'lib')],
              'source_sha256': digest}
    (dest/'backend.json').write_text(json.dumps(config, indent=2)+'\n')
    print('Backend ready:', dest/'backend.json')


if __name__ == '__main__':
    main()
