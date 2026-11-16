"""Cross-platform Docker job execution, no shell and no host numeric packages."""
import csv
import io
import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid


def process_options():
    return {'creationflags': subprocess.CREATE_NO_WINDOW} if os.name == 'nt' else {}


def docker_call(args, **kwargs):
    return subprocess.run(['docker', *args], **process_options(), **kwargs)


def diagnose(image='umi-slam:0.1'):
    info = docker_call(['info','--format','{{.OSType}}'], capture_output=True, text=True, timeout=20)
    if info.returncode: raise RuntimeError(info.stderr.strip() or 'Docker 未启动')
    if info.stdout.strip() != 'linux': raise RuntimeError('请将 Docker Desktop 切换为 Linux 容器')
    result = docker_call(['image','inspect',image], capture_output=True, text=True, timeout=20)
    if result.returncode: raise RuntimeError('没有找到后端镜像，请先导入离线镜像：'+image)
    entry = json.loads(result.stdout)[0]
    if entry.get('Os') != 'linux' or entry.get('Architecture') != 'amd64':
        raise RuntimeError('需要 Linux amd64 后端镜像')
    return {'image':image, 'id':entry['Id'], 'os':entry['Os'], 'architecture':entry['Architecture']}


def bind_mount(host, target, readonly=False):
    # --mount is CSV encoded, preserving drive letters, Chinese, spaces and commas.
    stream = io.StringIO()
    csv.writer(stream, lineterminator='').writerow(['type=bind', 'source='+str(host), 'target='+target] + (['readonly'] if readonly else []))
    return stream.getvalue()


def command(source, output, settings, profile, stride=5, max_frames=0, timeout=0,
            prepare_only=False, image='umi-slam:0.1', name=None, windows=None):
    source,output,settings,profile = (Path(x).expanduser().resolve() for x in (source,output,settings,profile))
    if not source.is_dir() or not settings.is_file() or not profile.is_file(): raise ValueError('输入、标定或设备配置不存在')
    if source == output or source.is_relative_to(output) or output.is_relative_to(source): raise ValueError('输入和结果目录不能互相嵌套')
    if output.exists(): raise ValueError('结果目录已存在，不能覆盖')
    if stride < 1 or max_frames < 0 or not math.isfinite(timeout) or timeout < 0: raise ValueError('运行参数无效')
    if not image or image.startswith('-') or any(c.isspace() for c in image): raise ValueError('镜像名称无效')
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = ['docker','run','--pull','never','--rm','--init','--network','none','--name',name or 'slam-'+uuid.uuid4().hex]
    is_windows = os.name == 'nt' if windows is None else windows
    if not is_windows: cmd += ['--user', f'{os.getuid()}:{os.getgid()}']
    for host,target,ro in [(source,'/input',True),(output.parent,'/output',False),(settings,'/config/settings.yaml',True),(profile,'/config/profile.json',True)]:
        cmd += ['--mount',bind_mount(host,target,ro)]
    cmd += [image,'--input','/input','--output','/output/'+output.name,'--settings','/config/settings.yaml',
            '--profile','/config/profile.json','--stride',str(stride),'--max-frames',str(max_frames),'--timeout',str(timeout)]
    if prepare_only: cmd += ['--prepare-only']
    return cmd


def execute(cmd, cancel, log):
    """Stop this named container, including cancellation while docker creates it."""
    if cancel.is_set(): return 130
    name = cmd[cmd.index('--name')+1]
    process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, **process_options())
    cancel_deadline = None
    while process.poll() is None:
        if cancel.wait(.2):
            if cancel_deadline is None: cancel_deadline = time.monotonic()+60
            if time.monotonic() > cancel_deadline:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait(timeout=5)
                raise RuntimeError('无法确认容器停止，请恢复 Docker 后检查容器：'+name)
            try:
                stopped = docker_call(['stop','--time','10',name], stdout=log, stderr=log, timeout=20)
            except subprocess.TimeoutExpired:
                stopped = None
            if stopped is not None and stopped.returncode == 0:
                try: process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.terminate(); process.wait(timeout=5)
                return 130
            # Container may not exist yet; retry instead of orphaning a late start.
            time.sleep(.3)
    return 130 if cancel.is_set() else process.returncode
