#!/usr/bin/env python3
"""Open the readable Ubuntu GUI with the bundled UMI example and latest result."""
from pathlib import Path
import sys
import os
BUNDLE = Path(__file__).resolve().parent
# Use Ubuntu Tk/Xft for smooth Chinese fonts. This runtime is local to this bundle.
vendor = BUNDLE / '.runtime/ubuntu-tk/usr/lib/python3.10'
if vendor.is_dir():
    if Path(sys.executable).resolve() != Path('/usr/bin/python3').resolve():
        os.execv('/usr/bin/python3', ['/usr/bin/python3', str(Path(__file__).resolve()), *sys.argv[1:]])
    sys.path[:0] = [str(vendor), str(vendor / 'lib-dynload')]
    import ctypes
    blt = BUNDLE / '.runtime/ubuntu-tk/usr/lib/libBLT.2.5.so.8.6'
    if blt.is_file(): ctypes.CDLL(str(blt), mode=ctypes.RTLD_GLOBAL)
import tkinter as tk
sys.path.insert(0, str(BUNDLE / 'software' / 'ubuntu_viewer'))
from app import App

root = tk.Tk()
app = App(root)
app.values['input'].set(str(BUNDLE / 'examples' / 'lissajous_20260915' / 'input'))
app.values['output'].set(str(BUNDLE / 'results' / 'ubuntu_demo'))
app.values['settings'].set(str(BUNDLE / 'software' / 'umi_slam' / 'profiles' / 'umi_20260908.yaml'))
app.values['profile'].set(str(BUNDLE / 'software' / 'umi_slam' / 'profiles' / 'umi_20260908.json'))
app.engine.set('Docker')
app.stride.set('5')
app.prepare.set(False)
app.status.set('示例已就绪 · 点击“开始处理”')
completed = [p for p in (BUNDLE / 'results/ubuntu_demo').glob('run_*') if p.is_dir() and (p / 'quality.json').is_file()]
if completed:
    app.restore_job(max(completed, key=lambda p: p.stat().st_mtime))
root.update()
print('Ubuntu GUI ready: smooth fonts, restored result and embedded report.', flush=True)
root.mainloop()
