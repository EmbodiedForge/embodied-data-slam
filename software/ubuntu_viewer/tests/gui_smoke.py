"""Run with the workstation's locally extracted Tk and DISPLAY set."""
from pathlib import Path
import sys,ctypes,subprocess
ROOT=Path(__file__).resolve().parents[3]
vendor=ROOT/'.runtime/ubuntu-tk/usr/lib/python3.10'
sys.path[:0]=[str(vendor),str(vendor/'lib-dynload'),str(ROOT/'software/ubuntu_viewer')]
ctypes.CDLL(str(ROOT/'.runtime/ubuntu-tk/usr/lib/libBLT.2.5.so.8.6'),mode=ctypes.RTLD_GLOBAL)
import tkinter as tk
from tkinter import font
from app import App,progress_from_log,read_trajectory
job=ROOT/'results/ubuntu_demo/run_20260922_163954_647896'
assert progress_from_log('PREPARE 100/200','preparing')==(50,'准备数据：100 / 200 帧')
assert progress_from_log('UMI_PROGRESS 4047/4047','running')[1].startswith('SLAM 帧处理完成')
assert progress_from_log('', 'completed')[0]==100
r=tk.Tk(); app=App(r)
app.values['input'].set(str(ROOT/'examples/lissajous_20260915/input'))
app.values['output'].set(str(ROOT/'results/ubuntu_demo'))
app.restore_job(job);r.update()
assert '84.19%' in app.status.get()
assert '3,407 / 4,047' in app.detail.get()
assert app.progress['value']==100
assert font.nametofont('TkDefaultFont').actual('family')=='Noto Sans CJK SC'
assert len(read_trajectory(job))==3407
assert len(app.preview.winfo_children())==3
for canvas in app.preview.winfo_children():
 lines=[item for item in canvas.find_all() if canvas.type(item)=='line' and canvas.itemcget(item,'width')=='2.5']
 assert len(lines)==3, 'Trajectory segments must not be connected'
app.report();r.update();assert app.report_window.winfo_exists()
app.report_window.destroy()
app.zoom.set('125%');app.change_zoom();r.update()
assert font.nametofont('TkDefaultFont').actual('size')==15
app.zoom.set('100%');app.change_zoom();r.update()
subprocess.run(['ffmpeg','-y','-loglevel','error','-f','x11grab','-video_size',f'{r.winfo_width()}x{r.winfo_height()}','-i',':91','-frames:v','1',str(ROOT/'software/ubuntu_viewer/界面预览.png')],check=True)
app.close();print('GUI passed: smooth CJK fonts, restored completed job, 3 gap-preserving projections, embedded report, zoom, progress.')
