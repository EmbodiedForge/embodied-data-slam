#!/usr/bin/env python3
"""Ubuntu desktop frontend for the offline IMU trajectory pipeline."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import webbrowser

HERE = Path(__file__).resolve().parent


class App:
    def __init__(self, root):
        self.root = root; self.process = None; self.logfile = None; self.job = None
        root.title('UMI · 双目 IMU 轨迹恢复'); root.geometry('920x690')
        self.values = {}
        frame = ttk.Frame(root, padding=20); frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='UMI 轨迹后处理', font=('', 20, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='输出 IMU 原点位姿 · 双目 + IMU · 本地离线处理').pack(anchor='w', pady=(3,15))
        rows = ttk.Frame(frame); rows.pack(fill='x'); rows.columnconfigure(1, weight=1)
        fields = [('input','采集数据目录',''),('output','结果保存目录',str(HERE.parent/'outputs/umi_app')),
                  ('settings','双目 IMU 标定',str(HERE/'profiles/umi_20260908.yaml')),
                  ('profile','数据格式配置',str(HERE/'profiles/umi_20260908.json')),
                  ('backend','计算后端配置',str(HERE/'.build/backend.json'))]
        self.backend_widgets=[]
        for row,(key,label,default) in enumerate(fields):
            var=tk.StringVar(value=default); self.values[key]=var
            widgets=[ttk.Label(rows,text=label),ttk.Entry(rows,textvariable=var),ttk.Button(rows,text='选择…',command=lambda k=key:self.choose(k))]
            widgets[0].grid(row=row,column=0,sticky='w',padx=(0,10),pady=5)
            widgets[1].grid(row=row,column=1,sticky='ew')
            widgets[2].grid(row=row,column=2,padx=(8,0))
            if key=='backend': self.backend_widgets=widgets
        controls=ttk.Frame(frame); controls.pack(fill='x',pady=12)
        self.engine=tk.StringVar(value='本机' if (HERE/'.build/backend.json').is_file() else 'Docker')
        self.engine.trace_add('write',self.select_engine)
        self.select_engine()
        ttk.Label(controls,text='运行方式：').pack(side='left')
        ttk.Combobox(controls,textvariable=self.engine,values=['Docker','本机'],state='readonly',width=7).pack(side='left',padx=(0,10))
        ttk.Label(controls,text='每 N 帧处理一帧：').pack(side='left')
        self.stride=tk.StringVar(value='5'); ttk.Spinbox(controls,from_=1,to=30,width=5,textvariable=self.stride).pack(side='left')
        self.prepare=tk.BooleanVar(); ttk.Checkbutton(controls,text='仅检查并准备数据',variable=self.prepare).pack(side='left',padx=20)
        self.start_button=ttk.Button(controls,text='开始处理',command=self.start); self.start_button.pack(side='left')
        self.stop_button=ttk.Button(controls,text='停止',command=self.stop,state='disabled'); self.stop_button.pack(side='left',padx=8)
        ttk.Label(frame,text='附带标定仅适用于现有 2026-09 UMI；新设备须选择对应标定。',foreground='#a15c00').pack(anchor='w')
        self.status=tk.StringVar(value='就绪'); ttk.Label(frame,textvariable=self.status,font=('',12,'bold')).pack(anchor='w',pady=10)
        self.log=tk.Text(frame,height=18,wrap='word',state='disabled'); self.log.pack(fill='both',expand=True)
        bottom=ttk.Frame(frame); bottom.pack(fill='x',pady=(10,0))
        ttk.Button(bottom,text='查看轨迹报告',command=self.report).pack(side='left')
        ttk.Button(bottom,text='打开结果目录',command=self.open_folder).pack(side='left',padx=8)
        root.protocol('WM_DELETE_WINDOW',self.close)
        root.after(500,self.poll)

    def choose(self,key):
        value=filedialog.askdirectory() if key in ('input','output') else filedialog.askopenfilename()
        if value: self.values[key].set(value)

    def select_engine(self,*_):
        for widget in self.backend_widgets:
            widget.grid_remove() if self.engine.get()=='Docker' else widget.grid()

    def start(self):
        if self.process: return
        try:
            stride=int(self.stride.get())
            if stride<1: raise ValueError('抽帧间隔必须大于零')
            for key in ('input','output','settings'):
                if not self.values[key].get().strip(): raise ValueError('请选择输入、输出和标定')
            parent=Path(self.values['output'].get()).expanduser(); parent.mkdir(parents=True,exist_ok=True)
            self.job=parent/('run_'+time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns()%1000000))
            entry='docker_runner.py' if self.engine.get()=='Docker' else 'pipeline.py'
            args=[sys.executable,'-u',str(HERE/entry),'--input',self.values['input'].get(),
                  '--output',str(self.job),'--settings',self.values['settings'].get(),'--stride',str(stride)]
            for key in (('profile',) if self.engine.get()=='Docker' else ('profile','backend')):
                if self.values[key].get(): args.extend(['--'+key,self.values[key].get()])
            if self.prepare.get(): args.append('--prepare-only')
            self.logfile=(parent/(self.job.name+'.log')).open('w')
            self.process=subprocess.Popen(args,stdout=self.logfile,stderr=subprocess.STDOUT,start_new_session=True)
            self.status.set('准备数据…'); self.start_button.configure(state='disabled'); self.stop_button.configure(state='normal')
        except Exception as e: messagebox.showerror('无法开始',str(e))

    def stop(self):
        if self.process and self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM); self.status.set('正在停止…')

    def poll(self):
        if self.process:
            text=''
            for path in (Path(self.logfile.name),self.job/'slam.log'):
                if path.exists():
                    with path.open('rb') as stream:
                        stream.seek(max(0,path.stat().st_size-18000)); text+=stream.read().decode('utf-8',errors='replace')+'\n'
            self.log.configure(state='normal'); self.log.delete('1.0','end'); self.log.insert('end',text); self.log.see('end'); self.log.configure(state='disabled')
            code=self.process.poll()
            if code is not None:
                self.logfile.close(); self.process=None
                state={}
                if (self.job/'job.json').exists(): state=json.loads((self.job/'job.json').read_text())
                labels={'prepared':'数据准备完成','completed':'处理完成','failed':'处理失败','cancelled':'已停止'}
                result=labels.get(state.get('status'),'处理失败' if code else '完成')
                if 'coverage_fraction' in state: result+=f" · 轨迹覆盖率 {state['coverage_fraction']:.1%}"
                self.status.set(result); self.start_button.configure(state='normal'); self.stop_button.configure(state='disabled')
            elif (self.job/'job.json').exists():
                state=json.loads((self.job/'job.json').read_text())
                if state.get('status')=='running': self.status.set('正在恢复轨迹…（进度见日志）')
        self.root.after(500,self.poll)

    def report(self):
        if self.job and (self.job/'report.html').exists(): webbrowser.open((self.job/'report.html').resolve().as_uri())
        else: messagebox.showinfo('轨迹报告','完成轨迹恢复后可查看报告。')

    def open_folder(self):
        if self.job and self.job.exists(): subprocess.Popen(['xdg-open',str(self.job)])

    def close(self):
        if self.process and self.process.poll() is None:
            self.stop(); self.root.after(300,self.close)
        else: self.root.destroy()


if __name__=='__main__':
    root=tk.Tk(); App(root); root.mainloop()
