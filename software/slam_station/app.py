#!/usr/bin/env python3
"""SLAM Station: Windows/Linux local Docker desktop."""
import json
import os
from pathlib import Path
import queue
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import webbrowser

from devices import load_device
from jobs import Job
from runtime import diagnose

HERE=Path(__file__).resolve().parent
BUNDLE=Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else HERE.parents[1]
PROFILES=HERE.parent/"umi_slam/profiles"
if not PROFILES.is_dir(): PROFILES=HERE/"profiles"
LABELS={'queued':'等待中','checking':'检查 Docker…','running':'正在处理','preparing':'准备数据',
        'prepared':'数据准备完成','completed':'处理完成','failed':'处理失败','cancelled':'已停止'}


def open_path(path):
    path=Path(path).resolve()
    if os.name == 'nt': os.startfile(str(path))
    elif sys.platform == 'darwin': subprocess.Popen(['open',str(path)])
    else: subprocess.Popen(['xdg-open',str(path)])


def read_json(path):
    try: return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError): return {}


class App:
    def __init__(self, root):
        self.root=root; self.job=None; self.selected=None; self.events=queue.Queue(); self.closing=False
        root.title('SLAM Station · UMI / EGO'); root.geometry('1080x800'); root.minsize(900,700)
        style=ttk.Style(root); style.theme_use('clam')
        style.configure('TFrame',background='#f4f6f9'); style.configure('TLabel',background='#f4f6f9',foreground='#203047')
        style.configure('TButton',padding=(12,7)); style.configure('Title.TLabel',font=('',22,'bold'))
        style.configure('Accent.TButton',background='#146b65',foreground='white')
        style.map('Accent.TButton',background=[('active','#20877f'),('disabled','#9aadaa')])
        outer=ttk.Frame(root,padding=22); outer.pack(fill='both',expand=True)
        header=ttk.Frame(outer); header.pack(fill='x')
        ttk.Label(header,text='SLAM Station',style='Title.TLabel').pack(side='left')
        ttk.Label(header,text='UMI / EGO   ·   离线轨迹后处理').pack(side='left',padx=18)
        self.environment=tk.StringVar(value='后端尚未检查')
        ttk.Label(outer,textvariable=self.environment).pack(anchor='w',pady=(6,14))
        tabs=ttk.Notebook(outer); tabs.pack(fill='both',expand=True)
        task=ttk.Frame(tabs,padding=16); devices=ttk.Frame(tabs,padding=16); history=ttk.Frame(tabs,padding=16)
        tabs.add(task,text='  处理任务  '); tabs.add(devices,text='  设备与扩展  '); tabs.add(history,text='  历史结果  ')
        form=ttk.Frame(task); form.pack(fill='x'); form.columnconfigure(1,weight=1)
        self.values={}; self.input_widgets=[]
        defaults=[('source','采集目录',''),('parent','结果保存目录',str(Path.home()/'SLAM_Results')),
                  ('settings','双目 / IMU 标定 YAML',''),('device','设备配置 JSON',str(HERE/'devices/generic_euroc.json'))]
        for row,(key,label,default) in enumerate(defaults):
            self.values[key]=tk.StringVar(value=default)
            ttk.Label(form,text=label).grid(row=row,column=0,sticky='w',padx=(0,12),pady=6)
            entry=ttk.Entry(form,textvariable=self.values[key]); entry.grid(row=row,column=1,sticky='ew')
            button=ttk.Button(form,text='浏览…',command=lambda k=key:self.choose(k)); button.grid(row=row,column=2,padx=(8,0))
            self.input_widgets += [entry,button]
        options=ttk.Frame(task); options.pack(fill='x',pady=12)
        self.stride=tk.StringVar(value='5'); self.limit=tk.StringVar(value='0'); self.timeout=tk.StringVar(value='0')
        for label,var,width in [('抽帧间隔',self.stride,5),('最多帧数（0=全部）',self.limit,7),('超时秒数（0=自动）',self.timeout,7)]:
            ttk.Label(options,text=label).pack(side='left',padx=(0,6))
            entry=ttk.Entry(options,textvariable=var,width=width); entry.pack(side='left',padx=(0,16)); self.input_widgets.append(entry)
        self.prepare=tk.BooleanVar(value=False)
        check=ttk.Checkbutton(task,text='仅检查并准备数据（不运行 SLAM）',variable=self.prepare); check.pack(anchor='w'); self.input_widgets.append(check)
        controls=ttk.Frame(task); controls.pack(fill='x',pady=12)
        self.start_button=ttk.Button(controls,text='开始处理',style='Accent.TButton',command=self.start); self.start_button.pack(side='left')
        self.stop_button=ttk.Button(controls,text='停止任务',command=self.stop,state='disabled'); self.stop_button.pack(side='left',padx=8)
        ttk.Button(controls,text='检查 Docker',command=self.check_docker).pack(side='left')
        self.example_button=ttk.Button(controls,text='填入现有 UMI 示例',command=self.example); self.example_button.pack(side='left',padx=8)
        self.status=tk.StringVar(value='请选择采集目录、标定和设备配置。')
        ttk.Label(task,textvariable=self.status,font=('',11,'bold'),wraplength=940).pack(anchor='w',pady=(0,8))
        self.progress=ttk.Progressbar(task,mode='indeterminate'); self.progress.pack(fill='x',pady=(0,8))
        self.log=tk.Text(task,height=12,background='#142332',foreground='#dbe8ed',insertbackground='white',wrap='word',state='disabled',font=('Consolas',10))
        self.log.pack(fill='both',expand=True)
        bottom=ttk.Frame(task); bottom.pack(fill='x',pady=(10,0))
        ttk.Button(bottom,text='查看轨迹报告',command=self.report).pack(side='left')
        ttk.Button(bottom,text='打开任务目录',command=self.folder).pack(side='left',padx=8)
        ttk.Label(bottom,text='轨迹默认是 IMU 原点位姿；覆盖率不代表定位精度。').pack(side='right')
        ttk.Label(devices,text='设备接口 v1',font=('',16,'bold')).pack(anchor='w')
        ttk.Label(devices,text='现有 UMI 原始协议可直接处理；新 UMI / EGO 先转换为同步双目 + IMU（EuRoC）。',wraplength=900).pack(anchor='w',pady=10)
        self.preset=tk.StringVar(value='generic_euroc.json')
        combo=ttk.Combobox(devices,textvariable=self.preset,values=sorted(p.name for p in (HERE/'devices').glob('*.json')),state='readonly',width=36)
        combo.pack(anchor='w'); combo.bind('<<ComboboxSelected>>',lambda _:self.show_device())
        self.device_text=tk.Text(devices,height=19,wrap='word',state='disabled'); self.device_text.pack(fill='both',expand=True,pady=12)
        dbuttons=ttk.Frame(devices); dbuttons.pack(fill='x')
        ttk.Button(dbuttons,text='使用此设备模板',command=self.use_device).pack(side='left')
        ttk.Button(dbuttons,text='另存模板…',command=self.save_template).pack(side='left',padx=8)
        ttk.Button(dbuttons,text='查看接口说明',command=lambda:open_path(HERE/'DEVICE_API.md')).pack(side='left')
        ttk.Label(devices,text='外参留空时不生成夹爪/头戴坐标轨迹。辅助夹爪、手部或关节流仅保留接口，当前不做人体姿态估计。',wraplength=900).pack(anchor='w',pady=12)
        ttk.Label(history,text='当前结果保存目录中的任务',font=('',14,'bold')).pack(anchor='w')
        ttk.Button(history,text='刷新列表',command=self.refresh_history).pack(anchor='w',pady=10)
        self.history=ttk.Treeview(history,columns=('status','device'),show='tree headings')
        self.history.heading('#0',text='任务'); self.history.heading('status',text='状态'); self.history.heading('device',text='设备')
        self.history.column('#0',width=350); self.history.column('status',width=110); self.history.column('device',width=280)
        self.history.pack(fill='both',expand=True); self.history.bind('<<TreeviewSelect>>',self.select_history)
        hbuttons=ttk.Frame(history); hbuttons.pack(fill='x',pady=10)
        ttk.Button(hbuttons,text='打开选中任务',command=self.folder).pack(side='left')
        ttk.Button(hbuttons,text='查看选中报告',command=self.report).pack(side='left',padx=8)
        self.show_device(); self.refresh_history()
        root.protocol('WM_DELETE_WINDOW',self.close); root.after(300,self.poll)

    def choose(self,key):
        value=filedialog.askdirectory() if key in ('source','parent') else filedialog.askopenfilename(filetypes=[('Configuration','*.json *.yaml *.yml'),('All files','*')])
        if value: self.values[key].set(value)
        if key == 'parent': self.refresh_history()

    def show_device(self):
        path=HERE/'devices'/self.preset.get()
        self.device_text.configure(state='normal'); self.device_text.delete('1.0','end')
        self.device_text.insert('end',path.read_text(encoding='utf-8')); self.device_text.configure(state='disabled')

    def use_device(self):
        if self.job and self.job.active: return
        self.values['device'].set(str(HERE/'devices'/self.preset.get()))
        self.values['settings'].set('')
        self.status.set('已选择设备模板，请选择该设备对应的标定 YAML。')

    def save_template(self):
        path=filedialog.asksaveasfilename(defaultextension='.json',initialfile=self.preset.get())
        if path: Path(path).write_text((HERE/'devices'/self.preset.get()).read_text(encoding='utf-8'),encoding='utf-8')

    def example(self):
        if self.job and self.job.active: return
        self.values['source'].set(str(BUNDLE/'examples/lissajous_20260915/input'))
        self.values['settings'].set(str(PROFILES/'umi_20260908.yaml'))
        self.values['device'].set(str(HERE/'devices/umi_20260908.json')); self.stride.set('5')
        self.status.set('已填入历史 UMI 示例；此标定仅适用于这套设备。')

    def check_docker(self):
        self.environment.set('正在检查 Docker…')
        def work():
            try:
                info=diagnose(); self.events.put('Docker 已就绪 · Linux amd64 · '+info['image'])
            except Exception as e: self.events.put('Docker 检查失败：'+str(e))
        threading.Thread(target=work,daemon=True).start()

    def start(self):
        if self.job and self.job.active: return
        try:
            if any(not v.get().strip() for v in self.values.values()): raise ValueError('请选择采集目录、结果目录、标定和设备配置')
            self.job=Job(**{k:v.get() for k,v in self.values.items()},stride=int(self.stride.get()),
                         max_frames=int(self.limit.get()),timeout=float(self.timeout.get()),prepare_only=self.prepare.get())
            self.selected=self.job.folder; self.job.start(); self.progress.start(12)
            for w in self.input_widgets+[self.start_button,self.example_button]: w.configure(state='disabled')
            self.stop_button.configure(state='normal'); self.refresh_history()
        except Exception as e: messagebox.showerror('无法开始',str(e))

    def stop(self):
        if self.job and self.job.active:
            self.job.stop(); self.status.set('正在停止容器并保存状态…'); self.stop_button.configure(state='disabled')

    def poll(self):
        try:
            while True: self.environment.set(self.events.get_nowait())
        except queue.Empty: pass
        if self.job:
            state=read_json(self.job.folder/'station_job.json')
            backend=read_json(self.job.result/'job.json')
            label=LABELS.get(state.get('status'),state.get('status',''))
            if self.job.active and self.job.cancel.is_set(): label='正在停止容器并保存状态…'
            elif self.job.active and state.get('status') == 'running': label=LABELS.get(backend.get('status'),'正在处理')
            if state.get('coverage_fraction') is not None: label+=f" · 轨迹覆盖率 {state['coverage_fraction']:.1%}"
            if state.get('error'): label+=' · '+state['error']
            self.status.set(label)
            texts=[]
            for path in (self.job.folder/'host.log',self.job.result/'slam.log'):
                try:
                    with path.open('rb') as f:
                        f.seek(0,2); f.seek(max(0,f.tell()-16000)); texts.append(f.read().decode('utf-8',errors='replace'))
                except OSError: pass
            self.log.configure(state='normal'); self.log.delete('1.0','end'); self.log.insert('end','\n'.join(texts)); self.log.see('end'); self.log.configure(state='disabled')
            if not self.job.active:
                self.progress.stop(); self.stop_button.configure(state='disabled')
                for w in self.input_widgets+[self.start_button,self.example_button]: w.configure(state='normal')
        if self.closing:
            if not self.job or not self.job.active: self.root.destroy(); return
        self.root.after(500,self.poll)

    def refresh_history(self):
        self.history.delete(*self.history.get_children())
        parent=Path(self.values['parent'].get()).expanduser()
        for path in sorted(parent.glob('run_*/station_job.json'),reverse=True)[:200]:
            d=read_json(path); self.history.insert('', 'end',iid=str(path.parent),text=path.parent.name,
                values=(LABELS.get(d.get('status'),d.get('status','未知')),d.get('device',{}).get('name','')))

    def select_history(self,_):
        values=self.history.selection()
        if values: self.selected=Path(values[0])

    def folder(self):
        if self.selected and self.selected.is_dir(): open_path(self.selected)
        else: messagebox.showinfo('结果','请先运行任务或选择历史任务。')

    def report(self):
        path=self.selected/'result/report.html' if self.selected else None
        if path and path.is_file(): webbrowser.open(path.resolve().as_uri())
        else: messagebox.showinfo('报告','选中的任务还没有轨迹报告；仅准备数据不会生成轨迹。')

    def close(self):
        self.closing=True
        if self.job and self.job.active: self.stop(); self.status.set('等待容器停止后关闭…')
        else: self.root.destroy()


def main():
    root=tk.Tk(); App(root); root.mainloop()


if __name__=='__main__': main()
