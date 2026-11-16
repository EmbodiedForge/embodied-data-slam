"""Readable Ubuntu frontend; original pipeline and archived GUI remain untouched."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import re
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox, font

HERE=Path(__file__).resolve().parent
LEGACY=HERE.parent/'umi_slam'
spec=importlib.util.spec_from_file_location('umi_legacy_app',LEGACY/'app.py')
legacy=importlib.util.module_from_spec(spec); spec.loader.exec_module(legacy)


def read_json(path):
    try: return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError,ValueError): return {}


def read_trajectory(job):
    with (job/'imu_trajectory.csv').open(encoding='utf-8',newline='') as stream:
        rows=list(csv.DictReader(stream))
    if not rows: raise ValueError('轨迹文件为空')
    points=[]
    for row in rows:
        p=tuple(float(row[k]) for k in ('x_m','y_m','z_m'))
        if not all(math.isfinite(v) for v in p): raise ValueError('轨迹含无效坐标')
        points.append((p,row['segment_id']))
    return points


def progress_from_log(text, state):
    if state in ('completed','prepared'): return 100,'全部处理完成' if state=='completed' else '数据准备完成'
    if state in ('failed','cancelled'): return 0,'任务失败' if state=='failed' else '任务已停止'
    slam=re.findall(r'UMI_PROGRESS\s+(\d+)/(\d+)',text)
    if slam:
        done,total=map(int,slam[-1]); percent=100*done/max(total,1)
        return percent,('SLAM 帧处理完成，正在保存轨迹…' if done>=total else f'SLAM 处理：{done:,} / {total:,} 帧（{percent:.1f}%）')
    prepared=re.findall(r'PREPARE\s+(\d+)/(\d+)',text)
    if prepared:
        done,total=map(int,prepared[-1]); return 100*done/max(total,1),f'准备数据：{done:,} / {total:,} 帧'
    return 0,'正在检查输入并启动后端…'


def draw_plot(canvas,points,axes,label):
    canvas.delete('all'); w=canvas.winfo_width(); h=canvas.winfo_height()
    if w<80 or h<80: return
    x,y=axes; margin=48
    xs=[p[x] for p,_ in points]; ys=[p[y] for p,_ in points]
    minx,maxx,miny,maxy=min(xs),max(xs),min(ys),max(ys)
    spanx,spany=maxx-minx,maxy-miny
    scale=min(max(1,w-2*margin)/max(spanx,1e-5),max(1,h-2*margin-35)/max(spany,1e-5))
    cx,cy=(maxx+minx)/2,(maxy+miny)/2
    for k in range(1,5):
        xx=margin+(w-2*margin)*k/5; yy=margin+(h-2*margin)*k/5
        canvas.create_line(xx,margin,xx,h-margin,fill='#e6ebf0')
        canvas.create_line(margin,yy,w-margin,yy,fill='#e6ebf0')
    canvas.create_text(w/2,18,text=label+' 平面 · 米',fill='#34465b',font='TkDefaultFont')
    palette=['#087f8c','#df8f2d','#855bb5','#438943']
    groups=[]
    for p,segment in points:
        if not groups or groups[-1][0]!=segment: groups.append((segment,[]))
        groups[-1][1].extend([w/2+(p[x]-cx)*scale,h/2-(p[y]-cy)*scale])
    for i,(_,coords) in enumerate(groups):
        if len(coords)>=4: canvas.create_line(*coords,fill=palette[i%len(palette)],width=2.5)
    first=points[0][0]; last=points[-1][0]
    for p,color in ((first,'#179653'),(last,'#d64e52')):
        px=w/2+(p[x]-cx)*scale; py=h/2-(p[y]-cy)*scale
        canvas.create_oval(px-4,py-4,px+4,py+4,fill=color,outline='white')
    canvas.create_text(w/2,h-18,text=f'范围 {spanx:.3f} × {spany:.3f} m',fill='#526579',font='TkSmallFont')


class App(legacy.App):
    def __init__(self,root):
        self.root=root; self.process=None; self.logfile=None; self.job=None; self.report_window=None
        self.last_text=None; self.completed_job=None; self.external_open=None
        self.base_scaling=float(root.tk.call('tk','scaling'))
        # Explicit scalable fonts avoid the bitmap defaults of the old Conda Tk.
        families=set(font.families(root)); family='Noto Sans CJK SC' if 'Noto Sans CJK SC' in families else 'DejaVu Sans'
        for name in ('TkDefaultFont','TkTextFont','TkMenuFont','TkHeadingFont','TkCaptionFont','TkTooltipFont'):
            font.nametofont(name).configure(family=family,size=12)
        font.nametofont('TkFixedFont').configure(family='DejaVu Sans Mono',size=11)
        self.small_font=font.Font(root,name='TkSmallFont',family=family,size=10)
        self.title_font=font.Font(root,family=family,size=21,weight='bold')
        self.status_font=font.Font(root,family=family,size=16,weight='bold')
        factor=max(1,self.base_scaling/(96/72))
        self.factor=factor
        root.title('UMI 轨迹恢复 · Ubuntu')
        root.geometry(f'{min(int(1180*factor),int(root.winfo_screenwidth()*.90))}x{min(int(820*factor),int(root.winfo_screenheight()*.85))}')
        root.minsize(int(900*min(factor,1.3)),int(650*min(factor,1.3)))
        style=ttk.Style(root); style.theme_use('clam')
        style.configure('.',font='TkDefaultFont')
        style.configure('TFrame',background='#f4f7fb'); style.configure('TLabel',background='#f4f7fb',foreground='#223449')
        style.configure('TButton',padding=(12,7)); style.configure('TEntry',padding=5)
        style.configure('Accent.TButton',background='#087f8c',foreground='white')
        style.map('Accent.TButton',background=[('active','#086873'),('disabled','#a6b8bf')])
        style.configure('TProgressbar',background='#087f8c',troughcolor='#e0e8ef')
        frame=ttk.Frame(root,padding=24); frame.pack(fill='both',expand=True)
        header=ttk.Frame(frame); header.pack(fill='x')
        ttk.Label(header,text='UMI 轨迹恢复',font=self.title_font).pack(side='left')
        self.zoom=tk.StringVar(value='100%')
        combo=ttk.Combobox(header,textvariable=self.zoom,values=['100%','125%','150%','175%'],state='readonly',width=7)
        combo.pack(side='right'); combo.bind('<<ComboboxSelected>>',self.change_zoom)
        ttk.Label(header,text='界面字号  ').pack(side='right')
        ttk.Label(frame,text='双目 + IMU 离线后处理  ·  输出 IMU 原点轨迹').pack(anchor='w',pady=(3,14))
        rows=ttk.Frame(frame); rows.pack(fill='x'); rows.columnconfigure(1,weight=1)
        fields=[('input','采集数据目录',''),('output','结果保存目录',''),('settings','双目 / IMU 标定',str(LEGACY/'profiles/umi_20260908.yaml')),
                ('profile','数据格式配置',str(LEGACY/'profiles/umi_20260908.json')),('backend','本机后端配置',str(LEGACY/'.build/backend.json'))]
        self.values={}; self.backend_widgets=[]
        for i,(key,label,default) in enumerate(fields):
            self.values[key]=tk.StringVar(value=default)
            widgets=[ttk.Label(rows,text=label),ttk.Entry(rows,textvariable=self.values[key]),ttk.Button(rows,text='选择…',command=lambda k=key:self.choose(k))]
            for j,widget in enumerate(widgets): widget.grid(row=i,column=j,sticky='ew' if j==1 else 'w',padx=(0,10),pady=5)
            if key=='backend': self.backend_widgets=widgets
        options=ttk.Frame(frame); options.pack(fill='x',pady=12)
        self.engine=tk.StringVar(value='Docker'); self.engine.trace_add('write',self.select_engine); self.select_engine()
        ttk.Label(options,text='运行方式 ').pack(side='left'); ttk.Combobox(options,textvariable=self.engine,values=['Docker','本机'],state='readonly',width=7).pack(side='left',padx=(0,18))
        ttk.Label(options,text='每 N 帧取一帧 ').pack(side='left'); self.stride=tk.StringVar(value='5')
        ttk.Spinbox(options,from_=1,to=30,width=4,textvariable=self.stride).pack(side='left')
        self.prepare=tk.BooleanVar(); ttk.Checkbutton(options,text='仅准备数据',variable=self.prepare).pack(side='left',padx=18)
        self.start_button=ttk.Button(options,text='开始处理',command=self.start,style='Accent.TButton'); self.start_button.pack(side='left')
        self.stop_button=ttk.Button(options,text='停止',command=self.stop,state='disabled'); self.stop_button.pack(side='left',padx=8)
        self.status=tk.StringVar(value='就绪 · 选择数据后开始处理')
        self.state_label=ttk.Label(frame,textvariable=self.status,font=self.status_font,wraplength=int(1080*factor)); self.state_label.pack(anchor='w',pady=(8,3))
        self.detail=tk.StringVar(value='附带标定仅用于现有 UMI 示例；新设备请更换匹配标定。')
        ttk.Label(frame,textvariable=self.detail,wraplength=int(1080*factor)).pack(anchor='w',pady=(0,8))
        self.progress=ttk.Progressbar(frame,maximum=100); self.progress.pack(fill='x',pady=(0,10))
        actions=ttk.Frame(frame); actions.pack(fill='x',pady=(0,12))
        self.report_button=ttk.Button(actions,text='查看轨迹报告',command=self.report,style='Accent.TButton',state='disabled'); self.report_button.pack(side='left')
        ttk.Button(actions,text='打开结果目录',command=self.open_folder).pack(side='left',padx=10)
        ttk.Button(actions,text='浏览器打开 HTML',command=self.open_html).pack(side='left')
        ttk.Label(actions,text='绿点：起点  /  红点：终点  /  断线：轨迹缺口',font='TkSmallFont').pack(side='right')
        self.tabs=ttk.Notebook(frame); self.tabs.pack(fill='both',expand=True)
        self.preview=ttk.Frame(self.tabs); logs=ttk.Frame(self.tabs)
        self.tabs.add(self.preview,text='  轨迹预览  '); self.tabs.add(logs,text='  运行日志  ')
        ttk.Label(self.preview,text='完成后在这里显示三个平面的轨迹；运行日志可随时切换查看。').pack(pady=35)
        self.log=tk.Text(logs,font='TkFixedFont',wrap='word',state='disabled',height=10,background='#152535',foreground='#d9e7ef',padx=12,pady=12)
        scroll=ttk.Scrollbar(logs,command=self.log.yview); self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right',fill='y'); self.log.pack(fill='both',expand=True)
        root.protocol('WM_DELETE_WINDOW',self.close); root.after(500,self.poll)

    def change_zoom(self,_=None):
        ratio=int(self.zoom.get().rstrip('%'))/100
        for name in ('TkDefaultFont','TkTextFont','TkMenuFont','TkHeadingFont','TkCaptionFont','TkTooltipFont'):
            font.nametofont(name).configure(size=round(12*ratio))
        font.nametofont('TkFixedFont').configure(size=round(11*ratio))
        self.small_font.configure(size=round(10*ratio)); self.title_font.configure(size=round(21*ratio)); self.status_font.configure(size=round(16*ratio))

    def start(self):
        if self.process: return
        self.completed_job=None
        if self.report_window and self.report_window.winfo_exists(): self.report_window.destroy()
        super().start()
        if self.process:
            self.report_button.configure(state='disabled'); self.progress['value']=0
            self.state_label.configure(foreground='#223449'); self.tabs.select(1)

    def render_preview(self,parent):
        points=read_trajectory(self.job)
        for child in parent.winfo_children(): child.destroy()
        for i,(axes,label) in enumerate([((0,1),'XY'),((0,2),'XZ'),((1,2),'YZ')]):
            parent.columnconfigure(i,weight=1,uniform='plots')
            canvas=tk.Canvas(parent,background='white',highlightthickness=1,highlightbackground='#e0e8ef',width=250,height=230)
            canvas.grid(row=0,column=i,sticky='nsew',padx=4,pady=4)
            canvas.bind('<Configure>',lambda _,c=canvas,a=axes,l=label:draw_plot(c,points,a,l))
        parent.rowconfigure(0,weight=1)

    def restore_job(self,path):
        if self.process: return
        self.job=Path(path); self.apply_state(read_json(self.job/'job.json'))

    def apply_state(self,state,code=None):
        phase=state.get('status')
        if phase=='completed' and code in (None,0):
            quality=read_json(self.job/'quality.json')
            fraction=quality.get('coverage_fraction',state.get('coverage_fraction',0))
            self.status.set(f'处理完成  ·  轨迹覆盖率 {fraction:.2%}')
            self.detail.set(f"有效位姿 {quality.get('output_frames',0):,} / {quality.get('selected_frames',0):,} 帧  ·  {quality.get('segments','—')} 个片段  ·  可直接点击“查看轨迹报告”")
            self.state_label.configure(foreground='#13804b'); self.progress['value']=100
            self.report_button.configure(state='normal')
            if self.completed_job!=self.job:
                try: self.render_preview(self.preview); self.tabs.select(0)
                except (OSError,ValueError,KeyError) as e: self.detail.set('轨迹预览读取失败：'+str(e))
                self.completed_job=self.job
        elif phase in ('failed','cancelled','prepared') or code is not None:
            self.status.set({'failed':'处理失败','cancelled':'已停止','prepared':'数据准备完成 · 尚未运行 SLAM'}.get(phase,f'后端退出（{code}），未确认完成'))
            self.detail.set(state.get('error','仅准备数据不会生成轨迹报告。' if phase=='prepared' else '详细信息见运行日志。'))
            self.state_label.configure(foreground='#b55818' if phase!='prepared' else '#223449')
            self.progress['value']=100 if phase=='prepared' else 0

    def poll(self):
        if self.job:
            text=''
            paths=[self.job.parent/(self.job.name+'.log'),self.job/'slam.log']
            for path in paths:
                try:
                    with path.open('rb') as f:
                        f.seek(0,2); f.seek(max(0,f.tell()-20000)); text+=f.read().decode('utf-8',errors='replace')+'\n'
                except OSError: pass
            if text!=self.last_text:
                self.log.configure(state='normal'); self.log.delete('1.0','end'); self.log.insert('end',text); self.log.see('end'); self.log.configure(state='disabled'); self.last_text=text
            if self.process:
                state=read_json(self.job/'job.json'); code=self.process.poll()
                if code is None:
                    value,label=progress_from_log(text,state.get('status'))
                    self.progress['value']=value; self.status.set(label)
                    self.detail.set('当前任务：'+self.job.name+'  ·  完成后自动显示轨迹预览')
                else:
                    self.logfile.close(); self.process=None
                    self.start_button.configure(state='normal'); self.stop_button.configure(state='disabled'); self.apply_state(state,code)
        self.root.after(500,self.poll)

    def report(self):
        if not self.job or not (self.job/'imu_trajectory.csv').is_file():
            messagebox.showinfo('轨迹报告','当前没有可显示的轨迹，请等待完整处理完成。'); return
        try:
            if self.report_window and self.report_window.winfo_exists():
                self.report_window.lift(); self.report_window.focus_force(); return
            window=tk.Toplevel(self.root); self.report_window=window; window.title('轨迹报告 · '+self.job.name)
            window.geometry(f'{min(int(1250*self.factor),int(self.root.winfo_screenwidth()*.9))}x{min(int(620*self.factor),int(self.root.winfo_screenheight()*.8))}')
            frame=ttk.Frame(window,padding=24); frame.pack(fill='both',expand=True)
            ttk.Label(frame,text=self.status.get(),font=self.status_font).pack(anchor='w')
            ttk.Label(frame,text=self.detail.get()).pack(anchor='w',pady=8)
            plots=ttk.Frame(frame); plots.pack(fill='both',expand=True); self.render_preview(plots)
            ttk.Label(frame,text='IMU 原点位姿 · 米 · 各图横纵轴等比例；绿色起点、红色终点，不连接缺失轨迹段。\n覆盖率表示有效轨迹占比，不代表定位精度。',font='TkSmallFont').pack(anchor='w',pady=12)
            window.lift(); window.focus_force()
        except Exception as e: messagebox.showerror('无法显示轨迹报告',str(e))

    def open_html(self):
        path=self.job/'report.html' if self.job else None
        if not path or not path.is_file(): messagebox.showinfo('HTML 报告','当前任务还没有生成 HTML 报告。'); return
        try:
            self.external_open=subprocess.Popen(['xdg-open',str(path)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            self.root.after(1000,self.check_html_open)
        except OSError as e: messagebox.showerror('浏览器启动失败',str(e)+'\n可以用“查看轨迹报告”直接在软件里查看。')

    def check_html_open(self):
        if self.external_open and self.external_open.poll() not in (None,0):
            messagebox.showerror('浏览器启动失败','系统浏览器没有成功启动。请用“查看轨迹报告”在软件里查看。')
