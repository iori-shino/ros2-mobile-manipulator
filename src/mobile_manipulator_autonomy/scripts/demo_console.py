"""Chinese desktop console for the mobile manipulation course demonstration."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import tkinter as tk
from tkinter import ttk

ROOT = Path(__file__).resolve().parents[3]
BG, PANEL, FG, MUTED, ACCENT = '#101929', '#1b2940', '#f2f5fb', '#aabbd3', '#ffb567'
FONT = 'Noto Sans CJK SC'
MODES = ('三色自主分类', '红块携物绕障')
STAGES = {
    'IDLE': '等待机器人和 Nav2 就绪', 'OBSERVE': '用 RGB 相机识别方块',
    'NAVIGATE_TO_PICK': '导航到取物位', 'VERIFY_PICK_DOCK': '取物位停稳',
    'REFRESH_VISION': '停稳后重新定位', 'CHECK_REACHABILITY': '检查机械臂可达性',
    'PICK': '接近、夹取并抬升', 'VERIFY_GRASP': '确认双指接触',
    'STOW_ARM': '收臂到运输姿态', 'NAVIGATE_TO_PLACE': 'Nav2 携物运输',
    'VERIFY_PLACE_DOCK': 'LiDAR 对准放置位', 'PLACE': '下降并释放方块',
    'VERIFY_RELEASE': '确认落稳', 'MARK_COMPLETE': '完成本次放置',
    'CLEAR_PLACEMENT_ARM': '机械臂收回', 'RETURN_TO_OBSERVE': '返回取物区',
    'TASK_COMPLETE': '任务结束，正在独立核对', 'FAILED': '任务中止，正在保存记录',
    'STOPPED': '已请求停止任务',
}


class Console:
    def __init__(self, root, mode, seed, headless):
        self.root, self.case, self.proc = root, None, None
        self.mode = tk.StringVar(value=MODES[mode])
        self.seed = tk.StringVar(value=str(seed))
        self.headless = tk.BooleanVar(value=headless)
        self.status = tk.StringVar(value='等待开始')
        self.detail = tk.StringVar()
        self.path = tk.StringVar()
        self.colors = {}
        self.log = None
        root.title('移动操作机器人 · 课程演示')
        width, height = 910, 585
        x = (root.winfo_screenwidth()-width)//2
        y = (root.winfo_screenheight()-height)//2
        root.geometry(f'{width}x{height}+{max(x,0)}+{max(y,0)}')
        root.minsize(760, 545)
        root.configure(bg=BG)
        style = ttk.Style(root)
        style.theme_use('clam')
        for name in ('TCombobox', 'TEntry', 'TButton', 'TCheckbutton'):
            style.configure(name, font=(FONT, 10), padding=6)
        style.configure('TCheckbutton', background=PANEL, foreground=FG)
        style.configure('TButton', background='#334962', foreground=FG)
        style.map('TButton', background=[('active', '#456582'), ('disabled', '#263349')])
        style.configure('TEntry', fieldbackground='#f4f6fb')
        style.configure('TCombobox', fieldbackground='#f4f6fb')
        section = tk.Frame(root,bg=BG,padx=24,pady=17)
        section.pack(fill='both',expand=True)
        self.label(section,'移动操作机器人',21,FG,bold=True).pack(anchor='w')
        self.label(section,'自主导航 · 视觉定位 · 物理抓取',11,MUTED).pack(anchor='w',pady=(1,12))
        settings = tk.Frame(section,bg=PANEL,padx=14,pady=10)
        settings.pack(fill='x')
        self.label(settings,'演示模式',10,MUTED,PANEL).grid(row=0,column=0,sticky='w')
        self.label(settings,'随机种子',10,MUTED,PANEL).grid(row=0,column=2,sticky='w',padx=(16,0))
        self.choice=ttk.Combobox(settings, values=MODES,textvariable=self.mode,state='readonly',width=21)
        self.choice.grid(row=1,column=0,columnspan=2,sticky='ew')
        self.choice.bind('<<ComboboxSelected>>',lambda _: self.refresh_mode())
        settings.columnconfigure(1,weight=1)
        self.seed_entry=ttk.Entry(settings,textvariable=self.seed,width=10)
        self.seed_entry.grid(row=1,column=2,padx=(16,4))
        self.random=ttk.Button(settings,text='生成种子',command=self.new_seed)
        self.random.grid(row=1,column=3,padx=4)
        self.base_seed=ttk.Button(settings,text='种子 51',command=lambda:self.seed.set('51'))
        self.base_seed.grid(row=1,column=4,padx=4)
        self.label(settings,'',10,MUTED,PANEL,var=self.detail).grid(row=2,column=0,columnspan=5,sticky='w',pady=(8,1))
        ttk.Checkbutton(settings,text='仅运行仿真，不显示 3D 窗口',variable=self.headless).grid(row=3,column=0,columnspan=5,sticky='w')

        cards=tk.Frame(section,bg=BG)
        cards.pack(fill='x',pady=(12,9))
        for col,(key,name,swatch) in enumerate(zip(('red','blue','yellow'),('红色方块','蓝色方块','黄色方块'),
                                                   ('#ff897f','#84baff','#ffdb79'))):
            cards.columnconfigure(col,weight=1,uniform='cubes')
            box=tk.Frame(cards,bg=PANEL,padx=11,pady=9)
            box.grid(row=0,column=col,sticky='ew',padx=(0 if col==0 else 4,0 if col==2 else 4))
            self.label(box,name,11,swatch,PANEL,bold=True).pack(anchor='w')
            self.colors[key]=tk.StringVar()
            self.label(box,'',10,FG,PANEL,var=self.colors[key]).pack(anchor='w',pady=(5,0))

        panel=tk.Frame(section,bg=PANEL,padx=13,pady=9)
        panel.pack(fill='x')
        self.label(panel,'当前阶段',9,MUTED,PANEL).pack(anchor='w')
        self.label(panel,'',14,FG,PANEL,bold=True,var=self.status).pack(anchor='w',pady=(2,1))
        self.label(panel,'',8,MUTED,PANEL,var=self.path,wrap=825).pack(anchor='w')
        actions=tk.Frame(section,bg=BG)
        actions.pack(fill='x',pady=(12,0))
        self.start_btn=tk.Button(actions,text='启动演示',command=self.start,bg=ACCENT,fg='#192232',
                                 activebackground='#ffd092',disabledforeground='#6f747b',
                                 relief='flat',font=(FONT,11,'bold'),padx=17,pady=7)
        self.start_btn.pack(side='left')
        self.stop_btn=ttk.Button(actions,text='停止演示',command=self.stop,state='disabled')
        self.stop_btn.pack(side='left',padx=8)
        ttk.Button(actions,text='打开基础操作面板',command=self.legacy).pack(side='right')
        ttk.Button(actions,text='打开运行记录',command=self.open_log).pack(side='right',padx=8)
        self.refresh_mode()
        root.protocol('WM_DELETE_WINDOW',self.close)
        self.tick()

    @staticmethod
    def label(parent,text,size,color,bg=BG,bold=False,var=None,wrap=0):
        return tk.Label(parent,text=text,textvariable=var,bg=bg,fg=color,
                        font=(FONT,size,'bold' if bold else 'normal'),justify='left',
                        wraplength=wrap,anchor='w')

    def new_seed(self):
        self.seed.set(str(secrets.randbelow(999999999)+1))

    def refresh_mode(self):
        detour=self.mode.get()==MODES[1]
        self.detail.set('固定种子 51 · 红块运输段设置一道可绕行的障碍。' if detour
                        else '红、蓝、黄顺序分类 · 种子 51 已通过完整演示。')
        self.random.configure(state='disabled' if detour else 'normal')
        self.seed_entry.configure(state='readonly' if detour else 'normal')
        if detour:
            self.seed.set('51')
        if self.proc is None or self.proc.poll() is not None:
            for color,var in self.colors.items():
                var.set('不参与本次演示' if detour and color!='red' else '待执行')

    def start(self):
        if self.proc is not None and self.proc.poll() is None:
            return
        mode='detour' if self.mode.get()==MODES[1] else 'sorting'
        try:
            seed=int(self.seed.get())
            if not 0<=seed<1000000000:
                raise ValueError()
        except ValueError:
            self.status.set('请输入 0 到 999999999 的整数种子')
            return
        if mode=='detour':
            seed=51
            self.seed.set('51')
        self.case=ROOT/'validation'/'demo_console'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.case.mkdir(parents=True)
        self.log=(self.case/'console.log').open('w')
        env={**os.environ,'ROS_DOMAIN_ID':str(162+secrets.randbelow(50)),'PYTHONUNBUFFERED':'1'}
        self.proc=subprocess.Popen([sys.executable,str(Path(__file__).with_name('demo_session.py')),
                                    str(self.case),mode,str(seed),'headless' if self.headless.get() else 'gui'],
                                   cwd=ROOT,env=env,stdout=self.log,stderr=subprocess.STDOUT,
                                   start_new_session=True)
        self.log.close()
        self.active_colors=['red'] if mode=='detour' else ['red','blue','yellow']
        for color,var in self.colors.items():
            var.set('待执行' if color in self.active_colors else '不参与本次演示')
        self.path.set(f'种子 {seed} · {self.case.name}')
        self.status.set('正在启动 Gazebo、控制器和 Nav2')
        self.start_btn.configure(state='disabled')
        self.stop_btn.configure(state='normal')
        for field in (self.choice,self.seed_entry,self.random,self.base_seed):
            field.configure(state='disabled')

    def tick(self):
        if self.proc is not None:
            report=self.case/'summary.json'
            if report.is_file():
                data=json.loads(report.read_text())
                self.status.set('验收通过 · 方块已落稳' if data['passed'] else
                                '已停止' if data.get('stopped') else '未通过 · 查看运行记录')
                for color in data.get('completed',[]):
                    self.colors[color].set('已验收' if data['passed'] else '任务已确认')
            elif (self.case/'task.log').is_file():
                with (self.case/'task.log').open('rb') as log:
                    log.seek(0,os.SEEK_END)
                    log.seek(max(0,log.tell()-65536))
                    text=log.read().decode(errors='replace')
                records=re.findall(r'^\[INFO\].*? M6 ([A-Z_]+)\s*$',text,re.MULTILINE)
                if records:
                    self.status.set(STAGES.get(records[-1],self.status.get()))
                    count=records.count('MARK_COMPLETE')
                    for index,color in enumerate(self.active_colors):
                        self.colors[color].set('已放置' if index<count else '执行中' if index==count else '待执行')
            if self.proc.poll() is not None:
                if not report.is_file():
                    self.status.set('演示进程已结束 · 查看运行记录')
                self.proc=None
                self.start_btn.configure(state='normal')
                self.choice.configure(state='readonly')
                self.base_seed.configure(state='normal')
                detour=self.mode.get()==MODES[1]
                self.random.configure(state='disabled' if detour else 'normal')
                self.seed_entry.configure(state='readonly' if detour else 'normal')
                self.stop_btn.configure(state='disabled')
        self.root.after(600,self.tick)

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            self.stop_btn.configure(state='disabled')
            self.status.set('正在停止演示')

    def close(self):
        self.stop()
        self.root.destroy()

    def legacy(self):
        subprocess.Popen([sys.executable,str(ROOT/'src/mobile_manipulator_gazebo/scripts/robot_panel.py'),
                          '--workspace',str(ROOT)],cwd=ROOT,start_new_session=True)

    def open_log(self):
        if self.case is not None:
            subprocess.Popen(['xdg-open',str(self.case)],cwd=ROOT,start_new_session=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=('sorting','detour'),default='sorting')
    parser.add_argument('--seed',type=int,default=51)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--start-after',type=float,metavar='SECONDS')
    args=parser.parse_args()
    root=tk.Tk()
    panel=Console(root,args.mode=='detour',args.seed,args.headless)
    if args.start_after is not None:
        root.after(max(0,int(args.start_after*1000)),panel.start)
    root.mainloop()


if __name__=='__main__':
    main()
