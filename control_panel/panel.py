"""Native offline console. Standard-library GUI; frozen Java controller and school plant."""
from pathlib import Path
import bisect,csv,gzip,hashlib,json,math,os,queue,random,shutil,subprocess,threading,time,uuid
import tkinter as tk
from tkinter import ttk,messagebox
if os.name=='nt':
 try:
  import ctypes
  ctypes.windll.shcore.SetProcessDpiAwareness(1)
 except (AttributeError,OSError):pass
ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'data/control_panel_runs'
JAVA=Path('E:/Processing/app/resources/jdk/bin/java.exe')
CP=';'.join(map(str,[ROOT/'build/control_panel',ROOT/'build/DigitalTwinParkingPrecision',Path('E:/Processing/app/resources/core/library/core-4.5.7.jar'),Path('E:/Processing/app/resources/modes/java/libraries/net/library/net.jar')]))
SHAPES={'矩形 / 正方形':'rectangle','圆形':'circle','椭圆形':'ellipse','自定义多边形':'polygon'}
MODES={'越线继续（外侧始终有支撑）':'continue','任一轮心越过检测线就停':'line','越过外侧余量后停':'support'}
STATES={'SEARCH':'广域搜索','LOCAL_SCAN':'局部扫描','TRACE_EDGE':'局部探测','ALIGN_ENTRY':'对准入口','GO_ENTRY':'驶向入口','VERIFY_CODE':'确认方向条纹','SCAN_APPROACH':'接近扫描位置','SCAN_ALIGN':'扫描对准','SCAN_CROSS':'横向确认','GO_TARGET':'驶向停车位','ALIGN_TARGET':'调整停车方向','SETTLE':'等待停稳','SUCCESS':'停车完成','FAILED':'停止 / 失败'}
RESULTS={'parked':'停车完成','user_stopped':'用户结束','wheel_support_lost':'轮心越过设定停止边界','storage_reserve_stop':'磁盘剩余空间达到保留值','run_storage_quota_stop':'达到本次存储上限','window_closed':'窗口关闭','local_encoder_stall_10s':'局部停车停滞','false_completion':'未通过独立停车验收'}
ROBOT_BODY_RADIUS_MM=50
WHEEL_HALF_TRACK_MM=45
WHEEL_SUPPORT_RADIUS_MM=5
START_CLEARANCE_MM=2

def segment_distance(x,y,a,b):
 dx,dy=b[0]-a[0],b[1]-a[1];t=max(0,min(1,((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy)))
 return math.hypot(x-a[0]-t*dx,y-a[1]-t*dy)
def clearance(c,x,y):
 if c['shape']=='rectangle':return min(c['width_mm']/2-abs(x),c['height_mm']/2-abs(y))
 if c['shape']=='circle':return c['width_mm']/2-math.hypot(x,y)
 inside=False;p=c['polygon'];d=1e100
 for a,b in zip(p,p[1:]+p[:1]):
  if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:inside=not inside
  d=min(d,segment_distance(x,y,a,b))
 return d if inside else -d
def geometry(shape,width,height,vertices=''):
 w,h=float(width)*1000,float(height)*1000
 if not all(math.isfinite(v) and 500<=v<=100000 for v in [w,h]):raise ValueError('尺寸需在0.5～100米之间。超过4米属于泛化测试。')
 if shape=='circle':h=w
 if shape=='polygon':
  p=[[float(v)*1000 for v in pair.strip().split(',')] for pair in vertices.replace('，',',').replace('；',';').split(';') if pair.strip()]
  if not 3<=len(p)<=64 or any(len(v)!=2 or not all(math.isfinite(t) for t in v) for v in p):raise ValueError('多边形需3～64个顶点：x,y; x,y（单位米）。')
  if p[0]==p[-1]:p.pop()
  for a,b in zip(p,p[1:]+p[:1]):
   if math.dist(a,b)<1:raise ValueError('相邻顶点不能重复。')
  def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
  def intersects(a,b,c,d):
   s=[cross(a,b,c),cross(a,b,d),cross(c,d,a),cross(c,d,b)]
   return s[0]*s[1]<=0 and s[2]*s[3]<=0 and max(min(a[0],b[0]),min(c[0],d[0]))<=min(max(a[0],b[0]),max(c[0],d[0])) and max(min(a[1],b[1]),min(c[1],d[1]))<=min(max(a[1],b[1]),max(c[1],d[1]))
  n=len(p)
  for i in range(n):
   for j in range(i+1,n):
    if j==i+1 or (i==0 and j==n-1):continue
    if intersects(p[i],p[(i+1)%n],p[j],p[(j+1)%n]):raise ValueError('多边形边不能自相交。')
  lo=[min(v[k] for v in p) for k in range(2)];hi=[max(v[k] for v in p) for k in range(2)]
  w,h=hi[0]-lo[0],hi[1]-lo[1]
  if min(w,h)<500 or max(w,h)>100000:raise ValueError('多边形包围尺寸需在0.5～100米之间。')
  p=[[v[k]-(lo[k]+hi[k])/2 for k in range(2)] for v in p]
 elif shape in ['ellipse','circle']:p=[[w/2*math.cos(i*2*math.pi/256),h/2*math.sin(i*2*math.pi/256)] for i in range(256)]
 else:p=[[-w/2,-h/2],[w/2,-h/2],[w/2,h/2],[-w/2,h/2]]
 return dict(shape=shape,width_mm=w,height_mm=h,polygon=p)
def marker_inside(c,x,y,a):
 cs,sn=math.cos(a),math.sin(a)
 # Sample the full A4 perimeter, including concave-polygon intersections.
 for k in range(33):
  u=k/32
  for px,py in [(-105+210*u,-148.5),(-105+210*u,148.5),(-105,-148.5+297*u),(105,-148.5+297*u)]:
   if clearance(c,x+cs*px-sn*py,y+sn*px+cs*py)<2:return False
 return True
def random_target(c,rng):
 for _ in range(10000):
  x=rng.uniform(-c['width_mm']/2,c['width_mm']/2);y=rng.uniform(-c['height_mm']/2,c['height_mm']/2);a=rng.uniform(-math.pi,math.pi)
  if marker_inside(c,x,y,a):return x,y,a
 raise ValueError('该形状放不下完整A4标记，请扩大场地或调整多边形。')
def random_robot(c,rng):
 for _ in range(10000):
  x=rng.uniform(-c['width_mm']/2,c['width_mm']/2);y=rng.uniform(-c['height_mm']/2,c['height_mm']/2)
  if clearance(c,x,y)>=60 and math.hypot(x-c['target_x'],y-c['target_y'])>230:return x,y,rng.uniform(-math.pi,math.pi)
 raise ValueError('剩余空间放不下小车，请扩大地图。')
def make_scenario(shape='rectangle',width=3,height=3,vertices='',seed=1):
 c=geometry(shape,width,height,vertices);r=random.Random(seed);c['target_x'],c['target_y'],c['target_yaw']=random_target(c,r);c['robot_x'],c['robot_y'],c['robot_yaw']=random_robot(c,r)
 c.update(encoder_seed=r.randrange(2**53),policy_seed=r.randrange(2**53),seed=seed,stop_mode='support',margin_mm=100,reserve_gb=2,quota_gb=5)
 c['memory_side']=max(192,4*math.ceil((2*math.hypot(c['width_mm'],c['height_mm'])+1000)/200))
 return c
def validate_start(c):
 """Check the complete simulated footprint against the map, ignoring outside support."""
 try:x,y,a=(float(c[k]) for k in ['robot_x','robot_y','robot_yaw'])
 except (KeyError,TypeError,ValueError):raise ValueError('小车起始位姿无效，请重新放置小车。') from None
 if not all(math.isfinite(v) for v in [x,y,a]):raise ValueError('小车起始位姿必须是有限数值。')
 body=clearance(c,x,y)-ROBOT_BODY_RADIUS_MM
 wheels=[clearance(c,x-s*WHEEL_HALF_TRACK_MM*math.sin(a),y+s*WHEEL_HALF_TRACK_MM*math.cos(a))-WHEEL_SUPPORT_RADIUS_MM for s in [-1,1]]
 if not all(math.isfinite(v) and v>=START_CLEARANCE_MM for v in [body,*wheels]):
  raise ValueError('起点不在桌面安全范围内：整个车身和两轮必须在边界内，并保留至少2毫米间隙。请点击“只随机小车”重新放置；允许越线或线外余量不能放宽起点限制。')
 return {'body_clearance_mm':body,'wheel_clearances_mm':wheels,'required_clearance_mm':START_CLEARANCE_MM}

def freeze_run(c,base=RUNS):
 start_validation=validate_start(c)
 cfg=json.loads((ROOT/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'));source=ROOT/cfg['policy'];raw=source.read_bytes()
 if hashlib.sha256(raw).hexdigest()!=cfg['sha256']:raise RuntimeError('模型校验失败。')
 base.mkdir(parents=True,exist_ok=True)
 if shutil.disk_usage(base).free<c['reserve_gb']*1e9+10_000_000:raise ValueError('磁盘可用空间不足以保留指定余量。')
 directory=base/(time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]);directory.mkdir();(directory/'policy.bin').write_bytes(raw)
 c=dict(c,policy=str(directory/'policy.bin'),model=cfg['model'],policy_sha256=cfg['sha256'],created=time.strftime('%Y-%m-%d %H:%M:%S'),format_version=2,start_validation=start_validation,
  speed_settings={'search_command_mm_s':200,'simulated_wheel_limit_mm_s':250,'physical_top_speed_mm_s':None},
  controller_sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'control_panel/java').glob('*.java')},
  notes='10ms archived; high precision plant; no total search deadline; integer sensors/encoders only; non-rectangular long straight-wall inference disabled; wheel-centre crossing is a 2D support proxy, not 3D falling physics.')
 (directory/'config.json').write_text(json.dumps(c,ensure_ascii=False,indent=2),encoding='utf-8');return directory,c
def start_worker(directory):
 err=(directory/'worker.log').open('w',encoding='utf-8')
 p=subprocess.Popen([str(JAVA),'-Xmx1g','-cp',CP,'PanelWorker',str(directory/'config.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW)
 return p,err
def decode_frame(row):
 out=[]
 for i,v in enumerate(row):
  if i in [4,5] or v=='unknown':out.append(v)
  else:out.append(float(v))
 return out
def load_replay(directory,limit=30000):
 c=json.loads((directory/'config.json').read_text(encoding='utf-8'));data=[];count=0;stride=1;last=None;partial=False
 for part in sorted(directory.glob('part-*.csv.gz')):
  try:
   with gzip.open(part,'rt',encoding='utf-8',newline='') as f:
    reader=csv.reader(f);next(reader,None)
    for row in reader:
     last=decode_frame(row)
     if count%stride==0:data.append(last)
     count+=1
     if len(data)>limit:data=data[::2];stride*=2
  except (EOFError,OSError,ValueError):partial=True
 if last is not None and (not data or data[-1][0]!=last[0]):data.append(last)
 if not data:raise ValueError('这条记录尚无可回放数据。')
 return c,data,count,partial

class Panel(tk.Tk):
 def __init__(self):
  super().__init__();self.title('重复停车 · 当前模型控制台（离线仿真）');self.geometry(f'{min(1320,self.winfo_screenwidth()-60)}x{min(870,self.winfo_screenheight()-90)}+20+20');self.minsize(1050,760);self.configure(bg='#edf2f7')
  style=ttk.Style(self);style.theme_use('clam');style.configure('TLabel',font=('Microsoft YaHei UI',10));style.configure('TButton',font=('Microsoft YaHei UI',10),padding=6);style.configure('TCombobox',padding=3)
  self.events=queue.Queue(maxsize=12);self.stop_event=threading.Event();self.pause_event=threading.Event();self.active=False;self.replaying=False;self.scenario=None;self.current_frame=None;self.visible=[];self.replay=[];self.times=[];self.directory=None;self.records=[];self.speed_value=50;self.packet=None
  self.shape=tk.StringVar(value=list(SHAPES)[0]);self.width=tk.StringVar(value='3');self.height=tk.StringVar(value='3');self.preset=tk.StringVar(value='3');self.mode=tk.StringVar(value=list(MODES)[2]);self.margin=tk.StringVar(value='100');self.quota=tk.StringVar(value='5');self.reserve=tk.StringVar(value='2');self.speed=tk.StringVar(value='50');self.random_target_var=tk.BooleanVar(value=True);self.random_robot_var=tk.BooleanVar(value=True);self.show_odom=tk.BooleanVar(value=False)
  self.status=tk.StringVar(value='准备就绪');self.stats=tk.StringVar();self.space=tk.StringVar();self.model_label=tk.StringVar(value='固定模型：edge00_seed2；后续：提前定位＋方向条纹确认＋停车\n搜索直行设定 200 mm/s｜仿真单轮上限 250 mm/s｜实体极速未实测')
  self.build();self.new_preview();self.refresh();self.after(50,self.poll);self.protocol('WM_DELETE_WINDOW',self.close)
 def build(self):
  top=ttk.Frame(self,padding=12);top.pack(fill='x');ttk.Label(top,text='重复停车实验台',font=('Microsoft YaHei UI',18,'bold')).pack(side='left');ttk.Label(top,textvariable=self.model_label).pack(side='right')
  body=ttk.Frame(self,padding=(12,0,12,12));body.pack(fill='both',expand=True);left=ttk.Frame(body,width=360);left.pack(side='left',fill='y',padx=(0,12));left.pack_propagate(False);right=ttk.Frame(body);right.pack(side='left',fill='both',expand=True)
  holder=ttk.Frame(left);holder.pack(fill='both',expand=True);options_canvas=tk.Canvas(holder,highlightthickness=0,width=330,height=470,bg='#dcdad5');bar=ttk.Scrollbar(holder,orient='vertical',command=options_canvas.yview);bar.pack(side='right',fill='y');options_canvas.pack(side='left',fill='both',expand=True);options_canvas.configure(yscrollcommand=bar.set);options=ttk.Frame(options_canvas);slot=options_canvas.create_window(0,0,anchor='nw',window=options);options.bind('<Configure>',lambda e:options_canvas.configure(scrollregion=options_canvas.bbox('all')));options_canvas.bind('<Configure>',lambda e:options_canvas.itemconfigure(slot,width=e.width))
  def scroll_options(event):
   if options_canvas.winfo_rootx()<=event.x_root<=options_canvas.winfo_rootx()+options_canvas.winfo_width() and options_canvas.winfo_rooty()<=event.y_root<=options_canvas.winfo_rooty()+options_canvas.winfo_height():options_canvas.yview_scroll(-int(event.delta/120),'units')
  self.bind_all('<MouseWheel>',scroll_options)
  arena=ttk.LabelFrame(options,text='① 地图 / 场地',padding=9);arena.pack(fill='x');self.shape_box=ttk.Combobox(arena,textvariable=self.shape,values=list(SHAPES),state='readonly');self.shape_box.pack(fill='x');self.shape_box.bind('<<ComboboxSelected>>',lambda e:self.new_preview())
  r=ttk.Frame(arena);r.pack(fill='x',pady=5);ttk.Label(r,text='快捷边长 / 直径（米）').pack(side='left');p=ttk.Combobox(r,textvariable=self.preset,values=['1','2','3','4','5'],width=5,state='readonly');p.pack(side='right');p.bind('<<ComboboxSelected>>',self.set_preset)
  r=ttk.Frame(arena);r.pack(fill='x');ttk.Label(r,text='宽 / 直径').pack(side='left');ttk.Entry(r,textvariable=self.width,width=6).pack(side='left');ttk.Label(r,text=' 高（米）').pack(side='left');ttk.Entry(r,textvariable=self.height,width=6).pack(side='left');ttk.Button(r,text='应用',command=self.new_preview).pack(side='right')
  ttk.Label(arena,text='多边形顶点（米）：x,y; x,y; …').pack(anchor='w',pady=(6,0));self.vertices=tk.Text(arena,height=2,font=('Consolas',10));self.vertices.pack(fill='x');self.vertices.insert('1.0','0,0; 4,0; 4,3; 2,2; 0,3')
  ttk.Label(arena,text='圆形按二维桌面模拟；椭圆用256条短边近似。\n超过4米及非矩形属于泛化测试。',foreground='#596c80',font=('Microsoft YaHei UI',9)).pack(anchor='w',pady=4)
  boundary=ttk.LabelFrame(options,text='② 越线与支撑',padding=9);boundary.pack(fill='x',pady=7);ttk.Combobox(boundary,textvariable=self.mode,values=list(MODES),state='readonly').pack(fill='x');r=ttk.Frame(boundary);r.pack(fill='x',pady=5);ttk.Label(r,text='线外支撑余量（毫米）').pack(side='left');ttk.Entry(r,textvariable=self.margin,width=8).pack(side='right');ttk.Label(boundary,text='三个选项都保留传感器触边检测与避让。\n停止按任一轮心判断，不是三维跌落模型。',font=('Microsoft YaHei UI',9),foreground='#596c80').pack(anchor='w')
  poses=ttk.LabelFrame(options,text='③ 随机位置与方向',padding=9);poses.pack(fill='x');ttk.Checkbutton(poses,text='每次开始：随机停车目标',variable=self.random_target_var).pack(anchor='w');ttk.Checkbutton(poses,text='每次开始：随机小车位姿',variable=self.random_robot_var).pack(anchor='w');r=ttk.Frame(poses);r.pack(fill='x');ttk.Button(r,text='只随机目标',command=lambda:self.randomize('target')).pack(side='left',expand=True,fill='x');ttk.Button(r,text='只随机小车',command=lambda:self.randomize('robot')).pack(side='left',expand=True,fill='x')
  store=ttk.LabelFrame(options,text='④ 自动保存',padding=9);store.pack(fill='x',pady=7);r=ttk.Frame(store);r.pack(fill='x');ttk.Label(r,text='单次上限GB').pack(side='left');ttk.Entry(r,textvariable=self.quota,width=5).pack(side='left');ttk.Label(r,text=' 保留空闲GB').pack(side='left');ttk.Entry(r,textvariable=self.reserve,width=5).pack(side='left');ttk.Label(store,textvariable=self.space,font=('Microsoft YaHei UI',9)).pack(anchor='w',pady=4);ttk.Button(store,text='打开记录文件夹',command=self.open_folder).pack(fill='x')
  ttk.Label(left,text='已保存的运行（双击回放）').pack(anchor='w',pady=(6,0));self.history=tk.Listbox(left,height=5,font=('Microsoft YaHei UI',9));self.history.pack(fill='x');self.history.bind('<Double-Button-1>',lambda e:self.open_replay());r=ttk.Frame(left);r.pack(fill='x');ttk.Button(r,text='刷新',command=self.refresh).pack(side='left');ttk.Button(r,text='载入回放',command=self.open_replay).pack(side='left',fill='x',expand=True)
  controls=ttk.Frame(right);controls.pack(fill='x',pady=(0,6));self.start_button=ttk.Button(controls,text='开始新运行',command=self.start);self.start_button.pack(side='left');self.pause_button=ttk.Button(controls,text='暂停',command=self.pause);self.pause_button.pack(side='left');ttk.Button(controls,text='结束并保存',command=self.stop).pack(side='left');ttk.Label(controls,text=' 仿真倍速 ×').pack(side='left');speed=ttk.Combobox(controls,textvariable=self.speed,values=['1','10','50','200','500'],width=5,state='readonly');speed.pack(side='left');speed.bind('<<ComboboxSelected>>',lambda e:setattr(self,'speed_value',int(self.speed.get())));ttk.Checkbutton(controls,text='显示估计位姿',variable=self.show_odom,command=self.draw).pack(side='right')
  self.canvas=tk.Canvas(right,bg='#fbfdff',highlightthickness=1,highlightbackground='#ced9e3');self.canvas.pack(fill='both',expand=True);self.canvas.bind('<Configure>',lambda e:self.draw());ttk.Label(right,text='青色：真实轨迹   橙色：估计位置   紫色：起点   灰色带：线外支撑余量',font=('Microsoft YaHei UI',9)).pack(anchor='w')
  r=ttk.Frame(right);r.pack(fill='x');ttk.Button(r,text='播放 / 暂停回放',command=self.toggle_replay).pack(side='left');self.slider=ttk.Scale(r,from_=0,to=1,command=self.seek);self.slider.pack(side='left',fill='x',expand=True,padx=6)
  ttk.Label(right,textvariable=self.status,font=('Microsoft YaHei UI',11,'bold'),foreground='#18567c').pack(anchor='w',pady=(5,0));ttk.Label(right,textvariable=self.stats).pack(anchor='w')
 def busy_check(self):
  if self.active:messagebox.showinfo('运行中','请先结束并保存当前运行，再修改地图或位姿。');return True
  return False
 def set_preset(self,event=None):self.width.set(self.preset.get());self.height.set(self.preset.get());self.new_preview()
 def settings(self):
  c=geometry(SHAPES[self.shape.get()],self.width.get(),self.height.get(),self.vertices.get('1.0','end'));c.update(stop_mode=MODES[self.mode.get()],margin_mm=float(self.margin.get()),reserve_gb=float(self.reserve.get()),quota_gb=float(self.quota.get()))
  if not 0<=c['margin_mm']<=10000 or not .1<=c['quota_gb']<=1000 or not .1<=c['reserve_gb']<=1000:raise ValueError('余量须在0～10000mm，存储上限和空闲余量须在0.1～1000GB。')
  c['memory_side']=max(192,4*math.ceil((2*math.hypot(c['width_mm'],c['height_mm'])+1000)/200));return c
 def new_preview(self):
  if self.busy_check():return
  try:
   self.scenario=self.settings();self.width.set(f"{self.scenario['width_mm']/1000:g}");self.height.set(f"{self.scenario['height_mm']/1000:g}");r=random.Random();self.scenario['target_x'],self.scenario['target_y'],self.scenario['target_yaw']=random_target(self.scenario,r);self.scenario['robot_x'],self.scenario['robot_y'],self.scenario['robot_yaw']=random_robot(self.scenario,r);self.reset_view();self.status.set('预览：点击开始后按勾选项重新随机位置。')
  except Exception as e:messagebox.showerror('配置错误',str(e))
 def randomize(self,what):
  if self.busy_check():return
  try:
   if what=='target':
    # Do not move the robot when only the target is requested.
    for _ in range(10000):
     x,y,a=random_target(self.scenario,random)
     if math.hypot(x-self.scenario['robot_x'],y-self.scenario['robot_y'])>230:break
    else:raise ValueError('没有足够空间独立重放目标。')
   else:x,y,a=random_robot(self.scenario,random)
   self.scenario[what+'_x'],self.scenario[what+'_y'],self.scenario[what+'_yaw']=x,y,a;self.reset_view()
  except Exception as e:messagebox.showerror('无法随机放置',str(e))
 def reset_view(self):self.visible=[];self.current_frame=None;self.replay=[];self.times=[];self.replaying=False;self.slider.configure(to=1);self.slider.set(0);self.draw()
 def start(self):
  if self.active:return
  try:
   new=self.settings();old=self.scenario
   if any(new[k]!=old[k] for k in ['shape','width_mm','height_mm','polygon']):raise ValueError('地图参数已改变，请先点“应用”。')
   c=dict(old,**new);r=random.Random();c['encoder_seed']=r.randrange(2**53);c['policy_seed']=r.randrange(2**53)
   if self.random_target_var.get():
    for _ in range(10000):
     tx,ty,ta=random_target(c,r)
     if self.random_robot_var.get() or math.hypot(tx-c['robot_x'],ty-c['robot_y'])>230:break
    else:raise ValueError('新目标与固定小车重叠，请重新放置。')
    c.update(target_x=tx,target_y=ty,target_yaw=ta)
   if self.random_robot_var.get():c['robot_x'],c['robot_y'],c['robot_yaw']=random_robot(c,r)
   self.directory,self.scenario=freeze_run(c);self.reset_view();self.active=True;self.stop_event.clear();self.pause_event.clear();self.pause_button.configure(text='暂停');self.start_button.configure(state='disabled');self.status.set('启动当前网络与后续停车流程…');self.speed_value=int(self.speed.get());threading.Thread(target=self.run_loop,args=(self.directory,),daemon=True).start()
  except Exception as e:messagebox.showerror('无法开始',str(e))
 def run_loop(self,directory):
  p=None;err=None
  try:
   p,err=start_worker(directory);ready=json.loads(p.stdout.readline());assert ready['ready']
   while True:
    if self.stop_event.is_set():command='STOP'
    elif self.pause_event.is_set():time.sleep(.05);continue
    else:command=str(max(1,min(1000,int(self.speed_value*5))))
    began=time.monotonic();p.stdin.write(command+'\n');p.stdin.flush();line=p.stdout.readline()
    if not line:raise RuntimeError('仿真进程停止，请查看该记录的worker.log。')
    result=json.loads(line);self.events.put(('packet',result))
    if result['done']:break
    delay=int(command)*.01/max(1,self.speed_value)-(time.monotonic()-began)
    if delay>0:self.stop_event.wait(min(delay,.1))
   p.wait(timeout=10);self.events.put(('finished',directory))
  except Exception as e:self.events.put(('error',str(e)))
  finally:
   if p and p.poll() is None:p.terminate();p.wait()
   if err:err.close()
 def pause(self):
  if not self.active:return
  if self.pause_event.is_set():self.pause_event.clear();self.pause_button.configure(text='暂停')
  else:self.pause_event.set();self.pause_button.configure(text='继续');self.status.set('已暂停，已有轨迹保留在当前记录中。')
 def stop(self):
  if self.active:self.stop_event.set();self.pause_event.clear();self.status.set('正在结束并保存…')
 def poll(self):
  changed=False
  for _ in range(12):
   try:kind,data=self.events.get_nowait()
   except queue.Empty:break
   if kind=='packet':
    self.packet=data;self.visible.extend(data['frames']);self.visible=self.visible[-12000:]
    if data['frames']:self.current_frame=data['frames'][-1]
    self.status.set(('已暂停：' if self.pause_event.is_set() else '运行中：')+STATES.get(self.current_frame[4],self.current_frame[4]) if not data['done'] and self.current_frame else '已保存：'+RESULTS.get(data['status'],data['status']));changed=True
   elif kind in ['finished','error']:
    self.active=False;self.start_button.configure(state='normal');self.pause_button.configure(text='暂停');self.refresh()
    if kind=='error':self.status.set('运行异常，已保留已有文件：'+data)
   elif kind=='replay':
    self.scenario,self.replay,n,partial=data;self.times=[f[0] for f in self.replay];self.shape.set(next(k for k,v in SHAPES.items() if v==self.scenario['shape']));self.width.set(f"{self.scenario['width_mm']/1000:g}");self.height.set(f"{self.scenario['height_mm']/1000:g}");self.mode.set(next(k for k,v in MODES.items() if v==self.scenario['stop_mode']));self.margin.set(str(self.scenario['margin_mm']));self.quota.set(str(self.scenario['quota_gb']));self.reserve.set(str(self.scenario['reserve_gb']));
    if self.scenario['shape']=='polygon':self.vertices.delete('1.0','end');self.vertices.insert('1.0','; '.join(f'{x/1000:g},{y/1000:g}' for x,y in self.scenario['polygon']))
    self.slider.configure(to=self.times[-1]/1000);self.slider.set(self.times[-1]/1000);self.status.set(f'回放：原始{n:,}帧，显示{len(self.replay):,}帧'+('（末块不完整，已恢复可读部分）' if partial else ''));changed=True
  if self.replaying and self.replay:
   v=self.slider.get()+.05*self.speed_value;self.slider.set(min(v,self.times[-1]/1000));self.replaying=v<self.times[-1]/1000;changed=True
  if changed:self.draw()
  self.after(50,self.poll)
 def refresh(self):
  RUNS.mkdir(parents=True,exist_ok=True);self.space.set(f'E盘可用 {shutil.disk_usage(RUNS).free/1e9:.1f} GB · 10ms轨迹分块压缩')
  self.records=sorted((p.parent for p in RUNS.glob('*/config.json')),reverse=True)[:500];self.history.delete(0,'end')
  for p in self.records:
   try:
    c=json.loads((p/'config.json').read_text(encoding='utf-8'));s=json.loads((p/'summary.json').read_text(encoding='utf-8')) if (p/'summary.json').exists() else {'status':'运行中 / 未正常关闭'};label=f"{p.name[:15]}  {c['width_mm']/1000:g}×{c['height_mm']/1000:g}m  {RESULTS.get(s['status'],s['status'])}";self.history.insert('end',label)
   except Exception:self.history.insert('end',p.name+'（记录待检查）')
 def open_folder(self):RUNS.mkdir(parents=True,exist_ok=True);os.startfile(str(RUNS))
 def open_replay(self):
  if self.busy_check():return
  selected=self.history.curselection()
  if not selected:return
  directory=self.records[selected[0]];self.directory=directory;self.replaying=False;self.status.set('正在读取压缩轨迹…')
  def read():
   try:self.events.put(('replay',load_replay(directory)))
   except Exception as e:self.events.put(('error',str(e)))
  threading.Thread(target=read,daemon=True).start()
 def seek(self,value):
  if not self.replay:return
  i=max(0,bisect.bisect_right(self.times,float(value)*1000)-1);self.current_frame=self.replay[i];self.visible=self.replay[:i+1];self.draw()
 def toggle_replay(self):
  if not self.replay:return
  if self.slider.get()>=self.times[-1]/1000:self.slider.set(0)
  self.replaying=not self.replaying
 def draw(self):
  if not self.scenario:return
  cv=self.canvas;cv.delete('all');w,h=cv.winfo_width(),cv.winfo_height()
  if w<10 or h<10:return
  c=self.scenario;extra=max(160,c.get('margin_mm',0));scale=min((w-70)/(c['width_mm']+extra*2),(h-60)/(c['height_mm']+extra*2));X=lambda x:w/2+x*scale;Y=lambda y:h/2-y*scale
  points=[v for x,y in c['polygon'] for v in (X(x),Y(y))];margin=c.get('margin_mm',0) if c.get('stop_mode')=='support' else 0
  if margin:cv.create_polygon(points,fill='#e4e9ed',outline='#e4e9ed',width=2*margin*scale,joinstyle=tk.ROUND)
  cv.create_polygon(points,fill='white',outline='#697f90',width=2)
  for meter in range(-math.ceil(c['width_mm']/2000),math.ceil(c['width_mm']/2000)+1):cv.create_text(X(meter*1000),h-12,text=str(meter)+'m',fill='#78909e',font=('Segoe UI',8))
  tx,ty,ta=c['target_x'],c['target_y'],c['target_yaw'];co,si=math.cos(ta),math.sin(ta)
  def rect(a,b,d,e,color):
   ps=[]
   for x,y in [(a,b),(d,b),(d,e),(a,e)]:ps.extend((X(tx+co*x-si*y),Y(ty+si*x+co*y)))
   cv.create_polygon(ps,fill=color,outline='')
  rect(-105,-148.5,105,148.5,'#e1e7eb')
  for q in [(-76,-91,-70,91),(70,-91,76,91),(-76,-91,76,-85),(-76,85,76,91),(-8,-91,8,65),(-50,-54,50,-46),(-50,-23,50,-7),(-50,41,50,49)]:rect(*q,'#152a39')
  cv.create_text(X(tx),Y(ty)-max(18,160*scale),text='停车框',fill='#273e50',font=('Microsoft YaHei UI',9))
  trail=self.visible[::max(1,len(self.visible)//6000)]
  if len(trail)>1:cv.create_line(*[v for f in trail for v in (X(f[1]),Y(f[2]))],fill='#168594',width=1.2)
  sx,sy=X(c['robot_x']),Y(c['robot_y']);cv.create_oval(sx-4,sy-4,sx+4,sy+4,fill='#9b4186',outline='')
  f=self.current_frame;x,y,a=(f[1],f[2],f[3]) if f else (c['robot_x'],c['robot_y'],c['robot_yaw']);radius=max(3,50*scale);cv.create_oval(X(x)-radius,Y(y)-radius,X(x)+radius,Y(y)+radius,fill='#79c5d1',outline='#16667c',width=2);cv.create_line(X(x),Y(y),X(x+80*math.cos(a)),Y(y+80*math.sin(a)),fill='#154f67',width=2,arrow=tk.LAST)
  for sign in [-1,1]:
   wx=x-sign*45*math.sin(a);wy=y+sign*45*math.cos(a);r=max(2,5*scale);cv.create_oval(X(wx)-r,Y(wy)-r,X(wx)+r,Y(wy)+r,fill='#193747',outline='')
  if f and self.show_odom.get() and isinstance(f[18],(int,float)):
   ex,ey=f[18],f[19];cv.create_oval(X(ex)-radius,Y(ey)-radius,X(ex)+radius,Y(ey)+radius,outline='#d67b36',width=2);cv.create_line(X(x),Y(y),X(ex),Y(ey),fill='#d67b36',dash=(3,3))
  if f:
   velocity=(f'仿真当前车速 {f[21]:.1f} mm/s    左轮 {f[22]:.1f} / 右轮 {f[23]:.1f} mm/s（负值为倒退）' if len(f)>=24 else '旧记录未采集瞬时车速')
   self.stats.set(f"时间 {f[0]/1000:.2f}s    触边 {int(f[6])}次    最小轮心余量 {f[7]:.2f}mm\n{velocity}\n{STATES.get(f[4],f[4])}    倍速只改变仿真/播放快慢；完整10ms轨迹已保存。")
  else:self.stats.set(f"场地 {c['width_mm']/1000:g} × {c['height_mm']/1000:g}m    网络未知初始位姿与目标坐标\n非矩形禁用长期直边推断，保留访问记忆与局部避让。")
 def close(self):
  if self.active:
   if not messagebox.askyesno('保存后关闭','结束当前运行、保存轨迹并关闭窗口？'):return
   self.stop();self.after(100,self.wait_close)
  else:self.destroy()
 def wait_close(self):
  if self.active:self.after(100,self.wait_close)
  else:self.destroy()

if __name__=='__main__':
 try:Panel().mainloop()
 except Exception:
  import traceback
  (ROOT/'control_panel/startup_error.log').write_text(traceback.format_exc(),encoding='utf-8');raise
