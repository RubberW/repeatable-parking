"""
文件职责：检查固定权重、方向约束、当前64场景参考、越线故障回归及真实启动。
输入：当前源码、模型和保留场景；输出：experiments/results中的验收记录。
调用关系：用户手动运行；与正常仿真分开，不训练或修改模型。
"""
from pathlib import Path
import argparse,csv,hashlib,json,subprocess,uuid,gzip
R=Path(__file__).resolve().parents[1]
OUT=R/'experiments/results'/('layout_'+uuid.uuid4().hex[:8])
JAVA='E:/Processing/app/resources/jdk/bin/java.exe'
CP=';'.join([str(R/'.cache/classes'),'E:/Processing/app/resources/core/library/core-4.5.7.jar'])
def settings(c,motion_model):
    """当前默认方案的全部开关，与 SimulationRunner.configureProject 读取同一份配置。"""
    b=c['boundary_motion'];g=c.get('search_gate',{})
    p=[f"-Dparking.boundaryMotion={b.get('mode','learned')}",f'-Dparking.motionPolicy={motion_model}',f"-Dparking.parallelDegrees={b['parallel_threshold_degrees']}",
       f"-Dparking.gate={g.get('mode','hull')}",f"-Dparking.redundancyWeight={c.get('redundancy_weight',0)}"]
    if 'safe_mm' in b:p.append(f"-Dparking.vectorSafeMm={b['safe_mm']}")
    l=c.get('local_parking',{})
    p.append(f"-Dparking.adaptiveEntry={str(l.get('adaptive_entry',False)).lower()}")
    p.append(f"-Dparking.directApproach={str(l.get('direct_approach',False)).lower()}")
    for key,prop in (('inward_deg','gateInwardDeg'),('memory_mm','gateMemoryMm'),('depart_mm','gateDepartMm')):
        if key in g:p.append(f'-Dparking.{prop}={g[key]}')
    return p
def run(*args):
    p=subprocess.run([JAVA,'-Xmx512m',*map(str,args)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,creationflags=subprocess.CREATE_NO_WINDOW)
    print(p.stdout,flush=True);p.check_returncode()
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--startup-only',action='store_true',help='Only verify the actual double-click launcher and saved frames')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=False)
    c=json.loads((R/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
    main_model=R/c['policy'];motion_model=R/c['boundary_motion']['policy']
    for p,digest in [(main_model,c['sha256']),(motion_model,c['boundary_motion']['sha256'])]:assert hashlib.sha256(p.read_bytes()).hexdigest()==digest,p
    if not args.startup_only:
        run('-cp',CP,'SimulationChecks',main_model,motion_model,R/'experiments/data/motion_parity.csv')
        run(*settings(c,motion_model),'-cp',CP,'ParkingEvaluation',R/'experiments/data/fresh_holdout_scenes.csv',main_model,OUT/'full_64.csv')
        original=list(csv.DictReader((R/c['boundary_motion']['regression_reference']).open()))
        current=list(csv.DictReader((OUT/'full_64.csv').open()))
        assert len(original)==len(current)==64
        for a,b in zip(original,current):assert a==b,dict(before=a,after=b)
        assert all(x['parked']=='true' and x['false_completion']=='false' and float(x['search_critical_s'])==0 for x in current)
        run(*settings(c,motion_model),'-cp',CP,'ParkingEvaluation',R/c['boundary_motion']['reported_scenes'],main_model,OUT/'reported_2.csv')
        reported=list(csv.DictReader((OUT/'reported_2.csv').open()))
        assert len(reported)==2
        assert all(x['parked']=='true' and x['false_completion']=='false' and float(x['search_critical_s'])==0 and float(x['search_min_wheel_mm'])>-40 for x in reported)
    # 走真实双击链：cmd → Windows PowerShell → 编译 → Processing窗口。
    # 检查参数让窗口渲染三帧后退出；正常双击不传该参数。
    image=R/'.cache/processing-window.png'
    previous_image_time=image.stat().st_mtime_ns if image.exists() else 0
    # 用真实控制台启动；隐藏控制台会影响Windows PowerShell启动GUI的方式。
    launcher_log=OUT/'launcher.log'
    with launcher_log.open('w',encoding='utf-8') as log:
        launch=subprocess.Popen(['cmd.exe','/d','/c',str(R/'停车控制台.cmd'),'-CheckFrames','3'],
            cwd=R.parent,stdout=log,stderr=subprocess.STDOUT,creationflags=subprocess.CREATE_NEW_CONSOLE)
        try:
            code=launch.wait(timeout=45)
        except subprocess.TimeoutExpired:
            # 仅清理本检查创建的进程树，避免失败检查遗留窗口。
            subprocess.run(['taskkill.exe','/PID',str(launch.pid),'/T','/F'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
            launch.wait(timeout=10)
            raise
    print(launcher_log.read_text(encoding='utf-8',errors='replace'),flush=True)
    if code:raise subprocess.CalledProcessError(code,launch.args)
    assert image.exists() and image.stat().st_size>1000 and image.stat().st_mtime_ns>previous_image_time
    directory=max((R/'runs/processing').glob('scene_*'),key=lambda p:p.stat().st_mtime)
    config=json.loads((directory/'config.json').read_text(encoding='utf-8'))
    parts=list(directory.glob('part-*.csv.gz'));assert parts
    frames=0
    for p in parts:
        with gzip.open(p,'rt',encoding='utf-8') as f:rows=list(csv.reader(f));assert len(rows[0])==30;frames+=len(rows)-1
    assert frames>0 and (directory/'summary.json').exists()
    assert config['policy_sha256']==c['sha256'] and config['motion_sha256']==c['boundary_motion']['sha256']
    result=dict(unchanged_model_hashes=True,startup_only=args.startup_only,
        model_and_direction_checks='not_run' if args.startup_only else 'passed',
        all_64_match_current_reference=None if args.startup_only else True,
        critical_crossing_cases=None if args.startup_only else 0,
        reported_2_regressions='not_run' if args.startup_only else 'passed',
        parked=None if args.startup_only else 64,actual_cmd_launcher_passed=True,
        processing_window_rendered=True,processing_screenshot=str(image.relative_to(R)),native_replay_frames=frames)
    (OUT/'verification.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
