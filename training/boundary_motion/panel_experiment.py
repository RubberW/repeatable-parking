"""Current conditional boundary-motion module; retained exploration/parking bytes frozen."""
from pathlib import Path
import sys,json,hashlib,subprocess
R=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(R/'training/boundary_heading_cert'))
import panel_experiment as base
panel=base.panel
selection=json.loads((R/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
motion=selection['boundary_motion'];raw=(R/motion['policy']).read_bytes()
if hashlib.sha256(raw).hexdigest()!=motion['sha256']:raise RuntimeError('触边模型校验失败')
panel.CP=';'.join(map(str,[R/'build/boundary_motion',R/'build/DigitalTwinParkingPrecision',Path('E:/Processing/app/resources/core/library/core-4.5.7.jar'),Path('E:/Processing/app/resources/modes/java/libraries/net/library/net.jar')]))
panel.RUNS=R/'data/boundary_motion_runs'
def freeze(c,base_dir=None):
    directory,c=base.freeze(c,base_dir or panel.RUNS)
    (directory/'motion_policy.bin').write_bytes(raw)
    c.update(boundary_motion='conditional_hold_reverse',motion_policy=str(directory/'motion_policy.bin'),motion_sha256=motion['sha256'],parallel_threshold_degrees=motion['parallel_threshold_degrees'],controller_sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (R/'training/boundary_motion/java').glob('*.java')})
    c['notes']='Sensor/encoder edge-angle selector. Near-parallel outward contact: hold outer wheel, reverse inner wheel, rejoin the chosen target. Other poses use original steering. Virtual supported exterior; wheel safety is evaluated, not guaranteed.'
    (directory/'config.json').write_text(json.dumps(c,ensure_ascii=False,indent=2),encoding='utf-8')
    return directory,c
def worker(directory):
    c=json.loads((directory/'config.json').read_text(encoding='utf-8'))
    err=(directory/'worker.log').open('w',encoding='utf-8')
    p=subprocess.Popen([str(panel.JAVA),'-Xmx512m','-Dparking.learnBoundary=true','-Dparking.hardHeading=true',f'-Dparking.policyTemperature={selection["temperature"]}','-Dparking.boundaryMotion=learned',f'-Dparking.motionPolicy={c["motion_policy"]}',f'-Dparking.parallelDegrees={c["parallel_threshold_degrees"]}','-cp',panel.CP,'PanelWorker',str(directory/'config.json')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=err,text=True,encoding='utf-8',creationflags=subprocess.CREATE_NO_WINDOW)
    return p,err
panel.freeze_run=freeze;panel.start_worker=worker
def create_app(size=3,seed=None):
    app=panel.Panel();app.title('重复停车 · 条件触边转弯')
    app.shape_box.configure(values=['矩形 / 正方形']);app.mode.set(next(k for k,v in panel.MODES.items() if v=='continue'));app.margin.set('0')
    def visit(widget):
        if isinstance(widget,panel.ttk.LabelFrame) and widget.cget('text').startswith('②'):
            for child in widget.winfo_children():child.destroy()
            panel.ttk.Label(widget,text='接近平行：靠边轮保持位置，内侧轮后退转向。\n转到向内的姿态后，接回主模型目标。\n其他姿态：直接转弯；越线仍记录。').pack(anchor='w');return
        for child in widget.winfo_children():visit(child)
    visit(app)
    app.model_label.set('探索：heading_seed4｜触框停车：early_fit\n触边运动：条件保持单轮＋内轮后退')
    app.width.set(str(size));app.height.set(str(size));app.preset.set(str(size));app.new_preview()
    if seed is not None:app.scenario=panel.make_scenario(width=size,height=size,seed=seed);app.scenario.update(stop_mode='continue',margin_mm=0);app.reset_view()
    return app
if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--size',type=int,choices=[1,2,3,4],default=3);p.add_argument('--seed',type=int,default=None);a=p.parse_args();create_app(a.size,a.seed).mainloop()
