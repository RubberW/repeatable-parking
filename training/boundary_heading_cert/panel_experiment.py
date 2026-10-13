"""Frozen validation-selected main policy; experimental direction-gated window."""
from pathlib import Path
import sys, json, hashlib, subprocess

R = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(R/'control_panel'))
import panel

selection = json.loads((R/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
raw = (R/selection['policy']).read_bytes()
if hashlib.sha256(raw).hexdigest() != selection['sha256']:
    raise RuntimeError('实验模型校验失败')
panel.CP = ';'.join(map(str, [R/'build/boundary_heading_cert', R/'build/DigitalTwinParkingPrecision',
    Path('E:/Processing/app/resources/core/library/core-4.5.7.jar'),
    Path('E:/Processing/app/resources/modes/java/libraries/net/library/net.jar')]))
panel.RUNS = R/'data/boundary_heading_runs'
original_freeze = panel.freeze_run

def freeze(c, base=None):
    # This experiment's completed evaluation covers rectangular tables only.
    if c['shape'] != 'rectangle':
        raise ValueError('本次方向学习实验仅开放矩形桌面；其他形状请使用原控制台。')
    c = dict(c, stop_mode='continue')
    directory, c = original_freeze(c, base or panel.RUNS)
    (directory/'policy.bin').write_bytes(raw)
    c.update(model=selection['model'], policy_sha256=selection['sha256'],
        policy_temperature=selection['temperature'], learn_boundary=True,
        heading_gate='recent_interior_witness',
        controller_sources={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (R/'training/boundary_heading_cert/java').glob('*.java')},
        notes=c['notes']+' Main-policy interior subgoal constraint; convex table assumed. '
            'Local odometry allowance 8mm is not hardware calibrated. '
            'Wheel-swept turning paths remain unconstrained. No automatic stop at wheel crossing.')
    (directory/'config.json').write_text(json.dumps(c, ensure_ascii=False, indent=2), encoding='utf-8')
    return directory, c

def worker(directory):
    err = (directory/'worker.log').open('w', encoding='utf-8')
    p = subprocess.Popen([str(panel.JAVA), '-Xmx1g', '-Dparking.learnBoundary=true',
        '-Dparking.hardHeading=true', f"-Dparking.policyTemperature={selection['temperature']}",
        '-cp', panel.CP, 'PanelWorker', str(directory/'config.json')],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err, text=True,
        encoding='utf-8', creationflags=subprocess.CREATE_NO_WINDOW)
    return p, err

panel.freeze_run = freeze
panel.start_worker = worker

class ExperimentPanel(panel.Panel):
    def draw(self):
        super().draw()
        if self.current_frame and len(self.current_frame) >= 30:
            f = self.current_frame
            self.stats.set(self.stats.get()+f'\n轮子越线比例 {f[24]:.0%} / {f[25]:.0%}｜严重越界 {int(f[28])} 次（原转弯执行器）')

def create_app(size=3, seed=None):
    app = ExperimentPanel()
    app.title('重复停车 · 当前保留方案')
    app.shape_box.configure(values=['矩形 / 正方形'])
    app.mode.set(next(k for k,v in panel.MODES.items() if v=='continue'))
    app.margin.set('0')
    def visit(widget):
        if isinstance(widget, panel.ttk.LabelFrame) and widget.cget('text').startswith('②'):
            for child in widget.winfo_children(): child.destroy()
            panel.ttk.Label(widget, text='触边后：只允许指向已确认桌内区域的子目标。\n主模型选择其中更利于搜索的方向。\n轮子越线继续记录；转弯执行器尚未优化。').pack(anchor='w')
            return
        for child in widget.winfo_children(): visit(child)
    visit(app)
    app.model_label.set('探索：heading_seed4｜触框停车：early_fit\n触边运动小模型：待开发｜1～4米已对照')
    app.width.set(str(size)); app.height.set(str(size)); app.preset.set(str(size))
    app.new_preview()
    if seed is not None:
        app.scenario=panel.make_scenario(width=size,height=size,seed=seed)
        app.scenario.update(stop_mode='continue',margin_mm=0)
        app.reset_view()
    return app

if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('--size',type=int,choices=[1,2,3,4],default=3)
    parser.add_argument('--seed',type=int,default=None)
    args=parser.parse_args()
    create_app(args.size,args.seed).mainloop()
