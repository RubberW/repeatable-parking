"""Distil empirical angle-conditioned manoeuvre parameters; no geometry in model inputs."""
from pathlib import Path
import csv,json,math,hashlib,struct,sys
import numpy as np
import torch
R=Path(__file__).resolve().parents[2];D=R/'results/boundary_motion_v1';M=R/'models/boundary_motion_candidate';M.mkdir(exist_ok=True)
contacts={(r['scene'],r['event']):r for r in csv.DictReader((D/'development_contacts.csv').open())}
rows=[r for r in csv.DictReader((D/'parameter_screen_v3.csv').open()) if r['mode']=='conditional' and int(r['scene'])<16]
groups={}
def objective(r):return 100*float(r['critical_s'])+float(r['depth_time'])+.5*float(r['extra_excursion'])+.2*float(r['time_s'])+5*(r['status']!='rejoined')
for r in rows:
    angle=round(abs(float(contacts[r['scene'],r['event']]['delta']))*180/math.pi,1)
    p=tuple(float(r[k]) for k in ('speed','tilt','advance'))
    groups.setdefault(angle,{}).setdefault(p,[]).append(r)
targets=[]
for angle,g in sorted(groups.items()):
    p=min(g,key=lambda q:np.mean([objective(r) for r in g[q]]))
    targets.append(dict(angle=angle,count=len(g[p]),parameters=p,objective=float(np.mean([objective(r) for r in g[p]]))))
torch.set_num_threads(2);torch.manual_seed(20261014)
net=torch.nn.Sequential(torch.nn.Linear(2,12),torch.nn.Tanh(),torch.nn.Linear(12,3),torch.nn.Sigmoid())
a=torch.tensor([t['angle']*math.pi/180 for t in targets],dtype=torch.float32)
x=torch.stack([torch.cos(a),torch.abs(torch.sin(a))],dim=1)
y=torch.tensor([[(t['parameters'][0]-50)/110,(t['parameters'][1]-8)/32,t['parameters'][2]/100] for t in targets],dtype=torch.float32)
opt=torch.optim.Adam(net.parameters(),lr=.015)
for epoch in range(2400):
    opt.zero_grad();loss=((net(x)-y)**2).mean();loss.backward();opt.step()
state=net.state_dict();raw=struct.pack('>I',0x424d4d31)
for key in ['0.weight','0.bias','2.weight','2.bias']:
    arr=state[key].detach().numpy().ravel();raw+=struct.pack('>'+str(len(arr))+'f',*arr)
p=M/'policy.bin'
if p.exists():raise RuntimeError('Preserve candidate weights')
p.write_bytes(raw)
meta=dict(format='BMM1',bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest(),inputs=['chosen_heading','original_heading'],features=['cos(chosen-original)','abs(sin(chosen-original))'],outputs=['inner_reverse_speed_mm_s','inward_tilt_deg','advance_mm'],training_scene_ids=list(range(16)),validation_scene_ids=list(range(16,24)),targets=targets,loss=float(loss.detach()),seed=20261014,method='27 candidate parameter sets per eligible contact; train scenes only; distilled angle-bucket means',activation='Measured axle-edge angle <=25deg; outward heading; asymmetric wheel clearance; reliable straight-edge fit',limitations='Simulation actuator assumptions. Two headings alone do not identify an edge angle or the nearer wheel; these are handled by a sensor/encoder gate.')
(M/'metadata.json').write_text(json.dumps(meta,indent=2),encoding='utf-8')
with (D/'motion_parity.csv').open('x',newline='') as f:
    w=csv.writer(f);w.writerow(['chosen','original','speed','tilt','advance'])
    for original in [-3.1,0,2.4]:
        for delta in np.linspace(-math.pi,math.pi,41):
            with torch.no_grad():v=net(torch.tensor([[math.cos(delta),abs(math.sin(delta))]],dtype=torch.float32))[0].numpy()
            w.writerow([original+delta,original,50+110*float(v[0]),8+32*float(v[1]),0 if v[2]<.05 else 100*float(v[2])])
print(json.dumps(meta,indent=2))
