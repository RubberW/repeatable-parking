"""
文件职责：用已有触边参数对照数据训练小网络，单独保留Python训练入口。
输入：experiments/data中的触边状态和参数筛选结果；输出：新权重及训练说明。
调用关系：手动训练时使用；不修改正在运行的模型，普通仿真不需要PyTorch。
"""
from pathlib import Path
import csv,json,math,hashlib,struct,uuid
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'experiments/data'
OUT=ROOT/'experiments/outputs'/('motion_'+uuid.uuid4().hex[:8])

def objective(row):
    # 优先减少严重越界；再比较外探、恢复时间和未完成动作。
    return 100*float(row['critical_s'])+float(row['depth_time'])+.5*float(row['extra_excursion'])+.2*float(row['time_s'])+5*(row['status']!='rejoined')

def main():
    contacts={(r['scene'],r['event']):r for r in csv.DictReader((DATA/'development_contacts.csv').open())}
    rows=[r for r in csv.DictReader((DATA/'parameter_screen_v3.csv').open()) if r['mode']=='conditional' and int(r['scene'])<16]
    groups={}
    for r in rows:
        angle=round(abs(float(contacts[r['scene'],r['event']]['delta']))*180/math.pi,1)
        parameters=tuple(float(r[k]) for k in ('speed','tilt','advance'))
        groups.setdefault(angle,{}).setdefault(parameters,[]).append(r)
    targets=[]
    for angle,g in sorted(groups.items()):
        p=min(g,key=lambda q:np.mean([objective(r) for r in g[q]]))
        targets.append(dict(angle=angle,parameters=p))

    # 两方向变成相对角度；左右镜像与同时旋转不改变运动参数。
    torch.set_num_threads(2);torch.manual_seed(20261014)
    net=torch.nn.Sequential(torch.nn.Linear(2,12),torch.nn.Tanh(),torch.nn.Linear(12,3),torch.nn.Sigmoid())
    angles=torch.tensor([t['angle']*math.pi/180 for t in targets],dtype=torch.float32)
    x=torch.stack([torch.cos(angles),torch.abs(torch.sin(angles))],dim=1)
    y=torch.tensor([[(t['parameters'][0]-50)/110,(t['parameters'][1]-8)/32,t['parameters'][2]/100] for t in targets],dtype=torch.float32)
    optimizer=torch.optim.Adam(net.parameters(),lr=.015)
    for _ in range(2400):
        optimizer.zero_grad();loss=((net(x)-y)**2).mean();loss.backward();optimizer.step()

    # 导出与Java读取器一致的大端浮点格式；不会覆盖models中的保留模型。
    raw=struct.pack('>I',0x424d4d31)
    for key in ['0.weight','0.bias','2.weight','2.bias']:
        values=net.state_dict()[key].detach().numpy().ravel()
        raw+=struct.pack('>'+str(len(values))+'f',*values)
    OUT.mkdir(parents=True,exist_ok=False);(OUT/'policy.bin').write_bytes(raw)
    result=dict(sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw),loss=float(loss.detach()),targets=targets,training_scene_ids=list(range(16)),outputs=['inner_reverse_speed_mm_s','inward_tilt_deg','advance_mm'])
    (OUT/'training.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps(dict(directory=str(OUT),**result),indent=2))
if __name__=='__main__':main()
