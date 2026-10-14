from pathlib import Path
import hashlib,json,shutil

R=Path(__file__).resolve().parents[2]
D=R/'results/boundary_motion_v1'
D.mkdir(parents=True,exist_ok=True)
source=R/'training/boundary_heading_cert/java'
target=Path(__file__).parent/'java'
target.mkdir(exist_ok=True)
frozen={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.glob('*.java')}
for p in [R/'models/boundary_heading_selected/policy.bin',R/'models/boundary_heading_selected/policy.pt',R/'configs/CURRENT_SEARCH.json']:
    frozen[str(p.relative_to(R))]=hashlib.sha256(p.read_bytes()).hexdigest()
manifest=D/'frozen_baseline.json'
if manifest.exists():
    saved=json.loads(manifest.read_text())
    assert all(saved[k]==v for k,v in frozen.items() if k.replace('\\','/')!='configs/CURRENT_SEARCH.json'),'Frozen policy/controller changed'
else:manifest.write_text(json.dumps(frozen,indent=2))
for p in source.glob('*.java'):
    q=target/p.name
    if not q.exists():shutil.copy2(p,q)
p=target/'CompactSearch.java'
s=p.read_text(encoding='utf-8')
if 'boundaryMotion' not in s:
    s=s.replace('public final HeadingGate headingGate=new HeadingGate();','public final HeadingGate headingGate=new HeadingGate();\n public final BoundaryMotion boundaryMotion=new BoundaryMotion();')
    s=s.replace('headingGate.observe(this,o);','headingGate.observe(this,o);\n  boundaryMotion.observe(this,o);')
    s=s.replace('public boolean needsAction(long now){return recovery==0&&','public boolean needsAction(long now){return !boundaryMotion.active && recovery==0&&')
    s=s.replace('decisions++;boundaryPivot=wasEdge&&boundaryMode.equals("short_pivot");}', 'decisions++;boundaryPivot=wasEdge&&boundaryMode.equals("short_pivot");boundaryMotion.begin(this,wasEdge,now);}')
    s=s.replace('if(contact||invalid)return velocity(0,0);','if(contact||invalid)return velocity(0,0);\n  ParkingCore.Command pivot=boundaryMotion.command(this);if(pivot!=null)return pivot;')
    p.write_text(s,encoding='utf-8')
print('Prepared isolated actuator overlay; exploration weights, heading gate and parking remain frozen.')
