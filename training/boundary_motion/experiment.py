"""Small controlled contact screen; complete-scene split; immutable output files."""
from pathlib import Path
import csv,random,math,subprocess,sys,json
R=Path(__file__).resolve().parents[2];D=R/'results/boundary_motion_v1'
JAVA='E:/Processing/app/resources/jdk/bin/java.exe'
CP=';'.join(map(str,[R/'build/boundary_motion',R/'build/DigitalTwinParkingPrecision',Path('E:/Processing/app/resources/core/library/core-4.5.7.jar'),Path('E:/Processing/app/resources/modes/java/libraries/net/library/net.jar')]))
def run(name,*args,props=()):
    p=subprocess.run([JAVA,'-Xmx512m',*props,'-cp',CP,name,*map(str,args)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,creationflags=subprocess.CREATE_NO_WINDOW)
    print(p.stdout,flush=True);p.check_returncode()
def scenes(path,n,seed):
    rng=random.Random(seed);rows=[]
    for i in range(n):
        size=1+i%4;half=size*500;near=i%8>=4;yaw=rng.uniform(-math.pi,math.pi)
        ex=105*abs(math.cos(yaw))+148.5*abs(math.sin(yaw));ey=105*abs(math.sin(yaw))+148.5*abs(math.cos(yaw))
        tx=rng.uniform(-half+max(230,ex+70),half-max(230,ex+70));ty=rng.uniform(-half+max(230,ey+70),half-max(230,ey+70))
        if near:
            edge=rng.randrange(4);pad=rng.uniform(10,50)
            if edge<2:tx=(-1 if edge==0 else 1)*(half-ex-pad)
            else:ty=(-1 if edge==2 else 1)*(half-ey-pad)
        while True:
            x,y=rng.uniform(-half+70,half-70),rng.uniform(-half+70,half-70)
            if math.hypot(x-tx,y-ty)>=200:break
        rows.append([size,i,'near_boundary' if near else 'interior',x,y,rng.uniform(-math.pi,math.pi),rng.randrange(2**62),tx,ty,yaw,rng.randrange(2**62),0])
    with path.open('x',newline='') as f:
        w=csv.writer(f);w.writerow('size,id,placement,x,y,yaw,encoder_seed,target_x,target_y,target_yaw,policy_seed,nearest_black_boundary_gap_mm'.split(','));w.writerows(rows)
if __name__=='__main__':
    stage=sys.argv[1]
    if stage=='capture':
        p=D/'development_scenes.csv'
        if not p.exists():scenes(p,24,2026101401)
        run('MotionCapture',p,R/'models/boundary_heading_selected/policy.bin',D/'development_contacts.csv')
    elif stage=='screen':run('MotionReplay',D/'development_contacts.csv',D/'initial_screen_v3.csv','baseline,conditional,always','fixed')
    elif stage=='grid':
        src=list(csv.DictReader((D/'development_contacts.csv').open()));dst=D/'eligible_contacts_v3.csv'
        rows=[r for r in src if r['reliable']=='true' and float(r['estimated_alpha'])<=25 and abs(math.sin(float(r['normal_delta'])))>=math.sin(math.radians(3)) and math.cos(float(r['normal_delta']))>0]
        with dst.open('x',newline='') as f:w=csv.DictWriter(f,fieldnames=src[0]);w.writeheader();w.writerows(rows)
        run('MotionReplay',dst,D/'parameter_screen_v3.csv','baseline,conditional','grid')
    elif stage=='validate':
        run('MotionChecks',R/'models/boundary_motion_candidate/policy.bin',D/'motion_parity.csv')
        run('MotionReplay',D/'development_contacts.csv',D/'learned_contact_screen.csv','learned','fixed',props=[f'-Dparking.motionPolicy={R / "models/boundary_motion_candidate/policy.bin"}'])
        for mode in ('baseline','learned'):
            run('MotionFullEval',D/'development_scenes.csv',R/'models/boundary_heading_selected/policy.bin',D/f'development_{mode}_full.csv',props=[f'-Dparking.boundaryMotion={mode}',f'-Dparking.motionPolicy={R / "models/boundary_motion_candidate/policy.bin"}'])
    elif stage=='holdout':
        p=D/'fresh_holdout_scenes.csv'
        if not p.exists():scenes(p,64,2026101402)
        for mode in ('baseline','learned'):
            run('MotionFullEval',p,R/'models/boundary_heading_selected/policy.bin',D/f'fresh_holdout_{mode}_full.csv',props=[f'-Dparking.boundaryMotion={mode}',f'-Dparking.motionPolicy={R / "models/boundary_motion_candidate/policy.bin"}'])
