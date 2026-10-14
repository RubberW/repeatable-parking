"""Share the current implementation and evidence; retain the earlier handoff unchanged."""
from pathlib import Path
import hashlib,json,shutil,zipfile
R=Path(__file__).resolve().parents[1];S=R/'delivery/github_motion_20261014';Z=R/'delivery/boundary_motion_handoff_20261014.zip'
S.mkdir(exist_ok=False)
previous=R/'delivery/github_ready_20261014'
old_manifest=json.loads((previous/'HANDOFF_MANIFEST.json').read_text(encoding='utf-8'))
for rel,digest in old_manifest['files'].items():
    p=previous/rel
    assert hashlib.sha256(p.read_bytes()).hexdigest()==digest,rel
    q=S/rel;q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
roots=['training/boundary_motion','models/boundary_motion_candidate']
items=['README.md','当前方案.txt','configs/CURRENT_SEARCH.json','configs/RETAINED_MODULES.json','docs/项目交接说明.md','docs/触边运动优化报告.md','output/pdf/重复停车项目交接说明.pdf','scripts/current_demo.ps1','scripts/verify_current.py','scripts/build_boundary_motion.ps1','scripts/boundary_motion_demo.ps1','scripts/build_handoff_doc.py','scripts/package_motion.py']
for folder in roots:
    items += [p.relative_to(R).as_posix() for p in (R/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc']
for p in (R/'results/boundary_motion_v1').iterdir():
    if p.is_file() and p.name not in ['initial_screen.csv','initial_screen_v2.csv']:
        items.append(p.relative_to(R).as_posix())
checks=sorted((R/'results/retention_20261013').glob('check_*/verification.json'),key=lambda p:p.stat().st_mtime)
items.append(checks[-1].relative_to(R).as_posix())
for rel in items:
    p=R/rel;q=S/rel;q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
cfg=json.loads((R/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
manifest=dict(updated='2026-10-14',policy_sha256=cfg['sha256'],motion_policy_sha256=cfg['boundary_motion']['sha256'],modules={'exploration':'frozen','local_parking':'frozen','boundary_motion':'implemented_simulation_validated'},files={p.relative_to(S).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in S.rglob('*') if p.is_file()})
(S/'HANDOFF_MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
with zipfile.ZipFile(Z,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in sorted(S.rglob('*')):
        if p.is_file():z.write(p,p.relative_to(S).as_posix())
with zipfile.ZipFile(Z) as z:
    assert z.testzip() is None
    for rel,sha in manifest['files'].items():assert hashlib.sha256(z.read(rel)).hexdigest()==sha,rel
print(json.dumps(dict(files=len(manifest['files'])+1,zip=str(Z),bytes=Z.stat().st_size)))
