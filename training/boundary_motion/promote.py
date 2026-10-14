"""Promote the completed wheel executor without changing retained policy/controller files."""
from pathlib import Path
import json,hashlib,shutil
R=Path(__file__).resolve().parents[2];p=R/'configs/CURRENT_SEARCH.json';c=json.loads(p.read_text(encoding='utf-8'))
backup=R/'results/boundary_motion_v1/baseline_selection.json'
if not backup.exists():backup.write_bytes(p.read_bytes())
meta=json.loads((R/'models/boundary_motion_candidate/metadata.json').read_text());summary=json.loads((R/'results/boundary_motion_v1/summary.json').read_text())
assert summary['fresh_holdout']['learned']['parked']==64
assert hashlib.sha256((R/'models/boundary_heading_selected/policy.bin').read_bytes()).hexdigest()==c['sha256']
c.update(runtime_overlay='training/boundary_motion/java',wheel_motion_model='conditional_hold_reverse_v1',scope='Keep exploration, direction gate and early_fit frozen; conditional boundary wheel motion implemented and simulation validated.')
c['boundary_motion']=dict(enabled=True,model='conditional_hold_reverse_v1',policy='models/boundary_motion_candidate/policy.bin',sha256=meta['sha256'],parallel_threshold_degrees=25,report='docs/触边运动优化报告.md',evaluation='results/boundary_motion_v1/summary.json',inference_inputs=['main_model_selected_heading','original_heading'],guard_inputs='reflectance and encoder edge evidence; estimated wheel velocity',simulated_only=True)
p.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
p=R/'configs/RETAINED_MODULES.json';c=json.loads(p.read_text(encoding='utf-8'));c['updated']='2026-10-14';m=c['modules'][2]
m.update(status='implemented_simulation_validated',inputs=['main-model selected heading','original heading'],outputs='Angle-conditioned inner-wheel reverse speed and inward tilt; encoder-feedback controller emits left/right commands.',policy='models/boundary_motion_candidate/policy.bin',sha256=meta['sha256'],guard='Sensor/encoder straight-edge estimate; outward near-parallel asymmetric contact only; direct steering otherwise.',constraints='Exploration weights, HeadingGate and early_fit remain frozen. Simulator truth is used only in evaluation. No hardware upload.',report='docs/触边运动优化报告.md',evaluation='results/boundary_motion_v1/summary.json',limitation='Reduces observed risk, does not eliminate wheel/centre excursion; motor inertia means the held wheel still moves initially.')
p.write_text(json.dumps(c,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
