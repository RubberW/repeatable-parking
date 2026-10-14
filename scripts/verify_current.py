"""Verify the retained runtime without changing frozen historical evidence."""
from pathlib import Path
import csv
import hashlib
import json
import struct
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results/retention_20261013'/('check_'+uuid.uuid4().hex[:8])
JAVA = 'E:/Processing/app/resources/jdk/bin/java.exe'
CP = ';'.join(map(str, [ROOT/'build/boundary_heading_cert', ROOT/'build/DigitalTwinParkingPrecision',
    Path('E:/Processing/app/resources/core/library/core-4.5.7.jar'),
    Path('E:/Processing/app/resources/modes/java/libraries/net/library/net.jar')]))

def main():
    OUT.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((ROOT/'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
    motion = cfg.get('boundary_motion', {}).get('enabled', False)
    check_cp = CP.replace(str(ROOT/'build/boundary_heading_cert'), str(ROOT/'build/boundary_motion')) if motion else CP
    policy = ROOT/cfg['policy']
    assert hashlib.sha256(policy.read_bytes()).hexdigest() == cfg['sha256']
    assert cfg['model'] == 'heading_seed4' and cfg['temperature'] == .5
    assert cfg['local_method'] == 'early_fit'
    flags = dict(check=True, creationflags=subprocess.CREATE_NO_WINDOW)
    subprocess.run([JAVA, '-Xmx256m', '-cp', check_cp, 'HeadingChecks', str(policy)], **flags)
    subprocess.run([JAVA, '-Xmx256m', '-cp', check_cp, 'BoundaryParity', str(policy), str(OUT/'parity.bin')], **flags)
    raw = (OUT/'parity.bin').read_bytes()
    golden = (ROOT/'results/boundary_heading_v2/final_compare/delivery_parity.bin').read_bytes()
    assert len(raw) == len(golden)
    actual = struct.unpack('>'+str(len(raw)//4)+'f', raw)
    expected = struct.unpack('>'+str(len(golden)//4)+'f', golden)
    error = max(abs(a-b) for a,b in zip(actual, expected))
    assert error < 2e-5, error

    if motion:
        import importlib.util
        spec = importlib.util.spec_from_file_location('current_motion_ui', ROOT/'training/boundary_motion/panel_experiment.py')
        ui = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ui)
        frozen = json.loads((ROOT/'results/boundary_motion_v1/frozen_baseline.json').read_text())
        for path, digest in frozen.items():
            if path.replace('\\', '/') != 'configs/CURRENT_SEARCH.json':
                assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == digest, path
        for name in ('HeadingGate.java', 'ParkingCore.java', 'EdgeTracer.java'):
            assert (ROOT/'training/boundary_motion/java'/name).read_bytes() == (ROOT/'training/boundary_heading_cert/java'/name).read_bytes()
        assert hashlib.sha256((ROOT/cfg['boundary_motion']['policy']).read_bytes()).hexdigest() == cfg['boundary_motion']['sha256']
        subprocess.run([JAVA, '-Xmx256m', '-cp', ui.panel.CP, 'MotionChecks', str(ROOT/cfg['boundary_motion']['policy']), str(ROOT/'results/boundary_motion_v1/motion_parity.csv')], **flags)
    else:
        sys.path.insert(0, str(ROOT/'training/boundary_heading_cert'))
        import panel_experiment as ui
    app = ui.create_app()
    app.withdraw()
    app.update_idletasks()
    app.destroy()
    scene_file = ROOT/('results/boundary_motion_v1/development_scenes.csv' if motion else 'results/boundary_heading_v2/final_compare/scenes.csv')
    scene = next(csv.DictReader(scene_file.open()))
    scenario = ui.panel.make_scenario(width=float(scene['size']), height=float(scene['size']), seed=20261013)
    for dst,src in [('robot_x','x'), ('robot_y','y'), ('robot_yaw','yaw'),
                    ('target_x','target_x'), ('target_y','target_y'), ('target_yaw','target_yaw')]:
        scenario[dst] = float(scene[src])
    for key in ['encoder_seed','policy_seed']:
        scenario[key] = int(scene[key])
    directory, frozen = ui.freeze(scenario, OUT/'integration_runs')
    process, log = ui.worker(directory)
    try:
        assert json.loads(process.stdout.readline())['ready']
        for _ in range(100):
            process.stdin.write('1000\n')
            process.stdin.flush()
            state = json.loads(process.stdout.readline())
            if state['done']:
                break
        else:
            raise AssertionError('Finite integration check did not finish')
        process.wait(timeout=20)
        assert process.returncode == 0
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=20)
        log.close()
    replay_cfg, frames, count, partial = ui.panel.load_replay(directory)
    reference_file = ROOT/('results/boundary_motion_v1/development_learned_full.csv' if motion else 'results/boundary_heading_v2/final_compare/selected_direction_training.csv')
    reference = next(csv.DictReader(reference_file.open()))
    assert state['status'] == 'parked', state
    assert state['first_black_ms']/1000 == float(reference['first_black_s'] if motion else reference['time_s'])
    if motion:
        assert state['time_ms']/1000 == float(reference['total_s'])
        assert replay_cfg['motion_sha256'] == cfg['boundary_motion']['sha256']
        assert (directory/'motion_policy.bin').read_bytes() == (ROOT/cfg['boundary_motion']['policy']).read_bytes()
        result_sources = frozen['controller_sources']
        assert result_sources['ParkingCore.java'] == hashlib.sha256((ROOT/'training/boundary_heading_cert/java/ParkingCore.java').read_bytes()).hexdigest()
    assert not partial and count > 0
    assert replay_cfg['policy_sha256'] == cfg['sha256']
    assert frozen['controller_sources']['HeadingGate.java'] == hashlib.sha256(
        (ROOT/'training/boundary_heading_cert/java/HeadingGate.java').read_bytes()).hexdigest()
    result = dict(policy_sha256=cfg['sha256'], heading_checks='passed', golden_parity_max_error=error,
                  gui_constructed=True, integrated_result=state['status'],
                  first_black_s=state['first_black_ms']/1000, total_s=state['time_ms']/1000,
                  replay_frames=count, replay_complete=True,
                  boundary_motion_enabled=motion,
                  note='One current-runtime integration scene; full paired evaluation is separately archived.')
    (OUT/'verification.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2), flush=True)

if __name__ == '__main__':
    main()
