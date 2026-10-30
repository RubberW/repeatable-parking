"""
文件职责：并行运行 SearchCoverageEvaluation，合并逐场景结果并打印搜索质量汇总。
输入：场景CSV、标签、若干 -D 设置；输出：experiments/results/search_coverage_20261030/<标签>.csv。
调用关系：手动运行；需先执行 scripts/build.ps1 -WithChecks。不训练、不修改模型。
"""
from pathlib import Path
import argparse, csv, json, statistics, subprocess

R = Path(__file__).resolve().parents[1]
OUT = R / 'experiments' / 'results' / 'search_coverage_20261030'
JAVA = 'E:/Processing/app/resources/jdk/bin/java.exe'
CP = ';'.join([str(R / '.cache/classes'), 'E:/Processing/app/resources/core/library/core-4.5.7.jar'])


def summarise(rows):
    found = [r for r in rows if float(r['first_black_s']) >= 0]
    mean = lambda k, rs=rows: statistics.fmean(float(r[k]) for r in rs) if rs else float('nan')
    contacts = sum(int(r['edge_contacts']) for r in rows)
    return dict(scenes=len(rows), found=len(found), parked=sum(r['parked'] == 'true' for r in rows),
                mean_first_black_s=round(mean('first_black_s', found), 2),
                median_first_black_s=round(statistics.median(float(r['first_black_s']) for r in found), 2) if found else None,
                mean_total_s=round(mean('total_s'), 2), coverage_eff=round(mean('coverage_eff'), 3),
                parallel_lt70=round(mean('parallel_lt70'), 3), parallel_70_220=round(mean('parallel_70_220'), 3),
                reverse_share=round(sum(int(r['reverse_gt150']) for r in rows) / max(1, contacts), 3),
                search_full_wheel_s=round(mean('search_full_wheel_s'), 3),
                scenes_with_critical=sum(float(r['total_critical_s']) > 0 for r in rows),
                worst_wheel_mm=round(min(float(r['search_min_wheel_mm']) for r in rows), 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scenes')
    ap.add_argument('label')
    ap.add_argument('-D', action='append', default=[], help='extra JVM property, e.g. parking.gate=direction')
    ap.add_argument('--shards', type=int, default=16)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((R / 'configs/CURRENT_SEARCH.json').read_text(encoding='utf-8'))
    main_model, motion_model = R / cfg['policy'], R / cfg['boundary_motion']['policy']
    props = ['-Dparking.boundaryMotion=learned', f'-Dparking.motionPolicy={motion_model}',
             f"-Dparking.parallelDegrees={cfg['boundary_motion'].get('parallel_threshold_degrees', 25)}"] + [f'-D{p}' for p in a.D]
    parts = [OUT / f'{a.label}.part{i}.csv' for i in range(a.shards)]
    procs = [subprocess.Popen([JAVA, '-Xmx384m', *props, '-cp', CP, 'SearchCoverageEvaluation', a.scenes, str(main_model), str(p), str(i), str(a.shards)],
                              stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True) for i, p in enumerate(parts)]
    for p in procs:
        _, err = p.communicate()
        if p.returncode:
            raise SystemExit(err)
    rows = []
    for p in parts:
        with p.open(encoding='utf-8') as f:
            rows += list(csv.DictReader(f))
        p.unlink()
    rows.sort(key=lambda r: int(r['scene']))
    final = OUT / f'{a.label}.csv'
    with final.open('x', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    s = summarise(rows)
    s.update(label=a.label, scenes_file=Path(a.scenes).name, properties=a.D)
    (OUT / f'{a.label}.summary.json').write_text(json.dumps(s, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(s, ensure_ascii=False))


if __name__ == '__main__':
    main()
