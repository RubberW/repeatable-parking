"""
文件职责：按场景配对比较两组 SearchCoverageEvaluation 结果，避免只在找到黑框的场景上求均值造成偏差。
输入：基线CSV、若干对照CSV；输出：打印配对汇总（找到数、配对时间差、几何平均比、胜负、覆盖与越线）。
调用关系：手动运行；只读结果文件。
"""
import csv, math, statistics, sys
from pathlib import Path


def load(p):
    with open(p, encoding='utf-8') as f:
        return {r['scene']: r for r in csv.DictReader(f)}


def line(name, base, var):
    common = [s for s in base if s in var]
    both = [s for s in common if float(base[s]['first_black_s']) >= 0 and float(var[s]['first_black_s']) >= 0]
    b = [float(base[s]['first_black_s']) for s in both]; v = [float(var[s]['first_black_s']) for s in both]
    ratio = math.exp(statistics.fmean(math.log(y / x) for x, y in zip(b, v))) if both else float('nan')
    wins = sum(y < x * .9 for x, y in zip(b, v)); losses = sum(y > x * 1.1 for x, y in zip(b, v))
    found = sum(float(var[s]['first_black_s']) >= 0 for s in common)
    parked = sum(var[s]['parked'] == 'true' for s in common)
    mean = lambda k: statistics.fmean(float(var[s][k]) for s in common)
    crit = sum(float(var[s]['total_critical_s']) > 0 for s in common)
    return (f"{name:<22} found {found:2d}/{len(common)} parked {parked:2d} | paired n={len(both):2d} "
            f"mean {statistics.fmean(b):6.1f}->{statistics.fmean(v):6.1f}s  median {statistics.median(b):6.1f}->{statistics.median(v):6.1f}s  "
            f"geo-ratio {ratio:.3f}  faster/slower(>10%) {wins:2d}/{losses:2d} | eff {mean('coverage_eff'):.3f} "
            f"par<70 {mean('parallel_lt70'):.3f} par70-220 {mean('parallel_70_220'):.3f} | fullwheel {mean('search_full_wheel_s'):5.2f}s critical-scenes {crit}")


def main():
    base = load(sys.argv[1])
    print(line(Path(sys.argv[1]).stem + ' (base)', base, base))
    for p in sys.argv[2:]:
        print(line(Path(p).stem, base, load(p)))


if __name__ == '__main__':
    main()
