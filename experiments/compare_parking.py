"""
文件职责：按场景配对比较两组 SearchCoverageEvaluation 结果的局部停车阶段（找到黑框之后）。
输入：基线CSV、对照CSV；输出：打印停车数、误报停车、局部用时（两组都停车的场景）及严重越线。
调用关系：手动运行；只读结果文件。搜索阶段两组应逐场景相同，否则单独报告。
"""
import csv, statistics, sys
from pathlib import Path


def load(p):
    with open(p, encoding='utf-8') as f:
        return {r['scene']: r for r in csv.DictReader(f)}


def local(r):
    return float(r['total_s']) - float(r['first_black_s'])


def main():
    base, var = load(sys.argv[1]), load(sys.argv[2])
    common = [s for s in base if s in var]
    same_search = sum(base[s]['first_black_s'] == var[s]['first_black_s'] for s in common)
    both = [s for s in common if base[s]['parked'] == var[s]['parked'] == 'true']
    b = sorted(local(base[s]) for s in both); v = sorted(local(var[s]) for s in both)
    q = lambda xs, f: xs[min(len(xs) - 1, int(f * len(xs)))]
    count = lambda rows, k: sum(rows[s][k] == 'true' for s in common)
    crit = lambda rows: sum(float(rows[s]['total_critical_s']) > 0 for s in common)
    faster = sum(local(var[s]) < local(base[s]) for s in both)
    print(f"{Path(sys.argv[2]).stem} vs {Path(sys.argv[1]).stem}: scenes {len(common)}, identical search phase {same_search}")
    print(f"  parked {count(base, 'parked')} -> {count(var, 'parked')}  false completion {count(base, 'false_completion')} -> {count(var, 'false_completion')}"
          f"  critical-crossing scenes {crit(base)} -> {crit(var)}")
    print(f"  local parking s (both parked, n={len(both)}): mean {statistics.fmean(b):.2f} -> {statistics.fmean(v):.2f}"
          f"  median {q(b, .5):.2f} -> {q(v, .5):.2f}  p90 {q(b, .9):.2f} -> {q(v, .9):.2f}  max {b[-1]:.2f} -> {v[-1]:.2f}  faster in {faster}/{len(both)}")
    print(f"  mean total s (all scenes): {statistics.fmean(float(base[s]['total_s']) for s in common):.1f} -> {statistics.fmean(float(var[s]['total_s']) for s in common):.1f}")


if __name__ == '__main__':
    main()
