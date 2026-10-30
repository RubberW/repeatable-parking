"""
文件职责：按开发集与固定64新场景的分布生成独立的调参场景，避免用最终测试集选参数。
输入：随机种子与每种尺寸、每种摆放的数量；输出：experiments/data/tuning_scenes_<seed>.csv。
调用关系：手动运行一次；场景与最终测试集无重叠。
分布依据：普通目标框心离边≥240毫米；靠边目标框心离最近边130～230毫米；
小车离边≥90毫米、离框心≥300毫米；朝向均匀随机。
"""
from pathlib import Path
import argparse, math, random

R = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=20261030)
    ap.add_argument('--per-cell', type=int, default=8)
    ap.add_argument('--replicate', type=int, default=0, help='把已生成的调参场景各复制N份，只换主模型采样种子')
    ap.add_argument('--prefix', default='tuning_scenes', help='确认测试集用 confirm_scenes，与调参集分开')
    a = ap.parse_args()
    if a.replicate:
        src = R / 'experiments' / 'data' / f'{a.prefix}_{a.seed}.csv'
        lines = src.read_text(encoding='utf-8').splitlines()
        rng = random.Random(a.seed + 1)
        out = src.with_name(f'{a.prefix}_{a.seed}_x{a.replicate}.csv')
        with out.open('x', encoding='utf-8', newline='') as f:
            f.write(lines[0] + '\n')
            sid = 0
            for rep in range(a.replicate):
                for row in lines[1:]:
                    c = row.split(',')
                    c[1] = str(sid); c[10] = str(rng.getrandbits(62)); sid += 1
                    f.write(','.join(c) + '\n')
        print(out, sid)
        return
    rng = random.Random(a.seed)
    out = R / 'experiments' / 'data' / f'{a.prefix}_{a.seed}.csv'
    rows, sid = [], 0
    for _ in range(a.per_cell):
        for size in (1, 2, 3, 4):
            for placement in ('interior', 'near_boundary'):
                h = size * 500
                if placement == 'interior':
                    tx, ty = rng.uniform(-(h - 240), h - 240), rng.uniform(-(h - 240), h - 240)
                else:
                    gap, along = rng.uniform(130, 230), rng.uniform(-(h - 240), h - 240)
                    side = rng.randrange(4)
                    tx, ty = [(h - gap, along), (-(h - gap), along), (along, h - gap), (along, -(h - gap))][side]
                while True:
                    x, y = rng.uniform(-(h - 90), h - 90), rng.uniform(-(h - 90), h - 90)
                    if math.hypot(x - tx, y - ty) >= 300:
                        break
                rows.append([size, sid, placement, x, y, rng.uniform(-math.pi, math.pi), rng.getrandbits(62),
                             tx, ty, rng.uniform(-math.pi, math.pi), rng.getrandbits(62), 0])
                sid += 1
    with out.open('x', encoding='utf-8', newline='') as f:
        f.write('size,id,placement,x,y,yaw,encoder_seed,target_x,target_y,target_yaw,policy_seed,nearest_black_boundary_gap_mm\n')
        for r in rows:
            f.write(','.join(str(v) for v in r) + '\n')
    print(out, len(rows))


if __name__ == '__main__':
    main()
