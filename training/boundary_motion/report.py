from pathlib import Path
import csv,json,statistics as st,hashlib,random,math
R=Path(__file__).resolve().parents[2];D=R/'results/boundary_motion_v1'
def read(name):return list(csv.DictReader((D/name).open()))
def summarize(rows):
    return dict(cases=len(rows),parked=sum(r['parked']=='true' for r in rows),false_completion=sum(r['false_completion']=='true' for r in rows),mean_search_s=st.mean(float(r['first_black_s']) for r in rows),mean_total_s=st.mean(float(r['total_s']) for r in rows),mean_full_wheel_s=st.mean(float(r['search_full_wheel_s']) for r in rows),mean_critical_s=st.mean(float(r['search_critical_s']) for r in rows),critical_cases=sum(float(r['search_critical_s'])>0 for r in rows),worst_wheel_excursion_mm=max(0,-min(float(r['search_min_wheel_mm']) for r in rows)),motion_starts=sum(int(r['motion_starts']) for r in rows),max_hold_drift_mm=max(float(r['max_hold_drift_mm']) for r in rows))
validation={m:summarize([r for r in read(f'development_{m}_full.csv') if int(r['scene'])>=16]) for m in ('baseline','learned')}
holdout={m:summarize(read(f'fresh_holdout_{m}_full.csv')) for m in ('baseline','learned')}
a=read('fresh_holdout_baseline_full.csv');b=read('fresh_holdout_learned_full.csv');assert [r['scene'] for r in a]==[r['scene'] for r in b]
rng=random.Random(20261014);ci={}
for metric in ('first_black_s','search_full_wheel_s','search_critical_s'):
    d=[float(y[metric])-float(x[metric]) for x,y in zip(a,b)]
    boot=sorted(st.mean(rng.choices(d,k=64)) for _ in range(3000));ci[metric]=dict(mean_difference=st.mean(d),paired_bootstrap_95_percent=[boot[75],boot[2924]])
meta=json.loads((R/'models/boundary_motion_candidate/metadata.json').read_text())
summary=dict(model_sha256=meta['sha256'],validation=validation,fresh_holdout=holdout,paired_differences=ci,train_scenes=16,validation_scenes=8,holdout_scenes=64,contact_states=len(read('development_contacts.csv')),selection='A single angle-conditioned candidate, frozen before fresh holdout. Validation retained parking success and reduced mean support-risk measures, with slower search and a larger worst excursion. No retuning on fresh holdout.',excluded_evidence=['initial_screen.csv: empty witness history caused fixture stop','initial_screen_v2.csv: inappropriate fixed 80mm progress criterion'],method='Identical geometry, seeds, dynamics, exploration policy and early_fit. Only wheel execution differs. Sensor-derived edge estimate at inference; true geometry only in evaluator.')
(D/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
old,new=holdout['baseline'],holdout['learned']
text=f'''# 触边运动小模型：条件保持单轮转弯

你的思路已做成可运行模块：轮轴与边线接近平行时，靠边轮用编码器反馈保持位置，内侧轮后退；转到朝桌内的姿态后，接回主模型选好的目标。其他姿态继续直接转弯。

## 模型与动作如何分工

网络只接收两个方向：主模型选好的目标方向、触边前的原始方向。先转换为相对夹角，再输出内侧轮后退速度和向内转多少。权重只有304字节，运行不依赖PyTorch。模型文件：`models/boundary_motion_candidate/policy.bin`。

这两个方向本身不能告诉小车边线在哪里、哪只轮子靠边，因此动作选择器另外使用反射率和编码器估计边线。估计轮轴与边线夹角不超过25度、车头仍朝外、两轮距边有差别，且直边证据充分时，才启用特殊动作。车头已向内、接近垂直、无法确认边线或两轮对称时，沿用原转弯。25度是本次仿真阈值，边线估计会有误差。

两条线的夹角按0到90度理解。标作170度的两条线仍接近平行；是否需要特殊动作还取决于车头朝里还是朝外。

网络根据目标方向学会不同的向内转角：主模型相对目标约112.5、135、157.5、180度时，训练筛选的向内倾角分别约22、10、22、35度。后退速度约150毫米/秒。当前选型将“转完后额外直走固定距离”设为0，直接接回原目标，避免走过短子目标。

## 为什么这样做

接近平行且车头朝外时，靠边轮若向前滚就容易越线。以它为支点，让桌内轮后退，能把车头转向桌内。保持轮有电机惯性，零PWM不等于锁住；控制器根据编码器位移和速度补偿，初始惯性位移仍存在。其余姿态用原控制器直接转，避免每次触边都增加退让动作。

## 实验与结果

开发使用24个完整场景：16个训练、8个验证，采集{summary['contact_states']}个实际触边状态。对符合条件的54个状态筛选27组参数，训练时只用训练场景。局部对照从相同位置、朝向、轮速、目标和传感器边线估计开始，不把同一事件的相邻帧拆进训练和验证。单目标局部夹具不重建方向凸包；该目标已经由原主模型和约束选出，完整运行一直保留原约束。

8个完整验证场景，两版均8/8停车。新动作的平均严重越界时间从{validation['baseline']['mean_critical_s']:.3f}降到{validation['learned']['mean_critical_s']:.3f}秒；搜索从{validation['baseline']['mean_search_s']:.2f}增加到{validation['learned']['mean_search_s']:.2f}秒，最坏轮心外探从{validation['baseline']['worst_wheel_excursion_mm']:.2f}增加到{validation['learned']['worst_wheel_excursion_mm']:.2f}毫米。按减少平均越界、保持停车成功的目的保留此候选，之后冻结权重。最坏事件还需要继续处理。

再用64个新场景做相同种子的完整搜索和停车配对测试：

| 指标 | 原运动执行器 | 条件触边小模型 |
| --- | --- | --- |
| 正确停车 | {old['parked']}/64 | {new['parked']}/64 |
| 错误报告停车完成 | {old['false_completion']} | {new['false_completion']} |
| 平均找到黑框 | {old['mean_search_s']:.2f}秒 | {new['mean_search_s']:.2f}秒 |
| 平均完成停车 | {old['mean_total_s']:.2f}秒 | {new['mean_total_s']:.2f}秒 |
| 平均整轮支撑代理越线时间 | {old['mean_full_wheel_s']:.3f}秒 | {new['mean_full_wheel_s']:.3f}秒 |
| 平均重心或双轮心越线时间 | {old['mean_critical_s']:.3f}秒 | {new['mean_critical_s']:.3f}秒 |
| 出现重心或双轮心越线的场景 | {old['critical_cases']}/64 | {new['critical_cases']}/64 |
| 最坏轮心外探 | {old['worst_wheel_excursion_mm']:.2f}毫米 | {new['worst_wheel_excursion_mm']:.2f}毫米 |

新模型在这批场景上减少了越线，也保持完整停车成功。搜索时间的配对差值95%自助区间为{ci['first_black_s']['paired_bootstrap_95_percent'][0]:.2f}到{ci['first_black_s']['paired_bootstrap_95_percent'][1]:.2f}秒，不能据此承诺任何场景都更快。全部时间为模拟时间。

保持阶段的最大编码器位移为{new['max_hold_drift_mm']:.2f}毫米，主要受触边时已有轮速和仿真假设制动能力影响。它是位置保持控制，不能宣称轮子瞬间静止。仍有{new['critical_cases']}/64个严重越界场景，因此这是已实现并经过仿真验证的改进模型，尚不能保证真桌沿不掉落。边线估计、角落和大角度直接转弯是接下来优先检查的剩余问题。

## 使用与保留

双击`停车控制台.cmd`即可运行当前三个模块。第1个探索权重、第2个early_fit控制器与HeadingGate源码均保持原文件哈希；新代码在`training/boundary_motion/java`，仅CompactSearch增加运动模块接入点。以前的原执行器代码和2026年10月13日冻结包仍保留。

验证：先运行`scripts/build_boundary_motion.ps1`，再运行`scripts/verify_current.py`。训练另需E盘`tools/pydeps`中的NumPy、PyTorch；普通运行只需现有Python/Tk与Processing。

逐场景结果和种子位于`results/boundary_motion_v1/fresh_holdout_*`，摘要在同目录`summary.json`。数值推理与Java的一致性最大误差小于0.00001，并验证了方向旋转和左右镜像一致性。探索和停车源码的冻结清单在`frozen_baseline.json`。
'''
(R/'docs/触边运动优化报告.md').write_text(text,encoding='utf-8')
print(json.dumps(holdout,indent=2))
