from pathlib import Path
R=Path(__file__).resolve().parents[2]
p=R/'docs/项目交接说明.md';s=p.read_text(encoding='utf-8')
s=s.replace('下一步开发第三个模块，专门优化触边后的轮子运动。第三个模块尚未实现，也没有可供运行的第三份神经网络权重。','第三个模块现已实现并完成仿真对照，接近平行触边时保持靠边轮、让内侧轮后退；其余姿态直接转弯。')
s=s.replace('| 触边运动小模型 | 待开发 | 怎样把选好的方向执行出来，减少转弯越线 |','| 条件触边运动小模型 | 已实现并经仿真验证 | 接近平行时保持靠边轮、内侧轮后退；其余直接转 |')
s=s.replace('C --> D[已有轮子运动执行器]','C --> D[条件轮子运动执行器]').replace('H[下一步 触边运动小模型] -.替换触边执行部分.-> D','H[触边姿态判断与小模型] --> D')
old='触边后，`HeadingGate.java` 根据近期反射率确认的桌内点和编码器里程计，构造向内收缩8毫米的凸包，限制有限子目标落在已确认的桌内区域。网络在允许的候选中自由选择；没有可确认候选时停止。8毫米是当前仿真假设，约束依赖凸形场地和较小的近期里程计误差。'
new='触边后，先把小车近期已经探到的桌内位置连成一块区域，并向内留8毫米余量。主模型只能选择终点落在这块区域里的方向；无法确认任何候选时停止。这样能限制子目标，但不能单独保证转弯途中每只轮子都留在桌内。`HeadingGate.java` 实现这项限制；8毫米是仿真假设，要求场地为凸形、近期编码器误差较小。'
s=s.replace(old,new)
s=s.replace('运行代码以 `training/boundary_heading_cert/java` 中的控制器为优先','当前运行代码在 `training/boundary_motion/java`；其中方向约束与停车源码和 `training/boundary_heading_cert/java` 完全相同')
s=s.replace('powershell -NoProfile -ExecutionPolicy Bypass -File .\\scripts\\build_boundary_heading_cert.ps1','powershell -NoProfile -ExecutionPolicy Bypass -File .\\scripts\\build_boundary_motion.ps1')
s=s.replace('正常窗口记录写入 `data/boundary_heading_runs`','正常窗口记录写入 `data/boundary_motion_runs`')
start=s.index('## 下一步只优化触边运动');end=s.index('## 文件目录和交接材料',start)
s=s[:start]+'''## 已完成的触边运动小模型

网络只输入两个方向：主模型选好的方向、触边前原始方向。输出内侧轮的后退速度与向内转角；权重304字节，普通运行不需要PyTorch。

两个方向不能辨认边线和靠边轮，所以反射率、编码器另负责姿态判断。估计轮轴与边线夹角不超过25度、车头朝外且两轮距边有差别，才使用“靠边轮保持位置、内侧轮后退”。转到朝桌内后，直接接回原目标。其他姿态沿用原控制器直接转弯。角度不明时不强行启用特殊动作。

保持轮使用编码器反馈抵消惯性，仍会有初始位移。网络不重新选择探索区域，真实地图和真实停车目标位置不进入推理。

开发24个场景按完整场景分为16个训练、8个验证；再冻结模型，使用64个新场景配对做完整搜索与停车。全部保持相同起点、目标、种子、电机动力学、主模型和early_fit，只替换运动模块。

| 新64场景完整对照 | 原执行器 | 条件触边模型 |
| --- | --- | --- |
| 正确停车 | 64/64 | 64/64 |
| 平均完成停车 | 203.07秒 | 196.50秒 |
| 平均整轮支撑代理越线时间 | 7.07秒 | 3.22秒 |
| 重心或双轮心越线场景 | 19/64 | 8/64 |
| 最坏轮心外探 | 116.48毫米 | 80.43毫米 |

新场景平均越界减少，但仍有8个严重越界场景。验证集上的搜索均时与最坏外探还增加过，不能承诺每次都更快、更安全。保持阶段最大初始轮位移30.30毫米，电机惯性不能忽略。这是仿真改进，尚不能保证实车在真实桌沿不掉落。

详细研究问题、原理、训练筛选和验收数据见 [触边运动优化报告](触边运动优化报告.md)。下一步固定探索和停车，继续检查边线估计、角落、大角度直接转弯造成的剩余越界。

'''+s[end:]
s=s.replace('| training/boundary_heading_cert/java | 阅读当前控制器与 HeadingGate |','| training/boundary_motion/java | 阅读当前运行代码和条件运动控制 |\n| training/boundary_heading_cert/java | 阅读保留的原执行器、停车与 HeadingGate |\n| models/boundary_motion_candidate | 当前触边小模型权重与训练元数据 |\n| results/boundary_motion_v1 | 本次触边和完整停车配对结果 |')
p.write_text(s,encoding='utf-8')
(R/'当前方案.txt').write_text('''当前保留方案（2026-10-14）

1. 探索主模型：heading_seed4＋近期桌内点方向约束，温度0.5。原权重不变。
2. 触黑框后的定位停车：early_fit几何反馈控制器，原源码不变。
3. 触边运动小模型：现已实现并完成仿真对照，权重304字节。
   models/boundary_motion_candidate/policy.bin，当前入口已启用。
   网络只输入主模型选好的方向、触边前原始方向；输出后退速度和向内转角。
   反射率与编码器判断边线和靠边轮：接近平行、车头朝外、两轮距边不同，才保持靠边轮，让内侧轮后退。
   转到朝桌内后接回原目标；其他姿态直接转，不加这段动作。

新64场景配对完整停车：两版64/64成功，无错误报告完成。
平均搜索183.49→176.12秒；平均总停车203.07→196.50秒。
平均整轮支撑代理越线7.07→3.22秒；重心或双轮心越线场景19→8。
仍有越线，保持轮有初始惯性位移，不能保证真桌沿不掉落。
验证集8/8停车，但均时和最坏外探增加过；新测试64场景不再调整模型。

当前运行代码：training/boundary_motion/java。
HeadingGate、ParkingCore、EdgeTracer与原运行源码哈希完全相同。
原执行器：training/boundary_heading_cert/java；20261013冻结包保持原样。
入口：停车控制台.cmd；唯一决定：configs/CURRENT_SEARCH.json。
报告：docs/触边运动优化报告.md；结果：results/boundary_motion_v1。
下一步继续检查剩余越界：边线估计、角落、大角度直接转弯。固定探索和停车。
旧归档默认不恢复；教师源码保持原样；未烧录实车。
''',encoding='utf-8')
