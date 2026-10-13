# 重复停车项目

本项目在课程提供的 Processing 数字孪生中，让小车从未知起点探索场地、找到停车标记，再定位并停稳。当前维护者负责 motor 电机子系统。我们已固定探索与停车模块，下一步只优化触边后的轮子运动。

**新接手者先读 [项目交接说明](docs/项目交接说明.md)。** 可直接分享的PDF在 [交接文档](output/pdf/重复停车项目交接说明.pdf)。

## 当前三个模块

| 模块 | 状态 | 作用 |
| --- | --- | --- |
| heading_seed4 探索主模型和桌内方向约束 | 已训练并保留 | 用访问记忆与持续子目标选择探索区域，触边后在可行方向中选择 |
| early_fit 定位停车控制器 | 已实现并保留 | 触黑框后提前定位、确认方向、按需扫描并验证停车 |
| 触边运动小模型 | 尚未实现 | 接收当前航向和主模型目标方向，优化左右轮回退与转向 |

**触边**是检测到场地边界；**触黑框**是首次检测到停车标记黑线。前者调整探索，后者开始定位停车。

当前选择只看 [CURRENT_SEARCH.json](configs/CURRENT_SEARCH.json)，模块定义看 [RETAINED_MODULES.json](configs/RETAINED_MODULES.json)。第二个模块是几何反馈控制器，没有独立神经权重；第三个模块没有已完成的模型文件。

## 快速运行

当前启动脚本支持Windows。需要Processing 4.5.7及其JDK、core和net库放在 `E:\Processing`，并使带有 `tkinter` 的Python能通过 `python` 命令运行。普通运行和当前校验不需要PyTorch。

1. 将项目解压到 `E:\SEMTM0043\RepeatableParking`，确认根目录直接包含 `configs`、`scripts` 和 `models`。
2. 双击 **停车控制台.cmd**，启动器会校验权重并编译当前代码。
3. 在矩形场地窗口中设置小车和目标，点击开始。默认3米，正式对照覆盖1至4米正方形。
4. 查看停车结果和保存的回放。其他尺寸、长宽比属于外推测试。

项目路径多数由脚本自动推导，Processing路径仍有多处写死。更换电脑时先确认路径，再运行校验。

在项目根目录启动2米窗口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\current_demo.ps1 -Size 2
```

`-Seed` 设置初始预览；开始按钮可能按随机选项重新生成位置和种子。严格复现请使用保存的配置和校验脚本。

## 当前验收

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\build_boundary_heading_cert.ps1
python .\scripts\verify_current.py
```

该脚本核对权重、方向测试、参考推理一致性、窗口构建、单场景停车和完整回放；新结果写入 `results/retention_20261013/check_*`，不覆盖历史正式结果。

## 已有结果和范围

| 实验 | 样本及结果 | 时间含义 |
| --- | --- | --- |
| 当前探索组合的新场景对照 | 64/64找到黑框，平均190.67秒 | 起步到首次触黑框，排除停车 |
| early_fit历史标称局部验证 | 64/64正确停车，平均19.91秒 | 首次触框到停车 |
| 2026年10月13日入口衔接检查 | 1场景停车成功，4597帧回放完整 | 28.21秒触框，45.97秒完成停车 |

这三项来自不同实验，不能把前两项均时相加当作实测总时间。当前方案由验证集选定，不是全局最优证明。

635次边界方向决策审查中错误方向为0，但64场景中仍有60条出现整轮支撑代理越线、28条出现重心或双轮心越线。因此下一步研究轮子实际转弯轨迹。当前是线外仍有支撑的虚拟边界仿真，未做实车标定或烧录。

## 主要文件

- `models/boundary_heading_selected/policy.bin` 和 `policy.pt`：当前主模型。
- `training/boundary_heading_cert/java`：运行时优先使用的控制器、方向约束和测试。
- `DigitalTwinParkingPrecision`：课程仿真模型适配与精确累计。
- `results/boundary_heading_v2/final_compare`：探索对照和方向审查依据。
- `results/unlimited_optimization/RESULTS.txt` 与 `data/local_opt`：局部停车依据。
- `data/boundary_heading_runs`：本机新运行的配置、压缩轨迹和摘要。

历史实验位于本机 `archives/20261013_cleanup`，2026年10月13日的冻结包为 `delivery/retained_modules_20261013.zip`。轻量分享版本不附大体积历史归档或缓存；旧报告中的旧路径需要从归档恢复，本README和当前配置代表现行入口。

## 下一步

固定探索权重及 `early_fit`，只开发触边运动小模型。用相同接触姿态、轮速和目标方向，对比现有执行器与候选模型，先检验越线深度、越线时长和恢复时间，再确认完整搜索及停车成功率。接口和对照设计见交接说明。

## 来源

[课程 motor 指导](https://semtm0043-42.github.io/motors.html) · [课程原始代码](https://github.com/paulodowd/RoboticsScienceAndSystems)

探索网络、方向约束和三模块划分是本项目工程选择。课程原始文件和历史评测记录保持各自来源；不要将本项目仿真成绩当作实车验证结果。
