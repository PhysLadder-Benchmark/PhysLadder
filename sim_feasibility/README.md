# 强物理交互任务：仿真初筛与验证

本目录用于嘉琪负责的仿真任务初筛。任务来源是当前飞书方案的 24 项候选；不调用 GPT、不连接真机，不以模型成功率决定任务是否可模拟。

**本轮结果：**完成 24 项源码与任务审查，初筛 16 项可行性较强、8 项短期困难；四类物体机制及 Franka 动作—RGB 链路已通过基础检查。螺纹接触已测试但尚未验证通过；完整 RoboDojo、R5 适配和 GPT 调用不在本轮完成范围。

## 先看什么

- [任务初筛表](docs/task_screening.md)：逐项列出结论、已有实现、缺口与要验证的内容。
- [实验记录](docs/experiments.md)：本机机制实验的范围、参数和运行结果。
- [进度](docs/progress.md)：已完成与未完成事项。
- [表格 CSV](docs/task_screening.csv)：可导入表格；[JSON](docs/task_screening.json)用于程序读取。

## 环境隔离

| 环境 | 位置 | 用途 |
|---|---|---|
| Windows Conda / Python 3.11 | `D:\conda\envs\isaac51` | Isaac Sim 5.1 及本机接触机制实验 |
| WSL Conda `sim-research` | `/root/miniforge3/envs/sim-research` | 源码核查、USD 静态检查和数据整理 |
| SoftVTBench 原始环境 | 尚未建立 | 原始依赖是 Isaac 4.5 / Python 3.10；不与 5.1 混装 |

代码、下载资产和结果均在本 workspace 的 `sim_feasibility` 下。Conda 本体及环境放在 D 盘独立目录，避免 Windows 长路径问题；没有修改原有 base 的依赖。

## 复现入口

本机环境已经安装。在 Windows PowerShell 中可直接运行：

```powershell
cd D:\mengjq\research_projects\llm_research\sim_feasibility
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_mechanism.ps1 -Kind insertion
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_mechanism.ps1 -Kind friction
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_mechanism.ps1 -Kind spring
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_deformable_probe.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_robot_smoke.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools\run_screw_probe.ps1
```

如需重新安装，入口是 `tools/bootstrap_windows.ps1` 和 `tools/install_isaac51.ps1`；上游资产与源码需按 `results/sources/` 中的记录准备。螺纹脚本输出测量数据，不以退出码 0 表示螺纹验证成功，须阅读实验结论。

安装脚本中的本地代理地址来自本机当前配置；换机器时需要调整或移除，不会修改系统代理设置。运行脚本按安装 Isaac Sim 的授权接受其运行许可。

实验输出位于 `results/experiments/<实验>-<时间>/`：`summary.json`、`trajectory.csv`（机器人为 `joints.csv`）、运行日志、USD 场景和渲染图像。早期调试目录保留；正式引用使用[实验记录](docs/experiments.md)列出的目录。没有结果文件不能引用为通过。

## 证据与目录

- `third_party/`：只读保存的上游仓库，本轮不修改上游代码。
- `assets/`：按需下载的资产；没有下载完整训练集。
- `experiments/`：独立的物理机制验证脚本。
- `results/sources/`：上游提交、资产哈希、任务清单和飞书读取截图。
- `results/system/`：安装日志及环境信息。

原始任务包在 Franka 或 X5 上的复用，与 R5 双臂适配是两件事；当前目录不声称完成了后者。
