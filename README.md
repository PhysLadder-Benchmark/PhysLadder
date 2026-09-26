# GPT Benchmark：R5 物理交互测评

基于方舟／ARX R5 双臂、Isaac Sim 和远程模型接口，搭建可记录、可复核的物理交互测评流程。

| 目录 | 用途 |
|---|---|
| [robot_task_framework](robot_task_framework/README.md) | 当前 R5 主项目：三路 RGB、动作控制、GPT 接口、记录和集成校验 |
| [sim_feasibility](sim_feasibility/README.md) | 前期任务可行性调研及独立物理机制实验，保留历史 Franka 实验供查阅 |

已验证 R5 关节／夹爪／末端运动、脚本方块抓放、三路相机和 GPT 动作反馈闭环。
圆柱插孔、双臂夹持力交接仍是任务模板，不能据此报告正式任务成功率。
相机内外参使用明确标记的临时值，未与现场完成标定。

## 首次配置

使用 Python 3.11+。真实仿真在独立的 Windows Isaac Sim 5.1 Conda 环境运行；
轻量单元测试可在 WSL 的 Python 环境运行。安装方法、启动命令与已知边界见主项目 README。

先复制两份本机配置；已有文件时保留原配置：

```powershell
cd robot_task_framework
if (!(Test-Path configs/models/gpt.toml)) { Copy-Item configs/models/gpt.example.toml configs/models/gpt.toml }
if (!(Test-Path configs/runtime/windows.json)) { Copy-Item configs/runtime/windows.example.json configs/runtime/windows.json }
python -m pip install --no-deps --no-build-isolation -e .
```

编辑 `gpt.toml` 填入服务商地址和实际模型 ID。Token 通过 `OPENAI_API_KEY` 环境变量提供。
编辑 `windows.json`，将科研缓存根目录设为自己的 D 盘路径。
WSL 可用 `cp` 代替 `Copy-Item`；只跑离线单元测试不需要真实模型配置或 Token。
VS Code 本机配置不提交，`.vscode/*.example.json` 提供可复制的调试示例。

```bash
cd robot_task_framework
python -m unittest discover -s tests -v
```

Windows Isaac 环境中的相机校验（不调用模型）：

```powershell
.\tools\run_r5_check.ps1 -Check cameras -Gui
```

## Git 中保留什么

保留源码、测试、任务配置、环境定义、调研文档，以及运行所需的 R5 描述和相机 USD。
R5 资产保留厂家来源与文件哈希；相机 CAD 来源和转换说明在 `assets/cameras/README.md`。

缓存、仿真构建产物、录像、逐次结果、机器审计报告、原始相机 STEP、下载依赖、第三方仓库、
本机模型渠道／路径配置和个人调试配置均保留在本地，由 `.gitignore` 排除。
文档中的 `runs/`、`reports/`、`results/` 记录为本机历史证据，不随代码仓库发布；可按文档命令重新生成。

厂家资产沿用其原始来源和使用条款，本仓库不为第三方资产重新授予许可。
