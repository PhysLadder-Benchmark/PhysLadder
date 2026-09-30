# GPT 抓放集成检查

这里只保留 `gpt-pick-place` 一个用例，直接复用 `GPT-PICK-PLACE` 的场景、GPT 策略、执行循环和评分。

```bash
python -m robot_tasks check gpt-pick-place --gui
```

默认配置为 `configs/checks/gpt_single_arm_pick_place.toml`，它引用公共 workflow。
支持 `--config`、`--model-config` 和 `--output`；输出仍统一放在本轮 `runs/` 目录。
该检查会启动 Isaac Sim 并真实调用模型；只有实测任务成功才返回通过。

| 文件 | 职责 |
|---|---|
| `gpt_pick_place_check.py` | 唯一用例，将任务实测结果转为集成检查结果 |
| `registry.py` | 检查名称、配置和入口解析 |
| `simulation_process.py` | 独立进程、日志、超时监控与退出处理 |
| `simulation_worker.py` | 子进程入口 |
| `debug_check.py` | WSL 编辑器连接 Windows Isaac Python 的调试入口 |

后四项是启动设施，不是额外测试场景。新任务通过主任务注册表和 `run <任务ID>` 运行，
不需要复制这里的抓放用例。离线软件回归保留在根目录 `tests/`，不启动真实仿真或模型请求。

批量场景诊断 `scene_smoke.py` 不注册为新的 GPT 用例，不调用模型：

```bash
python -m robot_tasks.integrationtest.scene_smoke --all
python -m robot_tasks.integrationtest.scene_smoke --all --no-preparation
```

前者验证共同准备动作、场景、RGB-D 和初始评分；后者跳过规划器，只验证物理场景。
加 `--prop-probe` 可对有检查定义的空腔承重、柔性杆、滑轨和螺纹做物理夹具诊断。
`prop_physics.py` 会明确设置诊断初态或施加测试驱动，不能作为机器人任务成功视频或 GPT 成功率。
结果位于 `runs/`，与正式测评分别保存。
