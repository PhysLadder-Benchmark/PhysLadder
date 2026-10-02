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

修改后的 L2 布置可运行 `python -m robot_tasks.integrationtest.scene_smoke L2-03 --prop-probe`，将任务 ID 分别替换为 L2-04、L2-05、L2-07。
`l2_prop_probe.py` 检查固定夹层抽取、预装小球碗的缓慢搬运、卡扣解锁与盒盖后限位停靠、带顶板抽屉的开启支撑；施加的是诊断夹具驱动，不是模型操作。
`python -m robot_tasks.integrationtest.l2_clearance` 在规划器环境检查三个单臂任务的采样抓取／操作姿态，默认写入 `runs/l2_layout_clearance.json`，可用 `--output` 改变位置。
范围包含诊断 IK、机械臂各连杆及末端／相机／支架包围盒与桌面、夹具的采样间隙；不代表连续路径、自碰撞、双臂互碰或抓持稳定性认证。

`L3-03` 的加高方瓶还提供独立空间诊断。在包含 NumPy、SciPy、trimesh 的规划器环境运行：

```bash
python -m robot_tasks.integrationtest.bottle_clearance
# 在 Isaac Python 中渲染上一步报告保存的检查姿态：
python -m robot_tasks.integrationtest.bottle_clearance --render
```

检查从瓶身接近、夹持到瓶盖旋转、张爪、退出及回转重抓的 1,420 个采样姿态，使用 URDF 网格包围盒和相机／支架 CAD 尺寸检查双臂互相及相机与道具的间隙；数值 IK 只用于此诊断，不接入生产控制。报告在 `runs/l3_03_square_clearance.json`，截图在同名目录。诊断姿态不替代模型抓持，不构成连续轨迹认证、真实接触稳定性、全身自碰撞或 GPT 成功证据；相机安装参数仍未标定。
