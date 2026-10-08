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

拓境实验室公共场景验收（需先安装锁定的 L20 私有交付包）：

```bash
python -m robot_tasks.integrationtest.tuojing_lab --gui
# 验收后保留窗口供鼠标查看：
python -m robot_tasks.integrationtest.tuojing_lab --gui --hold
```

复用生产 `IsaacSimBackend`、`R5Adapter` 与规划器，检查左右臂关节运动、开爪、
末端上移、URDF/物理位姿一致性、相机随机械臂跟随以及三路同步 RGB-D。
使用 `configs/checks/tuojing_lab.toml`；不调用 GPT、不加载任务道具或评分器。
独立子进程运行，父进程检查 `result.json`，避免将 Kit 的零退出码误读为通过。
证据在 `build/tuojing_lab_check/<时间>/`，包括图像、深度、标定、检查结果和日志。
这项验收不代表完整 L1–L3 任务或真实接触已验证。

批量场景诊断 `scene_smoke.py` 不注册为新的 GPT 用例，不调用模型：

```bash
python -m robot_tasks.integrationtest.scene_smoke --all
python -m robot_tasks.integrationtest.scene_smoke --all --no-preparation
```

短杆螺丝的空间诊断：在规划器环境运行 `python -m robot_tasks.integrationtest.screw_clearance`，检查接近、抓持、半圈旋转与张爪的采样姿态。复用 CAD／URDF 包围盒与数值 IK，只检查关节限位及桌面／孔座间隙；不执行机器人任务，不向 GPT 提供诊断姿态，也不证明实际啮合或连续路径可行。结果默认写入 `runs/screw_short_clearance.json`。

前者验证共同准备动作、场景、RGB-D 和初始评分；后者跳过规划器，只验证物理场景。
加 `--prop-probe` 可对有检查定义的空腔承重、滑轨和螺纹做物理夹具诊断。
`prop_physics.py` 会明确设置诊断初态或施加测试驱动，不能作为机器人任务成功视频或 GPT 成功率。
结果位于 `runs/`，与正式测评分别保存。

L2 原生资产诊断：`python -m robot_tasks.integrationtest.scene_smoke L2-03 --no-preparation --prop-probe`。ID 可换为 L2-01、L2-04、L2-06、L2-07。`native_l2_probe.py` 检查拔帽、抽书、满载碗搬运和原生滑轨；`orange_probe.py` 检查软体接触、压缩及释放。施加的是诊断夹具驱动，不是 GPT 或机器人任务成功。

`python -m robot_tasks.integrationtest.l2_clearance` 在规划器环境检查五个任务的采样抓取／操作姿态，默认输出 `reports/l2_asset_validation/clearance.json`。范围包括关节限位、机械臂／相机／支架对桌面和固定道具的包围盒间隙；不证明连续路径、全身自碰撞或真实抓持稳定性。

局部 R5 接触与执行检查（Isaac Python，不调用模型）：

```bash
python -m robot_tasks.integrationtest.l2_execution_probe L2-07 --poses reports/l2_asset_validation/clearance.json --output reports/l2_protocol_validation/L2-07-motion
```

它使用上一项的显式采样位姿布置张开的夹指，再通过生产控制器闭合、沿目标方向移动 20 mm，持续检查物理状态并保存三视角图片。ID 可换成五个 L2 任务。`--width` 检查不同夹爪开口，`--travel` 调整局部位移（不超过 50 mm），`--velocity-iterations` 隔离求解器参数实验。这些参数只影响工程检查。它不经过视觉选抓点，不是脚本任务参照，不构成完整机器人任务或 GPT 成功证据。正式任务的初态、指令和评分不被该检查修改。

`L3-03` 的加高方瓶还提供独立空间诊断。在包含 NumPy、SciPy、trimesh 的规划器环境运行：

```bash
python -m robot_tasks.integrationtest.bottle_clearance
# 在 Isaac Python 中渲染上一步报告保存的检查姿态：
python -m robot_tasks.integrationtest.bottle_clearance --render
```

检查从瓶身接近、夹持到瓶盖旋转、张爪、退出及回转重抓的 1,420 个采样姿态，使用 URDF 网格包围盒和相机／支架 CAD 尺寸检查双臂互相及相机与道具的间隙；数值 IK 只用于此诊断，不接入生产控制。报告在 `runs/l3_03_square_clearance.json`，截图在同名目录。诊断姿态不替代模型抓持，不构成连续轨迹认证、真实接触稳定性、全身自碰撞或 GPT 成功证据；相机安装参数仍未标定。

笔、纸杯和空顶帽的夹持回归可在 Isaac Python 中串行执行：

```bash
python -m robot_tasks.integrationtest.native_grasp L1-01 --output build/native_grasp/L1-01
python -m robot_tasks.integrationtest.native_grasp L1-08 --output build/native_grasp/L1-08
python -m robot_tasks.integrationtest.native_grasp L1-07 --output build/native_grasp/L1-07
```

检查各目录 `grasp.json` 的 `passed`。该诊断仅在开始时设置机械臂检查姿态，
随后靠真实关节驱动闭合、抬升和释放，检查双侧接触、初始无穿透、物体随动，
以及接触点到原始可见夹指 STL 的距离、生产 RGB-D 抓持验证是否通过；
不会调用 GPT，也不是自主任务成功结果。加 `--check-planner` 可检查从初始姿态
到抓持姿态的规划，诊断的实际夹持仍从重置姿态开始，不代表执行过整条接近路径。
需要 Isaac Python 环境中的 SciPy、trimesh 和 rtree。
拓境夹指使用贴合表面的凸分解，名义开口为原始 URDF 允许的 0–70 mm；
帽圈与杯壁可由竖直夹指从上方夹持，物体原生尺寸与空腔保留。

当前六个 L1 已使用拓境场景。逐项验收可用：

```bash
python -m robot_tasks.integrationtest.scene_smoke L1-03 --prop-probe --output build/tuojing_l1_migration/L1-03
```

将 ID 替换为 L1-01、L1-05、L1-07、L1-08、L1-09。覆盖双臂准备、三路 RGB-D、
初态评分、插座/杯腔/挂杆与刚体玫瑰承托；无需模型令牌。检查 `result.json` 的
`passed`，不能只看 Isaac 进程退出码。无任务执行轨迹或模型成功率含义。

L1-08 稳定夹持与脱手观测回归（不调用 GPT）：

```bash
PYTHONPATH=src python -m robot_tasks.integrationtest.native_grasp L1-08 --hold-s 30 --grasp-style wall --output build/grip_control/wall
PYTHONPATH=src python -m robot_tasks.integrationtest.native_grasp L1-08 --hold-s 30 --grasp-style body --output build/grip_control/body
```

使用 Isaac Python 串行运行。该检查复用生产夹爪控制，诊断复位后抬起并保持 30 秒，
张爪后做 5 mm 竖直诊断试退。它不执行完整套杯任务，也不向 GPT 提供诊断姿态。
检查 `grasp.json` 的 `control_passed`、`stability`、`actual_finger_limits`、
`release_physically_complete` 和 `release_after_probe`。杯身抓法在最大开口下可能仍卡住，
此时整体 `passed=false` 是需要保留的物理释放失败证据，不应通过增大杯口或关闭碰撞掩盖。
`release_after_probe` 只用 RGB-D 与机器人状态作提示；没有接触力、物体真值或评分输入。
遮挡、纹理重复和位移过小可返回未知，`stationary_after_probe` 也不认证承托或任务完成。

L1-08 和 L1-09 的 `stable_gripper_drive=true` 启用软件对称驱动、物理关节限速与闭合误差限制。
`finger_max_acceleration` 单位 m/s²，`finger_closing_lead` 为每指最大闭合领先量，
`finger_effort_limit` 为每指驱动力上限 N。软件同步不代表已确认真实夹爪机械联动。
其它任务默认保持其原有控制方式。新结果记录 `release_check_protocol`，避免与旧测评混淆。

加 `--place-nested` 会在保持检查之后调用生产规划器执行升高、搬运与浅入杯诊断。
规划失败写入 `placement_failure.json` 并使诊断失败；不会关闭碰撞检查或改用直接传送。
该选项使用明确的诊断几何目标，不属于 GPT 自主任务或成功率统计。

L1-09 原生花茎规划与稳定夹持回归（不调用 GPT）：

```bash
PYTHONPATH=src python -m robot_tasks.integrationtest.native_grasp L1-09 --plan-lift --hold-s 30 --output build/rose_grasp/full_lift
```

在 Isaac Python 中执行。沿花头长轴的诊断抓取姿态由复位设定，随后闭爪、规划抬升、保持和释放
使用物理驱动。`--plan-lift` 经过生产规划器的携带物碰撞检查；失败写入 `lift_failure.json`。
`--lift-m` 只改变诊断的抬升距离，不会写入任务提示词或向 GPT 提供抓取坐标。
检查 `grasp.json` 中的 `lift_plan`、`rise_m`、`control_passed` 和 `passed`，不能仅看
`result.json` 的场景检查或进程退出码。长花茎可能在短距离提花头后仍触桌，不能将花头
移动或双指有接触等同于整朵花离桌。

L1-09 静置启动回归（不调用 GPT）：

```bash
PYTHONPATH=src python -m robot_tasks.integrationtest.rose_rest --output build/rose_rest
```

复用生产初始化后，再自由推进 30 秒，检查 `rest_result.json` 与 `result.json` 均通过。
复位姿态使用锁定原生碰撞形状与重心的低势能落座方向：长轴在世界 XZ 平面，
在俯仰和绕长轴转角中最小化承托后的重心高度，最后给最低表面 0.01 mm 初始净空。
它只是场景的初始布置，不是抓取位姿或动作脚本。原资产质量、惯量、摩擦、阻尼未改。
生产入口最多等待 120 秒仿真时间，要求连续 2 秒线速度低于 1 mm/s、角速度低于
0.015 rad/s、位置漂移不超过 0.5 mm、完整姿态角变化不超过 0.01 rad，且由桌面承托、
没有夹指接触；不满足时在调用模型前报错，不锁定物体或强行清零速度。
这项启动检查使用仿真测量，只决定场景何时就绪，不将物体真值交给 GPT。
