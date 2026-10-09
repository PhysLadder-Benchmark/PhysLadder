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

它使用上一项的显式采样位姿布置张开的夹指，再通过生产控制器闭合、沿目标方向移动 20 mm，持续检查物理状态并保存三视角图片。ID 可换成五个 L2 任务。`--width` 检查不同夹爪开口，`--travel` 调整局部位移（通常不超过 50 mm，橘子提起检查允许到 80 mm），`--velocity-iterations` 隔离求解器参数实验。这些参数只影响工程检查。它不经过视觉选抓点，不是脚本任务参照，不构成完整机器人任务或 GPT 成功证据。正式任务的初态、指令和评分不被该检查修改。

`L3-03` 的原生东方树叶瓶由左臂扶持、右臂取盖；空间采样诊断检查一组扶瓶与抓盖姿态的可达性及双臂、相机支架间距。在包含 NumPy、SciPy、trimesh 的规划器环境运行：

```bash
python -m robot_tasks.integrationtest.bottle_clearance
# 在 Isaac Python 中渲染上一步报告保存的检查姿态：
```

使用 URDF 网格包围盒和相机／支架 CAD 尺寸检查双臂互相及相机与道具的间距，只排除预期的扶瓶夹指—瓶身接触。数值 IK 只用于此诊断，不接入生产控制；结果保存在 `reports/native_bottle_clearance.json`。这不是完整轨迹、自碰撞认证、实际夹持或 GPT 成功证明。旧方瓶的 1,420 姿态报告不适用于现行东方树叶瓶。

当前六个 L1 已使用拓境场景。逐项验收可用：

```bash
python -m robot_tasks.integrationtest.scene_smoke L1-03 --prop-probe --output build/tuojing_l1_migration/L1-03
```

将 ID 替换为 L1-01、L1-05、L1-07、L1-08、L1-09。覆盖双臂准备、三路 RGB-D、
初态评分、插座/杯腔/挂杆与刚体玫瑰承托；无需模型令牌。检查 `result.json` 的
`passed`，不能只看 Isaac 进程退出码。无任务执行轨迹或模型成功率含义。

L2-06 物性对照沿用 `l2_execution_probe`：`--soft-physics <JSON>` 只覆盖本次诊断的 `[orange_physics]` 参数，`result.json` 保存原生值和实际生效值。固定抓取姿态和评分条件，对比闭合、抬升；当前 `--travel 0.075 --finish` 检查承台上夹起 5 cm，不再搬运释放；源资产和正式任务配置不被诊断改写。工程夹具检查不调用 GPT，也不证明自主选抓点成功。

## L3 新道具接触检查

所有诊断都不调用模型，结果不要并入 GPT 成功率。`initial_gallery` 检查场景、RGB-D 和未完成初态；运行它可查看新摆放。

```bash
python -m robot_tasks.integrationtest.initial_gallery L3-11 --output reports/l3_assets_20261008/L3-11
python -m robot_tasks.integrationtest.l3_asset_probe L3-05 --output reports/l3_assets_20261008/L3-05-contact
python -m robot_tasks.integrationtest.l3_asset_probe L3-11 --output reports/l3_assets_20261008/L3-11-contact
python -m robot_tasks.integrationtest.l3_asset_probe L3-15 --output reports/l3_assets_20261008/L3-15-contact
```

`l3_asset_probe` 只做隔离部件诊断：螺丝用额外同轴导向和外力矩验证真实旋转推进；板擦通过测试速度和单独的抓持标志检查接触擦除，生产评分的未抓持状态仍不得擦除；簸箕预置方块验证承托/篮内判定，不代替扫入过程。正式任务中没有这些诊断导向、速度或标志。

规划器 Python 可运行 `screw_clearance` 和 `l3_tool_clearance`，检查若干夹爪/相机包围盒候选姿态与固定物的间隙；它们不生成供 GPT 使用的目标，不证明连续无碰撞轨迹或稳定抓持。

当前 L3-15 接触检查使用固定簸箕，只注入用于检验篮内范围与承托的方块初态；不代表扫入成功。L3-03 的 `l3_asset_probe` 使用临时同轴夹具和有限扭矩/拉力校验盖的物理退出，不会进入生产场景。

### L3-05 原生松螺母（2026-10-09）

当前任务是双臂旋下预松螺母，旧的粗螺丝旋入诊断不再作为任务入口。

```bash
python -m robot_tasks.integrationtest.nut_clearance --output reports/nut_clearance.json
python -m robot_tasks.integrationtest.nut_probe --output reports/nut_thread
python -m robot_tasks.integrationtest.nut_grasp_probe --poses reports/nut_clearance.json --output reports/nut_grasp
```

依次验证采样净空、受限夹具中的真实螺纹旋出、预置机器人姿态下双指夹持与 10 mm 搬动；均无 GPT，不代表完整任务成功。只在独立诊断中设置夹具和机器人初态，正式任务不调用它们。
