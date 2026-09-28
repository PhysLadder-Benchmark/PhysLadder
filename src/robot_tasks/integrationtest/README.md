# 仿真集成校验

这里集中存放真实 API 与 Isaac Sim 的工程集成校验。正式任务仍由 `tasks/` 定义，离线单元测试在 `tests/`。

| 文件 | 职责 |
|---|---|
| `registry.py` | 检查名称、默认／调试配置、实现函数与前置校验的唯一注册位置 |
| `motion_check.py` | 双臂关节、夹爪、同步运动和复位 |
| `cartesian_check.py` | TCP、IK、实际位姿、不可达目标拒绝和恢复 |
| `interaction_check.py` | 抓起方块、搬运、释放；左右臂正例和不闭爪反例 |
| `gpt_api_check.py` | 一次真实 API 图片请求与动作格式检查，不启动机器人 |
| `gpt_check.py` | 参数化抬手检查；执行循环复用公共 runner |
| `gpt_pick_place_check.py` | GPT 单臂抓放全链路入口；复用注册任务、执行循环与独立评分 |
| `camera_check.py` | 三路 RGB、左右腕部独立跟随、同步时间戳及模型输入打包；不调用 API |
| `simulation_process.py`、`simulation_worker.py` | 进程隔离、超时、退出码与结果联合校验 |
| `debug_motion.py` | WSL → Windows 调试连接，F11 进入对应检查 |

从框架根目录运行 `python -m robot_tasks check interaction --gui`。关节与末端检查分别为
`check joint`、`check cartesian`。默认配置由本目录 `registry.py` 指定，可用 `--config` 覆盖；
配置中的任务 ID 必须与选中的检查匹配。普通运行、子进程与 F5 调试共享同一注册表。
配置位于 `configs/checks/`；不需要 ROS、本地 GPT 或真实机器人连接。

交互控制流程是：配置中的已知位姿 → 分段末端目标 → R5 IK → 关节／夹爪位置驱动 → PhysX →
机器人状态与 RGB 回读。独立评分器读取方块真实位姿、速度和双指接触力；这些物体测量不作为视觉识别结果。

抓取期间夹爪受到物体阻挡，可能无法达到空载闭合位置；通用位置反馈显示 `executing` 并不直接意味着抓取失败。
交互评分另行要求双指接触和持续离桌；释放要求夹爪张开、退出、无残余接触以及物体在目标区稳定停留。

禁止在执行期间设置方块位置或把方块固定到夹爪。方块状态只在回合复位时设置，运行中保持自由刚体。
仿真接触力通过 [Isaac Sim 5.1 RigidPrim API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.prims/docs/index.html)
读取，传入物理步长将接触冲量换算为力。当前记录的是控制采样时刻的接触法向力，不覆盖每个物理子步的峰值。

`result.json` 中的 `passed` 表示集成检查符合预期；反例的 `interaction_success=false`、`passed=true`
表示“不闭爪无法抓取”被正确识别。`task_evaluated=false` 表示没有对正式测评任务或模型进行评估。

GPT 校验入口为 `python -m robot_tasks check api` 和 `python -m robot_tasks check gpt --gui`。
前者使用明确标注的测试色块；后者使用真实仿真 RGB，检查抬高左 TCP 20 mm、打开夹爪至 60 mm 和停止。
后者不是接触任务测评；始终保留 `task_evaluated=false`。模型配置在 `configs/models/gpt.toml`，
Token 从 `OPENAI_API_KEY` 读取。物理在请求期间暂停，模型调用耗时单独记录，不算入仿真录像时间。
模型请求、JSON 解析和 RGB 编码等共享实现位于 `policies/`，与这些工程验收入口分开。

`python -m robot_tasks check cameras --gui` 使用真实型号 CAD 和临时相机参数校验三路观测。
`configs/cameras/r5_orbbec_provisional.toml` 内的 `provisional_uncalibrated` 表示内外参未经现场标定。
该配置也由 GPT 集成检查使用；普通关节／抓放检查保留原来的单路工程相机配置。

`check gpt-pick-place` 运行 `GPT-PICK-PLACE` 任务，配置在 `configs/tasks/gpt_single_arm_pick_place.toml`。
它与 `run GPT-PICK-PLACE` 使用同一任务实现，区别是增加独立工作进程和 watchdog，并提供已有 F5 调试入口。
该入口会评估本次抓放（`task_evaluated=true`，无服务／仿真错误时），`passed` 要求真实抓放成功。

共享方块场景和物理判据移到 `tasks/pick_place_scene.py`、`tasks/pick_place_metrics.py`，
公共执行在 `runner.py`，录像编码在 `video.py`。集成检查复用生产代码，不持有另一套抓放实现。
