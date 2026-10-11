# GPT Benchmark：R5 物理交互测评

本项目使用 **ARX／方舟 R5 双臂平台**。已接入 17 个 L1—L3 仿真任务，以及独立的 GPT 单臂抓放工程检查。任务共用模型接口、三路 RGB（可选深度）、相机参数和图像标记、cuRobo 轨迹执行、录像及独立评分。

任务配置位于 `configs/tasks/`，每项包含英文 prompt、中文道具与布置说明、手臂角色和启动命令。橘子材料参数、弹簧卡扣、商业插头/钥匙、东方树叶瓶盖和扫帚采用明确标注的工程代理，不等同实物材料与损坏模型。L3-11 已实现原生板擦＋固定绿板的接触清洁代理，与湿毛巾真机任务不同。代码接入或场景检查通过不代表 GPT 已完成任务。

使用 `--gui` 启动时，窗口自由视角从当前任务实际加载的 overview 相机复制世界位姿和镜头参数；L1/L2 与 L3 分别按自己的场景坐标初始化，不再单独配置 GUI 的 `camera_position` / `camera_target`。窗口使用独立的 `/World/GUIView`，鼠标调整视角不会移动 GPT 输入或录像相机，也不会在每个控制步被重置。无相机配置的旧工程场景仍使用原有位置/目标参数作为回退。

## 核心设计原则：以观测验证驱动任务推进

**每个动作都应有可观察的预期结果；只有最新观测支持这一结果，模型才应进入下一阶段。** 本项目将这一原则写入所有任务的公用 prompt：先理解目标、简要拆解任务，再循环执行“观察 → 决策 → 动作 → 核实 → 必要时纠错”。计划随物理结果调整。

| 环节 | 模型需要做什么 |
|---|---|
| 理解与拆解 | 明确最终状态、过程约束和手臂角色，拆成适合当前任务的简短阶段；第一次 `note` 简述计划，可同时执行有用的观察或动作。 |
| 执行前检查 | 根据最新图像、深度、机器人状态和执行反馈，确认当前阶段的前提成立，例如物体仍被持握、工作端与目标对准。 |
| 执行与核实 | 选择一个结果可检查的动作，下一次决策先比较预期与实际；指令已下发或机械臂已到位，均不能单独证明物体操作成功。 |
| 失败后纠错 | 指出具体不一致，重新观察并修正抓点、对准或路径。连续规划拒绝后重新定位和检查净空，避免重复同一失败动作或不断扩大绕路范围；规划失败本身不能证明某处存在障碍。 |
| 最终确认 | 检查任务要求的放置、释放、支撑和稳定状态，再申请停止；最终成功由独立评分器判定。 |

每次 `note` 用简短英文记录**当前阶段、观测依据或偏差、下一动作及预期结果**，沿用 512 字符上限。文本历史保留已确认进展和失败尝试，但旧的模型描述只是待核实的判断；场景重置后必须重新确认抓持、位置与任务进展。无需每次重写长计划，也不统一强套“抓取—插入—释放”到所有任务。

例如 L1-01 插笔：先抓取并试抬确认笔随手移动，再估计笔相对夹指的偏移，使**笔下端对准笔筒内部开口**。确认笔身与夹指能避开筒沿后，小步垂直下放；笔的一端真正进入主杯腔且无明显穿模，即满足当前 L1-01 的完成要求；允许继续夹持，不要求触底、松手或等待，也不设置固定 4 cm 深度门槛。TCP 到了笔筒附近、笔悬在筒口上方或碰到筒沿，都不足以判断完成。

公用规则位于 [operating_notes.py](src/robot_tasks/policies/operating_notes.py)，由 [gpt.py](src/robot_tasks/policies/gpt.py) 注入模型指令；各任务 TOML 补充目标、道具和操作边界，任务评分器独立实现成功判据。当前 prompt 协议版本为 `shared_observe_plan_verify_task_goal_v6`。阶段拆解与纠错是模型遵循的提示规则，并非执行器已强制验证所有阶段的状态机；其效果仍需通过实验评估，也不能替代定位、控制和碰撞模型的正确性。

公共规则在每次模型请求中自动加入，覆盖全部 16 个可运行仿真任务及 GPT 抓放工程检查，无需在各 TOML 重复粘贴。任务要求优先决定是否释放：钥匙插锁允许继续握持；推三角块使用夹爪直接推动，无额外退开判据；扫块完成后不要求继续握持两件工具。柔性物体允许逐步闭爪。机械臂从全零关节姿态开始，不再预先朝下或自动把观察臂转向工作区；后续姿态由模型决定。L3-11 右臂持板擦、左臂观察，擦除至少 90% 的初始字迹即完成。

## Windows / Linux 共用运行方式

共享实验配置保持一致；本机只配置 Python、缓存和输出目录。启动器需要 Python 3.11+，
使用已有 Isaac Sim 环境，不自动安装依赖或更改驱动。Windows 默认 D3D12，Linux/WSL 默认 Vulkan。
这里按实际运行 Isaac Sim 的 Python 所在系统选择；从 WSL 经 PowerShell 启动 Windows Python 时仍默认 D3D12。

复制 `configs/runtime/local.example.toml` 为 `configs/runtime/local.toml`（已被 Git 忽略），
填写本机 Isaac Sim Python 路径，或者激活该环境并保留 `python = "current"`。
Windows 路径推荐使用正斜杠，例如 `D:/conda/envs/isaac51/python.exe`。
相对路径以本机配置文件所在目录为基准；配置中不得放实验参数、模型参数或密钥。

在仓库根目录运行（Linux 用 `python3`，Windows 用 `python`）：

```bash
python3 tools/launch.py list --checks
python3 tools/launch.py check gpt-pick-place --gui
```

唯一集成检查为 `gpt-pick-place`，需要有效的模型配置及对应登录或 API 凭据。
去掉 `--gui` 则后台运行。`--output` 可覆盖本机默认输出目录。

任务运行时，模型请求在独立线程中执行；GUI 主线程在等待请求和重试退避期间继续刷新，
可以切换和移动显示视角。等待期间不推进物理仿真，也不重新采集模型输入。
按 Ctrl+C 或关闭仿真窗口会取消待处理的模型请求，并清理其 Codex 子进程。
场景初始化和运动规划仍同步执行，耗时较长时界面可能暂时停顿。

Windows 普通检查也支持原来的 `tools/run_r5_check.ps1 -Check gpt-pick-place -Gui`；显式 `-TaskPython`
优先，没有新配置时保留旧默认 Python 路径。未显式配置缓存目录时，现有 Windows
`windows.json` 仍有效；Linux 不依赖这个文件。调试入口继续采用原 Windows 调试流程。
示例中的 `cache_dir = "auto"` 也属于显式配置，会改用系统用户缓存目录。
需要保留旧 D 盘缓存布局时，省略该字段；需要新的布局时填写自己的 D 盘缓存目录。

所有任务默认使用仓库内唯一的 [configs/models/gpt.toml](configs/models/gpt.toml)。
当前为 `codex_cli`、`gpt-6-astra`、`medium`、普通速度，单次请求超时 9000 秒，最多 100 次调用。
思考强度直接读取模型文件，公共 workflow 不再暗中覆盖；单任务和批量运行均无需传 `--model-config`。
需要 Responses API 时，在同一文件将 `transport` 改为 `"responses"`，填写服务地址与模型 ID，
并在环境变量中设置 `OPENAI_API_KEY`。配置中只保存环境变量名，不写入 Token。
临时对照实验仍可显式指定其他模型文件：

```bash
python3 tools/launch.py run GPT-PICK-PLACE --model-config /path/to/model.toml --gui
```

跨平台入口共用任务配置、控制、评分和录像实现。`transport="responses"` 使用 API 凭据；
`transport="codex_cli"` 通过本机 Codex CLI 已保存的 ChatGPT 登录进行模型决策，无需 API Key。
该方式每次启动独立的模型决策调用，不继承当前聊天的上下文；任务所需历史由项目显式提供。
不会读取、复制或保存 Codex 的登录令牌，也不会自动退回 API Key 或切换模型。

```bash
codex login status  # 应显示 Logged in using ChatGPT；未登录时运行 codex login
python3 tools/launch.py run L1-01 --gui
python3 tools/batch_run.py --all --gui
```

需要 Ultrafast 时，在同一份 `configs/models/gpt.toml` 中启用这行配置：

```toml
service_tier = "ultrafast"
```

每项任务仍采用公共的 100 次调用上限。`service_tier` 控制请求速度档位，独立于思考强度；
模型配置支持 `fast`、`ultrafast`，省略该字段时保留原调用行为。
项目会把速度档位显式传给每次 `codex exec`（同时启用 `features.fast_mode`），
不依赖被 `--ignore-user-config` 排除的用户设置；Responses API 也转发此字段。
运行元数据记录的是**请求档位**，不能作为服务端实际速度或权限已获确认的证据。
按 [OpenAI Docs](https://learn.chatgpt.com/docs/agent-configuration/speed)，
Codex Ultrafast 支持 Pro $500 和符合条件的 Enterprise／Edu；
GPT-6 Astra 的包含额度按标准模式的 8 倍速率消耗，实际端到端耗时仍需测量。

Windows Isaac 默认优先查找 Windows Codex；找不到时通过 WSL 调用已登录的 Codex。
需要固定 WSL 发行版和用户时，将 `configs/runtime/codex.example.toml` 复制为
`codex.local.toml`，设置 `host="wsl"`、`wsl_distribution` 和 `wsl_user`。
WSL 只需 Python 3.10+ 和 Codex，无需安装 Isaac 或另存登录令牌。若跨系统启动后缺少代理，
在这份本机文件中设置 WSL 可访问的 `wsl_proxy`。Linux 直接调用本机 Codex。

从 WSL 使用 Windows Isaac 的示例：

```bash
powershell.exe -NoProfile -Command '& "D:/conda/envs/isaac51/python.exe" tools/launch.py run GPT-PICK-PLACE --gui'
```

共享任务配置引用 `configs/models/gpt.toml`，该文件不含密钥并随仓库提供，无需复制模板。
临时图像和指令保存在项目 `build/codex/`，调用结束即清理。跨系统取消会先通知
WSL 结束本次 Codex 进程组；WSL 内部也有独立超时，避免只结束 Windows 启动器。

默认配置请求 `gpt-6-astra`、`medium`。需要能连通 ChatGPT 的网络，
终端可能需要继承桌面代理，例如 `export HTTPS_PROXY=http://127.0.0.1:<本机代理端口>`。
CLI 使用独立临时目录、只读沙箱、JSON Schema 和显式图像编号，只允许返回动作决策。
其 JSONL 协议不提供服务端实际返回的模型 ID；日志标记模型验证方式为显式 CLI 配置，
`response_id` 是 Codex thread ID。`model_calls` 统计决策调用，不等同于底层 HTTP 请求数；
Codex 内部连接重试也占用每次决策的超时预算。CLI 报告的 token 使用量保存到原有结果中。
抓放任务及其集成入口共用执行与记录模块，
通过 `run` 或 `check` 启动均保存 `runtime_config.json` 和 `storage_config.json` 环境快照；
`config.toml`、结果和日志记录实验，不记录 API 密钥。

Isaac 启动器保留两个可选环境覆盖：`GPT_BENCHMARK_PYTHON` 指定解释器，
`GPT_BENCHMARK_RUNTIME_CONFIG` 指定另一份本机配置。原来的 `doctor` 命令、
运行配置里的模型/图形后端/重试字段及 `--runtime-config` 全局选项已移出最小版本。

软件回归在 Windows/Ubuntu CI 执行；真实 GPU 仿真仍需双方各自验证。
双方比较实验时应使用一致的实验配置、模型设置和兼容的 Isaac Sim 版本。

## 公共运动执行依赖

所有任务使用同一个 cuRobo V2 规划器，没有 Lula 或直接 IK 的静默回退。GPT 选择 TCP 目标，规划器生成关节轨迹，执行层限制每步关节变化和参考速度，并读取实际执行结果。单动作超时会取消排队动作。

在 Linux / WSL 使用 `bash tools/setup_curobo.sh /你的科研盘/runtime` 建立隔离环境。复制 `configs/runtime/planner.example.toml` 为 `planner.local.toml`，填写该环境的 Python；Windows Isaac 会通过 WSL 启动它，原任务启动命令不变。也可以用 `GPT_BENCHMARK_PLANNER_PYTHON` 临时指定。机器路径不进入共享任务配置。

规划器读取桌面、静态夹具和另一只 R5 的碰撞几何；持物时使用双指实测接触和物体表面采样检查搬运净空。这是**规划层使用仿真真值的评测条件**，不会把物体坐标、目标位姿或评分提供给 GPT，也不在物理引擎中绑定物体。机器人球体近似、表面采样、未夹持活动物体的接触豁免，以及相机外壳不参与碰撞，意味着它不是完整的碰撞安全证明。正常就位允许 1 mm 的表面接触容差。

夹爪与轨迹执行语义参考 [RoboProbe EEF Agent 操作说明](https://github.com/RoboProbe/RoboProbe/blob/main/policy/RoboDojo_Agent_L3_Inspect_EEF/docs.py) 和 [执行代码](https://github.com/RoboProbe/RoboProbe/blob/main/policy/RoboDojo_Agent_L3_Inspect_EEF/policy.py)；本项目采用 R5 与 cuRobo V2，不是该基准的完整复现。

L1 使用独立的 `r5_tuojing_lab_l1.toml`，名义夹爪开口 0–70 mm，并采用 master 的薄物体夹指碰撞配置；L2 和常规 L3 保留原来的 12–70 mm，L3-05 仍为 10–70 mm。L2／螺母任务的碰撞参数不被 L1 覆盖。第三视角默认沿用公共配置，L1-03 保留真机插头实验专用构图；L1-08/09 的稳定夹爪驱动只在这两项启用。

夹爪反馈区分目标到位、闭合受阻或停滞、运动未完成。受阻不会自动证明抓取成功；模型仍需短距离抬升、观察物体跟随，必要时移动观测臂。失败的停止请求会返回“尚未确认完成”，继续使用同一场景、历史和剩余总预算；日志保留每次被拒绝的停止。

模型接口报告可重试的生成失败／未完成，或发生调用超时（Codex 决策超时、API 连接／读取超时或请求时间预算耗尽）时，公共运行流程固定允许首次请求后再重试 3 次，间隔 2、4、8 秒，无需新增配置。每次重试计入同一个 100 次总调用上限，保留原观测、场景和已接受历史，不执行部分响应，也不推进仿真。每次尝试仍使用模型配置的 `timeout_s`，Codex 超时进程先清理再重试。成功后重置连续重试计数；连续四次失败或总调用预算不足时按接口异常结束，不冒充任务失败。明确的鉴权、额度、拒绝及请求参数错误，以及 HTTP 错误状态和其他非超时网络错误不走这项重试。每次请求、重试进度与脱敏后的服务端原因记入运行日志，事件语义参考 [OpenAI Docs](https://developers.openai.com/api/docs/guides/streaming-responses)。

开启 `--depth` 后，抓放与 L1 共用基于深度的抓取核验（默认 RGB 模式不启用此拦截）：接近与闭爪分开决策；闭爪保存原始 RGB-D 基准帧，之后给模型提供三路“闭爪前／当前”对照图。基于已公开的 TCP、夹爪尺寸、桌面高度和 RGB-D，检查两指之间的可见表面是否随抬升移动。未通过时只允许保持、观察臂移动、单独开爪恢复，以及相对闭爪位置累计不超过 4 cm 的竖直试抬（水平偏移不超过 5 mm），拦截搬运和插入；达到最小开口会提示可能空夹。模型仍决定全部实际动作。

该核验不读取物体真值、接触力或评分，也不调用额外模型；使用表面几何、颜色匹配和抬升前后的可见空间变化，避免把手的移动或模型自述当成物体移动。遮挡、细小／反光／透明物体、相似表面和夹爪几何近似会造成误判或无法确认，核验通过不代表任务成功。当前适用于需要先抓持抬升的任务，不能直接当成不允许抬升的抽屉等任务协议。核验拦截、参考帧和通过次数进入同一运行目录，正式比较应注明 `rgbd_surface_comotion_v3_tracking_recovery` 条件。

## 任务池与运行

| 层级 | 已接入仿真任务 | 手臂角色 |
|---|---|---|
| L1 | L1-01 插笔、L1-03 插头 | 初始 TCP 更靠近操作物体的臂操作，另一臂主动观察；当前布局均为右操作、左观察 |
| L1 | L1-05 钥匙、L1-07 挂帽、L1-08 叠杯、L1-09 刚体玫瑰插花 | 左臂操作，右臂主动观察 |
| L2 | L2-01 拔笔帽 | 双臂操作 |
| L2 | L2-03 固定上下层抽书 | 书堆居中，右臂侧向抽取，左臂主动观察 |
| L2 | L2-06 承台橘子提起 5 厘米 | 左臂操作，右臂主动观察 |
| L2 | L2-04 右杯向左杯倒珍珠、L2-07 画面右侧固定化妆抽屉 | 右臂操作，左臂主动观察 |
| L3 | L3-03 扶瓶旋下松盖 | 左臂扶瓶，右臂旋盖 |
| L3 | L3-05 预松螺母旋出 | 左臂拿稳主体，右臂旋下螺母 |
| L3 | L3-14 推三角块、L3-15 两块扫入固定簸箕 | 右臂操作，左臂主动观察 |
| L3 清洁代理 | L3-11 板擦擦绿板 | 右臂操作，左臂主动观察 |

```bash
python3 tools/launch.py list
python3 tools/launch.py inspect L1-01
python3 tools/launch.py run L1-01 --gui
# 把 L1-01 换成上表任意仿真任务 ID。
```

首版采用项目内程序生成的道具，不依赖下载 RoboDojo/ManiSkill 资产；厂家 R5 及相机资产仍需按安装说明准备。默认 medium、最多 100 次模型调用。L1 详细定义见 [L1 任务说明](docs/l1_tasks.md)。每个任务的工程代理边界在 TOML 和运行记录中保留。

`L3-03` 使用东方树叶 335 ml 原生瓶身、瓶盖和贴图，瓶身可自由运动；盖预松 7.5 mm／300°，保留末段螺纹啮合。左臂扶住瓶身中下部，右臂旋下盖子，放到桌面并释放；瓶身可继续由左臂扶持，也可放稳后释放。不要求转满一圈、固定摆放点或静止等待，不测试破坏防盗环或开启紧盖力矩。[任务配置](configs/tasks/l3_03_bottle_cap_unscrew.toml)。

`L3-14` 使用原尺寸 32×17×10 mm 蓝色三角块，目标为约 200×180 mm 的绿色三角线框。右夹爪实际推动后，整个块进入区域并由桌面承托即完成；不限制朝向、精确中心或额外等待。从第三视角看，块在目标右上方，两者尖端均朝机械臂；目标线框与评分使用同一三角形，不能用夹起搬运代替推动。[任务配置](configs/tasks/l3_14_gripper_push_triangle.toml)。

`L3-15` 的簸箕固定，薄唇位于桌面；按第三视角，簸箕在左、手刷在右，两块位于中间；右臂持原生手刷将 2 块扫入篮内，左臂只观察。不再要求持簸箕或最后仍握两个工具；2 块在篮内且由簸箕承托即可。[任务配置](configs/tasks/l3_15_bimanual_sweep_blocks.toml)。

L2 已切换到锁定的 L20 资产和拓境实验室坐标。L2-01 使用可拔帽马克笔，双手从桌面拿起后分离；L2-03 使用原尺寸 220×140×20 mm、378.9 g 的书，上下固定、中层向画面右侧（世界 +X）伸出 40 mm，夹其 20 mm 厚度沿 +X 抽出约 110 mm；书堆与 70 mm 高的蓝色支撑盒居中放在双臂之间，由右臂从突出侧接近并抽取、左臂观察，布置参考真机抽书视频，尺寸和位置为估计值。L2-04 已替换为固定接收杯的纸杯间倒珍珠，详见下文。L2-07 将双抽屉收纳盒放在第三视角画面右侧、靠近右臂，右臂操作上层抽屉、左臂观察；原生轨道总行程 90 mm，初始预开 30 mm，继续拉至相对全闭位置的总开度至少 75 mm 后松手（从初态再拉至少 45 mm）。原生把手约 102×6×8 mm，不再添加蓝色抓持片或额外质量。原把手与 R5 的 12—70 mm 开口存在对夹限制，抓取可行性尚待验收，不能沿用加装把手时的验证结果；没有物体与机器人之间的隐藏绑定。

L2-06 当前为“从圆柱承台夹起橘子瓣 5 厘米”（`soft_pedestal_lift_v6`）：左臂抓起原生 GPU FEM 橘子，整体中心相对回合初始静置位置上升至少 50 mm、已离开承台且仍被左夹爪夹持即完成，不要求搬运、释放或额外等待。固定承台高 130 mm、位于双臂中间；橘子宽面平放，长轴沿世界 Y、第三视角画面纵向。原来的绿色目标区已移除。`[orange_physics]` 保持 100 kPa 弹性模量，将动摩擦从 0.9 提高到 2.0，保留原生网格、尺寸与质量。这些是便于抓持的工程参数，未做真实组织标定，覆盖值写入 `scene.json`。形变与局部 FEM 反转只记录，不作为夹坏阈值。新任务与旧搬运回合分别统计；命令仍是 `python tools/launch.py run L2-06 --gui`。

L2-04 复用两只原生波纹纸杯，装入 200 颗 8 mm 珍珠，初始料面约占杯腔高度的 48%，装料杯总质量约 75.8 g。接收杯固定在桌面，不滑动、不倾倒；源杯保持可抓取和倾倒。纸杯口约 85 mm，超过 R5 的 70 mm 开口；从侧面略向下接近杯底上方约 40 mm 的杯身位置，短距离试提后倾倒。提示词保持 160 颗目标；独立宽松评分为至少 10 颗停留在接收杯内、源杯释放且竖立受桌面支撑约 0.5 秒即完成；固定接收杯按位置校验，不要求桌面支撑力读数。允许残留，不要求零洒落，实际洒落数量单独记录。纸杯按刚体处理，不能据此证明真纸杯不会压坏。抓取、重力流出及 R5 持杯倾倒工程检查放在 `integrationtest/`；持杯检查从预置开爪接近姿态开始，通过生产规划器执行夹持、倾倒、回正和释放，不调用模型，也不代表 GPT 自主任务成功。v8 保留半杯装料，画面右侧为装料源杯，左侧为固定接收杯，中心相距 200 mm；右臂倾倒、左臂观察；提示先建立腕部净空，再同步转动和平移，并依据珍珠实际落点调整杯口位置，避免低位原地转腕或流束越过杯沿。与旧布局和旧版 60 颗回合分开统计。任务定义见 [L2-04 配置](configs/tasks/l2_04_cup_pearl_pouring.toml)。

L2 采用 `guided_visual_l2_v3` 提示协议和 `native_l2_validity_v2` 评分协议：任务已给尺寸、抓法及方向提示，因此测的是带提示的视觉闭环执行，默认 RGB、可选 RGB-D。模型发送位姿和夹爪开口，底层控制参数固定；没有直接力／力矩／速度接口，模型等待时仿真暂停。双臂动作共用结束时间，并保留规划器已检查的轨迹时间表，但不保证相对末端位姿或自动保持负载水平。

交付实验室的夹指采用单个凸包，会填平内侧凹陷，缩小实际可用夹持间隙。L2 在会话层将四根夹指改为贴合原网格的凸分解，接触余量为 1 mm、静止余量为 0；保留外观、质量和关节定义。覆盖项写入 `scene.json`，不修改锁定源资产，也不改变 L1 的碰撞设置。控制器另用 0.25 秒实测位置窗口补充判断停稳，避免接触瞬时速度抖动造成无效等待；目标误差、受阻闭合和抓取成功仍分别判断。

报告分开列整体结果、阶段进展和过程参考值。L2-06 的形变指标仅作诊断，不再触发失败；L2-04 倾倒按接收数量评分，洒落单独记录。固定抽屉轨道失效、非有限读数等无法继续有效测量的情况单列为框架／仿真故障，不计模型成功／失败。L2-06 的局部 FEM 单元反转仅记录质量提示，不再中止任务。观察臂持续实测接触道具／夹具会判角色违规；软体附近只有几何接近证据时转人工复核，不伪称测到了力。数值无效和需复核回合仍完整保留，不静默删除。

书本任务的上下层固定，中层落稳后与上层留 5 mm 净空，避免控制微小偏差形成楔紧；不考察支撑上层。抽屉速度参考超限单独记录，不等同材料损坏。马克笔按实际笔尖／帽腔关系确认整笔拿起，不要求两个 USD 原点始终重合；放置阶段只在拔帽后记录。

橘子的抓持判断使用 FEM 碰撞表面到夹指真实三角网格的距离，不把夹指包围盒内的空隙算成接触；距离是几何证据，不是力。局部 FEM 单元反转记录在 `simulation_quality` 和 `quality_warnings`，不作为几何抬升的否决条件；该成绩不代表材料响应可信或真实组织没有损伤。非有限读数等导致无法测量时仍按框架故障处理。重复读取同一仿真帧复用已有评分，不重复计进度；GUI 暂停期间等待 Play 恢复，不消耗动作步或继续调用模型。运行中的真实时钟故障按框架执行错误报告。资产机制检查、固定姿态的 R5 夹持／移动检查与正式 GPT 完整回合分开报告。



`L1-07` 使用交付黄色空顶帽：最低圆头挂杆穿过帽圈，并对已释放的帽子提供向上承托，整帽离桌即成功，允许自然晃动，无稳定等待。被动支撑垫为宽帽檐抓取留出空间；帽圈不被规划器的包围盒封闭。

所有任务先执行公共机器人准备动作，再开始 GPT 决策；准备动作不读取物体位置。当前实验室配置以零关节角静置准备，之后允许 GPT 按需移动本轮指定的观察臂；双臂任务两手均操作。所有手起初为空，拔笔帽从横放整笔开始；没有脚本代替 GPT 抓起或完成道具任务。

每次模型调用先打印 `R5_POLICY_REQUEST 1` 和 `[request 1] thinking... (waiting for model output)`。
API 与 Codex 调用均申请公开思考摘要，收到摘要行或摘要结束事件时打印 `[request 1] thinking (summary): ...`，并保存到本轮 `episode.jsonl` 的 `model_exchange.exchange.reasoning_summary`。
接口可能不提供摘要，此时会明确显示 `no reasoning summary returned`；等待提示不证明服务端正在计算。摘要仅用于观察日志，不加入下一轮模型历史，也不作为机器人动作执行。只有完整回复通过原有校验后才执行动作。

静态场景、复位、相机和评分检查集中在 `integrationtest/`，不调用 GPT：

```bash
# 在 Isaac Python 环境中运行；每个任务使用独立进程。
python -m robot_tasks.integrationtest.diagnostics.scene_smoke --all
# 只校验物理场景与 RGB-D，跳过规划器及准备动作；不能代替完整启动检查。
python -m robot_tasks.integrationtest.diagnostics.scene_smoke --all --no-preparation
```

场景检查不统计任务成功率；GPT 是否能完成仍由真实模型回合及独立评分判定。未建模的断裂、压坏或流体效应不作为已验证能力。

## 三路相机与临时参数

第三视角为轮趣 Gemini Pro（奥比 Gemini 模组），两路腕部为 Gemini 305，外壳使用供应商／官方 STEP 转换的 USD。
[相机配置](configs/cameras/r5_orbbec_provisional.toml) 集中保存分辨率、RGB 内参、畸变、安装位姿和来源。
当前内参由标称 RGB 视场角估算，畸变为零，安装位姿及 CAD 相对光心位置是用户授权的临时工程假设；
日志明确记录 `provisional_uncalibrated`。这可用于仿真开发，不代表与现场相机完成了标定对齐。

旧场景以桌面为 Z=0，第三视角光心 Z=0.54 m，对应 0.78 m 桌高时离地 1.32 m；
桌前沿 X=1.0 m，相机 X=1.25 m，水平距桌边 25 cm。保留原朝向，向下俯视 45°。
两路腕部相机均为相对末端 +X 向 -Z 倾斜 45°；不是相对地面固定 45°。

三路图像与深度直接由 GPT 抓放流程采集和记录，不再保留单独相机集成入口。

相机共享代码位于 `cameras/profile.py`（配置读取与校验）和 `cameras/isaac_rig.py`（挂载、渲染及读取）。
腕部相机已加入用户提供的白色固定座 CAD，两侧暂按相同方向安装；装配位姿和光心偏移仍是估计值。
外壳与固定座仅参与视觉，不附加质量或碰撞；现场内外参和曝光等参数仍待实测。
CAD 来源和转换方法见 [assets/cameras/README.md](assets/cameras/README.md)。

## 开发安装

克隆 `https://github.com/TuojingAI/GPT_Benchmark.git` 后，直接在仓库根目录 `GPT_Benchmark/` 安装和运行。
Python 源码位于 `src/robot_tasks/`，配置、厂家资产、测试和工具均位于同一仓库根目录下。
本项目使用源码检出的可编辑安装；首次使用或切换环境时运行一次，之后修改 `.py` 文件立即生效。

## 按名称运行

在已激活的 Isaac Sim Python 环境中，任务名会自动解析到默认配置与任务实现：

```bash
python -m robot_tasks list
python -m robot_tasks inspect GPT-PICK-PLACE
python -m robot_tasks dry-run GPT-PICK-PLACE
python -m robot_tasks run GPT-PICK-PLACE --gui

# 同一任务使用不同实验参数；配置中的 task.id 必须仍为 GPT-PICK-PLACE。
python -m robot_tasks run GPT-PICK-PLACE --config configs/checks/gpt_single_arm_pick_place.toml --gui

# 工程检查名称与默认配置也由注册表解析。
python -m robot_tasks list --checks
python -m robot_tasks check gpt-pick-place --gui
```

`run` 当前使用 GPT 策略；指定模型配置用 `--model-config`。只保留 GPT 抓放集成检查。
L3-11 使用 `python tools/launch.py run L3-11 --gui`，运行接触擦除代理，不代表真实墨迹清洁规律。
`run GPT-PICK-PLACE` 与 `check gpt-pick-place` 使用同一个自主抓放实现。单次结果不能代表总体成功率。
原来的 `check-r5*`、`check-gpt-api` 命令已统一替换为 `check <名称>`；仅传 `run --config ...` 的用法仍保留。
WSL 调用 Windows Isaac 环境时继续使用 `tools/run_r5_check.ps1 -Check gpt-pick-place -Gui`，F5 调试入口也已同步。

| 文件 | 职责 |
|---|---|
| `cli.py` | 通用命令参数与结果展示；不维护具体任务或检查分支 |
| `registry.py` | 正式任务 ID、默认配置、实现类和实现状态 |
| `execution.py` | 预检查、创建任务／机器人／策略，交给公共 `runner.py` 执行 |
| `integrationtest/registry.py` | 检查名称、普通／调试默认配置、实现入口与前置校验；CLI、子进程及调试器共用 |

同类实验用 `*.local.toml` 配合 `--config`；维护中的任务 TOML 按受信任的 `implementation` 自动注册。
新增工程检查时在 `integrationtest/` 实现检查函数并加入 `CHECKS`，无需改 CLI、子进程、调试器或 PowerShell。
配置覆盖不能切换任务类型；任务或检查名称与配置不匹配时，执行前报错。

## GPT 单臂抓放

使用已激活的 Isaac Sim 环境：

```bash
python -m robot_tasks run GPT-PICK-PLACE --gui
```

本机 WSL 通过 Windows Isaac 环境启动（带独立工作进程和超时监控）：

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass \
  -File "$(wslpath -w ./tools/run_r5_check.ps1)" -Check gpt-pick-place -Gui
```

在启动命令所在的终端按 **Ctrl+C** 中断。启动器会恢复 Windows 控制台的 Ctrl+C 处理，
即使工作进程暂时没有日志输出，也会检查中断。先给工作进程 3 秒保存记录并退出；
若卡在原生代码或网络等待中，则限时清理本次工作进程及其子进程，不影响其他 Python 任务。
终端显示 `R5_PROCESS_STOPPING`，退出码为 130；`process.json` 记录中断、强制结束和退出确认状态。
强制结束时末尾录像可能未完成封装，已写入的记录保留；用户中断不作为正常任务失败统计。

F5 选择 **R5: GPT pick-place GUI (WSL)**。它运行同一任务实现，可在 `runner.run_episode()`、
`GPTPolicy.act()`、`R5Adapter.apply()`、`SingleArmPickPlace.evaluate()` 处断点。

默认配置为 [gpt_single_arm_pick_place.toml](configs/checks/gpt_single_arm_pick_place.toml)。
它引用 [公共运行参数](configs/workflows/gpt_default.toml)。左臂根据三路 RGB（可选深度）、相机参数、机器人状态和反馈，
自主抓起红色方块并放到绿色目标区；右臂是观察臂，GPT 可移动其腕部相机寻找视角，但不能向它下发夹爪指令。
观察动作和深度查询沿用同一个决策、执行与反馈循环，共享预算。没有独立的观察控制链路，也没有预设换视角动作。
初始复位由程序完成，执行过程中的接近、抓取、搬运、松爪和退出均由 GPT 决定，没有脚本补动作。
物体是自由刚体，不在执行中修改位姿或绑定夹爪。物体真值和双指接触读数只用于独立评分。

| 配置位置 | 含义 |
|---|---|
| `[task]` | 英文指令；公开评分要求的占位符从同一参数生成，最终指令另存 `instruction.txt` |
| `[interaction]` | 方块、摩擦、初始布局、绿色目标位置，以及抬升、释放与稳定性评分阈值 |
| `[execution]` | 每动作最多等待多少控制周期、完成确认、空动作等待、启动沉降、停止后观察及决策预算 |
| `workflow_profile` | 公共步长、预算、相机选择和记录参数；任务中显式写出的同名参数覆盖公共值 |
| `[robot]` | `active_arms` 为操作臂，`observation_arms` 为仅用于移动相机的观察臂；剩余机械臂固定 |
| `[policy]` | 模型配置位置；显式 `reasoning_effort` 可用于单次实验覆盖；API Token 从环境变量读取 |

所有 GPT 任务默认使用 RGB、相机内外参、桌面/TCP 标记、动作说明、完整回合文字历史和 cuRobo 轨迹规划；深度与像素查询通过 `--depth` 开启。
这些操作逻辑由公共代码实现，不通过任务配置启用或关闭。
正式任务只提供场景、角色、目标、复位与评分。单臂任务要求一个**操作臂**，允许另一个臂观察；
双臂任务可将两个臂都设为操作臂，同样复用公共流程。双臂交接目前仍只有模板，不能作为已实现任务运行。
旧的优化开关与三份对照配置已移除，新增任务自动使用同一流程。

默认物理步长 1/240 秒、控制周期 1/12 秒；每动作最多等待 120 个控制周期，连续 3 次完成反馈后提前返回。
空动作等待 12 个周期（1 秒）。夹爪被物体挡住而未达到空载目标时，反馈 `action_timeout`，GPT 可继续观察并纠正。
默认使用 medium、最多 100 次模型调用，不设整场实验的控制步数或现实时间上限；保留每条动作的执行预算。
公共配置 `runtime.max_wall_time_s = 0` 表示不限总时长，`run` 和 `check` 均适用；填写正数可恢复限时（单位秒）。
单次接口请求超时仍由模型配置的 `timeout_s` 控制；物理步数预算仍可能先于调用上限耗尽。
模型和思考强度统一在 `configs/models/gpt.toml` 中修改；现有任务与公共 workflow 不额外设置思考强度。
显式任务 `[policy].reasoning_effort`（如批量冻结配置）仍可覆盖模型值，`execution.max_decisions` 覆盖模型调用上限。
批量 `--reasoning-effort` 是显式实验覆盖；不传时沿用所选模型文件。`-ModelConfig` 可选择其他模型配置。
程序在 API 请求期间暂停物理。录像按仿真时间播放，接口等待耗时单独记录。
公共层检查固定臂的意外移动，观察臂移动本身不会被判失败。观察臂“不接触物体”由模型指令约束；
当前没有完整避障，也没有覆盖全臂的接触违例检测，不能把允许观察等同于保证无碰撞。

成功要求实测持续抬起至少 7 厘米（含双指接触证据），再释放到目标区并稳定至少 1 秒，同时夹爪张开并退出。
评分每控制周期更新，不依赖 GPT 的口头宣告；推过去、短暂抬起或未松手不能通过。任务超时及过早停止记为失败，
非法模型动作记录原始响应和 `invalid_model_action` 拒绝原因，再请求模型纠正；仿真保持暂停，不执行坏指令，重试计入调用预算。服务／仿真错误单独保留为未完成评估。`result.json` 记录 `success`、`termination_reason`、`invalid_model_actions`、`action_timeouts`、
`rejected_actions`、`false_completion` 和详细过程评分。`check gpt-pick-place` 的 `passed` 要求本次实际抓放成功。

运行隔离沿用原机制：新的场景、机器人、任务评分器和 GPT 策略；回合开始清空对话、计数器及执行状态，
不读取之前的运行结果。一个回合内保留有限历史以支持纠错。单次命令是独立 Python 进程，`check` 入口另带子进程 watchdog。
每个回合的报告、评分、配置、日志和录像统一保存在 `runs/<回合目录>/`，运行结束自动生成 `report.md`。
`effective_config.json` 保存合并后实际生效的配置，`workflow_profile.json` 保存本次加载的公共配置快照；
`manifest.json` 记录操作/观察/固定角色，`policy_configuration` 记录最终模型参数，避免只看原始配置误判实验条件。
成功、失败和服务异常都生成报告；原始评分以 `result.json` 为准，不把工程检查通过算作任务成功。
动作结束时的 TCP 跟踪误差与物体放置误差分别呈现，缺失测量不填成零。

```text
runs/<回合目录>/
├── report.md               结果、误差、调用／超时统计及录像链接
├── result.json             原始任务评分或工程检查结果
├── episode.jsonl           决策、反馈、逐控制周期记录和评分
├── config.toml             本回合配置；机器人、相机配置也保存在同目录
├── instruction.txt         实际使用的任务指令
├── frames/<相机名>/         决策时的原始 RGB
└── video/<相机名>/          motion.mp4 和预览 GIF
```

不调用模型、不重跑仿真，也可为已有回合补生成报告：

```bash
python -m robot_tasks.reporting runs/<回合目录>
```

脚本工程检查同样生成 `report.md`；其逐步记录为 `motion.jsonl`，具体录像布局见报告中的链接。
这些是当前仿真与临时相机配置下的测试，不代表真机抓放能力。

GPT 抓放接入时的本机历史验证：Windows 与 WSL 各通过 102 项离线测试；真实接口的抬手检查经 3 次调用通过。
抓放完整试跑经 40 次调用、37 次动作执行、3 次动作拒绝后达到调用上限，`success=false`，没有系统错误；
这证明了当时流程与结果记录已接通。后续启用坐标／TCP 图像标记的 IK 试跑以 15 次调用完成抓放，
最终目标偏差约 1.15 厘米；这是单回合成功记录，不代表稳定成功率。
两次回合分别位于本机 `runs/R5_gpt_pick_place_20260927T124321Z_88558a0b/` 与
`runs/R5_gpt_pick_place_20260928T060414Z_3f865fba/`；实际成功录像与报告见本机运行目录。
这些运行产物由 Git 忽略，其他机器需自行复跑。

## 本机配置

首次克隆已包含 `configs/models/gpt.toml`；确认本机 Codex 登录，或在该文件切换到 Responses API。
跨平台运行按上文配置 `configs/runtime/local.toml`；沿用本项目原有 Windows D 盘布局或调试入口时，
再将 `configs/runtime/windows.example.json` 复制为 `windows.json` 并填写本机路径。
已有配置不覆盖，这些本机配置均由 Git 忽略，Linux 无需准备 `windows.json`。
VS Code 的 `launch.json`、`tasks.json`、`settings.json` 也只保留本地版本；
仓库提交对应的 `*.example.json`，使用前替换示例项目路径、Conda 路径和 `WINDOWS_WSL_HOST`。
下文中的本地历史结果位于忽略的 `runs/`、`reports/`，首次克隆需重新运行检查生成。

Windows Isaac Sim 环境，在 PowerShell 中执行：

```powershell
cd D:\mengjq\research_projects\GPT_Benchmark
. .\tools\research_storage.ps1
$previousResearchStorage = Set-ResearchStorageEnvironment
try {
    & 'D:\conda\envs\isaac51\python.exe' -m pip install --no-deps --no-build-isolation -e .
} finally {
    Restore-ResearchStorageEnvironment $previousResearchStorage
}
```

WSL 的轻量检查环境，在 Bash 中执行：

```bash
cd /mnt/d/mengjq/research_projects/GPT_Benchmark
/root/miniforge3/envs/sim-research/bin/python -m pip install --no-deps --no-build-isolation -e .
```

这两个现有环境已具备构建所需的 setuptools。本地安装不更新 Isaac Sim 或其他依赖；新建环境需先准备
`pyproject.toml` 中声明的构建依赖，或使用 `python -m pip install -e .` 自动准备构建环境。
命令仍然是 `python -m robot_tasks ...`；`src` 只用于组织源码，不是模块名的一部分。
默认配置路径和仿真子进程工作目录统一由 `src/robot_tasks/paths.py` 确定；配置中引用的相对文件路径仍以所在 TOML 的目录为基准。

目录和安装方式参考：[setuptools src 布局](https://setuptools.pypa.io/en/stable/userguide/package_discovery.html)、
[pip 可编辑安装](https://pip.pypa.io/en/stable/topics/local-project-installs/)。

## 科研环境与数据位置

Windows 的 Isaac Sim 安装在 `D:\conda\envs\isaac51`，项目在
`D:\mengjq\research_projects\GPT_Benchmark`。WSL 的 Ubuntu 虚拟磁盘位于
`D:\wsl\Ubuntu\ext4.vhdx`，因此 WSL 中的 `/root/miniforge3`、`/root/.cache` 和 `/tmp`
实际也占用 D 盘，无需移到 `/mnt/d` 才算保存在 D 盘。

Windows 科研进程的运行数据统一配置在 [configs/runtime/windows.json](configs/runtime/windows.json)：

| 数据 | 实际位置 |
|---|---|
| Isaac Sim 缓存、数据和日志 | `D:\mengjq\research_projects\.runtime\windows\isaacsim\5.1` |
| 临时文件、pip 缓存、Python 字节码 | 同一运行目录的 `tmp`、`python` 子目录 |
| CUDA、Warp、OptiX 缓存 | 同一运行目录的 `nvidia` 子目录 |
| Hugging Face、Torch 模型缓存 | 同一运行目录的 `models` 子目录 |
| 仿真结果、录像和配置快照 | 本项目的 `runs/`，同样位于 D 盘 |

Python 入口、PowerShell 启动器和 VS Code 的 R5 调试配置均已接入。
`isaac51` Conda 环境也已保存这些环境变量，重新激活后生效。没有修改 Windows 全局的
`TEMP`、用户目录或 AppData。C 盘原有的 Omniverse、pip 及上述 NVIDIA 缓存目录已迁移，
原路径使用目录联接指向 D 盘；在资源管理器中看到 C 盘路径不代表数据仍占用 C 盘。

**已确认的例外：** `%LOCALAPPDATA%\NVIDIA` 下的 `GLCache` 和 `DXCache`
属于多个应用共用的图形驱动缓存，按用户确认使用 C 盘默认目录。GLCache 已恢复为普通目录，
不再通过目录联接转到 D 盘。两项单独记录为系统共享缓存例外，不计入科研数据路径检查。

检查科研路径可在项目目录执行：

```powershell
& 'D:\conda\envs\isaac51\python.exe' -m robot_tasks.runtime
```

仿真实际启动后的路径写入每次结果的 `scene.json` → `storage`，其中包括 Kit 解析出的
缓存／数据／日志位置和 Warp 实际缓存位置。`all_research_paths_on_d` 表示这些已检查路径
均落在 D 盘；`shared_system_cache_exceptions` 单独列出 GLCache 和 DXCache。每次运行也保存
`storage_config.json`，便于追溯当时设置。这是对科研进程已配置路径的核验，不代表追踪了整个 Windows 的所有文件写入。

实现使用 [Kit portable-root](https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/configuring.html)
与 [Conda 环境变量配置](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html)。
已有缓存路径由运行配置管理；早期一次性迁移脚本已移除，历史迁移清单保留在运行目录的 `migration/`。

## 在 WSL 的 VS Code 中单步调试

选择 **R5: GPT pick-place GUI (WSL)**，按 F5。Windows 本地窗口选择 **R5: GPT pick-place GUI (Windows)**。
两者使用相同抓放任务、配置和公共执行循环。WSL 入口由 PowerShell 启动 Windows Isaac Python，
调试器连接后停在 `integrationtest/debug_check.py`，随后可进入 `execution.py`、`runner.py`、
`policies/gpt.py`、`robots/adapters.py` 和任务评分器。

WSL 调试入口不启用正常检查的进程 watchdog；任务自己的时间预算仍然有效，长时间断点可在任务配置中显式调整。
本地 `.vscode/launch.json`、`tasks.json` 保留机器路径和 `pathMappings`；Git 中只提交对应示例。
WSL 重启后若网关改变，同步修改 `connect.host` 与 `-DebugHost`，调试端口只绑定本机或私有地址。
一次运行一个调试会话；断开调试器不一定终止进程，需要时在任务终端按 Ctrl+C。

其他关节、末端、脚本抓放、抬手、相机、API 和独立 IK 集成入口已移除。

## 临时仿真配置

[configs/robots/r5_sim.toml](configs/robots/r5_sim.toml) 描述机器人安装与控制映射；公共 workflow 设置物理步长，任务文件的 `[simulation]` 设置相机、求解迭代次数和控制增益。现场硬件配置 [r5.toml](configs/robots/r5.toml) 保持待确认状态。

- 双臂间距、初始姿态和运行范围为工程测试配置，没有声称与现场一比一匹配。
- 夹爪总开口根据厂家 STL 指端内表面及 URDF 关节原点近似换算；只验证分离状态下的开合，不声称已完成接触零点标定。
- 采用带重力的力／力矩型位置驱动。这里的驱动类型不代表已经实现面向模型的力控接口。
- 已导入每侧 9 个部件的碰撞几何，采用凸分解；当前关闭同一机械臂的自碰撞。抓放检查单独设置接触材料与偏移，验证方块接触链路，不代表厂家网格接触精度或真机动力学已标定。
- 当前默认 GPT 抓放使用三路 RGB（可选深度）。真实相机内外参及现场腕部安装仍待确认。
- 仿真 TCP 已显式定义；末端检查要求 TCP 配置完整，现场版本和真机 TCP 标定仍待确认。
- TGS 使用 32 次位置迭代、0 次速度迭代，并在 `scene.json` 记录实际设置。保持原目标时不重复下发驱动命令，避免不断唤醒已经稳定的机械臂；只调整求解迭代次数不足以解决这一问题。完成阈值没有放宽，接触任务仍需单独验证求解参数。

求解设置参考 [PhysX 对 TGS 速度迭代的限制说明](https://nvidia-omniverse.github.io/PhysX/ovphysx/0.6.3/guides/limitations.html)。

实现参考：[Isaac Sim URDF 导入](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/importer_exporter/ext_isaacsim_asset_importer_urdf.html)、[Articulation Controller](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.html)。具体调用同时核对了本机安装版本的源码。

## R5 资产与配置

2026-10-05 / 2026-10-06 的实验室、照片道具、桌面交互物品、书本和橘子瓣交付包已整理到
[assets/tuojing](assets/tuojing/README.md)，提供固定版本、安装命令和 USD 入口清单。
公共实验室已接入现有 Isaac 后端、R5 控制器和三路 RGB（可选深度） 采集，并提供无模型调用的
工程验收入口 `python -m robot_tasks.integrationtest.diagnostics.tuojing_lab --gui`。
新场景使用交付 USD 中的机械臂、相机和桌面；配置见
`configs/checks/tuojing_lab.toml`。**当前六个 L1 任务均使用新实验室、原生 R5 双臂和三路相机**，
同步适配 0.78 米桌面、准备姿态、抓取检查、规划障碍和任务评分。
L1-01 在加载配置时自动选臂：`task.arm_selection = "nearest_initial_object"`，
`task.manipulation_object = "marker"`，比较初始关节姿态下两只 TCP 到笔初始包围盒中心的三维距离。
当前右臂约 0.160 m、左臂约 0.391 m，因此右臂操作、左臂观察。距离完全相等时选左臂。
本轮角色固定，不因搬运时距离改变而换手；距离只用于分工，不代替抓取姿态、可达性或避障规划。
配置、模型指令、夹爪权限与接触绑定使用同一分工；距离依据保存到场景报告的 `parameters.arm_selection`，
不会把物体真值坐标作为模型观测。L1-03 同样启用此规则，目标对象为 `plug`；
其他 L1 暂时保持左臂操作、右臂观察。

第三视角位置、朝向、分辨率与镜头内参统一在 `configs/workflows/gpt_default.toml` 的 `[simulation]` 配置，整套参数对齐真机 `15_book_150s.mp4` 的全局画面：

```toml
camera_reference_frame = "tuojing_lab"
camera_position = [0.1956512155, 0.55, 1.45]
camera_target = [0.1956512155, 1.4721758867156862, 0.78]
camera_resolution = [1280, 960]
camera_intrinsics = [1160.0, 1160.0, 640.0, 480.0]
```

仿真任务默认继承此设置，控制实际 overview 传感器、模型图像输入、录像和 GUI 初始视角；L1-03 也使用公共视角。开启深度不改变相机位置或内参。
坐标以拓境实验室为参考；L3 等旧场景按桌前沿、双臂中线和桌面高度转换，避免把实验室绝对坐标直接用于旧场景。
公共参数整体控制 overview，避免旧位置与新焦距混用。相机配置文件保留传感器绑定和腕部参数；其中 overview 参数仅作为未提供公共配置时的回退。
当前仍是工程估计参数，不是真机标定。修改默认值后重新启动任务即可生效。
腕部相机保留原光心位置，光轴相对 link6 的 +X 向 -Z 倾斜 45°，并随末端运动；不是始终相对地面 45°。
光学相机与外壳／支架一并在 USD 会话层调整，交付源文件不修改，模型外参和录像使用同一实际相机。
所有维护中的仿真机器人配置均将六个臂关节初始目标设为 0 rad；夹爪初始开口仍为 41 mm。
初始化仅静置，不自动朝下或转动观察臂；物理步进后的实测角度可能有驱动跟踪误差。

| 任务 | 交付资产接入 | 保留或变化 |
|---|---|---|
| L1-01 插笔 | 白板笔＋固定黑色陶瓷杯（2026-10-09） | 原生尺寸；照片布局左杯右笔、杯把朝前；笔身平底碰撞适配 |
| L1-03 插头 | USB 充电器＋自由黑色收纳底座＋固定插线板 | 照片布局左侧横放插线板、右侧充电头竖放底座；中间双孔最小单侧间隙仍为 0.55 mm |
| L1-05 钥匙 | 公共实验室 | 无对应交付钥匙/锁，保留原有几何代理 |
| L1-07 挂帽 | 原生黄色空顶帽＋木质挂架 | 最低圆头杆穿帽圈并承重；帽檐下方有被动拾取支撑，不缩放资产 |
| L1-08 套杯 | 两只原尺寸波纹纸杯 | 相同尺寸；较窄下部抓持，浅入杯口后释放，锥壁承托评分 |
| L1-09 刚体玫瑰插花 | 原生仿真玫瑰＋固定纸杯 | 整株刚体；从较宽花头/花萼抓取，放入后允许靠杯壁 |

五个 L2 任务也已接入同一实验室及对应原生资产，改动与限制见前面的任务说明；L3 继续使用原场景。L1/L2 需先安装锁定的私有交付包；不能只改环境名称就当作任务已适配。
工程验收复用现有 `scene_smoke`，检查准备动作、三路 RGB（可选深度）、初态评分及物理承托，
输出见 `build/tuojing_l1_migration/<任务ID>/`。不调用 GPT，不代表模型完成了任务。
L1-09 的最新刚体玫瑰验收见 `build/tuojing_rigid_rose/`；旧软茎记录属于历史版本。
任务 ID、运行命令和公共模型工作流均保留，速度档位统一在 `configs/models/gpt.toml` 中选择。

厂家 R5a URDF 和 9 个 STL 网格已保存在 [assets/r5](assets/r5/README.md)。来源固定到具体提交，每个文件保留 Git blob 哈希；原文件不做修改。该模型每侧包含 6 个转动关节和 2 个夹爪滑动关节。它与现场 R5 的具体版本是否一致仍需核对。

机器人属性集中在 [configs/robots/r5.toml](configs/robots/r5.toml)，任务配置只引用它并选择参与执行的机械臂：

```toml
# configs/tasks/*.toml 使用 schema_version = 2
[robot]
profile = "../robots/r5.toml"
active_arms = ["left"]  # 双臂交接改为 ["left", "right"]
```

**平台始终配置左右两只 R5；单臂任务只启用其中一侧。** 关节名在每个机械臂实例内独立使用，通过 `/World/R5/left`、`/World/R5/right` 区分；未收到新动作的机械臂继续保持已有目标。旧 schema 1 任务配置会明确拒绝，不自动猜测迁移。

下表是现场硬件配置 `r5.toml` 的状态；本轮仿真使用上文独立列出的 `r5_sim.toml`。

| 配置内容 | 当前情况 | 后续需要补充 |
|---|---|---|
| 模型、网格与关节映射 | 已导入 Isaac Sim，关节运动可检查 | 后续验证物体接触和碰撞精度 |
| 左右臂安装位置与初始关节角 | 未填写 | 现场测量，或注明来源的仿真布局 |
| 工具中心 TCP | 未填写；`link6` 仅是夹爪父链接 | 定义实际抓取中心相对 `link6` 的位姿 |
| 夹爪开口换算 | 未填写 | 标定两指总开口与两个滑动关节的关系 |
| 运行限位、力／力矩及速度上限 | 未填写 | 厂家资料或明确设定的仿真运行范围及来源 |
| 现场硬件版本匹配 | 待确认 | 核对机械臂和末端夹爪版本 |

现场配置中的缺项保留为 `None`／空值。临时仿真配置独立记录工程参数及依据。厂家文件含 ±10 rad、速度 1000 等宽泛数值，因此资产参数与实际运行限制分别保存。离线校验不证明动力学准确、碰撞模型可用或完整任务可执行。

## 整体流程

```mermaid
flowchart TD
    C[任务配置：机器人、物体、相机、指令、参数] --> R[执行器：初始化与复位]
    T[任务：场景搭建与初始状态] --> R
    R --> O[观测：RGB、可选机器人状态和执行反馈]
    O --> P[策略：根据观测生成动作；任务默认使用 GPT]
    P --> A[机器人适配：动作校验、控制器、可选 IK]
    A --> S[Isaac Sim：物理步进与接触]
    S --> O
    S -->|仅评分器可见的物体状态和接触信息| E[任务评分：成功、失败、过程指标]
    E --> R
    R --> L[配置快照、轨迹、评分、图像与录像]
```

策略根据观测给动作；控制器将动作转换为机器人可执行的目标；仿真计算动作后的物理结果；评分器独立判断任务是否完成。**IK（逆运动学）已实现末端目标位置和姿态到关节目标的转换，目前验证范围是空载运动。**

## 目录与职责

```text
GPT_Benchmark/
├── assets/                       厂家机器人资产、相机外壳及来源信息
├── configs/
│   ├── tasks/                    任务目标、场景、角色与评分参数
│   ├── workflows/                多任务共享的执行与观测配置
│   ├── robots/                   R5 结构、安装、TCP、夹爪映射与运行范围
│   ├── cameras/                  相机安装、投影与成像参数
│   ├── models/                   模型渠道配置；实际配置由 Git 忽略
│   └── runtime/                  本机 Python、缓存与输出位置
├── src/robot_tasks/
│   ├── cli.py / registry.py      任务入口与注册
│   ├── execution.py / runner.py  组装模块与统一执行循环
│   ├── config.py / workflow.py   配置读取、共享设置合并
│   ├── robot_roles.py            操作、观察、固定角色的权限与检查
│   ├── policies/                 模型请求、观测组装、动作解析
│   ├── robots/                   资产读取、机器人驱动与执行反馈
│   ├── cameras/ / backends/      RGB-D、仿真生命周期与占位后端
│   ├── tasks/                    各任务的场景、复位和评分
│   ├── recording.py / reporting.py  全过程记录与报告
│   └── integrationtest/          GPT 抓放检查及启动/调试设施
│       └── diagnostics/          独立场景、资产接触与夹持诊断（不调用 GPT）
├── tests/                        不调用模型、不启动真实仿真的离线回归
├── tools/                        启动、资产转换和环境工具
└── runs/                         每回合结果、配置快照、报告与录像
```

`integrationtest/gpt_pick_place_check.py` 是唯一集成用例。其余 `registry.py`、`simulation_process.py`、
`simulation_worker.py`、`debug_check.py` 负责名称解析、进程隔离、超时和调试，不实现第二套任务或评分。
保留的工程诊断集中在 [integrationtest/diagnostics/](src/robot_tasks/integrationtest/diagnostics/README.md)。
`run GPT-PICK-PLACE` 与 `check gpt-pick-place` 最终都调用 `execute_task()` 和 `run_episode()`。

## 先运行框架

要求 Python 3.11 或以上，先完成上面的可编辑安装。离线配置、资产和运动学检查不启动 Isaac Sim，所需依赖见 `environment.yml`；真实仿真使用现有 Isaac Sim 环境及其中的 Pillow、OpenCV。无需重新安装 Isaac Sim。`environment.yml` 仅用于轻量离线环境，不包含仿真依赖。

在 Windows PowerShell 中：

```powershell
cd D:\mengjq\research_projects\GPT_Benchmark
$taskPython = 'D:\conda\envs\isaac51\python.exe'

# 列出模板及状态
& $taskPython -m robot_tasks list

# 校验厂家资产和关节映射，同时列出待确认参数
& $taskPython -m robot_tasks validate-robot --profile configs/robots/r5.toml --output reports/r5_asset_audit.json

# 严格检查：运行参数未填写时返回退出码 2
& $taskPython -m robot_tasks validate-robot --profile configs/robots/r5.toml --require-ready

# 检查任务引用的机器人配置，不启动仿真
& $taskPython -m robot_tasks inspect GPT-PICK-PLACE

# 抓放任务的占位流程自检
& $taskPython -m robot_tasks dry-run GPT-PICK-PLACE

# 双臂交接模板的占位流程自检
& $taskPython -m robot_tasks dry-run L2-01
```

WSL 中进入同一目录，使用 `/root/miniforge3/envs/sim-research/bin/python -m robot_tasks ...` 即可。

`validate-robot` 默认只要求资产和映射有效，报告中 `execution_configuration_ready=false` 会明确保留缺项；加 `--require-ready` 才要求运行参数齐全。两种模式都不启动仿真、不连接真机或调用模型，`simulation_validated` 始终为 `false`。配置内的文件路径相对各自 TOML 所在目录解析。

`dry-run` 默认走 3 步，只产生占位动作和 2×2 黑色 RGB 帧，输出 `dry_run_complete`、`task_evaluated=false`、`success=null`。**它不会打开仿真窗口，也没有可展示任务效果的 GIF。**

真实仿真的统一入口已保留：

```powershell
& $taskPython -m robot_tasks run GPT-PICK-PLACE --gui
```

当前 `run` 会先拒绝未实现的任务；已实现任务再核查仿真运行条件，失败时退出码为 `2`。方块抓放已有场景与评分，默认 GPT 通过三路 RGB（可选深度）、机器人状态和执行反馈自主决策。唯一全链路集成检查为 `check gpt-pick-place`。

## 一次试验的约定

- **执行顺序：**打开后端 → 初始化机器人 → 搭场景 → 六个臂关节置零并静置 → 读取观测 → 策略动作 → 机器人执行 → 物理步进 → 新观测与评分，循环直到终止。夹爪保持原有初始开口，底座位置不变；不执行自动朝下或观察臂摆位。
- **动作：**关节位置、末端位姿或夹爪总开口；双臂命令放在同一 `Action` 中，在同一次物理步进前应用。具体 SDK 转换、关节限制和 IK 留给机器人适配层。尚未定义力控接口。
- **单位：**距离用米，角度用弧度，时间用秒；位姿四元数顺序固定为 `w,x,y,z`。夹爪开口指两指总距离。
- **模型可见信息：**三路 RGB（可选深度）、虚拟相机参数、坐标与 TCP 标记、机器人状态、执行反馈及本回合文字历史；历史图片数量按模型配置控制。这是逐次采集图像，不是原生视频流。
- **评分信息：**物体真值位姿、接触力等走 `EvaluationState`，不传进策略输入。若将力觉提供给 GPT，需显式扩展观测协议。
- **停止：**策略声称完成不等于成功。成功必须来自任务评分器；已启动评估的任务超时或提前停止记失败，服务／仿真异常与未实现模板另行记录。
- **时间预算：**区分物理时间和墙钟时间。模型请求有 socket 超时和返回后的总耗时检查，超时结果不执行；仿真 CLI 子进程另有总墙钟 watchdog。等待 API 期间仿真不推进，因此不能据此评价真实动态响应。
- **复现：**保存任务与机器人配置、资产来源清单、随机种子、代码哈希和逐步动作。配置快照保留原始内容，重放需使用相同仓库布局及锁定资产。传递 seed 不等于保证 GPU 物理完全确定，后续需记录仿真版本并复核重复性。

输出目录如下；每次运行使用独立目录，不覆盖已有结果：

```text
runs/<任务编号>_<模式>_<时间>_<随机后缀>/
├── config.toml             本次配置副本
├── robot_profile.toml      R5 共享配置副本
├── asset_source.lock.json  厂家资产来源与文件哈希副本
├── manifest.json           模式、种子、参与臂、配置哈希、代码哈希和资产路径
├── episode.jsonl           每步动作、观测引用、反馈、评分和调用耗时
├── result.json             终止原因、是否真实评分、成功状态和错误
├── frames/<相机名>/*.ppm    决策时的 RGB；dry-run 为占位帧
└── video/<相机名>/          MP4、GIF 与首帧（真实 GPT 运行）
```

## 固定观测与决策记录

通用 GPT 链路默认提供 RGB、相机参数、图像标记和决策说明，所有任务自动复用。当前抓放右臂用于主动观察；是否允许其他机械臂移动由公共角色权限决定。
角色选择、输入、预算和最终模型参数都进入本轮记录，不应把不同条件的结果当作同一个实验。

默认不采集或保存深度、不提供像素测距工具、不启用深度派生的抓取／释放核验。模型根据 RGB 自行观察；成功评分、物理碰撞和轨迹规划保持原有逻辑。规划器仍使用仿真碰撞几何，不能将此称为整个系统完全只依赖视觉。

```bash
# 默认 RGB；原命令无需改动
python tools/launch.py run L1-01 --gui
# 同一流程开启深度图、像素查询及适用任务的抓取／释放核验
python tools/launch.py run L1-01 --gui --depth
# 即使配置开了深度，也可在这次运行显式关闭
python tools/launch.py run L1-01 --gui --no-depth
```

也可在公共工作流或任务的 `[observation]` 中设置 `include_depth = true`；命令行参数优先。
`recording.save_depth` 未指定时随 `include_depth`；命令行开关同时控制采集与深度存盘。
`inspect`、`dry-run` 和 `python -m robot_tasks.batch` 同样支持 `--depth` / `--no-depth`。
批量任务冻结实际设置，续跑不会改变原批次模式。新模式写入 `effective_config.json`、`manifest.json` 和模型记录。

模型始终收到每个已选视角的 RGB、机器人状态和执行反馈；开启深度后额外收到对齐深度预览。缺少该模式的必要输入时在调用接口前报错。
虚拟相机内参和逐帧外参随观测提供；图像缩放时同步调整内参。
参数来自仿真，`hardware_calibrated=false`；深度为光轴 Z 方向的米制距离，
不模拟真实相机的噪声、盲区或双目匹配误差。

开启深度时，深度预览使用固定 0.02—2.00 m 色标。模型可通过 `depth_queries` 查询 RGB 图像中的像素，
每次最多 8 个点，返回对应可见表面的深度、相机坐标和世界坐标，不提供物体中心或评分真值。
查询不执行动作、不推进物理，但占用模型调用预算；不能与运动或停止请求同时提交。
无效深度返回 `no_depth`。运动、等待、停止时传空查询列表。

在已开启深度输入的前提下，`recording.save_depth=true` 将原始 float32 米制深度保存到 `depth/<camera>/<step>.npy`，
NaN 表示无效像素，同名 PNG 是预览。保存时刻与决策观测一致，复位帧单独保存；
不是逐物理步深度录像。记录开关只影响存盘，不影响 GPT 获得的输入。
参考：[Isaac Sim 相机深度 API](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.camera/docs/index.html)。

每次决策必须附带 `note`，
用一两句英文描述当前观察及动作目的，最多 512 字符。它只作为历史和日志记录，
不参与控制或评分；图片历史淘汰不会删除文字笔记，新的 run 会清空历史。

## 结束条件与记录

每次启动只运行一个回合：任务成功或评分器确认失败后立即结束，不复位场景、机器人或评分器，
也不在同一 run 内开启第二次尝试。再次运行命令才会创建新的回合和独立记录。
已移除 `execution.retry_failed_attempts` 配置；旧配置中的该字段需删除。

模型接口超时或可重试的生成异常仍最多重试三次，保留原场景，不执行动作；计入总调用预算。
无效动作、动作被拒绝或尚未成功时请求停止，可以在同一物理回合内继续决策。
任务失败、预算耗尽或故障则结束。报告记录 `evaluation_protocol=single_episode`。
历史报告中已有的多次尝试记录仍可读取，不修改已归档的实验结果。

L1-01 使用 `scoring_protocol=l1_entry_only_v1`：笔任一端真正进入主杯腔、无明显穿模即成功。
夹持送入与杯口上方松手后自然落入均可；松手本身不算成功，须观测到实际入杯。
允许仍在夹持、倾斜和运动，无松手、杯体承重、触底或静置门槛；保持 1 mm 数值接触容差，
不设置固定角度或 40 mm 深度。只在杯口上方对准、贴杯口、搭边或进入把手孔均不算入杯。
放回桌面或碰倒在桌上只记录过程信息，可在原场景继续调整；掉到桌下仍结束为失败。
释放、承托和运动数据继续写入结果，作为附加指标；旧结果保留旧版本，不与新成功率混算。

L1-03 使用 `scoring_protocol=l1_plug_entry_only_v1`：两片原生插脚分别实际进入中间对应孔即成功，
允许浅插、继续夹持与运动，不要求 15 mm 深度、外壳落座、松手、承托力、抓取历史或稳定等待。
沿用 0.15 mm 数值容差区分进入与面接触；两孔对准但悬空、仅一脚进入、明显穿壁或穿底都不算成功。
评分随插线板旋转，完整落座与释放承托仅作诊断；新版本不与旧严格版本成功率混算。

其余任务采用 `scoring_protocol=recoverable_progress_v1`，区分目标达成、必要过程约束和参考值记录：

| 情况 | 当前处理 |
|---|---|
| 插头／钥匙／套杯临时倾倒、抓空、放偏；螺丝在桌上倾倒 | 在原场景继续观察、重抓和调整，不复位 |
| 套杯、刚体插花 | 用实际容纳、承托和释放状态评分，取消额外的固定小角度或示例深度门槛；杯口搭边、穿模和悬空不算成功 |
| 拔笔帽后放下时短暂下落 | 可继续落桌并完成释放，不凭下落速度直接判失败 |
| 拔笔帽的力峰值、抽屉的力／速度峰值 | 作为未标定工程参考记录，不直接判失败，也不能解释为已测得损坏 |
| T 块暂时翘起／倾斜、扫块时短暂碰触、碰到非目标抽屉 | 记录过程异常并允许恢复；持续抓起搬运代替推动、直接用夹爪搬块、观察臂操作物体仍违反任务要求 |
| 小球洒出、弹性代理形变超标 | 允许完成剩余搬运，保留不可抹除的过程违反记录；到达目标也不能算满足全部要求的成功 |
| 物体丢失到桌下、抽屉持续脱轨／越过限位 | 结束为失败；洒出的小球按上一行单独记录，不中断仍可控的碗搬运 |

报告单列“任务目标与过程约束”。真实抓持、插入／承托、螺纹净旋转与推进、按压弹出过程等任务核心要求仍保留，
预算耗尽时未达到完整成功条件仍为未完成。这些改动只调整判定与记录，不代表已经验证模型成功率提高。
旧报告保留原评分，不重新计算；已启动的进程需要重新启动才会使用新规则。

MP4 保留所有控制周期的画面；GIF 是覆盖整轮的抽帧预览，最多约 240 帧，限制长回合内存占用。

## 每个任务需要补齐什么

同一种任务只改变布局、物体参数或 prompt：新增一份任务 TOML，保持 `task.id`，通过 `--config` 选择即可。
如果是新的任务类型，通常改三处：

| 位置 | 需要做什么 |
|---|---|
| `src/robot_tasks/tasks/<任务>.py` | 实现 `build_scene()` 创建物体/场景、`reset()` 复位、`evaluate()` 根据实测状态评分；需要时用 `prepare_config()` 检查参数并生成指令 |
| `configs/tasks/<任务>.toml` | 引用共享 workflow、机器人与相机，填写任务 ID、指令、操作/观察臂、场景和成功条件 |
| `tasks/bench/{l1,l2,l3}.py` 与 `scoring_l*.py` | 家用物体任务分别提供几何/关节定义和过程评分；TOML 的 `implementation = "household"` 由注册表自动绑定。新增另一类实现才修改注册表的受信任映射 |

新物体需要额外资产时放入 `assets/` 并记录来源。场景较复杂时可在 `tasks/` 内拆场景与评分辅助模块。
已有动作、观测和力学能力足够时，不改 CLI、GPT 客户端、IK、runner、recording，也不为每个任务复制集成入口。
新任务使用 `run <任务ID>`。若需要当前没有的力控、触觉或新动作类型，才扩展公共接口和适配器。

物体尺寸、初态、关节和评分阈值目前定义在对应任务实现中；TOML 不会覆盖代码中未公开的几何参数。

## 批量运行与录像

使用 `python3 tools/batch_run.py --all --output runs/gpt_master_baseline` 串行测评全部
仿真任务并保存三路录像。入口自动加载 `.env`，默认每任务 100 次调用、60 分钟，
支持 `--dry-run`、`--tasks`、`--resume`、结果/Token/视频汇总。详细步骤见
[批量运行说明](docs/batch_run.md)。

## 哪些配置影响任务

| 配置 | 影响内容 | 修改范围 |
|---|---|---|
| `configs/tasks/<任务>.toml` | instruction、角色、实现绑定与运行说明；`[simulation]` 还包括驱动刚度/阻尼、碰撞和求解器设置 | 当前选中的实验 |
| `configs/workflows/gpt_default.toml` | 相机选择、记录设置、物理/控制步长、时间和调用预算等公共运行参数 | 引用该文件且未覆盖相应字段的任务 |
| `configs/robots/r5_sim.toml` | URDF、左右底座、home、TCP、夹爪映射和关节限位，影响 IK 与动作执行 | 引用这个机器人配置的任务 |
| `configs/cameras/r5_orbbec_provisional.toml` | 相机型号/外壳、安装方向、分辨率、内参、畸变和视角 | 引用这个相机配置的任务 |
| `configs/models/gpt.toml`，或 `--model-config` 选中的文件 | 调用方式、模型 ID、思考强度、速度档位、API 地址、Token 环境变量名、超时、图片大小和历史图片数 | 选择该模型文件的运行 |
| `configs/runtime/local.toml`，可选 `windows.json` | 实际启动的 Python、缓存、输出和本机存储设置 | 本机运行环境；不放任务 prompt 或评分参数 |

生效规则：

1. `--config` 选择任务文件；不传时由 `registry.py` 选择默认文件。
2. 可调运行参数优先使用任务文件中的显式值，其次使用引用的 workflow 值，再使用代码默认值。固定操作逻辑不参与覆盖。
3. `--model-config` 优先于任务的 `policy.model_config` 选择模型文件。现有任务默认沿用模型文件的思考强度；只有显式的 `policy.reasoning_effort` 才覆盖它。`execution.max_decisions` 仍覆盖 `max_calls`。
4. `--gui` 覆盖 `simulation.headless`。图形后端未写时按实际 Isaac Python 所在系统选择。
5. 公共配置当前是 medium、最多 100 次决策；总控制步数和现实时间默认不限；每条动作仍有执行步数限制。未成功的停止请求消耗一次调用，保留场景与历史继续，成功或预算耗尽后结束。

路径以引用文件的目录为基准。`OPENAI_API_KEY` 是运行进程的凭据，不写入配置快照。
用 `inspect <任务ID>` 查看合并后的角色、输入和预算；每轮 `effective_config.json` 与日志里的 `policy_configuration` 保存实际实验条件。
旧 `[policy_options]` 或 `execution.gripper_timing` 配置会明确报错并提示删除，不能再退回旧链路。
`inspect` 和每轮 `policy_configuration.workflow` 会记录固定流程版本及能力。

## 框架检查

`tests/` 只运行离线软件测试，不需要 GPT Token 或 Isaac Sim。GPT 决策使用模拟客户端，
HTTP/SSE 测试使用模拟传输和响应；测试进程拦截网络连接，漏写 mock 会直接报错。
单元测试日志中的 `R5_POLICY_REQUEST` 表示执行循环进入了模拟策略，不代表真实模型请求。
CI 仅运行这些离线检查；真实 GPT 抓放留在 `integrationtest/`，通过 `check gpt-pick-place` 手动启动。

```powershell
& $taskPython -m unittest discover -s tests -v
# 或同时检查两个任务配置与自检入口：
powershell -NoProfile -ExecutionPolicy Bypass -File tools/check_framework.ps1
```

检查覆盖资产缺失与损坏、错误关节映射、单／双臂选择、运行参数缺项、夹爪换算与限位冲突，以及占位结果、模型信息边界和异常关闭等。它们验证软件约定，不验证任何物理任务。

## Git 中保留什么

保留源码、测试、配置示例、环境定义，以及运行所需的 R5 URDF／STL 和相机 USD。
缓存、仿真构建产物、录像、逐次结果、机器审计报告、原始相机 STEP、
本机模型渠道／路径配置和个人调试配置由 `.gitignore` 排除。
厂家资产保留来源与文件哈希，并沿用其原始使用条款；本仓库不为第三方资产重新授予许可。

## 公共 GPT 控制逻辑

抓放与后续新增任务都走同一实现，任务只定义场景、分工、指令、复位和评分。
运行预算与模型参数保留配置入口；默认 medium、100 次上限。

```bash
python -m robot_tasks run GPT-PICK-PLACE --gui
python -m robot_tasks check gpt-pick-place --gui
```

WSL 开发、Windows 运行 Isaac 时，继续使用现有 PowerShell 入口。例如：

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w ./tools/run_r5_check.ps1)" -Check gpt-pick-place -Gui
```

GPT 固定使用 `sparse_eef` 命名字段动作接口，使用绝对世界坐标；
`x_m/y_m/z_m` 为 null 时保留最新实测值。角度采用 `Rz(yaw) Ry(pitch) Rx(roll) R_reference`，
全零时 R5 TCP 的 +X 向下、夹爪闭合轴沿世界 +Y；三个角度都为 null 时直接保留实测四元数。
`gripper=0` 闭合、`gripper=1` 完全张开，`gripper=null` 保留已有夹爪驱动目标。0 对应配置的最小空载开口（当前 12 mm），并非 0 mm；接触物体后允许实际开口大于目标。内部仍转换成同一 `Action`，不执行模型生成的程序。

同一条命令中，机械臂运动到位并稳定后才执行夹爪目标。该顺序固定，不设开关。
动作超时取消尚未执行的开合，避免在下一轮等待时意外释放物体。反馈包含目标／实测 TCP、各轴剩余误差、
夹爪开口误差和执行阶段；“夹爪没合到目标”不会自动判为已抓到物体。

三路相机默认在同一物理状态完成两轮渲染后读取，空缓冲仍有限重试。记录的相机参数来自虚拟场景，并非现场手眼标定；参数始终传给模型，
图像缩放时同步缩放内参。单个像素与内外参仍不足以确定未知物体深度。

图像标记使用虚拟相机内外参，将已知桌面世界坐标和实测末端位置投影到三路图像上。
标记不读取物体真值、不标出目标答案，也不代表物体检测结果；遮挡处的投影不能当作可见物体。
标记仅作用于模型输入，原始 RGB 与录像保持原样；逐次调用记录标明已添加标记，并保留输入图像哈希。

正式对照应固定布局、相机、模型与预算，每次独立复位，并重复多次。各次运行的 `report.md` 与 `result.json`
保留失败和基础设施异常，报告调用、超时、末端误差、耗时和配置；少量试跑不能证明成功率改善。

此前未开启图像标记的 IK 工程试跑中，`low` 推理强度用 21 次调用完成一次抓放（约 524 秒，3 次动作拒绝、8 次动作超时）；
`medium`、100 次上限的另一回合在第 23 次调用主动停止，物体已搬运，但距目标中心约 3.4 厘米，超出 2.5 厘米阈值，判为失败。
三路相机投影检查误差均小于 1 像素。这些是单次试跑，不能据此确认成功率改善或某个组件的独立贡献；真机手眼标定仍需现场数据。

开启图像标记后，保持上述 `medium`、100 次上限及相同任务条件的一次 IK 抓放用 15 次调用完成，耗时约 458 秒。
方块最高抬起约 11.27 厘米，水平搬运约 13.89 厘米，最终误差约 1.15 厘米；0 次动作拒绝、8 次动作超时。
三路录像均完整，45 张模型输入图像的标记与记录哈希均已核对。这仍只是单次工程验证，不能据此估计成功率。

实现参考：[RoboProbe 的动作接口](https://github.com/RoboProbe/RoboProbe/tree/main/policy/RoboDojo_Agent_L3_Inspect_EEF)、
[Responses 会话状态](https://developers.openai.com/api/docs/guides/conversation-state)。本项目仍使用 R5，不复制参考项目的 X5／Franka 参数。

### L2-01：双臂拔笔帽（原生资产）

`python tools/launch.py run L2-01 --gui`。整支马克笔横放桌面，左臂拿起笔身，右臂拔帽，最后分别落桌释放。使用锁定的 L20 可拔帽资产和公共模型工作流；机制、评分及限制见 [任务说明](docs/l2_01_marker_uncap.md)。

### L3-05：双臂旋下预松螺母

`python tools/launch.py run L3-05 --gui`。原生 90 mm 膨胀螺栓横放在双臂中间桌面，第三视角看主体朝左、螺母朝右，螺母预松至剩约两圈。左臂拿起主体，右臂旋下螺母；实际分离且两手分别持握即可成功，无需放下或等待。保留真实 SDF 螺纹接触，无固定主体或同轴导轨。仅本任务采用 1920 Hz 物理步长与 10 mm 最小开口配置。详见 [任务边界与运行说明](docs/l3_05_coarse_screw.md)。旧旋入结果保留原口径，不按新任务重判。


### L3 新交付资产与简化完成条件

L3-03、L3-05、L3-11、L3-14、L3-15 使用拓境实验室场景及公共相机、零关节初态、模型工作流。L3-16 已从现行任务池移除。原始资产以 `physical_props_20261008` 包锁定，源文件不修改；运行时适配记录在场景报告中。

| 任务 | 资产和操作 | 完成条件 |
|---|---|---|
| L3-05 双臂旋下松螺母 | 原生独立主体/螺母，预松剩约两圈；左臂拿起、右臂旋出 | 有实际相对旋转和分离，两手分别持握即可；不要求放下、指定圈数或等待 |
| L3-11 板擦擦绿板 | 固定绿板平面旋转 180°、黑色 GPT6 字迹从第三视角倒置；板擦在右，右臂抓取擦拭、左臂观察 | 接触、滑动擦除至少 90% 的初始可见字迹，不要求松手、放回或等待 |
| L3-15 扫块入簸箕 | 右侧原生手刷、左侧固定簸箕、2 个 20×20×10 mm 方块；右刷左观察 | 实际持工具扫入，2 块都在篮内并由簸箕承托，不要求最后仍握工具或静止等待 |

螺丝改造改变尺寸，不代表原膨胀螺栓机制；字迹为等面积小格接触擦除，不模拟墨迹化学；刷毛为原资产的刚性胶囊束。代码不替模型抓取、旋拧或扫块。工程检查与 GPT 完整成功分开记录，结果目录为 `reports/l3_assets_20261008/`。

初始场景图片可统一导出（无 GPT 调用）：

```bash
python -m robot_tasks.integrationtest.diagnostics.initial_gallery --all --output reports/task_initial_gallery_20261008
```

输出 `index.html` 包含全部任务的公共第三视角、独立展示近景和中文布置说明。初态校验通过不等于机器人或模型能完成任务。
