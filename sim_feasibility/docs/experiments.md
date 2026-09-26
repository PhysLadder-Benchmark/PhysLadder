# 本机机制验证

## 验证范围

用脚本施加已知动作，检查仿真是否保留任务需要的因果关系。本轮不测试 GPT、不操作真机、不把物体级实验当成整机任务完成。

| 实验 | 对应候选 | 核心对照 | 通过后能说明什么 |
|---|---|---|---|
| 插入与碰撞 | L1-01～05 | 对准与错位；不同间隙；开启和关闭碰撞的负对照 | 当前碰撞表示可阻挡错误插入，并支持正确插入 |
| 摩擦与受力 | L2-01～04、07、08 | 固定外力，改变摩擦或法向载荷，检查运动与接触读数 | 接触参数会真实改变运动，具有搭建力调节任务的基础 |
| 弹性按压 | L2-09；L2-06 的前置机制 | 不同刚度、加载及卸载 | 可测力—位移关系；弹簧关节不等于柔性面包 |
| 柔性体压缩 | L2-06 | 软／硬材料与加载卸载，检查节点形变、数值稳定性 | 支持弹性形变代理指标，不证明不可逆压坏或断裂 |
| 螺纹反例 | L3-05，间接影响 L3-03 | 只轴向施力与旋转推进 | 判断现成资产是否真的保留螺纹耦合，不能只靠终态分数 |

前三项已完成；另已运行程序生成的柔性体和下载资产的螺纹实验。它们都只验证前置机制，不等于完成原任务。

另做 Franka 关节控制与 RGB 渲染的基础检查。它只证明上游机器人资产能加载、动作能执行、相机能出图，不表示已跑通 RoboDojo 的完整任务、末端 IK 或 R5 双臂链路。

各机制实验只覆盖表中任务的一部分前置机制。例如圆柱插孔通过，不能代替薄片、钥匙和插头各自的碰撞验证。

## 运行环境

- 主机：Windows 11，RTX 5080 Laptop GPU，约 16 GB 显存、32 GB 主机内存。
- Conda：`D:\conda\miniconda3`；仿真环境：`D:\conda\envs\isaac51`。
- Isaac Sim 选 5.1.0.0 / Python 3.11，与 RoboDojo 安装脚本对齐；这是复现版本，不是最新版本。
- SoftVTBench 原环境为 Isaac Sim 4.5 / Python 3.10，不能直接把整份依赖混装进 5.1。若仅迁移一个柔性资产做机制实验，记录为迁移实验，不称为完整复现。
- WSL 的 `sim-research` 仅做源码、配置与结果整理；没有向原 base 安装项目依赖。
- 未直接执行 RoboDojo 的 Linux 安装脚本；IsaacLab、cuRobo 与完整任务流程的 Windows 集成尚未验收。

官方安装说明：[Isaac Sim 5.1 Python 环境](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_python.html)。

## 已完成的验证

| 实验与记录 | 本机实测 | 结论与限制 |
|---|---|---|
| [圆柱插入](../results/experiments/insertion-20260925-215330/summary.json) | 5 个条件：0.5 / 3 mm 径向间隙分别测试对准、30 mm 错位，另设关闭孔壁碰撞的反例。对准与关闭碰撞时落到底部；错位且开启碰撞时被挡在孔口。输出 39 帧 RGB。 | 通过当前几何的碰撞检查；使用重力落入，不是机械臂完成插孔，也未验证钥匙等复杂几何。 |
| [摩擦受力](../results/experiments/friction-20260925-215528/summary.json) | 同样施加 1 N：低摩擦 1 kg 物体滑动约 1 m；高摩擦或 4 kg 物体基本不动。接触力读数分别约 9.81 / 39.24 N。输出 50 帧 RGB。 | 摩擦和载荷对运动的影响、接触力单位均符合预期；不等于已完成拔帽、交接或擦拭任务。 |
| [弹簧按压](../results/experiments/spring-20260925-220424/summary.json) | 2 N 下，100 / 1000 N/m 两种刚度分别压缩约 19.97 / 1.97 mm；卸载后回位。输出 50 帧 RGB。 | 支持选用弹簧按钮作为首版力—位移任务；不代表任意柔性材料。 |
| [柔性体压缩](../results/experiments/deformable_probe-20260925-221407/summary.json) | 两个 6 cm 弹性立方体，杨氏模量 10 / 100 kPa；同样施加 2 N，压缩约 4.42 / 1.66 mm。卸载后的高度残差约 0.009 / 0.005 mm，节点数值有限，输出 60 帧 RGB。 | 通过软硬差异与回弹的定性机制检查；未做网格收敛、真实材料标定、永久压坏、断裂或面包任务。 |
| [机器人与 RGB](../results/experiments/robot_smoke-20260925-215756/summary.json) | Franka 9 个自由度加载成功；关节运动范围约 0.573 rad，单侧夹爪位移约 30 mm，80 帧 RGB 随运动变化。 | 通过关节动作和图像基础链路；尚未接末端 IK、R5、GPT 或完整 RoboDojo 任务。 |
| [螺纹接触](../results/experiments/screw_probe-20260925-223020/summary.json) | GPU 后端下，关闭螺栓碰撞的控制条件可旋转约 7.99 rad；开启接触后，正／反向累计旋转约 4.66 / −2.13 rad，高度相对不转条件只变化约 +0.167 / −0.137 mm。尚未确认连续的旋转—推进关系，输出 79 帧 RGB。 | **未验证通过**。保留代码借鉴依据，暂不进入首批正式任务；应继续核查配合几何、轴线、约束、接触参数及驱动，而不是据此判断 GPT 或原论文失败。 |

三组基础机制的[实测曲线](../results/experiments/mechanism_curves.png)及[柔性／螺纹曲线](../results/experiments/contact_probe_curves.png)来自 CSV。每个结果目录保存参数、日志、轨迹、场景及 PNG / GIF；`run_manifest.json` 记录当次脚本哈希，已核对的运行保留 `script_snapshot.py`。完整索引见 [reviewed_runs.json](../results/experiments/reviewed_runs.json)。

## 螺纹为什么没有直接判为可用

第一次试跑出现弹跳，排查发现螺栓 USD 存在未列入 `xformOpOrder` 的 `scale=1.5`；导入工具重建变换顺序时会启用它。脚本现已明确设为单位缩放，并记录小尺度接触参数。另有一次画面因相机默认近裁剪面为 1 m 而看不到近处物体，现已改为 5 mm。

另发现早期 D6 夹具使用了无效的相等角度限位；已按 [USD 规则](https://openusd.org/dev/api/class_usd_physics_limit_a_p_i.html) 改为 `low > high` 锁定指定轴。前三次螺纹运行有求解器错误，已标记 `INVALID_RUN.txt`，不引用其数据。

还修正了角速度上限的单位：USD 属性用度／秒，Tensor API 回读用弧度／秒。最终采用 GPU PhysX / Warp，关闭螺栓碰撞并去掉轴向负载的控制条件可以自由旋转，说明驱动和夹具已允许旋转。CPU 调试运行中出现过 SDF 接触材质索引警告；只引用最终 GPU 运行作为本轮证据。当前资产接触仍未证明稳定的螺纹旋转—推进关系。公开 Factory 任务注册和资产说明是继续复用的依据，不替代本机结果。现有终态评分也不能证明物理啮合或滑牙损伤。

## 环境与复现注意项

- Vulkan 启动在 `rtx.scenedb.plugin.dll` 崩溃；项目通过 `runtime_config.py` 使用 DirectX 后启动、物理计算和 RGB 均可运行。没有修改显卡驱动或系统图形配置。NVIDIA 也记录过 [RTX 5080 / Windows 的相似 Vulkan 问题](https://forums.developer.nvidia.com/t/isaac-sim-6-0-0-crash-with-rtx-5080/373892?page=2)；本机结果只证明替代后端有效，不将驱动版本直接定为唯一根因。
- 图像使用 [Replicator 手动采集](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_getting_started.html)，`delta_time=0`，避免拍照额外推进物理时间。GPU 物理的画面还需先同步 Fabric。
- 当前 PyTorch 是 CPU 版。柔性机制实验使用 Isaac 自带 Warp GPU 接口；这不代表完整 RoboDojo 所需的 CUDA PyTorch / IsaacLab / cuRobo 已安装。
- 原 SoftVTBench 柔性资产目录未取得；本轮用自行生成的弹性立方体测试基础形变，不能称为复现了面包任务。
- 这是确定性机制检查，每个参数条件运行一次，没有统计模型成功率、真实物理参数标定或真实时间性能排名。失败的早期运行目录保留，但只引用表中已核对的运行。
