# Bowen · L1–L3 新资产与环境交接（2026-10-14，r3）

本分支保存当前 L1/L2 源码与资产修改，并新增 15 项任务的场景交付入口。r3 修改 14 项；L3-11 保留旧版。**当前交付是视觉修订 + 原轨迹重渲染，不是新一轮动力学验收。** 已知差异见 [修订记录](revision.json)。

## 队友最快使用方式

```bash
git fetch origin
git switch --track origin/bowen   # 已有本地 bowen 时用 git switch bowen
python3 tools/visual_release.py fetch
python3 tools/visual_release.py list
python3 tools/visual_release.py scene L1-01
```

`fetch` 在已挂载共享盘的服务器直接读取资源包。自己的电脑或其他服务器使用有共享盘访问权限的 SSH 账号：

```bash
python3 tools/visual_release.py fetch --ssh YOUR_SSH_ALIAS --port 22
# 或已拿到离线包：
python3 tools/visual_release.py fetch --archive /path/to/bowen_visual_r3_20261014.tar.gz
python3 tools/visual_release.py verify
```

该工具校验归档和解压后的每个文件，不覆盖已经存在的安装目录。资源包下载需要团队已有的共享盘/SSH 权限；GitHub 分支本身不包含这个大包。安装到仓库 `.cache/visual_alignment_r3_20261014/`，可用全局 `--root /path/to/release` 换位置。资源包内无绝对贴图依赖、无指向旧目录的符号链接；内置 MDL 材质由 **Isaac Sim 5.1** 提供。

## 三个入口怎么选

| 入口 | 内容和用途 |
|---|---|
| `<TASK>/static/scene.usdc` | 完整静态场景，机器人、环境、相机及任务布局一起保留。直接在 Isaac Sim 打开查看当前外观。 |
| `<TASK>/environment.usdc` | 当前任务的房间、桌子、桌纹、光线、背景、相机和固定底座；没有机器人或任务物体。 |
| `<TASK>/task_objects.usdc` | 当前任务的物体与支撑/夹具组合，保留场景坐标和材质；没有机器人、桌子、房间和相机。不是每件物品都归零的单资产库。 |
| `<TASK>/video/scene.usdc` | 视频所用的灵巧手场景，配套 `motion.npz` 等记录，用于重放。它与静态评审场景的初态/部分资产版本可能不同。 |

L3-11 仅提供原静态基线的可搬运副本，不应用 r3 外观修改，也不提供新动作。`scene_manifest.json` 记录每项入口、物体根节点、源场景哈希与贴图来源。

查看两个可复用入口：

```bash
python3 tools/visual_release.py scene L1-03 --kind task_objects
python3 tools/visual_release.py scene L1-03 --kind environment
```

环境与任务物体按任务分别导出，防止错误套用相机、布局或材质。最稳妥的组合方式是在新 USD 中把两者作为 sublayer 加载，保留 `/World` 下的层级、坐标与材质路径，再接入自己的机器人。只引用某个叶子节点时，需要连同它引用的材质和物理材质一起带入。

```python
from pathlib import Path
from pxr import Usd
bundle = Path('.cache/visual_alignment_r3_20261014').resolve()
stage = Usd.Stage.CreateNew('my_l103_scene.usda')
stage.GetRootLayer().subLayerPaths = [
    str(bundle / 'L1-03/task_objects.usdc'),
    str(bundle / 'L1-03/environment.usdc'),
]
stage.GetRootLayer().Save()
```

这是场景接入示例，不是启动完整任务的控制脚本。拆分文件保留既有几何和物理属性，但没有新增控制器或接触验收；默认任务启动器也不会自动切到这些快照。

## 渲染与原记录回放

在 Isaac Sim 5.1 Python 环境执行；同一台 GPU 仍用锁：

```bash
flock -E 75 -w 900 /tmp/task_suite_gpu.lock /path/to/isaac-sim/python.sh \
  .cache/visual_alignment_r3_20261014/render_revision.py L1-01 --mode static
# 下面是原轨迹回放，不能当作重新运行物理：
flock -E 75 -w 900 /tmp/task_suite_gpu.lock /path/to/isaac-sim/python.sh \
  .cache/visual_alignment_r3_20261014/render_revision.py L1-01 --mode video
```

渲染脚本依赖 Isaac Sim 自带的 pxr、numpy、Pillow、imageio/FFmpeg。仅播放已有姿态记录；原触觉/力数据随包保留，脚本输出的是相机画面，不自动重做网站上的触觉/力拼版。

## 文件和版本管理

- `release.json`：固定资源包位置、大小、SHA-256，以及网站快照位置。
- `scene_manifest.json`：场景和物体根节点索引。
- `revision.json`：逐项修改及未对齐部分。
- `verification.json`：文件哈希、USD 依赖和拆分入口检查结果；不是动力学验证。
- `source_snapshot.json`：从主工作目录保存的源码/资产文件清单和起点提交。
- `tools/pack_visual_release.py`：从已有 Sunday 修订场景构建可搬运包的脚本，需要原始共享盘数据和 pxr。
- `tools/verify_visual_release.py`：`python tools/verify_visual_release.py BUNDLE --usd` 在 USD 环境检查全部文件和引用闭包。
- `provenance/`：本轮建模/材质/重渲染脚本与共享配置；保留原始路径作为来源记录，不是可在任何电脑上从零重建整个项目的脚本。

包已将实际文件依赖改为包内相对路径并收集去重。完整场景是展开后的快照；保留原 prim 路径。拆分的环境/物体文件与场景共享 `textures/`，**请搬整个包，不要只复制一个 USD**。没有打包 NVIDIA 内置材质库。

本分支也保存主仓库此前尚未提交的 L1/L2 对齐、评分和测量代码；这些历史变化范围比 r3 材质修订更大。`.local.toml` 中的五项 L2 任务配置以显式文件例外纳入版本管理，模型/凭据配置仍不提交。历史 L3 源码来自另一个目录；当前分支通过完整场景快照交付其实际效果，没有把两个历史源码树强行合并。

## 验收边界与未完成项

- 本轮未调重力、质量、摩擦和控制，也未重跑动力学。原视频轨迹和力/触觉记录保留。
- L2-04 真机至少 10 颗弹珠，仿真仍为 5 颗且偏大；需要后续改资产、重跑碰撞。不能将当前 5 球评分称为真机对齐。
- 杯内明暗、部分物体轮廓、初始姿态、背景线材与真实视频仍有差异。
- 静态资产组合与视频初态分别保存；以完整场景入口为当前可查看结果，不假定通用任务配置能自动复现演示。

在线结果：https://dex-l1-l3-task-atlas-20261014.arthur12137.chatgpt.site/

## 当前检查结果

- 可搬运包 176 个文件的哈希与 57 个 USD 文件依赖检查通过；实际执行下载/解压工具后，在新目录再次检查。
- 相关 CPU 测试 59 项，58 项通过。`test_only_one_blade_entering_is_incomplete` 失败：期望一个插脚进入，实际判定为零。原始未提交工作目录中可复现同一失败，已记录在 `test_status.json`；本次打包不修改该评分逻辑。
- Python 编译和 `git -c core.whitespace=-blank-at-eof diff --cached --check` 通过（保留 USD 导出文件原有的末尾空行）。这些检查不包含新的动力学运行或任务成功验收。
