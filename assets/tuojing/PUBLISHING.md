# 资产发布与代码同步规则

## 存储职责

- `master` 保存代码、运行时 USD 覆盖、任务配置、测试、下载工具和版本清单。
- 本仓库的私有 GitHub Release 保存原始 ZIP；不把大文件塞入 Git 历史。
- `source.lock.json` 固定每个 ZIP 以及解压文件的 SHA256；`release.json` 固定仓库、Release tag、上传包、大小和未包含的包，并绑定锁文件的 SHA256。
- `source/`（按任务复制的资产）、`packages/`（完整原始交付）、`.cache/`、运行结果、视频、日志、凭据继续由 Git 忽略。Release 也不得包含它们；只上传清单列出的原始 ZIP 和公开给仓库协作者的校验元数据。
- `task_assets.json` 随代码保存，声明 `source/<任务编号>/<物品名>/` 的目录及依赖。安装工具从校验后的 `packages/` 生成各任务独立副本。
- 不直接修改原始资产。物理参数、碰撞几何和任务布局优先保存在代码运行时覆盖中；确需修改原包时，制作新包并更新锁文件及 Release 版本。

## 本次交付范围

固定版本：`assets-20261008-local-v1`，原始 ZIP 共 92,727,065 字节。

| 包 ID | 内容 | 本次发布 |
| --- | --- | --- |
| lab_v4 | 实验室场景 | 是 |
| photo_props | 拍摄道具 | 是 |
| orange_soft | 橘瓣软体 | 是 |
| tabletop_interactions | 桌面交互道具 | 否：不在此固定发布内 |
| book | 书本 | 否：不在此固定发布内 |
| physical_props_20261008 | 后续七类实物道具 | 否 |
| oriental_leaf_335ml | 东方树叶瓶 | 否 |
| anchor_removable_nut_20261008 | 可旋出螺母组件 | 否 |
| desktop_additions_20261009 | 承台、杯子和插头底座 | 否 |

此发布仍只有原来的三个 ZIP；合并后 `release.json` 绑定当前资产锁，并完整列出另外六个未发布包，不修改已有 tag 或 ZIP。

这不是完整的 L1/L2/L3 资产交付。需要缺失包的任务仍不能仅凭本次发布运行。
补齐时发布新版本，不在旧 tag 下替换或追加包以改变已发布的交付范围。

## 同事在新机器恢复

安装 Python 3.11+ 和 [GitHub CLI](https://cli.github.com/)，使用有本私有仓库读取权限的账号授权。
在仓库根目录运行：

```bash
git pull --ff-only origin master
gh auth login --hostname github.com --git-protocol ssh --web --skip-ssh-key
python3 tools/fetch_tuojing_release.py
```

下载工具使用 `release.json` 指定的版本，不跟随 `latest`。已缓存 ZIP 必须通过大小和 SHA256 校验；下载先写临时目录，校验后进入缓存。安装保留原有目录，发现本地文件被修改则报错，不覆盖。
没有网络时，缓存中已有完整的三个 ZIP 也可运行此命令，不需要调用 GitHub CLI。

已经安装旧版目录的机器可运行 `python3 tools/tuojing_assets.py --organize` 迁移；
只有部分包时加 `--packages` 明确选择。迁移不改原始文件内容，也不改变 Release 的哈希锁。

仅检查这次交付的本地文件：

```bash
python3 tools/tuojing_assets.py --packages lab_v4 photo_props orange_soft
```

不带 `--packages` 检查当前锁文件中的全部九个包；任何包缺失都会报错，不能把三个包的部分校验当成完整交付。
可加 `--usd` 检查 USD 组合依赖（需要 pxr），但文件检查不等于物理仿真或 GPT 任务测试通过。

## 维护者发布流程

1. 核对所有待发布 ZIP 和已解压文件与锁文件一致；只允许清单中的包进入发布目录。
2. 选择新的 `assets-日期-范围-vN` tag；修改原包时同步更新锁文件。生成 `release.json`，如实列出未包含的包。
3. 运行资产工具回归测试；提交代码和清单。拉取远端 master 后按正常快进或合并方式推送，禁止强推覆盖同事修改。
4. 在该代码提交上创建同仓库 Release 草稿，上传 ZIP、`release.json`、`source.lock.json` 和 `SHA256SUMS`。不得上传运行记录或环境配置。
5. 检查上传列表和大小，并将 Release 文件重新下载到临时目录逐个校验 SHA256，再发布草稿。
6. 发布后的 tag 和文件按团队规则保持不变，不用 `--clobber`。新版使用新 tag；回退代码时也回退对应清单。

Git 推送和 Release 上传不是一个原子操作。若上传或授权失败，应明确报告“代码已推送，资产发布待完成”，保留草稿和校验信息以便续传，不宣称已经交付完成。
