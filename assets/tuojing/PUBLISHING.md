# 资产发布与代码同步规则

## 存储职责

- `master` 保存代码、运行时 USD 覆盖、任务配置、测试、下载工具和版本清单。
- 本仓库的私有 GitHub Release 保存原始 ZIP；不把大文件塞入 Git 历史。
- `source.lock.json` 固定每个 ZIP 以及解压文件的 SHA256；`release.json` 固定仓库、Release tag、上传包、大小和未包含的包，并绑定锁文件的 SHA256。
- `source/`、`.cache/`、运行结果、视频、日志、凭据继续由 Git 忽略。Release 也不得包含它们；只上传清单列出的原始 ZIP 和公开给仓库协作者的校验元数据。
- 不直接修改原始资产。物理参数、碰撞几何和任务布局优先保存在代码运行时覆盖中；确需修改原包时，制作新包并更新锁文件及 Release 版本。

## 本次交付范围

固定版本：`assets-20261008-local-v1`，原始 ZIP 共 92,727,065 字节。

| 包 ID | 内容 | 本次发布 |
| --- | --- | --- |
| lab_v4 | 实验室场景 | 是 |
| photo_props | 拍摄道具 | 是 |
| orange_soft | 橘瓣软体 | 是 |
| tabletop_interactions | 桌面交互道具 | 否：本机缺少原包和解压文件 |
| book | 书本 | 否：本机缺少原包和解压文件 |
| desktop_additions | 2026-10-09 黑色陶瓷杯、圆柱承台、插头底座 | 否：本机已安装，尚未发布到 Release |

这不是完整的当前 L1/L2 资产交付；更新后的 L1-01 还需要单独提供的 `desktop_additions`。需要缺失包的任务仍不能仅凭本次发布运行。
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

仅检查这次交付的本地文件：

```bash
python3 tools/tuojing_assets.py --packages lab_v4 photo_props orange_soft
```

不带 `--packages` 仍检查锁文件中的全部六个包；本机缺少两个包时会报错，不能把部分校验当成完整交付。
可加 `--usd` 检查 USD 组合依赖（需要 pxr），但文件检查不等于物理仿真或 GPT 任务测试通过。

## 2026-10-09 本地新增包

当前 `release.json` 仍指向已发布的三个旧包，下载地址、ZIP 哈希及旧 tag 保持不变。
其锁文件摘要和 `omitted_packages` 已随当前代码更新，将新包明确列为未发布。
这是客户端清单对新增本地依赖的记录，并未更改云端 Release 文件。

拿到 `新增三类资产_圆柱杯子插头底座_20261009.zip` 后，将它放在仓库根目录，执行：

```bash
python3 tools/tuojing_assets.py --archives . --packages desktop_additions --usd
```

新包没有外层目录，锁文件以 `archive_root: ""` 显式表示；安装器仍核对所有成员路径、
完整文件清单和 SHA256，安装至 `assets/tuojing/source/desktop_additions_20261009/`。
不修改或重新打包原 ZIP。向协作者正式交付时，应按下面流程发布新 tag，纳入此包，
不能仅推送代码便宣称 L1-01 的新资产也已上传。

## 维护者发布流程

1. 核对所有待发布 ZIP 和已解压文件与锁文件一致；只允许清单中的包进入发布目录。
2. 选择新的 `assets-日期-范围-vN` tag；修改原包时同步更新锁文件。生成 `release.json`，如实列出未包含的包。
3. 运行资产工具回归测试；提交代码和清单。拉取远端 master 后按正常快进或合并方式推送，禁止强推覆盖同事修改。
4. 在该代码提交上创建同仓库 Release 草稿，上传 ZIP、`release.json`、`source.lock.json` 和 `SHA256SUMS`。不得上传运行记录或环境配置。
5. 检查上传列表和大小，并将 Release 文件重新下载到临时目录逐个校验 SHA256，再发布草稿。
6. 发布后的 tag 和文件按团队规则保持不变，不用 `--clobber`。新版使用新 tag；回退代码时也回退对应清单。

Git 推送和 Release 上传不是一个原子操作。若上传或授权失败，应明确报告“代码已推送，资产发布待完成”，保留草稿和校验信息以便续传，不宣称已经交付完成。
