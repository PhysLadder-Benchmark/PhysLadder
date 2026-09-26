# R5 厂家资产

来源：[ARXroboticsX/ARX_Model 的 R5/R5a 目录](https://github.com/ARXroboticsX/ARX_Model/tree/74386441845f12af4da203843b6400951d6c5530/R5/R5a)。

- 固定提交：`74386441845f12af4da203843b6400951d6c5530`。
- `R5a/` 内 17 个文件按厂家原样保存，含整臂 URDF 和 9 个 STL 网格。
- [source.lock.json](source.lock.json) 记录来源路径、提交及每个文件的 Git blob SHA-1，用于检测文件变化和恢复同一版本。
- 原包 [package.xml](R5a/package.xml) 的许可证字段声明为 `BSD`，原始声明随包保留。

URDF 定义单侧机械臂，框架将通过两个独立实例组织双臂。此处没有将腕部相机支架 CAD 文件当作完整机器人模型，也没有修改厂家文件来添加标定数据。

| 部分 | URDF 定义 | 含义与边界 |
|---|---|---|
| 基座 | `base_link` | 各臂自身坐标系的根 |
| 机械臂 | `joint1`—`joint6`，转动关节 | 按 `base_link` 到 `link6` 的顺序配置 |
| 末端 | `link6` | 两个夹爪关节的父链接；不等同于抓取中心 TCP |
| 两指 | `joint7`、`joint8`，滑动关节 | 分别连接 `link7`、`link8`；原模型没有 mimic 联动定义 |
| 夹爪导出范围 | 每指 `0`—`0.044 m` | 不能据此直接认定实际总开口为 `88 mm` |

开口映射采用 `q_i = scale_i × width + offset_i`，其中 `width` 是两指接触面间的总距离。`scale`、`offset` 和有效开口范围需基于模型几何／现场标定填写，不能假设 `q=0` 就是两指接触。

厂家转动关节导出范围为 ±10 rad，各关节速度上限为 1000。这些宽泛数值保留在原文件中；实际运行限制单独配置到 [r5.toml](../../configs/robots/r5.toml)，并记录来源。离线检查只验证文件、结构与配置一致性；碰撞网格、质量惯量的实际准确性、驱动参数及接触稳定性需要后续导入仿真验证。

## 获取与校验

在框架根目录，用现有 Conda 环境的 Python 执行：

```text
python tools/fetch_r5_assets.py
python -m robot_tasks validate-robot --profile configs/robots/r5.toml
```

下载工具跳过内容与锁定哈希一致的文件；缺失或损坏文件会从固定提交重新获取。必要时可显式传 `--proxy http://HOST:PORT`。它只下载数据，不安装或执行厂家程序，也不要求安装 ROS。

生成的 USD 和转换清单位于框架的 `build/r5/<输入哈希>/`，由 `check-r5` 自动创建。转换时在派生 URDF 中解析网格路径、应用临时仿真限位，保留此处原始文件不变。双臂安装使用独立仿真布局，现场标定与硬件版本匹配仍待确认。
