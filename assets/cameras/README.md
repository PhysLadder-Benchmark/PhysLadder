# 相机资产

| 文件 | 来源与用途 |
|---|---|
| `source/gemini_pro_supplier.stp` | 用户提供资料包 `三维模型/Gemini相机(客户图)/Gemini相机(客户图).stp`；资料说明为轮趣基于奥比 Gemini 模组开发的 Gemini Pro 整机 |
| `source/gemini305_v1.1.stp` | 奥比官方 Gemini 305 CAD V1.1；USB 型号，非 305g |
| `gemini_pro.usdc`、`gemini305.usdc` | 对应 CAD 转成米制、包围盒中心为原点的视觉网格 |
| `source/r5_wrist_camera_mount.stp`、`r5_wrist_camera_mount.usdc` | 用户提供的 `摄像头固定座新(1).stp` 及其米制视觉网格 |
| `gemini305_r5_mounted.usda` | Gemini 305 与白色固定座的临时装配；相对引用上述本地网格，两侧共用 |
| 同名 `.json` | 原始文件哈希、来源、包围盒、转换精度、USD 哈希及限制 |

来源：[用户资料文件夹](https://my.feishu.cn/drive/folder/VWd4fcP9xlrawpdkDeEcZH2JnIg)、
[305 官方 CAD 说明](https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Download%20CAD%20Files%20of%20Gemini%20305)、
[305 CAD 下载](https://doc.orbbec.com/attachments/18)。供应商资料中 `Gemini/image.png` 解释了 Gemini Pro 与 Gemini 模组的关系。

外形来自 CAD。装配中相机为浅灰、固定座为白色，均为示意颜色；未复制原厂材质、相机内部光学系统、质量或碰撞。
CAD 中心不是相机光心；两者的临时相对变换写在相机 TOML 的 `asset_pose_wxyz`，以后需按实际光心和安装图修正。
原始 CAD 坐标轴保留，毫米按 0.001 转成米；不按产品宣传尺寸强行缩放网格。
Git 保留运行所需 USD 与来源清单；`source/` 原始 STEP 留在本地，由 `.gitignore` 排除。
重新转换前按上述来源取得对应文件，并核对同名 JSON 中的 SHA-256。

## 腕部临时装配

安装参考为用户视频 `20260927-190354.mp4`：相机通过白色固定座装在夹爪上方。
两侧暂按相同局部方向处理，随各自 `link6` 转动；视频未单独核实另一侧。
固定座原始 CAD 坐标暂与 `link6` 对齐，这是工程估计，不能当作已测量的装配变换。

| 参数 | 当前取值与依据 |
|---|---|
| 固定座尺寸 | 约 81.20 × 57.00 × 89.19 mm，来自 STEP；不是外参 |
| 相机安装孔距 | 约 20 mm；固定座与相机背部孔距相符，微小面内偏转暂忽略 |
| 观察方向 | 按固定座安装斜面，朝 `link6` 的 +X、−Z；约向下倾斜 39.3° |
| 相机外壳中心 | 约 `[82.82, 0, 80.05]` mm，相对 `link6`；依赖上述临时装配假设 |
| 相机光心 | 外壳中心向观察方向偏移 14 mm，估计值；约 `[93.65, 0, 71.18]` mm |
| 图像旋转 | 图像右侧沿 `link6` 的 −Y，暂估；需现场 RGB 核对 |
| 内参与成像 | 继续使用标称视场角估算内参、零畸变与 640×400 腕部图像，未标定 |

装配根坐标沿用 Gemini 305 的中心 CAD 坐标；`Housing` 保持原点，`Bracket` 保存固定座相对此坐标的变换。
`gemini305_r5_mounted.json` 记录 CAD 依据、估计量、组成资产哈希和装配位姿。
相机 TOML 的 `optical_pose_wxyz` 是光心相对末端的位姿，`asset_pose_wxyz` 是装配相对光心的位姿。
后续标定若改变光心相对外壳的位置，需要同步更新这两项，避免外壳随光心修正而移位。
第三视角按现场描述改为双臂正前方、中线高处俯视：世界光心暂定 `[1.6, 0, 1.4]` m，
看向 `[0.28, 0, 0.20]` m，世界 +Z 朝上，俯角约 42.3°。
其中 +X 为机械臂前方，Y=0 为两侧底座的中线，桌面 Z=0；距离、高度和俯角均为仿真估计，尚未现场标定。
各仿真任务的窗口默认视角采用同一观察位置与目标；GPT 的 `overview` RGB、深度及内外参来自上述相机配置。
两侧腕部安装参数保持不变，三路相机均保留 `provisional_uncalibrated` 标记。

## 转换

本机在独立 Conda 环境 `camera-assets` 完成转换，未改动 Isaac Sim 环境依赖。
需要 CadQuery 2.8.0、numpy、usd-core；USD 文件可直接被 Windows Isaac Sim 加载，不需要运行时安装 CadQuery。
可用 `conda env create -f tools/camera_assets_environment.yml` 单独准备转换环境。

```bash
conda run -n camera-assets python tools/convert_camera_cad.py \
  --source assets/cameras/source/gemini305_v1.1.stp \
  --output assets/cameras/gemini305.usdc \
  --origin https://doc.orbbec.com/attachments/18
```

几何转换参考 [CadQuery STEP 导入文档](https://cadquery.readthedocs.io/en/latest/importexport.html)。
RGB 采用 [Isaac Sim 5.1 OpenCV 相机模型](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_camera.html)，
当前内参由 [Gemini 标称 RGB 视场角](https://orbbec.com.cn/index/Product/info.html?cate=38&id=16) 和
[Gemini 305 标称 RGB 视场角](https://www.orbbec.com/gemini-305/) 估算。相机内参的分辨率必须与对应 RGB 输出一致。
