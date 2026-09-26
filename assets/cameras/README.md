# 相机资产

| 文件 | 来源与用途 |
|---|---|
| `source/gemini_pro_supplier.stp` | 用户提供资料包 `三维模型/Gemini相机(客户图)/Gemini相机(客户图).stp`；资料说明为轮趣基于奥比 Gemini 模组开发的 Gemini Pro 整机 |
| `source/gemini305_v1.1.stp` | 奥比官方 Gemini 305 CAD V1.1；USB 型号，非 305g |
| `gemini_pro.usdc`、`gemini305.usdc` | 对应 CAD 转成米制、包围盒中心为原点的视觉网格 |
| 同名 `.json` | 原始文件哈希、来源、包围盒、转换精度、USD 哈希及限制 |

来源：[用户资料文件夹](https://my.feishu.cn/drive/folder/VWd4fcP9xlrawpdkDeEcZH2JnIg)、
[305 官方 CAD 说明](https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Download%20CAD%20Files%20of%20Gemini%20305)、
[305 CAD 下载](https://doc.orbbec.com/attachments/18)。供应商资料中 `Gemini/image.png` 解释了 Gemini Pro 与 Gemini 模组的关系。

外形来自 CAD。网格采用统一中性颜色，未复制原厂材质、相机内部光学系统、质量、碰撞或最终打印支架。
CAD 中心不是相机光心；两者的临时相对变换写在相机 TOML 的 `asset_pose_wxyz`，以后需按实际光心和安装图修正。
原始 CAD 坐标轴保留，毫米按 0.001 转成米；不按产品宣传尺寸强行缩放网格。
Git 保留运行所需 USD 与来源清单；`source/` 原始 STEP 留在本地，由 `.gitignore` 排除。
重新转换前按上述来源取得对应文件，并核对同名 JSON 中的 SHA-256。

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
