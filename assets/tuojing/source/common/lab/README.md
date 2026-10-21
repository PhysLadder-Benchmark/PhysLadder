# 拓境实验室 Isaac Sim V4

2026-10-05：按用户确认，将桌子更新为 **1.70 × 0.90 × 0.78 m**（长 × 深 × 高）。V3 保留。

## 机械臂和第三视角相机位置

坐标原点是**桌面左前角在地面的投影**：X 向右、Y 向桌后沿、Z 向上，单位米。桌后沿 Y = 0.900，桌面 Z = 0.780。

| 对象 | X | Y | Z |
|---|---:|---:|---:|
| R5_Left 轴心 | 0.782 | 0.825 | 0.790 |
| R5_Right 轴心 | 1.222 | 0.825 | 0.790 |
| 第三视角支架轴线 | 1.020 | -0.250 | 0.000 |
| 第三视角光心 | 1.020 | -0.215 | 1.350 |

- 双臂保持 −90° 朝向、底座前后对齐、共同肩肘折叠姿态，腕部相机分居外侧。
- 当前轴心间距 **44.07 cm**；两轴心距桌后沿均约 **7.48 cm**。这些位置继承先前多视角估计与用户对称修订，作为本版默认布局，属于估计安装位置。
- 桌面加深时保留原桌后沿在房间里的位置，向操作侧延伸 12 cm。房间内位置本身仍有建模不确定性。
- 相机支架随桌前沿向外移动 12 cm，按用户确认保持前沿外 **25 cm**；支架横向位置距左桌边 **102.0 cm**。
- **1.32 m 是用户确认沿用的照片标注点高度，1.35 m 是估计光心高度**。光轴向下约 34.1°，水平视场角暂为 70°。
- 已按用户授权综合判断，默认采用约 44.1 cm 轴距。原照片 51 cm 两端不能确定是轴心，另保留 `tuojing_lab_axis_51cm.usda`。68 + 51 + 48 = 167 cm，与本次确认的 170 cm 相差 3 cm；这三个标注不能同时当成精确轴心距离。

`placement_report.json` 保存上述坐标；`geometry_relationships.png` 为俯视位置图。相机变换和 K 矩阵见 `camera_calibration.json`，51 cm 备选场景另有矩阵文件。

## 第三视角画面

`observations/ThirdPerson_rgb.png` 是固定第三视角相机的实际 Isaac Sim 渲染。`camera_fov_comparison.jpg` 对比同一机位下的 60°、70°、85°。

相机内参、光心偏移、畸变与俯角仍为估计。精确校准需要该固定相机自身拍摄的原始画面、型号和分辨率。

## 在 L20 打开

```bash
cd /root/tuojing_lab_20261005_v4
/isaac-sim/python.sh launch_isaac.py
# 查看整体
/isaac-sim/python.sh launch_isaac.py --cutaway
# 检查双臂对齐
/isaac-sim/python.sh launch_isaac.py --camera FrontAligned
# 导出第三视角 RGB、光轴方向米制深度与矩阵
/isaac-sim/python.sh launch_isaac.py --headless --steps 120 --capture
# 腕部相机
/isaac-sim/python.sh launch_isaac.py --camera WristLeft
# 51 cm 轴距备选
/isaac-sim/python.sh launch_isaac.py --scene tuojing_lab_axis_51cm.usda
```

`tuojing_lab.usda` 可直接在 Isaac Sim 5.1 打开；运行脚本会设置折叠初始姿态。图形界面需要可用 DISPLAY。

## 资产与证据

- 两套原生 R5a 资产保持 scale = 1；每臂 6 个转动关节与 2 个夹爪移动关节。
- 腕部支架来自用户 STEP，毫米转米，尺寸约 **81.197 × 57.000 × 89.194 mm**，保留孔位及沉孔。
- 固定座按原 `link6` 基准装配，相机背面按 CAD 斜面安装。腕部 roll 暂为 +90° / −90°；壳体尺寸与光心偏移估计，载荷质量未额外加入。
- 原始 17 项资源的路径、SHA-256 与用途见 `evidence/source_index.json`。
- 雷达扫描报告列 Desk W/H/D 为 1.7/1.0/0.9 m；本次用户明确采用桌高 0.78 m。
- `reference/` 保留配准旧扫描及最新视频稀疏点云；固定房间结构沿用扫描，新家具位置优先参考最新视频。
- `reconstruction/` 和 `evidence/` 中的 SfM、束调整与残差报告是 **V3 旧桌面尺寸下的历史结果**。V4 没有重新进行束调整，旧残差不能作为 V4 定位精度证明。

## 验证与限制

构建脚本验证 480 步双臂独立动作、腕部相机刚性跟随、测试块桌面接触及 USD 依赖。`validation.json` 记录结果。`delivery_validation.json` 记录本版本的新进程加载、动作、51 cm 备选、三种第三视角视场角和双腕相机导出检查。

材质使用实拍纹理重复铺贴；相机外壳、椅子和外围设备仍有简化。摩擦、驱动参数、接触柔性和相机噪声尚未实测辨识。夹爪移动关节仍有数毫米跟踪偏差，测试未覆盖完整工作空间抓取。
