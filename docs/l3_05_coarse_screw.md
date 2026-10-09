# L3-05 双臂旋下预松螺母

```bash
python tools/launch.py run L3-05 --gui
```

现行配置仍是 `configs/tasks/l3_05_coarse_screw.toml`，名称保留以兼容已有启动命令；旧旋入任务不再作为当前入口。

原生 90 mm 膨胀螺栓与独立六角螺母，保留源尺寸、质量、SDF 碰撞。初态横放在双臂中间，第三视角看主体朝左、螺母朝右，双方空手；螺母沿原螺纹预松十圈/12.5 mm，剩约两圈。左臂从桌面拿起主体，右臂夹螺母旋出；没有预抓、固定主体、同轴导轨或自动螺旋动作。

成功条件：实测主体离桌且由左手持握，右手持握螺母并发生相对旋转/外移，螺母完全越过杆端、脱离螺杆接触。两手拿着分离零件即可结束；无固定圈数、放下、指定位置或静止等待。仅有位移、空转、移动整个组件均不足以成功。

评分中的 2 度和 0.05 mm 仅用于排除数值噪声，不是操作目标。碰撞接触负责约束实际螺纹运动，评分不操控物体。

本任务使用 1920 Hz 步进（交付机制验证条件）、原生 64/16 位置/速度求解迭代；控制周期仍为 1/12 秒。`r5_tuojing_lab_small_grasp.toml` 将可命令的最小夹爪开口从 12 mm 调为 10 mm，对应原有关节下限 5 mm/指，以便夹住原生 12 mm 套管。其他任务不受影响。

夹指采用 L2 已使用的凸分解碰撞设置，避免单凸包填平内侧夹持凹形；仅覆盖会话层，外观与运动学不变。

资产：`anchor_removable_nut_20261008`，源文件原样复制并锁定哈希。螺距、摩擦和间隙是估算，非真实紧固扭矩标定。不模拟套管膨胀、滑牙或损伤。

无 GPT 的诊断：

```bash
python -m robot_tasks.integrationtest.initial_gallery L3-05 --output reports/nut_initial
python -m robot_tasks.integrationtest.nut_clearance --output reports/nut_clearance.json
python -m robot_tasks.integrationtest.nut_probe --output reports/nut_thread
```

`nut_probe` 只在独立诊断夹具中固定主体并约束同轴，对螺母施加有限扭矩与拉力；正式任务没有这些辅助条件。机制检查、采样净空检查均不等于 R5/GPT 完整任务成功。

2026-10-09 验证记录在 `reports/l3_05_native_nut_20261009/`：原生螺纹夹具测试通过（直接拉拔不脱离，约两圈旋出）；采用凸分解夹指并深入夹持面的 R5 预置抓持测试通过，两零件随 10 mm 抬升分别上移约 10.2 / 9.8 mm，结束时仍保持两指接触。旧尖端采样抓点曾滑落，记录保留。抓持测试只证明该预置姿态能夹持，不证明从桌面自主拿起、换姿及旋出整个过程；未调用 GPT。
