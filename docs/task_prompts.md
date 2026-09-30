# GPT-6 物理交互任务 Prompt

本文保留任务级操作指导的候选稿，不是运行时自动加载的 Prompt。当前已实现任务使用共享系统操作说明与任务目标分离的版本；以 `policies/operating_notes.py`、任务 TOML 和每次运行保存的实际指令为准。L1-01 的单孔宽容版本及当前评分见 [L1 任务说明](l1_tasks.md)。

目标是在任务原有成功标准与资源约束内，提高完整完成任务的机会。以下是首版候选提示，尚未通过实测证明达到最高成功率。它们包含操作建议，结果应标注为“有操作提示的测评”，不与只有一句目标指令的结果混用。

范围依据本次对话明确给出的任务：L1 原有 8 项及新增柔软花茎插花，L2 保留 7 项，L3 原有 8 项及从 L2 移入的擦桌子，共 25 项。沿用已有编号；插花和擦桌子的新编号待确认。未取得最终名称与定义的额外 5 项 L3、未定义的 L0，不补造。此文件不代表飞书最新任务表已全部同步，也不预设各层实际成功率必然递减。

提示结构采用明确目标、必要上下文和可检验结果，并控制无关约束；依据 [OpenAI Docs：Prompt engineering](https://developers.openai.com/api/docs/guides/prompt-engineering) 与 [GPT-6 Astra 提示设计说明](https://developers.openai.com/blog/rethinking-skills-and-prompts-for-gpt-6-astra)。机器人操作建议是结合本项目任务定义与当前接口设计的候选方案，不是这些资料已验证的机器人成功率结论。

## 使用方法

模型端统一使用英文：发送英文的“共用执行约定＋当前任务正文”，不要把整份任务库发送给模型。代码块内为实际 Prompt；中文标题、“运行前填写”和“实现条件”供实验人员查看，不发送给模型。

将正文中的 `{{parameter_name}}` 替换为本轮实际值；物体描述、目标要求和文字反馈也统一用英文，数值保留约定单位，物体 ID、相机名、字段名和接口枚举保留原值。缺省信息用明确英文填写，例如 `not provided; infer from the available observations`，不能保留占位符直接运行。语言统一是实验一致性约定，不表示已证明英文优于中文；若比较语言，应单独设计对照实验。

这些参数用于指明物体、目标、允许操作和公开的完成标准，不提供隐藏的物体真值坐标、最佳抓点或评分结果。描述目标可用名称、可见标记或实际提供的参考图。若参数没有确定，先完成任务定义，不让模型猜测成功阈值。

现有 `GPTPolicy` 已提供机器人身份、关节限制、坐标／单位约定和 JSON 动作协议。本文件作为任务 `instruction` 的候选内容，不替换该协议。当前接口只有关节位置、末端位姿和夹爪总开口；没有目标力／力矩、显式速度、触觉或六维力反馈。改变夹爪开口不等于指定夹持力，减小位移步长也不等于有速度控制。

力控类任务的提示分清“操作目标”和“接口实现”：仅凭 RGB 与位姿可尝试可见的稳定操作，不能据此报告精确控力能力；需要力信号的任务必须先接入真实可用的观测与控制能力。文中的实现条件均不是已完成声明。

`run GPT-PICK-PLACE`、`run L1-01`～`run L1-08` 已接入任务场景与评分；唯一手动集成入口仍是 `check gpt-pick-place`。其他候选任务还需实现相应场景、复位、评分与停止处理，不能只替换 instruction 就视为已实现。本文中的动作建议不能靠文字新增接口字段，候选稿与实际运行条件必须分开记录。

## 共用执行约定

```text
You control an ARX R5 robot with two-finger grippers to complete the task below. Use the provided RGB images, robot state, execution feedback, and available history to choose grasp locations, approach directions, action order, and recovery actions. Use only the arms and action interfaces enabled for this episode.

Return only the JSON required by the system action contract, without code, undeclared fields or text outside the JSON. If the contract includes a note, use one or two brief sentences to describe the visible situation and purpose of this decision. Follow the declared coordinate frames, units, and action limits. End-effector poses are absolute targets, and quaternions must have unit length. When retaining orientation, preserve the measured orientation from feedback. Arms without new commands retain their previous targets. request_stop ends decision-making; it does not automatically open grippers or retract the robot.

Identify the current task stage and the information needed for the next action. When a view is occluded, first use the other available views. If visibility remains insufficient, adjust the viewpoint or object pose while preserving established support. Do not treat visual estimates as exact measurements or assume access to depth, force, tactile signals, or ground-truth object coordinates that have not been provided.

Make useful progress in clearly free space. Near objects, openings, edges, or changing contacts, use smaller translation or rotation increments and inspect feedback before proceeding. IK does not guarantee straight-line motion or collision avoidance; maintain clearance for the grippers, objects, and the other arm. Preserve any support that is still needed until a replacement support has been established.

A completed joint target does not establish task success. During a grasp, contact may prevent the fingers from reaching the requested closing position, so executing does not by itself mean grasp failure. Assess object motion, gripper state, and actual feedback together. If expected progress is absent, make an evidence-based change to alignment, grasp, or approach instead of repeatedly advancing the same blocked target.

Actively complete subsequent steps and recoverable errors within the episode budget. Before ending, inspect the required final state and release or support relationships. Use provided timestamps to verify hold durations. Empty commands with request_stop=false retain previous targets while waiting; first ensure those targets will not continue squeezing or pushing. Return empty commands with request_stop=true only when the goal is satisfied, the budget is exhausted, or further progress is no longer possible. A stop request is not a success claim; the external evaluator determines the outcome.
```

## L1：任务相关 Affordance 与几何操作

本组使用统一基础控制设置，主要保留自主选取操作部位、姿态与几何对准。提示提供操作原则，不指定抓点真值或预制轨迹。插花为新增柔性候选，最终分层需结合实际物体确认。

### L1-02 异形块嵌槽

运行前填写：目标块、目标槽、就位高度／表面关系、保持时间。实现条件：轻放配合，槽的碰撞几何保留凹形结构。

```text
Fully seat {{shaped_block_id}} in {{target_recess_id}}, satisfying {{seated_height_or_surface_relationship}}. After release and gripper withdrawal, it must remain stable for {{hold_time_s}} seconds without raised edges. This is a loose-fit placement task; do not achieve seating through press-fitting, deformation, or movement of the recess fixture.

Identify the matching outlines, asymmetric features, and front/back orientation. Choose a grasp that allows full seating. Align position and orientation using multiple views to reduce errors caused by perspective, then lower incrementally.

If one side contacts first, the block tilts, or lowering makes no progress, lift it clear of the constraint and correct its pose before trying again. Do not force it flat with the gripper. After release, inspect the edges relative to the recess surface and confirm full seating with stable support.
```

### L1-03 插头插接

运行前填写：插头、对应接口、插接到位标志、保持时间。实现条件：未通电、低插入力测试件；本任务不包含卡扣解锁或电气功能验证。

```text
Insert {{plug_id}} into {{matching_socket_id}} until {{seating_indicator}} is satisfied. After releasing and withdrawing the gripper, the connection must remain seated for {{hold_time_s}} seconds. The test connector is unpowered; keep the socket fixture in place.

Identify the plug's mating end, socket outline, keying orientation, and graspable housing. Avoid grasping pins, contacts, or locations that obstruct insertion travel. Align the mating axis and orientation before advancing incrementally. If a cable is present, leave enough slack to prevent it from pulling the plug out of alignment.

If only one side enters, the fixture moves, or advancement repeatedly fails, retract and check orientation and lateral alignment. Do not force an obviously incorrect orientation. Verify both the seating indicator and retention after release; touching the socket face is not completion.
```

### L1-04 薄片入槽

运行前填写：薄片、狭槽、接收区域标识。实现条件：刚性薄片与真实槽口碰撞，接收区可判定；不包含弯折薄片。

```text
Pass {{thin_plate_id}} completely through {{slot_id}} into {{receiving_region_id}}. Do not bend the plate, widen the slot, or bypass the slot through another opening.

Identify the plate's thickness direction and the slot's long axis. Choose a stable grasp that leaves room for the leading end to enter. Align the plate plane and insertion direction with the slot before approaching the entrance for smaller corrections.

Choose the release point according to whether the gripper can continue through the available space; do not wedge the gripper in the entrance. If the plate catches an edge or jams at an angle, retract while it remains controllable and realign rather than forcing it forward. Confirm that the entire plate has passed through the slot into the receiving region; partial insertion is insufficient.
```

### L1-05 钥匙插锁

运行前填写：钥匙、锁孔、插入深度／就位标记。实现条件：顺畅配合，锁体固定，仅插入。

```text
Insert {{key_id}} into {{keyhole_id}} to {{insertion_depth_or_seating_marker}}. Insert only: do not turn the key or attempt to unlock the lock. Keep the lock fixture fixed.

Identify the key handle, tooth or groove orientation, and keyhole outline. Choose a grasp that leaves the required insertion length exposed. Determine the correct face and rotational orientation, align the key axis with the keyhole, and advance incrementally.

If the tip does not enter, the orientation is incompatible, or insertion is blocked, retract outside the entrance and reassess orientation. Avoid levering the key sideways against the keyhole. Once the required position is reached, stop advancing and verify the final inserted state. Release only if separately required by the task.
```

### L1-06 工具柄定点抓取

运行前填写：工具、允许抓持的柄部判定、规定抓取方向、放置区域。实现条件：轻质刚性工具与柄部接触判定。

```text
Pick up {{tool_id}} and place it in {{placement_region_id}}. Gripper contact must lie within {{allowed_handle_region_definition}} and satisfy {{required_grasp_direction}}. The grasp must support the tool's subsequent normal use. Do not complete the transfer by grasping bristles, a scraper's working edge, or another functional end.

Distinguish the handle from the working end. Choose the specific grasp using the center of mass, available gripper clearance, and the intended placement. After closing, perform a short lift and check whether the tool moves stably with the gripper. If it slips or its orientation is unsuitable, restore stable support before adjusting the grasp.

Keep the working end clear of surrounding objects during transport. Establish stable support in the target region before releasing and withdrawing. Confirm that the tool has not rolled away and that the required handle region and grasp direction were used.
```

### L1-07 挂环上钩

运行前填写：刚性环、指定挂钩／钩口、保持时间。实现条件：开口刚性钩，无弹性卡扣，孔洞可真实穿过。

```text
Pass {{rigid_ring_id}} through {{specified_hook_and_opening_id}} and hang it. After gripper release, the hook must independently support the ring for {{hold_time_s}} seconds. Holding the ring up with the gripper is not a completed state.

Identify the ring opening, hook entrance, and load-bearing direction. Choose a grasp that leaves the threading path clear. Align the ring plane with the opening, then thread and seat it incrementally while checking clearance between the gripper and hook.

If the hook tip blocks the ring, withdraw along a reversible path and adjust orientation instead of using large swinging motions. Move the ring onto a load-bearing part of the hook, gradually transfer support, release, and withdraw. Use a new observation to confirm that the ring has not slipped out through the opening.
```

### L1-08 刚性积木定姿堆叠

运行前填写：积木标识、目标结构文字／实际提供的参考图、位置与朝向容差、保持时间。实现条件：支撑与稳定性评分；不能只用积木中心位置判断成功。

```text
Use {{block_set_ids}} to build {{target_structure_description_or_provided_reference_image}}, satisfying {{public_position_and_orientation_tolerances}}. After all grippers withdraw, the structure must stand independently for {{hold_time_s}} seconds.

Choose the assembly order, supporting faces, and grasps from the target structure. Build a stable base first. Before each placement, identify the block's bottom face, the supporting area below it, and the required orientation. Do not hide unresolved problems in lower layers by adding upper layers.

Lower incrementally near the supporting surface. Confirm support before releasing rather than dropping the block from above. After release, inspect position, tilt, and movement of lower blocks. If correction is needed, retrieve and replace the unstable block instead of continuing to build on a structure still held up by a gripper.
```

### 插花：柔软花茎、较重花头（新增，编号待确认）

运行前填写：实际花材、目标花瓶／瓶口、目标插入与花头位置、保持时间；由接口声明可用单臂或双臂。实现条件：先确认真实物体与柔性仿真，不能以刚性花茎替代。若持续形变修正成为主要难点，需重新审核 L1 归类。

```text
Place {{flower_id}} into {{vase_and_opening_id}}, satisfying {{required_insertion_and_flower_head_position}}. After robot release, the vase must naturally support the flower for {{hold_time_s}} seconds. The stem bends and droops like a rope, and the flower head is relatively heavy. Keep both intact.

Observe the flower head, receptacle, stem, and existing support relationships. Choose grasp or support locations that give control over the part being manipulated. Do not assume that holding an end far from the flower head will make the whole flower behave like a rigid rod. Avoid crushing petals or kinking the stem.

Plan around the available arms and support from the vase. If two arms are enabled, one may support a robust region near the flower head while the other guides the insertion end. With one arm, use permitted existing supports in stages; do not assume a second hand is supporting the flower. Account for the observed drooping shape, maintain control over a short segment near the insertion end, and feed it through the opening incrementally while transferring support.

If the stem droops out of control, kinks, or catches at the opening, restore support before changing the grasp or direction. Do not pull on the flower head to force insertion. Confirm that the stem is inside the vase, the head meets the target position requirement, and the vase is stable before gradually releasing and checking the final state.
```

## L2：稳定交互中的受力与运动调节

沿用老师修订后的边界：保留自主选择抓持与几何对准，利用物体自然存在的阻力、负载和易损性；建立主要操作关系后保持稳定。以下提示不预先给出最佳接触点。若正式协议明确提供抓点或方向，应把它列为模型可见的任务参数，而不是在后台替模型完成抓持。

### L2-01 双臂拔笔帽

运行前填写：马克笔、笔身与笔帽各自的放置区域。实现条件：双臂真实抓持和拔出阻力；当前开口控制不能直接指定夹持力。

```text
Remove the cap from {{marker_id}}. Place the marker body in {{marker_body_placement_region}} and the cap in {{cap_placement_region}}. Keep both parts intact without slipping or dropping them. Do not use the writing tip as a grasp or support point.

Identify the cap/body boundary and connection axis. Choose stable grasp locations for both arms that do not interfere with each other. Establish both grasps before incrementally increasing separation along the connection axis. Avoid gripper collisions and bending the marker.

Use relative motion in the images, gripper opening, and provided feedback to distinguish cap separation from the entire marker slipping within a grasp. If slipping occurs, restore a stable grasp first. If the axes are misaligned, correct alignment before continuing rather than advancing the same blocked target. Reduce subsequent motion once separation occurs, then place, release, and verify both parts.
```

### L2-02 双臂夹持力交接

运行前填写：被交接物、移交侧、接收侧、目标交接区域、独立持物时间。实现条件：双臂夹持与负载转移；定量评估夹持力还需实际力反馈／控制及评分。

```text
Transfer {{object_id}} from {{donor_arm}} to {{receiver_arm}} within {{handover_region}}. After the donor fully releases and withdraws, the receiver must hold the object independently and stably for {{independent_hold_time_s}} seconds. Do not drop, visibly deform, or damage the object.

Choose grasp locations and a handover pose that leave room for the receiving gripper. While the object remains reliably supported, approach with the receiver, close its gripper, and inspect the grasp relationship. Sending a closing command alone does not establish a successful receiving grasp.

Once the receiver has made contact and can constrain the object, gradually reduce the donor's grip and observe for slip within a recoverable range. Fully release the donor only after the receiver has established stable support. If the object slides or rotates, restore donor support before correcting the receiving grasp. Avoid pulling the object in opposing directions during load transfer.

Verify that the receiver holds the object independently and the donor is no longer providing hidden support. Use timestamps to confirm the required stable hold before ending.
```

### L2-03 抽取中层并支撑上层

对应原“固定接触点的抽取与支撑”；运行前填写：中层物体、上层物体、最终放置区域。实现条件：保留自主抓持；若实验固定抓点，需公开说明；不能把上层物体锁定为不受力的静态物体。

```text
Extract {{middle_object_id}} from the stack while preventing {{upper_object_id}} from falling onto the table or tipping over. Finally, place the middle and upper objects stably in {{respective_placement_regions}}.

Identify the current load-bearing relationships, extraction direction, and contact regions that can support the upper object without blocking the middle object's exit. Establish reliable support with one arm and grasp the middle object with the other. Do not begin substantial extraction before support is established.

Extract incrementally along a feasible exit direction. Adjust support according to visible sinking, tilting, or movement of the upper object with the middle object. Avoid pressing the upper object against the middle one and thereby increasing extraction resistance. If the upper object becomes unstable, stop increasing the extraction distance, restore support, and then decide whether to retract slightly or adjust the pose.

After full separation, continue supporting the upper object while placing both parts in an order that avoids collisions. Release them only after stable support is established. Verify that the upper object did not fall onto the table during extraction and that both objects are now stably placed.
```

### L2-04 持碗动态负载保持

运行前填写：碗、持碗侧、操作区域、投料完成信号及完成后保持时间。实现条件：倒料端按固定脚本执行；其完成状态需要实际提供给模型；如测定量力调节需补充接口。当前暂停式调用不能代表连续实时控制能力。

```text
Use {{holding_arm}} to hold {{bowl_id}} steadily within {{operation_region}} while it receives material poured by a fixed external script. The external process controls pouring; do not take over the pouring side. Prevent spillage, slipping, tipping, or crushing of the bowl throughout the task.

Before pouring begins, choose a grasp or support arrangement that accommodates the bowl and increasing load. Keep the opening accessible and approximately level. Once the main hold is established, maintain that relationship rather than repeatedly changing the grasp to chase individual falling items.

Adjust the hold and supporting pose using bowl orientation, visible slip, signs of changing load, and actual feedback. Use only adjustments expressible through the available interface. Do not treat gripper opening as measured force or invent force values without observations. If the bowl tilts or slips downward, restore stability before making large movements that could cause further spillage.

End only after receiving the actually provided {{pouring_completion_signal}} and maintaining a stable hold for another {{hold_time_s}} seconds. Keep holding the bowl when stopping; do not automatically release it. The evaluation includes stability, integrity, and spillage throughout the process.
```

### L2-06 夹面包

运行前填写：面包、目标区域、公开的形变／损伤限制、保持时间。实现条件：真实面包或经验证的压溃模型；弹性回弹物体不能直接替代真实压坏判据。

```text
Transport {{bread_id}} to {{target_region}}, satisfying {{deformation_and_damage_limits}}. After release, it must remain stable for {{hold_time_s}} seconds. Avoid slipping, crushing, tearing, or substantial crumbling throughout the task. Do not establish a grasp by piercing or flattening the bread.

Choose a forgiving grasp region that distributes the load, using the bread's shape, existing support, and gripper contact area. Do not assume that the narrowest region is the easiest to grasp. Establish contact gradually and use a short lift to assess holding stability while observing indentation and overall deformation.

If the bread slips while deformation remains acceptable, make small evidence-based grasp adjustments. If indentation keeps increasing or a region collapses, first restore table support and reduce compression, then improve the grasp location or support arrangement rather than simply reducing the opening further. Use coherent, incremental transport targets to avoid large swinging motions.

Establish support in the target region before gradually releasing and withdrawing. Inspect the shape, integrity, and position after release. Merely retaining the bread in the gripper does not prove that it was not crushed.
```

### L2-07 抽屉推拉与自然阻力适应

对应原“已知方向的变阻力抽屉推拉”；运行前填写：抽屉、开合目标与最终位置、是否公开滑轨方向。实现条件：采用真实存在的阻力条件，不为此提示额外制作调阻装置；准确力限值评价另需传感与接口。

```text
Operate {{drawer_id}} by its handle to satisfy {{opening_closing_requirement_and_final_position}}. Keep the cabinet or base in place and avoid impacts at travel limits, derailment, damage, and handle slip. Rail-direction information is {{provided_rail_direction_or_observe}}.

Choose a grasp and approach for the handle. Identify the motion axis from visible structure and any provided direction, then push or pull incrementally along it while maintaining the grasp. Reduce increments near a travel endpoint. Continuing to advance a target against a mechanical stop is not a completion strategy.

Use drawer progress, cabinet movement, and relative slip between gripper and handle to distinguish resistance from grasp instability or directional error. When blocked, first correct lateral misalignment or restore the grasp before changing the advancement target. If force feedback is available, use the actual signal to constrain adjustments; do not invent force values from vision.

Stop advancing once the required position is reached. Release and withdraw according to the permitted final state. Confirm that the drawer retains its position and the cabinet has not been dragged as a whole.
```

### L2-09 弹性件定点按压

运行前填写：弹性件、公开按压点／轴线、目标力范围、保持时间、允许过冲。实现条件：**当前接口条件未满足**；定量目标力任务必须接入已标定力信号及可用的调节方式。仅看变形不能证明达到目标力。

```text
Press {{elastic_component_id}} at {{specified_press_point_and_axis}}. Maintain the measured pressing force within {{target_force_range_n}} for {{hold_time_s}} seconds, with overshoot no greater than {{allowed_overshoot_n}}, then fully unload. Do not cause permanent damage.

Align the pressing tool or gripper contact face with the target region and maintain the specified axis during approach. After contact, adjust using the actual provided force signal rather than treating target displacement or gripper opening as force. Reduce adjustment magnitude near the target range, and continue checking timestamps and measured force throughout the hold.

If force rises faster than expected or approaches the overshoot limit, reduce penetration or use an explicitly available force-target command. If lateral slip occurs, reduce contact loading and restore alignment. After completing the hold, unload incrementally and confirm that force falls, contact is released, and the component recovers as required.

Without actual force feedback, do not claim completion of this quantitative force task. Visible displacement alone is not evidence of the required force.
```

## L3：变化接触下的持续联合调整

本组保留自主抓持、几何对准和稳定交互，进一步根据啮合、形变、支撑与阻力变化持续调整。任务正文描述操作目标与可观察的恢复线索；尚未提供的力／力矩信息不能由模型自行假定。

### L3-03 自主抓持与重抓的旋拧瓶盖

运行前填写：瓶子、旋开方向信息、瓶身与瓶盖放置区域。实现条件：真实螺纹／旋松与脱离机制；不能靠旋转达到角度后直接删除约束。准确扭矩控制仍需扩展接口。

```text
Unscrew the cap of {{bottle_id}} until it is completely detached. Place the bottle and cap in {{respective_placement_regions}}. Keep the bottle stable and preserve the cap and bottle opening. Opening-direction information is {{provided_opening_direction_or_observe}}.

Identify the bottle/cap boundary, rotation axis, and usable grasp regions. Stabilize the bottle with one arm and grasp the cap with the other, checking for interference. Rotate incrementally about the cap axis rather than sweeping it around an incorrect pivot. Adjust axial position as the cap loosens instead of continually pressing it onto the opening.

Distinguish cap rotation relative to the bottle from whole-bottle rotation or gripper slip on the cap. If no actual loosening occurs, first restore bottle stabilization, check direction, or improve the cap grasp. Near a wrist range limit, regrasp only while the cap has reliable support and its state can be maintained; do not accumulate infeasible large-angle targets.

When loosening evidence is present, make a small test lift to confirm that no threaded constraint remains before moving the cap away. Rotating through an angle or moving the gripper alone does not establish cap detachment.
```

### L3-04 带卡扣翻盖盒的开启与扣合

运行前填写：盒子、需取出的物体及放置区域、关闭扣合标志。实现条件：卡扣解锁与闭合保持机制，铰链与可取物场景。

```text
Open {{latching_box_id}}, remove {{contained_object_id}}, and place it in {{object_placement_region}}. Then close the lid and satisfy {{latch_engagement_indicator}}. Preserve the box, hinge, and latch.

Identify the latch, hinge, usable opening edge, and box support locations. Choose how to stabilize the box and release the latch. Do not make a large opening motion while the latch still constrains the lid. Use latch displacement and actual lid motion relative to the box to assess release.

Open along the hinge's permitted motion, leaving clearance for object removal and gripper withdrawal. If the lid cannot remain open stably, support it with the other arm. After removing the object, confirm that it and both grippers are clear of the closing region before closing along the hinge path.

Adjust alignment and pressing location according to lid-edge alignment and latch contact. Do not keep pressing one side while the other is misaligned. Release the applied load and verify latch engagement, edge gaps, and retention. Contact between lid and box alone does not establish successful closure.
```

### L3-05 对准、啮合并旋入螺丝

运行前填写：螺丝、目标螺纹孔、允许工具、旋入方向信息、目标就位标准。实现条件：实际螺纹啮合、旋转推进与卡住状态；未提前啮合，准确轴向力／扭矩限制需扩展。

```text
Using {{allowed_tools_or_gripper_only}}, insert and thread {{screw_id}} into {{target_threaded_hole_id}} from an initially unengaged state until {{target_seating_criterion}} is satisfied. Threading-direction information is {{provided_threading_direction_or_observe}}. Preserve the threads and parts. Do not force the screw sideways, bypass thread engagement, or merely rest it on the hole.

Identify the screw's working end, hole axis, and a grasp or tool engagement that can transmit rotation. Bring the screw and hole axes into alignment, then use small axial approaches and angular increments to establish normal thread engagement. Check whether rotation produces appropriate axial advancement; gripper rotation alone does not prove engagement.

If the screw tilts, fails to advance, or actual feedback indicates abnormal resistance, first relieve excessive pressing. If needed, reverse slightly and withdraw before restoring alignment and initial engagement. Do not simultaneously increase pressing and rotation by large amounts. Once engaged, coordinate axial targets and rotation with observed progress. Preserve support during any regrasp and avoid pulling against the threads.

Reduce increments near the seating criterion. Stop tightening when it is met, withdraw the tool or gripper, and confirm that the screw remains correctly seated. Without torque measurement, do not claim a precise tightening torque.
```

### L3-06 剥橘子

运行前填写：橘子、允许使用的工具、去皮范围、果肉损伤限制、果皮放置区域。实现条件：果皮分离、断裂与果肉损伤可评估；R5 夹爪能否建立起剥边缘需先验证，不默认人工预剥或预切。

```text
Use {{allowed_tools_or_gripper_only}} to peel {{orange_id}} over {{required_peeling_extent}}, satisfying {{pulp_damage_limit}}. Place removed peel in {{peel_placement_region}}. Do not drop the fruit or use crushing or breaking apart the pulp as a way to remove the peel.

Observe the peel, natural openings or locations where a starting edge could be formed, and the fruit's current support. Choose how to stabilize the fruit and establish a starting point using only permitted tools. Do not assume an unseen pre-cut opening exists.

Once a peel edge can be grasped, work in small sections. Choose new grasp locations and directions according to the advancing separation boundary, keeping the action on the peel rather than the pulp. Adjust actions using visible deformation, separation progress, and actual feedback. At a resistant region, change the local direction or regrasp instead of pulling off a large section together with the pulp.

As more pulp is exposed, adjust support to avoid crushing it with the grippers. Check remaining peel and damage progressively. Stop once the required extent is reached, keep the fruit stable, and place the removed peel in its designated region.
```

### L3-07 贴纸／胶带剥离

运行前填写：贴纸／胶带、基底、剥离范围、完整性与基底损伤限制、允许工具、剥离物放置区域。实现条件：粘附与剥离前缘可变化；不起边版本不能默认为已起边。

```text
Using {{allowed_tools_or_gripper_only}}, peel {{sticker_or_tape_id}} from {{substrate_id}} over {{required_peeling_extent}}. Satisfy {{peeled_material_integrity_and_substrate_damage_limits}} and place the removed material in {{placement_region}}. Do not complete the task by moving the substrate out of view, cutting away the substrate, or tearing regions that must remain intact.

Check whether a graspable edge exists. If not, establish one using permitted actions rather than assuming it has already been lifted manually. Stabilize the substrate and avoid grasping it together with the edge.

After establishing a grasp, adjust pulling direction and peel angle incrementally according to the moving separation front. If only the substrate moves, the material stretches, or a local tear develops, restore support and adjust the grasp, direction, or increment rather than increasing the pull in the same direction.

When regrasping, use the separated portion to maintain control and avoid pressing it back onto the adhesive surface. Verify that the required region is actually peeled and that the substrate and removed material satisfy the integrity requirements before placing the removed material.
```

### L3-08 柔性布面拉链开合

运行前填写：拉链、打开或关闭的目标位置、端部是否已啮合／需自行接合、最终放置要求。实现条件：拉链滑块、链齿与柔性布面联动，不能把布料两边的位置接近直接判为闭合。

```text
Operate {{fabric_zipper_id}} to {{target_open_or_closed_position}}. The initial end-engagement condition is {{zipper_end_engagement_condition}}, and the required final placement is {{final_placement_requirement}}. Preserve the fabric, pull tab, and teeth. Avoid tooth separation and jamming; moving the entire fabric does not count as opening or closing the zipper.

Identify the pull tab, slider, local zipper direction, and usable fabric support regions. Choose how one arm grasps the tab while the other arm or a permitted existing support stabilizes the fabric. If the task requires joining the zipper ends, first align and engage them before pulling.

As the slider moves and the fabric deforms, adjust the pulling direction in segments to follow the current local zipper direction rather than continuing along the initial direction. Observe actual slider progress, fabric entering the slider, and excessive tension on either side. If a problem appears, first relieve the pull, retract slightly if appropriate, and restore support and direction before continuing.

Reduce increments near the endpoint. Verify tooth engagement or separation and the achieved opening length before releasing according to the required final state. A tab near the endpoint is insufficient if the teeth remain incorrectly separated or fabric is trapped.
```

### L3-09 带卡扣连接器解锁拔出

运行前填写：连接器、允许的解锁操作、连接器放置区域。实现条件：卡扣按压、解锁和拔出阻力联动；未通电测试件，基座可稳定支持。

```text
Use {{permitted_unlatching_operation}} to release the latch of {{connector_id}}, fully unplug it, and place it in {{placement_region}}. Keep the base in place and preserve the latch, mating interfaces, and cable. Breaking the latch to separate the connector is not permitted.

Identify the latch, graspable housing, withdrawal direction, and base support region. Choose how to stabilize the base, actuate the latch, and hold the connector. Avoid pulling on the cable to transfer load directly to the interface.

Use only controlled small test motions along the withdrawal direction. Assess release using visible latch displacement and actual relative motion. If the connector does not withdraw, first check whether pressure is applied at the correct location, release is maintained, and the axes are aligned. Do not simply increase the pulling target.

Maintain the contact needed for unlatching until the connector has passed the locking region, then adjust the grasp and withdrawal direction as the contact state changes. After separation, place it stably and confirm an intact latch, an unmoved base, and genuinely separated interfaces.
```

### L3-10 柔性密封圈入槽

运行前填写：密封圈、环槽、完整就位标准、允许拉伸与损伤限制。实现条件：柔性环形变、自接触、槽边接触与保持；夹爪／允许工具能进入局部装配区域。

```text
Install {{seal_ring_id}} completely into {{annular_groove_id}}, satisfying {{complete_seating_criterion}} and {{stretch_and_damage_limits}}. Avoid twisting, pinching, local unseating, and damage. Do not cut or glue the ring, or keep it pressed down with a gripper to create an apparent seated state.

Identify the ring's cross-sectional orientation, groove entrance, and currently seated and unseated sections. Choose the first locating point and assembly order. After establishing reliable local seating, guide adjacent unseated sections into the groove while preserving the seated portion's support.

Adjust local grasp locations, stretching direction, and pressing points according to deformation and groove-edge contact. Do not pull already seated sections out to install a distant section. If a section flips, overlaps, or becomes pinched, reduce tension, back out that local section, straighten it, and then resume installation.

Inspect the full circumference for unseated sections, changing the viewing direction when needed. Do not assume an unseen region is seated. Gradually release external support and check that the ring remains in the groove, with complete seating and preserved integrity.
```

### 擦桌子（从 L2-08 移入 L3，新编号待确认）

运行前填写：桌面、可用抹布／擦拭工具、目标污渍区域、清洁标准、工具放置区域。实现条件：明确污渍去除与接触的关联、工具／布面形变及清洁评分；单纯沿路径移动不算擦干净，也不报告尚未接入的恒力能力。

```text
Use {{cloth_or_wiping_tool_id}} to clean {{target_dirty_region}} on {{table_surface_id}} until {{public_cleanliness_criterion}} is satisfied. Finally, place the tool in {{tool_placement_region}}. Preserve the tabletop. Do not treat pushing dirt into an unpermitted region or out of camera view as successful cleaning.

Observe the dirt distribution, table boundaries, obstacles, and current tool shape. Choose a grasp and contact arrangement that keeps the effective wiping surface against the table. Plan coverage around the dirt distribution with sufficient overlap between covered regions. Do not stop solely because a predetermined path has been traversed.

Wipe in segments and compare new images for actual cleaning progress. If the tool lifts, the cloth bunches, dirt spreads, or the table moves, adjust the grasp, contact location, direction, and actions that affect contact pressure. Without force feedback, do not claim to maintain an exact constant force. Restore effective contact before continuing when the wiping surface loses contact.

Target remaining dirt with additional passes. If the tool needs arranging or the task permits using another face, do so with stable support while avoiding contamination of cleaned regions. Once the cleanliness criterion is met, lift the tool away, inspect the entire target region again, place the tool, and verify the final state.
```

## 提示版本与验证

语言版本：英文模型输入，中文阅读说明。共用约定、25 项任务正文与模板变量名均已统一为英文；翻译保留原任务目标、成功条件和动作协议。

本文只提供首版候选提示，不记录虚构的成功率。先用少量独立预实验检查目标是否清楚、接口是否够用、模型是否把“动作完成”误当成“任务完成”；对有依据的失败修改提示，然后固定正式测试版本。保留失败、超时与未完成停止，不能只统计模型自报成功的回合。

比较两个 prompt 时，保持任务物体、初始条件分布、可见信息、动作接口和预算一致；以外部评分器判定的完整成功率为主，同时检查损伤、洒落、稳定性等任务约束。本文没有示范轨迹，属于带操作提示的指令方案，不等于已经开展 ICL 示范学习。



任务设计按照框架

任务设计原则、任务目标、成功条件、操作协议和可观察线索进行描述。每个任务正文包含：（自动化结果）

仿真跑出预期的结果（L1-L3 得分decay）

真机仿真对齐（任务层面）

主动交互（探索）






真机任务排期
机器到位
双臂gpt6接入
任务道具排布和测试
任务迭代和反馈


项目定位任务人员分配
