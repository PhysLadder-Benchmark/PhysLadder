"""Opt-in L1-01 visual comparison to Feishu video, preserving the baseline.

Photo-estimated pose, lighting and appearance. No hardware calibration, physics
steps, controller calls, source asset edits or task success evidence.
"""
PROFILE = "feishu_l1_01_video_20261013_v1"


def align_spec(spec, task):
    from dataclasses import replace
    from ..l1.specs import pose
    import math
    if task.get("visual_alignment_profile") not in (PROFILE, "photo_l1_01_detail_20261013_v2") or spec.design.task_id != "L1-01":
        raise ValueError("Unsupported visual alignment profile/task")
    holder_xy = (.032, 1.205)
    marker = next(a for a in spec.design.articles if a.name == "marker")
    goal = (*holder_xy, *marker.goal[2:])
    marker = replace(marker, initial=pose(.300, 1.180, .781, yaw=math.pi/2), goal=goal)
    design = replace(spec.design, articles=tuple(marker if a.name == "marker" else a for a in spec.design.articles),
                     fixture_pose=pose(*holder_xy,.78,yaw=-math.pi/2),
                     version=spec.design.version+"-video-alignment-v1",
                     provenance="Photo estimate from Feishu 02_pen_150s.mp4, frames 0/5/20 s; not metric calibration")
    return replace(spec, design=design, parameters={**spec.parameters,"holder_xy":holder_xy,"visual_alignment_profile":task["visual_alignment_profile"]})

def apply(backend,task):
    from pxr import Gf,UsdLux,UsdShade,UsdGeom
    import carb
    stage=backend.world.stage
    report={'scope':'L1-01 only; visual estimates from Feishu 02_pen_150s.mp4 at 0s, 5s, 20s; not metric calibration'}

    lights={}
    for name,intensity in [('Fill',1800),('Area0',5500),('Area1',4200)]:
        api=UsdLux.LightAPI(stage.GetPrimAtPath('/World/Lights/'+name))
        api.GetIntensityAttr().Set(intensity)
        api.GetColorAttr().Set(Gf.Vec3f(.70,.87,1.0))
        lights[name]={'intensity':intensity,'color':[.70,.87,1.0]}
    settings=carb.settings.get_settings()
    settings.set_bool('/rtx/indirectDiffuse/enabled',True)
    settings.set_bool('/rtx/post/histogram/enabled',False)
    settings.set_string('/rtx/rendermode','PathTracing')
    settings.set_int('/rtx/pathtracing/spp',32)
    settings.set_int('/rtx/pathtracing/totalSpp',128)
    settings.set_bool('/rtx/pathtracing/optixDenoiser/enabled',True)
    from robot_tasks.backends.appearance import apply_r5_appearance
    report['robot_appearance']={m.name:apply_r5_appearance(stage,m.prim_path) for m in task.config.robot.mounts}
    from pxr import Sdf
    window=UsdLux.RectLight.Define(stage,'/World/Lights/L1VideoFrontFill')
    window.CreateIntensityAttr(1700)
    window.CreateNormalizeAttr(True)
    window.CreateWidthAttr(1.2)
    window.CreateHeightAttr(.8)
    window.CreateColorAttr(Gf.Vec3f(.72,.88,1.))
    xf=UsdGeom.Xformable(window)
    xf.AddTranslateOp().Set(Gf.Vec3d(.05,.3,1.7))
    xf.AddRotateXOp().Set(55.)
    texture=UsdShade.Shader(stage.GetPrimAtPath(task.scene.root+'/marker/Body/Looks/Label/Texture'))
    texture.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.0,1.5,3.0,1.0))
    for mount in task.config.robot.mounts:
        for prim in stage.Traverse():
            if not str(prim.GetPath()).startswith(mount.prim_path+'/') or '/Appearance/Material_' not in str(prim.GetPath()) or not prim.IsA(UsdShade.Shader):
                continue
            shader=UsdShade.Shader(prim)
            color=shader.GetInput('diffuseColor').Get()
            if color[0]>.9 and color[1]<.01 and color[2]<.01:
                shader.GetInput('diffuseColor').Set(Gf.Vec3f(.55,.85,1.))
                shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.8,1.2,1.5))
                shader.GetInput('metallic').Set(0.)
            elif max(color)<.04:
                shader.GetInput('roughness').Set(.30)
                shader.GetInput('metallic').Set(.15)
    report['indicator_appearance']='photo-estimated cool-white indicator; no hardware state inference'
    report['front_fill']={'intensity':1700,'color':[.72,.88,1.],'position':[.05,.3,1.7],'rotate_x_deg':55}
    report['robot_joint_geometry_changed']=False
    report['physics_properties_changed']=False
    report['lights']=lights
    report['render_settings']={key:settings.get(key) for key in ['/rtx/rendermode','/rtx/post/tonemap/op','/rtx/post/tonemap/exposure','/rtx/post/histogram/enabled','/rtx/indirectDiffuse/enabled']}
    report['shaders']=[str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith(task.scene.root+'/') and p.IsA(UsdShade.Shader)]
    if task.config.raw["task"].get("visual_alignment_profile") == "photo_l1_01_detail_20261013_v2":
        from .l1_01_detail import apply as apply_detail
        report["photo_detail_v2"] = apply_detail(stage, task.config.robot)
        report["indicator_appearance"] = "Both cyan-blue ON; user request; no hardware state inference"
        report["scope"] = "L1-01; v1 layout plus user close-photo geometry and neutral bright lighting"
        report["lights"] = {name:{**data,"color":[.88,.94,1.]} for name,data in lights.items()}
        report["front_fill"]["color"] = [.84,.92,1.]
    return report
