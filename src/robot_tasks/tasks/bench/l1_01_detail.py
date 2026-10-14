"""Photo-derived, opt-in L1-01 visual details. No simulation or control calls.

Geometry additions are visual only. Dimensions of the camera envelope come from
the delivered Gemini305 CAD; small features and finishes are photo estimates.
"""
import math
import numpy as np

PROFILE = "photo_l1_01_detail_20261013_v2"


def static_wrist_pose(stage, profile):
    """Author q6=0 from URDF, retaining link1-5 and the robot bases.

    The delivered static USD has opposed +/-90 degree wrist rolls. The configured
    home q6 is zero. This authors a static pose, without a reset or articulation
    drive. Finger opening uses the existing 70 mm nominal simulation envelope.
    """
    from pxr import Gf, UsdGeom
    from robot_tasks.robots.urdf import read_urdf
    from robot_tasks.robots.transforms import origin_transform
    desc = read_urdf(profile.asset_path, dict(profile.package_roots))
    report = {}
    for mount in profile.mounts:
        root = mount.prim_path
        def world(name):
            return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(root+"/"+name))).T
        old = world("link6")
        joint = desc.joint("joint6")
        target = world("link5") @ origin_transform(joint.origin_xyz, joint.origin_rpy)
        targets = {"link6": target}
        for name in ("joint7", "joint8"):
            j = desc.joint(name)
            opening = np.eye(4)
            opening[:3, 3] = np.asarray(j.axis) * .035
            targets[j.child] = target @ origin_transform(j.origin_xyz, j.origin_rpy) @ opening
        for name, value in targets.items():
            p = stage.GetPrimAtPath(root+"/"+name)
            parent = np.array(UsdGeom.XformCache().GetParentToWorldTransform(p)).T
            xf = UsdGeom.Xformable(p)
            xf.MakeMatrixXform().Set(Gf.Matrix4d((np.linalg.inv(parent) @ value).T.tolist()))
        report[mount.name] = {"old_link6_world": old.tolist(), "new_link6_world": target.tolist(),
                              "q6_rad": 0., "finger_q_m": .035,
                              "source": "URDF joint6 origin; static authoring, no motor commands"}
    return report


class Details:
    def __init__(self, stage, root):
        from robot_tasks.backends.appearance import material
        self.stage, self.root = stage, root
        self.materials = {}
        for key, color, rough, metal, emission in [
            ("anodized", (.022,.025,.029), .31, .55, None),
            ("screw", (.21,.23,.25), .24, .85, None),
            ("recess", (.003,.004,.006), .48, .1, None),
            ("silver", (.55,.59,.63), .27, .75, None),
            ("white_print", (.82,.84,.82), .66, 0., None),
            ("glass", (.009,.012,.016), .1, .25, None),
            ("lens", (.022,.031,.045), .08, .45, None),
            ("red", (.43,.014,.012), .46, 0., None),
            ("red_led", (.5,.002,.001), .25, 0., (2.,.005,.001)),
        ]:
            self.materials[key] = material(stage,root+"/PhotoMaterials/"+key,color,roughness=rough,metallic=metal,emission=emission)
        self.count = 0

    def bind(self, prim, key):
        from pxr import UsdShade
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.materials[key])
        self.count += 1

    def mesh(self, path, points, faces, key):
        from pxr import UsdGeom
        m = UsdGeom.Mesh.Define(self.stage, self.root+"/"+path)
        points = np.asarray(points, dtype=np.float32)
        m.CreatePointsAttr(points)
        m.CreateExtentAttr(np.array([points.min(0),points.max(0)],dtype=np.float32))
        m.CreateFaceVertexCountsAttr([len(f) for f in faces])
        m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateDoubleSidedAttr(True)
        self.bind(m.GetPrim(),key)
        return m

    def cylinder(self, path, center, radius, height, axis, key):
        from pxr import Gf, UsdGeom
        c = UsdGeom.Cylinder.Define(self.stage,self.root+"/"+path)
        c.CreateRadiusAttr(radius); c.CreateHeightAttr(height); c.CreateAxisAttr(axis)
        UsdGeom.Xformable(c).AddTranslateOp().Set(Gf.Vec3d(*center))
        self.bind(c.GetPrim(),key)
        return c

    def box(self, path, center, dims, key):
        from pxr import Gf, UsdGeom
        c=UsdGeom.Cube.Define(self.stage,self.root+"/"+path)
        c.CreateSizeAttr(1.)
        xf=UsdGeom.Xformable(c)
        xf.AddTranslateOp().Set(Gf.Vec3d(*center)); xf.AddScaleOp().Set(Gf.Vec3f(*dims))
        self.bind(c.GetPrim(),key)
        return c

    def roundcase(self,path,center,dims,radius,key):
        # Rounded rectangle in YZ, extruded along X with an edge bevel.
        depth,width,height=dims
        yz=[]
        for cy,cz,start in [(width/2-radius,height/2-radius,0),(-width/2+radius,height/2-radius,90),
                            (-width/2+radius,-height/2+radius,180),(width/2-radius,-height/2+radius,270)]:
            for angle in np.linspace(start,start+90,9):
                t=math.radians(angle); yz.append((cy+radius*math.cos(t),cz+radius*math.sin(t)))
        points=[];bevel=min(.0007,depth/4)
        for x,scale in [(-depth/2,.97),(-depth/2+bevel,1),(depth/2-bevel,1),(depth/2,.97)]:
            points.extend(np.array(center)+np.array([x,y*scale,z*scale]) for y,z in yz)
        n=len(yz);faces=[list(range(n-1,-1,-1)),list(range(3*n,4*n))]
        for k in range(3):
            faces.extend([k*n+i,k*n+(i+1)%n,(k+1)*n+(i+1)%n,(k+1)*n+i] for i in range(n))
        return self.mesh(path,points,faces,key)

    def ring(self,path,center,outer,inner,depth,axis,key,segments=48):
        coords={'X':(1,2,0),'Y':(2,0,1),'Z':(0,1,2)}[axis]
        points=[]
        for a,r in [(-depth/2,outer),(depth/2,outer),(-depth/2,inner),(depth/2,inner)]:
            for theta in np.linspace(0,2*math.pi,segments,endpoint=False):
                v=np.array(center,dtype=float); v[coords[0]]+=r*math.cos(theta);v[coords[1]]+=r*math.sin(theta);v[coords[2]]+=a;points.append(v)
        faces=[]
        for k in range(segments):
            j=(k+1)%segments
            for a,b in [(0,1),(2,3),(0,2),(1,3)]:
                faces.append([a*segments+k,a*segments+j,b*segments+j,b*segments+k])
        return self.mesh(path,points,faces,key)

    def bolt(self,path,center,axis="Y",radius=.0025):
        # Metal socket head with a real hexagonal recess, not a painted dot.
        coords={'X':(1,2,0),'Y':(2,0,1),'Z':(0,1,2)}[axis]
        n=48;points=[]
        for depth,inner in [(-.00075,False),(.00075,False),(-.00075,True),(.00075,True)]:
            for theta in np.linspace(0,2*math.pi,n,endpoint=False):
                r=radius if not inner else radius*.49*math.cos(math.pi/6)/math.cos(theta%(math.pi/3)-math.pi/6)
                v=np.array(center,dtype=float);v[coords[0]]+=r*math.cos(theta);v[coords[1]]+=r*math.sin(theta);v[coords[2]]+=depth;points.append(v)
        faces=[]
        for i in range(n):
            j=(i+1)%n
            for a,b in [(0,1),(2,3),(0,2),(1,3)]:
                faces.append([a*n+i,a*n+j,b*n+j,b*n+i])
        self.mesh(path+"Head",points,faces,"screw")
        self.cylinder(path+"Socket",center,radius*.47,.00015,axis,"recess")


def apply(stage, profile):
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade
    report={"reference":"User close photos 1-8 and IMG_3479.MOV, 2026-10-13",
            "calibration":"Photo estimates; no metric hardware or photometric calibration",
            "static_pose":static_wrist_pose(stage,profile),"details":{}}
    for mount in profile.mounts:
        root=mount.prim_path; d=Details(stage,root)
        sensor="link6/WristSensor/Sensor"
        for name in ("Housing","Face","Lens0","Lens1"):
            UsdGeom.Imageable(stage.GetPrimAtPath(root+"/"+sensor+"/"+name)).MakeInvisible()
        # Existing white mounting bracket is the supplied STEP tessellation.
        d.bind(stage.GetPrimAtPath(root+"/link6/WristSensor/Mount/Geometry"),"white_print")
        prefix=sensor+"/PhotoDetail/"
        d.roundcase(prefix+"SilverHousing",(0,0,0),(.025278,.042035,.042034),.0045,"silver")
        d.roundcase(prefix+"FrontSeal",(.0126,0,-.0008),(.0009,.0395,.0375),.004,"recess")
        d.roundcase(prefix+"GlassFace",(.0132,0,-.0008),(.00065,.0382,.0362),.0035,"glass")
        for i,y in enumerate((-.0098,.0098)):
            d.ring(prefix+f"Lens{i}Bezel",(.01375,y,-.0029),.00635,.0059,.00045,"X","silver")
            d.ring(prefix+f"Lens{i}Barrel",(.0137,y,-.0029),.0058,.0034,.0004,"X","recess")
            d.cylinder(prefix+f"Lens{i}Glass",(.0140,y,-.0029),.0035,.00022,"X","lens")
            d.ring(prefix+f"Lens{i}InnerRing",(.01412,y,-.0029),.0027,.0025,.00012,"X","anodized")
        d.roundcase(prefix+"LowerWindow",(.01365,0,-.0160),(.00025,.010,.004),.001,"recess")
        # User explicitly excludes the protruding side plug AND red status LED.
        for y in (-.015,.015):
            d.bolt(prefix+("RearLeft" if y<0 else "RearRight"),(-.0129,y,.014),"X",.0016)
        # Three cap bolts: positions recovered from the source mesh hole loops.
        for i,(x,y) in enumerate([(.0635,.011741),(.08375,.00005),(.0635,-.011641)]):
            d.bolt(f"link4/PhotoDetail/Cap{i}",(x,y,-.08735),"Z",.00265)
        # Lower arm side plates: match existing 6.1 mm countersunk holes.
        for side,y in enumerate([-.03005,.03955]):
            for i,(x,z) in enumerate([(.0125,.021651),(-.0125,.021651),(-.025,0),(-.0125,-.021651),(.0125,-.021651),(.025,0),
                                      (-.2765,.021651),(-.2515,.021651),(-.239,0),(-.2515,-.021651),(-.2765,-.021651),(-.289,0)]):
                d.bolt(f"link2/PhotoDetail/Side{side}Bolt{i}",(x,y,z),"Y",.0027)
        for side,y in enumerate([-.044,.042]):
            for i,(x,z) in enumerate([(.015,-.0345),(.015,-.0855),(0,.016),(-.016,0),(.016,0)]):
                d.bolt(f"link3/PhotoDetail/Motor{side}Bolt{i}",(x,y,z),"Y",.0025)
        for side,y in enumerate([-.0315,.0300]):
            for i,(x,z) in enumerate([(.257503,-.038349),(.270003,-.06),(.257503,-.081651),(.232503,-.081651),(.220003,-.06),(.232503,-.038349)]):
                d.bolt(f"link3/PhotoDetail/End{side}Bolt{i}",(x,y,z),"Y",.0026)
        for side,y in enumerate([-.026,.024]):
            for i,(x,z) in enumerate([(.08304,-.075),(.14304,-.075),(.08304,-.045),(.14304,-.045)]):
                d.bolt(f"link3/PhotoDetail/Cover{side}Bolt{i}",(x,y,z),"Y",.00235)
        # Red bearing end seals travel with each finger; stainless rail and rack
        # already exist in the delivered vendor mesh and are retained.
        for link,sign in [("link7",1),("link8",-1)]:
            path=link+"/PhotoDetail/"
            d.box(path+"RedEndSeal",(-.0105,-sign*.0115,.0002),(.016,.0026,.013),"red")
            d.bolt(path+"BearingFastener",(-.0019,-sign*.0115,.0002),"X",.0017)
            for i,(y,z) in enumerate([(-sign*.007,.00774),(-sign*.007,-.00726),(sign*.003,-.00726)]):
                d.bolt(path+f"FingerFastener{i}",(.0005,y,z),"X",.0025)
        # The logo in the vendor color export is white. Photos show dark relief.
        logo=UsdShade.Shader(stage.GetPrimAtPath(root+'/link2/Appearance/Material_1/Surface'))
        logo.GetInput('diffuseColor').Set(Gf.Vec3f(.007,.009,.012))
        logo.GetInput('roughness').Set(.48)
        # Both indicators ON, per the user's explicit correction.
        for p in stage.Traverse():
            if not str(p.GetPath()).startswith(root+"/") or "/Appearance/Material_" not in str(p.GetPath()) or not p.IsA(UsdShade.Shader):
                continue
            sh=UsdShade.Shader(p);color=sh.GetInput("diffuseColor").Get()
            if sh.GetInput("emissiveColor"):
                active=True
                sh.GetInput("diffuseColor").Set(Gf.Vec3f(*((.015,.33,.64) if active else (.73,.77,.84))))
                sh.GetInput("emissiveColor").Set(Gf.Vec3f(*((.015,.55,1.4) if active else (0.,0.,0.))))
                sh.GetInput("roughness").Set(.42)
            elif max(color)<.04:
                sh.GetInput("diffuseColor").Set(Gf.Vec3f(.022,.025,.029))
                sh.GetInput("roughness").Set(.32);sh.GetInput("metallic").Set(.4)
        # The milky diffuser has a blue edge and cyan centre, rather than a flat
        # emissive rectangle. Vertex colors follow the existing mesh footprint.
        led=UsdGeom.Mesh(stage.GetPrimAtPath(root+'/link3/Appearance/Part_1'))
        points=np.array(led.GetPointsAttr().Get())
        center=np.exp(-((points[:,2]+.060)/.007)**2)
        end=np.clip(np.minimum(points[:,0]-.0505,.1755-points[:,0])/.018,0,1)
        bright=.20+.8*center*end
        colors=np.stack([np.full(len(points),.012),.05+.50*bright,np.full(len(points),1.25)],axis=1)
        UsdGeom.PrimvarsAPI(led).CreatePrimvar('photoLedColor',Sdf.ValueTypeNames.Color3fArray,'vertex').Set(colors.astype(np.float32))
        reader=UsdShade.Shader.Define(stage,root+'/link3/Appearance/Material_1/PhotoLedColor')
        reader.CreateIdAttr('UsdPrimvarReader_float3');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('photoLedColor')
        reader.CreateOutput('result',Sdf.ValueTypeNames.Float3)
        sh=UsdShade.Shader(stage.GetPrimAtPath(root+'/link3/Appearance/Material_1/Surface'))
        sh.GetInput('emissiveColor').ConnectToSource(reader.ConnectableAPI(),'result')
        report["details"][mount.name]={"visual_parts":d.count,"indicator":"cyan-blue ON","side_connector":False,"red_status_LED":False}
        # Refine the existing mounting blocks, keeping the mount footprint fixed.
        base='/World/Furniture/'+root.rsplit('/',1)[1]+'_Mount'
        bd=Details(stage,base)
        for i,x in enumerate([-.077,.077]):
            bd.bind(stage.GetPrimAtPath(base+f'/Controller{i}'),'anodized')
            UsdGeom.Imageable(stage.GetPrimAtPath(base+f'/Connector{i}')).MakeInvisible()
            bd.box(f'PhotoDetail/ClampTop{i}',(x,0,.018),(.029,.045,.003),'anodized')
            bd.ring(f'PhotoDetail/Washer{i}',(x,0,.0205),.0065,.0032,.001,'Z','silver')
            bd.bolt(f'PhotoDetail/BaseBolt{i}',(x,0,.022),'Z',.0048)
            for j,dy in enumerate([-.020,.020]):
                bd.box(f'PhotoDetail/ClampRim{i}_{j}',(x,dy,.021),(.036,.003,.006),'anodized')
    # A neutral room with a slightly cooler window; keep the bright v1 levels.
    for name in ("Fill","Area0","Area1"):
        UsdLux.LightAPI(stage.GetPrimAtPath('/World/Lights/'+name)).GetColorAttr().Set(Gf.Vec3f(.88,.94,1.))
    UsdLux.LightAPI(stage.GetPrimAtPath('/World/Lights/L1VideoFrontFill')).GetColorAttr().Set(Gf.Vec3f(.84,.92,1.))
    report.update(cables_added=False,side_connectors_added=False,both_indicators_on=True,physics_parameters_changed=False,collision_geometry_changed=False,
                  camera_intrinsics_changed=False,original_assets_edited=False)
    return report
