from pathlib import Path
import json,numpy as np,cv2,shutil,hashlib
from scipy.spatial.transform import Rotation
w=Path(__file__).resolve().parent;o=w.parent;ba=np.load(w/'ba.npz');rep=json.loads((w/'ba_report.json').read_text());names=list(ba['names']);poses=ba['poses'];rot=Rotation.from_rotvec(poses[:,:3]).as_matrix();intr=ba['intrinsics'];ci=ba['camera_of_im'];centers=ba['centers']
def ray(name,uv):
 i=names.index(name);f,k=intr[ci[i]];p=(np.array(uv)-centers[ci[i]])/f;q=p.copy()
 for _ in range(10):q=p/(1+k*q@q)
 return -rot[i].T@poses[i,3:],rot[i].T@np.r_[q,1]
def project(name,points):
 i=names.index(name);q=points@rot[i].T+poses[i,3:];uv=q[:,:2]/q[:,2,None];f,k=intr[ci[i]];return uv*(1+k*np.sum(uv**2,axis=1)[:,None])*f+centers[ci[i]]
def wallpoint(name,uv):
 C,v=ray(name,uv);return C+v*(float(ba['wall_y'])-C[1])/v[1]
for name,uv in [('IMG_3339.jpg',[40,480]),('new_21.jpg',[68,300]),('IMG_3339.jpg',[393,400]),('IMG_3340.jpg',[317,400])]:print('WALLANCHOR',name,uv,wallpoint(name,uv))
# Left partition anchor is less supported than table coordinates. Retain a 4cm uncertainty.
left_corner=np.mean([wallpoint('new_01.jpg',[85,460])[0],wallpoint('new_21.jpg',[68,300])[0]])
center=np.array([-1.42-left_corner,1.937-float(ba['wall_y']),0.]);print('CENTER',center)
# Derive cork panel boundaries from the two visible vertical trim lines.
cl=wallpoint('IMG_3339.jpg',[393,400])[0]+center[0];cr=wallpoint('IMG_3340.jpg',[285,400])[0]+center[0]
# Photos place the shaft ~25cm ahead of the tabletop. Use measured offset, video-estimated x.
stand=np.array([.17,-.64,0.])+center
third_eye=stand+np.array([0,.035,1.35]);third_target=center+np.array([.10,.10,.83])
robots=[]
for name,rv in zip(['R5_Left','R5_Right'],ba['robot_fit']):
 x,y,yaw,q2,q3,q4=rv
 robots.append({'name':name,'base_xyz_m':(center+np.array([x,y,.79])).tolist(),'base_xy_table_frame_m':[float(x),float(y)],'yaw_deg':float(np.degrees(yaw)),'home_q_rad_m':[0,float(q2),float(q3),float(q4),0,0,.01,.01],'scale':[1,1,1]})
P={'world_axes':'+X windows, +Y desk wall, +Z up; metre; origin at room floor center','room':{'dimensions_m':[2.84,3.93,2.6],'source':'Polycam plan + registered PLY/OBJ/USD shell'},'desk':{'center_xy_m':center[:2].tolist(),'size_m':[1.63,.78,.78],'top_thickness_m':.025,'corner_radius_m':.045,'source':'IMG3335/3336/3337 iPhone ruler; latest video orientation','shell_placement_uncertainty_m':.07},'robots':robots,'camera':{'name':'ThirdPerson','stand_xyz_m':stand.tolist(),'measurement_height_m':1.32,'eye_xyz_m':third_eye.tolist(),'target_xyz_m':third_target.tolist(),'resolution':[1280,720],'horizontal_fov_deg':70,'intrinsics_status':'provisional; source camera model, K, distortion and original camera feed unavailable','housing_dimensions_m':[.043,.025,.043],'eye_position_uncertainty_m':.04,'orientation_uncertainty_deg':10,'horizontal_fov_sweep_deg':[60,70,85],'evidence':['IMG3333 height','IMG3338 25cm offset','IMG3341.MOV multi-view geometry']},'wall_cork':{'x_min':float(cl),'x_max':float(cr)},'measurement_conflict':{'table_length_m':1.63,'ruler_left_gap_m':.68,'ruler_between_points_m':.51,'ruler_right_gap_m':.48,'closure_error_m':.04,'interpretation':'Ruler endpoints are visible surface points; robot rotation-axis centers are inferred with native CAD. Do not equate 51cm annotation to confirmed axis spacing.'},'ba_metrics':rep,'not_calibrated':['camera intrinsics/distortion','exact mounting-hole centers','camera housing geometry/mass','servo gains/friction/contact compliance','rolling shutter/exposure/noise'],'physical_defaults':{'static_friction':.7,'dynamic_friction':.55,'restitution':.05,'status':'unmeasured simulation defaults'}}
(o/'layout_parameters.json').write_text(json.dumps(P,indent=2));(o/'evidence').mkdir(exist_ok=True)
# Save all optimized handheld extrinsics, distinct from the stationary third-person camera.
cams=[]
for i,name in enumerate(names):
 im=cv2.imread(str(w/'sfm_images'/name));h,ww=im.shape[:2];f,k=intr[ci[i]];c=centers[ci[i]];T=np.eye(4);T[:3,:3]=rot[i].T;T[:3,3]=-rot[i].T@poses[i,3:]+center
 cams.append({'name':name,'resolution':[ww,h],'K':[[float(f),0,float(c[0])],[0,float(f),float(c[1])],[0,0,1]],'k1':float(k),'T_world_from_optical':T.tolist(),'optical_axes':'X right, Y down, Z forward','status':'estimated from SfM and metric constrained BA'})
(o/'evidence/handheld_cameras.json').write_text(json.dumps(cams,indent=2))
# Store transformed reconstruction point cloud as an optional reference in metres.
xyz=ba['points']+center;rgb=ba['colors'];lines=['ply','format ascii 1.0',f'element vertex {len(xyz)}','property float x','property float y','property float z','property uchar red','property uchar green','property uchar blue','end_header']
lines.extend(' '.join(map(str,[*q,*co])) for q,co in zip(xyz,rgb));(o/'reference/latest_video_sparse.ply').write_text('\n'.join(lines))
# Rectify observed tabletop texture patches; missing coverage is filled from a photographed clean patch.
W,H=1630,780;gx,gy=np.meshgrid(np.linspace(-.815,.815,W),np.linspace(.39,-.39,H));points=np.c_[gx.ravel(),gy.ravel(),np.full(W*H,.78)]
polys={'IMG_3334.jpg':[[5,850],[730,850],[730,1380],[5,1210]],'IMG_3335.jpg':[[25,490],[310,490],[405,720],[5,754]],'IMG_3340.jpg':[[95,1005],[380,1005],[405,1330],[5,1360]],'new_21.jpg':[[200,630],[580,690],[695,990],[240,828]],'new_28.jpg':[[70,515],[410,492],[450,620],[150,730]]}
canvas=np.zeros((H,W,3),np.float32);weight=np.zeros((H,W),np.float32)
for name,poly in polys.items():
 im=cv2.imread(str(w/'sfm_images'/name));mask=np.zeros(im.shape[:2],np.uint8);cv2.fillPoly(mask,[np.array(poly,np.int32)],255);uv=project(name,points).astype(np.float32).reshape(H,W,2);sample=cv2.remap(im,uv[:,:,0],uv[:,:,1],cv2.INTER_LINEAR);m=cv2.remap(mask,uv[:,:,0],uv[:,:,1],cv2.INTER_NEAREST)/255.;canvas+=sample*m[:,:,None];weight+=m
covered=weight>0;canvas[covered]/=weight[covered,None]
# A real, clean central wood patch repeats only where the source was occluded or outside frame.
patch=canvas[310:570,640:1030].copy();good=(patch.sum(2)>0)
if good.mean()<.8:patch=canvas[400:600,850:1150].copy()
fill=np.tile(patch,(H//patch.shape[0]+1,W//patch.shape[1]+1,1))[:H,:W];canvas[~covered]=fill[~covered];canvas=np.clip(canvas,0,255).astype(np.uint8)
cv2.imwrite(str(o/'textures/table_photo.jpg'),canvas);cv2.imwrite(str(o/'evidence/table_texture_coverage.png'),covered.astype(np.uint8)*255)
# Cork pattern from real wall photo; no generative texture or RGB hallucination.
im=cv2.imread(str(w/'sfm_images/IMG_3334.jpg'));cv2.imwrite(str(o/'textures/cork_photo.jpg'),im[70:465,115:670]);print('texture_coverage',covered.mean())
# All 16 user resources retained in evidence index with file hashes and explicit use.
src=Path('/Users/jingbowen/Downloads/拓境小型实验室');audit=[]
for p in sorted(src.rglob('*')):
 if p.is_file() and not p.name.startswith('.'):
  usage='multi-view SfM / constraints / visual audit' if p.suffix.lower() in ['.jpg','.mov'] else 'registered fixed shell / scale and structural plan / cross-format geometry check'
  if p.name=='IMG_3276.MOV':usage='earlier layout visual cross-check; latest video takes priority'
  audit.append({'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'usage':usage})
(o/'evidence/source_index.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2))
for n in ['metric_report.json','ba_report.json','asset_fit.json']:
 if (w/n).exists():shutil.copy2(w/n,o/'evidence'/n)
# Analysis scripts are already included next to this file.
for n in ['IMG_3334.jpg','IMG_3338.jpg','new_21.jpg','new_28.jpg']:shutil.copy2(w/'sfm_images'/n,o/'evidence'/n)
