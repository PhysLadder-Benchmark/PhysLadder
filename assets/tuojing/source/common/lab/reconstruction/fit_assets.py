from pathlib import Path
import numpy as np,cv2,json
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
w=Path(__file__).resolve().parent;ba=np.load(w/'ba.npz');names=list(ba['names']);poses=ba['poses'];ins=ba['intrinsics'];centers=ba['centers'];ci=ba['camera_of_im'];rots=Rotation.from_rotvec(poses[:,:3]).as_matrix()
def ray(name,uv):
 i=names.index(name);f,k=ins[ci[i]];c=centers[ci[i]];p=(np.array(uv)-c)/f;u=p.copy()
 for _ in range(10):u=p/(1+k*(u@u))
 v=rots[i].T@np.r_[u,1];return -rots[i].T@poses[i,3:],v/np.linalg.norm(v)
def project(name,xyz):
 i=names.index(name);q=np.atleast_2d(xyz)@rots[i].T+poses[i,3:];u=q[:,:2]/q[:,2,None];f,k=ins[ci[i]];return u*(1+k*np.sum(u*u,axis=1)[:,None])*f+centers[ci[i]]
def atplane(name,uv,z=.78):
 C,v=ray(name,uv);return C+v*(z-C[2])/v[2]
def triang(obs):
 A=[];b=[]
 for name,uv in obs:
  C,v=ray(name,uv);P=np.eye(3)-np.outer(v,v);A.append(P);b.append(P@C)
 X=np.linalg.lstsq(np.concatenate(A),np.concatenate(b),rcond=None)[0];err=[float(np.linalg.norm(project(name,X)[0]-uv)) for name,uv in obs];return X,err
landmarks={
 'left_wrist_top':[('IMG_3334.jpg',[81,574]),('new_21.jpg',[431,448]),('new_03.jpg',[318,332])],
 'right_wrist_top':[('IMG_3334.jpg',[494,598]),('new_22.jpg',[309,374]),('new_28.jpg',[250,177])],
 'left_elbow_top':[('IMG_3334.jpg',[179,532]),('new_21.jpg',[518,435]),('new_03.jpg',[218,338])],
 'right_elbow_top':[('IMG_3334.jpg',[533,558]),('new_22.jpg',[418,363]),('new_28.jpg',[140,148])],
 'stand_hub':[('IMG_3333.jpg',[375,1250]),('IMG_3338.jpg',[443,1198])],
 'stand_collar':[('IMG_3333.jpg',[375,728]),('IMG_3338.jpg',[512,667])],
 'camera_rear_center':[('IMG_3333.jpg',[313,151]),('new_18.jpg',[608,365]),('new_24.jpg',[354,141])],
 'camera_tip':[('IMG_3333.jpg',[275,6]),('new_18.jpg',[570,216])]
}
out={}
for name,obs in landmarks.items():
 X,err=triang(obs);out[name]={'xyz':X.tolist(),'reprojection_px':err,'observations':obs};print(name,X.round(4),np.round(err,2))
print('base annotation',atplane('IMG_3334.jpg',[128,730],.79),atplane('IMG_3334.jpg',[507,776],.79))
print('shaft height.78',atplane('IMG_3338.jpg',[518,730]),'table mark',atplane('IMG_3338.jpg',[315,648]))
# URDF FK: parameterize only observable folded-arm joints. Unobservable yaw absorbed into base orientation.
orig=[([0,0,.0565],[0,0,0],[0,0,1]),([.02,0,.047],[0,0,0],[0,1,0]),([-.264,0,0],[-np.pi,0,0],[0,1,0]),([.245,-.00005,-.06],[0,0,0],[0,1,0])]
def tf(xyz,rpy):
 m=np.eye(4);m[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();m[:3,3]=xyz;return m
def fk(q):
 M=np.eye(4);ret=[]
 for (xyz,rpy,ax),val in zip(orig,q):
  j=np.eye(4);j[:3,:3]=Rotation.from_rotvec(np.array(ax)*val).as_matrix();M=M@tf(xyz,rpy)@j;ret.append(M.copy())
 return ret
zero=fk([0,0,0,0]);p_elbow=(np.linalg.inv(zero[2])@np.r_[[-.227,0,.1915],1])[:3];p_wrist=(np.linalg.inv(zero[3])@np.r_[[.071,0,.25],1])[:3]
def robot_points(v):
 x,y,yaw,q2,q3,q4=v;F=fk([0,q2,q3,q4]);M=tf([x,y,.79],[0,0,yaw]);return {'elbow_top':(M@F[2]@np.r_[p_elbow,1])[:3],'wrist_top':(M@F[3]@np.r_[p_wrist,1])[:3]}
fit={}
for idx,side in enumerate(['left','right']):
 v=ba['robot_fit'][idx]; pp=robot_points(v);err=[]
 for mark in ['elbow_top','wrist_top']:
  for im,uv in landmarks[side+'_'+mark]:err.append(np.linalg.norm(project(im,pp[mark])[0]-uv))
 fit[side]={'params':v.tolist(),'predicted_landmarks':{k:x.tolist() for k,x in pp.items()},'landmark_rmse_pixels':float(np.sqrt(np.mean(np.array(err)**2)))}
 print('ROBOT',side,fit[side])
# Camera shaft position: line projection constraints from two photos and one video view.
lines={'IMG_3333.jpg':[[374,240],[375,1220]],'IMG_3338.jpg':[[574,250],[447,1190]],'new_24.jpg':[[220,292],[222,1130]]}
def shaft_res(v):
 xx,yy=v;out=[]
 for im,uvs in lines.items():
  line=np.cross(np.r_[uvs[0],1.],np.r_[uvs[1],1.]);line/=np.linalg.norm(line[:2]);px=project(im,[[xx,yy,.3],[xx,yy,1.20]]);out.extend((np.c_[px,np.ones(2)]@line)/3)
 return out
sh=least_squares(shaft_res,[.16,-.64],loss='soft_l1');print('SHAFT',sh.x,shaft_res(sh.x))
# Mount inclination from visible long holder side, with camera position estimated near white device.
# Retain measured 1.32m mark independently of SfM landmark z.
report={'landmarks':out,'robot_fit':fit,'stand_xy_table_frame':sh.x.tolist(),'shaft_projection_residual_pixels':(np.array(shaft_res(sh.x))*3).tolist(),'camera_model_confirmed':False,'desk_wall_y':float(ba['wall_y'])}
(w/'asset_fit.json').write_text(json.dumps(report,indent=2))
# Diagnostic overlays with fitted table grid, robot datum points and shaft.
for im in ['IMG_3334.jpg','IMG_3338.jpg','IMG_3339.jpg','IMG_3340.jpg','new_21.jpg','new_28.jpg']:
 pic=cv2.imread(str(w/'sfm_images'/im))
 def line(a,b,col):
  px=project(im,[a,b]);cv2.line(pic,tuple(px[0].astype(int)),tuple(px[1].astype(int)),col,2)
 for xx in np.linspace(-.815,.815,9):line([xx,-.39,.78],[xx,.39,.78],(40,220,40))
 for yy in np.linspace(-.39,.39,5):line([-.815,yy,.78],[.815,yy,.78],(40,220,40))
 for side,rr in fit.items():
  for name,X in rr['predicted_landmarks'].items():
   px=project(im,X)[0].astype(int);cv2.circle(pic,tuple(px),6,(30,30,255),2)
 line([*sh.x,0],[*sh.x,1.32],(255,190,30))
 cv2.imwrite(str(w/(im[:-4]+'_fit.jpg')),pic)
