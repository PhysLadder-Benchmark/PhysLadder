from pathlib import Path
import pycolmap as pc, numpy as np, cv2, json
from scipy.optimize import least_squares
w=Path(__file__).resolve().parent;r=pc.Reconstruction(w/'sfm/0');ims={im.name:im for im in r.images.values()}
def ray(name,uv):
 im=ims[name];cam=r.cameras[im.camera_id];M=im.cam_from_world().matrix();C=-M[:,:3].T@M[:,3];v=M[:,:3].T@np.r_[cam.cam_from_img(np.array(uv,dtype=float)),1];return C,v/np.linalg.norm(v)
def intersect(name,uv,n,d):
 C,v=ray(name,uv);return C-v*(C@n+d)/(v@n)
polys={'IMG_3334.jpg':[[5,865],[730,865],[730,1410],[5,1190]],'IMG_3335.jpg':[[20,505],[325,505],[410,705],[15,745]],'IMG_3340.jpg':[[100,1010],[380,1010],[405,1320],[15,1350]],'new_21.jpg':[[140,635],[600,710],[680,978],[225,824]]}
ids=set()
for name,poly in polys.items():
 for p in ims[name].points2D:
  if p.has_point3D() and cv2.pointPolygonTest(np.array(poly,np.float32),tuple(p.xy),False)>=0:ids.add(p.point3D_id)
pts=np.array([r.points3D[k].xyz for k in ids]);rng=np.random.default_rng(4);best=[]
for _ in range(2000):
 a,b,c=pts[rng.choice(len(pts),3,replace=False)];n=np.cross(b-a,c-a);ln=np.linalg.norm(n)
 if ln<1e-12:continue
 n/=ln;d=-n@a;ind=np.where(abs(pts@n+d)<.012)[0]
 if len(ind)>len(best):best=ind
pp=pts[best];c=pp.mean(0);n=np.linalg.svd(pp-c)[2][-1];
if (ims['IMG_3335.jpg'].projection_center()-c)@n<0:n=-n
d=-n@c
# Table edge samples stay clear of rounded corners. Orthogonal rectangle is fit to all edges.
edges={
 'right':[('IMG_3335.jpg',[[446,415],[450,650],[451,732]]),('IMG_3340.jpg',[[410,970],[424,1240]])],
 'front':[('IMG_3335.jpg',[[90,781],[250,780],[373,781]]),('IMG_3340.jpg',[[35,1399],[280,1389]]),('new_21.jpg',[[260,860],[610,1009]])],
 'back':[('IMG_3339.jpg',[[85,774],[425,768]]),('IMG_3340.jpg',[[168,765],[361,747]]),('new_21.jpg',[[279,533],[375,554]])],
 'left':[('IMG_3339.jpg',[[64,807],[45,870]]),('new_26.jpg',[[145,700],[171,830]])]
}
# Establish +Y along table side towards wall, +X right as seen from in front.
y=intersect('IMG_3335.jpg',[442,340],n,d)-intersect('IMG_3335.jpg',[453,775],n,d);y/=np.linalg.norm(y);x=np.cross(y,n);x/=np.linalg.norm(x);y=np.cross(n,x)
B=np.array([x,y,n]);edgepoints={k:np.array([B@(intersect(im,uv,n,d)-c) for im,uvs in v for uv in uvs]) for k,v in edges.items()}
# 5 unknowns: yaw, scale, center x/y. Length/depth exact constraints; robust edge fit.
def f(v):
 yaw,s,tx,ty=v;R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
 out=[]
 for k,p in edgepoints.items():
  q=s*p[:,:2]@R.T+np.array([tx,ty]);ax=0 if k in ['left','right'] else 1;target={'left':-.815,'right':.815,'front':-.39,'back':.39}[k];out.extend((q[:,ax]-target)/.015)
 return out
s0=.78/np.linalg.norm(intersect('IMG_3335.jpg',[442,340],n,d)-intersect('IMG_3335.jpg',[453,775],n,d))
res=least_squares(f,[0,s0,0,0],loss='soft_l1',max_nfev=1000)
a,s,tx,ty=res.x;Rz=np.array([[np.cos(a),-np.sin(a),0],[np.sin(a),np.cos(a),0],[0,0,1]])
R=Rz@B;t=np.array([tx,ty,.78])-s*R@c
T=np.eye(4);T[:3,:3]=s*R;T[:3,3]=t
np.savez(w/'metric.npz',T=T,n=n,d=d,R=R,s=s)
cloud=np.array([p.xyz for p in r.points3D.values()]);np.savez(w/'metric_cloud.npz',xyz=s*cloud@R.T+t,rgb=np.array([p.color for p in r.points3D.values()]))
print('PLANE',len(pp),'of',len(pts),'residual_m',np.median(abs(pp@n+d))*s,'SCALE',s,'EDGE_RMSE',np.sqrt(np.mean(np.array(f(res.x))**2))*.015)
for k,p in edgepoints.items():print(k,(s*p@Rz.T+np.array([tx,ty,.78])).round(3).tolist())
def metric_point(name,uv,z=.78):
 nr=R[2];dr=(t[2]-z)/s;return s*R@intersect(name,uv,nr,dr)+t
def triang(obs):
 I=np.eye(3);AA=[];bb=[]
 for name,uv in obs:
  C,v=ray(name,uv);P=I-np.outer(v,v);AA.append(P);bb.append(P@C)
 X=np.linalg.lstsq(np.concatenate(AA),np.concatenate(bb),rcond=None)[0];errors=[]
 for name,uv in obs:
  im=ims[name];cam=r.cameras[im.camera_id];M=im.cam_from_world().matrix();cp=M[:,:3]@X+M[:,3];px=cam.img_from_cam(cp);errors.append(float(np.linalg.norm(px-uv)))
 return s*R@X+t,errors
# Stable identified landmarks on devices, mount and measured points.
landmarks={
 'stand_floor':[('IMG_3333.jpg',[375,1337]),('IMG_3338.jpg',[436,1292])],
 'stand_top_measure':[('IMG_3333.jpg',[377,178])],
 'camera_lens':[('IMG_3333.jpg',[313,151]),('new_18.jpg',[445,424])],
 'left_base_top':[('IMG_3334.jpg',[134,727]),('new_21.jpg',[452,572]),('new_03.jpg',[303,476])],
 'right_base_top':[('IMG_3334.jpg',[507,776]),('new_22.jpg',[338,533]),('new_28.jpg',[218,341])],
}
for name,obs in landmarks.items():
 if len(obs)>1:
  X,err=triang(obs);print('LANDMARK',name,X.round(4),np.round(err,2))
print('PHOTO 51cm endpoints', metric_point('IMG_3334.jpg',[128,730],.79),metric_point('IMG_3334.jpg',[507,776],.79))
print('25cm table endpoint',metric_point('IMG_3338.jpg',[315,648]));print('25cm stand endpoint',metric_point('IMG_3338.jpg',[527,736]));
meta={'registered_views':len(ims),'points3D':len(r.points3D),'mean_reprojection_error_pixels':r.compute_mean_reprojection_error(),'sfm_to_table_world':T.tolist(),'sfm_scale_m_per_unit':s,'table_plane_points':len(pp),'table_plane_median_residual_m':float(np.median(abs(pp@n+d))*s),'edge_fit_rmse_m':float(np.sqrt(np.mean(np.array(f(res.x))**2))*.015),'edge_samples':edges,'table_plane_polygons':polys}
(w/'metric_report.json').write_text(json.dumps(meta,indent=2))
# Draw metric grid into reference photos to inspect alignment.
for name in ['IMG_3334.jpg','IMG_3338.jpg','IMG_3339.jpg','IMG_3340.jpg','new_21.jpg','new_28.jpg']:
 im=ims[name];cam=r.cameras[im.camera_id];M=im.cam_from_world().matrix();pic=cv2.imread(str(w/'sfm_images'/name));inv=np.linalg.inv(T)
 def project(X):
  X=(inv@np.r_[X,1])[:3];X=M[:,:3]@X+M[:,3];return tuple(np.round(cam.img_from_cam(X)).astype(int))
 for xx in np.linspace(-.815,.815,9):cv2.line(pic,project([xx,-.39,.78]),project([xx,.39,.78]),(0,220,0),1)
 for yy in np.linspace(-.39,.39,5):cv2.line(pic,project([-.815,yy,.78]),project([.815,yy,.78]),(0,220,0),1)
 cv2.imwrite(str(w/(name.replace('.jpg','_grid.jpg'))),pic)
