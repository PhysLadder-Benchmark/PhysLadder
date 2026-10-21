from pathlib import Path
import json,numpy as np,pycolmap as pc,cv2
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation
w=Path(__file__).resolve().parent;r=pc.Reconstruction(w/'sfm/0');metric=np.load(w/'metric.npz');T=metric['T'];s=float(metric['s']);R=metric['R'];t=T[:3,3];meta=json.loads((w/'metric_report.json').read_text());ims=sorted(r.images.values(),key=lambda im:im.name); imidx={im.name:i for i,im in enumerate(ims)};pids=list(r.points3D);pmap={p:i for i,p in enumerate(pids)};cids=list(r.cameras);cmap={c:i for i,c in enumerate(cids)}
pts=np.array([s*R@r.points3D[p].xyz+t for p in pids]);poses=[]
for im in ims:
 M=im.cam_from_world().matrix();rot=M[:,:3]@R.T;trans=s*M[:,3]-rot@t;poses.append(np.r_[Rotation.from_matrix(rot).as_rotvec(),trans])
intr=np.array([[r.cameras[c].params[0],r.cameras[c].params[3]] for c in cids]);centers=np.array([r.cameras[c].params[1:3] for c in cids]);camera_of_im=np.array([cmap[im.camera_id] for im in ims]);oi=[];op=[];uv=[]
for j,im in enumerate(ims):
 for pt in im.points2D:
  if pt.has_point3D():oi.append(j);op.append(pmap[pt.point3D_id]);uv.append(pt.xy)
oi=np.array(oi);op=np.array(op);uv=np.array(uv);oc=camera_of_im[oi]
def select(polys):
 out=set()
 for name,poly in polys.items():
  for pt in ims[imidx[name]].points2D:
   if pt.has_point3D() and cv2.pointPolygonTest(np.array(poly,np.float32),tuple(pt.xy),False)>=0:out.add(pmap[pt.point3D_id])
 return np.array(sorted(out),int)
table=select(meta['table_plane_polygons']);table=table[abs(pts[table,2]-.78)<.015]
floor=select({'IMG_3338.jpg':[[40,1100],[295,1130],[365,1510],[40,1520]],'IMG_3333.jpg':[[440,1390],[700,1390],[700,1560],[430,1560]]});floor=floor[abs(pts[floor,2]+.10)<.04]
wall=select({'IMG_3334.jpg':[[100,50],[700,50],[700,480],[100,480]]});wall=wall[abs(pts[wall,1]-.75)<.03]
edge_im=[];edge_uv=[];edge_axis=[];edge_val=[];edge_init=[]
# Initial other coordinate from calibrated ray intersection; these coordinates remain free along each edge.
inv=np.linalg.inv(T)
for name,groups in meta['edge_samples'].items():
 axis=0 if name in ['left','right'] else 1;val={'left':-.815,'right':.815,'front':-.39,'back':.39}[name]
 for image,uvs in groups:
  j=imidx[image];rr=Rotation.from_rotvec(poses[j][:3]).as_matrix();C=-rr.T@poses[j][3:]
  for px in uvs:
   v=rr.T@np.r_[r.cameras[ims[j].camera_id].cam_from_img(np.array(px,dtype=float)),1];X=C+v*(.78-C[2])/v[2]
   edge_im.append(j);edge_uv.append(px);edge_axis.append(axis);edge_val.append(val);edge_init.append(X[1-axis])
edge_im=np.array(edge_im);edge_uv=np.array(edge_uv);edge_axis=np.array(edge_axis);edge_val=np.array(edge_val);E=len(edge_im);N=len(ims);C=len(cids);P=len(pts);O=len(oi)
a=N*6;b=a+C*2;c=b+P*3;e=c+E
v0=np.r_[np.array(poses).ravel(),intr.ravel(),pts.ravel(),edge_init,.75]
def unpack(v):return v[:a].reshape(N,6),v[a:b].reshape(C,2),v[b:c].reshape(P,3),v[c:e],v[e]
def project(pos,intr,points,images):
 rs=Rotation.from_rotvec(pos[:,:3]).as_matrix();q=np.einsum('nij,nj->ni',rs[images],points)+pos[images,3:];norm=q[:,:2]/q[:,2,None];cams=camera_of_im[images];return norm*(1+intr[cams,1,None]*np.sum(norm**2,axis=1)[:,None])*intr[cams,0,None]+centers[cams]
def fun(v):
 pos,ins,xyz,along,wy=unpack(v);ep=np.zeros((E,3));ep[:,2]=.78;ep[np.arange(E),edge_axis]=edge_val;ep[np.arange(E),1-edge_axis]=along
 # All plane and edge constraints are marked modeling assumptions, not survey ground truth.
 return np.r_[(project(pos,ins,xyz[op],oi)-uv).ravel(),((xyz[table,2]-.78)/.003),xyz[floor,2]/.01,(xyz[wall,1]-wy)/.005,((project(pos,ins,ep,edge_im)-edge_uv)/1.5).ravel(),(ins[:,1]-intr[:,1])/.1]
J=lil_matrix((len(fun(v0)),len(v0)),dtype=int)
for k,(ii,pp,cc) in enumerate(zip(oi,op,oc)):
 J[2*k:2*k+2,ii*6:ii*6+6]=1;J[2*k:2*k+2,a+cc*2:a+cc*2+2]=1;J[2*k:2*k+2,b+pp*3:b+pp*3+3]=1
z=O*2
for group,axis in [(table,2),(floor,2),(wall,1)]:
 for pp in group:J[z,b+pp*3+axis]=1;J[z,e]=int(axis==1);z+=1
for k,ii in enumerate(edge_im):
 cc=camera_of_im[ii];J[z:z+2,ii*6:ii*6+6]=1;J[z:z+2,a+cc*2:a+cc*2+2]=1;J[z:z+2,c+k]=1;z+=2
for cc in range(C):J[z+cc,a+cc*2+1]=1
# Add metric ruler and native CAD constraints to resolve the height/scale ambiguity.
ns={"np":np,"Rotation":Rotation};source=(w/'fit_assets.py').read_text();exec(source[source.index('orig='):source.index('fit={}')],ns);robot_points=ns['robot_points']
landmarks={
 'left':{'wrist_top':[('IMG_3334.jpg',[81,574]),('new_21.jpg',[431,448]),('new_03.jpg',[318,332])],'elbow_top':[('IMG_3334.jpg',[179,532]),('new_21.jpg',[518,435]),('new_03.jpg',[218,338])]},
 'right':{'wrist_top':[('IMG_3334.jpg',[494,598]),('new_22.jpg',[309,374]),('new_28.jpg',[250,177])],'elbow_top':[('IMG_3334.jpg',[533,558]),('new_22.jpg',[418,363]),('new_28.jpg',[140,148])]} }
basefun=fun;oldn=len(v0);oldm=J.shape[0]
# Start from unconstrained reconstruction with hard table dimensions.
v0=np.r_[v0,[-.1483,.34,-1.57,0,0,.5],[.3483,.34,-1.57,0,0,.5],[.16,-.64]]
extra_deps=[]
def fun(v):
 out=list(basefun(v[:oldn]));pos,ins,xyz,along,wy=unpack(v[:oldn])
 for side,off in [('left',oldn),('right',oldn+6)]:
  rv=v[off:off+6];pp=robot_points(rv)
  for mark,obs in landmarks[side].items():
   for image,pixel in obs:
    ii=imidx[image];out.extend(((project(pos,ins,np.array([pp[mark]]),np.array([ii]))[0]-pixel)/3).tolist())
  target=-.1483 if side=='left' else .3483
  out.extend([(rv[0]-target)/.06,rv[3]/.2,rv[4]/.25,rv[5]/.6])
 # 51cm measurement endpoints are ruler anchors, with two cm uncertainty per endpoint in x.
 for side,off,pixel in [('left',oldn,[128,730]),('right',oldn+6,[507,776])]:
  ii=imidx['IMG_3334.jpg'];X=np.array([[v[off]+(-.03 if side=='left' else 0),v[off+1]+(.015 if side=='left' else -.015),.79]])
  out.extend(((project(pos,ins,X,np.array([ii]))[0]-pixel)/8).tolist())
 # Height line was drawn onto the stand. Both endpoints correspond to the same vertical line.
 for z,pixel in [(0,[375,1337]),(1.32,[377,178])]:
  ii=imidx['IMG_3333.jpg'];X=np.array([[*v[-2:],z]])
  out.extend(((project(pos,ins,X,np.array([ii]))[0]-pixel)/4).tolist())
 return np.array(out)
J2=lil_matrix((len(fun(v0)),len(v0)),dtype=int);J2[:oldm,:oldn]=J;row=oldm
for side,off in [('left',oldn),('right',oldn+6)]:
 for mark,obs in landmarks[side].items():
  for image,pixel in obs:
   ii=imidx[image];cc=camera_of_im[ii];J2[row:row+2,ii*6:ii*6+6]=1;J2[row:row+2,a+cc*2:a+cc*2+2]=1;J2[row:row+2,off:off+6]=1;row+=2
 J2[row:row+4,off:off+6]=1;row+=4
for off in [oldn,oldn+6]:
 ii=imidx['IMG_3334.jpg'];cc=camera_of_im[ii];J2[row:row+2,ii*6:ii*6+6]=1;J2[row:row+2,a+cc*2:a+cc*2+2]=1;J2[row:row+2,off:off+6]=1;row+=2
for z in [0,1.32]:
 ii=imidx['IMG_3333.jpg'];cc=camera_of_im[ii];J2[row:row+2,ii*6:ii*6+6]=1;J2[row:row+2,a+cc*2:a+cc*2+2]=1;J2[row:row+2,-2:]=1;row+=2
print('START',len(v0),'parameters',len(fun(v0)),'residuals',flush=True)
lower=np.full(len(v0),-np.inf);upper=np.full(len(v0),np.inf)
for off,lo,hi in [(oldn,-.23,.03),(oldn+6,.27,.45)]:
 lower[off:off+6]=[lo,.22,-1.9,-.3,-.4,0];upper[off:off+6]=[hi,.46,-1.2,.35,.4,1.0]
res=least_squares(fun,v0,bounds=(lower,upper),jac_sparsity=J2.tocsr(),x_scale='jac',loss='soft_l1',f_scale=3,max_nfev=400,ftol=1e-5,verbose=2)
pos,ins,xyz,along,wy=unpack(res.x[:oldn]);projerr=np.linalg.norm(project(pos,ins,xyz[op],oi)-uv,axis=1)
print('DONE',res.message,'px',np.quantile(projerr,[.5,.9,.99]),'floor',np.quantile(xyz[floor,2],[.1,.5,.9]),'table',np.quantile(xyz[table,2],[.1,.5,.9]),'wall',wy,'robot',res.x[oldn:],flush=True)
np.savez(w/'ba.npz',poses=pos,intrinsics=ins,points=xyz,colors=np.array([r.points3D[p].color for p in pids]),centers=centers,camera_of_im=camera_of_im,names=np.array([im.name for im in ims]),wall_y=wy,robot_fit=res.x[oldn:oldn+12].reshape(2,6),stand_fit=res.x[-2:])
(w/'ba_report.json').write_text(json.dumps({'status':res.message,'nfev':res.nfev,'registered_views':N,'points3D':P,'observations':O,'mean_reprojection_error_px':float(projerr.mean()),'median_reprojection_error_px':float(np.median(projerr)),'p90_reprojection_error_px':float(np.quantile(projerr,.9)),'plane_constraints_counts':[len(table),len(floor),len(wall)],'floor_z_median_m':float(np.median(xyz[floor,2])),'table_z_median_m':float(np.median(xyz[table,2])),'desk_wall_y_table_frame_m':float(wy),'robot_fit':res.x[oldn:oldn+12].reshape(2,6).tolist(),'stand_xy':res.x[-2:].tolist(),'intrinsics':ins.tolist(),'edge_reprojection_rmse_px':float(np.sqrt(np.mean(basefun(res.x[:oldn])[2*O+len(table)+len(floor)+len(wall):-C]**2))*1.5),'note':'Metric constrained bundle adjustment: table 1.63x0.78m at z0.78; floor z0; perpendicular wall; native R5 CAD landmarks; 68/51/48 cm reconciled with 4cm closure error; 1.32m stand height. Constraint residuals are not independent accuracy.'},indent=2))
