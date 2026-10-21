from pathlib import Path
import pycolmap as pc
import json, shutil
from PIL import Image
w=Path(__file__).resolve().parent; img=w/'sfm_images';img.mkdir(exist_ok=True)
for p in (w/'frames').glob('new_*.jpg'):shutil.copy2(p,img/p.name)
for p in Path('/Users/jingbowen/Downloads/拓境小型实验室/视频图片资源').glob('*.JPG'):
 im=Image.open(p); im.thumbnail((1320,1600));im.save(img/(p.stem+'.jpg'))
db=w/'sfm.db'
e=pc.FeatureExtractionOptions();e.num_threads=6;e.sift.max_num_features=6000
r=pc.ImageReaderOptions();r.camera_model='SIMPLE_RADIAL'
pc.extract_features(db,img,image_names=sorted(p.name for p in img.glob('new*')),camera_mode=pc.CameraMode.SINGLE,reader_options=r,extraction_options=e,device=pc.Device.cpu)
pc.extract_features(db,img,image_names=sorted(p.name for p in img.glob('IMG*')),camera_mode=pc.CameraMode.PER_IMAGE,reader_options=r,extraction_options=e,device=pc.Device.cpu)
m=pc.FeatureMatchingOptions();m.num_threads=6
pc.match_exhaustive(db,matching_options=m,device=pc.Device.cpu)
o=pc.IncrementalPipelineOptions();o.num_threads=6;o.min_model_size=5;o.mapper.init_min_tri_angle=8;o.max_runtime_seconds=240
out=w/'sfm';out.mkdir(exist_ok=True)
models=pc.incremental_mapping(db,img,out,options=o)
for k,rec in models.items():print('MODEL',k,rec.summary(),flush=True)
