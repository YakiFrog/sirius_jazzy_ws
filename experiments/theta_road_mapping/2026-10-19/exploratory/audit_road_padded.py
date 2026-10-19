from pathlib import Path
import json,cv2,numpy as np,yaml,rclpy,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage,LaserScan
from tf2_msgs.msg import TFMessage
from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
from sirius_navigation.theta_bev_projection import build_blend_maps
root=Path('/home/kotantu-desktop/sirius_jazzy_ws');bag=Path('/home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered');out=root/'maps_waypoints/maps/theta_road_visibility_20261019_padded025'
cal=root/'src/sirius/sirius_navigation/config/theta_calibration_real.yaml'
rclpy.init(args=['--ros-args','-p',f'calibration:={cal}','-p','lidar_strict_visibility:=true','-p','lidar_margin_m:=0.10','-p','lidar_obstacle_padding_m:=0.25'])
n=ThetaGroundCloudNode();r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));r.set_filter(rosbag2_py.StorageFilter(topics=['/tf','/tf_static','/scan3','/theta/dual_fisheye/image_raw/compressed']))
index=0;pending=[];rows=[];stats=[];maps=None
size=n.bev_size
cols,rr=np.meshgrid(np.arange(size),np.arange(size));x=(.5-(rr+.5)/size)*n.extent;y=(.5-(cols+.5)/size)*n.extent
while r.has_next():
 topic,data,bag_ns=r.read_next()
 if topic in ['/tf','/tf_static']:
  for tf in deserialize_message(data,TFMessage).transforms:
   if topic=='/tf_static':n.visibility_tf.set_transform_static(tf,'bag')
   else:n.visibility_tf.set_transform(tf,'bag')
 elif topic=='/scan3':n._on_scan(deserialize_message(data,LaserScan))
 else:
  if index in [1500,3000,4500]:pending.append((index,deserialize_message(data,CompressedImage),bag_ns))
  index+=1
 if pending and bag_ns-pending[0][2]>500000000:
  idx,msg,_=pending.pop(0);raw=cv2.imdecode(np.frombuffer(msg.data,np.uint8),cv2.IMREAD_COLOR)
  if maps is None:maps=build_blend_maps(yaml.safe_load(cal.read_text()),raw.shape[1],raw.shape[0])
  a,b,c,d,alpha=maps
  bev=np.clip(cv2.remap(raw,a,b,cv2.INTER_LINEAR).astype(float)*alpha[...,None]+cv2.remap(raw,c,d,cv2.INTER_LINEAR).astype(float)*(1-alpha[...,None]),0,255).astype(np.uint8)
  valid=(x*x+y*y>=1.2**2)&(x*x+y*y<=4.8**2)&(bev.max(2)>=8)
  keep=n._strict_visibility(x,y,msg.header.stamp,'sirius3/base_footprint')
  n.lidar_padding=0.0
  oldkeep=n._strict_visibility(x,y,msg.header.stamp,'sirius3/base_footprint')
  n.lidar_padding=.25
  before=bev.copy();before[~valid]=35
  previous=before.copy();previous[valid&~oldkeep]=35
  after=before.copy();after[valid&~keep]=35
  extra=valid&oldkeep&~keep
  overlay=previous.copy();overlay[extra]=(overlay[extra]*.3+np.array([0,0,255])*.7).astype(np.uint8)
  tiles=[]
  for image,title in [(previous,'Previous: no lateral padding'),(overlay,'Additional exclusion (red)'),(after,'Padding: 25 cm')]:
   tile=cv2.resize(image,(500,500),interpolation=cv2.INTER_AREA);tile=cv2.copyMakeBorder(tile,35,0,0,0,cv2.BORDER_CONSTANT,value=(35,35,35));cv2.putText(tile,f'{idx}: {title}',(10,24),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1,cv2.LINE_AA);tiles.append(tile)
  row=np.hstack(tiles);rows.append(row);cv2.imwrite(str(out/f'observation_{idx}.jpg'),row)
  stats.append({'frame':idx,'candidate_pixels':int(valid.sum()),'excluded_pixels':int((valid&~keep).sum()),'previous_excluded_pixels':int((valid&~oldkeep).sum()),'extra_excluded_pixels':int(extra.sum())})
  print(stats[-1],flush=True)
  if len(stats)==3:break
cv2.imwrite(str(out/'observation_comparison.jpg'),np.vstack(rows));(out/'observation_stats.json').write_text(json.dumps(stats,indent=2))
n.destroy_node();rclpy.shutdown()
