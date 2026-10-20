import os, time, json, struct
from collections import defaultdict
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
os.environ.setdefault('ROS_DOMAIN_ID', '53')
import numpy as np, psutil, rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2,Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener,TransformException
rclpy.init();node=rclpy.create_node('sirius_read_only_timing_probe');buffer=Buffer();listener=TransformListener(buffer,node)
start=time.monotonic();events=defaultdict(list);authorities=defaultdict(set);values=defaultdict(list);statuses=[];lookups=defaultdict(list);resources=[]
def raw(topic,data):
 now=time.monotonic();order='<' if data[1]==1 else '>';sec,nsec=struct.unpack_from(order+'iI',data,4);stamp=sec+nsec*1e-9
 events[topic].append([now-start,stamp,time.time()-stamp])
def tf(m,info):
 now=time.monotonic()-start;wall=time.time();gid=bytes(info.get('publisher_gid', []) if isinstance(info,dict) else info.publisher_gid).hex()
 events['/tf (all messages)'].append([now,0.,0.])
 for t in m.transforms:
  name=t.header.frame_id+' -> '+t.child_frame_id;stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
  events[name].append([now,stamp,wall-stamp]);authorities[name].add(gid)
  q=t.transform.rotation;p=t.transform.translation;values[name].append([now,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
def status(m):
 events['/map_localizer/status'].append([time.monotonic()-start,0.,0.]);statuses.append([time.monotonic()-start,m.data])
diagnostics, tf_statuses = [], []
def diagnostic_cb(m):
 try: diagnostics.append(json.loads(m.data))
 except ValueError: pass
def tf_status_cb(m):
 try: tf_statuses.append(json.loads(m.data))
 except ValueError: pass
subs=[node.create_subscription(TFMessage,'/tf',tf,qos_profile_sensor_data),node.create_subscription(String,'/map_localizer/status',status,10)]
subs += [node.create_subscription(String, '/fastlio/diagnostics', diagnostic_cb, 10), node.create_subscription(String, '/map_localizer/tf_status', tf_status_cb, 10)]
for topic,typ in [('/velodyne_points',PointCloud2),('/cloud_registered_body',PointCloud2),('/imu',Imu),('/odom',Odometry),('/Odometry',Odometry),('/map_localizer/pose',PoseStamped)]:
 subs.append(node.create_subscription(typ,topic,lambda data,k=topic:raw(k,data),qos_profile_sensor_data,raw=True))
def check():
 now=node.get_clock().now()
 for offset in [None,.1,.2,.3]:
  key='latest' if offset is None else f'{offset:.1f}s_past'
  stamp=Time() if offset is None else Time(nanoseconds=now.nanoseconds-int(offset*1e9))
  try:
   tf=buffer.lookup_transform('map','sirius3/base_footprint',stamp)
   t=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9;lookups[key].append([time.monotonic()-start,True,time.time()-t,''])
  except TransformException as e:lookups[key].append([time.monotonic()-start,False,0.,type(e).__name__])
node.create_timer(.05,check)
def resource():
 row={'t':time.monotonic()-start,'cpu_percent':psutil.cpu_percent(),'memory_percent':psutil.virtual_memory().percent,'swap_used_bytes':psutil.swap_memory().used}
 resources.append(row)
node.create_timer(5.,resource)
end=start+45
while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
summary={}
for key,rows in events.items():
 a=np.array([row for row in rows if row[0]>=5])
 if not len(a): continue
 dt=np.diff(a[:,0]);stampdt=np.diff(a[:,1]);latency=a[:,2]
 def stats(x):return {'p50':float(np.percentile(x,50)),'p95':float(np.percentile(x,95)),'p99':float(np.percentile(x,99)),'max':float(np.max(x))} if len(x) else {}
 rates=[]
 for left in range(5,40,5):rates.append(float(np.count_nonzero((a[:,0]>=left)&(a[:,0]<left+5))/5))
 s={'count':len(a),'average_hz':float((len(a)-1)/(a[-1,0]-a[0,0])) if len(a)>1 else 0,'interval_ms':stats(dt*1e3),'five_second_hz':rates,'gaps_over_200ms':int(np.count_nonzero(dt>.2)),'gaps_over_500ms':int(np.count_nonzero(dt>.5))}
 if key not in ['/tf (all messages)','/map_localizer/status']:
  s.update(age_ms=stats(latency*1e3),header_interval_ms=stats(stampdt*1e3),backwards_stamps=int(np.count_nonzero(stampdt<0)))
 summary[key]=s
publisher_map={bytes(x.endpoint_gid).hex():x.node_namespace.rstrip('/')+'/'+x.node_name for x in node.get_publishers_info_by_topic('/tf')}
for key,gids in authorities.items():summary[key]['publishers']=[publisher_map.get(g,g) for g in gids]
for key,rows in values.items():
 a=np.array(rows);delta=np.linalg.norm(np.diff(a[:,1:4],axis=0),axis=1)
 q=a[:,4:8];angle=2*np.degrees(np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1],axis=1)),0,1)))
 summary[key]['value_changes']=int(np.count_nonzero((delta>1e-8)|(angle>1e-5)))
 summary[key]['max_translation_step_m']=float(delta.max()) if len(delta) else 0
 summary[key]['max_rotation_step_deg']=float(angle.max()) if len(angle) else 0
lookup_summary={}
for key,rows in lookups.items():
 # Ignore initial DDS/TF buffer warm-up.
 rows=[r for r in rows if r[0]>=5];ok=[r for r in rows if r[1]]
 lookup_summary[key]={'attempts':len(rows),'success':len(ok),'failures':len(rows)-len(ok),'max_age_ms':max([r[2]*1e3 for r in ok],default=None),'failure_types':sorted(set(r[3] for r in rows if not r[1]))}
result={'measured_at_jst':datetime.now(ZoneInfo('Asia/Tokyo')).isoformat(),'duration_s':time.monotonic()-start,'topics_and_frames':summary,'lookup_map_base':lookup_summary,'status_summary':{'total':len(statuses),'accepted':sum(s.startswith('ok ') for _,s in statuses),'rejected':sum(s.startswith('reject ') for _,s in statuses),'latest':statuses[-1] if statuses else None},'resources':resources,'events':dict(events),'statuses':statuses,'lookups':dict(lookups)}
result['pipeline_diagnostics'] = diagnostics
result['tf_statuses'] = tf_statuses
p=Path.home()/'.local/state/sirius_localization_v15';p.mkdir(parents=True,exist_ok=True);out=p/('timing_'+datetime.now(ZoneInfo('Asia/Tokyo')).strftime('%Y%m%d_%H%M%S')+'.json');out.write_text(json.dumps(result,indent=2));result.pop('events');result.pop('statuses');result.pop('lookups');result.pop('pipeline_diagnostics');result.pop('tf_statuses');print('SAVED',out,flush=True);print(json.dumps(result,indent=2),flush=True)
node.destroy_node();rclpy.shutdown()
