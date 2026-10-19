from pathlib import Path
import os,sys,subprocess,time,signal,json,requests,yaml
import rclpy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from builtin_interfaces.msg import Time
root=Path('/home/kotantu-desktop/sirius_jazzy_ws')
bag=Path('/home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered')
out=root/'maps_waypoints/maps/theta_road_visibility_20261019'
out.mkdir(parents=True,exist_ok=True)
state=requests.get('http://localhost:8080/debug_state',timeout=5).json()
thresholds=requests.get('http://localhost:8080/class_thresholds',timeout=5).json()
(out/'server_before.json').write_text(json.dumps(state,indent=2))
processes=[]
def start(args,label):
 f=(out/(label+'.log')).open('w');p=subprocess.Popen(args,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append((p,f));return p
try:
 launch=start(['ros2','launch','sirius_navigation','theta_road_visibility_trial.launch.py'],'nodes')
 time.sleep(6)
 if launch.poll() is not None:raise RuntimeError('trial launch failed')
 print('RUNNING',out,flush=True)
 replay=start(['ros2','bag','play',str(bag),'--clock','--rate','4','--disable-keyboard-controls','--topics','/tf','/tf_static','/scan3','/theta/dual_fisheye/image_raw/compressed'],'replay')
 rc=replay.wait()
 if rc:raise RuntimeError(f'bag replay exit {rc}')
 print('Replay completed; flushing pending road observations',flush=True)
 rclpy.init();node=rclpy.create_node('road_trial_save');clock=node.create_publisher(Clock,'/clock',10)
 pubs={k:node.create_publisher(String,f'/trial/{k}/save',10) for k in ['baseline','visible']}
 meta=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
 end=meta['starting_time']['nanoseconds_since_epoch']+meta['duration']['nanoseconds']
 for i in range(160):
  ns=end+i*200000000;clock.publish(Clock(clock=Time(sec=ns//1000000000,nanosec=ns%1000000000)))
  rclpy.spin_once(node,timeout_sec=.05)
 for name,pub in pubs.items():
  d=out/name;d.mkdir(exist_ok=True)
  prefix=d/('road_'+name)
  pub.publish(String(data=str(prefix)+'.colored'))
 for i in range(60):rclpy.spin_once(node,timeout_sec=.1)
 for name in pubs:
  p=out/name/('road_'+name)
  if not p.with_suffix('.texture.png').exists():raise RuntimeError(f'No map saved: {name}')
 print('SAVED',out,flush=True)
 node.destroy_node();rclpy.shutdown()
finally:
 for p,f in reversed(processes):
  if p.poll() is None:
   os.killpg(p.pid,signal.SIGINT)
   try:p.wait(timeout=12)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=6)
  f.close()
 for route,data in [('prompt',{'prompt':state['sam3_settings']['prompt']}),('threshold',{'threshold':state['sam3_settings']['confidence_threshold']}),('class_thresholds',thresholds),('source_mode',{'mode':state['mode']})]:
  try:requests.post('http://localhost:8080/'+route,json=data,timeout=5).raise_for_status()
  except Exception as e:print('Restore error',route,e,flush=True)
