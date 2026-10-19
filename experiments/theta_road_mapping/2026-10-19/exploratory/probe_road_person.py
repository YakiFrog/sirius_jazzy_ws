from pathlib import Path
import time,json,cv2,numpy as np,requests,yaml
from sirius_navigation.theta_sam3_preview import sample_frames
from sirius_navigation.theta_perspective import build_perspective_remap
bag=Path('/home/kotantu-desktop/rosbag2_data/0920/real_both_mapping_20260920_113447_recovered')
frames=sample_frames(bag,'mcap','/theta/dual_fisheye/image_raw/compressed',[3000]);bgr=frames[0][0]
cal=yaml.safe_load(Path('/home/kotantu-desktop/sirius_jazzy_ws/src/sirius/sirius_navigation/config/theta_calibration_real.yaml').read_text());mx,my=build_perspective_remap(cal,0,bgr.shape[:2],(640,480),120)
view=cv2.remap(bgr,mx,my,cv2.INTER_LINEAR);cv2.imwrite('/tmp/road_person_probe_view.jpg',view)
u='http://localhost:8080';before=requests.get(u+'/debug_state').json()
try:
 for route,data in [('source_mode',{'mode':'network'}),('prompt',{'prompt':'person'}),('threshold',{'threshold':.5})]:requests.post(u+'/'+route,json=data,timeout=5).raise_for_status()
 _,jpg=cv2.imencode('.jpg',view);requests.post(u+'/upload_frame',data=jpg.tobytes(),timeout=5).raise_for_status()
 target=requests.get(u+'/debug_state').json()['network_frame_version'];start=time.monotonic()
 while True:
  state=requests.get(u+'/debug_state').json()
  if state['sam3_inference_frame_version']==target:break
  if time.monotonic()-start>40:raise RuntimeError('SAM3 inference timeout')
  time.sleep(.1)
 score=cv2.imdecode(np.frombuffer(requests.get(u+'/semantic_score.png').content,np.uint8),cv2.IMREAD_GRAYSCALE);mask=score>=128
 overlay=view.copy();overlay[mask]=(overlay[mask]*.4+np.array([0,0,255])*.6).astype(np.uint8);cv2.imwrite('/tmp/road_person_probe_overlay.jpg',overlay)
 print(json.dumps(dict(inference_s=time.monotonic()-start,scores=state['class_scores'],masked_pixels=int(mask.sum()),shape=list(score.shape)),ensure_ascii=False))
finally:
 for route,data in [('prompt',{'prompt':before['sam3_settings']['prompt']}),('threshold',{'threshold':before['sam3_settings']['confidence_threshold']}),('source_mode',{'mode':before['mode']})]:requests.post(u+'/'+route,json=data,timeout=5)
