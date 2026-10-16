import argparse,hashlib,json,pathlib,xml.etree.ElementTree as ET,yaml
parser=argparse.ArgumentParser(description="Export the checked-in Sirius hardware contract for the Web simulator")
parser.add_argument("--output", required=True)
args=parser.parse_args()
ws=pathlib.Path(__file__).resolve().parents[2]
paths={'roboteq':'src/roboteq_ros2_jazzy_driver/roboteq_ros2_driver/config/roboteq.yaml','imu':'params/wt905.yaml','sdf':'src/sirius/sirius_description/sdf/sirius3.sdf'}
raw={k:(ws/v).read_bytes() for k,v in paths.items()}
r=yaml.safe_load(raw['roboteq'])['roboteq_ros2_driver']['ros__parameters'];i=yaml.safe_load(raw['imu'])['witmotion']['ros__parameters']['imu_publisher'];sdf=ET.fromstring(raw['sdf']).find('model')
mount=lambda name:list(map(float,sdf.find(f"link[@name='{name}']/pose").text.split()))[:3]
p={'name':'Sirius VLP16 / WT905 / Roboteq','sources':{k:{'path':v,'sha256':hashlib.sha256(raw[k]).hexdigest()} for k,v in paths.items()},'wheel':{'circumference':r['wheel_circumference'],'track':r['track_width'],'countsPerRevolution':r['pulse']*r['gear_ratio'],'publishHz':r['odom_publish_hz'],'velocitySamples':5,'maxStepRevolutions':r['max_encoder_step_revolutions']},'mounts':{'lidar':mount('lidar_link'),'imu':mount('imu_link'),'foot':mount('lidar_link2')},'imu':{'publishHz':20,'rateSource':'Historical real bags 2026-09-19/20: approximately 20 Hz; polling_interval is NOT output rate','covariance':{key:value['covariance'] for key,value in i['measurements'].items()}},'lidar':{'scanHz':10,'rangeNoiseStd':.01,'noiseSource':'Simulation scenario value, not calibrated from hardware'}}
pathlib.Path(args.output).write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n')
