"""Recorded-TF/scan-gated native and direct texture observations, without replay."""
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from theta_texture_quality import aggregate_frame,camera_rotation,sample_floor

DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<f4'),('weight','<f4'),('semantic_id','u1')])


class Capture:
    msg=None
    def publish(self,msg):self.msg=msg


def observations(source,reference,indices,native=True,direct=True):
    """Yield packets at exact recorded camera times; do not publish ROS data."""
    import rclpy,rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage,LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    from sirius_navigation.theta_bev_node import ThetaBevNode
    from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
    from sirius_navigation.theta_indexed_map_node import quaternion_to_matrix
    from sirius_navigation.theta_road_weights import directional_weights
    source=Path(source);reference=Path(reference)
    manifest=json.loads((source/'manifest.json').read_text());meta=json.loads((reference/'manifest.json').read_text())
    frame_meta={f['frame']:f for f in meta['frames']};grid=meta['grid'];origin=np.asarray(grid['origin']);res=grid['resolution'];width=grid['width'];height=grid['height']
    rclpy.init(args=['--ros-args','-p',f'calibration:={(source/"calibration_used.yaml").resolve()}',
        '-p','publish_raw:=false','-p','stride:=2','-p','lidar_strict_visibility:=true',
        '-p','lidar_margin_m:=0.10','-p','lidar_obstacle_padding_m:=0.25','-p','rear_observation_weight:=1.0'])
    buffer=Buffer(cache_time=Duration(seconds=4096));reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=manifest['bag'],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf','/tf_static','/scan3']))
    scans=[]
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic=='/scan3':
            scan=deserialize_message(data,LaserScan);scans.append((scan.header.stamp.sec*10**9+scan.header.stamp.nanosec,scan))
        else:
            for tf in deserialize_message(data,TFMessage).transforms:
                if topic=='/tf_static':buffer.set_transform_static(tf,'bag')
                else:buffer.set_transform(tf,'bag')
    scans.sort(key=lambda x:x[0]);times=np.asarray([s[0] for s in scans],np.int64)
    bev=ThetaBevNode();bev.buffer=buffer;bev.pub=Capture();ground=ThetaGroundCloudNode();ground.visibility_tf=buffer
    ground.lidar_gate=True;ground.pub=Capture();ground.semantic_pub=None
    print('Loaded recorded TF and',len(scans),'scans',flush=True)
    try:
        for number,index in enumerate(sorted(indices)):
            info=manifest['frames'][str(index)];ns=info['stamp_ns'];data=(source/'inputs'/f'{index}.jpg').read_bytes()
            assert hashlib.sha256(data).hexdigest()==info['sha256']
            msg=CompressedImage();msg.header.stamp.sec=ns//10**9;msg.header.stamp.nanosec=ns%10**9;msg.data=data
            k=int(np.searchsorted(times,ns));near=min((j for j in (k-1,k) if 0<=j<len(scans)),key=lambda j:abs(int(times[j])-ns))
            if abs(int(times[near])-ns)>150000000:raise RuntimeError(f'Unmatched scan at frame {index}')
            ground.strict_scans={int(times[near]):scans[near][1]}
            tf=buffer.lookup_transform('map','sirius3/base_footprint',msg.header.stamp);q,t=tf.transform.rotation,tf.transform.translation
            rotation=quaternion_to_matrix(q.x,q.y,q.z,q.w);translation=np.array([t.x,t.y,t.z])
            np.testing.assert_allclose(translation,frame_meta[index]['xyz'],atol=1e-10,rtol=0)
            np.testing.assert_allclose([q.x,q.y,q.z,q.w],frame_meta[index]['quaternion'],atol=1e-10,rtol=0)
            raw=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR) if direct else None
            packet={};details=dict(index=index,stamp_ns=ns,scan_stamp_ns=int(times[near]),pose=translation.tolist(),quaternion=[q.x,q.y,q.z,q.w],input_sha256=info['sha256'])
            if native:
                bev.pub.msg=None;bev.receive(msg);bmsg=bev.pub.msg
                if bmsg is None:raise RuntimeError(f'No BEV at {index}')
                image=np.frombuffer(bmsg.data,np.uint8).reshape(bmsg.height,bmsg.width,3)
                ground.pub.msg=None;ground._emit(bmsg.header.stamp,bmsg.header.frame_id,image,None,publish_main=True,publish_semantic=False)
                cloud=ground.pub.msg
                if cloud is None:raise RuntimeError(f'No native floor points at {index}')
                points=np.frombuffer(cloud.data,DTYPE)
                assert len(points)==frame_meta[index]['ground_points'],(index,len(points),frame_meta[index]['ground_points'])
                details['native_point_count_checked']=len(points)
                local=np.column_stack((points['x'],points['y'],points['z']));world=local@rotation.T+translation
                gx=np.floor((world[:,0]-origin[0])/res).astype(int);gy=np.floor((world[:,1]-origin[1])/res).astype(int)
                inside=(gx>=0)&(gx<width)&(gy>=0)&(gy<height)
                packed=points['rgb'].copy().view(np.uint32);bgr=np.column_stack((packed&255,(packed>>8)&255,(packed>>16)&255))
                rays=(local-np.asarray(bev.calibration['camera_position']))@camera_rotation(bev.calibration)
                rays/=np.linalg.norm(rays,axis=1,keepdims=True);lens=.15+.85*rays[:,0]**2
                rear=directional_weights(local[:,0],local[:,1],.5)
                packet['native']=aggregate_frame((gy*width+gx)[inside],bgr[inside],points['weight'][inside],lens[inside],rear[inside])
            else:bev.apply_pose(bev.resolve_pose(msg.header.stamp))
            if direct:
                # Inverse sample fixed world-cell centers, without BEV/scatter rounding.
                lo=np.maximum(np.floor((translation[:2]-4.85-origin)/res).astype(int),0)
                hi=np.minimum(np.ceil((translation[:2]+4.85-origin)/res).astype(int),[width,height])
                gx,gy=np.meshgrid(np.arange(lo[0],hi[0]),np.arange(lo[1],hi[1]));dx=origin[0]+(gx+.5)*res-t.x;dy=origin[1]+(gy+.5)*res-t.y
                inv=np.linalg.inv(rotation[:2,:2]);x=inv[0,0]*dx+inv[0,1]*dy;y=inv[1,0]*dx+inv[1,1]*dy
                rgb,lens=sample_floor(raw,bev.calibration,x,y)
                radius2=x*x+y*y;valid=(radius2>=ground.min_radius**2)&(radius2<=ground.max_radius**2)&(rgb.max(axis=2)>=ground.min_value)
                visible=np.zeros(x.shape,bool);visible[valid]=ground._strict_visibility(x[valid],y[valid],msg.header.stamp,'sirius3/base_footprint');valid&=visible
                quality=(ground.camera_height/np.sqrt(radius2+ground.camera_height**2))**ground.quality_power
                rear=directional_weights(x,y,.5)
                packet['direct']=dict(index=(gy[valid]*width+gx[valid]).astype(np.int32),bgr=rgb[valid],q=quality[valid].astype(np.float32),
                    weight=quality[valid].astype(np.float32),lens=lens[valid],rear=rear[valid])
                details['direct_cells']=int(valid.sum())
            yield packet,details
            if (number+1)%50==0:print('Projected',number+1,'/',len(indices),flush=True)
    finally:
        bev.destroy_node();ground.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('reference',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--centers',default='900,1500,2700,3000,4500,5100');p.add_argument('--half-window',type=int,default=48);p.add_argument('--step',type=int,default=2);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True);centers=[int(c) for c in a.centers.split(',')]
    wanted=sorted({i for c in centers for i in range(c-a.half_window,c+a.half_window+1,a.step)})
    missing=[i for i in wanted if not (a.output/f'frame{i}.npz').exists()];rows=[]
    if (a.output/'manifest.json').exists():
        prior=json.loads((a.output/'manifest.json').read_text())
        expected=dict(source=str(a.source.resolve()),reference=str(a.reference.resolve()),centers=centers,half_window=a.half_window,step=a.step)
        assert all(prior[k]==v for k,v in expected.items()),'Observation cache input/settings mismatch'
        rows=prior['frames']
    cfg=dict(source=str(a.source.resolve()),reference=str(a.reference.resolve()),centers=centers,half_window=a.half_window,step=a.step,frames=rows)
    for packet,detail in observations(a.source,a.reference,missing):
        np.savez_compressed(a.output/f'frame{detail["index"]}.npz',**{k+'_'+key:value for k,payload in packet.items() for key,value in payload.items()})
        rows.append(detail);cfg['frames']=rows;(a.output/'manifest.json').write_text(json.dumps(cfg,indent=2))
    print('CACHE COMPLETE',len(rows),'matched observations',flush=True)


if __name__=='__main__':main()
