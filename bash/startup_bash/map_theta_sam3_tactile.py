"""Map cached ten-interval semantic observations using recorded TF and scan gates.

The existing full-route RGB texture is preserved. Semantic coverage is limited
by the170 evaluated frames, not represented as a full-route semantic inference.
"""
import argparse,hashlib,json,shutil
from pathlib import Path
import cv2,numpy as np,yaml,rclpy,rosbag2_py
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage,LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from sirius_navigation.theta_bev_node import ThetaBevNode
from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
from sirius_navigation.theta_indexed_map_node import quaternion_to_matrix
from sirius_navigation.theta_perspective import build_bev_to_view_maps
from compare_theta_sam3_projection import project_labels

DTYPE=np.dtype([('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<f4'),('weight','<f4'),('semantic_id','u1')])


class Capture:
    msg=None
    def publish(self,msg):self.msg=msg


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);p.add_argument('texture',type=Path);p.add_argument('output',type=Path);p.add_argument('--method',choices=['causal','bidirectional'],default='bidirectional');a=p.parse_args()
    root=a.study;out=a.output;out.mkdir(parents=True,exist_ok=True);manifest=json.loads((root/'manifest.json').read_text());calfile=root/'calibration_used.yaml';cal=yaml.safe_load(calfile.read_text())
    metadata=json.loads(a.texture.with_suffix('.json').read_text()) if a.texture.with_suffix('.json').exists() else json.loads(Path(str(a.texture).replace('.texture.png','.colored.json')).read_text())
    texture=cv2.imread(str(a.texture),-1);missing_texture=texture[...,3]==0
    h,w=metadata['height'],metadata['width'];fill_sum=np.zeros((h,w,3),np.float32);fill_weight=np.zeros((h,w),np.float32);res=metadata['resolution'];origin=np.asarray(metadata['origin']);wanted={int(i) for i in manifest['frames']}
    rclpy.init(args=['--ros-args','-p',f'calibration:={calfile.resolve()}','-p','publish_raw:=false','-p','stride:=2','-p','lidar_strict_visibility:=true','-p','lidar_margin_m:=0.10','-p','lidar_obstacle_padding_m:=0.25'])
    buffer=Buffer(cache_time=Duration(seconds=4096));reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=manifest['bag'],storage_id='mcap'),rosbag2_py.ConverterOptions('',''));reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf','/tf_static','/scan3',manifest['camera_topic']]))
    scans=[];images={};index=0
    while reader.has_next():
        topic,data,_=reader.read_next()
        if topic in ('/tf','/tf_static'):
            for tf in deserialize_message(data,TFMessage).transforms:
                if topic=='/tf_static':buffer.set_transform_static(tf,'bag')
                else:buffer.set_transform(tf,'bag')
        elif topic=='/scan3':
            scan=deserialize_message(data,LaserScan);scans.append((scan.header.stamp.sec*10**9+scan.header.stamp.nanosec,scan))
        else:
            if index in wanted:
                msg=deserialize_message(data,CompressedImage);assert hashlib.sha256(bytes(msg.data)).hexdigest()==manifest['frames'][str(index)]['sha256'];images[index]=msg
            index+=1
    scans.sort(key=lambda x:x[0]);times=np.asarray([s[0] for s in scans],np.int64);print('Loaded',len(images),'images,',len(scans),'scans and recorded TF',flush=True)
    bev=ThetaBevNode();bev.buffer=buffer;bev.pub=Capture();ground=ThetaGroundCloudNode();ground.visibility_tf=buffer;ground.lidar_gate=True;ground.pub=Capture();ground.semantic_pub=None
    votes={name:np.zeros((h,w,4),np.float32) for name in ('baseline','temporal')};coverage=np.zeros((h,w),np.uint16);frame_rows=[];skipped=[];maps=None
    for index,msg in sorted(images.items()):
        center=min(manifest['centers'],key=lambda c:abs(c-index));ns=manifest['frames'][str(index)]['stamp_ns'];k=int(np.searchsorted(times,ns));near=min((i for i in [k-1,k] if 0<=i<len(scans)),key=lambda i:abs(int(times[i])-ns))
        if abs(int(times[near])-ns)>150000000:skipped.append({'frame':index,'reason':'scan time mismatch'});continue
        try:tf=buffer.lookup_transform('map','sirius3/base_footprint',msg.header.stamp)
        except Exception as e:skipped.append({'frame':index,'reason':str(e)});continue
        ground.strict_scans={int(times[near]):scans[near][1]};bev.pub.msg=None;bev.receive(msg);bmsg=bev.pub.msg
        if bmsg is None:skipped.append({'frame':index,'reason':'no BEV'});continue
        image=np.frombuffer(bmsg.data,np.uint8).reshape(bmsg.height,bmsg.width,3)
        if maps is None:
            maps=[build_bev_to_view_maps(bev.calibration,yaw,bmsg.width,(640,480),120,1.2,4.8) for yaw in (0,180)]
            # Inverse of perspective rays with u+.5/v+.5 pixel-center convention.
            maps=[(u-.5,v-.5,valid) for u,v,valid in maps]
        clouds={};sample_counts={}
        for name in ('baseline','temporal'):
            samples=[]
            for yaw,(u,v,valid) in zip((0,180),maps):
                folder=root/f'seg{center}'/f'frame{index}';stem=f'yaw{yaw}.'
                if name=='temporal':folder=root/'tactile'/f'seg{center}'/f'frame{index}';stem+=a.method+'.'
                cls=cv2.imread(str(folder/(stem+'classes.png')),0);score=cv2.imread(str(folder/(stem+'scores.png')),0)
                if cls is None or score is None:raise RuntimeError('Missing cached inference')
                samples.append((cls,score,u,v,valid,False))
            semantic,_=project_labels(samples,.5,image.shape[:2]);ground.pub.msg=None;ground._emit(bmsg.header.stamp,bmsg.header.frame_id,image,semantic,publish_main=True,publish_semantic=False)
            cloud=ground.pub.msg
            if cloud is None:continue
            assert cloud.point_step==DTYPE.itemsize;points=np.frombuffer(cloud.data,DTYPE);clouds[name]=points
            q,t=tf.transform.rotation,tf.transform.translation;rot=quaternion_to_matrix(q.x,q.y,q.z,q.w);xyz=np.column_stack([points['x'],points['y'],points['z']])@rot.T+[t.x,t.y,t.z]
            gx=np.floor((xyz[:,0]-origin[0])/res).astype(int);gy=np.floor((xyz[:,1]-origin[1])/res).astype(int);sid=points['semantic_id'];weight=points['weight'];inside=(gx>=0)&(gx<w)&(gy>=0)&(gy<h)
            valid=inside&(sid>=3)&(sid<=6);np.add.at(votes[name],(gy[valid],gx[valid],sid[valid]-3),weight[valid]);sample_counts[name]=int(valid.sum())
            if name=='temporal':
                np.add.at(coverage,(gy[inside],gx[inside]),1)
                candidates=np.flatnonzero(inside);candidates=candidates[missing_texture[h-1-gy[candidates],gx[candidates]]]
                packed=points['rgb'][candidates].copy().view(np.uint32);bgr=np.column_stack([packed&255,(packed>>8)&255,(packed>>16)&255]).astype(np.float32)
                np.add.at(fill_weight,(gy[candidates],gx[candidates]),weight[candidates]);np.add.at(fill_sum,(gy[candidates],gx[candidates]),bgr*weight[candidates,None])
        if len(clouds)==2:
            for field in ('x','y','z','rgb','weight'):np.testing.assert_array_equal(clouds['baseline'][field],clouds['temporal'][field])
        if 'temporal' not in clouds:skipped.append({'frame':index,'reason':'no visible ground'});continue
        frame_rows.append({'frame':index,'segment':center,'stamp_ns':ns,'scan_stamp_ns':int(times[near]),'xyz':[t.x,t.y,t.z],'quaternion':[q.x,q.y,q.z,q.w],'ground_points':len(clouds['temporal']),'semantic_samples':sample_counts})
        if len(frame_rows)%25==0:print('Mapped',len(frame_rows),'frames',flush=True)
    labels={}
    for name,grid in votes.items():labels[name]=np.where(grid.sum(axis=2)>=.3,np.argmax(grid,axis=2)+3,0).astype(np.uint8)
    assert np.all(labels['temporal'][labels['baseline']==4]==4),'Original tactile map cells were lost'
    changed=labels['temporal']!=labels['baseline'];assert np.all(labels['temporal'][changed]==4),'Unexpected changes outside tactile class'
    prefix=out/'road_temporal';cv2.imwrite(str(prefix)+'.colored.pgm',labels['temporal'][::-1]);cv2.imwrite(str(out/'baseline.class_ids.png'),labels['baseline'][::-1]);cv2.imwrite(str(out/'semantic_coverage.png'),((coverage>0)*255).astype(np.uint8)[::-1]);merged=texture.copy();fill=(fill_weight>0)[::-1]&missing_texture
    color=np.rint(fill_sum/np.maximum(fill_weight,1e-8)[...,None]).clip(0,255).astype(np.uint8)[::-1]
    merged[fill,:3]=color[fill];merged[fill,3]=255
    np.testing.assert_array_equal(merged[~missing_texture],texture[~missing_texture])
    assert not ((labels['temporal'][::-1]>0)&(merged[...,3]==0)).any(), 'Semantic cells outside RGB support'
    cv2.imwrite(str(prefix)+'.texture.png',merged)
    metadata.update({'semantic_encoding':'class_id','semantic_source':'ten evaluated dense intervals; all other regions unknown','temporal_method':a.method});(out/'road_temporal.colored.json').write_text(json.dumps(metadata,indent=2));(out/'calibration_used.yaml').write_text(yaml.safe_dump(bev.calibration));np.savez_compressed(out/'map_evidence.npz',baseline_votes=votes['baseline'],temporal_votes=votes['temporal'],sample_coverage=coverage)
    result={'bag':manifest['bag'],'method':a.method,'frame_count':len(frame_rows),'frames':frame_rows,'skipped':skipped,'texture_source':str(a.texture.resolve()),'texture_sha256':hashlib.sha256(a.texture.read_bytes()).hexdigest(),'same_geometry_rgb_weights_asserted':True,'existing_observed_rgb_unchanged':True,'texture_holes_filled_cells':int(fill.sum()),'half_pixel_inverse_compensation':-.5,'semantic_min_weight':.3,'lidar_max_age_sec':.15,'lidar_margin_m':.10,'lidar_obstacle_padding_m':.25,'grid':{'origin':origin.tolist(),'resolution':res,'width':w,'height':h},'baseline_cells':{str(i):int((labels['baseline']==i).sum()) for i in range(3,7)},'temporal_cells':{str(i):int((labels['temporal']==i).sum()) for i in range(3,7)},'new_tactile_cells':int(changed.sum()),'semantic_observed_cells':int((labels['temporal']>0).sum()),'note':'Full-route RGB context; semantic inference only170 selected frames. Not a complete semantic map or occupancy-cost validation.'};(out/'manifest.json').write_text(json.dumps(result,indent=2));ground.destroy_node();bev.destroy_node();rclpy.shutdown();print('MAP COMPLETE',len(frame_rows),'frames;',int(changed.sum()),'new tactile cells',flush=True)


if __name__=='__main__':main()
