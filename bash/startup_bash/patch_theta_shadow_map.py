"""Replace selected frame votes in a full map, permitting only new tactile cells.

The full reference texture and all original tactile cells remain unchanged.
No coordinate-specific correction or invented line completion is performed.
"""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import cv2,numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('shadow',type=Path);p.add_argument('reference_map',type=Path);p.add_argument('output',type=Path);a=p.parse_args();region=a.shadow;out=a.output;out.mkdir(parents=True,exist_ok=True)
    cfg=json.loads((region/'dense/config.json').read_text());source=Path(cfg['source']);manifest=json.loads((source/'manifest.json').read_text());oldmeta=json.loads((a.reference_map/'manifest.json').read_text());selected=cfg['frames'];prepared=region/'map_input';prepared.mkdir(exist_ok=True);method='shade_wb45';processed=region/('processed_'+method)
    local={**manifest,'frames':{str(i):manifest['frames'][str(i)] for i in selected},'full_bag':False,'selection':'contiguous shadow experiment','camera_frames':len(selected)};(prepared/'manifest.json').write_text(json.dumps(local,indent=2));shutil.copy2(source/'calibration_used.yaml',prepared/'calibration_used.yaml');hashes={}
    for index in selected:
        center=min(manifest['centers'],key=lambda c:abs(c-index));old=source/'yellow'/f'seg{center}'/f'frame{index}';dest=prepared/f'seg{center}'/f'frame{index}';new=prepared/'yellow'/f'seg{center}'/f'frame{index}';dest.mkdir(parents=True,exist_ok=True);new.mkdir(parents=True,exist_ok=True)
        for yaw in (0,180):
            for ext in ('classes.png','scores.png'):
                shutil.copy2(old/f'yaw{yaw}.bidirectional.{ext}',dest/f'yaw{yaw}.{ext}');src=processed/f'frame{index}'/f'yaw{yaw}.tracked.{ext}';shutil.copy2(src,new/f'yaw{yaw}.bidirectional.{ext}');hashes[str(src.relative_to(region))]=hashlib.sha256(src.read_bytes()).hexdigest()
    (region/'map_input_sha256.json').write_text(json.dumps(hashes,indent=2));subset=region/'native_subset';script=Path(__file__).with_name('map_theta_sam3_tactile.py');env={**os.environ,'ROS_DOMAIN_ID':'73'}
    with (region/'native_subset.log').open('w') as log:subprocess.run([sys.executable,str(script),str(prepared),str(a.reference_map/'road_temporal.texture.png'),str(subset),'--semantic-dir','yellow','--allow-reclassification'],env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    sm=json.loads((subset/'manifest.json').read_text());assert sm['frame_count']==len(selected) and not sm['skipped'];oldframes={f['frame']:f for f in oldmeta['frames']}
    assert all(f['ground_points']==oldframes[f['frame']]['ground_points'] and f['xyz']==oldframes[f['frame']]['xyz'] for f in sm['frames'])
    old=np.load(a.reference_map/'map_evidence.npz');sub=np.load(subset/'map_evidence.npz');before=old['temporal_votes'];remaining=before-sub['baseline_votes'];minimum=float(remaining.min());assert minimum>=-.01,minimum;updated=np.maximum(remaining,0)+sub['temporal_votes'];prior=cv2.imread(str(a.reference_map/'road_temporal.colored.pgm'),0)
    proposed=np.where(updated.sum(axis=2)>=.3,np.argmax(updated,axis=2)+3,0).astype(np.uint8)[::-1];labels=prior.copy();added=(proposed==4)&(prior!=4);labels[added]=4
    assert np.all(labels[prior==4]==4);assert np.all(labels[labels!=prior]==4)
    texture=cv2.imread(str(a.reference_map/'road_temporal.texture.png'),-1);assert not ((labels>0)&(texture[...,3]==0)).any();cv2.imwrite(str(out/'road_temporal.colored.pgm'),labels);cv2.imwrite(str(out/'baseline.class_ids.png'),prior)
    shutil.copy2(a.reference_map/'road_temporal.texture.png',out/'road_temporal.texture.png');shutil.copy2(a.reference_map/'calibration_used.yaml',out/'calibration_used.yaml');shutil.copy2(a.reference_map/'semantic_coverage.png',out/'semantic_coverage.png');np.savez_compressed(out/'map_evidence.npz',baseline_votes=before,temporal_votes=updated,sample_coverage=old['sample_coverage'])
    meta={**oldmeta,'baseline_definition':'previous full-map temporal result','baseline_cells':{str(k):int((prior==k).sum()) for k in range(3,7)},'temporal_cells':{str(k):int((labels==k).sum()) for k in range(3,7)},'new_tactile_cells':int(added.sum()),'removed_tactile_cells':0,'changed_semantic_cells':int(added.sum()),'semantic_observed_cells':int((labels>0).sum()),'shadow_rescue':{'camera_frames':len(selected),'first_frame':min(selected),'last_frame':max(selected),'prompt':'yellow tactile paving in shadow','method':'shadow white balance + gamma0.45, auxiliary target-only512, dark/color gates, bidirectional tracking','score_min':.5,'yellow_fraction':.2,'saturation_min':45,'original_component_median_v_max':90,'only_tactile_additions':True,'same_point_counts_and_poses_checked':True,'subtraction_roundoff_minimum':minimum},'reference_map':str(a.reference_map.resolve()),'reference_labels_sha256':hashlib.sha256((a.reference_map/'road_temporal.colored.pgm').read_bytes()).hexdigest(),'entire_texture_unchanged':True,'label_policy':'Retain all old labels except new ID4 cells supported by updated native vote argmax.'};(out/'manifest.json').write_text(json.dumps(meta,indent=2))
    palette=json.loads((a.reference_map/'road_temporal.colored.json').read_text());palette.update({'semantic_source':'full bag plus validated shadow-only tactile additions','shadow_rescue':meta['shadow_rescue']});(out/'road_temporal.colored.json').write_text(json.dumps(palette,indent=2));print('SHADOW MAP COMPLETE',len(selected),'replaced frame votes;',int(added.sum()),'new tactile cells',flush=True)


if __name__=='__main__':main()
