"""Validate full-bag tactile unions and keep only tactile-class fusion scores.

Also upgrades the in-flight first full run, preserving its prior score PNGs.
No SAM3 inference or geometric/mask changes are involved.
"""
import argparse,hashlib,json,zipfile
from pathlib import Path
import cv2,numpy as np
from sirius_navigation.theta_semantic_temporal import merge_tactile_tracks


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('study',type=Path);a=p.parse_args();root=a.study
    manifest=json.loads((root/'manifest.json').read_text());assert json.loads((root/'completed.json').read_text())['complete']
    out=root/'runs'/'bidirectional_score_fix';out.mkdir(parents=True,exist_ok=True);changes=[];count=0;mask_pixels=0
    archive=out/'original_scores.zip'
    with zipfile.ZipFile(archive,'a',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        stored=set(z.namelist())
        for index in sorted(int(i) for i in manifest['frames']):
            center=min(manifest['centers'],key=lambda c:abs(c-index));folder=root/'yellow'/f'seg{center}'/f'frame{index}'
            for yaw in (0,180):
                stem=f'yaw{yaw}.'
                def read(ext):
                    image=cv2.imread(str(folder/(stem+ext)),0)
                    assert image is not None and image.shape==(480,640)
                    return image
                raw=read('classes.png');forward=read('causal.classes.png');fs=read('causal.scores.png');reverse=read('reverse.classes.png');rs=read('reverse.scores.png');old=read('bidirectional.classes.png');oldscore=read('bidirectional.scores.png')
                merged,scores=merge_tactile_tracks(raw,forward,fs,reverse,rs);np.testing.assert_array_equal(merged,old)
                protected=np.isin(raw,(1,2,3));np.testing.assert_array_equal(merged[protected],raw[protected]);assert np.all(scores[merged==4]>=128)
                mask_pixels+=int((merged==4).sum());changed=oldscore!=scores
                if changed.any():
                    path=folder/(stem+'bidirectional.scores.png');name=str(path.relative_to(root));before=hashlib.sha256(path.read_bytes()).hexdigest()
                    if name not in stored:z.write(path,name);stored.add(name)
                    assert cv2.imwrite(str(path),scores)
                    changes.append({'path':name,'changed_pixels':int(changed.sum()),'sha256_before':before,'sha256_after':hashlib.sha256(path.read_bytes()).hexdigest()})
                count+=1
    result={'complete':True,'observations':count,'expected_observations':len(manifest['frames'])*2,'all_tactile_masks_unchanged':True,'protected_classes_unchanged':True,'tactile_pixels':mask_pixels,'corrected_score_files':len(changes),'corrected_score_pixels':sum(r['changed_pixels'] for r in changes),'changes':changes,'reason':'When reverse adds tactile over road/unknown, use its tactile score; only max with forward score if forward also says tactile. No unrelated class confidence is inherited.'}
    assert count==result['expected_observations'];(root/'fusion_completed.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:v for k,v in result.items() if k!='changes'}),flush=True)


if __name__=='__main__':main()
