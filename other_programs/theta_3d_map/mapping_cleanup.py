"""Optional scan-evidence finishing, independent of floor/ROI and object classes.

A local heuristic, not a reproduction of any published dynamic-mapping method.
Unknown space and transient observation alone do not justify removing a point.
"""
import numpy as np
from scipy.spatial import cKDTree


def polish_map(xyz, keep, frame_points, poses, times, args, progress=None):
    import open3d as o3d
    if times is None:
        raise ValueError('Map finishing requires frame timestamps')
    times=np.asarray(times,dtype=float)
    if times.shape!=(len(poses),) or not np.isfinite(times).all() or np.any(np.diff(times)<0):
        raise ValueError('Map finishing requires ordered finite timestamps')
    xyz=np.asarray(xyz);keep=np.asarray(keep)
    if keep.dtype!=bool or keep.shape!=(len(xyz),) or not np.isfinite(xyz).all():
        raise ValueError('Invalid map/mask for finishing')
    policy=dict(component_radius_m=getattr(args,'dynamic_fragment_radius',.12),
        max_component_points=getattr(args,'dynamic_fragment_max_points',120),
        max_component_diameter_m=getattr(args,'dynamic_fragment_max_extent',.8),
        seed_radius_m=getattr(args,'dynamic_fragment_seed_radius',.25),
        hit_radius_m=getattr(args,'dynamic_fragment_match_radius',.08),
        persistent_span_s=getattr(args,'dynamic_fragment_revisit_gap',8.),
        min_persistent_hits=3,min_free_observations=3,min_free_span_s=2.,
        protect_connected_surfaces=True,protect_component_on_any_persistent_hit=True,
        after_last_hit_s=.5,min_beams=3,beam_tube_radius_m=.05,
        max_beam_angle_deg=.5,clearance_m=.2,range_margin_ratio=.005,
        slender_span_m=.2,slender_variance_ratio=9.,
        planar_min_points=6,planar_min_variance_fraction=.01)
    report=dict(enabled=True,method='local ray-consensus finishing with temporal/static-feature protection',
        policy=policy,removed_points=0,removed_indices=[],floor_or_roi_mask=False,
        limitation='finite ray footprints and pose error can misclassify thin or single-view static details; no semantic ground truth')
    ids=np.flatnonzero(keep)
    if not len(ids) or not len(poses):return keep.copy(),report
    points=xyz[ids].astype(float)
    labels=np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).cluster_dbscan(
        eps=policy['component_radius_m'],min_points=1))
    counts=np.bincount(labels);small=counts[labels]<=policy['max_component_points']
    selected=np.zeros(len(points),bool);protected_slender=np.zeros(len(points),bool)
    protected_planar=np.zeros(len(points),bool)
    eligible=np.zeros(len(points),bool)
    order=np.argsort(labels,kind='stable');offsets=np.r_[0,np.cumsum(counts)]
    for label in np.unique(labels[small]):
        group=order[offsets[label]:offsets[label+1]];p=points[group];center=p.mean(axis=0)
        span=2*np.linalg.norm(p-center,axis=1).max()
        if span>policy['max_component_diameter_m']:continue
        if len(group)>=3:
            values=np.linalg.eigvalsh((p-center).T@(p-center))
            if span>=policy['slender_span_m'] and values[-1]>policy['slender_variance_ratio']*max(values[-2],1.e-10):
                protected_slender[group]=True
                continue
            if len(group)>=policy['planar_min_points'] and values[0]<policy['planar_min_variance_fraction']*max(values.sum(),1.e-10):
                protected_planar[group]=True
                continue
        selected[group]=True
        eligible[group]=True
    if np.any(~keep):
        selected|=cKDTree(xyz[~keep]).query(points,distance_upper_bound=policy['seed_radius_m'])[0]<=policy['seed_radius_m']
    selected&=~(protected_slender|protected_planar)
    subset=np.flatnonzero(selected);candidate=points[subset];n=len(candidate)
    report.update(candidate_points=n,protected_slender_points=int(protected_slender.sum()),
                  protected_planar_points=int(protected_planar.sum()))
    if not n:return keep.copy(),report
    hits=np.zeros(n,np.uint32);first=np.full(n,np.inf);last=np.full(n,-np.inf)
    free_count=np.zeros(n,np.uint32)
    free_first=np.full(n,np.inf);free_last=np.full(n,-np.inf)
    max_range=getattr(args,'dynamic_range',30.)
    for i,pose in enumerate(poses):
        scan=np.asarray(frame_points(i),dtype=float)
        ranges=np.linalg.norm(scan,axis=1)
        scan=scan[np.isfinite(scan).all(axis=1)&(ranges>0)&(ranges<=max_range)]
        if not len(scan):continue
        local=(candidate-pose[:3,3])@pose[:3,:3]
        distance=np.linalg.norm(local,axis=1)
        active=np.flatnonzero((distance>0)&(distance<=max_range))
        if not len(active):continue
        p=local[active];r=distance[active]
        seen=np.isfinite(cKDTree(scan).query(p,distance_upper_bound=policy['hit_radius_m'])[0])
        hit=active[seen];hits[hit]+=1;first[hit]=np.minimum(first[hit],times[i]);last[hit]=times[i]
        # A later direct return invalidates all earlier "departed" evidence.
        free_count[hit]=0;free_first[hit]=np.inf;free_last[hit]=-np.inf
        if len(scan)>=policy['min_beams']:
            depths=np.linalg.norm(scan,axis=1);directions=scan/depths[:,None]
            ray_tree=cKDTree(directions)
            angles,nearest=ray_tree.query(p/r[:,None],k=policy['min_beams'])
            allowed=np.minimum(2*np.sin(np.deg2rad(policy['max_beam_angle_deg'])/2),policy['beam_tube_radius_m']/r)
            margin=np.maximum(policy['clearance_m'],r*policy['range_margin_ratio'])
            empty=(angles[:,-1]<=allowed)&np.all(depths[nearest]>r[:,None]+margin[:,None],axis=1)&~seen
            # Any nearer return in the whole footprint vetoes a free-space vote,
            # even if it was not among the three closest ray directions.
            proposed=np.flatnonzero(empty)
            neighborhoods=ray_tree.query_ball_point(p[proposed]/r[proposed,None],allowed[proposed])
            for j,neighbors in zip(proposed,neighborhoods):
                if np.any(depths[neighbors]<=r[j]+margin[j]):empty[j]=False
            later=active[empty & (times[i]>last[active]+policy['after_last_hit_s'])]
            free_count[later]+=1
            free_first[later]=np.minimum(free_first[later],times[i]);free_last[later]=times[i]
        if progress is not None:
            progress.update(i+1,f'追加仕上げ: 再観測・レーザー通過証拠 {i+1}/{len(poses)}')
    persistent=(hits>=policy['min_persistent_hits'])&(last-first>=policy['persistent_span_s'])
    # Even one persistently observed point protects the whole small feature.
    persistent_labels=labels[subset[persistent]]
    protected_counts=np.bincount(persistent_labels,minlength=len(counts))
    protect_group=small[subset]&(protected_counts[labels[subset]]>0)
    contradicted=(hits>0)&~persistent&~protect_group&(free_count>=policy['min_free_observations'])&(free_last-free_first>=policy['min_free_span_s'])
    # Preserve large connected structures and every attached part. Their meaning
    # cannot be resolved safely from approximate ray footprints alone.
    remove=contradicted&eligible[subset]
    removed=ids[subset[remove]];result=keep.copy();result[removed]=False
    report.update(removed_points=len(removed),removed_indices=removed.tolist(),tested_keyframes=len(poses),
        persistent_points=int(persistent.sum()),protected_component_points=int(protect_group.sum()),
        protected_connected_points=int((contradicted&~eligible[subset]).sum()),
        no_hit_points=int((hits==0).sum()),points_with_later_free_evidence=int((free_count>0).sum()),
        evidence=dict(hits=hits.tolist(),free_after=free_count.tolist(),
                      candidate_indices=ids[subset].tolist(),persistent=persistent.tolist()))
    return result,report
