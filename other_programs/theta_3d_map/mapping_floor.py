"""Optional known-single-floor refinement; not a flat-world default.

The caller must explicitly establish that these views share one planar floor.
Individual scan planes are measured below the base, not fitted to the fused map.
Poses are adjusted; laser returns are never projected onto a fabricated plane.
"""
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


def measure_floor_scans(cache, count, mount):
    import open3d as o3d
    observations = []
    for i in range(count):
        with np.load(Path(cache)/f'{i:06d}.npz') as frame:
            points = frame['xyz'].astype(float)
        base = points @ mount[:3,:3].T + mount[:3,3]
        mask = ((base[:,2]>-.35)&(base[:,2]<.25)
                &(np.linalg.norm(base[:,:2],axis=1)<6.))
        points = points[mask]
        if len(points)<80:
            continue
        # Reserve points for a local plane-thickness diagnostic.
        training = np.arange(len(points))%4 != 0
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[training]))
        o3d.utility.random.seed(42)
        equation, ids = cloud.segment_plane(.025,3,400)
        normal = np.asarray(equation[:3])
        if len(ids)<60 or abs((mount[:3,:3]@normal)[2])<np.cos(np.deg2rad(15.)):
            continue
        normal *= np.sign((mount[:3,:3]@normal)[2])
        center = points[training][ids].mean(axis=0)
        held = points[~training]
        distances = abs((held-center)@normal)
        # This classifies candidate floor returns; the residual is conditional,
        # not a surveyed ground-truth error or a test of all lower-scene points.
        held = held[distances<.06]
        if len(held)<20:
            continue
        observations.append(dict(view=i,normal=normal,center=center,
            points=points[training][ids],heldout=held,
            local_heldout_p95_m=float(np.percentile(abs((held-center)@normal),95))))
    return observations


def refine_single_floor(poses, observations, links=(), height_sigma=.015,
                        normal_sigma_deg=.25, max_evaluations=100):
    """Soft single-floor pose prior with first pose fixed and XY unchanged.

The common plane is measured at the first observed scan. It is not an assumed
world Z value or an IMU calibration. Every fifth scan is held out from fitting.
"""
    if len(observations)<6 or observations[0]['view']!=0:
        raise ValueError('Known-floor refinement needs a floor observation in the first scan and >=6 views')
    initial = np.asarray(poses)
    origin = observations[0]
    normal = initial[0,:3,:3]@origin['normal']
    center = initial[0,:3,:3]@origin['center']+initial[0,:3,3]
    offset = -normal@center
    # Keep both observed endpoints constrained; held-out scans interpolate
    # between observations rather than requiring unobservable extrapolation.
    last_view=observations[-1]['view']
    train = [o for o in observations if o['view']%5!=4 or o['view']==last_view]
    heldout = [o for o in observations if o['view']%5==4 and o['view']!=last_view]
    views = np.array([o['view'] for o in train])
    normals = np.array([o['normal'] for o in train])
    means = np.array([o['center'] for o in train])
    links = sorted(set(tuple(sorted((int(i),int(j)))) for i,j in links if i!=j)
                   | {(i,i+1) for i in range(len(poses)-1)})
    a,b = np.asarray(links,dtype=int).T
    distance = np.linalg.norm(initial[a,:3,3]-initial[b,:3,3],axis=1)
    link_sigma = np.column_stack([.02+.01*distance,
        np.deg2rad(.2+.1*distance),np.deg2rad(.2+.1*distance)])
    def unpack(x):
        delta = np.vstack([np.zeros(3),x.reshape(-1,3)])
        axes = np.column_stack([delta[:,1:],np.zeros(len(delta))])
        rotations = Rotation.from_rotvec(axes).as_matrix() @ initial[:,:3,:3]
        positions = initial[:,:3,3].copy();positions[:,2]+=delta[:,0]
        return delta,rotations,positions
    def residual(x):
        delta,rotations,positions = unpack(x)
        world_normals = np.einsum('nij,nj->ni',rotations[views],normals)
        world_centers = np.einsum('nij,nj->ni',rotations[views],means)+positions[views]
        observed = np.column_stack([(world_centers@normal+offset)/height_sigma,
            (world_normals-normal)/np.deg2rad(normal_sigma_deg)]).ravel()
        smooth = ((delta[b]-delta[a])/link_sigma).ravel()
        anchor = (delta[1:]/np.array([2.,np.deg2rad(8.),np.deg2rad(8.)])).ravel()
        return np.r_[observed,smooth,anchor]
    size=3*(len(poses)-1)
    pattern=lil_matrix((4*len(views)+3*len(links)+size,size),dtype=int)
    for row,view in enumerate(views):
        if view:pattern[4*row:4*row+4,3*(view-1):3*view]=1
    start=4*len(views)
    for row,(i,j) in enumerate(links):
        for view in (i,j):
            if view:pattern[start+3*row:start+3*row+3,3*(view-1):3*view]=1
    start+=3*len(links)
    pattern[start:,:]=lil_matrix(np.eye(size,dtype=int))
    result=least_squares(residual,np.zeros(size),jac_sparsity=pattern.tocsr(),
        loss='huber',f_scale=2.,max_nfev=max_evaluations,x_scale='jac',ftol=1.e-7)
    delta,rotations,positions=unpack(result.x)
    candidate=initial.copy();candidate[:,:3,:3]=rotations;candidate[:,:3,3]=positions
    if not np.isfinite(candidate).all():raise ValueError('Nonfinite floor refinement')
    def quality(ps,rows):
        distances=[];angles=[]
        for o in rows:
            pose=ps[o['view']];world=o['heldout']@pose[:3,:3].T+pose[:3,3]
            distances.extend(abs(world@normal+offset))
            angles.append(np.rad2deg(np.arccos(np.clip((pose[:3,:3]@o['normal'])@normal,-1,1))))
        return dict(views=len(rows),point_p95_distance_m=float(np.percentile(distances,95)),
                    normal_p95_deg=float(np.percentile(angles,95))) if distances else dict(views=0)
    return list(candidate),dict(model='explicit known-single-planar-floor; measured first-scan plane; soft pose constraints',
        assumption='user-established same planar indoor floor; not an outdoor default',
        success=bool(result.success),evaluations=result.nfev,cost=float(result.cost),
        reference_plane=[*normal.tolist(),float(offset)],floor_observation_views=len(observations),
        fitted_views=len(train),heldout_views=len(heldout),
        heldout_before=quality(initial,heldout),heldout_after=quality(candidate,heldout),
        local_scan_floor_thickness_p95_m=float(np.percentile([o['local_heldout_p95_m'] for o in observations],95)),
        max_z_correction_m=float(abs(delta[:,0]).max()),
        max_tilt_correction_deg=float(np.rad2deg(np.linalg.norm(delta[:,1:],axis=1).max())),
        xy_positions_fixed=True,point_projection_to_plane=False,absolute_accuracy_measured=False)


def floor_frame(plane):
    """Choose a map gauge whose Z axis is the measured reference floor normal."""
    normal=np.asarray(plane[:3]);normal/=np.linalg.norm(normal)
    axis=np.cross(normal,[0.,0.,1.]);s=np.linalg.norm(axis);c=normal[2]
    rotation=(np.eye(3) if s<1.e-12 else Rotation.from_rotvec(axis/s*np.arctan2(s,c)).as_matrix())
    transform=np.eye(4);transform[:3,:3]=rotation;transform[2,3]=plane[3]
    return transform


def refine_known_floor(cache, poses, times, mount, links=()):
    """Validate an explicit single-floor candidate and choose its map frame."""
    from mapping_quality import floor_overlap_quality, trajectory_jump_quality
    observations=measure_floor_scans(cache,len(poses),mount)
    candidate,refinement=refine_single_floor(poses,observations,links)
    before=floor_overlap_quality(cache,poses,times,mount)
    after=floor_overlap_quality(cache,candidate,times,mount,reference=before)
    walls=wall_revisit_quality(cache,poses,candidate,before['pairs'],
                             np.array(refinement['reference_plane'][:3]))
    jumps=trajectory_jump_quality(candidate,times)
    key='p95_pair_p95_height_difference_m'
    if (not refinement['success'] or jumps['jump_count']>0
            or refinement['heldout_after'].get('point_p95_distance_m',np.inf)
               >=refinement['heldout_before'].get('point_p95_distance_m',np.inf)
            or after.get('revisit_pairs',0)<.9*before.get('revisit_pairs',0)
            or after.get(key,np.inf)>before.get(key,0.)+.002
            or after.get('max_pair_p95_height_difference_m',np.inf)
               >before.get('max_pair_p95_height_difference_m',0.)+.005
            or not walls['pairs'] or walls['after_p95_m']>walls['before_p95_m']+.02):
        raise ValueError('Known-floor candidate rejected: floor/wall/trajectory consistency insufficient')
    level=floor_frame(refinement['reference_plane'])
    refinement.update(accepted=True,floor_quality_before=before,floor_quality_candidate=after,
        wall_quality=walls,frame_transform_old_map_to_new_map=level.tolist(),
        map_frame='reference-floor normal defines map Z; not calibrated IMU gravity')
    print('単一床補正: 未使用スキャンの床距離p95 '
          f"{refinement['heldout_before']['point_p95_distance_m']*100:.2f} "
          f"-> {refinement['heldout_after']['point_p95_distance_m']*100:.2f} cm",flush=True)
    return [level@pose for pose in candidate],refinement


def wall_revisit_quality(cache, before, after, pairs, reference_normal):
    """Hold fixed baseline wall correspondences; do not optimize on this metric."""
    from mapping_geometry import registration_cloud
    from scipy.spatial import cKDTree
    ids=sorted({p[k] for p in pairs for k in ('from','to')})
    scans={}
    for i in ids:
        with np.load(Path(cache)/f'{i:06d}.npz') as frame:p=frame['xyz'].astype(float)
        p=p[np.linalg.norm(p,axis=1)<10.]
        pc,labels=registration_cloud(p,.2,return_labels=True)
        points=np.asarray(pc.points);normals=np.asarray(pc.normals)
        world_n=normals@before[i][:3,:3].T
        mask=(labels>=0)&(abs(world_n@reference_normal)<.3)
        scans[i]=(points[mask],normals[mask])
    rows=[]
    for pair in pairs:
        i,j=pair['from'],pair['to'];p,n=scans[i];q,m=scans[j]
        if min(len(p),len(q))<30:continue
        x=p@before[i][:3,:3].T+before[i][:3,3]
        y=q@before[j][:3,:3].T+before[j][:3,3]
        distance,nearest=cKDTree(y).query(x)
        normal=m[nearest]@before[j][:3,:3].T
        selected=(distance<.2)&(abs(np.sum((n@before[i][:3,:3].T)*normal,axis=1))>.95)
        if selected.sum()<20:continue
        old=abs(np.sum((x[selected]-y[nearest[selected]])*normal[selected],axis=1))
        xx=p[selected]@after[i][:3,:3].T+after[i][:3,3]
        yy=q[nearest[selected]]@after[j][:3,:3].T+after[j][:3,3]
        nn=m[nearest[selected]]@after[j][:3,:3].T
        new=abs(np.sum((xx-yy)*nn,axis=1))
        rows.append(dict(from_view=i,to_view=j,matches=int(selected.sum()),
            before_p95_m=float(np.percentile(old,95)),after_p95_m=float(np.percentile(new,95))))
    return dict(pairs=len(rows),fixed_correspondences=True,used_as_constraint=False,
        before_p95_m=float(np.percentile([r['before_p95_m'] for r in rows],95)) if rows else None,
        after_p95_m=float(np.percentile([r['after_p95_m'] for r in rows],95)) if rows else None,
        details=rows)
