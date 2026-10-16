"""Joint scan/plane refinement. Planes are local observations, never a flat-world prior.

Each view contributes its point mean and covariance, preserving the exact mean
squared point-to-plane error without retaining a residual for every laser return.
The voxel plane association follows the approach used by LiDAR bundle adjustment;
this is an explicit plane-landmark implementation, not BALM's analytical solver.
"""
import numpy as np
from collections import defaultdict
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import spsolve
from scipy.spatial.transform import Rotation


def shared_plane_observations(clouds, poses, scan_normals, scan_labels):
    """Associate raw scan plane identities through spatial overlap.

    A single observed plane can span many voxels. Keeping that measured identity
    prevents independent voxel planes from following a bent trajectory. Different
    planes observed in the same scan veto a merge, including slope transitions.
    """
    points, owners, identities, normals = [], [], [], []
    feature_view, feature_center, feature_normal = [], [], []
    for view, (p, ns, labels, pose) in enumerate(zip(clouds, scan_normals, scan_labels, poses)):
        world = p@pose[:3, :3].T+pose[:3, 3]
        for label in np.unique(labels):
            indices = labels == label
            number = len(feature_view)
            q = world[indices]
            n = pose[:3, :3]@ns[indices].mean(axis=0)
            n /= np.linalg.norm(n)
            feature_view.append(view); feature_center.append(q.mean(axis=0)); feature_normal.append(n)
            points.append(q); owners.extend([view]*len(q)); identities.extend([number]*len(q))
    if not points:
        return [], []
    world = np.concatenate(points)
    identities = np.asarray(identities)
    feature_view = np.asarray(feature_view)
    feature_center, feature_normal = np.asarray(feature_center), np.asarray(feature_normal)
    axis = np.argmax(abs(feature_normal), axis=1)
    orientation = 2*axis+(feature_normal[np.arange(len(axis)), axis] > 0)
    cells = np.column_stack([np.floor((world+.3)/.75).astype(np.int64), orientation[identities]])
    unique, groups = np.unique(cells, axis=0, return_inverse=True)
    order = np.argsort(groups)
    cuts = np.r_[0, np.flatnonzero(np.diff(groups[order]))+1, len(order)]
    matches = defaultdict(list)
    for lo, hi in zip(cuts[:-1], cuts[1:]):
        selected = order[lo:hi]
        features, counts = np.unique(identities[selected], return_counts=True)
        features = features[counts >= 3]
        if len(features) < 2:
            continue
        center = world[selected].mean(axis=0)
        distances = np.abs(np.einsum('ni,ni->n', feature_normal[features], center-feature_center[features]))
        features = features[distances < .15]
        for k, i in enumerate(features):
            for j in features[k+1:]:
                if feature_view[i] == feature_view[j] or feature_normal[i]@feature_normal[j] < np.cos(np.deg2rad(5.)):
                    continue
                matches[(int(i), int(j))].append(center)
    parent = np.arange(len(feature_view))
    members = {i: [i] for i in parent}
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return int(i)
    candidates = sorted(matches.items(), key=lambda item: len(item[1]), reverse=True)
    for (i, j), centers in candidates:
        if len(centers) < 3:
            continue
        q = np.asarray(centers)
        if np.linalg.eigvalsh(np.cov(q.T))[1] < .04:
            continue  # shared line/corner cannot prove that two surfaces are one plane
        i, j = root(i), root(j)
        if i == j:
            continue
        combined = members[i]+members[j]
        average = feature_normal[combined].mean(axis=0)
        average /= np.linalg.norm(average)
        if np.min(feature_normal[combined]@average) < np.cos(np.deg2rad(3.)):
            continue
        valid = True
        # If a scan observes two distinct planes, temporal associations must not
        # turn them into one (e.g. a flat landing next to an inclined ramp).
        for a in members[i]:
            for b in members[j]:
                if feature_view[a] != feature_view[b]:
                    continue
                if (feature_normal[a]@feature_normal[b] < np.cos(np.deg2rad(1.))
                        or abs(feature_normal[a]@(feature_center[b]-feature_center[a])) > .03):
                    valid = False
                    break
            if not valid:
                break
        if valid:
            parent[j] = i
            members[i] = combined
            del members[j]
    roots = np.array([root(i) for i in identities])
    local = np.concatenate([p[labels == label] for p, labels in zip(clouds, scan_labels)
                            for label in np.unique(labels)])
    owners = np.asarray(owners)
    landmarks, observations = [], []
    for identity in np.unique(roots):
        indices = np.flatnonzero(roots == identity)
        views = np.unique(owners[indices])
        if len(views) < 3:
            continue
        q = world[indices]
        center = q.mean(axis=0)
        eig, vectors = np.linalg.eigh((q-center).T@(q-center)/len(q))
        if eig[1] < .01:
            continue
        number = len(landmarks)
        landmarks.append((vectors[:, 0], center, vectors[:, 1:]))
        for view in views:
            p = local[indices[owners[indices] == view]]
            mean = p.mean(axis=0)
            values, axes = np.linalg.eigh((p-mean).T@(p-mean)/len(p))
            observations.append((number, int(view), mean, axes*np.sqrt(np.maximum(values, 0.))))
    return landmarks, observations


def plane_observations(clouds, poses, voxel=2., sigma=.05, scan_normals=None):
    local = np.concatenate(clouds)
    owners = np.repeat(np.arange(len(clouds)), [len(c) for c in clouds])
    rotations = np.array([p[:3, :3] for p in poses])
    positions = np.array([p[:3, 3] for p in poses])
    world = np.einsum('nij,nj->ni', rotations[owners], local) + positions[owners]
    cells = np.floor(world/voxel).astype(np.int64)
    if scan_normals is not None:
        normal = np.einsum('nij,nj->ni', rotations[owners], np.concatenate(scan_normals))
        axis = np.argmax(abs(normal), axis=1)
        orientation = 2*axis+(normal[np.arange(len(normal)), axis] > 0)
        # A wall and floor in one spatial voxel are distinct surfaces. Scanner-
        # oriented signs also keep opposite sides of a thin wall separate.
        cells = np.column_stack([cells, orientation])
    _, groups = np.unique(cells, axis=0, return_inverse=True)
    order = np.argsort(groups)
    cuts = np.r_[0, np.flatnonzero(np.diff(groups[order]))+1, len(order)]
    planes, observations = [], []
    for lo, hi in zip(cuts[:-1], cuts[1:]):
        indices = order[lo:hi]
        if len(indices) < 30 or len(np.unique(owners[indices])) < 3:
            continue
        points = world[indices]
        center = points.mean(axis=0)
        values, vectors = np.linalg.eigh((points-center).T@(points-center)/len(points))
        if values[1] < .01 or values[0] > min((2*sigma)**2, .03*values[1]):
            continue
        view_rows = []
        for view in np.unique(owners[indices]):
            p = local[indices[owners[indices] == view]]
            if len(p) < 8:
                continue
            mean = p.mean(axis=0)
            eig, axes = np.linalg.eigh((p-mean).T@(p-mean)/len(p))
            # Covariance square root: sum of the three residual squares gives
            # n^T R covariance R^T n, including each scan's internal thickness.
            basis = axes*np.sqrt(np.maximum(eig, 0.))
            view_rows.append((int(view), mean, basis))
        if len(view_rows) < 3:
            continue
        number = len(planes)
        normal = vectors[:, 0]
        planes.append((normal, center, vectors[:, 1:]))
        observations.extend((number, *row) for row in view_rows)
    return planes, observations


def refine_surfaces(poses, cache, edges, args, graph_residual, progress=None, edge_residual=None):
    """Optimize scan poses and shared local planes while retaining motion factors."""
    from pathlib import Path
    from mapping_geometry import registration_cloud
    clouds, scan_normals, scan_labels = [], [], []
    for i in range(len(poses)):
        with np.load(Path(cache)/f'{i:06d}.npz') as frame:
            p = frame['xyz'].astype(float)
        p = p[np.linalg.norm(p, axis=1) < 15.]
        pc, labels = registration_cloud(p, max(args.voxel, .15), return_labels=True)
        planar = labels >= 0
        clouds.append(np.asarray(pc.points)[planar])
        scan_normals.append(np.asarray(pc.normals)[planar])
        scan_labels.append(labels[planar])
    reports = []
    current = [p.copy() for p in poses]
    for iteration in range(getattr(args, 'surface_rounds', 3)):
        landmarks, observations = shared_plane_observations(clouds, current, scan_normals, scan_labels)
        local_landmarks, local_observations = plane_observations(
            clouds, current, sigma=args.lidar_sigma, scan_normals=scan_normals)
        offset = len(landmarks)
        landmarks.extend(local_landmarks)
        observations.extend((o[0]+offset, *o[1:]) for o in local_observations)
        if not landmarks:
            reports.append({'planes': 0, 'reason': 'insufficient shared planar observations'})
            break
        n0 = np.array([p[0] for p in landmarks])
        anchors = np.array([p[1] for p in landmarks])
        tangent = np.array([p[2] for p in landmarks])
        plane = np.array([o[0] for o in observations])
        views = np.array([o[1] for o in observations])
        means = np.array([o[2] for o in observations])
        bases = np.array([o[3] for o in observations])
        pose_size = 6*(len(current)-1)
        pose_x = np.array([np.r_[p[:3, 3], Rotation.from_matrix(p[:3, :3]).as_rotvec()]
                           for p in current])
        x0 = np.r_[pose_x[1:].ravel(), np.zeros(3*len(landmarks))]
        def unpack(x):
            state = np.vstack([pose_x[0], x[:pose_size].reshape(-1, 6)])
            r = Rotation.from_rotvec(state[:, 3:]).as_matrix()
            return state, r
        def residual(x):
            if progress is not None:
                progress.evaluated()
            state, r = unpack(x)
            parameters = x[pose_size:].reshape(-1, 3)
            n = n0 + np.einsum('nij,nj->ni', tangent, parameters[:, :2])
            n /= np.linalg.norm(n, axis=1)[:, None]
            centers = np.einsum('nij,nj->ni', r[views], means) + state[views, :3]
            normals = n[plane]
            center_error = np.einsum('ni,ni->n', normals, centers-anchors[plane])+parameters[plane, 2]
            local_normals = np.einsum('nji,nj->ni', r[views], normals)
            spread_error = np.einsum('nji,nj->ni', bases, local_normals)
            surfaces = np.column_stack([center_error, spread_error]).ravel()/args.lidar_sigma
            graph_poses = np.repeat(np.eye(4)[None], len(state), axis=0)
            graph_poses[:, :3, :3] = r
            graph_poses[:, :3, 3] = state[:, :3]
            return np.r_[surfaces, graph_residual(graph_poses)]
        def skew(v):
            z = np.zeros(len(v))
            return np.stack([z, -v[:, 2], v[:, 1], v[:, 2], z, -v[:, 0],
                             -v[:, 1], v[:, 0], z], axis=1).reshape(-1, 3, 3)
        def jacobian(x):
            state, r = unpack(x)
            w = state[:, 3:]
            theta = np.linalg.norm(w, axis=1)
            k = skew(w)
            t2 = theta**2
            aa = np.divide(1-np.cos(theta), t2, out=np.full_like(theta, .5), where=theta > 1e-5)
            bb = np.divide(theta-np.sin(theta), theta*t2, out=np.full_like(theta, 1/6), where=theta > 1e-5)
            jr = np.eye(3)-aa[:, None, None]*k+bb[:, None, None]*(k@k)
            parameters = x[pose_size:].reshape(-1, 3)
            raw = n0+np.einsum('nij,nj->ni', tangent, parameters[:, :2])
            norm = np.linalg.norm(raw, axis=1)
            n = raw/norm[:, None]
            dn = (tangent-n[:, :, None]*np.einsum('ni,nij->nj', n, tangent)[:, None, :])/norm[:, None, None]
            normals = n[plane]
            local_n = np.einsum('nji,nj->ni', r[views], normals)
            vectors = np.concatenate([means[:, None, :], bases.transpose(0, 2, 1)], axis=1)
            rot_jac = np.cross(vectors, local_n[:, None, :])@jr[views]
            pose_jac = np.zeros((len(views), 4, 6))
            pose_jac[:, 0, :3] = normals
            pose_jac[:, :, 3:] = rot_jac
            world_vectors = np.einsum('nij,nkj->nki', r[views], vectors)
            world_vectors[:, 0] += state[views, :3]-anchors[plane]
            plane_jac = np.zeros((len(views), 4, 3))
            plane_jac[:, :, :2] = world_vectors@dn[plane]
            plane_jac[:, 0, 2] = 1.
            rr = np.arange(4*len(views)).reshape(-1, 4, 1)
            valid = views > 0
            row_parts = [np.broadcast_to(rr, plane_jac.shape).ravel(),
                         np.broadcast_to(rr, pose_jac.shape)[valid].ravel()]
            col_parts = [np.broadcast_to((pose_size+3*plane)[:, None, None]+np.arange(3), plane_jac.shape).ravel(),
                         np.broadcast_to((6*(views-1))[:, None, None]+np.arange(6), pose_jac.shape)[valid].ravel()]
            data_parts = [plane_jac.ravel()/args.lidar_sigma, pose_jac[valid].ravel()/args.lidar_sigma]
            gp = np.repeat(np.eye(4)[None], len(state), axis=0)
            gp[:, :3, :3], gp[:, :3, 3] = r, state[:, :3]
            row = 4*len(views)
            for edge in edges:
                i, j = edge[:2]
                value = edge_residual(gp[i], gp[j], edge)
                count = len(value)
                for view in set((i, j)):
                    if not view:
                        continue
                    block = np.zeros((count, 6))
                    for axis in range(6):
                        q = gp[view].copy()
                        if axis < 3:
                            q[axis, 3] += 1e-6
                        else:
                            v = state[view, 3:].copy()
                            v[axis-3] += 1e-6
                            q[:3, :3] = Rotation.from_rotvec(v).as_matrix()
                        sample = edge_residual(q if view == i else gp[i], q if view == j else gp[j], edge)
                        block[:, axis] = (sample-value)/1e-6
                    row_parts.append(np.broadcast_to(np.arange(row, row+count)[:, None], block.shape).ravel())
                    col_parts.append(np.broadcast_to(np.arange(6*(view-1), 6*view), block.shape).ravel())
                    data_parts.append(block.ravel())
                row += count
            return coo_matrix((np.concatenate(data_parts), (np.concatenate(row_parts), np.concatenate(col_parts))),
                              shape=(row, len(x))).tocsr()
        before = residual(x0)
        def cost(v):
            v = np.abs(v)
            return float(np.sum(np.where(v <= 2., .5*v*v, 2.*v-2.)))
        x, value, damping, success = x0.copy(), before, 1e-3, False
        for evaluations in range(1, min(args.max_iterations, 60)+1):
            jac = jacobian(x)
            weights = np.sqrt(np.minimum(1., 2./np.maximum(np.abs(value), 1e-12)))
            jac = jac.multiply(weights[:, None]).tocsr()
            h = (jac.T@jac).tocsc()
            gradient = jac.T@(weights*value)
            diagonal = np.maximum(h.diagonal(), 1e-6)
            old_cost = cost(value)
            accepted = False
            for attempt in range(8):
                step = spsolve(h+diags(damping*diagonal), -gradient)
                if not np.isfinite(step).all():
                    damping *= 10.
                    continue
                candidate = residual(x+step)
                new_cost = cost(candidate)
                if new_cost < old_cost:
                    x += step
                    value = candidate
                    damping = max(1e-8, damping/3.)
                    accepted = True
                    success = (old_cost-new_cost < 1e-6*max(old_cost, 1.) or np.linalg.norm(step) < 1e-5)
                    break
                damping *= 10.
            if success or not accepted:
                break
        state, r = unpack(x)
        for i in range(len(current)):
            current[i][:3, :3] = r[i]
            current[i][:3, 3] = state[i, :3]
        surface_rows = 4*len(observations)
        report = {'round': iteration+1, 'planes': len(landmarks), 'view_plane_observations': len(observations),
                  'success': success, 'evaluations': evaluations,
                  'cost': cost(value), 'initial_surface_rms_m': float(np.sqrt(np.mean(before[:surface_rows]**2))*args.lidar_sigma),
                  'final_surface_rms_m': float(np.sqrt(np.mean(value[:surface_rows]**2))*args.lidar_sigma)}
        reports.append(report)
        print(f'Surface refinement {iteration+1}: {len(landmarks)} planes, '
              f'RMS {report["initial_surface_rms_m"]:.4f} -> {report["final_surface_rms_m"]:.4f} m', flush=True)
    return current, {'enabled': True, 'rounds': reports,
                     'model': 'joint local plane landmarks and scan poses; no floor-height or gravity clamp'}
