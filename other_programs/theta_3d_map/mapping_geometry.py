"""Reusable scan features. Fitted planes avoid normals blended across wall/floor edges."""
from collections import OrderedDict
import hashlib
import numpy as np

_FEATURES = OrderedDict()


def registration_cloud(points, voxel, planar=True, return_labels=False):
    import open3d as o3d
    points = np.ascontiguousarray(points)
    key = (hashlib.blake2b(points.view(np.uint8), digest_size=16).digest(), voxel, planar)
    if key in _FEATURES:
        _FEATURES.move_to_end(key)
        pc, mask, labels = _FEATURES[key]
        return (pc, labels) if return_labels else (pc, mask)
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pc = pc.voxel_down_sample(voxel)
    pc.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*4, max_nn=30))
    pts, normals = np.asarray(pc.points), np.array(pc.normals)
    mask = np.zeros(len(pts), dtype=bool)
    labels = np.full(len(pts), -1, dtype=np.int32)
    remaining = np.arange(len(pts))
    minimum = max(40, int(.04*len(pts)))
    if planar:
        for label in range(8):
            if len(remaining) < minimum:
                break
            sub = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[remaining]))
            o3d.utility.random.seed(42)
            equation, indices = sub.segment_plane(.03, 3, 180)
            if len(indices) < minimum:
                break
            normal = np.asarray(equation[:3])
            if normal@pts[remaining[indices]].mean(axis=0) > 0:
                normal = -normal  # oriented toward this observing scanner
            normals[remaining[indices]] = normal
            mask[remaining[indices]] = True
            labels[remaining[indices]] = label
            keep = np.ones(len(remaining), dtype=bool)
            keep[indices] = False
            remaining = remaining[keep]
    pc.normals = o3d.utility.Vector3dVector(normals)
    _FEATURES[key] = (pc, mask, labels)
    if len(_FEATURES) > 128:
        _FEATURES.popitem(last=False)
    return (pc, labels) if return_labels else (pc, mask)
