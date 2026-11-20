"""Timestamped offline poses and verified reuse of completed camera observations."""
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np


def read_trajectory(path):
    with np.load(path, allow_pickle=False) as data:
        stamps, poses = data['stamp_ns'], data['poses']
    if (stamps.dtype.kind not in 'iu' or stamps.ndim != 1 or len(stamps) < 2
            or poses.shape != (len(stamps), 3) or not np.isfinite(poses).all()
            or stamps[0] <= 0 or np.any(np.diff(stamps) <= 0)):
        raise ValueError('Invalid offline trajectory: expected sorted nanosecond stamps and finite x/y/yaw')
    return zip(stamps, *poses.T)


class CompletedObservations:
    def __init__(self, root, config, calibration):
        self.root = Path(root).resolve()
        manifest_file = self.root/'manifest.json'
        self.manifest = json.loads(manifest_file.read_text())
        completed = json.loads((self.root/'completed.json').read_text())
        if completed.get('status') != 'complete' or not self.manifest.get('full_bag'):
            raise ValueError('Observation source must be a completed full-BAG run')
        identity = json.loads((self.root/'run_identity.json').read_text())
        old = dict(self.manifest['config']['semantic'])
        old.setdefault('input_tone', config['semantic']['input_tone'])
        # Historical sources predate input tone and therefore used unmodified images.
        if ('input_tone' not in self.manifest['config']['semantic']
                and config['semantic']['input_tone']['enabled']):
            raise ValueError('Old observations were generated without SAM3 input tone')
        if old != config['semantic'] or identity['calibration'] != calibration:
            raise ValueError('Completed observation calibration or semantic settings differ')
        if any(self.manifest['config'][key] != config[key] for key in ('camera_topic', 'camera_frame', 'robot_frame')):
            raise ValueError('Completed observation camera topics or frames differ')
        self.identity = {'directory': str(self.root),
                         'manifest_sha256': hashlib.sha256(manifest_file.read_bytes()).hexdigest(),
                         'run_identity_sha256': identity['sha256']}

    def stage(self, cache, valid, input_digest, bev, time_type):
        if self.manifest['input_stream_sha256'] != input_digest:
            raise ValueError('Completed observation BAG stream differs')
        rows = self.manifest['frames']
        if len(rows) != len(valid):
            raise ValueError('Completed observation frame coverage differs')
        for record, source in zip(valid, rows):
            for key in ('frame', 'stamp_ns', 'sha256', 'bag_ns', 'occurrence'):
                if record[key] != source[key]:
                    raise ValueError(f'Completed observation {key} mismatch at frame {record["frame"]}')
            position, rpy, origin = bev.resolve_pose(time_type(nanoseconds=record['stamp_ns']))
            mount = source['camera_mount']
            if (not np.allclose(position, mount['position'], rtol=0, atol=1e-9)
                    or not np.allclose(rpy, mount['rpy_degrees'], rtol=0, atol=1e-8)):
                raise ValueError(f'Completed observation camera pose mismatch at frame {record["frame"]}')
            record['camera_mount'] = dict(position=position, rpy_degrees=rpy, source=origin)
            name = str(record['frame'])
            packet = self.root/'observations'/f'{name}.npz'
            with np.load(packet, allow_pickle=False) as data:
                if str(data['sha256']) != record['sha256'] or not {
                    'raw', 'scores', 'extra', 'extra_scores',
                    'forward_normal', 'forward_normal_scores', 'forward_shadow', 'forward_shadow_scores',
                }.issubset(data.files):
                    raise ValueError(f'Completed inference cache mismatch at frame {name}')
            for suffix in ('.npz', '.json'):
                source_file = packet.with_suffix(suffix)
                target = cache/source_file.name
                temporary = target.with_suffix('.tmp')
                shutil.copyfile(source_file, temporary)
                temporary.replace(target)
