"""Import legacy sequential SAM3 observations only after exact input checks."""
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


class SemanticBaselineCache:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.manifest = json.loads((self.root/'manifest.json').read_text())

    def load(self, record, views, cfg):
        from sirius_navigation.sam3_experiment_client import mono_transport
        index = record['frame']
        frame = self.manifest['frames'].get(str(index))
        if frame is None:
            return None
        if frame['sha256'] != record['sha256'] or frame['stamp_ns'] != record['stamp_ns']:
            raise ValueError(f'Baseline cache camera mismatch at frame {index}')
        center = min(self.manifest['centers'], key=lambda c: abs(c-index))
        folder = self.root/'yellow'/f'seg{center}'/f'frame{index}'
        ids, scores, metadata = [], [], []
        expected = {p.strip(): cfg['score_min'] for p in cfg['prompt'].split(',')}
        for image, yaw in zip(views, cfg['view_yaws']):
            stem = f'yaw{yaw}.'
            path = folder/(stem+'json')
            if not path.exists():
                return None
            meta = json.loads(path.read_text())
            setting = meta['settings']
            if (setting['prompt'] != cfg['prompt'] or setting['resolution'] != cfg['resolution']
                    or setting['class_thresholds'] != expected or setting['confidence_threshold'] != 1.):
                raise ValueError(f'Baseline cache model settings mismatch at frame {index}')
            transport, _ = mono_transport(image)
            ok, jpeg = cv2.imencode('.jpg', transport, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if not ok or hashlib.sha256(jpeg.tobytes()).hexdigest() != meta['uploaded_jpeg_sha256']:
                raise ValueError(f'Baseline cache projection/JPEG mismatch at frame {index}, yaw {yaw}')
            cls = cv2.imread(str(folder/(stem+'classes.png')), cv2.IMREAD_GRAYSCALE)
            score = cv2.imread(str(folder/(stem+'scores.png')), cv2.IMREAD_GRAYSCALE)
            if cls is None or score is None or cls.shape != image.shape[:2] or score.shape != cls.shape:
                raise ValueError(f'Baseline cache mask shape mismatch at frame {index}')
            if (cls > 6).any():
                raise ValueError('Baseline cache contains unsupported class IDs')
            ids.append(cls); scores.append(score)
            metadata.append(dict(kind='base_cached', yaw=yaw, source=str(path),
                source_metadata_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                classes_sha256=hashlib.sha256((folder/(stem+'classes.png')).read_bytes()).hexdigest(),
                scores_sha256=hashlib.sha256((folder/(stem+'scores.png')).read_bytes()).hexdigest(), **meta))
        return dict(classes=ids, scores=scores, metadata=metadata)
