"""Deterministic exposure lift for detector input; never changes map texture.

Apply one bounded value curve with a common RGB scale. This preserves image
geometry, black padding and highlights, and does not manufacture semantic IDs.
"""
import numpy as np
from theta_texture_tone import tone_lut, validate_tone


def inference_image(image, config):
    validate_tone(config)
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected a uint8 BGR detector image')
    result = image.copy()
    if not config['enabled'] or config['gain'] == 1:
        return result
    value = image.max(axis=2)
    mask = (value > 0) & (value < config['pivot'])
    before = value[mask].astype(np.float32)
    after = tone_lut(config['gain'], config['pivot'])[value[mask]]
    result[mask] = np.rint(image[mask].astype(np.float32) * (after / before)[:, None]).clip(0, 255).astype(np.uint8)
    return result
