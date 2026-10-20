"""Bounded, monotone shadow lift for map readability, preserving RGB ratios.

The curve operates on encoded RGB value, not a physical illumination estimate.
It contains no shadow detector, spatial filter, semantic rule or coordinate warp.
Dark materials are also lifted; original fused RGB must be retained separately.
"""
import numpy as np


def validate_tone(config):
    if not isinstance(config['enabled'], bool):
        raise ValueError('texture.tone.enabled must be boolean')
    gain, pivot = config['gain'], config['pivot']
    if not np.isfinite(gain) or not 1 <= gain <= 8:
        raise ValueError('texture.tone.gain must lie in [1, 8]')
    if not np.isfinite(pivot) or not 1 <= pivot <= 255:
        raise ValueError('texture.tone.pivot must lie in [1, 255]')


def tone_lut(gain=3., pivot=180.):
    """f(v)=gain*v/(1+(gain-1)*v/pivot) below pivot; identity above."""
    v = np.arange(256, dtype=np.float64)
    lifted = v*gain/(1+(gain-1)*v/pivot)
    return np.rint(np.where(v < pivot, lifted, v)).clip(0, 255).astype(np.uint8)


def lift_texture(texture, config):
    validate_tone(config)
    if texture.dtype != np.uint8 or texture.ndim != 3 or texture.shape[2] != 4:
        raise ValueError('Expected uint8 BGRA texture')
    result = texture.copy()
    if not config['enabled'] or config['gain'] == 1:
        return result
    value = texture[..., :3].max(axis=2)
    mask = (texture[..., 3] > 0) & (value > 0) & (value < config['pivot'])
    y, x = np.nonzero(mask)
    before = value[y, x].astype(np.float32)
    after = tone_lut(config['gain'], config['pivot'])[value[y, x]].astype(np.float32)
    scale = after/before
    # One factor for all channels preserves hue/saturation up to 8-bit rounding.
    result[y, x, :3] = np.rint(texture[y, x, :3].astype(np.float32)*scale[:, None]).clip(0, 255).astype(np.uint8)
    return result


def tone_report(before, after, config):
    valid = before[..., 3] > 0
    v0, v1 = before[..., :3].max(axis=2), after[..., :3].max(axis=2)
    dark = valid & (v0 > 0) & (v0 < 80)
    highlight = valid & (v0 >= config['pivot'])
    return dict(parameters=dict(config), observed_cells=int(valid.sum()),
        changed_cells=int((valid & np.any(before != after, axis=2)).sum()),
        original_dark_cells=int(dark.sum()),
        dark_mean_value_before=float(v0[dark].mean()) if dark.any() else None,
        dark_mean_value_after=float(v1[dark].mean()) if dark.any() else None,
        alpha_unchanged=bool(np.array_equal(before[..., 3], after[..., 3])),
        unobserved_rgb_unchanged=bool(np.array_equal(before[~valid], after[~valid])),
        highlights_unchanged=bool(np.array_equal(before[highlight], after[highlight])),
        new_clipped_cells=int((valid & (v1 == 255) & (v0 < 255)).sum()),
        method='monotone bounded value curve; identical scale per RGB channel; no spatial filtering')
