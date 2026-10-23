import numpy as np
import pytest

from theta_texture_tone import lift_texture, tone_lut, tone_report


@pytest.mark.parametrize('gain', [1., 2., 3., 4., 8.])
def test_monotone_curve_preserves_black_and_highlights(gain):
    lut = tone_lut(gain, 180)
    assert lut[0] == 0
    assert (np.diff(lut.astype(int)) >= 0).all()
    assert np.array_equal(lut[180:], np.arange(180, 256))
    assert (lut >= np.arange(256)).all()


def test_lift_preserves_color_ratios_alpha_and_hidden_pixels():
    source = np.array([[[20, 40, 80, 255], [2, 5, 8, 0], [100, 200, 250, 128], [0, 0, 0, 255]]], np.uint8)
    original = source.copy(); cfg = dict(enabled=True, gain=3., pivot=180.)
    result = lift_texture(source, cfg)
    np.testing.assert_array_equal(source, original)
    np.testing.assert_array_equal(result[..., 3], source[..., 3])
    np.testing.assert_array_equal(result[0, 1:], source[0, 1:])
    scale = result[0, 0, 2]/80
    np.testing.assert_allclose(result[0, 0, :3], source[0, 0, :3]*scale, atol=.5)
    report = tone_report(source, result, cfg)
    assert report['alpha_unchanged'] and report['highlights_unchanged']
    assert report['unobserved_rgb_unchanged'] and report['new_clipped_cells'] == 0


def test_disabled_is_exact_identity_and_bad_settings_rejected():
    texture = np.arange(4*5*4, dtype=np.uint8).reshape(4, 5, 4)
    np.testing.assert_array_equal(lift_texture(texture, dict(enabled=False, gain=3., pivot=180.)), texture)
    with pytest.raises(ValueError):
        lift_texture(texture, dict(enabled=True, gain=float('nan'), pivot=180.))


@pytest.mark.parametrize('prefix', ['road_temporal', 'map'])
def test_reexport_uses_raw_rgb_and_preserves_semantic_evidence(tmp_path, monkeypatch, prefix):
    import cv2
    import json
    import yaml
    import brighten_theta_road_map as module
    source = tmp_path/'source'; source.mkdir()
    texture = np.full((2, 3, 4), [20, 30, 40, 255], np.uint8)
    labels = np.full((2, 3), 4, np.uint8)
    cv2.imwrite(str(source/f'{prefix}.texture.png'), texture)
    cv2.imwrite(str(source/f'{prefix}.colored.pgm'), labels)
    from sirius_navigation.indexed_road_map import road_palette
    (source/f'{prefix}.colored.json').write_text(json.dumps(dict(
        resolution=.05, width=3, height=2, origin=[0., 0.],
        palette=road_palette().tolist(), labels={'4': dict(name='tactile paving', global_cost=0, local_cost=50)})))
    np.savez_compressed(source/'map_evidence.npz', votes=labels)
    (source/'manifest.json').write_text(json.dumps(dict(frame_count=1, grid=dict(resolution=.05))))
    (source/'config_used.yaml').write_text(yaml.safe_dump(dict(texture={})))
    monkeypatch.setattr(module, 'render_review', lambda *args: None)
    cfg = dict(enabled=True, gain=3., pivot=180.)
    first, second = tmp_path/'first', tmp_path/'second'
    module.brighten(source, first, cfg)
    module.brighten(first, second, cfg)
    for name in (f'{prefix}.texture.png', f'{prefix}.raw.texture.png',
                 f'{prefix}.colored.pgm', f'{prefix}.colored.json', 'map_evidence.npz'):
        assert (first/name).read_bytes() == (second/name).read_bytes()
    assert (source/f'{prefix}.texture.png').read_bytes() == (first/f'{prefix}.raw.texture.png').read_bytes()
    with pytest.raises(FileExistsError):
        module.brighten(source, first, cfg)
