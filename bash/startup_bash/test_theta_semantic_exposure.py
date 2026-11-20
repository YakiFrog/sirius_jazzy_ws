import numpy as np
import pytest
from theta_semantic_exposure import inference_image

CONFIG = dict(enabled=True, gain=3., pivot=180.)


def test_dark_detail_is_lifted_without_changing_source_geometry_or_highlights():
    image = np.array([[[0,0,0],[8,20,32],[10,45,70],[200,220,250]]],np.uint8)
    before = image.copy()
    adjusted = inference_image(image, CONFIG)
    np.testing.assert_array_equal(image,before)
    assert adjusted.shape==image.shape and adjusted.dtype==image.dtype
    np.testing.assert_array_equal(adjusted[:,[0,3]],image[:,[0,3]])
    assert (adjusted[:,1:3].max(axis=2)>image[:,1:3].max(axis=2)).all()
    assert np.all(np.abs(adjusted[0,2]/adjusted[0,2].max()-image[0,2]/image[0,2].max())<.015)


def test_curve_is_monotone_and_cannot_create_new_clipping():
    ramp=np.repeat(np.arange(256,dtype=np.uint8)[None,:,None],3,axis=2)
    result=inference_image(ramp,CONFIG)
    assert (np.diff(result[0,:,0].astype(int))>=0).all()
    assert not ((result==255)&(ramp<255)).any()


def test_explicit_disable_returns_independent_unmodified_image():
    image=np.full((6,8,3),35,np.uint8)
    result=inference_image(image,dict(CONFIG,enabled=False))
    np.testing.assert_array_equal(result,image)
    assert not np.shares_memory(image,result)


@pytest.mark.parametrize('bad',[dict(CONFIG,gain=float('nan')),dict(CONFIG,gain=0),dict(CONFIG,pivot=300)])
def test_invalid_exposure_parameters_fail(bad):
    with pytest.raises(ValueError):inference_image(np.zeros((4,4,3),np.uint8),bad)
