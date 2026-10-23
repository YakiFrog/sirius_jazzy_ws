import json
from pathlib import Path
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation'))
from sirius_navigation.indexed_road_map import (
    RGB_STEPS, class_labels, encode_road_map, indexed_rgb, road_palette,
)
from indexed_road_export import export_existing


def metadata():
    names = ['unknown', 'wall', 'floor', 'grass', 'tactile paving', 'roadway', 'sidewalk']
    return dict(resolution=.05, origin=[1., 2.], width=4, height=2,
                palette=road_palette().tolist(),
                labels={str(i):dict(name=n, global_cost=100 if i==3 else 0,
                                    local_cost=50 if i==4 else 0) for i,n in enumerate(names)})


def test_all_125_rgb_colors_have_separate_nonsemantic_indices():
    rgb=np.array([[r,g,b] for r in RGB_STEPS for g in RGB_STEPS for b in RGB_STEPS],np.uint8).reshape(5,25,3)
    texture=np.dstack([rgb[:,:,::-1],np.full((5,25),255,np.uint8)])
    result,palette,report=encode_road_map(np.zeros((5,25),np.uint8),texture)
    assert result.min()==7 and result.max()==131
    assert len(np.unique(result))==125
    np.testing.assert_array_equal(palette[result],rgb)
    assert report['texture_index_cells']==125


def test_preserves_semantics_structure_and_unobserved_cells_without_spatial_fill():
    labels=np.array([[0,1,2,3,4,5,6,0]],np.uint8)
    texture=np.full((1,8,4),255,np.uint8);texture[0,-1,3]=0
    original=labels.copy()
    grid,palette,_=encode_road_map(labels,texture)
    np.testing.assert_array_equal(labels,original)
    np.testing.assert_array_equal(grid[0,1:],labels[0,1:])
    assert grid[0,0]==131  # white RGB is distinct from reserved floor ID 2
    np.testing.assert_array_equal(class_labels(grid),labels)
    rgb,mask=indexed_rgb(grid,metadata())
    np.testing.assert_array_equal(rgb[0,0],[255,255,255])
    assert mask[0,0] and not mask[0,-1]
    _,semantic_mask=indexed_rgb(grid,metadata(),mode='semantic')
    assert semantic_mask.tolist()==[[False,False,False,True,True,True,True,False]]


def test_quantization_errors_are_bounded_and_channels_do_not_swap():
    rng=np.random.default_rng(42)
    rgb=rng.integers(0,256,(30,40,3),dtype=np.uint8)
    texture=np.dstack([rgb[:,:,::-1],np.full((30,40),255,np.uint8)])
    grid,palette,_=encode_road_map(np.zeros((30,40),np.uint8),texture)
    assert np.abs(palette[grid].astype(int)-rgb.astype(int)).max()<=32
    with pytest.raises(ValueError):
        indexed_rgb(np.array([[255]],np.uint8),metadata())
    with pytest.raises(ValueError,match='PNG loading'):
        indexed_rgb(grid,metadata(),mode='texture')


def test_export_works_without_rgb_sidecar_at_consumption_and_keeps_originals(tmp_path):
    source=tmp_path/'source';source.mkdir()
    labels=np.array([[0,3,4,0],[0,5,6,0]],np.uint8)
    texture=np.full((2,4,4),[20,100,180,255],np.uint8);texture[1,-1,3]=0
    cv2.imwrite(str(source/'map.colored.pgm'),labels)
    cv2.imwrite(str(source/'map.texture.png'),texture)
    (source/'map.colored.json').write_text(json.dumps(metadata()))
    (source/'manifest.json').write_text(json.dumps(dict(frame_count=1,full_bag=True,bag='example')))
    originals={p.name:p.read_bytes() for p in source.iterdir()}
    output=tmp_path/'output';report=export_existing(source,output)
    assert {p.name:p.read_bytes() for p in source.iterdir()}==originals
    assert not (output/'map.texture.png').exists()
    loaded=cv2.imread(str(output/'map.colored.pgm'),0)
    meta=json.loads((output/'map.colored.json').read_text())
    rgb,mask=indexed_rgb(loaded,meta)
    assert loaded[0,0]>=7 and loaded[1,-1]==0
    assert meta['labels']==metadata()['labels']
    assert report['semantic_ids_unchanged'] and report['semantic_costs_unchanged']
    preview=cv2.imread(str(output/'indexed.png'),-1)
    np.testing.assert_array_equal(preview[mask,:3][:,::-1],rgb[mask])
    with pytest.raises(FileExistsError):
        export_existing(source,output)


def test_real_loader_reads_only_pgm_json_and_includes_rgb_without_changing_costs(tmp_path,monkeypatch):
    pytest.importorskip('rclpy')
    from builtin_interfaces.msg import Time
    from sirius_navigation.sam3_colored_map_loader import SAM3ColoredMapLoader
    grid=np.array([[0,3,7,131],[4,5,6,8]],np.uint8)
    cv2.imwrite(str(tmp_path/'map.colored.pgm'),grid)
    (tmp_path/'map.colored.json').write_text(json.dumps(metadata()))
    reads=[];original=cv2.imread
    def read(path,*args):
        reads.append(path)
        assert path.endswith('.pgm'), 'Runtime must not read a PNG sidecar'
        return original(path,*args)
    monkeypatch.setattr(cv2,'imread',read)
    captured={}
    def cloud(rows,cols,colors,*args):
        captured.update(rows=rows,cols=cols,colors=colors)
        return captured
    fake=SimpleNamespace(visualization_mode='indexed',semantic_include_structure=False,
        z_offset=-.2,map_frame='map',cloud_msgs={},cloud_topics={'indexed':'/example'},
        soft_semantic_inflation_radius=0.,soft_semantic_inflation_cost=0,
        get_logger=lambda:SimpleNamespace(info=lambda *_:None,warn=lambda *_:None,error=lambda msg:pytest.fail(msg)),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())),
        _create_cloud=cloud)
    SAM3ColoredMapLoader.load_map(fake,str(tmp_path/'map'))
    assert len(reads)==1
    rows,cols=np.where(grid[::-1]>0)
    np.testing.assert_array_equal(captured['colors'],road_palette()[grid[::-1][rows,cols]])
    assert len(rows)==7  # includes RGB IDs 7, 8 and 131, not only class IDs
    global_cost=np.array(fake.grid_msg.data).reshape(2,4)[::-1]
    local_cost=np.array(fake.soft_grid_msg.data).reshape(2,4)[::-1]
    assert global_cost[0,1]==100 and local_cost[1,0]==50
    assert np.all(global_cost[grid>=7]==0) and np.all(local_cost[grid>=7]==0)
