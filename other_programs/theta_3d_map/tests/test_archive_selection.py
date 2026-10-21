from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from map_selection import bag_candidates,map_candidates


def test_archived_bags_and_maps_remain_discoverable_with_link_cycles(tmp_path):
    ssd=tmp_path/'ssd';hdd=tmp_path/'hdd';ssd.mkdir();hdd.mkdir()
    bag=hdd/'recorded';bag.mkdir();(bag/'metadata.yaml').write_text('finished')
    output=bag/'offline';output.mkdir();colored=output/'map_3d_colored.ply';colored.write_text('ply')
    (output/'map_3d.ply').write_text('ply')
    (ssd/'recorded').symlink_to(bag,target_is_directory=True)
    (bag/'back_to_ssd').symlink_to(ssd,target_is_directory=True)
    assert bag_candidates(ssd)==[ssd/'recorded']
    assert map_candidates(ssd)==[ssd/'recorded/offline/map_3d_colored.ply']
    assert len(map_candidates([ssd,hdd]))==1
