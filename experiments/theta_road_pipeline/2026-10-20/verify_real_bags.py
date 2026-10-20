"""Small integration checks; not an independent semantic accuracy evaluation."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import requests


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag1', type=Path); p.add_argument('bag2', type=Path)
    p.add_argument('--calibration', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--reference', type=Path)
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[3]
    common = [sys.executable, str(repo/'bash/startup_bash/generate_theta_road_map.py')]
    cases, report = [], {}
    for number, (bag, frames) in enumerate(((a.bag1, 6), (a.bag2, 4)), 1):
        output = a.output/f'bag{number}'
        args = common + [str(bag), str(output), '--calibration', str(a.calibration), '--max-frames', str(frames)]
        with (a.output/f'bag{number}.log').open('w') as log:
            subprocess.run(args, cwd=repo, stdout=log, stderr=subprocess.STDOUT, check=True)
        manifest = json.loads((output/'manifest.json').read_text())
        assert manifest['frame_count'] == frames and not manifest['skipped']
        restored = json.loads((output/'server_restoration.json').read_text())
        assert restored['settings_restored'] and restored['registry_unchanged'] and not restored['errors']
        cases.append((args, output))
        report[f'bag{number}'] = dict(mapped_frames=frames, skipped=0, server_restored=True,
            input_stream_sha256=manifest['input_stream_sha256'])
    if a.reference:
        prior = {f['frame']: f for f in json.loads((a.reference/'manifest.json').read_text())['frames']}
        actual = json.loads((cases[0][1]/'manifest.json').read_text())['frames']
        for frame in actual:
            old = prior[frame['frame']]
            assert all(frame[key] == old[key] for key in ('ground_points', 'xyz', 'quaternion'))
        report['native_point_counts_and_poses_equal_reference'] = True
    args, output = cases[0]
    names = ['road_temporal.colored.pgm', 'road_temporal.texture.png', 'baseline.class_ids.png', 'map_evidence.npz']
    digest = lambda: {name: hashlib.sha256((output/name).read_bytes()).hexdigest() for name in names}
    version = lambda: requests.get('http://localhost:8080/debug_state', timeout=3).json()['network_frame_version']
    before, network_before = digest(), version()
    with (a.output/'resume.log').open('w') as log:
        subprocess.run(args+['--resume'], cwd=repo, stdout=log, stderr=subprocess.STDOUT, check=True)
    assert before == digest() and network_before == version()
    change = a.output/'changed.yaml'; change.write_text('texture:\n  rear_weight: 0.6\n')
    failed = subprocess.run(args+['--resume', '--config', str(change)], cwd=repo, capture_output=True, text=True)
    assert failed.returncode != 0 and 'Resume refused' in failed.stderr
    foreign = a.output/'foreign'; foreign.mkdir()
    failed = subprocess.run(common+[str(a.bag1), str(foreign), '--calibration', str(a.calibration), '--resume'],
                            cwd=repo, capture_output=True, text=True)
    assert failed.returncode != 0 and 'no pipeline run identity' in failed.stderr
    report.update(resume_map_bytes_identical=True, resume_no_sam3_upload=True,
        changed_config_refused=True, unrelated_directory_refused=True, first_map_sha256=before,
        limitation='First 6/4 frames only; functionality checks, not full-route semantic accuracy.')
    (a.output/'verification.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
