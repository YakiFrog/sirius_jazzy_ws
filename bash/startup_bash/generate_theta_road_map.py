#!/usr/bin/env python3
"""Generate a reproducible THETA road map directly from any compatible ROSBAG.

No reference map, selected frame intervals or route coordinates are required.
The bag is read directly: inference speed never drops camera observations.
Recorded TF and scan visibility determine geometry; no image-based map warp.
"""
import argparse
from contextlib import nullcontext
import hashlib
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import yaml

from theta_road_pipeline import (DEFAULT_CONFIG, load_config, fingerprint,
    enhanced_shadow, dark_mask, add_shadow, refine_semantics)
from theta_texture_quality import TextureFusion, aggregate_frame
from theta_texture_tone import lift_texture, tone_report


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False))
    temporary.replace(path)


def save_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f'Cannot write {path}')


def stamp_ns(msg):
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


class Capture:
    msg = None
    def publish(self, msg):
        self.msg = msg


def open_reader(bag, topics):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def index_bag(bag, cfg, max_frames):
    from rclpy.serialization import deserialize_message
    from rclpy.duration import Duration
    from sensor_msgs.msg import CompressedImage, LaserScan
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer
    buffer = Buffer(cache_time=Duration(seconds=10**8))
    topics = ['/tf', '/tf_static', cfg['scan_topic'], cfg['camera_topic']]
    reader = open_reader(bag, topics)
    records, scans = [], []
    digest = hashlib.sha256()
    index, last_ns, ordinal = 0, None, 0
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        digest.update(topic.encode()); digest.update(int(bag_ns).to_bytes(8, 'little', signed=True)); digest.update(data)
        if topic in ('/tf', '/tf_static'):
            for transform in deserialize_message(data, TFMessage).transforms:
                setter = buffer.set_transform_static if topic == '/tf_static' else buffer.set_transform
                setter(transform, 'recorded_bag')
        elif topic == cfg['scan_topic']:
            scan = deserialize_message(data, LaserScan)
            scans.append((stamp_ns(scan), scan))
        else:
            msg = deserialize_message(data, CompressedImage)
            ordinal = ordinal + 1 if bag_ns == last_ns else 0
            last_ns = bag_ns
            if not max_frames or index < max_frames:
                records.append(dict(frame=index, bag_ns=bag_ns, occurrence=ordinal,
                    stamp_ns=stamp_ns(msg), sha256=hashlib.sha256(bytes(msg.data)).hexdigest()))
            index += 1
    if not records:
        raise ValueError('No compressed camera frames found on ' + cfg['camera_topic'])
    if cfg['visibility']['strict'] and not scans:
        raise ValueError('Strict visibility requires recorded scans on ' + cfg['scan_topic'])
    scans.sort(key=lambda item: item[0])
    return records, scans, buffer, index, digest.hexdigest()


def read_camera(reader, record):
    """Exact indexed bag message, including duplicate bag timestamps."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage
    # MCAP can treat seek(same_timestamp) as a no-op after read_next().
    reader.seek(record['bag_ns'] - 1)
    reader.seek(record['bag_ns'])
    for _ in range(record['occurrence'] + 1):
        if not reader.has_next():
            raise RuntimeError('Camera message missing during indexed reread')
        _, data, timestamp = reader.read_next()
        if timestamp != record['bag_ns']:
            raise RuntimeError('Camera bag timestamp changed during generation')
    msg = deserialize_message(data, CompressedImage)
    if stamp_ns(msg) != record['stamp_ns'] or hashlib.sha256(bytes(msg.data)).hexdigest() != record['sha256']:
        raise RuntimeError('Camera bytes/timestamp changed during generation')
    return msg


class SemanticProcessor:
    def __init__(self, cfg, calibration, session):
        from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency
        self.cfg, self.calibration, self.session = cfg, calibration, session
        self.remaps = {}
        s, shadow = cfg['semantic'], cfg['semantic']['shadow']
        def trackers(fraction):
            return {yaw: CausalTactileConsistency(max_gap=s['max_gap_sec'], score_min=s['score_min'],
                        yellow_fraction=fraction, min_saturation=shadow['min_saturation']) for yaw in s['view_yaws']}
        self.normal = trackers(s['yellow_fraction'])
        self.shadow = trackers(shadow['yellow_fraction'])
        self.calls = 0

    def views(self, raw, calibration):
        from sirius_navigation.theta_perspective import build_perspective_remap
        s = self.cfg['semantic']
        key = fingerprint(dict(calibration=calibration, shape=raw.shape))
        if key not in self.remaps:
            self.remaps = {key: [build_perspective_remap(calibration, yaw, raw.shape[:2],
                tuple(s['view_size']), s['hfov_deg']) for yaw in s['view_yaws']]}
        return [cv2.remap(raw, *maps, cv2.INTER_LINEAR) for maps in self.remaps[key]]

    def infer(self, views, baseline=None):
        from sirius_navigation.theta_semantic_temporal import yellow_tactile_seeds
        s, shadow = self.cfg['semantic'], self.cfg['semantic']['shadow']
        base, scores, extra, escores = [], [], [], []
        self.last_inference_metadata = []
        # Reconfigure once per group, rather than once per view.
        if baseline is None:
            self.session.configure(s['prompt'], {p.strip(): s['score_min'] for p in s['prompt'].split(',')},
                                   threshold=1., resolution=s['resolution'])
        for i, image in enumerate(views):
            if baseline is not None:
                ids, score = baseline['classes'][i].copy(), baseline['scores'][i].copy()
                self.last_inference_metadata.append(baseline['metadata'][i])
            else:
                ids, score, meta = self.session.infer(image)
                self.calls += 1
                self.last_inference_metadata.append(dict(kind='base', yaw=s['view_yaws'][i], **meta))
            if (baseline is None and 'yellow tactile paving' in [p.strip() for p in s['prompt'].split(',')]
                    and 'yellow tactile paving' not in self.session.registry_before['ids']):
                ids = ids.copy()
                ids[(ids == 0) & (score > 0)] = 4
            base.append(ids); scores.append(score)
            extra.append(np.zeros(ids.shape, np.uint8)); escores.append(np.zeros(ids.shape, np.uint8))
        candidates = [i for i, image in enumerate(views) if
            (image.max(axis=2) <= shadow['max_original_value']).sum() >= shadow['min_component_area']]
        if shadow['enabled'] and candidates:
            prompt = shadow['prompt']
            self.session.configure(prompt, {prompt: s['score_min']}, threshold=1., resolution=s['resolution'])
            registered_id = self.session.registry_before['ids'].get(prompt)
            for i in candidates:
                image = enhanced_shadow(views[i], shadow)
                ids, score, meta = self.session.infer(image); self.calls += 1
                self.last_inference_metadata.append(dict(kind='shadow', yaw=s['view_yaws'][i], **meta))
                # The target-only prompt can be absent from the server registry.
                # Only its positive-score output may be mapped to reserved ID4.
                tactile = (ids == registered_id) if registered_id is not None else ((ids == 0) & (score > 0))
                ids = np.where(tactile, 4, 0).astype(np.uint8)
                mask = yellow_tactile_seeds(image, ids, score, s['score_min'],
                    yellow_fraction=shadow['yellow_fraction'], min_saturation=shadow['min_saturation'])
                mask &= dark_mask(views[i], mask, shadow) & ~np.isin(base[i], (1, 2, 3))
                extra[i] = np.where(mask, 4, 0).astype(np.uint8)
                escores[i] = np.where(mask, score, 0).astype(np.uint8)
        return dict(raw=np.stack(base), scores=np.stack(scores),
                    extra=np.stack(extra), extra_scores=np.stack(escores))

    def track(self, views, packet, stamp):
        s, shadow = self.cfg['semantic'], self.cfg['semantic']['shadow']
        result, confidence, aux, aux_conf = [], [], [], []
        for i, yaw in enumerate(s['view_yaws']):
            ids, score, _ = self.normal[yaw].apply(views[i], packet['raw'][i], packet['scores'][i], stamp)
            extra, es = packet['extra'][i].copy(), packet['extra_scores'][i].copy()
            anchor = ids == 4
            extra[anchor], es[anchor] = 4, score[anchor]
            protected = np.isin(ids, (1, 2, 3))
            extra[protected] = ids[protected]
            light = enhanced_shadow(views[i], shadow) if shadow['enabled'] else views[i]
            e, esc, _ = self.shadow[yaw].apply(light, extra, es, stamp)
            result.append(ids); confidence.append(score); aux.append(e); aux_conf.append(esc)
        return dict(normal=np.stack(result), normal_scores=np.stack(confidence),
                    shadow=np.stack(aux), shadow_scores=np.stack(aux_conf))

    def merge(self, views, packet, forward, backward):
        from sirius_navigation.theta_semantic_temporal import merge_tactile_tracks
        classes, scores = [], []
        for i in range(len(views)):
            ids, score = merge_tactile_tracks(packet['raw'][i], forward['normal'][i], forward['normal_scores'][i],
                backward['normal'][i], backward['normal_scores'][i])
            if self.cfg['semantic']['shadow']['enabled']:
                extra, es = merge_tactile_tracks(ids, forward['shadow'][i], forward['shadow_scores'][i],
                    backward['shadow'][i], backward['shadow_scores'][i])
                ids, score = add_shadow(ids, score, extra, es, views[i], self.cfg['semantic']['shadow'])
            classes.append(ids); scores.append(score)
        return dict(classes=np.stack(classes), confidence=np.stack(scores))


def render_review(output, labels, texture, resolution):
    from render_theta_semantic_map import frame, COLORS
    from html import escape
    rgb = texture[..., :3].copy(); rgb[texture[..., 3] == 0] = 248
    semantic = np.full_like(rgb, 248); overlay = rgb.copy()
    for cid, color in COLORS.items():
        mask = labels == cid
        semantic[mask] = color[::-1]
        overlay[mask] = np.rint(rgb[mask]*.4 + np.asarray(color[::-1])*.6).astype(np.uint8)
    html = '<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>THETA路面地図</title><style>body{font:16px system-ui;margin:12px;background:#13212d;color:white}img{width:100%;height:auto;max-width:1200px}a{color:#9de1ff}</style><h1>THETA路面地図</h1><p>設定と処理記録は manifest.json / config_used.yaml に保存しています。</p>'
    for name, title, image in [('texture', '路面テクスチャ', rgb), ('overlay', '重ね合わせ', overlay), ('semantic', 'セマンティックのみ', semantic)]:
        frame(image, title, '同じ座標・縮尺／未観測へは拡張しない', resolution, width=1200).save(output/(name+'.jpg'), quality=93)
        html += f'<h2>{escape(title)}</h2><a href="{name}.jpg"><img src="{name}.jpg" alt="{escape(title)}"></a>'
    (output/'images.html').write_text(html+'</html>')


def run(args):
    import rclpy
    from rclpy.time import Time
    from sirius_navigation.theta_bev_node import ThetaBevNode
    from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
    from sirius_navigation.theta_indexed_map_node import quaternion_to_matrix
    from sirius_navigation.theta_perspective import build_bev_to_view_maps
    from sirius_navigation.theta_road_weights import directional_weights
    from sirius_navigation.sam3_experiment_client import Sam3ExperimentClient
    from compare_theta_sam3_projection import project_labels
    cfg = load_config(args.config)
    if args.no_semantic:
        cfg['semantic']['enabled'] = False
    output, bag = args.output.resolve(), args.bag.resolve()
    if output == bag or bag in output.parents:
        raise ValueError('Output must be outside the original ROSBAG directory')
    if output.exists():
        if not args.resume:
            raise ValueError('Output already exists: use a new directory or --resume')
        if not (output/'run_identity.json').is_file():
            raise ValueError('Resume refused: this directory has no pipeline run identity')
    output.mkdir(parents=True, exist_ok=True)
    calibration = yaml.safe_load(args.calibration.read_text())
    calibration['bev_size'] = cfg['projection']['bev_size']
    if calibration['bev_extent_m']/2 < cfg['projection']['max_radius']:
        raise ValueError('Projection extent must cover max_radius')
    # Never publish to the live robot graph; nodes are used synchronously below.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    calfile = output/'calibration_used.yaml'
    nav_dir = Path(__file__).resolve().parents[2]/'src/sirius/sirius_navigation/sirius_navigation'
    source_files = [Path(__file__), Path(__file__).with_name('theta_road_pipeline.py'),
        Path(__file__).with_name('theta_texture_quality.py'),
        Path(__file__).with_name('theta_texture_tone.py'),
        Path(__file__).with_name('theta_road_cache.py'),
        Path(__file__).with_name('refine_theta_road_regions.py'),
        Path(__file__).with_name('experiment_theta_tactile_context.py'),
        Path(__file__).with_name('experiment_theta_grass_extent.py'),
        Path(__file__).with_name('render_theta_semantic_map.py'),
        Path(__file__).with_name('compare_theta_sam3_projection.py')]
    source_files += [nav_dir/name for name in ('theta_bev_node.py', 'theta_bev_projection.py',
        'theta_ground_cloud_node.py', 'theta_perspective.py', 'theta_semantic_temporal.py',
        'theta_road_weights.py', 'theta_road_visibility.py', 'theta_indexed_map_node.py',
        'semantic_costs.py', 'sam3_experiment_client.py') if (nav_dir/name).exists()]
    identity = dict(config=cfg, calibration=calibration, bag=str(bag), max_frames=args.max_frames,
                    input_files={str(p.relative_to(bag)): dict(size=p.stat().st_size, mtime_ns=p.stat().st_mtime_ns)
                        for p in sorted(bag.iterdir()) if p.is_file()},
                    pipeline_version=1, server=args.server if cfg['semantic']['enabled'] else None,
                    libraries=dict(numpy=np.__version__, opencv=cv2.__version__),
                    source_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in source_files})
    baseline_cache = None
    if args.reuse_study:
        from theta_road_cache import SemanticBaselineCache
        baseline_cache = SemanticBaselineCache(args.reuse_study)
        identity['baseline_cache'] = dict(path=str(args.reuse_study.resolve()),
            manifest_sha256=hashlib.sha256((args.reuse_study/'manifest.json').read_bytes()).hexdigest())
    identity_hash = fingerprint(identity)
    if (output/'run_identity.json').exists():
        if json.loads((output/'run_identity.json').read_text())['sha256'] != identity_hash:
            raise ValueError('Resume refused: inputs, calibration, code or settings changed')
    write_json(output/'run_identity.json', dict(sha256=identity_hash, **identity))
    calfile.write_text(yaml.safe_dump(calibration)); (output/'config_used.yaml').write_text(yaml.safe_dump(cfg))
    cache = output/'observations'; cache.mkdir(exist_ok=True)
    overrides = {'calibration': str(calfile), 'publish_raw': False, 'output_frame': cfg['robot_frame'],
        'robot_frame': cfg['robot_frame'], 'camera_frame': cfg['camera_frame'],
        'stride': cfg['projection']['stride'], 'min_radius': cfg['projection']['min_radius'],
        'max_radius': cfg['projection']['max_radius'], 'quality_power': cfg['texture']['quality_power'],
        'lidar_strict_visibility': cfg['visibility']['strict'], 'lidar_max_age_sec': cfg['visibility']['max_age_sec'],
        'lidar_fixed_frame': cfg['visibility']['fixed_frame'],
        'lidar_margin_m': cfg['visibility']['margin_m'], 'lidar_obstacle_padding_m': cfg['visibility']['obstacle_padding_m']}
    rosargs = ['--ros-args']
    for key, value in overrides.items():
        rosargs += ['-p', key+':='+str(value).lower() if isinstance(value, bool) else key+':='+str(value)]
    rclpy.init(args=rosargs)
    started = time.time(); session = None; bev = ground = None
    try:
        records, scans, buffer, total_frames, input_digest = index_bag(bag, cfg, args.max_frames)
        digest_file = output/'input_stream_sha256.json'
        if digest_file.exists() and json.loads(digest_file.read_text())['sha256'] != input_digest:
            raise ValueError('Resume refused: camera/TF/scan message stream changed')
        write_json(digest_file, dict(sha256=input_digest, topics=[cfg['camera_topic'], cfg['scan_topic'], '/tf', '/tf_static']))
        # A previous completed marker must not describe a partial resume.
        (output/'completed.json').unlink(missing_ok=True)
        write_json(output/'camera_index.json', records)
        valid, skipped = [], []
        for record in records:
            try:
                if record['stamp_ns'] <= 0:
                    raise ValueError('Unset camera timestamp: latest-TF fallback is not permitted')
                tf = buffer.lookup_transform(cfg['map_frame'], cfg['robot_frame'], Time(nanoseconds=record['stamp_ns']))
                t, q = tf.transform.translation, tf.transform.rotation
                record.update(xyz=[t.x, t.y, t.z], quaternion=[q.x, q.y, q.z, q.w])
                valid.append(record)
            except Exception as error:
                skipped.append(dict(frame=record['frame'], reason=str(error)))
        if not valid:
            raise ValueError('No camera frames have a recorded map-to-robot transform')
        res = cfg['resolution']; margin = cfg['projection']['max_radius'] + res*2
        xy = np.asarray([r['xyz'][:2] for r in valid])
        origin = np.floor((xy.min(axis=0)-margin)/res)*res
        width, height = np.ceil((xy.max(axis=0)+margin-origin)/res).astype(int)
        size = int(width*height)
        if size > cfg['max_grid_cells']:
            raise ValueError(f'Grid has {size} cells, exceeding configured limit; check TF or resolution')
        fuser = TextureFusion(size, relative_quality=cfg['texture']['relative_quality'], decay=cfg['texture']['decay'])
        votes = np.zeros((height, width, 4), np.float32)
        coverage = np.zeros((height, width), np.uint32)
        bev, ground = ThetaBevNode(), ThetaGroundCloudNode()
        bev.buffer = buffer; bev.pub = Capture()
        ground.visibility_tf = buffer; ground.lidar_gate = cfg['visibility']['strict']
        ground.pub = Capture(); ground.semantic_pub = None
        reader = open_reader(bag, [cfg['camera_topic']])
        times = np.asarray([s[0] for s in scans], np.int64)
        semantic = cfg['semantic']['enabled']
        session = Sam3ExperimentClient(args.server) if semantic else None
        mapped, inference_rows = [], []
        reused_baseline_frames = 0
        # A completed forward cache needs no SAM3 connection during resume.
        need_server = semantic and any(not (cache/f'{r["frame"]}.npz').exists() for r in valid)
        with session if need_server else nullcontext():
            if need_server:
                registry = session.registry_before['ids']
                for name, cid in [('grass', 3), ('roadway', 5), ('sidewalk', 6)]:
                    if registry.get(name) != cid:
                        raise ValueError(f'SAM3 registry requires {name}=ID{cid}; configure the shared class registry first')
                unknown = set(p.strip() for p in cfg['semantic']['prompt'].split(',')) - set(registry)
                if unknown - {'yellow tactile paving'}:
                    raise ValueError('Unregistered semantic prompts: ' + str(sorted(unknown)))
                if registry.get('yellow tactile paving', 4) != 4:
                    raise ValueError('Yellow tactile paving must use reserved ID4')
                write_json(output/'server_before.json', session.before)
                write_json(output/'registry_used.json', session.registry_before)
            processor = SemanticProcessor(cfg, calibration, session) if semantic else None
            for n, record in enumerate(valid, 1):
                msg = read_camera(reader, record)
                bev.pub.msg = None; bev.receive(msg)
                if bev.pub.msg is None:
                    raise RuntimeError(f'BEV failed for camera frame {record["frame"]}')
                record['camera_mount'] = dict(position=bev.calibration['camera_position'],
                    rpy_degrees=bev.calibration['rpy_degrees'], source=bev.resolve_pose(msg.header.stamp)[2])
                if semantic:
                    raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                    views = processor.views(raw, bev.calibration)
                    path = cache/f'{record["frame"]}.npz'
                    if path.exists():
                        with np.load(path, allow_pickle=False) as stored:
                            if str(stored['sha256']) != record['sha256']:
                                raise ValueError('Cached inference camera hash mismatch')
                            packet = {k: stored[k] for k in ('raw', 'scores', 'extra', 'extra_scores')}
                    else:
                        cached_base = baseline_cache.load(record, views, cfg['semantic']) if baseline_cache else None
                        packet = processor.infer(views, baseline=cached_base)
                        reused_baseline_frames += int(cached_base is not None)
                        write_json(path.with_suffix('.json'), processor.last_inference_metadata)
                    forward = processor.track(views, packet, record['stamp_ns']/1e9)
                    # Atomic cache: interrupted inference cannot appear complete.
                    temporary = path.with_suffix('.tmp')
                    with temporary.open('wb') as stream:
                        np.savez_compressed(stream, sha256=record['sha256'], **packet,
                            **{'forward_'+k: v for k, v in forward.items()})
                    temporary.replace(path)
                    inference_rows.append(dict(frame=record['frame'], sha256=record['sha256']))
                if n % 25 == 0 or n == len(valid):
                    progress = dict(phase='forward', completed=n, frames=len(valid), elapsed_sec=time.time()-started)
                    write_json(output/'progress.json', progress); print(json.dumps(progress), flush=True)
        if session is not None and hasattr(session, 'restoration'):
            write_json(output/'server_restoration.json', session.restoration)
        reverse = SemanticProcessor(cfg, calibration, None) if semantic else None
        # Reverse tracking and native projection share a second bag pass.
        # RGB fusion is replayed in chronological order afterwards (EMA order matters).
        packets = output/'projection'; packets.mkdir(exist_ok=True)
        for n, record in enumerate(reversed(valid), 1):
            msg = read_camera(reader, record); bev.pub.msg = None; bev.receive(msg); bmsg = bev.pub.msg
            if bmsg is None:
                raise RuntimeError('BEV failed during reverse pass')
            image = np.frombuffer(bmsg.data, np.uint8).reshape(bmsg.height, bmsg.width, 3)
            label = None
            if semantic:
                raw = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
                views = reverse.views(raw, bev.calibration)
                with np.load(cache/f'{record["frame"]}.npz', allow_pickle=False) as stored:
                    packet = {k: stored[k] for k in ('raw', 'scores', 'extra', 'extra_scores')}
                    forward = {k: stored['forward_'+k] for k in ('normal', 'normal_scores', 'shadow', 'shadow_scores')}
                backward = reverse.track(views, packet, -record['stamp_ns']/1e9) if cfg['semantic']['bidirectional'] else forward
                merged = reverse.merge(views, packet, forward, backward)
                samples = []
                for i, yaw in enumerate(cfg['semantic']['view_yaws']):
                    u, v, ok = build_bev_to_view_maps(bev.calibration, yaw, bmsg.width,
                        tuple(cfg['semantic']['view_size']), cfg['semantic']['hfov_deg'],
                        cfg['projection']['min_radius'], cfg['projection']['max_radius'])
                    samples.append((merged['classes'][i], merged['confidence'][i], u-.5, v-.5, ok, False))
                label, _ = project_labels(samples, cfg['semantic']['score_min'], image.shape[:2])
            if cfg['visibility']['strict']:
                k = int(np.searchsorted(times, record['stamp_ns']))
                nearest = min((j for j in (k-1, k) if 0 <= j < len(scans)), key=lambda j: abs(int(times[j])-record['stamp_ns']))
                if abs(int(times[nearest])-record['stamp_ns']) > cfg['visibility']['max_age_sec']*1e9:
                    skipped.append(dict(frame=record['frame'], reason='No synchronized scan')); continue
                ground.strict_scans = {int(times[nearest]): scans[nearest][1]}
            ground.camera_height = float(bev.calibration['camera_position'][2])
            ground.pub.msg = None
            ground._emit(bmsg.header.stamp, bmsg.header.frame_id, image, label, publish_main=True, publish_semantic=False)
            cloud = ground.pub.msg
            if cloud is None or cloud.width == 0:
                skipped.append(dict(frame=record['frame'], reason='No scan-visible ground')); continue
            dtype = np.dtype([('x','<f4'), ('y','<f4'), ('z','<f4'), ('rgb','<f4'), ('weight','<f4'), ('semantic_id','u1')])
            if cloud.point_step != dtype.itemsize:
                raise RuntimeError('Unexpected native cloud layout')
            points = np.frombuffer(cloud.data, dtype)
            q = record['quaternion']; rotation = quaternion_to_matrix(*q)
            xyz = np.column_stack([points['x'], points['y'], points['z']])@rotation.T + record['xyz']
            gx = np.floor((xyz[:, 0]-origin[0])/res).astype(int); gy = np.floor((xyz[:, 1]-origin[1])/res).astype(int)
            if not ((gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)).all():
                raise RuntimeError('Native samples exceed trajectory grid bounds')
            flat = gy*width+gx; weight = points['weight']; sid = points['semantic_id']
            keep = (sid >= 3) & (sid <= 6)
            np.add.at(votes, (gy[keep], gx[keep], sid[keep]-3), weight[keep])
            np.add.at(coverage, (gy, gx), 1)
            packed = points['rgb'].copy().view(np.uint32)
            bgr = np.column_stack([packed & 255, (packed >> 8) & 255, (packed >> 16) & 255]).astype(np.float32)
            rear = directional_weights(points['x'], points['y'], cfg['texture']['rear_weight'])
            native = aggregate_frame(flat, bgr, weight, np.ones(len(points)), rear)
            # Store only aggregated RGB packets, never duplicate original JPEGs.
            np.savez_compressed(packets/f'{record["frame"]}.npz', **native)
            mapped.append(dict(**record, ground_points=len(points), scan_stamp_ns=(int(times[nearest]) if cfg['visibility']['strict'] else None)))
            if n % 25 == 0 or n == len(valid):
                progress = dict(phase='reverse_and_projection', completed=n, frames=len(valid), elapsed_sec=time.time()-started)
                write_json(output/'progress.json', progress); print(json.dumps(progress), flush=True)
        mapped.sort(key=lambda r: r['frame'])
        if not mapped:
            raise ValueError('No valid scan-visible map observations')
        for record in mapped:
            with np.load(packets/f'{record["frame"]}.npz') as p:
                fuser.update(p['index'], p['bgr'], p['q']*p['rear'], p['weight']*p['rear'])
        color, observed = fuser.result()
        texture = np.dstack((color.reshape(height, width, 3), observed.reshape(height, width).astype(np.uint8)*255))[::-1].copy()
        labels = np.where(votes.sum(axis=2) >= cfg['semantic']['min_weight'], np.argmax(votes, axis=2)+3, 0).astype(np.uint8)[::-1].copy()
        baseline = labels.copy()
        labels, refinements, layers = refine_semantics(labels, texture, votes[::-1], coverage[::-1], res, cfg)
        if ((labels > 0) & (texture[..., 3] == 0)).any():
            raise AssertionError('Semantic labels outside observed texture')
        # Refine semantics with original colors; tone affects exported RGB only.
        if cfg['texture']['tone']['enabled']:
            save_image(output/'road_temporal.raw.texture.png', texture)
        toned = lift_texture(texture, cfg['texture']['tone'])
        texture_tone = tone_report(texture, toned, cfg['texture']['tone'])
        texture = toned
        save_image(output/'road_temporal.texture.png', texture)
        save_image(output/'road_temporal.colored.pgm', labels)
        save_image(output/'baseline.class_ids.png', baseline)
        for name, image in layers.items():
            save_image(output/(name+'.png'), image)
        np.savez_compressed(output/'map_evidence.npz', temporal_votes=votes, sample_coverage=coverage)
        from sirius_navigation.theta_indexed_map_node import SEMANTIC_CLASSES
        from sirius_navigation.semantic_costs import load_semantic_costs, costs_for
        costs, _ = load_semantic_costs(); palette = np.zeros((125, 3), np.uint8)
        palette[:7] = [[127,127,127], [0,0,0], [255,255,255], [0,255,0], [255,255,0], [0,0,255], [128,128,128]]
        metadata = dict(resolution=res, origin=origin.tolist(), width=int(width), height=int(height),
            palette=palette.tolist(), semantic_encoding='class_id', map_frame=cfg['map_frame'],
            labels={str(cid): dict(name=info['name'], color=info['color'], **costs_for(costs, info['name']))
                    for cid, info in SEMANTIC_CLASSES.items()})
        write_json(output/'road_temporal.colored.json', metadata)
        manifest = dict(bag=str(bag), run_identity_sha256=identity_hash, input_stream_sha256=input_digest, config=cfg,
            full_bag=not args.max_frames, input_camera_frames=total_frames, selected_camera_frames=len(records),
            inferred_frames=len(inference_rows), frame_count=len(mapped), frames=mapped, skipped=skipped,
            reused_baseline_frames=reused_baseline_frames,
            grid={k: metadata[k] for k in ('origin', 'resolution', 'width', 'height')},
            refinements=refinements, camera_sha256_verified=True, recorded_tf_only=True,
            texture_tone=texture_tone,
            no_coordinate_warp=True, unobserved_space_not_filled=True,
            semantic_cells={str(k): int((labels == k).sum()) for k in range(3, 7)},
            elapsed_sec=time.time()-started, status='complete')
        render_review(output, labels, texture, res)
        write_json(output/'manifest.json', manifest)
        # Small packets are a temporary pass bridge; inference caches and provenance remain.
        for path in packets.glob('*.npz'):
            path.unlink()
        packets.rmdir()
        write_json(output/'completed.json', dict(status='complete', frame_count=len(mapped), identity=identity_hash))
        print(f'MAP COMPLETE: {output}/road_temporal ({len(mapped)}/{len(records)} frames)', flush=True)
        return manifest
    finally:
        if session is not None and hasattr(session, 'restoration'):
            write_json(output/'server_restoration.json', session.restoration)
        for node in (bev, ground):
            if node is not None:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('bag', type=Path); p.add_argument('output', type=Path)
    p.add_argument('--calibration', required=True, type=Path)
    p.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    p.add_argument('--server', default='http://localhost:8080')
    p.add_argument('--reuse-study', type=Path, help='Verified legacy full-image SAM3 cache; exact JPEG/settings checks required')
    p.add_argument('--domain', type=int, default=74)
    p.add_argument('--resume', action='store_true'); p.add_argument('--no-semantic', action='store_true')
    p.add_argument('--max-frames', type=int, default=0, help='Smoke test only; 0 processes all frames')
    args = p.parse_args()
    if args.max_frames < 0:
        p.error('--max-frames must be nonnegative')
    run(args)


if __name__ == '__main__':
    main()
