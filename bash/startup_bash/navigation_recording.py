"""Navigation-bag topics, discovery report and map/configuration copies."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
from datetime import datetime

import yaml

TOPICS = '''
/experiment_metadata /scan3 /scan /odom /odom/filtered /odometry/filtered /imu /magnetometer
/Odometry /fastlio/diagnostics /diagnostics /rosout /tf /tf_static /robot_description /joint_states
/cmd_vel /cmd_vel_nav /cmd_vel_teleop /cmd_vel_web /cmd_vel_smoothed /cmd_vel_direct /cmd_vel_idle
/amcl_pose /initialpose /map /map_updates /plan /optimal_trajectory /goal_pose
/global_costmap/costmap /global_costmap/costmap_updates /global_costmap/costmap_raw
/global_costmap/costmap_raw_updates /global_costmap/published_footprint /global_costmap/footprint
/local_costmap/costmap /local_costmap/costmap_updates /local_costmap/costmap_raw
/local_costmap/costmap_raw_updates /local_costmap/published_footprint /local_costmap/footprint
/roboteq/odom /roboteq/status /blinker_led_command /npc/odom
/target_detector/target_markers /target_follower/status /robot_path /robot_path_marker
/llm_instruction /sirius/command_queue /sirius/landmark_status /sirius/landmark_markers
/nav_control /stop /sirius/battery_status /battery_state /parameter_events
/sam3/static_texture_map_cloud /sam3/static_colored_map_cloud
/sam3/static_colored_map_grid /sam3/static_semantic_cost_grid
/map_localizer/pose /map_localizer/status /map_localizer/correction
/map_localizer/tf_status /localization_ready
'''.split()


def query(*args):
    try:
        result = subprocess.run(['ros2', *args], capture_output=True, text=True, timeout=5)
        return result.stdout.strip() if result.returncode == 0 else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def copy_assets(prefix, destination):
    """Copy only the files the loaders use; preserve the original bytes."""
    prefix=Path(prefix).expanduser()
    candidates=[Path(str(prefix)+suffix) for suffix in ('.colored.pgm','.colored.json','.pgm','.yaml','.yml')]
    if prefix.is_file():
        candidates.append(prefix)
        if prefix.suffix.lower() in ('.yaml','.yml'):
            try:
                image=yaml.safe_load(prefix.read_text()).get('image')
                if isinstance(image,str):candidates.append(prefix.parent/image)
            except (OSError,ValueError,yaml.YAMLError):pass
    copied=[]
    for source in dict.fromkeys(candidates):
        if not source.is_file():continue
        destination.mkdir(parents=True,exist_ok=True)
        target=destination/source.name
        shutil.copy2(source,target)
        copied.append(dict(source=str(source),copy=str(target),sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    return copied


def snapshot(destination, workspace):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    report=dict(created_at=datetime.now().astimezone().isoformat(),ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),
                topics=TOPICS,parameters=[],maps=[],warnings=[])
    nodes=query('node','list')
    if nodes is None:
        report['warnings'].append('ROSノード一覧を取得できませんでした')
        nodes=''
    for node in dict.fromkeys(nodes.splitlines()):
        leaf=node.rsplit('/',1)[-1]
        if leaf not in {'sam3_colored_map_loader','map_server','laser_mapping','map_localizer','robot_state_publisher'}:continue
        dump=query('param','dump',node)
        if dump:
            target=destination/'parameters'/(node.strip('/').replace('/','_')+'.yaml')
            target.parent.mkdir(exist_ok=True);target.write_text(dump+'\n')
            report['parameters'].append(dict(node=node,file=str(target)))
        key={'sam3_colored_map_loader':'map_path','map_server':'yaml_filename'}.get(leaf)
        if key:
            value=query('param','get','--hide-type',node,key)
            if value:
                files=copy_assets(value,destination/'maps'/node.strip('/').replace('/','_'))
                report['maps'].append(dict(node=node,parameter=key,value=value,files=files))
                if not files:report['warnings'].append(node+' の地図ファイルが見つかりません：'+value)
    for relative in ['params/fast_lio_sirius_real.yaml','params/fast_lio_sirius_sim.yaml',
                     'src/sirius/sirius_navigation/config/fast_lio_sirius.yaml',
                     'bash/startup_bash/start_3d_lio.sh','params/rosbag_navigation_qos.yaml']:
        source=Path(workspace)/relative
        if source.is_file():
            target=destination/'configuration'/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    (destination/'recording_context.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(f'設定・地図の控え: {destination}（地図 {sum(len(row["files"]) for row in report["maps"])}ファイル）',flush=True)
    for warning in report['warnings']:print('  注意: '+warning,flush=True)
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topics',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--capture',type=Path)
    parser.add_argument('--workspace',type=Path,default=Path.home()/'sirius_jazzy_ws')
    args=parser.parse_args()
    if args.topics:print('\n'.join(TOPICS))
    if args.check:
        live=query('topic','list','--include-hidden-topics')
        found=set((live or '').splitlines())
        print('記録対象（✓ ROSグラフで検出 / … 未検出・開始後も検出を待ちます）:')
        for topic in TOPICS:print(('  ✓ ' if topic in found else '  … ')+topic)
        if live is None:print('  トピック一覧を取得できませんでした。ROS_DOMAIN_IDを確認してください。')
    if args.capture:snapshot(args.capture,args.workspace)


if __name__=='__main__':main()
