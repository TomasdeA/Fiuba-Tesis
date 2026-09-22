#!/usr/bin/env python3
"""Replay a bag in an isolated ROS domain; save poses and tracking diagnostics.

Run in the sourced ROS environment. Each run gets a fresh odometry process;
automatic resets remain disabled so a reset cannot masquerade as recovery.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import OdomInfo
import yaml

BASE = {
    'frame_id': 'camera_color_optical_frame', 'odom_frame_id': 'odom',
    'publish_tf': False, 'approx_sync': True, 'approx_sync_max_interval': .08,
    'wait_imu_to_init': True, 'Vis/MaxDepth': '4.0', 'Vis/MinInliers': '15',
    'Vis/FeatureType': '8', 'Vis/MaxFeatures': '600', 'OdomF2M/MaxSize': '3000',
    'Odom/Strategy': '0', 'Odom/ResetCountdown': '0', 'Odom/GuessMotion': 'true',
    'Odom/FilteringStrategy': '1', 'Vis/DepthAsMask': 'true',
    'always_check_imu_tf': False,
}
VARIANTS = {
    'baseline': {},
    'no_imu': {'wait_imu_to_init': False},
    'no_filter': {'Odom/FilteringStrategy': '0'},
    'tight_sync': {'approx_sync_max_interval': .01},
    'depth_unlimited': {'Vis/MaxDepth': '0'},
    'optical_flow_f2f': {'Vis/CorType': '1', 'Odom/Strategy': '1'},
    'f2f_descriptors': {'Odom/Strategy': '1'},
    'flow_no_filter': {'Vis/CorType': '1', 'Odom/Strategy': '1', 'Odom/FilteringStrategy': '0'},
    'flow_decimation2': {'Vis/CorType': '1', 'Odom/Strategy': '1', 'Odom/ImageDecimation': '2',
                         'Odom/FilteringStrategy': '0', 'approx_sync': False},
    'more_features': {'Vis/MaxFeatures': '1500', 'OdomF2M/MaxSize': '6000',
                      'Vis/GridRows': '3', 'Vis/GridCols': '4'},
    'decimation2': {'Odom/ImageDecimation': '2'},
    'reprojection4': {'Vis/PnPReprojError': '4'},
    'reprojection8': {'Vis/PnPReprojError': '8'},
    'orb': {'Vis/FeatureType': '2'},
    'exact_sync': {'approx_sync': False},
    'sift': {'Vis/FeatureType': '1'},
    'robust_orb': {'Vis/FeatureType': '2', 'Odom/ImageDecimation': '2',
                   'Vis/MaxFeatures': '1500', 'OdomF2M/MaxSize': '6000',
                   'Vis/GridRows': '3', 'Vis/GridCols': '4', 'Vis/PnPReprojError': '4'},
    'robust_gftt': {'Odom/ImageDecimation': '2', 'Vis/MaxFeatures': '1500',
                    'OdomF2M/MaxSize': '6000', 'Vis/GridRows': '3', 'Vis/GridCols': '4',
                    'Vis/PnPReprojError': '4'},
}
VARIANTS['robust_unfiltered'] = VARIANTS['robust_gftt'] | {'Odom/FilteringStrategy': '0', 'approx_sync_max_interval': .01}
VARIANTS['dense_decimation2'] = VARIANTS['robust_gftt'] | {'Vis/PnPReprojError': '2', 'approx_sync_max_interval': .01}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--variant', choices=VARIANTS, default='baseline')
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--domain', type=int, default=83)
    parser.add_argument('--override', type=Path, help='YAML parameter overrides')
    parser.add_argument('--full-stack', action='store_true', help='Run v2 perception and mapping as well')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    params = BASE | VARIANTS[args.variant]
    if args.override:
        overrides = yaml.safe_load(args.override.read_text())
        for root in ('rgbd_odometry', '/**'):
            if root in overrides:
                overrides = overrides[root]['ros__parameters']
                break
        params.update(overrides)
    config = args.output / 'parameters.yaml'
    config.write_text(yaml.safe_dump({'/**': {'ros__parameters': params}}))
    children, logs = [], []
    def start(name, command):
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        return process
    rclpy.init()
    node = rclpy.create_node('odometry_benchmark_observer')
    poses, infos = [], []
    def stamp(message):
        return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
    def odom(message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        poses.append({'stamp': stamp(message), 'valid': bool(message.pose.covariance[0] < 1000),
                      'p': [p.x, p.y, p.z], 'q': [q.x, q.y, q.z, q.w]})
    def info(message):
        infos.append(dict(stamp=stamp(message), lost=message.lost,
                          inliers=message.inliers, matches=message.matches,
                          features=message.features, time=message.time_estimation,
                          distance=message.distance_travelled))
    node.create_subscription(Odometry, '/nav_odom', odom, 1000)
    node.create_subscription(OdomInfo, '/odom_info', info, 1000)
    try:
        if not args.full_stack and args.variant != 'no_imu':
            start('imu', ['ros2', 'run', 'imu_filter_madgwick', 'imu_filter_madgwick_node',
                '--ros-args', '-p', 'use_mag:=false', '-p', 'publish_tf:=false',
                '-p', 'world_frame:=enu', '-p', 'gain:=0.01', '-p', 'zeta:=0.0',
                '-r', 'imu/data_raw:=/camera/camera/imu',
                '-r', 'imu/data:=/imu/data_filtered'])
        odometry_command = ['ros2', 'run', 'rtabmap_odom', 'rgbd_odometry',
            '--ros-args', '--params-file', str(config),
            '-r', 'rgb/image:=/camera/camera/color/image_raw',
            '-r', 'rgb/camera_info:=/camera/camera/color/camera_info',
            '-r', 'depth/image:=/camera/camera/aligned_depth_to_color/image_raw',
            '-r', 'imu:=/imu/data_filtered', '-r', 'odom:=/nav_odom']
        if args.full_stack:
            odometry_command = ['ros2', 'launch', 'nav_bringup', 'nav.launch.py',
                'use_realsense:=false', 'use_bag:=false', 'use_hw:=false',
                'use_perception:=true', 'pipeline_mode:=filtered', 'use_local_mapper:=true',
                'odom_source:=rtabmap_odom', 'use_spatial_awareness:=true',
                'use_viz:=false', 'use_rviz:=false', 'performance:=false',
                'rtabmap_odom_config:=' + str(config)]
        start('odometry', odometry_command)
        deadline = time.monotonic() + (8 if args.full_stack else 4)
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        playback = ['ros2', 'bag', 'play', str(args.bag), '--rate', str(args.rate)]
        source_topics = ['/camera/camera/color/image_raw', '/camera/camera/color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw', '/camera/camera/imu', '/tf_static']
        if args.full_stack:
            source_topics += ['/camera/camera/depth/image_rect_raw', '/camera/camera/depth/camera_info',
                '/camera/camera/aligned_depth_to_color/camera_info',
                '/camera/camera/gyro/sample', '/camera/camera/accel/sample']
        # Never replay previously generated odometry, dynamic TF or map outputs.
        playback += ['--topics', *source_topics]
        player = start('player', playback)
        timeout = time.monotonic() + 600 / args.rate
        while player.poll() is None and time.monotonic() < timeout:
            rclpy.spin_once(node, timeout_sec=.05)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.shutdown()
    result = {'bag': str(args.bag), 'variant': args.variant, 'rate': args.rate, 'full_stack': args.full_stack,
              'parameters': params, 'poses': poses, 'info': infos}
    (args.output / 'data.json').write_text(json.dumps(result))
    valid = [p for p in poses if p['valid']]
    summary = dict(total=len(poses), valid=len(valid), invalid_percent=100*(1-len(valid)/max(1,len(poses))))
    summary['unsupported_settings'] = [line for line in (args.output/'odometry.log').read_text().splitlines()
                                       if 'not supported' in line]
    if valid:
        t0 = poses[0]['stamp']
        longest_loss, loss_start = 0., None
        for pose in poses:
            if not pose['valid'] and loss_start is None:
                loss_start = pose['stamp']
            if pose['valid'] and loss_start is not None:
                longest_loss = max(longest_loss, pose['stamp']-loss_start)
                loss_start = None
        if loss_start is not None:
            longest_loss = max(longest_loss, poses[-1]['stamp']-loss_start)
        summary.update(last_valid_s=valid[-1]['stamp']-t0,
                       longest_loss_s=longest_loss,
                       max_displacement_m=float(np.max(np.linalg.norm(np.array([p['p'] for p in valid])-valid[0]['p'], axis=1))),
                       median_inliers=float(np.median([i['inliers'] for i in infos if not i['lost']])))
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary), flush=True)
    if not poses:
        raise RuntimeError('No odometry received; inspect logs')

if __name__ == '__main__':
    main()
