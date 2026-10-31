#!/usr/bin/env python3
"""Evaluate a finished live recording without publishing any ROS messages."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

import yaml


def evaluate(recording, world, output, launch_log=None, status_path=None, min_duration=600.):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from mapping_analysis import (
        compose, evaluate_geometry, load_truth, normalize_samples,
        scan_points, stamp_seconds, transform_points, transform_pose, yaw_from_quaternion,
    )
    from replay_mapping import bag_reader
    output.mkdir(parents=True, exist_ok=False)
    reader = bag_reader(recording/'rosbag')
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    poses, odom, static, scans = [], [], [], {}
    grid = None
    first_clock = last_clock = None
    counts = Counter()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ('/scan', '/pose', '/map', '/tf', '/tf_static', '/clock'):
            continue
        message = deserialize_message(data, types[topic])
        counts[topic] += 1
        if topic == '/clock':
            stamp = stamp_seconds(message.clock)
            if last_clock is not None and stamp < last_clock-1e-6:
                raise ValueError('Recording contains a clock reset')
            first_clock = stamp if first_clock is None else first_clock
            last_clock = stamp
        elif topic == '/scan':
            scans[round(stamp_seconds(message.header.stamp), 8)] = message
        elif topic == '/pose':
            p = message.pose.pose
            poses.append((stamp_seconds(message.header.stamp), p.position.x, p.position.y,
                          yaw_from_quaternion(p.orientation)))
        elif topic == '/map':
            grid = message
        elif topic == '/tf_static':
            static.extend(message.transforms)
        elif topic == '/tf':
            for t in message.transforms:
                if t.header.frame_id.lstrip('/') == 'odom' and t.child_frame_id.lstrip('/') == 'base_footprint':
                    odom.append((stamp_seconds(t.header.stamp), *transform_pose(t)))
    frames = {'base_footprint': (0., 0., 0.)}
    while True:
        added = {}
        for t in static:
            parent, child = t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/')
            if parent in frames and child not in frames:
                added[child] = compose(frames[parent], transform_pose(t))
        if not added:
            break
        frames.update(added)
    localized = []
    for pose in poses:
        scan = scans.get(round(pose[0], 8))
        if scan is not None:
            frame = scan.header.frame_id.lstrip('/')
            if frame not in frames:
                raise ValueError(f'Missing recorded laser extrinsics: {frame}')
            localized.append((pose[0], transform_points(scan_points(scan, 3),
                                                       compose(pose[1:], frames[frame]))))
    truth = load_truth(recording)
    report = evaluate_geometry(poses, truth, localized, grid, world, output)
    odom = normalize_samples(odom)
    from mapping_events import write_correction_diagnostics, write_event_windows
    sources = {'slam_pose': poses, 'odom_tf': odom, 'ground_truth': truth}
    _, diagnostics = write_correction_diagnostics(poses, odom, output, sources, recording)
    if report.get('first_prefix_direction_violation'):
        write_event_windows([report['first_prefix_direction_violation']], output,
                            sources, recording, 'wall_fit_events')
    if report.get('trajectory', {}).get('first_threshold_crossing'):
        write_event_windows([report['trajectory']['first_threshold_crossing']], output,
                            sources, recording, 'trajectory_events')
    jumps = diagnostics['correction_jumps']
    duration = last_clock-first_clock if first_clock is not None else 0.
    report.update(duration_sim_s=duration, processed_scans=len(poses),
                  localized_scans=len(localized), correction_jump_stamps=[s['stamp_s'] for s in jumps],
                  correction_diagnostics=diagnostics,
                  message_counts=dict(counts), truth_published=False,
                  navigation_acceptance='not_evaluated')
    if jumps:
        report.update(acceptance='fail', reason='Unexpected map->odom correction jump')
    if duration < min_duration or not poses or len(localized) < .95*len(poses) or diagnostics['missing_odom_brackets']:
        report.update(acceptance='incomplete', reason=f'Need >={min_duration} seconds, paired scans/poses, and odometry brackets')
    if launch_log:
        log = launch_log.read_text(errors='replace')
        success = log.count('导航终止：成功')
        failures = log.count('导航终止：失败/取消')
        state = yaml.safe_load(status_path.read_text()) if status_path and status_path.exists() else None
        recent = [p for p in truth if truth[-1][0]-p[0] <= 60.] if truth else []
        movement = sum(math.hypot(b[1]-a[1], b[2]-a[2]) for a, b in zip(recent, recent[1:]))
        # Sensor jitter is small, but use displacement span as an additional guard.
        span = max((math.hypot(p[1]-recent[0][1], p[2]-recent[0][2]) for p in recent), default=0.)
        completed = bool(state and state.get('completed'))
        stalled = bool(state and not state.get('completed'))
        nav_pass = duration >= min_duration and success > 0 and not stalled and (completed or span >= .10)
        report['navigation'] = {'goals_succeeded': success, 'goals_failed_or_cancelled': failures,
                                'final_exploration_state': state, 'last_60s_distance_m': movement,
                                'last_60s_displacement_span_m': span}
        report['navigation_acceptance'] = 'pass' if nav_pass else 'fail'
    (output/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('map_walls', 'scan_walls')}, indent=2))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--world', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--launch-log', type=Path)
    parser.add_argument('--exploration-status', type=Path)
    parser.add_argument('--min-duration', type=float, default=600., help='Required simulation duration; use 1500 for long patrol acceptance')
    args = parser.parse_args(argv)
    if not math.isfinite(args.min_duration) or args.min_duration <= 0:
        parser.error('--min-duration must be finite and positive')
    try:
        result = evaluate(args.recording, args.world, args.output, args.launch_log,
                          args.exploration_status, args.min_duration)
        return 0 if (result['acceptance'] == 'pass'
                     and result['navigation_acceptance'] in ('pass', 'not_evaluated')) else 1
    except (OSError, ValueError, RuntimeError) as error:
        print(f'Evaluation failed: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
