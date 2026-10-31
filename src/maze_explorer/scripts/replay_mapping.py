#!/usr/bin/env python3
"""Filter a recorded bag, then compare SLAM configurations in an isolated domain.

No navigation, simulator, EKF, velocity publisher, or ground-truth TF is started.
The only dynamic input TF allowed is odom -> base_footprint. Truth is read only
after replay, for evaluation. Run --help after sourcing ROS 2 Jazzy.
"""

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

PROFILE_NAMES = ('baseline', 'baseline_turn', 'near_chains', 'heading_prior',
                 'heading_prior_turn', 'simulation_stable', 'odometry_only')


def bag_reader(path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def prepare(source, destination, raw_odom='/model/my_robot/odometry'):
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from sensor_msgs.msg import LaserScan
    from mapping_analysis import allowed_transforms, robot_static_frames, stamp_seconds

    if destination.exists():
        raise ValueError(f'Refusing to overwrite {destination}')
    reader = bag_reader(source)
    permitted = {'/clock', '/scan', '/imu', '/odometry/filtered', raw_odom,
                 '/tf', '/tf_static'}
    topics = [t for t in reader.get_all_topics_and_types() if t.name in permitted]
    required = {'/clock', '/scan', '/tf', '/tf_static'}
    if required - {t.name for t in topics}:
        raise ValueError(f'Missing replay topics: {required - {t.name for t in topics}}')
    static_transforms = []
    odom_first = odom_last = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            static_transforms.extend(deserialize_message(data, TFMessage).transforms)
        elif topic == '/tf':
            for t in allowed_transforms(deserialize_message(data, TFMessage)):
                stamp = stamp_seconds(t.header.stamp)
                odom_first = stamp if odom_first is None else min(odom_first, stamp)
                odom_last = stamp if odom_last is None else max(odom_last, stamp)
    if odom_first is None:
        raise ValueError('No odom -> base_footprint TF coverage')
    frames = robot_static_frames(static_transforms)
    reader = bag_reader(source)
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    for topic in topics:
        writer.create_topic(topic)
    counts, removed = Counter(), Counter()
    clock_first = clock_last = None
    uncovered_scans = []
    digest = hashlib.sha256()
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in permitted:
            continue
        if topic in ('/tf', '/tf_static'):
            message = deserialize_message(data, TFMessage)
            transforms = allowed_transforms(message, static=topic == '/tf_static', frames=frames)
            removed[topic] += len(message.transforms) - len(transforms)
            if not transforms:
                continue
            message.transforms = transforms
            data = serialize_message(message)
        elif topic == '/clock':
            clock = stamp_seconds(deserialize_message(data, Clock).clock)
            if clock_last is not None and clock < clock_last - 1e-6:
                raise ValueError('Simulation clock reset inside input; record runs separately')
            clock_first = clock if clock_first is None else clock_first
            clock_last = clock
        elif topic == '/scan':
            scan_stamp = stamp_seconds(deserialize_message(data, LaserScan).header.stamp)
            if scan_stamp < odom_first-1e-9 or scan_stamp > odom_last+1e-9:
                # Recording can stop between the final scan and its odometry TF.
                # Exclude only unsupported boundary scans, equally for all profiles.
                uncovered_scans.append(scan_stamp)
                continue
        writer.write(topic, data, stamp)
        counts[topic] += 1
        digest.update(topic.encode() + str(stamp).encode() + data)
    del writer
    if not counts['/tf'] or not counts['/tf_static']:
        raise ValueError('No whitelisted odometry/static TF survived filtering')
    report = {'source': str(source.resolve()), 'topics': dict(counts),
              'raw_odom_topic': raw_odom,
              'static_robot_frames': sorted(frames),
              'odom_tf_coverage_s': [odom_first, odom_last],
              'excluded_scans_outside_odom_tf': uncovered_scans,
              'removed_tf_edges': dict(removed),
              'duration_sim_s': clock_last-clock_first if clock_first is not None else 0,
              'sha256': digest.hexdigest(), 'truth_published': False}
    (destination/'filter_report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return report


class Replay:
    def __init__(self, args):
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from lifecycle_msgs.srv import ChangeState
        from nav_msgs.msg import OccupancyGrid
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from tf2_ros import Buffer
        from mapping_analysis import transform_pose, stamp_seconds

        self.args = args
        self.rclpy = rclpy
        self.deserialize = deserialize_message
        self.get_message = get_message
        rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.node = Node('mapping_replay', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.buffer = Buffer()
        self.publishers = {}
        self.types = {}
        self.odom = []
        self.static = []
        all_static = []
        manifest = args.input/'filter_report.json'
        if not manifest.is_file():
            raise ValueError('No completed filter_report.json; run prepare first')
        reader = bag_reader(args.input)
        for topic in reader.get_all_topics_and_types():
            if (topic.name not in {'/clock', '/scan', '/imu', '/odometry/filtered',
                                   '/tf', '/tf_static'}
                    and not (topic.name.startswith('/model/') and topic.name.endswith('/odometry')
                             and topic.type == 'nav_msgs/msg/Odometry')):
                raise ValueError(f'Unfiltered topic {topic.name}; run prepare first')
            self.types[topic.name] = get_message(topic.type)
            qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
            if topic.name == '/tf_static':
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.publishers[topic.name] = self.node.create_publisher(self.types[topic.name], topic.name, qos)
        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic in ('/tf', '/tf_static'):
                message = deserialize_message(data, self.types[topic])
                if topic == '/tf_static':
                    self.static.append(message)
                    all_static.extend(message.transforms)
                    for t in message.transforms:
                        self.buffer.set_transform_static(t, 'recorded_static')
                else:
                    from mapping_analysis import allowed_transforms
                    if len(allowed_transforms(message)) != len(message.transforms):
                        raise ValueError('Input contains forbidden dynamic TF edges; run prepare first')
                    for t in message.transforms:
                        self.odom.append((stamp_seconds(t.header.stamp), *transform_pose(t)))
        from mapping_analysis import allowed_transforms, normalize_samples, robot_static_frames
        frames = robot_static_frames(all_static)
        for message in self.static:
            if len(allowed_transforms(message, static=True, frames=frames)) != len(message.transforms):
                raise ValueError('Input contains unrelated static TF edges; run prepare first')
        self.odom = normalize_samples(self.odom)
        self.pose_rows = []
        self.scans = {}
        self.localized_scans = []
        self.map = None
        self.residuals = []
        self.last_residual_stamp = -math.inf
        self.processes = []
        self.streams = []
        self.cancelled = False
        self.pose_subscription = self.node.create_subscription(
            PoseWithCovarianceStamped, '/pose', self.pose_callback, 1000)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_subscription = self.node.create_subscription(
            OccupancyGrid, '/map', lambda message: setattr(self, 'map', message), map_qos)
        self.change = self.node.create_client(ChangeState, '/slam_toolbox/change_state')

    def pose_callback(self, message):
        from mapping_analysis import (
            compose, interpolate, inverse, nearest_distances, occupied_points,
            percentiles, scan_points, stamp_seconds, transform_points,
            transform_pose, yaw_from_quaternion,
        )
        from rclpy.time import Time
        stamp = stamp_seconds(message.header.stamp)
        pose = message.pose.pose
        sample = (stamp, pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation))
        self.pose_rows.append(sample)
        scan = self.scans.pop(round(stamp, 8), None)
        if scan is None:
            return
        try:
            laser = self.buffer.lookup_transform('base_footprint', scan.header.frame_id, Time())
        except Exception as error:
            raise RuntimeError(f'Missing static laser extrinsics: {error}') from error
        laser_pose = compose(sample[1:], transform_pose(laser))
        points = transform_points(scan_points(scan, stride=3), laser_pose)
        # Keep sampled scans for revisit geometry; timestamp always comes from the sensor.
        self.localized_scans.append((stamp, points))
        odom = interpolate(self.odom, stamp, .15)
        correction = compose(sample[1:], inverse(odom[1:])) if odom else None
        if stamp-self.last_residual_stamp >= 1. and self.map is not None:
            stats = percentiles(nearest_distances(points, occupied_points(self.map)))
            self.residuals.append({'stamp_s': stamp, **stats,
                                   'map_to_odom_yaw_deg': math.degrees(correction[2]) if correction else None})
            self.last_residual_stamp = stamp

    def spin_for(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline and not self.cancelled:
            self.rclpy.spin_once(self.node, timeout_sec=min(.01, max(0., deadline-time.monotonic())))

    def call(self, client, request, timeout=20.):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        future = client.call_async(request)
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic() < deadline and not self.cancelled:
            self.rclpy.spin_once(self.node, timeout_sec=.05)
        if not future.done():
            raise RuntimeError(f'Service timed out: {client.srv_name}')
        return future.result()

    def start(self):
        from lifecycle_msgs.srv import ChangeState
        from ament_index_python.packages import get_package_prefix
        self.spin_for(.5)
        others = [name for name, _ in self.node.get_node_names_and_namespaces() if name != 'mapping_replay']
        if others:
            raise RuntimeError(f'ROS domain is not isolated; existing nodes: {others}')
        config = yaml.safe_load(self.args.params.read_text())
        profiles = yaml.safe_load(self.args.profiles.read_text())
        if self.args.profile not in profiles:
            raise ValueError(f'Unknown profile {self.args.profile}')
        config['slam_toolbox']['ros__parameters'].update(profiles[self.args.profile])
        config['slam_toolbox']['ros__parameters'].update({'use_sim_time': True, 'use_lifecycle_manager': False})
        resolved = self.args.output/'resolved_params.yaml'
        resolved.write_text(yaml.safe_dump(config))
        executable = Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/async_slam_toolbox_node'
        log = (self.args.output/'slam.log').open('w')
        self.streams.append(log)
        self.processes.append(subprocess.Popen([str(executable), '--ros-args', '--params-file', str(resolved)],
                                               stdout=log, stderr=log, start_new_session=True))
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            if not self.call(self.change, request).success:
                raise RuntimeError(f'SLAM lifecycle transition {transition} failed')
        self.spin_for(.5)
        for message in self.static:
            self.publishers['/tf_static'].publish(message)
        self.spin_for(.3)

    def run(self):
        from mapping_analysis import stamp_seconds
        from rcl_interfaces.srv import GetParameters
        self.start()
        # Capture values from the node, not just from the YAML written above.
        client = self.node.create_client(GetParameters, '/slam_toolbox/get_parameters')
        request = GetParameters.Request()
        request.names = ['use_scan_matching', 'do_loop_closing', 'use_scan_barycenter',
                         'link_scan_maximum_distance', 'link_match_minimum_response_fine',
                         'angle_variance_penalty', 'coarse_search_angle_offset', 'use_response_expansion',
                         'check_min_dist_and_heading_precisely', 'minimum_travel_distance',
                         'minimum_travel_heading', 'minimum_time_interval', 'transform_timeout',
                         'minimum_angle_penalty', 'coarse_angle_resolution', 'fine_search_angle_offset',
                         'restamp_tf', 'resolution', 'scan_queue_size']
        response = self.call(client, request)
        from rclpy.parameter import parameter_value_to_python
        (self.args.output/'runtime_params.json').write_text(json.dumps(
            {k: parameter_value_to_python(v) for k, v in zip(request.names, response.values)}, indent=2))
        reader = bag_reader(self.args.input)
        start_wall = time.monotonic()
        first = last = None
        next_notice = start_wall+20.
        for_message_count = Counter()
        last_clock_publish_stamp = None
        clock_period_ns = int(1e9 / self.args.clock_frequency)
        while reader.has_next() and not self.cancelled:
            topic, data, stamp = reader.read_next()
            first = stamp if first is None else first
            last = stamp
            # The simulator records /clock at 1 kHz, while the fastest sensor
            # used here is 100 Hz. Publishing and spinning all 1.5 million
            # clock samples can starve scan callbacks during accelerated replay.
            # Keep ROS time monotonic at a sensor-safe rate instead.
            if (topic == '/clock' and last_clock_publish_stamp is not None and
                    stamp-last_clock_publish_stamp < clock_period_ns):
                continue
            due = start_wall + (stamp-first)*1e-9/self.args.rate
            self.spin_for(max(0., due-time.monotonic()))
            message = self.deserialize(data, self.types[topic])
            if topic == '/scan':
                self.scans[round(stamp_seconds(message.header.stamp), 8)] = message
                # Bound unprocessed scans (motion thresholds legitimately skip many).
                while len(self.scans) > 200:
                    self.scans.pop(next(iter(self.scans)))
            self.publishers[topic].publish(message)
            if topic == '/clock':
                last_clock_publish_stamp = stamp
            for_message_count[topic] += 1
            self.rclpy.spin_once(self.node, timeout_sec=0.)
            if self.processes[0].poll() is not None:
                raise RuntimeError('SLAM exited; inspect slam.log')
            if time.monotonic() >= next_notice:
                print(f'{self.args.profile}: {(stamp-first)*1e-9:.1f} sim s; '
                      f'{len(self.pose_rows)} processed poses', flush=True)
                next_notice = time.monotonic()+20.
        # A few simulation seconds of clock let the final map publication finish.
        from rosgraph_msgs.msg import Clock
        clock = Clock()
        final_sim = self.node.get_clock().now().nanoseconds
        for offset in range(1, 31):
            clock.clock.sec, clock.clock.nanosec = divmod(final_sim + offset*100_000_000, 1_000_000_000)
            self.publishers['/clock'].publish(clock)
            self.spin_for(.03)
        self.spin_for(1.)
        return {'published': dict(for_message_count), 'cancelled': self.cancelled,
                'playback_duration_s': (last-first)*1e-9 if first is not None else 0.,
                'wall_duration_s': time.monotonic()-start_wall}

    def evaluate(self, playback):
        from mapping_analysis import (
            compose, evaluate_geometry, inverse, interpolate, load_truth, save_grid,
        )
        directory = self.args.output
        with (directory/'slam_poses.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['stamp_s', 'x_m', 'y_m', 'yaw_rad'])
            writer.writerows(self.pose_rows)
        with (directory/'scan_map_residual.csv').open('w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['stamp_s', 'count', 'median_m', 'p95_m', 'max_m', 'map_to_odom_yaw_deg'])
            writer.writeheader()
            writer.writerows(self.residuals)
        from mapping_events import write_correction_diagnostics, write_event_windows
        truth = load_truth(self.args.recording) if self.args.recording else []
        corrections, diagnostics = write_correction_diagnostics(
            self.pose_rows, self.odom, directory,
            {'slam_pose': self.pose_rows, 'odom_tf': self.odom, 'ground_truth': truth},
            self.args.recording)
        with (directory/'map_to_odom.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['scan_stamp_s', 'x_m', 'y_m', 'yaw_rad'])
            writer.writerows(corrections)
        report = {'profile': self.args.profile, 'playback': playback,
                  'processed_scans': len(self.pose_rows), 'truth_published': False,
                  'navigation_acceptance': 'not_evaluated',
                  'acceptance': 'incomplete', 'reason': 'Ground truth/reference/revisits required'}
        provenance = self.args.input/'filter_report.json'
        report['input_sha256'] = json.loads(provenance.read_text()).get('sha256') if provenance.exists() else None
        jumps = diagnostics['correction_jumps']
        report['correction_diagnostics'] = diagnostics
        report['correction_jumps'] = jumps
        if self.map is not None:
            save_grid(self.map, directory)
        if self.args.recording:
            report.update(evaluate_geometry(self.pose_rows, truth, self.localized_scans,
                                            self.map, self.args.world, directory))
            if report.get('first_prefix_direction_violation'):
                write_event_windows([report['first_prefix_direction_violation']], directory,
                                    {'slam_pose': self.pose_rows, 'odom_tf': self.odom, 'ground_truth': truth},
                                    self.args.recording, 'wall_fit_events')
            if report.get('trajectory', {}).get('first_threshold_crossing'):
                write_event_windows([report['trajectory']['first_threshold_crossing']], directory,
                                    {'slam_pose': self.pose_rows, 'odom_tf': self.odom, 'ground_truth': truth},
                                    self.args.recording, 'trajectory_events')
            if jumps:
                report.update(acceptance='fail', reason='Unexpected map->odom correction jump')
        log = (directory/'slam.log').read_text(errors='replace')
        report['message_filter_failures'] = log.count('Message Filter dropping message')
        if (self.cancelled or not self.pose_rows or self.map is None
                or report['message_filter_failures'] or diagnostics['missing_odom_brackets']):
            report['acceptance'] = 'incomplete'
            report['reason'] = 'Cancelled, missing outputs, or dropped input scans; do not compare as a pass'
        if playback['playback_duration_s'] < 600.:
            report['acceptance'] = 'incomplete'
            report['reason'] = 'Less than 600 simulation seconds; a short replay is not acceptance'
        (directory/'summary.json').write_text(json.dumps(report, indent=2))
        print(json.dumps({k: v for k, v in report.items() if k not in ('map_walls', 'scan_walls')}, indent=2), flush=True)
        return report

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5)
        for stream in self.streams:
            stream.close()
        self.node.destroy_node()
        self.rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    filtering = sub.add_parser('prepare', help='Create a new bag containing only safe replay inputs')
    filtering.add_argument('--input', type=Path, required=True)
    filtering.add_argument('--output', type=Path, required=True)
    filtering.add_argument('--raw-odom', default='/model/my_robot/odometry')
    run = sub.add_parser('run', help='Start one SLAM instance and replay a filtered bag')
    run.add_argument('--input', type=Path, required=True)
    run.add_argument('--output', type=Path, required=True)
    run.add_argument('--profile', choices=PROFILE_NAMES, required=True)
    run.add_argument('--params', type=Path)
    run.add_argument('--profiles', type=Path)
    run.add_argument('--recording', type=Path, help='Recorder output containing poses.csv truth (evaluation only)')
    run.add_argument('--world', type=Path, help='Generator world, used only after replay to evaluate walls')
    run.add_argument('--domain', type=int, default=185, help='Unused local ROS domain, never the live robot domain')
    run.add_argument('--rate', type=float, default=1., help='Wall-time replay speed; lower if scans are dropped')
    run.add_argument('--clock-frequency', type=float, default=100.,
                     help='Replay /clock publication rate; 100 Hz matches the fastest recorded sensor')
    compare = sub.add_parser('compare', help='Filter once and run all four profiles sequentially')
    compare.add_argument('--recording', type=Path, required=True, help='Recorder output directory')
    compare.add_argument('--world', type=Path, required=True)
    compare.add_argument('--output', type=Path, required=True, help='New output directory')
    compare.add_argument('--domain', type=int, default=185)
    compare.add_argument('--rate', type=float, default=1.)
    compare.add_argument('--clock-frequency', type=float, default=100.)
    compare.add_argument('--params', type=Path)
    compare.add_argument('--compare-profiles', nargs='+', choices=PROFILE_NAMES,
                         default=['baseline', 'baseline_turn', 'heading_prior', 'heading_prior_turn'],
                         help='Configurations sharing exactly the same filtered inputs')
    args = parser.parse_args(argv)
    replay = None
    try:
        if args.command == 'prepare':
            prepare(args.input, args.output, args.raw_odom)
            return 0
        if (not math.isfinite(args.rate) or args.rate <= 0 or
                not math.isfinite(args.clock_frequency) or args.clock_frequency < 100 or
                args.clock_frequency > 1000 or not 0 <= args.domain <= 232):
            parser.error('rate must be positive, clock frequency in [100,1000], and domain in 0..232')
        if args.command == 'compare':
            args.output.mkdir(parents=True, exist_ok=False)
            report = prepare(args.recording/'rosbag', args.output/'filtered')
            results = {'input': report, 'profiles': {}}
            for profile in dict.fromkeys(args.compare_profiles):
                command = [sys.executable, str(Path(__file__).resolve()), 'run',
                           '--input', str(args.output/'filtered'), '--output', str(args.output/profile),
                           '--recording', str(args.recording), '--world', str(args.world),
                           '--profile', profile, '--domain', str(args.domain), '--rate', str(args.rate),
                           '--clock-frequency', str(args.clock_frequency)]
                if args.params:
                    command += ['--params', str(args.params)]
                result = subprocess.run(command, check=False)
                summary = args.output/profile/'summary.json'
                results['profiles'][profile] = json.loads(summary.read_text()) if summary.exists() else {
                    'acceptance': 'incomplete', 'exit_code': result.returncode}
                (args.output/'comparison_summary.json').write_text(json.dumps(results, indent=2))
            # A diagnostic-only winner is never installed as the navigation default.
            candidate_passed = any(r['acceptance'] == 'pass' for p, r in results['profiles'].items()
                                   if p not in ('baseline', 'odometry_only'))
            incomplete = any(r['acceptance'] == 'incomplete' for r in results['profiles'].values())
            return 0 if candidate_passed and not incomplete else 1
        from ament_index_python.packages import get_package_share_directory
        config = Path(get_package_share_directory('maze_explorer'))/'config'
        args.params = args.params or config/'slam_toolbox.yaml'
        args.profiles = args.profiles or config/'slam_profiles.yaml'
        args.output.mkdir(parents=True, exist_ok=False)
        os.environ['ROS_DOMAIN_ID'] = str(args.domain)
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
        os.environ.pop('ROS_LOCALHOST_ONLY', None)
        os.environ['ROS_LOG_DIR'] = str(args.output.resolve()/'ros_logs')
        replay = Replay(args)
        signal.signal(signal.SIGINT, lambda *_: setattr(replay, 'cancelled', True))
        signal.signal(signal.SIGTERM, lambda *_: setattr(replay, 'cancelled', True))
        result = replay.evaluate(replay.run())
        return 0 if result['acceptance'] == 'pass' else 1
    except (OSError, ValueError, RuntimeError) as error:
        print(f'Replay failed: {error}', file=sys.stderr)
        return 2
    finally:
        if replay is not None:
            replay.close()


if __name__ == '__main__':
    sys.exit(main())
