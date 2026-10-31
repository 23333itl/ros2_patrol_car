#!/usr/bin/env python3
"""Read-only ROS/Gazebo recorder for separating odometry and SLAM drift.

The SceneBroadcaster's dynamic_pose/info contains model poses, not DiffDrive
odometry. Read its JSON directly: Pose_V -> TFMessage conversion discards the
model name and the outer timestamp needed to identify these samples.
"""

import argparse
import bisect
import csv
from datetime import datetime, timezone
from functools import partial
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def stamp_seconds(stamp):
    if isinstance(stamp, dict):
        return float(stamp.get('sec', 0)) + float(
            stamp.get('nsec', stamp.get('nanosec', 0))) * 1e-9
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    if isinstance(q, dict):
        x, y, z, w = (float(q.get(k, 0)) for k in ('x', 'y', 'z', 'w'))
    else:
        x, y, z, w = q.x, q.y, q.z, q.w
    if x * x + y * y + z * z + w * w < 0.5:
        return None
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def interpolate(samples, stamp, max_gap):
    """Interpolate unwrapped yaw; never cross an absent/paused/reset interval."""
    if not samples:
        return None
    index = bisect.bisect_left(samples, stamp, key=lambda sample: sample[0])
    if index < len(samples) and abs(samples[index][0] - stamp) < 1e-8:
        return samples[index]
    if index == 0 or index == len(samples):
        return None
    before, after = samples[index - 1], samples[index]
    if stamp - before[0] > max_gap or after[0] - stamp > max_gap:
        return None
    fraction = (stamp - before[0]) / (after[0] - before[0])
    return (stamp, *(a + fraction * (b - a)
                     for a, b in zip(before[1:], after[1:])))


def normalize_samples(samples):
    """Drop duplicate stamps and unwrap yaw for interpolation/turn statistics."""
    by_stamp = {sample[0]: sample for sample in samples}
    result = []
    last_raw = last_unwrapped = None
    for stamp, x, y, yaw in sorted(by_stamp.values()):
        unwrapped = yaw if last_raw is None else (
            last_unwrapped + angle_difference(yaw, last_raw))
        result.append((stamp, x, y, unwrapped))
        last_raw, last_unwrapped = yaw, unwrapped
    return result


def detect_turns(ground_truth):
    """Find turns above 20 degrees, allowing 0.5 s of slow motion at edges."""
    turns = []
    start = last_moving = None
    for previous, current in zip(ground_truth, ground_truth[1:]):
        dt = current[0] - previous[0]
        if dt <= 0 or dt > 0.5:
            start = last_moving = None
            continue
        turning = abs((current[3] - previous[3]) / dt) > 0.04
        if turning:
            if start is None:
                start = previous
            last_moving = current
        elif start is not None and current[0] - last_moving[0] > 0.5:
            if abs(last_moving[3] - start[3]) >= math.radians(20):
                turns.append((start, last_moving))
            start = last_moving = None
    if start is not None and abs(last_moving[3] - start[3]) >= math.radians(20):
        turns.append((start, last_moving))
    return turns


class CsvOutput:
    def __init__(self, directory):
        self.directory = directory
        self.files = []

    def writer(self, name, columns):
        stream = (self.directory / name).open('w', newline='', encoding='utf-8')
        self.files.append(stream)
        writer = csv.writer(stream)
        writer.writerow(columns)
        return writer

    def close(self):
        for stream in self.files:
            stream.close()


class Diagnostics:
    def __init__(self, args):
        # Delay ROS imports so --help also works in an unsourced terminal.
        import rclpy
        from nav_msgs.msg import Odometry
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_msgs.msg import TFMessage
        from tf2_ros import Buffer, TransformListener
        from rcl_interfaces.srv import GetParameters

        self.args = args
        self.rclpy = rclpy
        rclpy.init(args=[], signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
        self.node = Node('mapping_diagnostics', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.output = CsvOutput(args.output)
        self.timing_csv = self.output.writer('timing.csv', [
            'stream', 'receive_sim_s', 'stamp_s', 'age_s', 'receive_monotonic_s',
            'frame_id', 'child_frame_id'])
        self.pose_csv = self.output.writer('poses.csv', [
            'source', 'stamp_s', 'receive_sim_s', 'x_m', 'y_m', 'yaw_rad'])
        self.imu_csv = self.output.writer('imu.csv', [
            'stamp_s', 'yaw_rad', 'angular_velocity_z_rad_s',
            'linear_acceleration_x_m_s2', 'linear_acceleration_y_m_s2'])
        self.scan_csv = self.output.writer('scan.csv', [
            'stamp_s', 'beam_count', 'valid_beams', 'scan_time_s',
            'time_increment_s', 'minimum_range_m'])
        self.correction_csv = self.output.writer('map_to_odom.csv', [
            'tf_stamp_s', 'receive_sim_s', 'x_m', 'y_m', 'yaw_rad'])
        self.tf_csv = self.output.writer('tf_samples.csv', [
            'stamp_s', 'receive_sim_s', 'topic', 'parent', 'child',
            'x_m', 'y_m', 'yaw_rad', 'publisher_gid'])
        self.command_csv = self.output.writer('commands.csv', [
            'topic', 'receive_sim_s', 'linear_x_m_s', 'angular_z_rad_s'])
        self.map_csv = self.output.writer('map_timing.csv', [
            'stamp_s', 'receive_sim_s', 'width', 'height', 'resolution_m'])
        self.publisher_csv = self.output.writer('tf_publishers.csv', [
            'receive_sim_s', 'topic', 'publisher_gid', 'node_namespace', 'node_name'])
        self.tf_publishers = {}
        self.tf_edge_publishers = {}
        self.parameter_names = [
            'check_min_dist_and_heading_precisely', 'use_scan_matching', 'do_loop_closing',
            'minimum_travel_distance', 'minimum_travel_heading', 'minimum_time_interval',
            'angle_variance_penalty', 'minimum_angle_penalty', 'use_scan_barycenter',
            'link_scan_maximum_distance', 'link_match_minimum_response_fine',
            'transform_timeout', 'restamp_tf', 'resolution', 'distance_variance_penalty',
            'minimum_distance_penalty', 'correlation_search_space_dimension',
            'correlation_search_space_resolution', 'correlation_search_space_smear_deviation',
            'coarse_search_angle_offset', 'coarse_angle_resolution', 'fine_search_angle_offset']
        self.parameter_client = self.node.create_client(GetParameters, '/slam_toolbox/get_parameters')
        self.parameter_future = None
        self.runtime_parameters = None
        self.residual_csv = self.output.writer('scan_map_residual.csv', [
            'scan_stamp_s', 'map_stamp_s', 'points', 'median_m', 'p95_m', 'max_m'])
        self.latest_map = None
        self.pending_scans = []
        self.last_residual_stamp = -math.inf
        self.events = self.output.writer('events.csv', ['wall_utc', 'event', 'details'])
        self.raw_gt = (args.output / 'ground_truth.jsonl').open('w', encoding='utf-8')
        self.lock = threading.Lock()
        self.poses = {}
        self.statistics = {}
        self.processes = []
        self.process_logs = []
        self.stopping = threading.Event()
        self.clock_seen = False
        self.start_sim = None
        self.last_clock = None
        self.clock_reset = False
        self.ground_truth_error = None
        self.runtime_error = None
        self.last_slam_stamp = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            self.node.create_subscription(Clock, '/clock', self.clock_callback, qos),
            self.node.create_subscription(LaserScan, '/scan', self.scan_callback, qos),
            self.node.create_subscription(Imu, '/imu', self.imu_callback, qos),
            self.node.create_subscription(Odometry, args.raw_odom,
                lambda msg: self.odom_callback('raw_odom', msg), qos),
            self.node.create_subscription(Odometry, '/odometry/filtered',
                lambda msg: self.odom_callback('filtered_odom', msg), qos),
            self.node.create_subscription(TFMessage, '/tf',
                lambda msg, info: self.tf_callback('/tf', msg, info), qos),
            self.node.create_subscription(TFMessage, '/tf_static',
                lambda msg, info: self.tf_callback('/tf_static', msg, info), static_qos),
            self.node.create_subscription(OccupancyGrid, '/map',
                self.map_callback, static_qos),
            self.node.create_subscription(PoseWithCovarianceStamped, '/pose',
                lambda msg: self.odom_callback('slam_pose', msg), qos),
        ]
        for topic in ('/cmd_vel', f'/model/{args.robot}/cmd_vel'):
            self.subscriptions.append(self.node.create_subscription(
                Twist, topic, partial(self.command_callback, topic), qos))

    def event(self, name, detail):
        self.events.writerow([datetime.now(timezone.utc).isoformat(), name, detail])

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def timing(self, stream, stamp, frame='', child=''):
        now = self.now()
        age = now - stamp if self.clock_seen else None
        with self.lock:
            self.timing_csv.writerow([stream, now, stamp, age, time.monotonic(), frame, child])
            stats = self.statistics.setdefault(stream, {
                'count': 0, 'first_stamp_s': stamp, 'last_stamp_s': stamp,
                'age_sum_s': 0., 'age_count': 0, 'max_age_s': None,
                'non_monotonic_stamps': 0})
            if stamp < stats['last_stamp_s']:
                stats['non_monotonic_stamps'] += 1
            stats['count'] += 1
            stats['last_stamp_s'] = stamp
            if age is not None:
                stats['age_sum_s'] += age
                stats['age_count'] += 1
                stats['max_age_s'] = age if stats['max_age_s'] is None else max(age, stats['max_age_s'])

    def pose(self, source, stamp, x, y, yaw):
        if yaw is None or not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            return
        with self.lock:
            self.poses.setdefault(source, []).append((stamp, x, y, yaw))
            self.pose_csv.writerow([source, stamp, self.now(), x, y, yaw])

    def clock_callback(self, msg):
        stamp = stamp_seconds(msg.clock)
        if self.last_clock is not None and stamp + 1e-6 < self.last_clock:
            self.clock_reset = True
            self.event('clock_reset', 'Stopping: do not compare samples across simulation reset')
            self.stopping.set()
        self.clock_seen = True
        if self.start_sim is None:
            self.start_sim = stamp
        self.last_clock = stamp
        self.timing('/clock', stamp)

    def odom_callback(self, source, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.timing(source, stamp, msg.header.frame_id,
                    getattr(msg, 'child_frame_id', self.args.base_frame))
        self.pose(source, stamp, msg.pose.pose.position.x,
                  msg.pose.pose.position.y, yaw_from_quaternion(msg.pose.pose.orientation))

    def imu_callback(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.timing('/imu', stamp, msg.header.frame_id)
        yaw = None if msg.orientation_covariance[0] < 0 else yaw_from_quaternion(msg.orientation)
        self.imu_csv.writerow([stamp, yaw, msg.angular_velocity.z,
                               msg.linear_acceleration.x, msg.linear_acceleration.y])
        self.pose('imu', stamp, 0., 0., yaw)

    def scan_callback(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.timing('/scan', stamp, msg.header.frame_id)
        valid = [v for v in msg.ranges
                 if math.isfinite(v) and msg.range_min <= v <= msg.range_max]
        self.scan_csv.writerow([stamp, len(msg.ranges), len(valid), msg.scan_time,
                               msg.time_increment, min(valid) if valid else None])
        self.pending_scans.append(msg)
        self.pending_scans = self.pending_scans[-20:]

    def command_callback(self, topic, msg):
        self.command_csv.writerow([topic, self.now(), msg.linear.x, msg.angular.z])

    def map_callback(self, msg):
        self.latest_map = msg
        self.timing('/map', stamp_seconds(msg.header.stamp), msg.header.frame_id)
        self.map_csv.writerow([stamp_seconds(msg.header.stamp), self.now(),
                              msg.info.width, msg.info.height, msg.info.resolution])

    def sample_publishers(self):
        from rcl_interfaces.srv import GetParameters
        if self.runtime_parameters is None:
            if self.parameter_future is None and self.parameter_client.service_is_ready():
                self.parameter_future = self.parameter_client.call_async(
                    GetParameters.Request(names=self.parameter_names))
            elif self.parameter_future is not None and self.parameter_future.done():
                from rosidl_runtime_py.convert import message_to_ordereddict
                response = self.parameter_future.result()
                if len(response.values) == len(self.parameter_names) and all(v.type != 0 for v in response.values):
                    self.runtime_parameters = dict(zip(self.parameter_names,
                        [message_to_ordereddict(v) for v in response.values]))
                    (self.args.output/'runtime_slam_parameters.json').write_text(
                        json.dumps(self.runtime_parameters, indent=2))
                else:
                    # Services exist before lifecycle configure declares mapper parameters.
                    self.parameter_future = None
        for topic in ('/tf', '/tf_static'):
            for endpoint in self.node.get_publishers_info_by_topic(topic):
                gid = bytes(endpoint.endpoint_gid).hex()
                key = topic + ':' + gid
                if key not in self.tf_publishers:
                    self.tf_publishers[key] = {'topic': topic, 'publisher_gid': gid,
                                              'node_namespace': endpoint.node_namespace,
                                              'node_name': endpoint.node_name}
                    self.publisher_csv.writerow([self.now(), topic, gid,
                                                 endpoint.node_namespace, endpoint.node_name])

    def tf_callback(self, topic, msg, info=None):
        # Jazzy can pass a mapping here; other rclpy releases expose MessageInfo.
        raw_gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
        gid = bytes(raw_gid).hex() if raw_gid is not None else 'unavailable'
        for transform in msg.transforms:
            parent, child = transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/')
            self.tf_edge_publishers.setdefault(f'{parent}->{child}', set()).add(gid)
            if (parent, child) in ((self.args.map_frame, 'odom'), ('odom', self.args.base_frame)):
                t = transform.transform
                self.tf_csv.writerow([stamp_seconds(transform.header.stamp), self.now(), topic,
                                      parent, child, t.translation.x, t.translation.y,
                                      yaw_from_quaternion(t.rotation), gid])
                if parent == 'odom':
                    self.pose('odom_tf', stamp_seconds(transform.header.stamp),
                              t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))
            self.timing(f'{topic}:{transform.header.frame_id}->{transform.child_frame_id}',
                        stamp_seconds(transform.header.stamp),
                        transform.header.frame_id, transform.child_frame_id)
            if (transform.header.frame_id.lstrip('/') == self.args.map_frame
                    and transform.child_frame_id.lstrip('/') == 'odom'):
                t = transform.transform
                self.correction_csv.writerow([
                    stamp_seconds(transform.header.stamp), self.now(),
                    t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation)])

    def sample_scan_map(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        from mapping_analysis import (nearest_distances, occupied_points, percentiles,
                                      scan_points, transform_points, transform_pose)
        if self.latest_map is None:
            return
        for scan in reversed(self.pending_scans):
            stamp = stamp_seconds(scan.header.stamp)
            if stamp-self.last_residual_stamp < 1.:
                break
            try:
                transform = self.buffer.lookup_transform(
                    self.args.map_frame, scan.header.frame_id, Time.from_msg(scan.header.stamp))
            except TransformException:
                continue
            points = transform_points(scan_points(scan, stride=3), transform_pose(transform))
            stats = percentiles(nearest_distances(points, occupied_points(self.latest_map)))
            self.residual_csv.writerow([stamp, stamp_seconds(self.latest_map.header.stamp),
                                       stats['count'], stats['median_m'], stats['p95_m'], stats['max_m']])
            self.last_residual_stamp = stamp
            break

    def sample_slam(self):
        from rclpy.time import Time
        from tf2_ros import TransformException
        try:
            transform = self.buffer.lookup_transform(
                self.args.map_frame, self.args.base_frame, Time())
        except TransformException:
            return
        stamp = stamp_seconds(transform.header.stamp)
        if stamp == self.last_slam_stamp:
            return
        self.last_slam_stamp = stamp
        self.timing('slam', stamp, self.args.map_frame, self.args.base_frame)
        self.pose('slam', stamp, transform.transform.translation.x,
                  transform.transform.translation.y,
                  yaw_from_quaternion(transform.transform.rotation))

    def start_process(self, command, name, stdout=None):
        log = (self.args.output / f'{name}.log').open('wb')
        self.process_logs.append(log)
        process = subprocess.Popen(command, stdout=stdout or log, stderr=log,
                                   start_new_session=True)
        self.processes.append((name, process))
        return process

    def ground_truth_reader(self, process):
        decoder = json.JSONDecoder()
        pending = ''
        try:
            while not self.stopping.is_set():
                data = os.read(process.stdout.fileno(), 65536)
                if not data:
                    break
                pending += data.decode('utf-8')
                while pending.strip():
                    pending = pending.lstrip()
                    try:
                        message, consumed = decoder.raw_decode(pending)
                    except json.JSONDecodeError:
                        if len(pending) > 16 * 1024 * 1024:
                            raise ValueError('Gazebo JSON record exceeded 16 MB; inspect ground_truth.log')
                        break
                    pending = pending[consumed:]
                    self.raw_gt.write(json.dumps(message, separators=(',', ':')) + '\n')
                    stamp = stamp_seconds(message.get('header', {}).get('stamp', {}))
                    matches = [p for p in message.get('pose', [])
                               if p.get('name') == self.args.robot]
                    if len(matches) > 1:
                        raise ValueError('Ambiguous model name in Gazebo truth; require a unique top-level robot')
                    if matches:
                        pose = matches[0]
                        point = pose.get('position', {})
                        self.timing('ground_truth', stamp, 'world', self.args.robot)
                        self.pose('ground_truth', stamp, float(point.get('x', 0)),
                                  float(point.get('y', 0)),
                                  yaw_from_quaternion(pose.get('orientation', {})))
        except (OSError, ValueError) as exc:
            self.ground_truth_error = str(exc)
            self.stopping.set()

    def start(self):
        topic = f'/world/{self.args.world}/dynamic_pose/info'
        process = self.start_process(
            ['gz', 'topic', '-e', '--json-output', '-t', topic],
            'ground_truth', stdout=subprocess.PIPE)
        self.gt_thread = threading.Thread(target=self.ground_truth_reader,
                                          args=(process,), daemon=True)
        self.gt_thread.start()
        if not self.args.no_bag:
            self.start_process(['ros2', 'bag', 'record', '--use-sim-time',
                                '--disable-keyboard-controls',
                                '-o', str(self.args.output / 'rosbag'),
                                '--topics', '/scan', '/imu', self.args.raw_odom, '/odometry/filtered',
                                '/tf', '/tf_static', '/clock', '/map', '/pose', '/plan',
                                '/cmd_vel', f'/model/{self.args.robot}/cmd_vel',
                                '/slam_toolbox/graph_visualization',
                                '/global_costmap/costmap', '/local_costmap/costmap',
                                '/lookahead_point', '/lookahead_collision_arc'], 'rosbag')
        self.event('started', f'read-only; Gazebo truth={topic}; scenario={self.args.scenario}')
        print(f'Recording to {self.args.output}; Ctrl+C saves all summaries.', flush=True)

    def run(self):
        started = last_slam_poll = time.monotonic()
        last_publisher_poll = -math.inf
        next_notice = started + 10
        while not self.stopping.is_set() and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
            now = time.monotonic()
            if now-last_publisher_poll >= 1.:
                self.sample_publishers()
                last_publisher_poll = now
            if now - last_slam_poll >= .1:
                self.sample_slam()
                self.sample_scan_map()
                last_slam_poll = now
            if self.args.duration > 0 and self.start_sim is not None:
                if self.now() - self.start_sim >= self.args.duration:
                    break
            if self.args.wall_timeout > 0 and now - started >= self.args.wall_timeout:
                self.event('wall_timeout', str(self.args.wall_timeout))
                break
            for name, process in self.processes:
                if process.poll() is not None:
                    raise RuntimeError(f'{name} exited with {process.returncode}; inspect {name}.log')
            if now >= next_notice:
                if not self.clock_seen or 'ground_truth' not in self.poses:
                    print('Waiting for /clock and named Gazebo truth; check world/robot names, '
                          'SceneBroadcaster, and gz discovery.', flush=True)
                next_notice = now + 30

    def compare(self):
        series = {source: normalize_samples(samples) for source, samples in self.poses.items()}
        truth = series.get('ground_truth', [])
        comparisons = self.output.writer('comparison.csv', [
            'source', 'stamp_s', 'relative_yaw_error_deg', 'aligned_position_error_m'])
        turns_csv = self.output.writer('turns.csv', [
            'turn', 'start_sim_s', 'end_sim_s', 'source', 'truth_turn_deg',
            'measured_turn_deg', 'turn_error_deg'])
        results = {}
        turns = detect_turns(truth)
        for source, samples in series.items():
            if source == 'ground_truth':
                continue
            initial = None
            yaw_errors, position_errors = [], []
            for sample in samples:
                gt = interpolate(truth, sample[0], self.args.max_sync_gap)
                if gt is None:
                    continue
                if initial is None:
                    initial = sample, gt
                first, first_gt = initial
                yaw_error = math.degrees((sample[3] - first[3]) - (gt[3] - first_gt[3]))
                # Fix only the initial SE(2) offset; subsequent drift remains visible.
                rotation = first_gt[3] - first[3]
                dx, dy = sample[1] - first[1], sample[2] - first[2]
                x = first_gt[1] + math.cos(rotation) * dx - math.sin(rotation) * dy
                y = first_gt[2] + math.sin(rotation) * dx + math.cos(rotation) * dy
                position_error = None if source == 'imu' else math.hypot(x - gt[1], y - gt[2])
                comparisons.writerow([source, sample[0], yaw_error, position_error])
                yaw_errors.append(abs(yaw_error))
                if position_error is not None:
                    position_errors.append(position_error)
            results[source] = {
                'matched_samples': len(yaw_errors),
                'max_abs_relative_yaw_error_deg': max(yaw_errors, default=None),
                'max_aligned_position_error_m': max(position_errors, default=None)}
            for number, (start, end) in enumerate(turns, 1):
                before = interpolate(samples, start[0], self.args.max_sync_gap)
                after = interpolate(samples, end[0], self.args.max_sync_gap)
                gt_angle = math.degrees(end[3] - start[3])
                measured = math.degrees(after[3] - before[3]) if before and after else None
                turns_csv.writerow([number, start[0], end[0], source, gt_angle, measured,
                                     measured - gt_angle if measured is not None else None])
        return results, len(turns)

    def close(self):
        self.stopping.set()
        for _, process in self.processes:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        for name, process in self.processes:
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
                self.event('forced_process_stop', name)
        if hasattr(self, 'gt_thread'):
            self.gt_thread.join(timeout=3)
        self.raw_gt.close()
        results, turns = self.compare()
        # Flush before slicing context windows; the files are still open here.
        for stream in self.output.files:
            stream.flush()
        from mapping_events import write_correction_diagnostics
        _, correction_report = write_correction_diagnostics(
            self.poses.get('slam_pose', []), self.poses.get('odom_tf', []),
            self.args.output, self.poses, self.args.output)
        (self.args.output/'tf_sources.json').write_text(json.dumps({
            'publishers': self.tf_publishers,
            'edge_publisher_gids': {edge: sorted(gids) for edge, gids in self.tf_edge_publishers.items()}}, indent=2))
        for stats in self.statistics.values():
            duration = stats['last_stamp_s'] - stats['first_stamp_s']
            stats['stamp_frequency_hz'] = (stats['count'] - 1) / duration if duration > 0 else None
            stats['mean_age_s'] = stats['age_sum_s'] / stats['age_count'] if stats['age_count'] else None
        missing = [name for name in ('/clock', '/scan', '/imu', 'raw_odom', 'filtered_odom',
                                     'ground_truth', 'slam', 'slam_pose', '/map') if name not in self.statistics]
        for topic in ('/tf', '/tf_static'):
            if not any(name.startswith(topic + ':') for name in self.statistics):
                missing.append(topic)
        summary = {
            'status': 'incomplete' if missing or self.clock_reset or self.ground_truth_error or self.runtime_error else 'recorded',
            'missing_streams': missing, 'clock_reset': self.clock_reset,
            'ground_truth_error': self.ground_truth_error,
            'runtime_error': self.runtime_error,
            'recorded_duration_sim_s': (self.last_clock-self.start_sim
                                      if self.start_sim is not None else 0.),
            'requested_duration_sim_s': self.args.duration,
            'streams': self.statistics, 'comparison': results, 'detected_turns': turns,
            'correction_diagnostics': correction_report,
            'notes': ['recorded is a data-availability status, not a mapping-quality pass',
                      'yaw/position errors use initial alignment, not absolute frame orientation',
                      'TF age can be negative for intentionally future-dated SLAM transforms',
                      'wall alignment / repeated-wall acceptance requires inspecting the saved map']}
        (self.args.output / 'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        self.output.close()
        for log in self.process_logs:
            log.close()
        self.node.destroy_node()
        self.rclpy.shutdown()
        print(f'Saved summary.json; status={summary["status"]}; missing={missing}', flush=True)
        return summary['status'] == 'recorded'


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default='maze', help='Gazebo <world name>, not a file path')
    parser.add_argument('--robot', default='my_robot', help='Unique top-level Gazebo model name')
    parser.add_argument('--duration', type=float, default=120., help='Simulation seconds; 0 records until Ctrl+C')
    parser.add_argument('--wall-timeout', type=float, default=0., help='Optional real seconds limit, including pauses; 0 disables')
    parser.add_argument('--output', type=Path, required=True, help='New output directory; existing paths are refused')
    parser.add_argument('--scenario', choices=('straight', 'left', 'right', 'four-turns', 'explore'), default='explore')
    parser.add_argument('--seed', default='', help='Maze seed recorded as metadata')
    parser.add_argument('--run', type=int, default=1, help='Repeat number, recorded as metadata')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--max-sync-gap', type=float, default=.15, help='Maximum interpolation distance to each bounding sample, seconds')
    parser.add_argument('--no-bag', action='store_true', help='CSV/JSON only; laser ranges and full raw ROS messages will not be retained')
    args = parser.parse_args(argv)
    if args.duration < 0 or args.wall_timeout < 0 or args.max_sync_gap <= 0 or args.run < 1:
        parser.error('durations must be nonnegative; sync gap and run number must be positive')
    args.output = args.output.expanduser().resolve()
    args.raw_odom = f'/model/{args.robot}/odometry'
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        args.output.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        print(f'Refusing to overwrite existing output: {args.output}', file=sys.stderr)
        return 2
    metadata = {key: str(value) if isinstance(value, Path) else value
                for key, value in vars(args).items()}
    metadata['started_utc'] = datetime.now(timezone.utc).isoformat()
    metadata['ground_truth_source'] = f'/world/{args.world}/dynamic_pose/info (SceneBroadcaster)'
    (args.output / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    # Keep diagnostics self-contained, including ROS subprocess logs.
    os.environ['ROS_LOG_DIR'] = str(args.output / 'ros_logs')
    recorder = None
    failed = False
    try:
        recorder = Diagnostics(args)
        signal.signal(signal.SIGINT, lambda *_: recorder.stopping.set())
        signal.signal(signal.SIGTERM, lambda *_: recorder.stopping.set())
        recorder.start()
        recorder.run()
    except Exception as exc:
        failed = True
        if recorder is not None:
            recorder.runtime_error = str(exc)
        print(f'Diagnostics failed: {exc}', file=sys.stderr)
        (args.output / 'error.txt').write_text(str(exc) + '\n', encoding='utf-8')
    finally:
        if recorder is not None:
            failed = not recorder.close() or failed
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
