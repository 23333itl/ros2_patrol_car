#!/usr/bin/env python3
"""Exercise real bag filtering and drift measurements without DDS or Gazebo."""
import csv
import json
import math
import signal
import subprocess
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import rosbag2_py
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from mapping_analysis import (compare_trajectory, compose, inverse, transform_points,
                              wall_measurements, wall_direction_timeline)
from replay_mapping import bag_reader, prepare, publish_tf_until, stop_owned_process
from record_mapping_diagnostics import Diagnostics
from mapping_events import (correction_steps, write_correction_diagnostics,
                            correction_statistics, correction_gate, data_health)


def edge(parent, child):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.
    return transform


class MappingReplayTest(unittest.TestCase):
    def test_replay_cleanup_reaps_owned_slam_even_if_it_ignores_soft_signals(self):
        from unittest.mock import Mock, patch, call
        process = Mock(pid=1234)
        process.poll.return_value = None
        process.wait.side_effect = [subprocess.TimeoutExpired('slam', 8),
                                   subprocess.TimeoutExpired('slam', 5), 0]
        with patch('replay_mapping.os.killpg') as kill:
            stop_owned_process(process)
        self.assertEqual(kill.call_args_list, [call(1234, signal.SIGINT),
                         call(1234, signal.SIGTERM), call(1234, signal.SIGKILL)])
        self.assertEqual(process.wait.call_count, 3)
        process.poll.return_value = 0
        with patch('replay_mapping.os.killpg') as kill:
            stop_owned_process(process)
        kill.assert_not_called()

    def test_tf_preroll_keeps_original_messages_and_does_not_repeat_or_extrapolate(self):
        first, second = TFMessage(transforms=[edge('odom', 'base_footprint')]), TFMessage(transforms=[edge('odom', 'base_footprint')])
        samples, sent = [(1., first), (1.1, second)], []
        index = publish_tf_until(samples, 0, 1.05, sent.append)
        self.assertEqual(index, 1)
        self.assertIs(sent[0], first)
        self.assertEqual(publish_tf_until(samples, index, 1.05, sent.append), 1)
        index = publish_tf_until(samples, index, 1.2, sent.append)
        self.assertEqual(index, 2)
        self.assertIs(sent[1], second)
        self.assertEqual(first.transforms[0].header.stamp.sec, 0)
        self.assertEqual(len(sent), 2)

    def test_recording_stop_uses_same_clock_as_reported_duration(self):
        from types import SimpleNamespace
        from unittest.mock import Mock
        recorder = Diagnostics.__new__(Diagnostics)
        recorder.args = SimpleNamespace(duration=600., wall_timeout=0.)
        recorder.start_sim, recorder.last_clock = 0., 0.
        recorder.stopping = SimpleNamespace(is_set=lambda: False)
        ticks = iter((599.999, 600.001))
        def spin(*_args, **_kwargs):
            recorder.last_clock = next(ticks)
        recorder.rclpy = SimpleNamespace(ok=lambda: True, spin_once=Mock(side_effect=spin))
        recorder.node = object()
        recorder.now = lambda: 601.  # The node TimeSource is ahead of our callback.
        recorder.sample_publishers = recorder.sample_slam = recorder.sample_scan_map = lambda: None
        recorder.processes = []
        recorder.run()
        self.assertEqual(recorder.rclpy.spin_once.call_count, 2)
        self.assertGreaterEqual(recorder.last_clock-recorder.start_sim, 600.)

    def test_data_health_distinguishes_input_gaps_from_skipped_processing(self):
        self.assertTrue(data_health([0., .1, .2])['complete'])
        self.assertFalse(data_health([0., .2])['complete'])
        warning = '[controller_server-1] [WARN] [{}] [controller]: Control loop missed its desired rate'
        self.assertTrue(data_health([0., .1], warning.format('100.1'))['complete'])
        log = '\n'.join(warning.format(str(t)) for t in (100.1, 101.2, 102.3))
        self.assertFalse(data_health([0., .1], log)['complete'])
        self.assertFalse(data_health([0., .1], 'Message Filter dropping message')['complete'])

    def test_motion_statistics_include_scan_free_time_and_separate_revisit(self):
        odom = []
        for index in range(501):
            t = index/10
            x = 0. if t <= 5 else (min(2., (t-5)*.2) if t <= 25 else max(0., 2.-(t-25)*.1))
            yaw = max(0., min(1., (t-15)*.1))
            odom.append((t, x, 0., yaw))
        steps = [dict(stamp_s=t, position_step_m=.01, yaw_step_deg=.1)
                 for t in (2., 10., 20., 35., 46.)]
        stats = correction_statistics(steps, odom)
        self.assertEqual([r['motion'] for r in steps],
                         ['stationary', 'straight', 'turn', 'straight', 'stationary'])
        self.assertTrue(steps[-1]['revisit'])
        self.assertAlmostEqual(stats['all']['duration_sim_s'], 50.)
        self.assertAlmostEqual(stats['all']['position_m_per_sim_s'], .05/50.)
        self.assertAlmostEqual(stats['turn']['position_p95_m'], .01)
        self.assertGreater(stats['revisit']['duration_sim_s'], 0.)
        fewer = correction_statistics(steps[:1], odom)
        self.assertEqual(stats['all']['duration_sim_s'], fewer['all']['duration_sim_s'])
        self.assertIsNone(fewer['turn']['position_p95_m'])

    @staticmethod
    def gate_report(profile, position=.01, yaw=.1, cumulative=.01):
        return {'profile': profile, 'input_sha256': 'same-input', 'acceptance': 'pass',
                'playback': {'playback_duration_s': 601.},
                'correction_diagnostics': {'correction_jumps': [], 'missing_odom_brackets': 0,
                    'by_motion': {'moving': {'position_p95_m': position, 'yaw_p95_deg': yaw},
                        'all': {'position_m_per_sim_s': cumulative, 'yaw_deg_per_sim_s': cumulative},
                        'turn': {'duration_sim_s': 10.}, 'revisit': {'duration_sim_s': 10.}}}}

    def test_prior_gate_requires_relative_gain_and_bounded_cumulative_correction(self):
        base = self.gate_report('simulation_stable')
        candidate = self.gate_report('stable_pose_prior', position=.007, yaw=.105, cumulative=.0105)
        self.assertEqual(correction_gate(base, candidate)['acceptance'], 'pass')
        for changes in ({'position': .0071}, {'yaw': .106}, {'cumulative': .0106}):
            with self.subTest(changes=changes):
                values = dict(position=.007, yaw=.1, cumulative=.01)
                values.update(changes)
                self.assertEqual(correction_gate(base, self.gate_report('stable_pose_prior', **values))['acceptance'], 'fail')

    def test_prior_gate_rejects_diagnostic_winners_missing_data_or_different_inputs(self):
        base = self.gate_report('simulation_stable')
        for alteration in ('diagnostic', 'hash', 'short', 'missing', 'jump', 'no_revisit'):
            candidate = self.gate_report('stable_translation_prior', position=.006)
            if alteration == 'diagnostic': candidate['profile'] = 'stable_no_match_control'
            if alteration == 'hash': candidate['input_sha256'] = 'different'
            if alteration == 'short': candidate['playback']['playback_duration_s'] = 599.
            if alteration == 'missing': candidate['acceptance'] = 'incomplete'
            if alteration == 'jump': candidate['correction_diagnostics']['correction_jumps'] = [{'stamp_s': 40}]
            if alteration == 'no_revisit': candidate['correction_diagnostics']['by_motion']['revisit']['duration_sim_s'] = 0.
            with self.subTest(alteration=alteration):
                self.assertEqual(correction_gate(base, candidate)['acceptance'], 'fail')
        base['acceptance'] = 'fail'  # A complete but distorted baseline must remain comparable.
        base['correction_diagnostics']['correction_jumps'] = [{'stamp_s': 10.}]
        self.assertEqual(correction_gate(base, self.gate_report('stable_pose_prior', position=.006))['acceptance'], 'pass')

    def test_incremental_wall_fit_matches_full_fit_and_locates_crossing(self):
        segments = np.array([[[0., 0.], [2., 0.]], [[0., 1.], [2., 1.]]])
        x = np.linspace(.2, 1.8, 80)
        slope = math.tan(math.radians(1.5))
        a = np.column_stack((x, slope*(x-1)))
        b = np.column_stack((x, 1-slope*(x-1)))
        rows = wall_direction_timeline([(0., a), (10., b)], segments)
        final = wall_measurements(np.concatenate((a,b)), segments)
        self.assertAlmostEqual(rows[-1]['parallel_direction_spread_deg'], final['parallel_direction_spread_deg'])
        self.assertAlmostEqual(rows[-1]['max_direction_error_deg'], final['max_direction_error_deg'])
        crossing = next(r for r in rows if (r['parallel_direction_spread_deg'] or 0) > 2)
        self.assertEqual(crossing['stamp_s'], 10.)

    def test_tf_publisher_gid_accepts_jazzy_mapping_and_message_info(self):
        from types import SimpleNamespace
        recorder = Diagnostics.__new__(Diagnostics)
        recorder.args = SimpleNamespace(map_frame='map', base_frame='base_footprint')
        recorder.now = lambda: 10.
        recorder.timing = lambda *args: None
        recorder.pose = lambda *args: None
        recorder.tf_edge_publishers = {}
        rows = []
        recorder.tf_csv = SimpleNamespace(writerow=rows.append)
        recorder.correction_csv = SimpleNamespace(writerow=lambda row: None)
        message = TFMessage(transforms=[edge('map', 'odom'), edge('odom', 'base_footprint')])
        recorder.tf_callback('/tf', message, {'publisher_gid': [1, 2, 3]})
        recorder.tf_callback('/tf', message, SimpleNamespace(publisher_gid=bytes([4, 5, 6])))
        self.assertEqual(recorder.tf_edge_publishers['map->odom'], {'010203', '040506'})
        self.assertEqual(rows[0][-1], '010203')

    def test_correction_jump_independent_of_map_and_odom_origins(self):
        odom = [(0., 100., 60., .3), (1., 100.1, 60., .3)]
        first, second = (0., 0., 0.), (0., 0., .02)
        poses = [(0., *compose(first, odom[0][1:])), (1., *compose(second, odom[1][1:]))]
        _, steps, missing = correction_steps(poses, odom)
        self.assertEqual(missing, 0)
        self.assertGreater(steps[0]['position_step_m'], .1)
        self.assertTrue(steps[0]['jump'])
        a, b = (300., -400., 1.3), (-99., -61., -.7)
        shifted_poses = [(p[0], *compose(a, p[1:])) for p in poses]
        shifted_odom = [(p[0], *compose(b, p[1:])) for p in odom]
        _, shifted, _ = correction_steps(shifted_poses, shifted_odom)
        self.assertAlmostEqual(steps[0]['position_step_m'], shifted[0]['position_step_m'])
        self.assertAlmostEqual(steps[0]['yaw_step_deg'], shifted[0]['yaw_step_deg'])

    def test_large_transform_translation_can_leave_robot_in_place(self):
        odom = [(0., 100., 0., 0.), (1., 100., 0., 0.)]
        rotation = .01
        correction = (100.-100.*math.cos(rotation), -100.*math.sin(rotation), rotation)
        poses = [(0., 100., 0., 0.), (1., *compose(correction, odom[1][1:]))]
        _, steps, _ = correction_steps(poses, odom)
        self.assertLess(steps[0]['position_step_m'], 1e-9)
        self.assertFalse(steps[0]['jump'])

    def test_correction_yaw_wrap_does_not_create_jump(self):
        odom = [(0., 0., 0., 0.), (1., 0., 0., 0.)]
        _, steps, _ = correction_steps([(0., 0., 0., 3.14), (1., 0., 0., -3.14)], odom)
        self.assertLess(abs(steps[0]['yaw_step_deg']), .2)
        self.assertFalse(steps[0]['jump'])

    def test_missing_tf_bracket_breaks_correction_chain(self):
        poses = [(0., 0., 0., 0.), (1., 50., 0., 1.), (2., 1., 0., 0.)]
        _, steps, missing = correction_steps(poses, [(0., 0., 0., 0.), (2., 1., 0., 0.)])
        self.assertEqual(missing, 1)
        self.assertFalse(steps)

    def test_jump_exports_five_second_context_and_plot(self):
        with tempfile.TemporaryDirectory() as path:
            output = Path(path)
            poses = [(0., 0., 0., 0.), (1., .2, 0., 0.)]
            _, report = write_correction_diagnostics(poses, [(0., 0., 0., 0.), (1., 0., 0., 0.)], output)
            self.assertEqual(len(report['correction_jumps']), 1)
            event = next((output/'jump_events').iterdir())
            self.assertEqual(json.loads((event/'event.json').read_text())['window_sim_s'], [-4., 6.])
            self.assertTrue((event/'poses.png').stat().st_size > 1000)

    def test_jump_exports_actual_scan_ranges_and_marks_recorded_tf_provenance(self):
        with tempfile.TemporaryDirectory() as path:
            root = Path(path)
            self.write_bag(root/'recording/rosbag')
            poses = [(0., 0., 0., 0.), (1., .2, 0., 0.)]
            write_correction_diagnostics(poses, [(0., 0., 0., 0.), (1., 0., 0., 0.)],
                                         root/'report', context_directory=root/'recording')
            event = next((root/'report/jump_events').iterdir())
            scans = [json.loads(line) for line in (event/'scan.jsonl').read_text().splitlines()]
            self.assertEqual(len(scans), 1)
            self.assertIn('ranges', scans[0])
            self.assertTrue(json.loads((event/'tf_static.json').read_text()))
            self.assertIn('original recording', json.loads((event/'event.json').read_text())['context_tf_note'])

    def test_scan_stamped_slam_pose_has_no_child_frame_field(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from types import SimpleNamespace
        recorder = Diagnostics.__new__(Diagnostics)
        recorder.args = SimpleNamespace(base_frame='base_footprint')
        timed, poses = [], []
        recorder.timing = lambda *args: timed.append(args)
        recorder.pose = lambda *args: poses.append(args)
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp.sec = 4
        message.pose.pose.orientation.w = 1.
        recorder.odom_callback('slam_pose', message)
        self.assertEqual(timed[0], ('slam_pose', 4., 'map', 'base_footprint'))
        self.assertEqual(poses[0], ('slam_pose', 4., 0., 0., 0.))

    def write_bag(self, directory, reset=False, late_scan=False):
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {'/tf': 'tf2_msgs/msg/TFMessage', '/tf_static': 'tf2_msgs/msg/TFMessage',
                 '/scan': 'sensor_msgs/msg/LaserScan', '/clock': 'rosgraph_msgs/msg/Clock',
                 '/map': 'nav_msgs/msg/OccupancyGrid', '/cmd_vel': 'geometry_msgs/msg/Twist'}
        for name, message_type in types.items():
            writer.create_topic(rosbag2_py.TopicMetadata(id=0, name=name, type=message_type,
                                                        serialization_format='cdr'))
        writer.write('/tf', serialize_message(TFMessage(transforms=[
            edge('map', 'odom'), edge('odom', 'base_footprint'), edge('world', 'truth')])), 0)
        writer.write('/tf_static', serialize_message(TFMessage(transforms=[
            edge('base_footprint', 'base_link'), edge('base_link', 'laser_link'),
            edge('map', 'fake_static'), edge('world', 'truth')])), 1)
        writer.write('/scan', serialize_message(LaserScan()), 2)
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import Twist
        writer.write('/map', serialize_message(OccupancyGrid()), 3)
        writer.write('/cmd_vel', serialize_message(Twist()), 4)
        for stamp, seconds in ((5, 2), (6, 1 if reset else 3)):
            clock = Clock()
            clock.clock.sec = seconds
            writer.write('/clock', serialize_message(clock), stamp)
        if late_scan:
            scan = LaserScan()
            scan.header.stamp.sec = 4
            writer.write('/scan', serialize_message(scan), 7)
        del writer

    def test_bag_filters_edges_and_excludes_outputs_and_commands(self):
        with tempfile.TemporaryDirectory() as path:
            source, destination = Path(path)/'original', Path(path)/'filtered'
            self.write_bag(source)
            report = prepare(source, destination)
            self.assertEqual(report['removed_tf_edges'], {'/tf': 2, '/tf_static': 2})
            reader = bag_reader(destination)
            names = {t.name for t in reader.get_all_topics_and_types()}
            self.assertNotIn('/map', names)
            self.assertNotIn('/cmd_vel', names)
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic == '/tf':
                    transforms = deserialize_message(data, TFMessage).transforms
                    self.assertEqual([(t.header.frame_id, t.child_frame_id) for t in transforms],
                                     [('odom', 'base_footprint')])
            self.assertFalse(report['truth_published'])

    def test_clock_reset_rejected(self):
        with tempfile.TemporaryDirectory() as path:
            source = Path(path)/'original'
            self.write_bag(source, reset=True)
            with self.assertRaisesRegex(ValueError, 'clock reset'):
                prepare(source, Path(path)/'filtered')

    def test_existing_output_refused(self):
        with tempfile.TemporaryDirectory() as path:
            with self.assertRaisesRegex(ValueError, 'overwrite'):
                prepare(Path(path)/'missing', Path(path))

    def test_scan_without_recorded_odometry_coverage_is_reported_and_excluded(self):
        with tempfile.TemporaryDirectory() as path:
            source = Path(path)/'original'
            self.write_bag(source, late_scan=True)
            report = prepare(source, Path(path)/'filtered')
            self.assertEqual(report['excluded_scans_outside_odom_tf'], [4.])
            self.assertEqual(report['topics']['/scan'], 1)

    def test_fixed_alignment_preserves_late_drift(self):
        with tempfile.TemporaryDirectory() as path:
            truth = [(0., 5., 7., math.pi/2), (1., 5., 8., math.pi/2),
                     (2., 5., 9., math.pi/2)]
            samples = [(0., 0., 0., 0.), (1., 1., 0., 0.), (2., 2., .2, .1)]
            _, result = compare_trajectory(samples, truth, Path(path)/'comparison.csv')
            self.assertAlmostEqual(result['max_yaw_error_deg'], math.degrees(.1))
            self.assertAlmostEqual(result['max_position_error_m'], .2)
            self.assertEqual(result['first_threshold_crossing']['stamp_s'], 2.)

    def test_angle_wrap_is_not_a_360_degree_error(self):
        with tempfile.TemporaryDirectory() as path:
            truth = [(0., 0., 0., 3.13), (1., 0., 0., 3.15)]
            samples = [(0., 0., 0., 3.13), (1., 0., 0., 3.15-2*math.pi)]
            _, result = compare_trajectory(samples, truth, Path(path)/'comparison.csv')
            self.assertLess(result['max_yaw_error_deg'], 1e-10)

    def test_revisit_requires_separate_observations(self):
        segments = np.array([[[0., 0.], [2., 0.]]])
        x = np.linspace(.2, 1.8, 80)
        first = np.column_stack((x, np.zeros_like(x)))
        later = np.column_stack((x, np.full_like(x, .12)))
        continuous = wall_measurements(first, segments, np.linspace(0., 5., len(x)))
        self.assertIsNone(continuous['max_revisit_displacement_m'])
        revisited = wall_measurements(np.concatenate((first, later)), segments,
                                     np.concatenate((np.linspace(0., 5., len(x)),
                                                     np.linspace(30., 35., len(x)))))
        self.assertAlmostEqual(revisited['max_revisit_displacement_m'], .12)

    def test_far_points_remain_in_quality_report(self):
        segments = np.array([[[0., 0.], [2., 0.]]])
        points = np.column_stack((np.linspace(.2, 1.8, 80), np.full(80, .3)))
        result = wall_measurements(points, segments)
        self.assertEqual(result['unmatched_fraction'], 1.)
        self.assertAlmostEqual(result['residual']['p95_m'], .3)
        self.assertIsNone(result['max_direction_error_deg'])

    def test_transform_composition_rotates_translation(self):
        a, b = (2., 1., math.pi/2), (1., 0., -.3)
        actual = compose(a, b)
        self.assertTrue(np.allclose(actual[:2], (2., 2.)))
        self.assertTrue(np.allclose(compose(a, inverse(a)), (0., 0., 0.)))
        self.assertTrue(np.allclose(transform_points([[1., 0.]], a), [[2., 2.]]))

    def test_parallel_walls_can_disagree_despite_small_individual_errors(self):
        segments = np.array([[[0., 0.], [2., 0.]], [[0., 1.], [2., 1.]]])
        x = np.linspace(.2, 1.8, 80)
        slope = math.tan(math.radians(1.5))
        points = np.concatenate((np.column_stack((x, slope*(x-1))),
                                 np.column_stack((x, 1-slope*(x-1)))))
        result = wall_measurements(points, segments)
        self.assertLess(result['max_direction_error_deg'], 2.)
        self.assertAlmostEqual(result['parallel_direction_spread_deg'], 3., places=5)


if __name__ == '__main__':
    unittest.main()
