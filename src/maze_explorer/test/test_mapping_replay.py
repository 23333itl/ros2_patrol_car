#!/usr/bin/env python3
"""Exercise real bag filtering and drift measurements without DDS or Gazebo."""
import csv
import json
import math
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
from replay_mapping import bag_reader, prepare
from record_mapping_diagnostics import Diagnostics
from mapping_events import correction_steps, write_correction_diagnostics


def edge(parent, child):
    transform = TransformStamped()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.rotation.w = 1.
    return transform


class MappingReplayTest(unittest.TestCase):
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
