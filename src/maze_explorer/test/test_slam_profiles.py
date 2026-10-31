#!/usr/bin/env python3
"""Check resolved profile ablations and the launch -> xacro -> lidar path."""
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import yaml
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('explore_launch', ROOT/'launch/explore_maze.launch.py')
launch_file = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch_file)


class SlamProfilesTest(unittest.TestCase):
    def test_controlled_profiles_change_only_requested_parameters(self):
        profiles = yaml.safe_load((ROOT/'config/slam_profiles.yaml').read_text())
        stable = profiles['simulation_stable']
        expected = {
            'stable_no_match_control': {'use_scan_matching': False},
            'stable_translation_prior': {'distance_variance_penalty': .03},
            'stable_pose_prior': {'distance_variance_penalty': .03, 'angle_variance_penalty': .003},
        }
        for name, changes in expected.items():
            with self.subTest(name=name):
                self.assertEqual(profiles[name], {**stable, **changes})
        self.assertEqual(stable['resolution'], .025)
        self.assertEqual(stable['minimum_travel_distance'], .02)
        self.assertEqual(stable['minimum_travel_heading'], .02)
        self.assertTrue(stable['use_scan_matching'])
        self.assertFalse(stable['do_loop_closing'])
        self.assertEqual(stable['distance_variance_penalty'], .05)
        self.assertEqual(stable['angle_variance_penalty'], .005)
        self.assertEqual(profiles['odometry_only'], {'use_scan_matching': False, 'do_loop_closing': False})

    def test_invalid_noise_rejected_before_starting_nodes(self):
        for value in ('-0.1', 'nan', 'inf', '-inf', 'bad', '', '0; exit 1'):
            with self.subTest(value=value), self.assertRaises(ValueError):
                context = LaunchContext()
                context.launch_configurations['lidar_noise_stddev'] = value
                launch_file._validate_lidar_noise(context)

    def test_launch_defaults_and_actual_xacro_noise_wiring(self):
        commands = []
        command_type = launch_file.Command
        def capture(value):
            command = command_type(value)
            commands.append(command)
            return command
        with patch.object(launch_file, 'Command', side_effect=capture):
            description = launch_file.generate_launch_description()
        declarations = {a.name: a for a in description.entities
                        if isinstance(a, launch_file.DeclareLaunchArgument)}
        context = LaunchContext()
        for declaration in declarations.values():
            declaration.execute(context)
        self.assertEqual(context.launch_configurations['slam_profile'], 'odometry_only')
        self.assertEqual(Path(context.launch_configurations['world']).resolve(),
                         ROOT.parents[1]/'maze_output/maze.world')
        base = yaml.safe_load((ROOT/'config/slam_toolbox.yaml').read_text())['slam_toolbox']['ros__parameters']
        profiles = yaml.safe_load((ROOT/'config/slam_profiles.yaml').read_text())
        base.update(profiles[context.launch_configurations['slam_profile']])
        self.assertFalse(base['use_scan_matching'])
        self.assertFalse(base['do_loop_closing'])
        self.assertEqual(base['resolution'], .05)  # Preserve the user's explicit odometry_only command.
        self.assertEqual(context.launch_configurations['lidar_noise_stddev'], '0.01')
        documents = []
        for value in ('0.01', '0', '0.02'):
            context.launch_configurations['lidar_noise_stddev'] = value
            for action in launch_file._validate_lidar_noise(context):
                action.execute(context)
            robot = ET.fromstring(commands[0].perform(context))
            node = robot.find(".//sensor[@name='lidar']/lidar/noise/stddev")
            self.assertTrue(math.isclose(float(node.text), float(value), abs_tol=1e-12))
            node.text = 'noise-control'
            documents.append(ET.tostring(robot))
        self.assertEqual(documents[0], documents[1])
        self.assertEqual(documents[0], documents[2])


if __name__ == '__main__':
    unittest.main()
