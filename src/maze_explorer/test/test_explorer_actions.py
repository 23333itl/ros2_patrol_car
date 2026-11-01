#!/usr/bin/env python3
"""Exercise the installed C++ explorer against deterministic fake Nav2 servers.

Build and source the workspace, then run:
  /usr/bin/python3 src/maze_explorer/test/test_explorer_actions.py
  /usr/bin/python3 src/maze_explorer/test/test_explorer_actions.py --scenario cancel

No Gazebo or other ROS nodes are required. Each case uses its own ROS domain,
temporary map outputs, and an accelerated /clock; no robot commands are published.
"""

import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


OPTIONS = None


class FakeNav2(Node):
    """Keep maps and TF fresh while recording every action transition."""

    def __init__(self, context, clock_rate, plan_succeeds=True):
        super().__init__('fake_nav2', context=context)
        self.group = ReentrantCallbackGroup()
        self.lock = threading.RLock()
        self.stopping = threading.Event()
        self.release_navigation = threading.Event()
        self.cancel_received = threading.Event()
        self.clock_rate = clock_rate
        self.epoch = time.monotonic()
        self.plan_succeeds = plan_succeeds
        self.plan_only_first = False
        self.unsafe_plan = False
        self.navigation_succeeds = False
        self.large_room = False
        self.far_plans_only = False
        self.planned_positions = []
        self.publish_maps = True
        self.publish_scans = True
        self.scan_span = 2.0 * math.pi
        self.has_frontiers = True
        self.robot_cell_free = True
        self.robot_x = self.robot_y = 0.0
        self.rounded_endgame = False
        self.blocked_prefix_for_frontiers = False
        self.residual_speck = False
        self.scan_frame = 'base_footprint'
        self.scan_time_offset = 0.0
        self.move_on_success = False
        self.lifecycle_requests = 0
        self.counts = dict(plan=0, navigation=0, backup=0, spin=0)
        self.active_motion = 0
        self.max_active_motion = 0
        self.events = []
        self.requests = dict(backup=[], spin=[], navigation=[])
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.tf = TransformBroadcaster(self)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.lifecycle_services = [self.create_service(
            GetState, '/' + name + '/get_state', self.lifecycle,
            callback_group=self.group)
            for name in ('bt_navigator', 'planner_server', 'controller_server',
                         'behavior_server', 'velocity_smoother')]
        self.servers = [
            ActionServer(self, ComputePathToPose, 'compute_path_to_pose',
                         self.plan, callback_group=self.group,
                         goal_callback=self.accept_goal, cancel_callback=self.accept_cancel),
            ActionServer(self, NavigateToPose, 'navigate_to_pose',
                         self.navigate, callback_group=self.group,
                         goal_callback=self.accept_goal, cancel_callback=self.cancel_navigation),
            ActionServer(self, BackUp, 'backup', self.backup,
                         callback_group=self.group, goal_callback=self.accept_goal,
                         cancel_callback=self.accept_cancel),
            ActionServer(self, Spin, 'spin', self.spin,
                         callback_group=self.group, goal_callback=self.accept_goal,
                         cancel_callback=self.accept_cancel),
        ]
        self.timer = self.create_timer(0.05, self.publish_environment,
                                      callback_group=self.group)

    def stamp(self):
        value = 100.0 + (time.monotonic() - self.epoch) * self.clock_rate
        msg = Clock().clock
        msg.sec = int(value)
        msg.nanosec = int((value - int(value)) * 1e9)
        return msg

    def lifecycle(self, request, response):
        del request
        with self.lock:
            self.lifecycle_requests += 1
        response.current_state.id = State.PRIMARY_STATE_ACTIVE
        response.current_state.label = 'active'
        return response

    @staticmethod
    def accept_goal(request):
        del request
        return GoalResponse.ACCEPT

    @staticmethod
    def accept_cancel(handle):
        del handle
        return CancelResponse.ACCEPT

    def cancel_navigation(self, handle):
        del handle
        with self.lock:
            self.events.append(('navigation_cancel_accepted', time.monotonic()))
        self.cancel_received.set()
        return CancelResponse.ACCEPT

    def publish_environment(self):
        stamp = self.stamp()
        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)
        transforms = []
        for parent, child in [('map', 'odom'), ('odom', 'base_footprint')]:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.rotation.w = 1.0
            if child == 'base_footprint':
                transform.transform.translation.x = self.robot_x
                transform.transform.translation.y = self.robot_y
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        if self.publish_scans:
            scan = LaserScan()
            scan.header.stamp = stamp
            value = stamp.sec + stamp.nanosec * 1e-9 + self.scan_time_offset
            scan.header.stamp.sec = int(value)
            scan.header.stamp.nanosec = int((value - int(value)) * 1e9)
            scan.header.frame_id = self.scan_frame
            scan.angle_min = -self.scan_span / 2.0
            scan.angle_max = self.scan_span / 2.0
            scan.angle_increment = self.scan_span / 359.0
            scan.range_min = 0.05
            scan.range_max = 8.0
            scan.ranges = [2.0] * 360
            self.scan_pub.publish(scan)
        if self.publish_maps:
            self.publish_map()

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.stamp()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.1
        grid.info.width = grid.info.height = 20
        grid.info.origin.position.x = grid.info.origin.position.y = -1.0
        grid.info.origin.orientation.w = 1.0
        data = [100] * 400
        # A known room, with enough clearance behind the fixed robot for BackUp.
        for y in range(6, 14):
            for x in range(4, 14):
                data[y * 20 + x] = 0
        if self.has_frontiers:
            # One reachable frontier cluster with several safe standpoints.
            for y in range(8, 12):
                for x in range(14, 19):
                    data[y * 20 + x] = -1
        if not self.robot_cell_free:
            data[10 * 20 + 10] = -1  # Robot at world (0, 0) is outside known free space.
        if self.residual_speck:
            data[14 * 20 + 14] = -1  # Diagonal-only contact through two occupied corners.
        grid.data = data
        if self.large_room:
            grid.info.resolution = 0.05
            grid.info.width = grid.info.height = 30
            grid.info.origin.position.x = -0.375
            grid.info.origin.position.y = -0.775
            data = [-1] * 900
            for y in range(3, 27):
                for x in range(3, 27):
                    data[y * 30 + x] = 0
            grid.data = data
        if self.rounded_endgame:
            grid.info.resolution = 0.025
            grid.info.width = grid.info.height = 40
            grid.info.origin.position.x = grid.info.origin.position.y = 0.0
            data = [100] * 1600
            for y in range(4, 36):
                for x in range(5, 34):
                    data[y * 40 + x] = 0
            if self.has_frontiers:
                for y in range(10, 26):
                    for x in range(34, 40):
                        data[y * 40 + x] = -1
            grid.data = data
        self.map_pub.publish(grid)

    def begin(self, kind, request=None):
        with self.lock:
            self.counts[kind] += 1
            self.events.append((kind + '_started', time.monotonic()))
            if kind != 'plan':
                self.active_motion += 1
                self.max_active_motion = max(self.max_active_motion, self.active_motion)
            if request is not None:
                self.requests[kind].append(request)

    def end(self, kind):
        with self.lock:
            self.events.append((kind + '_terminal', time.monotonic()))
            if kind != 'plan':
                self.active_motion -= 1

    def plan(self, handle):
        self.begin('plan')
        goal = handle.request.goal.pose.position
        self.planned_positions.append((goal.x, goal.y))
        result = ComputePathToPose.Result()
        if (self.plan_succeeds and (not self.plan_only_first or self.counts['plan'] == 1)
                and (not self.far_plans_only or goal.x > 0.6)):
            result.path.header = handle.request.goal.header
            start = PoseStamped()
            start.header = handle.request.goal.header
            start.pose.orientation.w = 1.0
            start.pose.position.x = self.robot_x
            start.pose.position.y = self.robot_y
            result.path.poses = [start, handle.request.goal]
            if self.rounded_endgame:
                start.pose.position.x = .30
                start.pose.position.y = .225
                join = PoseStamped()
                join.header = start.header
                join.pose.position.x, join.pose.position.y = .325, .25
                join.pose.orientation.w = 1.0
                result.path.poses.insert(1, join)
                if self.blocked_prefix_for_frontiers and goal.x > .55:
                    # Reconnection would require skipping more than 5 cm.
                    join.pose.position.x, join.pose.position.y = .36, .225
            if self.unsafe_plan:
                # The goal is safe, but this route crosses the room's occupied wall.
                detour = PoseStamped()
                detour.header = start.header
                detour.pose.position.y = 0.8
                detour.pose.orientation.w = 1.0
                result.path.poses.insert(1, detour)
            handle.succeed()
        else:
            result.error_code = ComputePathToPose.Result.NO_VALID_PATH
            handle.abort()
        self.end('plan')
        return result

    def navigate(self, handle):
        self.begin('navigation', handle.request)
        result = NavigateToPose.Result()
        while not self.stopping.wait(0.02):
            if handle.is_cancel_requested and self.release_navigation.is_set():
                handle.canceled()
                self.end('navigation')
                return result
            if self.navigation_succeeds and not handle.is_cancel_requested:
                if self.move_on_success:
                    self.robot_x = handle.request.pose.pose.position.x
                    self.robot_y = handle.request.pose.pose.position.y
                    self.stopping.wait(.10)  # Allow the new pose to reach the TF listener.
                handle.succeed()
                self.end('navigation')
                return result
        handle.abort()
        self.end('navigation')
        return result

    def backup(self, handle):
        return self.recover('backup', BackUp, handle)

    def spin(self, handle):
        return self.recover('spin', Spin, handle)

    def recover(self, kind, action, handle):
        self.begin(kind, handle.request)
        self.stopping.wait(0.1)
        result = action.Result()
        if handle.is_cancel_requested:
            handle.canceled()
        elif self.stopping.is_set():
            handle.abort()
        else:
            handle.succeed()
        self.end(kind)
        return result


class ExplorerHarness:
    def __init__(self, case_number, plan_succeeds=True, frontiers=True, stream_maps=True,
                 stream_scans=True, robot_cell_free=True, no_progress_timeout=2.0,
                 rounded_endgame=False, residual_speck=False, scan_frame='base_footprint',
                 scan_time_offset=0.0, blocked_prefix=False):
        self.directory = tempfile.TemporaryDirectory(prefix='maze-explorer-actions-')
        self.root = Path(self.directory.name)
        self.output = self.root / 'map'
        self.log_path = self.root / 'explorer.log'
        self.context = Context()
        self.domain = (OPTIONS.domain_id + case_number) % 232
        rclpy.init(context=self.context, domain_id=self.domain)
        self.fake = FakeNav2(self.context, OPTIONS.clock_rate, plan_succeeds)
        self.fake.rounded_endgame = rounded_endgame
        self.fake.blocked_prefix_for_frontiers = blocked_prefix
        if rounded_endgame:
            self.fake.robot_x, self.fake.robot_y = .30, .245
        self.fake.residual_speck = residual_speck
        self.fake.scan_frame = scan_frame
        self.fake.scan_time_offset = scan_time_offset
        self.fake.has_frontiers = frontiers
        self.fake.publish_maps = stream_maps
        self.fake.publish_scans = stream_scans
        self.fake.robot_cell_free = robot_cell_free
        self.executor = MultiThreadedExecutor(num_threads=8, context=self.context)
        self.executor.add_node(self.fake)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.log_file = self.log_path.open('w', encoding='utf-8')
        env = dict(os.environ, ROS_DOMAIN_ID=str(self.domain),
                   ROS_LOG_DIR=str(self.root / 'ros_logs'), ROS_LOCALHOST_ONLY='1')
        self.process = subprocess.Popen([
            str(OPTIONS.explorer_binary), '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'no_progress_timeout:=' + str(no_progress_timeout),
            '-p', 'required_movement:=0.1', '-p', 'action_response_timeout:=10.0',
            '-p', 'map_save_path:=' + str(self.output),
        ], stdout=self.log_file, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.fake.stopping.set()
        self.fake.release_navigation.set()
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=2.0)
        self.log_file.close()
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)
        for server in self.fake.servers:
            server.destroy()
        self.fake.destroy_node()
        rclpy.shutdown(context=self.context)
        if exc_type is not None:
            print('\nExplorer child log:\n' + self.log_path.read_text(encoding='utf-8')[-8000:])
        if OPTIONS.keep_output:
            # TemporaryDirectory's finalizer otherwise removes the directory.
            self.directory._finalizer.detach()
            print('Retained integration outputs:', self.root)
        else:
            self.directory.cleanup()

    def wait_for(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            if self.process.poll() is not None:
                raise AssertionError('explorer exited before ' + reason)
            time.sleep(0.025)
        raise AssertionError('timed out waiting for ' + reason + '; counts=' +
                             str(self.fake.counts))

    def status(self):
        path = Path(str(self.output) + '.status.yaml')
        return path.read_text(encoding='utf-8') if path.exists() else ''

    def sim_wait(self, seconds):
        time.sleep(seconds / OPTIONS.clock_rate)


class ExplorerActionTests(unittest.TestCase):
    def test_bounded_rounds_try_short_goals_and_do_not_starve_far_safe_goals(self):
        with ExplorerHarness(7, no_progress_timeout=100.0) as h:
            h.fake.large_room = True
            h.fake.far_plans_only = True
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 23.0,
                       'an untried far safe goal after nearby routes repeatedly fail')
            self.assertTrue(any(math.hypot(x, y) < 0.35
                                for x, y in h.fake.planned_positions),
                            'ordinary candidates consumed the short-goal opportunity')
            self.assertGreater(h.fake.planned_positions[-1][0], 0.6)
            self.assertLessEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_unsafe_paths_and_cooldowns_cannot_extend_selection_indefinitely(self):
        with ExplorerHarness(5) as h:
            h.fake.unsafe_plan = True
            h.wait_for(lambda: h.fake.counts['plan'] > 0, 8.0, 'first unsafe route')
            h.wait_for(lambda: h.fake.counts['backup'] == 1, 10.0,
                       'selection budget ends before repeatedly expiring cooldowns')
            first_plan = next(t for e, t in h.fake.events if e == 'plan_started')
            backup = next(t for e, t in h.fake.events if e == 'backup_started')
            self.assertLessEqual((backup - first_plan) * OPTIONS.clock_rate, 16.0)
            self.assertGreater(h.fake.counts['plan'], 1)
            self.assertEqual(h.fake.counts['navigation'], 0,
                             'an unsafe route must never be sent to navigation')
            self.assertEqual(h.fake.max_active_motion, 1)
            h.wait_for(lambda: 'resumable: true' in h.status(), 15.0,
                       'failed recovery remains resumable rather than falsely complete')
            self.assertEqual(h.fake.counts['backup'], 1)
            h.fake.unsafe_plan = False
            h.fake.robot_x = .12  # A real pose improvement, not just cooldown expiry.
            h.wait_for(lambda: h.fake.counts['navigation'] >= 1, 8.0,
                       'safe path resumes navigation without granting more recoveries')
            self.assertEqual(h.fake.counts['backup'], 1)

    def test_lifecycle_refresh_during_navigation_avoids_goal_handoff_pause(self):
        with ExplorerHarness(6, no_progress_timeout=100.0) as h:
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 8.0, 'first navigation')
            h.sim_wait(6.0)  # Old code lets the lifecycle cache expire after five seconds.
            self.assertGreaterEqual(h.fake.lifecycle_requests, 15)
            h.fake.navigation_succeeds = True
            h.wait_for(lambda: h.fake.counts['navigation'] >= 2, 4.0,
                       'next goal without a new lifecycle discovery wait')
            terminal = next(t for e, t in h.fake.events if e == 'navigation_terminal')
            next_goal = [t for e, t in h.fake.events if e == 'navigation_started'][1]
            self.assertLess((next_goal - terminal) * OPTIONS.clock_rate, 1.5)

    def test_cancel_waits_for_terminal_and_recovery_is_single_flight(self):
        with ExplorerHarness(0) as h:
            h.fake.plan_only_first = True
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 8.0,
                       'first navigation goal')
            h.wait_for(h.fake.cancel_received.is_set, 5.0,
                       'watchdog cancellation accepted by fake Nav2')
            # Cancel acceptance alone must not release ownership of motion.
            h.sim_wait(3.0)
            self.assertEqual(h.fake.counts['navigation'], 1)
            self.assertEqual(h.fake.counts['backup'], 0)
            self.assertEqual(h.fake.counts['spin'], 0)
            self.assertEqual(h.fake.active_motion, 1)
            h.fake.release_navigation.set()
            h.wait_for(lambda: any(event == 'spin_terminal' for event, _ in h.fake.events),
                       75.0, 'candidate exhaustion then BackUp and Spin after navigation terminal')
            h.sim_wait(6.0)
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.counts['spin'], 1)
            self.assertEqual(h.fake.max_active_motion, 1)
            events = dict(h.fake.events)
            first_cancel = next(t for e, t in h.fake.events
                                if e == 'navigation_cancel_accepted')
            self.assertLess(first_cancel, events['backup_started'])
            first_nav_terminal = next(t for e, t in h.fake.events
                                      if e == 'navigation_terminal')
            self.assertLessEqual(first_nav_terminal, events['backup_started'])
            self.assertLessEqual(events['backup_terminal'], events['spin_started'])
            backup = h.fake.requests['backup'][0]
            self.assertAlmostEqual(backup.target.x, -0.15, places=5)
            self.assertAlmostEqual(backup.speed, 0.05, places=5)
            self.assertLessEqual(backup.time_allowance.sec, 8)
            spin = h.fake.requests['spin'][0]
            self.assertAlmostEqual(abs(spin.target_yaw), math.pi, places=5)
            self.assertLessEqual(spin.time_allowance.sec, 20)

    def test_completion_requires_five_fresh_frontier_free_maps(self):
        with ExplorerHarness(1, frontiers=False, stream_maps=False) as h:
            h.wait_for(lambda: h.fake.map_pub.get_subscription_count() > 0,
                       5.0, 'map subscription')
            # Lifecycle discovery is driven by decide(), which first needs a map.
            h.fake.publish_map()
            h.wait_for(lambda: h.fake.lifecycle_requests >= 5, 5.0,
                       'all lifecycle states queried')
            h.sim_wait(2.6)
            self.assertFalse(h.status())
            for _ in range(3):
                h.fake.publish_map()
                h.sim_wait(2.6)
                self.assertFalse(h.status(), 'completed before five new maps')
            h.fake.publish_map()
            h.wait_for(lambda: bool(h.status()), 4.0, 'completion status after fifth map')
            self.assertIn('status: completed', h.status())
            self.assertIn('completed: true', h.status())
            self.assertIn('map_saved: true', h.status())
            self.assertTrue(Path(str(h.output) + '.pgm').is_file())
            self.assertTrue(Path(str(h.output) + '.yaml').is_file())
            self.assertEqual(sum(h.fake.counts.values()), 0)

    def test_unplannable_frontiers_stall_instead_of_completing(self):
        with ExplorerHarness(2, plan_succeeds=False) as h:
            h.wait_for(lambda: h.fake.counts['plan'] > 0, 8.0,
                       'planner validation of reachable frontier standpoints')
            h.wait_for(lambda: 'status: stalled' in h.status(), 85.0,
                       'bounded stalled outcome for unplannable frontiers')
            status = h.status()
            self.assertIn('status: stalled', status)
            self.assertIn('completed: false', status)
            self.assertNotIn('status: completed', status)
            self.assertEqual(h.fake.counts['navigation'], 0)
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.counts['spin'], 1)
            self.assertEqual(h.fake.max_active_motion, 1)
            self.assertIn('resumable: true', status)
            # Let any previously untried standpoints finish their finite pass.
            h.sim_wait(20.0)
            plans = h.fake.counts['plan']
            h.sim_wait(20.0)
            self.assertEqual(h.fake.counts['plan'], plans,
                             'unchanged maps/cooldown expiry restarted failed planning')
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.counts['spin'], 1)
            h.fake.plan_succeeds = True
            h.fake.robot_x = .12
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 12.0,
                       'automatic navigation resumption after a safe plan becomes available')
            self.assertIn('completed: false', h.status())
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_rounded_start_navigates_then_finishes_endgame(self):
        with ExplorerHarness(8, rounded_endgame=True, no_progress_timeout=100.0) as h:
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 10.0,
                       'full-body-safe connection from the actual pose')
            self.assertIn('仅重接起点 0.05m', h.log_path.read_text())
            h.fake.move_on_success = h.fake.navigation_succeeds = True
            h.fake.has_frontiers = False
            h.wait_for(lambda: 'completed: true' in h.status(), 12.0,
                       'endgame navigation followed by five fresh clean maps')
            self.assertIn('map_saved: true', h.status())
            self.assertEqual(h.fake.counts['backup'], 0)
            self.assertEqual(h.fake.counts['spin'], 0)
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_only_invisible_wall_speck_can_complete_with_residual_report(self):
        with ExplorerHarness(9, frontiers=False, residual_speck=True) as h:
            h.wait_for(lambda: 'completed: true' in h.status(), 12.0,
                       'no actionable frontier, with a separately reported wall speck')
            self.assertIn('raw_frontier_cells: 1', h.status())
            self.assertIn('residual_frontier_cells: 1', h.status())
            self.assertIn('actionable_frontier_cells: 0', h.status())
            self.assertEqual(sum(h.fake.counts.values()), 0)

    def test_common_blocked_start_uses_one_waypoint_without_false_progress(self):
        with ExplorerHarness(12, rounded_endgame=True, no_progress_timeout=100.0,
                             blocked_prefix=True) as h:
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 12.0,
                       'one clearance waypoint after two common-prefix failures')
            self.assertIn('中间站位', h.log_path.read_text())
            goal = h.fake.requests['navigation'][0].pose.pose.position
            self.assertGreaterEqual(math.hypot(goal.x - .30, goal.y - .245), .15 - 1e-6)
            self.assertLessEqual(math.hypot(goal.x - .30, goal.y - .245), .35 + 1e-6)
            # An Action success without motion or observation must not replenish
            # the waypoint/recovery budget or count as real exploration progress.
            h.fake.navigation_succeeds = True
            h.wait_for(lambda: 'resumable: true' in h.status(), 24.0,
                       'bounded recovery followed by passive waiting')
            plans = h.fake.counts['plan']
            h.sim_wait(20.0)
            self.assertEqual(h.fake.counts['plan'], plans,
                             'a blocked common start reopened untried distant targets')
            self.assertEqual(h.fake.counts['navigation'], 1)
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.counts['spin'], 1)
            self.assertIn('completed: false', h.status())

    def test_missing_scan_tf_does_not_reopen_failed_goals_on_cooldown(self):
        with ExplorerHarness(10, plan_succeeds=False, scan_frame='missing_laser') as h:
            h.wait_for(lambda: 'resumable: true' in h.status(), 15.0,
                       'wait for the precise scan-time TF dependency')
            self.assertIn('扫描时刻', h.status())
            plans = h.fake.counts['plan']
            h.sim_wait(24.0)
            self.assertEqual(h.fake.counts['plan'], plans)
            self.assertEqual(h.fake.counts['backup'], 0)
            h.fake.scan_frame = 'base_footprint'
            h.wait_for(lambda: h.fake.counts['spin'] == 1, 8.0,
                       'the single deferred recovery after TF input returns')
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_recovery_can_use_older_fresh_scan_when_latest_tf_is_pending(self):
        with ExplorerHarness(11, plan_succeeds=False, scan_time_offset=.05) as h:
            h.wait_for(lambda: h.fake.counts['spin'] == 1, 18.0,
                       'a fresh complete older scan whose exact-time TF is available')
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_recovery_waits_for_full_scan_and_resumes_after_global_map_overlap_clears(self):
        with ExplorerHarness(4, stream_scans=False, robot_cell_free=False) as h:
            h.wait_for(lambda: 'resumable: true' in h.status(), 12.0,
                       'passive wait with missing lidar and global map overlap')
            self.assertEqual(h.fake.counts['backup'], 0)
            self.assertEqual(h.fake.counts['spin'], 0)
            # A fresh but partial scan does not make a backwards motion observable.
            h.fake.scan_span = math.pi
            h.fake.publish_scans = True
            h.sim_wait(5.0)
            self.assertEqual(h.fake.counts['backup'], 0)
            self.assertEqual(h.fake.counts['spin'], 0)
            h.fake.scan_span = 2.0 * math.pi
            h.wait_for(lambda: any(e == 'spin_terminal' for e, _ in h.fake.events),
                       8.0, 'one locally checked recovery despite displaced global-map wall')
            h.sim_wait(20.0)
            self.assertEqual(h.fake.counts['backup'], 1)
            self.assertEqual(h.fake.counts['spin'], 1)
            self.assertIn('resumable: true', h.status())
            h.fake.robot_cell_free = True
            h.wait_for(lambda: h.fake.counts['navigation'] == 1, 8.0,
                       'map/pose improvement wakes the paused explorer')
            self.assertEqual(h.fake.max_active_motion, 1)

    def test_interrupted_empty_map_sequence_requires_five_new_clean_maps(self):
        with ExplorerHarness(3, frontiers=False, stream_maps=False) as h:
            h.wait_for(lambda: h.fake.map_pub.get_subscription_count() > 0,
                       5.0, 'map subscription')
            h.fake.publish_map()
            h.wait_for(lambda: h.fake.lifecycle_requests >= 5, 5.0,
                       'all lifecycle states queried')
            h.sim_wait(2.6)
            for _ in range(3):
                h.fake.publish_map()
                h.sim_wait(2.6)
            self.assertFalse(h.status(), 'completed before initial fifth map')

            # This interruption lasts less than the 2 s decision interval: its
            # receipt must reset completion evidence even if decide() misses it.
            h.fake.has_frontiers = True
            h.fake.robot_cell_free = False
            h.fake.publish_map()
            h.sim_wait(0.4)
            h.fake.has_frontiers = False
            h.fake.robot_cell_free = True
            h.fake.publish_map()
            h.sim_wait(2.6)
            self.assertFalse(h.status(), 'interruption did not reset four empty maps')
            for _ in range(3):
                h.fake.publish_map()
                h.sim_wait(2.6)
                self.assertFalse(h.status(), 'completed before five new clean maps')
            h.fake.publish_map()
            h.wait_for(lambda: bool(h.status()), 4.0,
                       'completion after five consecutive clean maps following interruption')
            self.assertIn('status: completed', h.status())
            self.assertIn('map_saved: true', h.status())
            self.assertEqual(sum(h.fake.counts.values()), 0)


def main():
    global OPTIONS
    workspace = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--scenario', choices=['all', 'cancel', 'complete', 'stalled',
                                               'interrupted', 'recovery_scan', 'selection', 'handoff',
                                               'fairness', 'endgame', 'residual', 'missing_tf', 'pending_tf',
                                               'waypoint'],
                        default='all', help='run one regression scenario or all cases')
    parser.add_argument('--executable', '--explorer-binary', dest='explorer_binary', type=Path,
                        default=workspace / 'install/maze_explorer/lib/maze_explorer/explorer_node')
    parser.add_argument('--domain-id', type=int, default=40 + os.getpid() % 100,
                        help='first isolated ROS domain; next cases use adjacent domains')
    parser.add_argument('--clock-rate', type=float, default=2.0,
                        help='simulated seconds per wall second (default: 2)')
    parser.add_argument('--keep-output', action='store_true',
                        help='retain each child log and saved map in /tmp')
    OPTIONS = parser.parse_args()
    if not OPTIONS.explorer_binary.is_file():
        parser.error('explorer binary missing; build maze_explorer first or pass --explorer-binary')
    if not 0 <= OPTIONS.domain_id <= 231:
        parser.error('--domain-id must be between 0 and 231')
    if not 0.5 <= OPTIONS.clock_rate <= 4.0:
        parser.error('--clock-rate must be between 0.5 and 4.0')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    names = {
        'cancel': 'test_cancel_waits_for_terminal_and_recovery_is_single_flight',
        'complete': 'test_completion_requires_five_fresh_frontier_free_maps',
        'stalled': 'test_unplannable_frontiers_stall_instead_of_completing',
        'interrupted': 'test_interrupted_empty_map_sequence_requires_five_new_clean_maps',
        'recovery_scan': 'test_recovery_waits_for_full_scan_and_resumes_after_global_map_overlap_clears',
        'selection': 'test_unsafe_paths_and_cooldowns_cannot_extend_selection_indefinitely',
        'handoff': 'test_lifecycle_refresh_during_navigation_avoids_goal_handoff_pause',
        'fairness': 'test_bounded_rounds_try_short_goals_and_do_not_starve_far_safe_goals',
        'endgame': 'test_rounded_start_navigates_then_finishes_endgame',
        'residual': 'test_only_invisible_wall_speck_can_complete_with_residual_report',
        'missing_tf': 'test_missing_scan_tf_does_not_reopen_failed_goals_on_cooldown',
        'pending_tf': 'test_recovery_can_use_older_fresh_scan_when_latest_tf_is_pending',
        'waypoint': 'test_common_blocked_start_uses_one_waypoint_without_false_progress',
    }
    selected = names.values() if OPTIONS.scenario == 'all' else [names[OPTIONS.scenario]]
    suite = unittest.TestSuite(ExplorerActionTests(name) for name in selected)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
