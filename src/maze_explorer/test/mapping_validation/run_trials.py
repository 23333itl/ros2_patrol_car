#!/usr/bin/env python3
"""Serial isolated Gazebo noise controls or three gated 1510-s patrol trials.

Source the workspace first. Only owned process groups receive signals. Truth
is recorded for evaluation and never published as localization.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import yaml


def run_trial(args, root, name, noise, repeat, stop):
    folder = root/name
    folder.mkdir()
    controlled = args.mode == 'noise-control'
    duration = args.duration or (610. if controlled else 1510.)
    info = yaml.safe_load(args.world.with_name(args.world.stem+'_info.yaml').read_text())
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               GZ_PARTITION='maze_prior_'+root.name+'_'+name,
               ROS_LOG_DIR=str(folder/'ros_logs'), GZ_HOMEDIR=str(folder/'gz'),
               MPLCONFIGDIR=str(folder/'mpl'), OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
    env.pop('ROS_LOCALHOST_ONLY', None)
    processes, streams = [], []
    result = dict(status='incomplete', profile=args.profile, lidar_noise_stddev_m=noise,
                  world=str(args.world), start=info['start'], seed=info['seed'])

    def start(label, command):
        stream = (folder/(label+'.log')).open('w')
        streams.append(stream)
        process = subprocess.Popen(command, env=env, stdout=stream, stderr=stream, start_new_session=True)
        processes.append(process)
        return process

    try:
        launch = start('launch', ['ros2', 'launch', 'maze_explorer', 'explore_maze.launch.py',
            'world:='+str(args.world), 'slam_profile:='+args.profile,
            'lidar_noise_stddev:='+str(noise), 'rviz:=false', 'gz_gui:=false',
            'explore:='+('false' if controlled else 'true'), 'map_save_path:='+str(folder/'map')])
        if stop.wait(3.):
            raise RuntimeError('Interrupted before recording')
        recorder = start('recorder', ['ros2', 'run', 'maze_explorer', 'record_mapping_diagnostics.py',
            '--world', info['name'], '--duration', str(duration), '--wall-timeout', str(args.wall_timeout),
            '--seed', str(info['seed']), '--run', str(repeat), '--output', str(folder/'recording')])
        route = (start('route', [sys.executable, str(Path(__file__).with_name('drive_route.py')),
                 str(folder/'route_events.json')]) if controlled else None)
        began = notice = time.monotonic()
        ready = False
        while recorder.poll() is None and not stop.wait(.5):
            if launch.poll() is not None:
                raise RuntimeError('Launch exited')
            if route and route.poll() not in (None, 0):
                raise RuntimeError('Fixed route failed; keep data but do not accept the control')
            if not ready:
                log = (folder/'launch.log').read_text(errors='replace')
                ready = 'Managed nodes are active' in log
                if not ready and time.monotonic()-began > 90:
                    raise RuntimeError('Nav2 lifecycle startup failed')
            if time.monotonic()-began > args.wall_timeout+30:
                raise RuntimeError('Wall timeout')
            if time.monotonic()-notice >= 45:
                print(name, 'wall seconds', round(time.monotonic()-began), flush=True)
                notice = time.monotonic()
        result['recorder_exit'] = recorder.poll()
        if recorder.poll() == 0 and not stop.is_set():
            summary = json.loads((folder/'recording/summary.json').read_text())
            actual = (summary.get('runtime_robot_model') or {}).get('lidar_noise_stddev_m')
            if actual is None or not math.isclose(actual, noise, abs_tol=1e-12):
                raise RuntimeError('Runtime lidar noise differs from requested control')
            if summary['recorded_duration_sim_s'] < duration:
                raise RuntimeError('Recording shorter than requested')
            result['status'] = 'recorded'  # Availability only, never geometry acceptance.
        else:
            result['error'] = 'Recorder failed or experiment interrupted'
    except Exception as error:
        result['error'] = str(error)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
        for stream in streams:
            stream.close()
        (folder/'trial.json').write_text(json.dumps(result, indent=2))
        print(name, result, flush=True)
    return result


def main():
    workspace = Path(__file__).resolve().parents[4]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=('noise-control', 'patrol'), default='noise-control')
    parser.add_argument('--profile', default='simulation_stable')
    parser.add_argument('--world', type=Path, default=workspace/'maze_output/maze.world')
    parser.add_argument('--domain', '--domain-start', type=int, default=151)
    parser.add_argument('--duration', type=float, default=0., help='0: 610 s controls / 1510 s patrols')
    parser.add_argument('--noise-levels', nargs='+', type=float, choices=(.01, 0.), default=[.01, 0.],
                        help='Serial control levels; select one to retry a failed/missing recording')
    parser.add_argument('--wall-timeout', type=float, default=3600.)
    parser.add_argument('--replay-comparisons', nargs=2, type=Path,
                        help='Both noise-control comparison_summary.json files, required for prior patrols')
    args = parser.parse_args()
    minimum = 600. if args.mode == 'noise-control' else 1500.
    if (not 0 <= args.domain <= 232 or not math.isfinite(args.duration) or
            args.duration != 0 and args.duration < minimum or
            not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0):
        parser.error('Use a valid domain, a positive wall timeout, and full-length simulation durations')
    if args.mode == 'noise-control' and args.profile != 'simulation_stable':
        parser.error('Noise controls must use the unchanged simulation_stable baseline')
    if args.mode == 'patrol' and args.profile != 'simulation_stable':
        if not args.replay_comparisons:
            parser.error('Prior candidates require passing replay reports at both noise levels')
        noises = []
        for path in args.replay_comparisons:
            report = json.loads(path.read_text())
            if report.get('prior_gates', {}).get(args.profile, {}).get('acceptance') != 'pass':
                parser.error('Candidate has not passed both replay gates')
            noises.append((report.get('runtime_robot_model') or {}).get('lidar_noise_stddev_m'))
        if sorted(n for n in noises if n is not None) != [0., .01]:
            parser.error('Replay reports must contain verified 0 and 0.01 m runtime noise controls')
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    args.world = args.world.resolve()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    items = ([('noise_0p01' if noise else 'noise_0', noise, 1)
              for noise in dict.fromkeys(args.noise_levels)] if args.mode == 'noise-control' else
             [(f'navigation_{i}', .01, i) for i in range(1, 4)])
    results = {}
    for name, noise, repeat in items:
        if stop.is_set():
            break
        results[name] = run_trial(args, root, name, noise, repeat, stop)
        (root/'trials.json').write_text(json.dumps(results, indent=2))
        if results[name]['status'] != 'recorded':
            break
    return 0 if len(results) == len(items) and all(r['status'] == 'recorded' for r in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
