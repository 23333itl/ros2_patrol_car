"""Scan-time correction diagnostics; never publish TF or motion commands."""
import csv
import bisect
import json
import math
import re
from pathlib import Path

import numpy as np

from mapping_analysis import compose, inverse, interpolate, normalize_samples, angle_difference, detect_turns


def correction_steps(poses, odometry, max_gap=.15):
    """Measure C_new O(t) versus C_old O(t), independent of either frame origin.

    C is map->odom inferred from the corrected scan pose and *that scan's*
    odometry. A missing interpolation bracket breaks the chain: no interpolation
    across a missing TF interval, and no guessed jump on either side of it.
    """
    odometry = normalize_samples(odometry)
    corrections, steps = [], []
    previous = None
    missing = 0
    for pose in normalize_samples(poses):
        odom = interpolate(odometry, pose[0], max_gap)
        if odom is None:
            previous = None
            missing += 1
            continue
        current = (pose[0], *compose(pose[1:], inverse(odom[1:])))
        corrections.append(current)
        if previous is not None:
            predicted = compose(previous[1:], odom[1:])
            actual = compose(current[1:], odom[1:])
            distance = math.hypot(actual[0]-predicted[0], actual[1]-predicted[1])
            yaw = math.degrees(angle_difference(actual[2], predicted[2]))
            steps.append({'stamp_s': pose[0], 'previous_stamp_s': previous[0],
                          'interval_s': pose[0]-previous[0],
                          'position_step_m': distance, 'yaw_step_deg': yaw,
                          'jump': distance > .10 or abs(yaw) > 2.})
        previous = current
    return corrections, steps, missing


def scan_intervals(poses, odometry):
    odometry = normalize_samples(odometry)
    poses = normalize_samples(poses)
    rows = []
    for before, after in zip(poses, poses[1:]):
        a, b = (interpolate(odometry, p[0], .15) for p in (before, after))
        rows.append({'stamp_s': after[0], 'interval_s': after[0]-before[0],
                     'translation_m': math.hypot(b[1]-a[1], b[2]-a[2]) if a and b else None,
                     'rotation_deg': math.degrees(angle_difference(b[3], a[3])) if a and b else None})
    return rows


def correction_statistics(steps, odometry):
    """Classify with input odometry, never with the candidate's SLAM result.

    Motion classes partition valid odometry time. Revisit is an overlapping
    spatial label: within 0.30 m of an observation >=30 s and >=1 m of travel
    earlier. This is a trajectory revisit, not a wall-quality measurement.
    Rates use odometry time, including intervals without a processed scan, so
    skipping scan updates cannot silently change the denominator.
    """
    odometry = normalize_samples(odometry)
    names = ('all', 'moving', 'stationary', 'straight', 'turn', 'revisit')
    durations = dict.fromkeys(names, 0.)
    stamps, labels, history = [], [], {}
    travelled, last_stored = 0., None
    for a, b in zip(odometry, odometry[1:]):
        dt = b[0]-a[0]
        distance = math.hypot(b[1]-a[1], b[2]-a[2])
        travelled += distance
        if not 0 < dt <= .15:
            continue
        speed = distance/dt
        angular = abs(angle_difference(b[3], a[3]))/dt
        motion = ('stationary' if speed < .01 and angular < .02 else
                  'turn' if angular >= .05 else 'straight')
        cell = (math.floor(b[1]/.3), math.floor(b[2]/.3))
        revisit = any(b[0]-old[0] >= 30. and travelled-old[3] >= 1. and
                      math.hypot(b[1]-old[1], b[2]-old[2]) <= .3
                      for dx in (-1, 0, 1) for dy in (-1, 0, 1)
                      for old in history.get((cell[0]+dx, cell[1]+dy), ()))
        if last_stored is None or travelled-last_stored >= .05:
            history.setdefault(cell, []).append((b[0], b[1], b[2], travelled))
            last_stored = travelled
        groups = ['all', motion]
        if motion != 'stationary':
            groups.append('moving')
        if revisit:
            groups.append('revisit')
        stamps.append(b[0])
        labels.append((a[0], groups))
        for group in groups:
            durations[group] += dt
    selected = {name: [] for name in names}
    for step in steps:
        index = bisect.bisect_left(stamps, step['stamp_s'])
        groups = labels[index][1] if (index < len(stamps) and
                  labels[index][0] <= step['stamp_s'] <= stamps[index]) else []
        step['motion'] = next((g for g in groups if g in ('stationary', 'straight', 'turn')), 'unknown')
        step['revisit'] = 'revisit' in groups
        for group in groups:
            selected[group].append(step)
    report = {}
    for name, values in selected.items():
        position = [v['position_step_m'] for v in values]
        yaw = [abs(v['yaw_step_deg']) for v in values]
        duration = durations[name]
        report[name] = {
            'count': len(values), 'duration_sim_s': duration,
            'position_p95_m': float(np.percentile(position, 95)) if values else None,
            'position_max_m': max(position, default=None),
            'yaw_p95_deg': float(np.percentile(yaw, 95)) if values else None,
            'yaw_max_deg': max(yaw, default=None),
            'position_total_m': sum(position), 'yaw_total_deg': sum(yaw),
            'position_m_per_sim_s': sum(position)/duration if duration > 0 else None,
            'yaw_deg_per_sim_s': sum(yaw)/duration if duration > 0 else None,
        }
    return report


def correction_gate(baseline, candidate):
    """Replay gate only; passing never replaces the three live patrol trials."""
    reasons = []
    if candidate.get('profile') not in ('stable_translation_prior', 'stable_pose_prior'):
        reasons.append('Not a scan-matching prior candidate')
    if not baseline.get('input_sha256') or baseline.get('input_sha256') != candidate.get('input_sha256'):
        reasons.append('Inputs are missing a hash or are not identical')
    for label, report in (('baseline', baseline), ('candidate', candidate)):
        if (report.get('acceptance') not in (('pass', 'fail') if label == 'baseline' else ('pass',)) or
                report.get('playback', {}).get('playback_duration_s', 0) < 600):
            reasons.append(f'{label}: geometry/data/duration acceptance not passed')
        diagnostics = report.get('correction_diagnostics', {})
        if (label == 'candidate' and diagnostics.get('correction_jumps') or
                diagnostics.get('missing_odom_brackets', 0)):
            reasons.append(f'{label}: correction jumps or missing TF')
        groups = diagnostics.get('by_motion', {})
        if any(groups.get(g, {}).get('duration_sim_s', 0) <= 0 for g in ('turn', 'revisit')):
            reasons.append(f'{label}: turns and revisits required')
    old = baseline.get('correction_diagnostics', {}).get('by_motion', {})
    new = candidate.get('correction_diagnostics', {}).get('by_motion', {})
    changes = {}
    for group, metrics in (('moving', ('position_p95_m', 'yaw_p95_deg')),
                           ('all', ('position_m_per_sim_s', 'yaw_deg_per_sim_s'))):
        for metric in metrics:
            a, b = old.get(group, {}).get(metric), new.get(group, {}).get(metric)
            if a is None or b is None or not math.isfinite(a) or not math.isfinite(b):
                reasons.append(f'Missing metric: {group}.{metric}')
                continue
            ratio = b/a if a > 1e-12 else (1. if b <= 1e-12 else None)
            changes[metric] = {'baseline': a, 'candidate': b, 'ratio': ratio}
            if ratio is None or ratio > 1.05+1e-9:
                reasons.append(f'{metric} increased by more than 5%')
    if not any(changes.get(k, {}).get('baseline', 0) > 1e-12 and
               changes[k]['ratio'] is not None and changes[k]['ratio'] <= .70+1e-9
               for k in ('position_p95_m', 'yaw_p95_deg')):
        reasons.append('Neither moving correction P95 improved by at least 30%')
    return {'acceptance': 'pass' if not reasons else 'fail', 'reasons': reasons,
            'changes': changes, 'live_trials_required': 3, 'live_min_duration_sim_s': 1500}


def data_health(scan_stamps, log=''):
    """10-Hz input coverage and logged persistent controller underruns.

    Motion thresholds legitimately skip processed scans. Only the input stream
    is checked for gaps. Three reported controller misses within 3.5 wall
    seconds count as sustained, without failing a run on one isolated warning.
    """
    stamps = sorted(set(scan_stamps))
    gaps = [{'stamp_s': b, 'gap_s': b-a} for a, b in zip(stamps, stamps[1:]) if b-a > .15+1e-9]
    misses = []
    for line in log.splitlines():
        if 'Control loop missed its desired rate' in line:
            times = re.findall(r'\[(\d+\.\d+)\]', line)
            if times:
                misses.append(float(times[0]))
    sustained = any(c-a <= 3.5 for a, c in zip(misses, misses[2:]))
    return {'input_scan_count': len(stamps), 'input_scan_gaps': gaps,
            'message_filter_failures': log.count('Message Filter dropping message'),
            'controller_rate_warnings': log.count('Control loop missed its desired rate'),
            'sustained_controller_underrun': sustained,
            'complete': len(stamps) > 1 and not gaps and not sustained and
                        'Message Filter dropping message' not in log}


def write_rows(path, rows, columns):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def write_correction_diagnostics(poses, odometry, output, sources=None, context_directory=None):
    """Write every update and +/-5 simulation-second windows for actual jumps.

    Raw time/TF/command tables are sliced by receive time where available; pose
    plots use sensor stamps. Thus future-dated TF is retained without mistaking
    its stamp for the time the correction was computed.
    """
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    poses, odometry = normalize_samples(poses), normalize_samples(odometry)
    corrections, steps, missing = correction_steps(poses, odometry)
    by_motion = correction_statistics(steps, odometry)
    write_rows(output/'correction_by_motion.csv',
               [dict(category=name, **values) for name, values in by_motion.items()],
               ['category', *by_motion['all']])
    write_rows(output/'correction_steps.csv', steps,
               ['stamp_s', 'previous_stamp_s', 'interval_s', 'position_step_m', 'yaw_step_deg', 'jump',
                'motion', 'revisit'])
    intervals = scan_intervals(poses, odometry)
    write_rows(output/'processed_scan_intervals.csv', intervals,
               ['stamp_s', 'interval_s', 'translation_m', 'rotation_deg'])
    turns = []
    for start, end in detect_turns(normalize_samples(odometry)):
        stamps = [p[0] for p in poses if start[0] <= p[0] <= end[0]]
        bounds = [start[0], *sorted(set(stamps)), end[0]]
        turns.append({'start_sim_s': start[0], 'end_sim_s': end[0],
                      'turn_deg': math.degrees(end[3]-start[3]),
                      'net_translation_m': math.hypot(end[1]-start[1], end[2]-start[2]),
                      'processed_scans': len(stamps),
                      'max_scan_gap_s': max((b-a for a, b in zip(bounds, bounds[1:])), default=None)})
    write_rows(output/'turn_scan_coverage.csv', turns,
               ['start_sim_s', 'end_sim_s', 'turn_deg', 'net_translation_m', 'processed_scans', 'max_scan_gap_s'])
    jumps = [s for s in steps if s['jump']]
    sources = sources or {'slam_pose': poses, 'odom_tf': odometry}
    write_event_windows(jumps, output, sources, context_directory)
    report = {'metric': 'C_new O(t) versus C_old O(t), at the same scan time',
              'missing_odom_brackets': missing, 'paired_corrections': len(corrections),
              'correction_steps': len(steps), 'correction_jumps': jumps,
              'max_position_step_m': max((s['position_step_m'] for s in steps), default=None),
              'max_abs_yaw_step_deg': max((abs(s['yaw_step_deg']) for s in steps), default=None),
              'by_motion': by_motion,
              'motion_classification': {'linear_stationary_below_m_s': .01,
                  'angular_stationary_below_rad_s': .02, 'turn_above_rad_s': .05,
                  'revisit_radius_m': .3, 'revisit_min_gap_s': 30., 'revisit_min_travel_m': 1.,
                  'revisit_overlaps_motion_classes': True,
                  'p95_weighting': 'per processed correction; promotion uses moving samples',
                  'rate_denominator': 'valid input odometry simulation time, including skipped scans'},
              'turn_scan_coverage': turns,
              'max_processed_scan_interval_s': max((s['interval_s'] for s in intervals), default=None)}
    (output/'correction_diagnostics.json').write_text(json.dumps(report, indent=2))
    return corrections, report


def scan_event_context(events, context_directory):
    """Extract original range data once per category, without publishing it."""
    selected = [[] for _ in events]
    static = []
    if not events or not context_directory or not (Path(context_directory)/'rosbag/metadata.yaml').exists():
        return selected, static
    from replay_mapping import bag_reader
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    reader = bag_reader(Path(context_directory)/'rosbag')
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/tf_static':
            static.append(message_to_ordereddict(deserialize_message(data, types[topic])))
        elif topic == '/scan':
            message = deserialize_message(data, types[topic])
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            matches = [i for i, e in enumerate(events) if abs(stamp-e['stamp_s']) <= 5.]
            if matches:
                record = {'stamp_s': stamp, 'frame_id': message.header.frame_id,
                    **{name: getattr(message, name) for name in ('angle_min', 'angle_max',
                        'angle_increment', 'time_increment', 'scan_time', 'range_min', 'range_max')},
                    'ranges': [float(v) if math.isfinite(v) else None for v in message.ranges],
                    'intensities': [float(v) if math.isfinite(v) else None for v in message.intensities]}
                for index in matches:
                    selected[index].append(record)
    return selected, static


def write_event_windows(events, output, sources, context_directory=None, category='jump_events'):
    output = Path(output)
    scans, static = scan_event_context(events, context_directory)
    for index, event in enumerate(events, 1):
        directory = output/category/f'{index:04d}_{event["stamp_s"]:.3f}'
        directory.mkdir(parents=True, exist_ok=True)
        low, high = event['stamp_s']-5., event['stamp_s']+5.
        selected = {name: [p for p in values if low <= p[0] <= high]
                    for name, values in sources.items()}
        if context_directory and (Path(context_directory)/'poses.csv').exists():
            with (Path(context_directory)/'poses.csv').open() as stream:
                for row in csv.DictReader(stream):
                    name = row['source']
                    # Replay supplies its own SLAM result. Never mix in old map outputs.
                    if name in sources or name not in ('raw_odom', 'filtered_odom', 'imu'):
                        continue
                    stamp = float(row['stamp_s'])
                    if low <= stamp <= high:
                        selected.setdefault(name, []).append(tuple(float(row[k]) for k in
                            ('stamp_s', 'x_m', 'y_m', 'yaw_rad')))
        (directory/'scan.jsonl').write_text(''.join(json.dumps(row, allow_nan=False)+'\n'
                                                   for row in scans[index-1]))
        (directory/'tf_static.json').write_text(json.dumps(static, indent=2))
        (directory/'event.json').write_text(json.dumps({**event, 'window_sim_s': [low, high],
            'raw_scan_count': len(scans[index-1]), 'invalid_scan_ranges': 'null',
            'context_tf_note': 'tf_samples.csv is the original recording, not the replay result; '
                               'use this profile map_to_odom.csv for replay corrections'}, indent=2))
        write_rows(directory/'poses.csv', [dict(source=name, stamp_s=p[0], x_m=p[1], y_m=p[2], yaw_rad=p[3])
                                          for name, values in selected.items() for p in values],
                   ['source', 'stamp_s', 'x_m', 'y_m', 'yaw_rad'])
        if context_directory:
            for name in ('timing.csv', 'commands.csv', 'tf_samples.csv', 'map_timing.csv', 'tf_publishers.csv'):
                path = Path(context_directory)/name
                if not path.exists():
                    continue
                with path.open() as stream:
                    reader = csv.DictReader(stream)
                    columns = reader.fieldnames
                    rows = []
                    for row in reader:
                        stamp = row.get('receive_sim_s') or row.get('stamp_s')
                        if stamp and low <= float(stamp) <= high:
                            rows.append(row)
                write_rows(directory/name, rows, columns)
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        colors = {name: f'C{index % 10}' for index, name in enumerate(selected)}
        for name, values in selected.items():
            if not values:
                continue
            values = normalize_samples(values)
            times = [p[0] for p in values]
            for axis, component in zip(axes, (1, 2, 3)):
                if name == 'imu' and component != 3:
                    continue
                scale = 180/math.pi if component == 3 else 1.
                axis.plot(times, [(p[component]-values[0][component])*scale for p in values],
                          label=name, color=colors[name])
        for axis, label in zip(axes, ('relative x (m)', 'relative y (m)', 'relative yaw (deg)')):
            axis.axvline(event['stamp_s'], color='red', linestyle='--')
            axis.set_ylabel(label)
            axis.grid(True)
        axes[-1].legend(loc='best')
        axes[-1].set_xlabel('simulation seconds (sensor stamps)')
        fig.suptitle(category + ': each source relative to its first window sample')
        fig.tight_layout()
        fig.savefig(directory/'poses.png', dpi=120)
        plt.close(fig)
