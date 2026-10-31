"""Scan-time correction diagnostics; never publish TF or motion commands."""
import csv
import json
import math
from pathlib import Path

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
    write_rows(output/'correction_steps.csv', steps,
               ['stamp_s', 'previous_stamp_s', 'interval_s', 'position_step_m', 'yaw_step_deg', 'jump'])
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
              'turn_scan_coverage': turns,
              'max_processed_scan_interval_s': max((s['interval_s'] for s in intervals), default=None)}
    (output/'correction_diagnostics.json').write_text(json.dumps(report, indent=2))
    return corrections, report


def write_event_windows(events, output, sources, context_directory=None, category='jump_events'):
    output = Path(output)
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
        (directory/'event.json').write_text(json.dumps({**event, 'window_sim_s': [low, high]}, indent=2))
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
