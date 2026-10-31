"""Planar, offline mapping measurements. Ground truth never leaves this module.

The fixed initial alignment is shared by every later pose and scan; fitting an
independent transform at each time would hide exactly the drift being measured.
"""

import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from record_mapping_diagnostics import (
    angle_difference, detect_turns, interpolate, normalize_samples,
    stamp_seconds, yaw_from_quaternion,
)


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1],
            angle_difference(a[2] + b[2], 0.))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0] - s*a[1], s*a[0] - c*a[1], -a[2])


def transform_points(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


def transform_pose(transform):
    t = transform.transform
    return (t.translation.x, t.translation.y, yaw_from_quaternion(t.rotation))


def robot_static_frames(transforms, base='base_footprint', odom='odom'):
    frames = {base}
    edges = [(t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
             for t in transforms if t.header.frame_id.lstrip('/') not in ('map', odom)
             and t.child_frame_id.lstrip('/') not in ('map', odom, base)]
    while True:
        added = {child for parent, child in edges if parent in frames} - frames
        if not added:
            return frames
        frames.update(added)


def allowed_transforms(message, static=False, odom='odom', base='base_footprint', frames=None):
    # Whitelist the *edge*, not the /tf topic: it also contains old map->odom.
    if static and frames is None:
        frames = robot_static_frames(message.transforms, base, odom)
    return [t for t in message.transforms
            if ((static and t.header.frame_id.lstrip('/') in frames
                 and t.child_frame_id.lstrip('/') in frames
                 and t.child_frame_id.lstrip('/') != base)
                or (not static and t.header.frame_id.lstrip('/') == odom
                    and t.child_frame_id.lstrip('/') == base))]


def scan_points(scan, stride=1):
    ranges = np.asarray(scan.ranges, dtype=float)[::stride]
    angles = scan.angle_min + np.arange(len(scan.ranges))[::stride]*scan.angle_increment
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges < scan.range_max)
    return np.column_stack((ranges[valid]*np.cos(angles[valid]),
                            ranges[valid]*np.sin(angles[valid])))


def occupied_points(grid, boundary=False):
    cells = np.asarray(grid.data, dtype=int).reshape(grid.info.height, grid.info.width)
    occupied = cells >= 65
    if boundary:
        free = (cells >= 0) & (cells < 25)
        adjacent = np.zeros_like(free)
        adjacent[1:] |= free[:-1]
        adjacent[:-1] |= free[1:]
        adjacent[:, 1:] |= free[:, :-1]
        adjacent[:, :-1] |= free[:, 1:]
        occupied &= adjacent
    rows, cols = np.where(occupied)
    points = (np.column_stack((cols, rows)) + .5)*grid.info.resolution
    origin = grid.info.origin
    return transform_points(points, (origin.position.x, origin.position.y,
                                      yaw_from_quaternion(origin.orientation)))


def nearest_distances(points, reference):
    if not len(reference) or not len(points):
        return np.array([])
    # Bound temporary arrays; scipy is not required by the ROS installation.
    return np.concatenate([np.sqrt(np.min(np.sum(
        (part[:, None, :] - reference[None, :, :])**2, axis=2), axis=1))
        for part in np.array_split(points, max(1, math.ceil(len(points)/64)))])


def percentiles(values):
    data = np.asarray(values, dtype=float)
    return ({'count': int(len(data)), 'median_m': float(np.median(data)),
             'p95_m': float(np.percentile(data, 95)), 'max_m': float(np.max(data))}
            if len(data) else {'count': 0, 'median_m': None, 'p95_m': None, 'max_m': None})


def world_segments(path):
    """Read generator wall surfaces, including their thickness, in world XY."""
    result = []
    root = ET.parse(path).getroot()
    for model in root.findall('.//world/model'):
        if model.get('name') != 'maze_walls':
            continue
        mp = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
        for link in model.findall('link'):
            pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
            size = link.findtext('collision/geometry/box/size')
            if size is None:
                continue
            sx, sy, _ = map(float, size.split())
            local = np.array([[-sx/2, -sy/2], [sx/2, -sy/2],
                              [sx/2, sy/2], [-sx/2, sy/2]])
            corners = transform_points(local, compose((mp[0], mp[1], mp[5]),
                                                       (pose[0], pose[1], pose[5])))
            for a, b in zip(corners, np.roll(corners, -1, axis=0)):
                if np.linalg.norm(b-a) >= .4:
                    result.append([a.tolist(), b.tolist()])
    if not result:
        raise ValueError('No generator maze_walls box surfaces in reference world')
    return np.asarray(result)


def assign_segments(points, segments):
    a = segments[:, 0]
    delta = segments[:, 1] - a
    assignments, residuals, fractions = [], [], []
    for offset in range(0, len(points), 1024):
        part = points[offset:offset+1024]
        fraction = np.sum((part[:, None, :] - a)*delta, axis=2)/np.sum(delta**2, axis=1)
        closest = a + np.clip(fraction, 0, 1)[:, :, None]*delta
        distances = np.linalg.norm(part[:, None, :] - closest, axis=2)
        which = np.argmin(distances, axis=1)
        assignments.append(which)
        residuals.append(distances[np.arange(len(part)), which])
        fractions.append(fraction[np.arange(len(part)), which])
    return np.concatenate(assignments), np.concatenate(residuals), np.concatenate(fractions)


def wall_measurements(points, segments, stamps=None, direction_min_span=.4):
    """Fit each observed wall once, and compare separate visits to that wall.

    Unmatched points remain in the reported residual; they are never silently
    trimmed out to manufacture a passing score. Endpoints are excluded only
    from direction fitting because two different wall normals meet there.
    """
    if not len(points):
        return {'status': 'missing', 'walls': [], 'revisits': []}
    which, distance, fraction = assign_segments(points, segments)
    walls, revisits, directions = [], [], {}
    for index, (a, b) in enumerate(segments):
        length = np.linalg.norm(b-a)
        tangent = (b-a)/length
        normal = np.array([-tangent[1], tangent[0]])
        mask = ((which == index) & (distance <= .20) &
                (fraction*length >= .10) & ((1-fraction)*length >= .10))
        selected = points[mask]
        fit_span = float(np.ptp(selected @ tangent)) if len(selected) else 0.
        if len(selected) < 12 or fit_span < .40:
            continue
        _, _, vectors = np.linalg.svd(selected - selected.mean(axis=0), full_matrices=False)
        angle = math.degrees(math.acos(np.clip(abs(vectors[0] @ tangent), 0, 1)))
        fitted = vectors[0] if vectors[0] @ tangent >= 0 else -vectors[0]
        signed_angle = math.degrees(math.atan2(tangent[0]*fitted[1]-tangent[1]*fitted[0],
                                              tangent @ fitted))
        orientation = round(math.atan2(tangent[1], tangent[0]) % math.pi, 6)
        resolved = fit_span >= direction_min_span
        if resolved:
            directions.setdefault(orientation, []).append(signed_angle)
        walls.append({'surface': index, 'samples': len(selected),
                      'direction_error_deg': angle,
                      'signed_direction_error_deg': signed_angle,
                      'observed_span_m': fit_span, 'direction_resolved': resolved,
                      'normal_offset_m': float(np.median((selected-a) @ normal))})
        if stamps is not None:
            times = np.asarray(stamps)[mask]
            order = np.argsort(times)
            # A revisit needs a genuine observation gap, not two adjacent scans.
            chunks = np.split(order, np.where(np.diff(times[order]) >= 15.)[0] + 1)
            visits = []
            for chunk in chunks:
                if len(chunk) >= 30 and np.ptp(selected[chunk] @ tangent) >= .4:
                    visits.append((float(times[chunk].min()), float(times[chunk].max()),
                                   float(np.median((selected[chunk]-a) @ normal))))
            for first, later in zip(visits, visits[1:]):
                revisits.append({'surface': index, 'first_visit_s': first[:2],
                                 'later_visit_s': later[:2],
                                 'normal_displacement_m': abs(later[2]-first[2])})
    return {'status': 'measured', 'residual': percentiles(distance),
            'unmatched_fraction': float(np.mean(distance > .20)), 'walls': walls,
            'direction_min_span_m': direction_min_span,
            'unresolved_wall_count': sum(not w['direction_resolved'] for w in walls),
            'max_direction_error_deg': max((w['direction_error_deg'] for w in walls
                                            if w['direction_resolved']), default=None),
            'parallel_direction_spread_deg': max((max(v)-min(v) for v in directions.values()
                                                  if len(v) >= 2), default=None),
            'revisits': revisits,
            'max_revisit_displacement_m': max((v['normal_displacement_m'] for v in revisits), default=None)}


def load_truth(directory):
    truth = []
    with (Path(directory)/'poses.csv').open() as f:
        for row in csv.DictReader(f):
            if row['source'] == 'ground_truth':
                truth.append(tuple(float(row[k]) for k in ('stamp_s', 'x_m', 'y_m', 'yaw_rad')))
    return normalize_samples(truth)


def wall_direction_timeline(localized_scans, segments):
    """Incremental TLS fits with the same point/length rules as scan_walls.

    This locates the first growing-fit violation; an early noisy fit can later
    recover. It is diagnostic evidence, not a replacement for final acceptance.
    Sufficient statistics avoid refitting all previous points at every scan.
    """
    length = np.linalg.norm(segments[:, 1]-segments[:, 0], axis=1)
    tangent = (segments[:, 1]-segments[:, 0])/length[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    stats, rows = {}, []
    for stamp, points in sorted(localized_scans, key=lambda item: item[0]):
        if not len(points):
            continue
        which, distance, fraction = assign_segments(points, segments)
        for index in np.unique(which):
            mask = ((which == index) & (distance <= .20) &
                    (fraction*length[index] >= .10) & ((1-fraction)*length[index] >= .10))
            selected = points[mask]-segments[index, 0]
            if not len(selected):
                continue
            u, v = selected @ tangent[index], selected @ normal[index]
            old = stats.setdefault(int(index), np.array([0., 0., 0., 0., 0., 0., math.inf, -math.inf]))
            old[:6] += [len(u), u.sum(), v.sum(), u@u, u@v, v@v]
            old[6], old[7] = min(old[6], float(u.min())), max(old[7], float(u.max()))
        angles, groups = {}, {}
        for index, (count, su, sv, suu, suv, svv, lo, hi) in stats.items():
            if count < 12 or hi-lo < .4:
                continue
            angle = math.degrees(.5*math.atan2(2*(suv-su*sv/count),
                                              (suu-su*su/count)-(svv-sv*sv/count)))
            angles[index] = angle
            orientation = round(math.atan2(tangent[index, 1], tangent[index, 0]) % math.pi, 6)
            groups.setdefault(orientation, []).append((angle, index))
        pairs = [(max(values)[0]-min(values)[0], min(values)[1], max(values)[1])
                 for values in groups.values() if len(values) >= 2]
        spread, first, second = max(pairs, default=(None, None, None))
        worst = max(angles, key=lambda i: abs(angles[i])) if angles else None
        rows.append({'stamp_s': stamp, 'max_direction_error_deg': abs(angles[worst]) if worst is not None else None,
                     'parallel_direction_spread_deg': spread, 'direction_surface': worst,
                     'parallel_surface_a': first, 'parallel_surface_b': second})
    return rows


def compare_trajectory(samples, truth, output, max_gap=.15):
    samples = normalize_samples(samples)
    rows = []
    alignment = None
    for sample in samples:
        gt = interpolate(truth, sample[0], max_gap)
        if gt is None:
            continue
        if alignment is None:
            alignment = compose(gt[1:], inverse(sample[1:]))
        aligned = compose(alignment, sample[1:])
        rows.append((sample[0], math.degrees(angle_difference(aligned[2], gt[3])),
                     math.hypot(aligned[0]-gt[1], aligned[1]-gt[2])))
    with Path(output).open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['stamp_s', 'relative_yaw_error_deg', 'aligned_position_error_m'])
        writer.writerows(rows)
    crossed = next((r for r in rows if abs(r[1]) > 2. or r[2] > .10), None)
    crossing = dict(zip(('stamp_s', 'yaw_error_deg', 'position_error_m'), crossed)) if crossed else None
    if crossing:
        gt = interpolate(truth, crossed[0], max_gap)
        crossing['truth_xy_m'] = list(gt[1:3])
    return alignment, {
        'matched_samples': len(rows),
        'max_yaw_error_deg': max((abs(r[1]) for r in rows), default=None),
        'max_position_error_m': max((r[2] for r in rows), default=None),
        'first_threshold_crossing': crossing}


def write_turn_errors(truth, samples, destination):
    samples = normalize_samples(samples)
    with Path(destination).open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['start_s', 'end_s', 'truth_turn_deg', 'slam_turn_deg', 'error_deg'])
        for start, end in detect_turns(truth):
            before = interpolate(samples, start[0], .7)
            after = interpolate(samples, end[0], .7)
            expected = math.degrees(end[3]-start[3])
            actual = math.degrees(after[3]-before[3]) if before and after else None
            writer.writerow([start[0], end[0], expected, actual,
                             actual-expected if actual is not None else None])


def save_grid(grid, destination):
    """Save the final OccupancyGrid as a Nav2-compatible trinary map pair."""
    import yaml
    directory = Path(destination)
    cells = np.asarray(grid.data, dtype=int).reshape(grid.info.height, grid.info.width)
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells < 25)] = 254
    pixels[cells >= 65] = 0
    (directory/'map.pgm').write_bytes(
        f'P5\n{grid.info.width} {grid.info.height}\n255\n'.encode() + pixels[::-1].tobytes())
    origin = grid.info.origin
    (directory/'map.yaml').write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': grid.info.resolution,
        'origin': [origin.position.x, origin.position.y, yaw_from_quaternion(origin.orientation)],
        'negate': 0, 'occupied_thresh': .65, 'free_thresh': .25}))


def write_overlay(points, segments, destination):
    lower = segments.min(axis=(0, 1)) - .2
    upper = segments.max(axis=(0, 1)) + .2
    width, height = upper-lower
    items = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{lower[0]} {-upper[1]} {width} {height}">',
             '<rect x="-100" y="-100" width="200" height="200" fill="white"/>']
    for a, b in segments:
        items.append(f'<path d="M{a[0]} {-a[1]} L{b[0]} {-b[1]}" stroke="#333" stroke-width="0.015"/>')
    for x, y in points:
        items.append(f'<circle cx="{x:.4f}" cy="{-y:.4f}" r=".012" fill="#d33" opacity=".6"/>')
    Path(destination).write_text('\n'.join(items) + '</svg>\n')


def evaluate_geometry(samples, truth, localized_scans, grid, world, destination):
    """Common geometry evaluation for both replay and live-run recordings."""
    directory = Path(destination)
    alignment, trajectory = compare_trajectory(samples, truth, directory/'comparison.csv')
    write_turn_errors(truth, samples, directory/'turns.csv')
    result = {'trajectory': trajectory, 'acceptance': 'incomplete',
              'reason': 'Ground truth/reference/map/revisits required'}
    if grid is not None:
        save_grid(grid, directory)
    if alignment is None or world is None or grid is None or not localized_scans:
        return result
    segments = world_segments(world)
    boundary = transform_points(occupied_points(grid, boundary=True), alignment)
    if not len(boundary):
        result['reason'] = 'No occupied map boundary'
        return result
    # A single raster step over L produces atan(resolution/L) direction change.
    # Retain short fits for inspection, but verify their direction on continuous
    # laser points instead of claiming sub-pixel angular accuracy from the map.
    map_span = max(.4, grid.info.resolution/math.tan(math.radians(2.)))
    result['map_walls'] = wall_measurements(boundary, segments, direction_min_span=map_span)
    write_overlay(boundary, segments, directory/'wall_overlay.svg')
    aligned_scans = [(t, transform_points(p, alignment)) for t, p in localized_scans]
    points = np.concatenate([p for _, p in aligned_scans])
    stamps = np.concatenate([np.full(len(p), t) for t, p in localized_scans])
    result['scan_walls'] = wall_measurements(points, segments, stamps)
    timeline = wall_direction_timeline(aligned_scans, segments)
    with (directory/'wall_direction_timeline.csv').open('w', newline='') as stream:
        columns = ['stamp_s', 'max_direction_error_deg', 'parallel_direction_spread_deg',
                   'direction_surface', 'parallel_surface_a', 'parallel_surface_b']
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(timeline)
    first = next((r for r in timeline if (r['max_direction_error_deg'] or 0) > 2.
                  or (r['parallel_direction_spread_deg'] or 0) > 2.), None)
    if first:
        first = dict(first)
        gt = interpolate(truth, first['stamp_s'], .15)
        first['truth_xy_m'] = list(gt[1:3]) if gt else None
        first['note'] = 'First prefix fit crossing; may be transient. Final acceptance uses all points.'
    result['first_prefix_direction_violation'] = first
    walls, revisits = result['map_walls'], result['scan_walls']
    direction = walls.get('max_direction_error_deg')
    displacement = revisits.get('max_revisit_displacement_m')
    violations = []
    if direction is not None and direction > 2.:
        violations.append('wall direction > 2 degrees')
    spread = walls.get('parallel_direction_spread_deg')
    if spread is not None and spread > 2.:
        violations.append('parallel wall direction spread > 2 degrees')
    scan_direction = revisits.get('max_direction_error_deg')
    scan_spread = revisits.get('parallel_direction_spread_deg')
    if scan_direction is not None and scan_direction > 2.:
        violations.append('continuous scan wall direction > 2 degrees')
    if scan_spread is not None and scan_spread > 2.:
        violations.append('continuous scan parallel wall spread > 2 degrees')
    if walls['residual']['p95_m'] is not None and walls['residual']['p95_m'] > .10:
        violations.append('map wall residual P95 > 0.10 m')
    if walls['unmatched_fraction'] > .01:
        violations.append('unmatched map boundary > 1 percent')
    if displacement is not None and displacement > .10:
        violations.append('revisit displacement > 0.10 m')
    result['failed_checks'] = violations
    if violations:
        result.update(acceptance='fail', reason='; '.join(violations))
    elif direction is not None and displacement is not None:
        result.update(acceptance='pass', reason='Geometric checks passed; live navigation must be evaluated separately')
    return result
