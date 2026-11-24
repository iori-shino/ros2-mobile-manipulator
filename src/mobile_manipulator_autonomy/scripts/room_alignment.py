"""Refine a station pose against the known rectangular workcell walls."""

import math

import numpy as np


def scan_heading_shift(points, prior):
    """Find the closest room-axis heading from adjacent LiDAR returns."""
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    segments = segments[(lengths > .008) & (lengths < .16)]
    if len(segments) < 24:
        return None
    headings = np.arctan2(segments[:, 1], segments[:, 0])
    consensus = np.mean(np.exp(4j*headings))
    if abs(consensus) < .50:
        return None
    return math.remainder(-float(np.angle(consensus))/4-prior, math.pi/2)


def fit_room(points, seed, bounds):
    """Fit an SE(2) pose from LiDAR hits near both perpendicular wall axes."""
    points = np.asarray(points, dtype=float)
    if len(points) < 40:
        raise RuntimeError('Too few LiDAR returns for room alignment')
    try:
        return fit_candidate(points, seed, bounds, 0.)
    except RuntimeError as initial_error:
        candidates = []
        heading = scan_heading_shift(points, seed[2])
        for shift in (-.21, .21):
            try:
                candidates.append(fit_candidate(points, seed, bounds, shift))
            except RuntimeError:
                continue
        # A well-defined wall direction can seed a larger correction while
        # staying clear of the rectangular room's 90 degree ambiguity.
        if heading is not None and abs(heading) < .65:
            try:
                candidates.append(fit_candidate(points, seed, bounds, heading, heading))
            except RuntimeError:
                pass
        if not candidates:
            raise initial_error
        if any(math.dist(candidates[0], candidate) > .025 for candidate in candidates[1:]):
            raise RuntimeError('Multiple incompatible room fits')
        return candidates[0]


def fit_candidate(points, seed, bounds, shift, scan_shift=None):
    pose = np.array(seed, dtype=float)
    pose[2] += shift
    edges = np.array(bounds, dtype=float)
    for limit in (.42, .26, .15, .085, .05, .035):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        rotated = points @ np.array(((c, s), (-s, c)))
        shifted = rotated + pose[:2]
        residuals = shifted[:, (0, 0, 1, 1)]-edges
        wall = np.abs(residuals).argmin(axis=1)
        residual = residuals[np.arange(len(points)), wall]
        valid = np.abs(residual) < limit
        x_wall = wall < 2
        for axis, mask in ((1, valid & x_wall), (0, valid & ~x_wall)):
            if np.count_nonzero(mask) < 8 or np.ptp(shifted[mask, axis]) < .35:
                raise RuntimeError('Insufficient perpendicular wall returns for room alignment')
        jacobian = np.column_stack((x_wall, ~x_wall,
                                    np.where(x_wall, -rotated[:, 1], rotated[:, 0])))
        update, _, rank, _ = np.linalg.lstsq(jacobian[valid], -residual[valid], rcond=None)
        if rank != 3:
            raise RuntimeError('Degenerate LiDAR room alignment')
        pose += update
    # Near a station the SLAM yaw can lag after a reversing turn. Before
    # trusting a different heading, require dense, long wall support on both axes.
    rotation = pose[2]-seed[2]
    rotation_supported = abs(rotation) <= .50 or (
        scan_shift is not None and abs(rotation-scan_shift) < .025)
    if np.linalg.norm(pose[:2]-seed[:2]) > .43 or not rotation_supported:
        raise RuntimeError(f'LiDAR alignment is outside its search envelope: {pose.tolist()}, prior {seed}')
    c, s = math.cos(pose[2]), math.sin(pose[2])
    shifted = points @ np.array(((c, s), (-s, c))) + pose[:2]
    residuals = shifted[:, (0, 0, 1, 1)]-edges
    nearest = np.abs(residuals).argmin(axis=1)
    errors = residuals[np.arange(len(points)), nearest]
    for axis in (0, 1):
        support = (nearest//2 == axis) & (np.abs(errors) < .018)
        if (np.count_nonzero(support) < 12 or np.ptp(shifted[support, 1-axis]) < .60
                or np.sqrt(np.mean(errors[support]**2)) > .010):
            raise RuntimeError('LiDAR room fit lacks consistent perpendicular wall support')
    return tuple(map(float, pose))
