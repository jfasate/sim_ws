"""
Utility functions for LPV-MPC waypoint tracking.
nearest_point() adapted from f1tenth_planning (MIT License).
"""

import numpy as np


def nearest_point(point, trajectory):
    """Return the nearest point on a piecewise-linear trajectory.

    Args:
        point: (2,) array [x, y]
        trajectory: (N, 2) array of [x, y] waypoints

    Returns:
        nearest_point: (2,) closest point on trajectory
        nearest_dist: float distance
        t: interpolation parameter on the closest segment
        i: index of closest segment start
    """
    diffs = trajectory[1:, :] - trajectory[:-1, :]
    l2s = diffs[:, 0]**2 + diffs[:, 1]**2
    dots = np.sum((point - trajectory[:-1, :]) * diffs, axis=1)
    t = np.clip(dots / l2s, 0.0, 1.0)
    projections = trajectory[:-1, :] + (t[:, np.newaxis] * diffs)
    dists = np.linalg.norm(point - projections, axis=1)
    min_idx = np.argmin(dists)
    return projections[min_idx], dists[min_idx], t[min_idx], min_idx


def precompute_segments(trajectory):
    """Precompute the per-segment vectors and squared lengths for a *closed*
    loop, including the wrap-around segment (last -> first).

    Returns:
        diffs: (N, 2) segment vectors, diffs[i] = traj[(i+1) % N] - traj[i]
        l2s:   (N,) squared segment lengths (with a tiny epsilon for safety)
    """
    nxt = np.roll(trajectory, -1, axis=0)
    diffs = nxt - trajectory
    l2s = diffs[:, 0]**2 + diffs[:, 1]**2 + 1e-12
    return diffs, l2s


def nearest_point_windowed(point, trajectory, diffs, l2s, seed_idx, back, fwd):
    """Nearest point on a closed piecewise-linear loop, searching only a local
    window of segments around ``seed_idx`` (with wrap-around).

    The full O(N) scan in ``nearest_point`` is replaced by an O(back+fwd) scan:
    the car only advances a fraction of a waypoint per control tick, so the true
    nearest segment is always within a small window of the previous one. The
    window is also robust on tracks that pass near themselves, where a global
    argmin could otherwise snap to the wrong lap section.

    ``diffs``/``l2s`` come from :func:`precompute_segments` (built once) so no
    per-call segment arithmetic over the whole track is needed.

    Returns: (projection, dist, t, abs_idx) — same shape as ``nearest_point``,
    with ``abs_idx`` the absolute segment-start index into ``trajectory``.
    """
    n = trajectory.shape[0]
    idxs = np.arange(seed_idx - back, seed_idx + fwd + 1) % n
    seg_start = trajectory[idxs]
    d = diffs[idxs]
    t = np.clip(np.sum((point - seg_start) * d, axis=1) / l2s[idxs], 0.0, 1.0)
    proj = seg_start + t[:, np.newaxis] * d
    dists = np.einsum('ij,ij->i', point - proj, point - proj)  # squared dist
    k = np.argmin(dists)
    return proj[k], np.sqrt(dists[k]), t[k], idxs[k]
