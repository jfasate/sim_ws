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
