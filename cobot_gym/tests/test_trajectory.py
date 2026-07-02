import numpy as np
import pytest

from cobot_gym.trajectory import RandomSplineTrajectory


BOUNDS = (np.array([-0.5, -0.5, 0.2]), np.array([0.5, 0.5, 0.8]))


def test_point_at_start_and_end_are_within_bounds():
    traj = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=42)
    start = traj.point_at(0.0)
    end = traj.point_at(10.0)
    for p in (start, end):
        assert p.shape == (3,)
        assert np.all(p >= BOUNDS[0] - 1e-6)
        assert np.all(p <= BOUNDS[1] + 1e-6)


def test_point_at_is_deterministic_for_same_seed():
    traj_a = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=7)
    traj_b = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=7)
    for t in (0.0, 2.5, 6.1, 10.0):
        np.testing.assert_allclose(traj_a.point_at(t), traj_b.point_at(t))


def test_different_seeds_produce_different_paths():
    traj_a = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=1)
    traj_b = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=2)
    assert not np.allclose(traj_a.point_at(5.0), traj_b.point_at(5.0))


def test_point_at_clamps_outside_duration():
    traj = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=3)
    np.testing.assert_allclose(traj.point_at(-5.0), traj.point_at(0.0))
    np.testing.assert_allclose(traj.point_at(999.0), traj.point_at(10.0))


def test_path_is_smooth_no_large_jumps_between_close_samples():
    traj = RandomSplineTrajectory(BOUNDS, num_waypoints=5, duration_s=10.0, seed=4)
    ts = np.linspace(0, 10, 200)
    pts = np.array([traj.point_at(t) for t in ts])
    diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    assert diffs.max() < 0.05
