from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline


class RandomSplineTrajectory:
    """Randomized smooth 3D path through waypoints sampled within bounds."""

    def __init__(
        self,
        bounds: tuple[np.ndarray, np.ndarray],
        num_waypoints: int = 5,
        duration_s: float = 25.0,
        seed: int | None = None,
    ) -> None:
        low, high = bounds
        rng = np.random.default_rng(seed)
        waypoints = rng.uniform(low, high, size=(num_waypoints, 3))
        self.duration_s = duration_s
        times = np.linspace(0.0, duration_s, num_waypoints)
        self._spline = CubicSpline(times, waypoints, axis=0, bc_type="clamped")

    def point_at(self, t: float) -> np.ndarray:
        t_clamped = min(max(t, 0.0), self.duration_s)
        return np.asarray(self._spline(t_clamped), dtype=np.float64)
