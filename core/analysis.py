"""
analysis.py — Gait Analysis Engine
====================================
Provides analysis of foot trajectories produced by the Jansen mechanism.

Metrics computed:
    - Stride length (X-span of foot trajectory)
    - Step height (Y-span of foot trajectory)
    - Area Under Closed Trajectory (AUCT) via Shoelace formula
    - Duty factor (stance phase fraction of total cycle)
    - Grashof condition check

References
----------
- Jadav et al., SN Applied Sciences, 2024 — AUCT, duty factor.
- Sulzer et al., ASME J. Mechanisms Robotics, 2018 — area error %.
"""

import numpy as np
from typing import Dict, Tuple


class GaitAnalyzer:
    """
    Service class for extracting gait performance metrics from
    a foot trajectory.
    """

    # ──────────────────────────────────────────────────────────────
    # Core metrics
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def compute_metrics(foot: np.ndarray) -> Dict[str, float]:
        """
        Compute primary gait metrics from the foot trajectory.

        Parameters
        ----------
        foot : (N, 2) array
            Foot-point X-Y positions (may contain NaNs).

        Returns
        -------
        dict with keys:
            x_span     — stride length (cm)
            y_span     — step height (cm)
            area       — enclosed area via Shoelace (cm²)
            duty_factor— fraction of cycle in stance phase
            x_min, x_max, y_min, y_max — bounding box
        """
        valid = ~np.isnan(foot[:, 0])
        x, y = foot[valid, 0], foot[valid, 1]

        if len(x) < 3:
            return {
                "x_span": 0.0, "y_span": 0.0, "area": 0.0,
                "duty_factor": 0.0, "x_min": 0.0, "x_max": 0.0,
                "y_min": 0.0, "y_max": 0.0,
            }

        # Shoelace area
        xc = np.append(x, x[0])
        yc = np.append(y, y[0])
        area = 0.5 * np.abs(np.sum(xc[:-1] * yc[1:] - xc[1:] * yc[:-1]))

        # Duty factor: bottom 15% of Y-range is stance phase
        y_threshold = y.min() + 0.15 * (y.max() - y.min())
        stance_frames = np.sum(y <= y_threshold)
        duty_factor = stance_frames / len(y)

        return {
            "x_span": float(x.max() - x.min()),
            "y_span": float(y.max() - y.min()),
            "area": float(area),
            "duty_factor": float(duty_factor),
            "x_min": float(x.min()),
            "x_max": float(x.max()),
            "y_min": float(y.min()),
            "y_max": float(y.max()),
        }

    # ──────────────────────────────────────────────────────────────
    # Stance / Swing Phase Decomposition
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def stance_swing_split(
        foot: np.ndarray,
        threshold_pct: float = 0.15,
    ) -> Tuple[np.ndarray, np.ndarray, dict]:
        """
        Decompose the foot trajectory into stance and swing phases.

        Parameters
        ----------
        foot : (N, 2) foot positions
        threshold_pct : float
            Bottom fraction of Y-range that counts as stance.

        Returns
        -------
        stance_mask : (N,) bool
        swing_mask  : (N,) bool
        info : dict
            duty_factor, stance_count, swing_count, y_threshold
        """
        valid = ~np.isnan(foot[:, 0])
        y = foot[:, 1].copy()

        # For NaN entries, treat as swing
        y[~valid] = np.inf

        y_min = np.nanmin(foot[valid, 1]) if valid.any() else 0
        y_max = np.nanmax(foot[valid, 1]) if valid.any() else 1
        y_thresh = y_min + threshold_pct * (y_max - y_min)

        stance_mask = (y <= y_thresh) & valid
        swing_mask = (~stance_mask) & valid

        duty_factor = stance_mask.sum() / valid.sum() if valid.any() else 0.0

        info = {
            "duty_factor": duty_factor,
            "stance_count": int(stance_mask.sum()),
            "swing_count": int(swing_mask.sum()),
            "y_threshold": float(y_thresh),
        }

        return stance_mask, swing_mask, info

    # ──────────────────────────────────────────────────────────────
    # Grashof Condition Check
    # ──────────────────────────────────────────────────────────────

    @staticmethod
    def grashof_check(links: Dict[str, float]) -> Tuple[bool, str]:
        """
        Check Grashof's criterion for the primary four-bar sub-loop
        of the 12-link mechanism.

        The primary input loop is P0-P1-P2-P3 with links:
            L1 (crank), L2 (coupler), L3 (rocker), L4 (ground).

        Grashof criterion: s + l ≤ p + q
        where s = shortest, l = longest, p, q = remaining links.

        Returns
        -------
        is_grashof : bool
        explanation : str
        """
        loop_links = {
            "L1": links.get("L1", 11.29),
            "L2": links.get("L2", 45.0),
            "L3": links.get("L3", 36.0),
            "L4": links.get("L4", 32.93),
        }

        vals = sorted(loop_links.values())
        s, p, q, l = vals[0], vals[1], vals[2], vals[3]

        is_grashof = (s + l) <= (p + q)

        if is_grashof:
            expl = (
                f"Grashof condition SATISFIED: s + l = {s:.2f} + {l:.2f} "
                f"= {s + l:.2f} <= p + q = {p:.2f} + {q:.2f} = {p + q:.2f}.\n"
                f"The shortest link ({s:.2f} cm) can fully rotate."
            )
        else:
            expl = (
                f"Grashof condition NOT satisfied: s + l = {s:.2f} + {l:.2f} "
                f"= {s + l:.2f} > p + q = {p:.2f} + {q:.2f} = {p + q:.2f}.\n"
                f"The mechanism is a non-Grashof (triple-rocker) linkage."
            )

        return is_grashof, expl
