"""
Script 3 — Static Schematic Diagram
=====================================
Clean, publication-quality schematic of the 12-link modified Jansen
mechanism at a representative pose (foot at lowest point).

Features:
  - All joints labeled (P0–P6, PE)
  - Link lengths annotated on selected links
  - Ground level dashed line
  - Foot trajectory overlay
  - Color-coded adjustable links (L1, L4, L8)

Output: output/static_schematic.png
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.kinematics import JansenMechanism
from core.analysis import GaitAnalyzer

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)

# Joint label names matching the 8-joint array
JOINT_NAMES = ["P₀", "P₁", "P₂", "P₃", "P₄", "P₅", "P₆", "Pₑ"]

# Label offsets for each joint (avoid overlapping links)
LABEL_OFFSETS = [
    (-12, -8),   # P0
    (-5, 8),     # P1
    (-5, 8),     # P2
    (6, -8),     # P3
    (6, 4),      # P4
    (6, -8),     # P5
    (6, 4),      # P6
    (-5, -12),   # PE
]

# Which edges correspond to the 3 adjustable links (highlight them)
ADJUSTABLE = {0: "L1", 3: "L4", 7: "L8"}  # edge index → link name


def build():
    print("[S03] Generating static schematic ...")

    mech = JansenMechanism()
    thetas, foot, joints = mech.sweep(720)

    # Pick pose at lowest foot point
    valid_foot = foot.copy()
    valid_foot[np.isnan(valid_foot[:, 0])] = np.inf
    pose_idx = np.argmin(valid_foot[:, 1])
    J = joints[pose_idx].copy()

    # Shift so foot is centered horizontally, ground at y=0
    foot_valid = foot[~np.isnan(foot[:, 0])]
    foot_shifted = foot_valid.copy()
    shift_x = -J[7, 0] + np.mean(foot_valid[:, 0]) * 0.0  # keep original
    shift_y = -foot_valid[:, 1].min()
    J[:, 1] += shift_y
    foot_shifted[:, 0] = foot_valid[:, 0]
    foot_shifted[:, 1] = foot_valid[:, 1] + shift_y

    # ── Figure ──
    fig, ax = plt.subplots(figsize=(9, 10))
    ax.set_aspect("equal")

    # Ground level
    ax.axhline(0, color="#95A5A6", linestyle="--", lw=1.5, alpha=0.7, zorder=1)
    ax.text(foot_shifted[:, 0].min() - 2, -2.5, "Ground Level",
            fontsize=11, color="#7F8C8D", fontweight="bold")

    # Foot trajectory (light)
    ax.plot(foot_shifted[:, 0], foot_shifted[:, 1],
            color="#E74C3C", lw=2, alpha=0.4, zorder=2, label="Foot trajectory")

    # Draw links
    for edge_idx, (i, j) in enumerate(mech.EDGES):
        color = "#2C3E50"  # default dark
        lw = 2.0
        ls = "-"

        if edge_idx in ADJUSTABLE:
            color = "#E74C3C"  # red for adjustable
            lw = 3.0

        ax.plot([J[i, 0], J[j, 0]], [J[i, 1], J[j, 1]],
                color=color, lw=lw, ls=ls, solid_capstyle="round", zorder=3)

        # Annotate link length at midpoint for adjustable links
        if edge_idx in ADJUSTABLE:
            mx = (J[i, 0] + J[j, 0]) / 2
            my = (J[i, 1] + J[j, 1]) / 2
            link_name = ADJUSTABLE[edge_idx]
            ax.annotate(
                f"{link_name} = {mech.L[link_name]:.2f}",
                (mx, my), fontsize=8, color="#E74C3C",
                fontweight="bold",
                bbox=dict(facecolor="white", edgecolor="#E74C3C",
                          boxstyle="round,pad=0.2", alpha=0.85),
                ha="center", va="center", zorder=7
            )

    # Joint markers and labels
    for k in range(8):
        if k in (0, 3):  # Fixed pivots
            ax.plot(J[k, 0], J[k, 1], "s", color="#E74C3C", ms=10,
                    markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        elif k == 7:  # Foot
            ax.plot(J[k, 0], J[k, 1], "D", color="#E74C3C", ms=10,
                    markeredgecolor="white", markeredgewidth=1.2, zorder=5)
        else:
            ax.plot(J[k, 0], J[k, 1], "o", color="#E74C3C", ms=8,
                    markeredgecolor="white", markeredgewidth=1, zorder=5)

        ox, oy = LABEL_OFFSETS[k]
        ax.annotate(JOINT_NAMES[k], (J[k, 0], J[k, 1]),
                    xytext=(ox, oy), textcoords="offset points",
                    fontsize=11, fontweight="bold", color="#2C3E50",
                    zorder=8)

    # Legend for adjustable vs fixed
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color="#E74C3C", lw=3, label="Adjustable links (L1, L4, L8)"),
        Line2D([0], [0], color="#2C3E50", lw=2, label="Fixed links"),
        Line2D([0], [0], marker="s", color="#E74C3C", lw=0, ms=10,
               markeredgecolor="white", label="Fixed pivots"),
        Line2D([0], [0], marker="D", color="#E74C3C", lw=0, ms=10,
               markeredgecolor="white", label="Foot point (PE)"),
        Line2D([0], [0], color="#E74C3C", lw=2, alpha=0.4, label="Foot trajectory"),
    ]
    ax.legend(handles=legend_elements, loc="upper right", fontsize=9,
              framealpha=0.9)

    ax.axis("off")
    ax.set_title("12-Link Modified Jansen Mechanism — Schematic",
                 fontsize=14, fontweight="bold", loc="left", pad=15)

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "static_schematic.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"      saved: {out_path}")


if __name__ == "__main__":
    build()
