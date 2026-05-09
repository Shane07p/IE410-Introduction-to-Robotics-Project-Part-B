"""
Script 1 — Foot Trajectory Plot
=================================
Plots the closed foot-tip trajectory of the 12-link modified Jansen
mechanism over one full crank revolution, with crank-angle tick marks,
stride/step/area annotations, and stance-phase highlighting.

Output: output/foot_trajectory.png
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.kinematics import JansenMechanism
from core.analysis import GaitAnalyzer

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)


def build():
    print("[S01] Generating foot trajectory plot ...")

    mech = JansenMechanism()
    thetas, foot, _ = mech.sweep(n=720)
    metrics = GaitAnalyzer.compute_metrics(foot)

    valid = ~np.isnan(foot[:, 0])
    fx, fy = foot[valid, 0], foot[valid, 1]

    # ── Figure ──
    fig, ax = plt.subplots(figsize=(10, 5.5))

    # Full trajectory
    ax.plot(fx, fy, color="#E74C3C", lw=2.2, label="Foot trajectory", zorder=3)

    # Stance phase (bottom 15% of Y range)
    stance, swing, info = GaitAnalyzer.stance_swing_split(foot)
    ax.plot(foot[stance, 0], foot[stance, 1],
            color="#3498DB", lw=4.5, alpha=0.45,
            label=f"Stance phase (duty = {info['duty_factor']*100:.1f}%)",
            zorder=2)

    # Crank-angle tick marks every 30°
    n_total = len(thetas)
    for deg in range(0, 360, 30):
        idx = int(deg / 360 * n_total)
        if not np.isnan(foot[idx, 0]):
            ax.plot(foot[idx, 0], foot[idx, 1], "o",
                    color="#2C3E50", ms=5, zorder=4)
            # Offset text smartly
            ox, oy = 5, 5
            if deg > 180:
                oy = -10
            ax.annotate(f"{deg}°", (foot[idx, 0], foot[idx, 1]),
                        xytext=(ox, oy), textcoords="offset points",
                        fontsize=7.5, color="#2C3E50")

    # Start marker
    ax.plot(foot[0, 0], foot[0, 1], "o", color="#27AE60", ms=10,
            label="Start (θ = 0°)", zorder=5)

    # Ground level line
    ax.axhline(metrics["y_min"], color="gray", ls=":", lw=0.8, alpha=0.6)
    ax.text(metrics["x_min"] + 0.5, metrics["y_min"] - 1.2,
            f"ground level (y ≈ {metrics['y_min']:.1f} cm)",
            fontsize=8, color="gray")

    # Annotation box
    info_text = (
        f"Stride length: {metrics['x_span']:.1f} cm\n"
        f"Step height:   {metrics['y_span']:.1f} cm\n"
        f"Enclosed area: {metrics['area']:.1f} cm²\n"
        f"Duty factor:   {info['duty_factor']*100:.1f}%"
    )
    ax.text(0.02, 0.97, info_text, transform=ax.transAxes,
            verticalalignment="top", fontsize=10,
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#FFF9C4",
                      edgecolor="#F9A825", alpha=0.9),
            fontfamily="monospace")

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (cm)", fontsize=11)
    ax.set_ylabel("y (cm)", fontsize=11)
    ax.set_title(
        f"Foot Trajectory — 12-Link Modified Jansen Mechanism\n"
        f"(L1 = {mech.L['L1']:.2f} cm,  L4 = {mech.L['L4']:.2f} cm,  "
        f"L8 = {mech.L['L8']:.2f} cm)",
        fontsize=12, fontweight="bold"
    )
    ax.legend(loc="upper right", fontsize=9)
    plt.tight_layout()

    out_path = os.path.join(OUT_DIR, "foot_trajectory.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      saved: {out_path}")
    print(f"      stride = {metrics['x_span']:.1f} cm, "
          f"step height = {metrics['y_span']:.1f} cm, "
          f"area = {metrics['area']:.1f} cm²")


if __name__ == "__main__":
    build()
