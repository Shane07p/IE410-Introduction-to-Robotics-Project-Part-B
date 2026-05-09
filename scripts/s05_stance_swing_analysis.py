"""
Script 5 — Stance/Swing Phase Analysis
========================================
Decomposes the foot trajectory into stance (ground contact) and
swing (airborne) phases. Computes duty factor and visualizes
the phase decomposition.

Key insight from the papers: the Jansen mechanism naturally
produces a flat stance phase at the bottom of the trajectory
(ground contact) with a smooth lift-off and touch-down.

Output: output/stance_swing_analysis.png
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


def build():
    print("[S05] Generating stance/swing phase analysis ...")

    mech = JansenMechanism()
    n = 720
    thetas, foot, _ = mech.sweep(n)
    theta_deg = np.degrees(thetas)

    stance, swing, info = GaitAnalyzer.stance_swing_split(foot)
    metrics = GaitAnalyzer.compute_metrics(foot)

    # ── Figure ──
    fig = plt.figure(figsize=(14, 8))
    gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.35)

    # (0, 0:2) — Trajectory with phase colors
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(foot[swing, 0], foot[swing, 1], "o", color="#3498DB",
             ms=2.5, alpha=0.7, label="Swing phase", zorder=3)
    ax1.plot(foot[stance, 0], foot[stance, 1], "o", color="#E74C3C",
             ms=3, alpha=0.9, label="Stance phase", zorder=4)

    # Ground line
    ax1.axhline(info["y_threshold"], color="#95A5A6", ls="--", lw=1,
                label=f"Stance threshold (y = {info['y_threshold']:.1f} cm)")

    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)
    ax1.set_xlabel("x (cm)", fontsize=10)
    ax1.set_ylabel("y (cm)", fontsize=10)
    ax1.set_title("Foot Trajectory — Stance vs. Swing Phase", fontsize=12,
                  fontweight="bold")
    ax1.legend(fontsize=9, loc="upper right")

    # (0, 2) — Duty factor bar/pie
    ax2 = fig.add_subplot(gs[0, 2])
    duty = info["duty_factor"]
    colors = ["#E74C3C", "#3498DB"]
    labels = [f"Stance\n{duty*100:.1f}%", f"Swing\n{(1-duty)*100:.1f}%"]
    wedges, texts = ax2.pie(
        [duty, 1 - duty], labels=labels, colors=colors,
        startangle=90, wedgeprops=dict(width=0.5, edgecolor="white", linewidth=2),
        textprops=dict(fontsize=11, fontweight="bold")
    )
    ax2.set_title("Duty Factor", fontsize=12, fontweight="bold")

    # (1, 0) — Y-position vs crank angle (showing phase)
    ax3 = fig.add_subplot(gs[1, 0])
    valid = ~np.isnan(foot[:, 0])
    ax3.plot(theta_deg[valid], foot[valid, 1], color="#2C3E50", lw=1, alpha=0.5)
    ax3.fill_between(theta_deg, info["y_threshold"],
                     np.where(valid, foot[:, 1], np.nan),
                     where=stance, alpha=0.3, color="#E74C3C",
                     label="Stance")
    ax3.axhline(info["y_threshold"], color="#E74C3C", ls="--", lw=1)
    ax3.set_xlabel("Crank angle (°)", fontsize=10)
    ax3.set_ylabel("y position (cm)", fontsize=10)
    ax3.set_title("Y-Position vs. Crank Angle", fontsize=11)
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(0, 360)
    ax3.legend(fontsize=8)

    # (1, 1) — X-position vs crank angle
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(theta_deg[valid], foot[valid, 0], color="#2C3E50", lw=1.2)
    ax4.fill_between(theta_deg, foot[:, 0].min() if valid.any() else 0,
                     np.where(valid, foot[:, 0], np.nan),
                     where=stance, alpha=0.15, color="#E74C3C")
    ax4.set_xlabel("Crank angle (°)", fontsize=10)
    ax4.set_ylabel("x position (cm)", fontsize=10)
    ax4.set_title("X-Position vs. Crank Angle", fontsize=11)
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(0, 360)

    # (1, 2) — Metrics summary box
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.axis("off")
    summary = (
        f"━━━ Gait Metrics ━━━\n\n"
        f"Stride length:  {metrics['x_span']:.1f} cm\n"
        f"Step height:    {metrics['y_span']:.1f} cm\n"
        f"Enclosed area:  {metrics['area']:.1f} cm²\n"
        f"Duty factor:    {info['duty_factor']*100:.1f}%\n\n"
        f"Stance frames:  {info['stance_count']} / {info['stance_count'] + info['swing_count']}\n"
        f"Swing frames:   {info['swing_count']} / {info['stance_count'] + info['swing_count']}\n\n"
        f"Y threshold:    {info['y_threshold']:.1f} cm\n"
        f"(bottom 15% of Y-range)"
    )
    ax5.text(0.1, 0.95, summary, transform=ax5.transAxes,
             fontsize=10, fontfamily="monospace", verticalalignment="top",
             bbox=dict(boxstyle="round,pad=0.6", facecolor="#FDEBD0",
                       edgecolor="#E67E22", alpha=0.9))

    fig.suptitle(
        "Stance / Swing Phase Decomposition — 12-Link Jansen Mechanism",
        fontsize=14, fontweight="bold", y=1.01
    )

    plt.tight_layout()
    out_path = os.path.join(OUT_DIR, "stance_swing_analysis.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      saved: {out_path}")
    print(f"      duty factor = {info['duty_factor']*100:.1f}%")


if __name__ == "__main__":
    build()
