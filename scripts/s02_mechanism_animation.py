"""
Script 2 — Mechanism Animation
================================
Animated GIF of the 12-link modified Jansen mechanism with:
  - Color-coded links by sub-assembly
  - Dark background for visual impact
  - Crank angle badge
  - Progressive foot trace build-up
  - Fixed/moving pivot markers

Output: output/mechanism_animation.gif
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import io

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.kinematics import JansenMechanism

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
os.makedirs(OUT_DIR, exist_ok=True)

BG_COLOR = "#0A0E17"
GRID_COLOR = "#1A2332"


def render_frame(joints_frame, foot_history, ax_lim, theta_deg, mech):
    """Render one animation frame and return as PIL Image."""
    fig, ax = plt.subplots(figsize=(8, 8), facecolor=BG_COLOR)
    ax.set_facecolor(BG_COLOR)

    # Foot trail
    if len(foot_history) > 1:
        fh = np.array(foot_history)
        ax.plot(fh[:, 0], fh[:, 1], "-", color="#FF6B9D",
                lw=2.0, alpha=0.85, zorder=2)

    # Draw links by sub-assembly groups
    J = joints_frame
    for name, colour, path in mech.ASSEMBLY_GROUPS:
        xs = [J[k, 0] for k in path]
        ys = [J[k, 1] for k in path]
        ax.plot(xs, ys, "-", color=colour, lw=4, solid_capstyle="round",
                zorder=3, alpha=0.9)

    # Joint markers
    for k in range(8):
        if k in (0, 3):  # Fixed pivots
            ax.plot(J[k, 0], J[k, 1], "D", color="#E8588E",
                    ms=9, markeredgecolor="white", markeredgewidth=1.2,
                    zorder=6)
        elif k == 7:  # Foot
            ax.plot(J[k, 0], J[k, 1], "o", color="#F1C40F",
                    ms=8, markeredgecolor="white", markeredgewidth=1,
                    zorder=6)
        else:
            ax.plot(J[k, 0], J[k, 1], "o", color="white",
                    ms=5, markeredgecolor="#2C3E50", markeredgewidth=0.8,
                    zorder=5)

    # Crank angle badge
    ax.text(0.04, 0.95, f"Crank Angle: {theta_deg:.1f}°",
            transform=ax.transAxes, color="#7CDB9A", fontsize=14,
            fontweight="bold", verticalalignment="top",
            fontfamily="monospace",
            bbox=dict(facecolor="#1A2438", edgecolor="#7CDB9A",
                      boxstyle="round,pad=0.5", linewidth=1.5))

    # Link lengths info
    ax.text(0.04, 0.06,
            f"L1={mech.L['L1']:.2f}  L4={mech.L['L4']:.2f}  L8={mech.L['L8']:.2f} cm",
            transform=ax.transAxes, color="#95A5A6", fontsize=9,
            fontfamily="monospace",
            bbox=dict(facecolor="#1A2438", edgecolor="#34495E",
                      boxstyle="round,pad=0.3", linewidth=0.8))

    ax.set_xlim(ax_lim[0], ax_lim[1])
    ax.set_ylim(ax_lim[2], ax_lim[3])
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.15, color=GRID_COLOR, linestyle="--")
    ax.tick_params(colors="#95A5A6")
    for sp in ax.spines.values():
        sp.set_color("#34495E")
    ax.set_xlabel("X Position (cm)", color="#95A5A6", fontsize=10)
    ax.set_ylabel("Y Position (cm)", color="#95A5A6", fontsize=10)
    ax.set_title("12-Link Modified Jansen Mechanism",
                 color="white", fontsize=13, fontweight="bold")

    fig.tight_layout()

    # Convert to PIL Image
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=90, facecolor=BG_COLOR,
                bbox_inches="tight")
    plt.close(fig)
    buf.seek(0)
    return Image.open(buf).convert("RGB")


def build():
    print("[S02] Generating mechanism animation ...")

    mech = JansenMechanism()
    n = 180  # 2° per frame
    thetas, foot, joints = mech.sweep(n)

    # Compute axis limits from all joints
    all_pts = joints.reshape(-1, 2)
    all_pts = all_pts[~np.isnan(all_pts[:, 0])]
    valid_foot = foot[~np.isnan(foot[:, 0])]
    all_pts = np.vstack([all_pts, valid_foot])
    pad = 8
    xlim = (all_pts[:, 0].min() - pad, all_pts[:, 0].max() + pad)
    ylim = (all_pts[:, 1].min() - pad, all_pts[:, 1].max() + pad)
    ax_lim = (xlim[0], xlim[1], ylim[0], ylim[1])

    # Render frames
    n_frames = 90  # subsample for manageable file size
    step = max(1, n // n_frames)
    frames = []
    foot_history = []

    for i in range(0, n, step):
        if np.isnan(joints[i, 0, 0]):
            continue
        if not np.isnan(foot[i, 0]):
            foot_history.append(foot[i])
        theta_deg = np.degrees(thetas[i])
        img = render_frame(joints[i], foot_history, ax_lim, theta_deg, mech)
        frames.append(img)

    if frames:
        gif_path = os.path.join(OUT_DIR, "mechanism_animation.gif")
        frames[0].save(
            gif_path, save_all=True, append_images=frames[1:],
            duration=60, loop=0, optimize=True
        )
        print(f"      saved: {gif_path}  ({len(frames)} frames)")
    else:
        print("      ERROR: No valid frames generated.")


if __name__ == "__main__":
    build()
