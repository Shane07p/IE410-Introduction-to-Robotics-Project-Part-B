"""
Script 6 — Sensitivity Analysis (2D Sweep Only)
=================================================
Sweeps the three adjustable links (L1, L4, L8) individually
to show how they affect the foot trajectory shape, stride,
step height, and duty factor.

Output:
  - output/sensitivity_2d.png
  - output/sensitivity_data.txt
"""
import os, sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.kinematics import JansenMechanism
from core.analysis import GaitAnalyzer

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
os.makedirs(OUT, exist_ok=True)
LINKS = ["L1", "L4", "L8"]
SCALES = [0.85, 0.90, 0.95, 1.00, 1.05, 1.10, 1.15]


def build():
    print("[S06] Generating sensitivity analysis ...")

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5))
    log = []

    for ax, name in zip(axes, LINKS):
        cmap = plt.cm.viridis(np.linspace(0.05, 0.95, len(SCALES)))
        base = JansenMechanism.DEFAULT_LINKS[name]

        for sc, col in zip(SCALES, cmap):
            val = base * sc
            mech = JansenMechanism(custom_links={name: val})
            _, foot, _ = mech.sweep(720)
            mt = GaitAnalyzer.compute_metrics(foot)
            v = ~np.isnan(foot[:, 0])
            lw = 2.8 if abs(sc - 1.0) < 1e-6 else 1.3
            ax.plot(foot[v, 0], foot[v, 1], color=col, lw=lw,
                    label=f"{sc:.2f}x ({val:.2f})")
            _, _, info = GaitAnalyzer.stance_swing_split(foot)
            log.append((name, sc, val, mt["x_span"], mt["y_span"],
                        mt["area"], info["duty_factor"]))

        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.set_title(f"Vary {name} (default {base:.2f} cm)",
                     fontsize=11, fontweight="bold")
        ax.set_xlabel("x (cm)")
        ax.set_ylabel("y (cm)")
        ax.legend(fontsize=7, title="Scale", loc="upper right")

    fig.suptitle(
        "Effect of Adjustable Links on Foot Trajectory & Gait",
        fontsize=14, fontweight="bold", y=1.02
    )
    plt.tight_layout()
    out_2d = os.path.join(OUT, "sensitivity_2d.png")
    plt.savefig(out_2d, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      saved: {out_2d}")

    # Write metrics table
    txt_path = os.path.join(OUT, "sensitivity_data.txt")
    with open(txt_path, "w") as f:
        header = (f"{'Link':>5} {'Scale':>6} {'Value':>8} "
                  f"{'Stride':>8} {'Height':>8} {'Area':>8} {'Duty%':>8}")
        f.write(header + "\n")
        f.write("-" * len(header) + "\n")
        for name, sc, val, xs, ys, ar, df in log:
            f.write(f"{name:>5} {sc:>6.2f} {val:>8.2f} "
                    f"{xs:>8.2f} {ys:>8.2f} {ar:>8.2f} {df*100:>7.1f}%\n")
    print(f"      saved: {txt_path}")

    # Print summary
    for name in LINKS:
        entries = [(sc, xs, ys) for n, sc, _, xs, ys, _, _ in log if n == name]
        print(f"      {name}: stride range "
              f"[{min(e[1] for e in entries):.1f}, {max(e[1] for e in entries):.1f}] cm, "
              f"height range "
              f"[{min(e[2] for e in entries):.1f}, {max(e[2] for e in entries):.1f}] cm")


if __name__ == "__main__":
    build()
