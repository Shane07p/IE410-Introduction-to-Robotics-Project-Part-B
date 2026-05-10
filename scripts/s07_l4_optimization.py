"""
Script 7 — L4 Optimisation
============================
Sweeps L4 over 25–55 cm to find the value that minimises
|step_height − 12.81 cm| (Jadav 2024 target).

Output:
  - output/l4_optimization.png
"""
import os, sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from core.kinematics import JansenMechanism
from core.analysis import GaitAnalyzer

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "output")
os.makedirs(OUT, exist_ok=True)

TARGET_HEIGHT = 12.81   # cm (Jadav et al. 2024)
L4_BASELINE   = JansenMechanism.DEFAULT_LINKS["L4"]  # 32.93 cm


def build():
    print("[S07] Running L4 optimisation sweep ...")

    l4_values      = np.linspace(25, 55, 150)
    step_heights   = []
    stride_lengths = []

    for l4 in l4_values:
        mech = JansenMechanism(custom_links={"L4": l4})
        _, foot, _ = mech.sweep(720)
        mt = GaitAnalyzer.compute_metrics(foot)
        step_heights.append(mt["y_span"])
        stride_lengths.append(mt["x_span"])

    step_heights   = np.array(step_heights)
    stride_lengths = np.array(stride_lengths)

    # Baseline values
    idx_base = int(np.argmin(np.abs(l4_values - L4_BASELINE)))
    h_base   = step_heights[idx_base]
    s_base   = stride_lengths[idx_base]

    # Optimal L4
    idx_opt = int(np.argmin(np.abs(step_heights - TARGET_HEIGHT)))
    l4_opt  = l4_values[idx_opt]
    h_opt   = step_heights[idx_opt]
    s_opt   = stride_lengths[idx_opt]

    print(f"      Baseline : L4 = {L4_BASELINE:.2f} cm  ->  step height = {h_base:.2f} cm  (stride {s_base:.2f} cm)")
    print(f"      Target   :                         step height = {TARGET_HEIGHT:.2f} cm")
    print(f"      Optimal  : L4 = {l4_opt:.2f} cm  ->  step height = {h_opt:.2f} cm  (stride {s_opt:.2f} cm)")
    improvement = (h_base - h_opt) / h_base * 100
    print(f"      Improvement: {improvement:.1f}% reduction in step height")

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("L4 Optimisation: Matching Target Step Height (Jadav 2024)",
                 fontsize=14, fontweight="bold")

    # Left: step height vs L4
    ax1.plot(l4_values, step_heights, color="#3498DB", lw=2.2, zorder=3,
             label="Step height")
    ax1.axhline(TARGET_HEIGHT, color="#E74C3C", lw=1.6, ls="--", zorder=2,
                label=f"Target {TARGET_HEIGHT} cm (Jadav 2024)")
    ax1.axvline(L4_BASELINE, color="#95A5A6", lw=1.2, ls=":", zorder=2,
                label=f"Baseline L4 = {L4_BASELINE:.2f} cm")
    ax1.axvline(l4_opt, color="#2ECC71", lw=1.6, ls="-.", zorder=4,
                label=f"Optimal L4 = {l4_opt:.2f} cm")
    ax1.plot(l4_opt, h_opt, "o", color="#2ECC71", ms=9,
             markeredgecolor="white", markeredgewidth=1.2, zorder=5)
    ax1.plot(L4_BASELINE, h_base, "s", color="#95A5A6", ms=8,
             markeredgecolor="white", markeredgewidth=1, zorder=5)
    ax1.set_xlabel("L4 (cm)", fontsize=11)
    ax1.set_ylabel("Step Height (cm)", fontsize=11)
    ax1.set_title("Step Height vs L4", fontweight="bold")
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # Right: stride length vs L4
    ax2.plot(l4_values, stride_lengths, color="#E67E22", lw=2.2, zorder=3,
             label="Stride length")
    ax2.axvline(L4_BASELINE, color="#95A5A6", lw=1.2, ls=":", zorder=2,
                label=f"Baseline L4 = {L4_BASELINE:.2f} cm")
    ax2.axvline(l4_opt, color="#2ECC71", lw=1.6, ls="-.", zorder=4,
                label=f"Optimal L4 = {l4_opt:.2f} cm")
    ax2.plot(l4_opt, s_opt, "o", color="#2ECC71", ms=9,
             markeredgecolor="white", markeredgewidth=1.2, zorder=5)
    ax2.plot(L4_BASELINE, s_base, "s", color="#95A5A6", ms=8,
             markeredgecolor="white", markeredgewidth=1, zorder=5)
    ax2.set_xlabel("L4 (cm)", fontsize=11)
    ax2.set_ylabel("Stride Length (cm)", fontsize=11)
    ax2.set_title("Stride Length vs L4", fontweight="bold")
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    out_path = os.path.join(OUT, "l4_optimization.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"      saved: {out_path}")

    return l4_opt, h_opt, s_opt


if __name__ == "__main__":
    build()
