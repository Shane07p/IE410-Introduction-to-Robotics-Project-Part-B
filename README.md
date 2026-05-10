# IE410 Project Part B — Simulation of a Theo Jansen Walking Mechanism

## Overview
Python simulation of the **12-link modified Jansen mechanism** for gait generation,
as used in single-DOF robotic gait trainers (Sulzer et al. 2018, Jadav et al. 2024).

The project simulates the mechanism kinematics, generates foot trajectory plots,
animations, sensitivity analysis, L4 optimisation, and compares with reference human gait data.

## Quick Start
```bash
pip install -r requirements.txt
python run_all.py
```
All outputs are saved to `output/`.

## Project Structure
| File | Description |
|------|-------------|
| `core/kinematics.py` | 12-link Jansen solver (CCI method) |
| `core/analysis.py` | Gait metrics, Grashof check, stance/swing split |
| `scripts/s01_foot_trajectory.py` | Foot trajectory plot with angle ticks |
| `scripts/s02_mechanism_animation.py` | Color-coded animated GIF |
| `scripts/s03_static_schematic.py` | Publication-quality schematic |
| `scripts/s05_stance_swing_analysis.py` | Stance/swing phase decomposition |
| `scripts/s06_sensitivity_analysis.py` | Link length sensitivity sweep |
| `scripts/s07_l4_optimization.py` | L4 parameter optimisation to match target step height |
| `scripts/s10_full_report.py` | Auto-generated metrics report |
| `demo.py` | Live animation demo for screen recording |
| `report.tex` | Full LaTeX project report |

## Mechanism
The 12-link modified Jansen mechanism uses 3 adjustable links:
- **L1** = 11.29 cm (crank length)
- **L4** = 32.93 cm (ground pivot offset)
- **L8** = 41.78 cm (lower rocker)

## L4 Optimisation
A parameter sweep over L4 (25–55 cm) finds the value that minimises the difference
from the Jadav 2024 target step height of 12.81 cm.

| | L4 | Step Height | Stride |
|---|---|---|---|
| Baseline | 32.93 cm | 19.23 cm | 48.04 cm |
| **Optimal** | **35.47 cm** | **12.75 cm** | 45.02 cm |
| Target (Jadav 2024) | — | 12.81 cm | — |

A 2.54 cm increase in L4 reduces step height by 33.7% and matches the reference target within 0.06 cm.

## References
1. Sulzer et al., "Design of a Single-DOF, Customizable Gait Trainer", ASME JMRD, 2018.
2. Jadav et al., "Kinematic Performance of a Single-DOF Gait Trainer", SN Applied Sciences, 2024.
