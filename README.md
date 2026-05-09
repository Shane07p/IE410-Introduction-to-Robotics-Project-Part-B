# IE410 Project Part B — Simulation of a Theo Jansen Walking Mechanism

## Overview
Python simulation of the **12-link modified Jansen mechanism** for gait generation,
as used in single-DOF robotic gait trainers (Sulzer et al. 2018, Jadav et al. 2024).

The project simulates the mechanism kinematics, generates foot trajectory plots,
animations, sensitivity analysis, and compares with reference human gait data.

## Quick Start
```bash
pip install -r requirements.txt
python run_all.py
```
All outputs are saved to `output/`.

## Project Structure
| File | Description |
|------|-------------|
| `core/kinematics.py` | 12-link and 11-link Jansen solvers (CCI method) |
| `core/analysis.py` | Gait metrics, velocity, Fourier, Grashof check |
| `scripts/s01_foot_trajectory.py` | Foot trajectory plot with angle ticks |
| `scripts/s02_mechanism_animation.py` | Color-coded animated GIF |
| `scripts/s03_static_schematic.py` | Publication-quality schematic |
| `scripts/s05_stance_swing_analysis.py` | Stance/swing phase decomposition |
| `scripts/s06_sensitivity_analysis.py` | 2D sweep + 3D surface plots |
| `scripts/s10_full_report.py` | Auto-generated metrics report |

## Mechanism
The 12-link modified Jansen mechanism uses 3 adjustable links:
- **L1** = 11.29 cm (crank length)
- **L4** = 32.93 cm (ground pivot offset)
- **L8** = 41.78 cm (lower rocker)

## References
1. Sulzer et al., "Design of a Single-DOF, Customizable Gait Trainer", ASME JMRD, 2018.
2. Jadav et al., "Kinematic Performance of a Single-DOF Gait Trainer", SN Applied Sciences, 2024.
3. Jansen, T., "The Great Pretender", 010 Publishers, 2007.
