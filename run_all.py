"""run_all.py -- Execute all scripts in order."""
import os, sys, runpy, time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

scripts = [
    "scripts/s01_foot_trajectory.py",
    "scripts/s02_mechanism_animation.py",
    "scripts/s03_static_schematic.py",
    "scripts/s05_stance_swing_analysis.py",
    "scripts/s06_sensitivity_analysis.py",
    "scripts/s10_full_report.py",
]

if __name__ == "__main__":
    print("=" * 60)
    print("  Theo Jansen Walking Mechanism - Full Simulation Suite")
    print("  IE410: Introduction to Robotics - Winter 2026")
    print("=" * 60)
    t0 = time.time()
    for s in scripts:
        print(f"\n{'-'*55}")
        path = os.path.join(HERE, s)
        try:
            runpy.run_path(path, run_name="__main__")
        except Exception as e:
            print(f"  ERROR in {s}: {e}")
    elapsed = time.time() - t0
    print(f"\n{'='*60}")
    print(f"  All done! Total time: {elapsed:.1f}s")
    print(f"  Outputs saved in: {os.path.join(HERE, 'output')}")
    print(f"{'='*60}")
