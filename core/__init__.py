"""
Jansen Mechanism — Core Package
================================
Provides kinematics solvers and gait analysis tools for both the
classic 11-link Theo Jansen mechanism and the 12-link modified
Jansen gait trainer.
"""

from .kinematics import JansenMechanism, JansenClassic
from .analysis import GaitAnalyzer
