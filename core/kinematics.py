"""
kinematics.py — Forward Kinematics Solvers
============================================
Two mechanism models:

1. **JansenMechanism** (12-link, Shin et al. / Jadav et al.)
   - Modified Jansen used as a gait trainer.
   - Three adjustable link lengths: L1 (crank), L4 (ground offset),
     L8 (lower rocker).
   - 12 links labelled L1–L12, 8 joints labelled P0–P6 + PE.

2. **JansenClassic** (11-link, Theo Jansen "holy numbers")
   - The original Strandbeest leg with published optimal proportions.
   - 11 links labelled a–m, 7 joints labelled O, M, A, B, C, D, G.

Both solvers use the circle-circle-intersection (CCI) method to
resolve each dyad.

References
----------
- Sulzer et al., ASME J. Mechanisms Robotics 10(4), 2018.
- Jadav et al., SN Applied Sciences, 2024.
- Jansen, T., "The Great Pretender", 010 Publishers, 2007.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple, List


# ──────────────────────────────────────────────────────────────────────
# Data classes for joint poses
# ──────────────────────────────────────────────────────────────────────

@dataclass
class JointPose12:
    """Joint positions for the 12-link modified Jansen mechanism."""
    P0: np.ndarray   # Fixed crank pivot
    P1: np.ndarray   # Crank tip
    P2: np.ndarray   # Upper knee
    P3: np.ndarray   # Fixed ground pivot
    P4: np.ndarray   # Upper triangle apex
    P5: np.ndarray   # Lower knee
    P6: np.ndarray   # Lower triangle apex
    PE: np.ndarray   # Foot / end-effector

    NAMES: list = field(default_factory=lambda: [
        "P0", "P1", "P2", "P3", "P4", "P5", "P6", "PE"
    ], repr=False)

    def as_array(self) -> np.ndarray:
        """Return (8, 2) array of all joint positions."""
        return np.array([self.P0, self.P1, self.P2, self.P3,
                         self.P4, self.P5, self.P6, self.PE])


@dataclass
class JointPose11:
    """Joint positions for the classic 11-link Jansen leg."""
    O: np.ndarray    # Fixed crank pivot
    M: np.ndarray    # Fixed ground pivot
    A: np.ndarray    # Crank tip
    B: np.ndarray    # Upper knee
    C: np.ndarray    # Lower knee
    D: np.ndarray    # Triangle apex
    G: np.ndarray    # Foot tip

    NAMES: list = field(default_factory=lambda: [
        "O", "M", "A", "B", "C", "D", "G"
    ], repr=False)

    def as_array(self) -> np.ndarray:
        """Return (7, 2) array of all joint positions."""
        return np.array([self.O, self.M, self.A, self.B,
                         self.C, self.D, self.G])


# ──────────────────────────────────────────────────────────────────────
# Circle-Circle Intersection (CCI)
# ──────────────────────────────────────────────────────────────────────

def circle_circle_intersect(
    c1: np.ndarray, r1: float,
    c2: np.ndarray, r2: float,
    branch: int = +1
) -> Optional[np.ndarray]:
    """
    Compute the intersection of two circles.

    Parameters
    ----------
    c1, c2 : array-like, shape (2,)
        Circle centres.
    r1, r2 : float
        Circle radii.
    branch : {+1, -1}
        +1 selects the intersection to the LEFT of the directed
        vector c1 → c2 (positive perpendicular).
        -1 selects the intersection to the RIGHT.

    Returns
    -------
    np.ndarray or None
        The 2-D intersection point, or None if circles do not
        intersect (mechanism cannot assemble).
    """
    c1 = np.asarray(c1, dtype=float)
    c2 = np.asarray(c2, dtype=float)
    d_vec = c2 - c1
    d = np.linalg.norm(d_vec)

    # Degenerate or non-intersecting cases
    if d < 1e-12 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return None

    a = (r1**2 - r2**2 + d**2) / (2.0 * d)
    h_sq = r1**2 - a**2
    h = np.sqrt(max(h_sq, 0.0))

    # Point on the line between centres
    p_along = c1 + (a / d) * d_vec

    # Perpendicular direction
    perp = np.array([-d_vec[1], d_vec[0]]) / d

    return p_along + branch * h * perp


# ──────────────────────────────────────────────────────────────────────
# 12-Link Modified Jansen Mechanism  (Shin et al. / Jadav et al.)
# ──────────────────────────────────────────────────────────────────────

class JansenMechanism:
    """
    12-link modified Jansen mechanism for gait training.

    Topology (from Jadav et al., SN Applied Sciences, 2024):
        Fixed pivots : P0 = (0, 0), P3 = (L4, 0)
        Crank        : P0 → P1 at angle θ₂, length L1
        Upper loop   : P1-P2-P3 via links L2, L3
        Upper triangle: P2-P4-P3 via links L5, L6
        Lower loop   : P1-P5-P3 via links L7, L8
        Lower triangle: P4-P6-P5 via links L10, L9
        Foot triangle : P5-PE-P6 via links L11, L12

    Three adjustable links: L1, L4, L8.
    Default values from the paper (cm): L1=11.29, L4=32.93, L8=41.78.
    """

    # Default link lengths (cm) from Jadav et al.
    DEFAULT_LINKS: Dict[str, float] = {
        "L1": 11.29, "L2": 45.0,  "L3": 36.0,  "L4": 32.93,
        "L5": 48.5,  "L6": 41.5,  "L7": 60.5,  "L8": 41.78,
        "L9": 42.0,  "L10": 43.0, "L11": 26.5,  "L12": 54.5,
    }

    # Assembly-mode branch signs (produce the canonical teardrop)
    BRANCH_P2 = +1
    BRANCH_P5 = -1
    BRANCH_P4 = +1
    BRANCH_P6 = +1
    BRANCH_PE = -1

    # Edges for drawing (pairs of joint indices in the 8-joint array)
    EDGES: List[Tuple[int, int]] = [
        (0, 1),  # P0-P1  crank
        (1, 2),  # P1-P2  L2
        (2, 3),  # P2-P3  L3
        (0, 3),  # P0-P3  L4 (ground link)
        (2, 4),  # P2-P4  L5
        (3, 4),  # P3-P4  L6
        (1, 5),  # P1-P5  L7
        (5, 3),  # P5-P3  L8
        (5, 6),  # P5-P6  L9
        (4, 6),  # P4-P6  L10
        (5, 7),  # P5-PE  L11
        (6, 7),  # P6-PE  L12
    ]

    # Link labels for each edge (for annotated diagrams)
    EDGE_LABELS: List[str] = [
        "L1", "L2", "L3", "L4", "L5", "L6",
        "L7", "L8", "L9", "L10", "L11", "L12",
    ]

    # Color scheme for sub-assemblies
    EDGE_COLORS: List[str] = [
        "#FF6B35",  # L1  crank        (orange)
        "#4ECDC4",  # L2  upper coupler (teal)
        "#4ECDC4",  # L3  upper rocker  (teal)
        "#95A5A6",  # L4  ground        (grey)
        "#2ECC71",  # L5  upper tri     (green)
        "#2ECC71",  # L6  upper tri     (green)
        "#E74C3C",  # L7  lower coupler (red)
        "#E74C3C",  # L8  lower rocker  (red)
        "#9B59B6",  # L9  lower tri     (purple)
        "#9B59B6",  # L10 lower tri     (purple)
        "#F1C40F",  # L11 foot          (gold)
        "#F1C40F",  # L12 foot          (gold)
    ]

    # Sub-assembly groupings for legend
    ASSEMBLY_GROUPS = [
        ("Crank (L1)",                "#FF6B35", [0, 1]),
        ("Ground (L4)",               "#95A5A6", [0, 3]),
        ("Upper Loop (L2, L3)",       "#4ECDC4", [1, 2, 3]),
        ("Upper Triangle (L5, L6)",   "#2ECC71", [2, 4, 3]),
        ("Lower Loop (L7, L8)",       "#E74C3C", [1, 5, 3]),
        ("Lower Triangle (L9, L10)",  "#9B59B6", [4, 6, 5]),
        ("Foot Triangle (L11, L12)",  "#F1C40F", [5, 7, 6]),
    ]

    def __init__(self, custom_links: Optional[Dict[str, float]] = None):
        """
        Initialize the mechanism.

        Parameters
        ----------
        custom_links : dict, optional
            Overrides for any of the 12 link lengths, e.g.
            ``{'L1': 12.0, 'L4': 35.0}``.
        """
        self.L = {**self.DEFAULT_LINKS, **(custom_links or {})}
        self.P0 = np.array([0.0, 0.0])
        self.P3 = np.array([self.L["L4"], 0.0])

    def solve(self, theta: float) -> Optional[JointPose12]:
        """
        Solve the full mechanism pose for crank angle θ (radians).

        Returns None if any dyad fails to assemble.
        """
        L = self.L
        cci = circle_circle_intersect

        # Crank tip
        P1 = self.P0 + L["L1"] * np.array([np.cos(theta), np.sin(theta)])

        # Upper four-bar: P2 = CCI(P1, L2) ∩ (P3, L3)
        P2 = cci(P1, L["L2"], self.P3, L["L3"], self.BRANCH_P2)
        if P2 is None:
            return None

        # Lower four-bar: P5 = CCI(P1, L7) ∩ (P3, L8)
        P5 = cci(P1, L["L7"], self.P3, L["L8"], self.BRANCH_P5)
        if P5 is None:
            return None

        # Upper triangle: P4 = CCI(P2, L5) ∩ (P3, L6)
        P4 = cci(P2, L["L5"], self.P3, L["L6"], self.BRANCH_P4)
        if P4 is None:
            return None

        # Lower triangle: P6 = CCI(P4, L10) ∩ (P5, L9)
        P6 = cci(P4, L["L10"], P5, L["L9"], self.BRANCH_P6)
        if P6 is None:
            return None

        # Foot: PE = CCI(P5, L11) ∩ (P6, L12)
        PE = cci(P5, L["L11"], P6, L["L12"], self.BRANCH_PE)
        if PE is None:
            return None

        return JointPose12(self.P0, P1, P2, self.P3, P4, P5, P6, PE)

    def sweep(self, n: int = 720) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulate one full crank revolution.

        Parameters
        ----------
        n : int
            Number of samples over [0, 2π).

        Returns
        -------
        thetas : (n,) crank angles in radians
        foot   : (n, 2) foot-point (PE) positions
        joints : (n, 8, 2) all joint positions
        """
        thetas = np.linspace(0, 2 * np.pi, n, endpoint=False)
        foot = np.full((n, 2), np.nan)
        joints = np.full((n, 8, 2), np.nan)

        for i, th in enumerate(thetas):
            pose = self.solve(th)
            if pose is not None:
                joints[i] = pose.as_array()
                foot[i] = pose.PE

        return thetas, foot, joints


# ──────────────────────────────────────────────────────────────────────
# 11-Link Classic Jansen Leg  (Theo Jansen "holy numbers")
# ──────────────────────────────────────────────────────────────────────

class JansenClassic:
    """
    Classic 11-link Theo Jansen leg with "holy numbers".

    Topology:
        Fixed pivots : O = (0, 0) crank pivot,
                       M = (-a, -l) ground pivot
        Crank tip    : A rotates around O at radius m
        Upper dyad   : B = CCI(M,j) ∩ (A,b)
        Lower dyad   : C = CCI(M,k) ∩ (A,c)
        Apex         : D = CCI(B,e) ∩ (C,d)
        Foot         : G = CCI(C,h) ∩ (D,i)

    Holy numbers (cm):
        m=15, a=38, l=7.8, j=50, k=61.9,
        b=41.5, c=39.3, d=40.1, e=55.8,
        f=39.4, g=36.7, h=65.7, i=49.0
    """

    HOLY: Dict[str, float] = {
        "m": 15.0,  "a": 38.0,  "l": 7.8,
        "j": 50.0,  "k": 61.9,
        "b": 41.5,  "c": 39.3,
        "d": 40.1,  "e": 55.8,
        "f": 39.4,  "g": 36.7,
        "h": 65.7,  "i": 49.0,
    }

    # Branch selections that produce the canonical shoe-sole trajectory
    BRANCH_B = +1
    BRANCH_C = -1
    BRANCH_D = -1
    BRANCH_G = +1

    # Edges for drawing (index into 7-joint array: O M A B C D G)
    EDGES: List[Tuple[int, int]] = [
        (0, 2),  # O-A  crank (m)
        (0, 1),  # O-M  ground (a, l)
        (1, 3),  # M-B  (j)
        (2, 3),  # A-B  (b)
        (1, 4),  # M-C  (k)
        (2, 4),  # A-C  (c)
        (3, 5),  # B-D  (e)
        (4, 5),  # C-D  (d)
        (4, 6),  # C-G  (h)
        (5, 6),  # D-G  (i)
    ]

    EDGE_COLORS: List[str] = [
        "#FF6B35",  # crank
        "#95A5A6",  # ground
        "#4ECDC4",  # j
        "#4ECDC4",  # b
        "#E74C3C",  # k
        "#E74C3C",  # c
        "#2ECC71",  # e
        "#2ECC71",  # d
        "#F1C40F",  # h
        "#F1C40F",  # i
    ]

    ASSEMBLY_GROUPS = [
        ("Crank (m)",                     "#FF6B35", [0, 2]),
        ("Fixed Frame (a, l)",            "#95A5A6", [0, 1]),
        ("Driving Rods (j, k)",           "#4ECDC4", [3, 1, 4]),
        ("Upper Assembly (b, d, e)",      "#2ECC71", [2, 3, 5, 4]),
        ("Lower Assembly (c, h, i)",      "#F1C40F", [4, 6, 5]),
    ]

    def __init__(self, links: Optional[Dict[str, float]] = None,
                 scale: float = 1.0):
        """
        Initialize the classic Jansen leg.

        Parameters
        ----------
        links : dict, optional
            Overrides for any holy number.
        scale : float
            Multiply all lengths by this factor (useful for display).
        """
        self.L = dict(self.HOLY)
        if links:
            self.L.update(links)

        # Apply scale
        self.scale = scale
        self._Ls = {k: v * scale for k, v in self.L.items()}

        self.O = np.array([0.0, 0.0])
        self.M = np.array([-self._Ls["a"], -self._Ls["l"]])

    def solve(self, theta: float) -> Optional[JointPose11]:
        """Solve pose for crank angle θ (radians)."""
        Ls = self._Ls
        cci = circle_circle_intersect

        A = self.O + Ls["m"] * np.array([np.cos(theta), np.sin(theta)])

        B = cci(self.M, Ls["j"], A, Ls["b"], self.BRANCH_B)
        if B is None:
            return None

        C = cci(self.M, Ls["k"], A, Ls["c"], self.BRANCH_C)
        if C is None:
            return None

        D = cci(B, Ls["e"], C, Ls["d"], self.BRANCH_D)
        if D is None:
            return None

        G = cci(C, Ls["h"], D, Ls["i"], self.BRANCH_G)
        if G is None:
            return None

        return JointPose11(self.O, self.M, A, B, C, D, G)

    def sweep(self, n: int = 720) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Simulate one full crank revolution.

        Returns
        -------
        thetas : (n,) crank angles in radians
        foot   : (n, 2) foot-point (G) positions
        joints : (n, 7, 2) all joint positions
        """
        thetas = np.linspace(0, 2 * np.pi, n, endpoint=False)
        foot = np.full((n, 2), np.nan)
        joints = np.full((n, 7, 2), np.nan)

        for i, th in enumerate(thetas):
            pose = self.solve(th)
            if pose is not None:
                joints[i] = pose.as_array()
                foot[i] = pose.G

        return thetas, foot, joints
