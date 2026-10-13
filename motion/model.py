"""Rigid bodies, joints and the mechanism container.

Convention: every body has a *reference pose* (the geometry exactly as drawn,
world coordinates).  Its current pose is described by 6 numbers
q = (tx, ty, tz, wx, wy, wz): a rotation vector w about the body's reference
centre ``c`` followed by a translation t.  q = 0 is the drawn configuration.
Lengths are in whatever unit the model uses (IngeTrazo: metres or mm),
angles in radians.
"""
from dataclasses import dataclass, field
import numpy as np


def rotation(w):
    """Rodrigues: rotation matrix of rotation vector ``w``."""
    th = float(np.linalg.norm(w))
    if th < 1e-12:
        K = _skew(w)
        return np.eye(3) + K + 0.5 * K @ K
    k = np.asarray(w, float) / th
    K = _skew(k)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def _skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)


def unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("zero-length axis")
    return v / n


def perp_basis(u):
    """Two unit vectors orthogonal to ``u`` (and to each other)."""
    u = unit(u)
    a = np.array([1.0, 0, 0]) if abs(u[0]) < 0.9 else np.array([0, 1.0, 0])
    n1 = unit(np.cross(u, a))
    return n1, np.cross(u, n1)


@dataclass
class Box:
    """Box in the body's reference pose: centre, half extents and an
    optional orientation ``R`` (columns = the box's own axes; identity =
    aligned with the world axes, as in V0.1)."""
    center: np.ndarray
    half: np.ndarray
    R: np.ndarray = None
    name: str = ""

    def __post_init__(self):
        self.center = np.asarray(self.center, float)
        self.half = np.asarray(self.half, float)
        self.R = np.eye(3) if self.R is None else np.asarray(self.R, float)


def bar(p0, p1, width, thickness, side=(0, 1.0, 0), name=""):
    """Box of a straight member from ``p0`` to ``p1`` (axis to axis):
    ``thickness`` along ``side``, ``width`` across, in the plane."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    e1 = unit(p1 - p0)
    e2 = unit(np.asarray(side, float) - e1 * (e1 @ np.asarray(side, float)))
    e3 = np.cross(e1, e2)
    half = (np.linalg.norm(p1 - p0) / 2, thickness / 2, width / 2)
    return Box((p0 + p1) / 2, half, np.column_stack([e1, e2, e3]), name)


@dataclass
class Body:
    name: str
    center: np.ndarray = field(default_factory=lambda: np.zeros(3))
    boxes: list = field(default_factory=list)
    fixed: bool = False
    index: int = -1            # slot in the pose vector (-1 for ground)

    def world_point(self, q, p):
        """World position of the body-fixed reference point ``p``."""
        if self.fixed:
            return np.asarray(p, float)
        R = rotation(q[3:])
        return self.center + q[:3] + R @ (np.asarray(p, float) - self.center)

    def rot(self, q):
        return np.eye(3) if self.fixed else rotation(q[3:])


@dataclass
class Pivot:
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None       # (min, max) radians, relative angle b w.r.t. a
    name: str = ""
    kind: str = "pivot"


@dataclass
class Slider:
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None       # (min, max) travel along axis
    name: str = ""
    kind: str = "slider"


class Mechanism:
    def __init__(self):
        self.bodies = []
        self.joints = []
        self.ground = Body("ground", fixed=True)
        self.driver = None     # (joint, lo, hi)

    # -- building -------------------------------------------------------
    def body(self, name, center=(0, 0, 0), boxes=()):
        b = Body(name, np.asarray(center, float), list(boxes),
                 index=len(self.bodies))
        self.bodies.append(b)
        return b

    def pivot(self, a, b, point, axis, limits=None, name=""):
        j = Pivot(a, b, np.asarray(point, float), unit(axis), limits,
                  name or f"pivot{len(self.joints)}")
        self.joints.append(j)
        return j

    def slider(self, a, b, point, axis, limits=None, name=""):
        j = Slider(a, b, np.asarray(point, float), unit(axis), limits,
                   name or f"slider{len(self.joints)}")
        self.joints.append(j)
        return j

    def drive(self, joint, lo, hi):
        """Make ``joint`` the driver: 0 % -> ``lo``, 100 % -> ``hi``."""
        self.driver = (joint, float(lo), float(hi))

    # -- helpers --------------------------------------------------------
    @property
    def n(self):
        return 6 * len(self.bodies)

    def pose(self, x, body):
        if body.fixed:
            return np.zeros(6)
        return x[6 * body.index: 6 * body.index + 6]

    def joint_value(self, j, x):
        """Current relative angle (pivot) or travel (slider)."""
        qa, qb = self.pose(x, j.a), self.pose(x, j.b)
        Ra, Rb = j.a.rot(qa), j.b.rot(qb)
        if j.kind == "pivot":
            n1, n2 = perp_basis(j.axis)
            v = Ra.T @ (Rb @ n1)
            return float(np.arctan2(n2 @ v, n1 @ v))
        d = j.b.world_point(qb, j.point) - j.a.world_point(qa, j.point)
        return float((Ra @ j.axis) @ d)
