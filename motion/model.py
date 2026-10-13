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

    def local_point(self, q, w):
        """Reference-pose point that sits at world point ``w`` (inverse of
        :meth:`world_point`)."""
        w = np.asarray(w, float)
        if self.fixed:
            return w
        return self.center + rotation(q[3:]).T @ (w - self.center - q[:3])


class Path:
    """Rail centre line: straight pieces and circular arcs, end to end,
    measured by arc length ``s`` from the start (0 .. ``length``).

    ``Path(start).line_to(p).arc(center, angle, axis)...``: ``arc`` turns
    from the current end about ``center`` by ``angle`` (radians, right-hand
    about ``axis``)."""

    def __init__(self, start):
        self.start = np.asarray(start, float)
        self.end = self.start.copy()
        self.pieces = []           # (kind, s0, length, data)
        self.length = 0.0

    def line_to(self, p):
        p = np.asarray(p, float)
        L = float(np.linalg.norm(p - self.end))
        if L < 1e-12:
            raise ValueError("zero-length rail piece")
        self.pieces.append(("line", self.length, L, (self.end.copy(), (p - self.end) / L)))
        self.length += L
        self.end = p
        return self

    def arc(self, center, angle, axis):
        c, k = np.asarray(center, float), unit(axis)
        r0 = self.end - c
        r0 = r0 - k * (k @ r0)
        R = float(np.linalg.norm(r0))
        if R < 1e-12 or angle <= 0:
            raise ValueError("arc needs a radius and a positive angle")
        L = R * angle
        self.pieces.append(("arc", self.length, L, (c, k, r0, np.cross(k, r0), R, angle)))
        self.length += L
        self.end = self._arc_point(self.pieces[-1][3], angle)
        return self

    @staticmethod
    def _arc_point(d, th):
        c, k, r0, r1, R, _ = d
        return c + np.cos(th) * r0 + np.sin(th) * r1

    def point(self, s):
        s = min(max(float(s), 0.0), self.length)
        for kind, s0, L, d in self.pieces:
            if s <= s0 + L + 1e-15:
                u = s - s0
                if kind == "line":
                    return d[0] + u * d[1]
                return self._arc_point(d, u / d[4])
        return self.end.copy()

    def tangent(self, s):
        s = min(max(float(s), 0.0), self.length)
        for kind, s0, L, d in self.pieces:
            if s <= s0 + L + 1e-15:
                if kind == "line":
                    return d[1].copy()
                th = (s - s0) / d[4]
                return unit(-np.sin(th) * d[2] + np.cos(th) * d[3])
        return self.tangent(self.length - 1e-12)

    def closest(self, p):
        """(s, point) of the rail point nearest to ``p``."""
        p = np.asarray(p, float)
        best = (np.inf, 0.0, self.start)
        for kind, s0, L, d in self.pieces:
            if kind == "line":
                u = min(max((p - d[0]) @ d[1], 0.0), L)
                q = d[0] + u * d[1]
            else:
                c, k, r0, r1, R, A = d
                v = p - c
                th = float(np.arctan2(v @ r1, v @ r0))
                if th < 0 and th < -(2 * np.pi - A) / 2:
                    th += 2 * np.pi          # nearer to the arc's far end
                th = min(max(th, 0.0), A)
                u = th * R
                q = self._arc_point(d, th)
            dist = float(np.linalg.norm(p - q))
            if dist < best[0] - 1e-15:
                best = (dist, s0 + u, q)
        return best[1], best[2]


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


@dataclass
class Guide:
    """Point ``point`` of body b (a roller, a pin) runs on ``path``, a rail
    fixed to body a (both given in the reference pose)."""
    a: Body
    b: Body
    point: np.ndarray
    path: Path
    limits: tuple = None       # (min, max) arc length along the rail
    name: str = ""
    kind: str = "guide"


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

    def guide(self, a, b, point, path, limits=None, name=""):
        """Rail: ``point`` of ``b`` stays on ``path`` (fixed to ``a``).
        Default limits: the two ends of the rail."""
        j = Guide(a, b, np.asarray(point, float), path,
                  (0.0, path.length) if limits is None else limits,
                  name or f"guide{len(self.joints)}")
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
        """Current relative angle (pivot), travel (slider) or position
        along the rail (guide)."""
        qa, qb = self.pose(x, j.a), self.pose(x, j.b)
        if j.kind == "guide":
            p = j.a.local_point(qa, j.b.world_point(qb, j.point))
            return float(j.path.closest(p)[0])
        Ra, Rb = j.a.rot(qa), j.b.rot(qb)
        if j.kind == "pivot":
            n1, n2 = perp_basis(j.axis)
            v = Ra.T @ (Rb @ n1)
            return float(np.arctan2(n2 @ v, n1 @ v))
        d = j.b.world_point(qb, j.point) - j.a.world_point(qa, j.point)
        return float((Ra @ j.axis) @ d)
