"""Rigid bodies, joints and the mechanism container.

Convention: every body has a *reference pose* (the geometry exactly as drawn,
world coordinates).  Its current pose is described by 6 numbers
q = (tx, ty, tz, wx, wy, wz): a rotation vector w about the body's reference
centre ``c`` followed by a translation t.  q = 0 is the drawn configuration.
Lengths are in whatever unit the model uses (IngeTrazo: metres or mm),
angles in radians.
"""
from dataclasses import dataclass, field
import math
import numpy as np


def rotation(w):
    """Rodrigues: rotation matrix of rotation vector ``w`` (written out:
    the solvers call it for every part at every step)."""
    x, y, z = float(w[0]), float(w[1]), float(w[2])
    th = math.sqrt(x * x + y * y + z * z)
    if th < 1e-12:
        s, c = 1.0, 0.5
    else:
        x, y, z = x / th, y / th, z / th
        s, c = math.sin(th), 1.0 - math.cos(th)
    xx, yy, zz, xy, xz, yz = x * x, y * y, z * z, x * y, x * z, y * z
    return np.array([[1.0 - c * (yy + zz), -s * z + c * xy, s * y + c * xz],
                     [s * z + c * xy, 1.0 - c * (xx + zz), -s * x + c * yz],
                     [-s * y + c * xz, s * x + c * yz, 1.0 - c * (xx + yy)]])


def rotations(W):
    """Rodrigues for many rotation vectors at once: (n, 3) -> (n, 3, 3)."""
    W = np.asarray(W, float).reshape(-1, 3)
    th = np.linalg.norm(W, axis=1)
    small = th < 1e-12
    k = np.where(small[:, None], W, W / np.where(small, 1.0, th)[:, None])
    K = np.zeros((len(W), 3, 3))
    K[:, 0, 1], K[:, 0, 2] = -k[:, 2], k[:, 1]
    K[:, 1, 0], K[:, 1, 2] = k[:, 2], -k[:, 0]
    K[:, 2, 0], K[:, 2, 1] = -k[:, 1], k[:, 0]
    s = np.where(small, 1.0, np.sin(th))[:, None, None]
    c = np.where(small, 0.5, 1 - np.cos(th))[:, None, None]
    return np.eye(3)[None] + s * K + c * (K @ K)


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
    color: tuple = None        # (r, g, b) for drawings; None = the body's

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
    color: tuple = None        # (r, g, b) for drawings; None = automatic
    mass: float = None         # kg (from the materials), None = unknown
    com: np.ndarray = None     # centre of gravity, reference pose
    inertia: np.ndarray = None # 3x3 about ``com``, world axes, kg m^2

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
    spring: tuple = None       # (stiffness N m/rad, rest angle) — dynamics
    damper: float = 0.0        # N m s/rad — dynamics


@dataclass
class Slider:
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None       # (min, max) travel along axis
    name: str = ""
    kind: str = "slider"
    spring: tuple = None       # (stiffness N/m, rest travel) — dynamics
    damper: float = 0.0        # N s/m — dynamics


@dataclass
class Cylindrical:
    """Pivot glissant: b turns about and slides along the axis through
    ``point``. Its value is the angle (motors, couplings, ``limits``);
    ``slide_limits`` bound the travel."""
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "cylindrical"
    spring: tuple = None       # on the angle
    damper: float = 0.0
    slide_limits: tuple = None
    slide_spring: tuple = None # (N/m, rest travel)
    slide_damper: float = 0.0


@dataclass
class Ball:
    """Rotule: the point ``point`` of a and of b stay together; every
    rotation is free."""
    a: Body
    b: Body
    point: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "ball"
    damper: float = 0.0        # N m s/rad, on the relative rotation


@dataclass
class Lock:
    """Encastrement: a and b move as one."""
    a: Body
    b: Body
    point: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "lock"


@dataclass
class Annular:
    """Linéaire annulaire: the point ``point`` of b (a ball's centre) stays
    on the line through ``point`` along ``axis`` of a (a bore's axis): it
    slides along it and every rotation is free."""
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "annular"


@dataclass
class PointPlane:
    """Ponctuelle: the point ``point`` of b stays on the plane through
    ``point`` square to ``axis`` (the normal) of a."""
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "pointplane"


@dataclass
class Planar:
    """Appui plan: b slides on a's plane (through ``point``, normal
    ``axis``) and turns about the normal."""
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    limits: tuple = None
    name: str = ""
    kind: str = "planar"


@dataclass
class Screw:
    """Hélicoïdale (vis-écrou): b turns about the axis and advances
    ``pitch`` (m) per turn, positive with the angle. Its value is the
    angle (motors, couplings)."""
    a: Body
    b: Body
    point: np.ndarray
    axis: np.ndarray
    pitch: float = 0.001
    limits: tuple = None
    name: str = ""
    kind: str = "screw"
    spring: tuple = None
    damper: float = 0.0


@dataclass
class Elastic:
    """Liaison élastique (a silent-block, a flexible mount): rigid for the
    kinematics; in the dynamics the point ``point`` of b is held to a's
    by springs and dampers in every direction (translations: N/m, N s/m;
    rotations: N m/rad, N m s/rad)."""
    a: Body
    b: Body
    point: np.ndarray
    k_lin: float = 1e5
    k_rot: float = 1e3
    c_lin: float = 50.0
    c_rot: float = 1.0
    limits: tuple = None
    name: str = ""
    kind: str = "elastic"


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


@dataclass
class Coupling:
    """Fixed ratio between two joints: value(jb) = ratio * value(ja),
    both counted from the drawn configuration. Gears (two pivots, ratio
    -z1/z2 for external teeth), rolling without slip (a wheel pivot and a
    slider, ratio = -radius or +radius by direction), rack and pinion.
    Pivot angles are taken continuously (whole turns counted), which holds
    for planar mechanisms (axes parallel)."""
    ja: object
    jb: object
    ratio: float
    name: str = ""
    kind: str = "coupling"
    limits: tuple = None

    @property
    def a(self):
        return self.ja.b

    @property
    def b(self):
        return self.jb.b

    def bodies(self):
        return [self.ja.a, self.ja.b, self.jb.a, self.jb.b]


class Mechanism:
    def __init__(self):
        self.bodies = []
        self.joints = []
        self.ground = Body("ground", fixed=True)
        self.drivers = []      # [(joint, lo, hi)]: the motors
        # parts placed by a rule once the joints are solved (chain links):
        # objects with ``bodies`` and ``place(mech, x)`` -> x
        self.followers = []
        self.no_contact = set()   # {frozenset((id(a), id(b)))}: never checked

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

    def cylindrical(self, a, b, point, axis, limits=None, name="", slide_limits=None):
        j = Cylindrical(a, b, np.asarray(point, float), unit(axis), limits,
                        name or f"pivot glissant{len(self.joints)}", slide_limits=slide_limits)
        self.joints.append(j)
        return j

    def ball(self, a, b, point, name=""):
        j = Ball(a, b, np.asarray(point, float), name=name or f"rotule{len(self.joints)}")
        self.joints.append(j)
        return j

    def lock(self, a, b, point=None, name=""):
        p = np.asarray(point if point is not None else b.center, float)
        j = Lock(a, b, p, name=name or f"encastrement{len(self.joints)}")
        self.joints.append(j)
        return j

    def annular(self, a, b, point, axis, name=""):
        j = Annular(a, b, np.asarray(point, float), unit(axis),
                    name=name or f"linéaire annulaire{len(self.joints)}")
        self.joints.append(j)
        return j

    def point_plane(self, a, b, point, normal, name=""):
        j = PointPlane(a, b, np.asarray(point, float), unit(normal),
                       name=name or f"ponctuelle{len(self.joints)}")
        self.joints.append(j)
        return j

    def planar(self, a, b, point, normal, name=""):
        j = Planar(a, b, np.asarray(point, float), unit(normal),
                   name=name or f"appui plan{len(self.joints)}")
        self.joints.append(j)
        return j

    def screw(self, a, b, point, axis, pitch, limits=None, name=""):
        j = Screw(a, b, np.asarray(point, float), unit(axis), float(pitch), limits,
                  name or f"hélicoïdale{len(self.joints)}")
        self.joints.append(j)
        return j

    def elastic(self, a, b, point, k_lin=1e5, k_rot=1e3, c_lin=50.0, c_rot=1.0, name=""):
        j = Elastic(a, b, np.asarray(point, float), k_lin, k_rot, c_lin, c_rot,
                    name=name or f"liaison élastique{len(self.joints)}")
        self.joints.append(j)
        return j

    def couple(self, ja, jb, ratio, name=""):
        """``jb`` follows ``ja`` at ``ratio`` (see :class:`Coupling`)."""
        j = Coupling(ja, jb, float(ratio), name or f"couplage{len(self.joints)}")
        self.joints.append(j)
        return j

    def drive(self, joint, lo, hi):
        """Make ``joint`` THE driver (the only motor): 0 % -> ``lo``,
        100 % -> ``hi``."""
        self.drivers = [(joint, float(lo), float(hi))]

    def add_driver(self, joint, lo, hi):
        """One more motor. Every motor runs on the same 0..100 % sweep from
        its own ``lo`` to its own ``hi``: different speeds are different
        travels, a phase is a different ``lo``. The first motor leads (its
        value is what ``solver.solve`` is given)."""
        self.drivers = [d for d in self.drivers if d[0] is not joint]
        self.drivers.append((joint, float(lo), float(hi)))

    @property
    def driver(self):
        """The leading motor (joint, lo, hi), or None."""
        return self.drivers[0] if self.drivers else None

    @driver.setter
    def driver(self, value):
        self.drivers = [] if value is None else [tuple(value)]

    def driver_targets(self, value):
        """Target of every motor when the leading one is at ``value``."""
        j0, lo0, hi0 = self.drivers[0]
        t = 0.0 if hi0 == lo0 else (value - lo0) / (hi0 - lo0)
        return [value] + [lo + t * (hi - lo) for _, lo, hi in self.drivers[1:]]

    # -- helpers --------------------------------------------------------
    @property
    def n(self):
        return 6 * len(self.bodies)

    def follower_bodies(self):
        return [b for f in self.followers for b in f.bodies]

    def place(self, x):
        """Pose of the follower parts (chain links) from the solved joints."""
        for f in self.followers:
            x = f.place(self, x)
        return x

    def pose(self, x, body):
        if body.fixed:
            return np.zeros(6)
        return x[6 * body.index: 6 * body.index + 6]

    def slide_value(self, j, x):
        """Travel along the axis (slider, pivot glissant)."""
        qa, qb = self.pose(x, j.a), self.pose(x, j.b)
        d = j.b.world_point(qb, j.point) - j.a.world_point(qa, j.point)
        return float((j.a.rot(qa) @ j.axis) @ d)

    def continuous_value(self, j, x):
        """Like :meth:`joint_value`, but a pivot's angle keeps counting
        past +-180 deg (from the rotation vectors: planar mechanisms)."""
        if j.kind in ("pivot", "cylindrical", "screw"):
            qa, qb = self.pose(x, j.a), self.pose(x, j.b)
            return float((qb[3:] - qa[3:]) @ j.axis)
        return self.joint_value(j, x)

    def multi_turn(self, drv=None):
        """The (leading, or given) motor is a pivot turning more than one
        revolution."""
        drv = drv if drv is not None else self.driver
        return (drv is not None and drv[0].kind in ("pivot", "cylindrical", "screw")
                and abs(drv[2] - drv[1]) > 2 * np.pi + 1e-9)

    def joint_value(self, j, x):
        """Current relative angle (pivot), travel (slider) or position
        along the rail (guide)."""
        if j.kind == "coupling":
            return self.continuous_value(j.jb, x)
        qa, qb = self.pose(x, j.a), self.pose(x, j.b)
        if j.kind == "guide":
            p = j.a.local_point(qa, j.b.world_point(qb, j.point))
            return float(j.path.closest(p)[0])
        Ra, Rb = j.a.rot(qa), j.b.rot(qb)
        if j.kind in ("ball", "lock", "elastic", "annular", "pointplane", "planar"):
            return 0.0
        if j.kind in ("pivot", "cylindrical", "screw"):
            n1, n2 = perp_basis(j.axis)
            v = Ra.T @ (Rb @ n1)
            return float(np.arctan2(n2 @ v, n1 @ v))
        d = j.b.world_point(qb, j.point) - j.a.world_point(qa, j.point)
        return float((Ra @ j.axis) @ d)
