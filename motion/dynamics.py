"""Rigid-body dynamics of a mechanism: weight, motors, reactions.

Every moving part has a mass, a centre of gravity and an inertia (from its
materials, :mod:`motion.massprops`). Its motion is described by the
velocity of its centre of gravity ``v`` and its angular velocity ``w``
(world axes); Newton-Euler for each part,

    m dv/dt = F + reactions,      I dw/dt + w x I w = T + reactions,

with the joints as constraints C(x) = 0 (the same residuals as the
kinematic solver). At each step the accelerations and the joint reactions
(Lagrange multipliers) are solved together:

    [ M  G^T ] [ a      ]   [ F - w x I w                    ]
    [ G  0   ] [ -lambda] = [ -gamma - 2 k C' - k^2 C + motor ]

G = dC/d(velocities) and gamma = d(G)/dt . u are taken by finite
differences along the motion; k (Baumgarte) pulls back any drift. Then
positions move by u dt + a dt^2 / 2 (exact under constant forces: a free
fall is exact) and velocities by a dt.

Motors: a *speed* motor imposes a joint's value in time (its multiplier is
the torque or force it must give); a *torque* motor applies a torque (or
a force, on a slider) and lets the mechanism answer.
"""
import math
import numpy as np
from .model import rotation, rotations
from . import solver as _solver


def log_rotation(R):
    """Rotation vector of rotation matrix ``R`` (inverse of model.rotation),
    accurate at every angle: the angle from atan2(sin, cos), the axis from
    the skew part, or near half a turn from the symmetric part."""
    sk = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s2 = float(np.linalg.norm(sk))                    # 2 sin(theta)
    c = (np.trace(R) - 1.0) / 2.0
    th = math.atan2(0.5 * s2, c)
    if th < 1e-9:
        return 0.5 * sk
    if c > -0.5:                                      # away from a half turn
        return th * sk / s2
    S = 0.5 * (R + R.T) - c * np.eye(3)               # (1 - c) n n^T
    k = int(np.argmax(np.diag(S)))
    n = S[:, k] / math.sqrt(max(S[k, k] * (1 - c), 1e-300))
    n /= np.linalg.norm(n)
    if sk @ n < 0:
        n = -n
    return th * n


def _unwrap(v, near):
    """The rotation vector of the same rotation as ``v`` (angle + 2 pi k
    about its axis) nearest to ``near``: whole turns keep counting, as the
    kinematic solver's do."""
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        n = near / np.linalg.norm(near) if np.linalg.norm(near) > 1e-12 else None
        if n is None:
            return v
        k = round(float(np.linalg.norm(near)) / (2 * math.pi))
        return n * 2 * math.pi * k + v
    n = v / th
    best, bd = v, np.linalg.norm(v - near)
    base = round(float(near @ n) / (2 * math.pi))
    for k in (base - 1, base, base + 1):
        for s in (1.0, -1.0):
            c = s * n * (th + 2 * math.pi * k) if s > 0 else -n * (2 * math.pi * k - th)
            d = np.linalg.norm(c - near)
            if d < bd:
                best, bd = c, d
    return best


def log_rotations(R):
    """:func:`log_rotation` of (n, 3, 3) matrices at once (rows near half a
    turn go through the one-matrix version)."""
    R = np.asarray(R, float)
    sk = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0],
                   R[:, 1, 0] - R[:, 0, 1]], axis=1)
    s2 = np.sqrt(np.einsum("ij,ij->i", sk, sk))
    c = (np.trace(R, axis1=1, axis2=2) - 1.0) / 2.0
    th = np.arctan2(0.5 * s2, c)
    out = 0.5 * sk
    big = th >= 1e-9
    usual = big & (c > -0.5)
    out[usual] = (th[usual] / s2[usual])[:, None] * sk[usual]
    for i in np.nonzero(big & ~usual)[0]:
        out[i] = log_rotation(R[i])
    return out


def unwraps(V, near):
    """:func:`_unwrap` of (n, 3) rotation vectors at once (the same choice,
    ties included: the first nearest candidate in the same order)."""
    V, near = np.asarray(V, float), np.asarray(near, float)
    out = V.copy()
    th = np.sqrt(np.einsum("ij,ij->i", V, V))
    small = th < 1e-12
    if small.any():                              # no turn: keep near's whole turns
        nr = near[small]
        ln = np.sqrt(np.einsum("ij,ij->i", nr, nr))
        has = ln > 1e-12
        k = np.round(ln / (2 * math.pi))
        n = np.where(has[:, None], nr / np.where(has, ln, 1.0)[:, None], 0.0)
        out[small] = np.where(has[:, None], n * (2 * math.pi * k)[:, None] + V[small], V[small])
    ok = ~small
    if not ok.any():
        return out
    v, nr, t = V[ok], near[ok], th[ok]
    n = v / t[:, None]
    base = np.round(np.einsum("ij,ij->i", nr, n) / (2 * math.pi))
    lens = [t]                                   # the order _unwrap tries them in
    for dk in (-1, 0, 1):
        k = base + dk
        lens += [t + 2 * math.pi * k, t - 2 * math.pi * k]
    L = np.stack(lens, axis=1)                   # (m, 7): candidate = n * L
    C = n[:, None, :] * L[:, :, None]
    d = np.linalg.norm(C - nr[:, None, :], axis=2)
    pick = np.argmin(d, axis=1)
    best = C[np.arange(len(C)), pick]
    out[ok] = np.where((pick == 0)[:, None], v, best)
    return out


def _cross(a, b):
    """np.cross for 3-vectors (or rows of them), without its general
    overhead: the joints call it tens of thousands of times a second."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    if a.ndim == 1 and b.ndim == 1:
        a0, a1, a2 = a
        b0, b1, b2 = b
        return np.array([a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0])
    out = np.empty(np.broadcast_shapes(a.shape, b.shape))
    out[..., 0] = a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1]
    out[..., 1] = a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2]
    out[..., 2] = a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    return out


_EYE3 = np.eye(3)


def _skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], float)


class Dynamics:
    """Time simulation of ``mech``.

    ``gravity``: m/s^2 (z up by default). ``motors``: {joint: ("vitesse",
    f) or ("couple", f)} with f(t) the joint value (rad or m) or the
    torque (N m) / force (N). Joints not listed are free. ``damping``:
    {joint: c} viscous friction in a joint (N m s/rad or N s/m).
    ``default_density``: kg/m^3 for parts without a mass (their boxes'
    volume). ``stiffness``: Baumgarte constant (1/s). ``contacts``: True
    (or a dict of :class:`motion.impacts.Impacts` options) for shocks and
    resting contacts between the parts' boxes; parts held by a joint do
    not knock each other."""

    def __init__(self, mech, gravity=(0.0, 0.0, -9.81), motors=None, damping=None,
                 default_density=700.0, stiffness=30.0, contacts=None):
        self.m = mech
        self.g = np.asarray(gravity, float)
        key = lambda j: j if isinstance(j, str) else j.name
        self.motors = {key(j): v for j, v in (motors or {}).items()}
        self.damping = {key(j): v for j, v in (damping or {}).items()}
        self.k = float(stiffness)
        fol = {id(b) for b in getattr(mech, "follower_bodies", lambda: [])()}
        self.bodies = [b for b in mech.bodies if id(b) not in fol]
        self.mass, self.com, self.Iref = {}, {}, {}
        self.assumed = []
        for b in self.bodies:
            if getattr(b, "mass", None):
                self.mass[id(b)] = float(b.mass)
                self.com[id(b)] = np.asarray(b.com, float)
                self.Iref[id(b)] = np.asarray(b.inertia, float)
            else:                                   # boxes, default density
                vol = sum(8 * float(np.prod(bx.half)) for bx in b.boxes) or 1e-6
                m = default_density * vol
                c = (np.mean([bx.center for bx in b.boxes], axis=0) if b.boxes
                     else np.asarray(b.center, float))
                I = np.zeros((3, 3))
                for bx in b.boxes:
                    mb = m * 8 * float(np.prod(bx.half)) / vol
                    h2 = (2 * bx.half) ** 2
                    Il = mb / 12 * np.diag([h2[1] + h2[2], h2[0] + h2[2], h2[0] + h2[1]])
                    d = bx.center - c
                    I += bx.R @ Il @ bx.R.T + mb * (d @ d * np.eye(3) - np.outer(d, d))
                self.mass[id(b)], self.com[id(b)], self.Iref[id(b)] = m, c, I
                self.assumed.append(b.name)
        self.joints = [j for j in mech.joints]   # elastic ones act as springs only
        self.n = 6 * len(self.bodies)
        self.impacts = None
        if contacts:
            from .impacts import Impacts
            self.impacts = Impacts(self, **(contacts if isinstance(contacts, dict) else {}))

    # -- pose <-> centre-of-gravity coordinates ----------------------------
    def _R(self, x, b):
        return rotation(x[6 * b.index + 3: 6 * b.index + 6])

    def cog(self, x, b):
        return b.world_point(self.m.pose(x, b), self.com[id(b)])

    def displace(self, x, du):
        """Pose after moving each part's centre of gravity by du[:3] and
        turning it about that centre by du[3:] (world rotation vector)."""
        x = np.array(x, float)
        nb = len(self.bodies)
        if not nb:
            return x
        D = np.asarray(du, float).reshape(nb, 6)
        act = np.nonzero(np.any(D != 0, axis=1))[0]
        if not len(act):
            return x
        a = self._arrays()
        X = x.reshape(-1, 6)
        rows = a["index"][act]
        q = X[rows]
        R = rotations(q[:, 3:])
        r0 = a["com"][act] - a["center"][act]
        c = a["center"][act] + q[:, :3] + np.einsum("nij,nj->ni", R, r0)
        R2 = rotations(D[act, 3:]) @ R
        c2 = c + D[act, :3]
        X[rows, :3] = c2 - np.einsum("nij,nj->ni", R2, r0) - a["center"][act]
        X[rows, 3:] = unwraps(log_rotations(R2), q[:, 3:])
        return x

    # -- constraints -----------------------------------------------------------
    def _rows(self):
        rows = []
        for j in self.joints:
            mot = self.motors.get(j.name)
            if j.kind == "elastic":               # springs, not a constraint
                continue
            n = _solver.rows_of(j)
            rows.append((j, n, False))
            if mot is not None and mot[0] == "vitesse":
                rows.append((j, 1, True))
        return rows

    def C(self, x, t):
        out = []
        for j, n, drv in self._rows():
            if drv:
                out.append([self.m.continuous_value(j, x) - self.motors[j.name][1](t)])
            else:
                out.append(_solver._joint_residual(self.m, j, x))
        return np.concatenate(out) if out else np.zeros(0)

    def G(self, x, t, eps=1e-6):
        return self._kin(x, np.zeros(self.n), t, eps)[1]

    # -- masses and forces ------------------------------------------------------
    def _arrays(self):
        if getattr(self, "_arr", None) is None:
            b = self.bodies
            self._arr = dict(
                index=np.array([bb.index for bb in b], int),
                mass=np.array([self.mass[id(bb)] for bb in b], float),
                Iref=np.array([self.Iref[id(bb)] for bb in b], float).reshape(-1, 3, 3),
                center=np.array([bb.center for bb in b], float).reshape(-1, 3),
                com=np.array([self.com[id(bb)] for bb in b], float).reshape(-1, 3))
        return self._arr

    def poses(self, x):
        """(R, cog) of every moving part at once: (n, 3, 3), (n, 3)."""
        a = self._arrays()
        q = np.asarray(x, float).reshape(-1, 6)[a["index"]]
        R = rotations(q[:, 3:])
        cog = a["center"] + q[:, :3] + np.einsum("nij,nj->ni", R, a["com"] - a["center"])
        return R, cog

    def _blocks(self, x, u, t):
        """(R, cog, Iw, F): the parts' rotations, centres of gravity, world
        inertias (n, 3, 3) and the forces (weight, gyroscopic, joints,
        contacts). The mass matrix is block diagonal: kept as blocks."""
        F = np.zeros(self.n)
        nb = len(self.bodies)
        R = cog = Iw = None
        if nb:
            a = self._arrays()
            R, cog = self.poses(x)
            Iw = R @ a["Iref"] @ np.transpose(R, (0, 2, 1))
            U = np.asarray(u, float).reshape(nb, 6)
            w = U[:, 3:]
            Fb = np.zeros((nb, 6))
            Fb[:, :3] = a["mass"][:, None] * self.g
            Iww = np.einsum("nij,nj->ni", Iw, w)
            Fb[:, 3:] = -np.stack([w[:, 1] * Iww[:, 2] - w[:, 2] * Iww[:, 1],
                                   w[:, 2] * Iww[:, 0] - w[:, 0] * Iww[:, 2],
                                   w[:, 0] * Iww[:, 1] - w[:, 1] * Iww[:, 0]], axis=1)
            F += Fb.reshape(-1)
        if self.joints:
            idx = {id(b): k for k, b in enumerate(self.bodies)}
            for j in self.joints:
                self._joint_efforts(j, x, u, t, F, idx)
        if self.impacts is not None:
            self.impacts.forces(x, u, F, R, cog)
        return R, cog, Iw, F

    def M_and_forces(self, x, u, t):
        R, cog, Iw, F = self._blocks(x, u, t)
        M = np.zeros((self.n, self.n))
        if Iw is not None:
            a = self._arrays()
            for k in range(len(self.bodies)):
                M[6 * k: 6 * k + 3, 6 * k: 6 * k + 3] = a["mass"][k] * np.eye(3)
                M[6 * k + 3: 6 * k + 6, 6 * k + 3: 6 * k + 6] = Iw[k]
        return M, F

    def _inverse_inertias(self, R):
        """World inverse inertias R I^+ R^T (pinv: a thin rod has no inertia
        about its own axis); the reference pseudo-inverse is taken once."""
        a = self._arrays()
        if "Iref_pinv" not in a:
            a["Iref_pinv"] = np.linalg.pinv(a["Iref"], rcond=1e-12)
        return R @ a["Iref_pinv"] @ np.transpose(R, (0, 2, 1))

    # -- efforts in the joints: torque motors, springs, dampers, stops --------
    def _stop(self, v, rate, lim, inertia):
        """Effort pushing back a joint past one of its stops: a stiff spring
        (about 40 Hz on the part behind it) with near-critical damping."""
        if not lim or inertia <= 0:
            return 0.0
        lo, hi = lim
        k = inertia * (2 * math.pi * 40.0) ** 2
        c = 2 * 0.8 * math.sqrt(k * inertia)
        if lo is not None and v < lo:
            return k * (lo - v) - c * min(rate, 0.0)
        if hi is not None and v > hi:
            return -k * (v - hi) - c * max(rate, 0.0)
        return 0.0

    def _inertia_about(self, x, body, point, axis):
        if id(body) not in self.mass:
            return 0.0
        R = self._R(x, body)
        Iw = R @ self.Iref[id(body)] @ R.T
        r = self.cog(x, body) - point
        r_perp = r - (r @ axis) * axis
        return float(axis @ Iw @ axis + self.mass[id(body)] * r_perp @ r_perp)

    def _torque(self, F, idx, j, tq):
        for body, s in ((j.b, 1.0), (j.a, -1.0)):
            k = idx.get(id(body))
            if k is not None:
                F[6 * k + 3: 6 * k + 6] += s * tq

    def _force_at(self, F, idx, x, j, point, f):
        for body, s in ((j.b, 1.0), (j.a, -1.0)):
            k = idx.get(id(body))
            if k is None:
                continue
            F[6 * k: 6 * k + 3] += s * f
            F[6 * k + 3: 6 * k + 6] += s * _cross(point - self.cog(x, body), f)

    def _joint_efforts(self, j, x, u, t, F, idx):
        if j.kind not in ("pivot", "slider", "cylindrical", "ball", "elastic", "screw"):
            return
        mot = self.motors.get(j.name)
        motor = mot[1](t) if (mot is not None and mot[0] == "couple") else 0.0
        extra_c = self.damping.get(j.name, 0.0)
        qa = self.m.pose(x, j.a)
        Ra = j.a.rot(qa)
        ka, kb = idx.get(id(j.a)), idx.get(id(j.b))
        wa = u[6 * ka + 3: 6 * ka + 6] if ka is not None else np.zeros(3)
        wb = u[6 * kb + 3: 6 * kb + 6] if kb is not None else np.zeros(3)
        pa = j.a.world_point(qa, j.point)
        if j.kind == "ball":
            c = getattr(j, "damper", 0.0) + extra_c
            if c:
                self._torque(F, idx, j, -c * (wb - wa))
            return
        if j.kind == "elastic":
            pb = j.b.world_point(self.m.pose(x, j.b), j.point)
            va = self._point_velocity(x, u, j.a, pa, idx)
            vb = self._point_velocity(x, u, j.b, pb, idx)
            f = -j.k_lin * (pb - pa) - j.c_lin * (vb - va)
            self._force_at(F, idx, x, j, pb, f)
            phi = Ra @ log_rotation(Ra.T @ j.b.rot(self.m.pose(x, j.b)))
            self._torque(F, idx, j, -j.k_rot * phi - j.c_rot * (wb - wa))
            return
        ax = Ra @ j.axis
        if j.kind in ("pivot", "cylindrical", "screw"):
            th = self.m.continuous_value(j, x)
            rate = float((wb - wa) @ ax)
            tau = motor - (getattr(j, "damper", 0.0) + extra_c) * rate
            if getattr(j, "spring", None):
                k, rest = j.spring
                tau -= k * (th - rest)
            if j.limits:
                tau += self._stop(self.m.joint_value(j, x), rate, j.limits,
                                  self._inertia_about(x, j.b, pa, ax)
                                  or self._inertia_about(x, j.a, pa, ax))
            if tau:
                self._torque(F, idx, j, tau * ax)
        if j.kind in ("slider", "cylindrical"):
            s_ = self.m.slide_value(j, x)
            va = self._point_velocity(x, u, j.a, pa, idx)
            vb = self._point_velocity(x, u, j.b, pa, idx)
            rate = float((vb - va) @ ax)
            if j.kind == "slider":
                f, spring, damper, lim = motor, j.spring, j.damper + extra_c, j.limits
            else:
                f, spring, damper, lim = 0.0, j.slide_spring, j.slide_damper, j.slide_limits
            f -= damper * rate
            if spring:
                f -= spring[0] * (s_ - spring[1])
            if lim:
                mb = self.mass.get(id(j.b)) or self.mass.get(id(j.a)) or 0.0
                f += self._stop(s_, rate, lim, mb)
            if f:
                self._force_at(F, idx, x, j, pa, f * ax)

    def _point_velocity(self, x, u, b, p, idx):
        k = idx.get(id(b))
        if k is None:
            return np.zeros(3)
        return u[6 * k: 6 * k + 3] + _cross(u[6 * k + 3: 6 * k + 6], p - self.cog(x, b))

    # -- stepping ---------------------------------------------------------------
    def motor_rates(self, t, h=1e-5):
        """Second derivative of every speed motor's law at ``t``."""
        out = []
        for j, n, drv in self._rows():
            if drv:
                f = self.motors[j.name][1]
                out.append((f(t + h) - 2 * f(t) + f(t - h)) / (h * h))
            else:
                out.append(np.zeros(n))
        return np.concatenate([np.atleast_1d(v) for v in out]) if out else np.zeros(0)

    def _kin(self, x, u, t, eps=1e-6):
        """(C, G, gamma) row block by row block: pivots analytically (the
        point and the axis of both parts), the other joints and the motors
        by finite differences on their own parts only."""
        idx = {id(b): k for k, b in enumerate(self.bodies)}
        Cs, Gs, gs = [], [], []
        piv = self._pivots_batch(x, u, idx)
        for j, n, drv in self._rows():
            Gb = np.zeros((n, self.n))
            if j.kind == "pivot" and not drv:
                C, g = self._pivot_fill(piv, j, Gb)
            else:
                parts = j.bodies() if j.kind == "coupling" else [j.a, j.b]
                ks = sorted({idx[id(o)] for o in parts if id(o) in idx})
                f = ((lambda xx: np.atleast_1d(self.m.continuous_value(j, xx))) if drv
                     else (lambda xx: np.atleast_1d(_solver._joint_residual(self.m, j, xx))))
                C = f(x)
                for k in ks:
                    for c in range(6):
                        du = np.zeros(self.n)
                        du[6 * k + c] = eps
                        Gb[:, 6 * k + c] = (f(self.displace(x, du)) - f(self.displace(x, -du))) / (2 * eps)
                um = np.zeros(self.n)
                for k in ks:
                    um[6 * k: 6 * k + 6] = u[6 * k: 6 * k + 6]
                nu = float(np.linalg.norm(um))
                if nu > 1e-12:
                    h = 1e-4 / nu
                    g = (f(self.displace(x, h * um)) - 2 * C + f(self.displace(x, -h * um))) / (h * h)
                else:
                    g = np.zeros(n)
                if drv:
                    C = C - self.motors[j.name][1](t)
            Cs.append(C)
            Gs.append(Gb)
            gs.append(g)
        if not Cs:
            return np.zeros(0), np.zeros((0, self.n)), np.zeros(0)
        return np.concatenate(Cs), np.vstack(Gs), np.concatenate(gs)

    def _pivots_batch(self, x, u, idx):
        """Every pivot's rows at once (same equations as :meth:`_pivot_rows`):
        {id(joint): (C, gamma, [(rows, column, 3x3 block)])}."""
        st = getattr(self, "_piv_static", None)
        if st is None:
            js = [j for j, n, drv in self._rows() if j.kind == "pivot" and not drv]

            def side(bodies):
                k = np.array([idx.get(id(b), -1) for b in bodies], int)
                mov = k >= 0
                rows = np.array([b.index for b, m_ in zip(bodies, mov) if m_], int)
                cen = np.array([b.center for b in bodies], float).reshape(-1, 3)
                com = np.array([self.com[id(b)] if id(b) in self.com else b.center
                                for b in bodies], float).reshape(-1, 3)
                return k, mov, rows, cen, com
            st = self._piv_static = (js, side([j.a for j in js]), side([j.b for j in js]),
                                     np.array([j.point for j in js], float).reshape(-1, 3),
                                     np.array([j.axis for j in js], float).reshape(-1, 3))
        js, sa, sb, P, A = st
        if not js:
            return {}
        X = np.asarray(x, float).reshape(-1, 6)
        u = np.asarray(u, float)
        U = u.reshape(-1, 6) if len(u) else np.zeros((0, 6))
        nj = len(js)

        def pose(sd):
            k, mov, rows, cen, com = sd
            R = np.repeat(_EYE3[None], nj, axis=0)
            t = np.zeros((nj, 3))
            w = np.zeros((nj, 3))
            if len(rows):
                q = X[rows]
                R[mov] = rotations(q[:, 3:])
                t[mov] = q[:, :3]
                w[mov] = U[k[mov], 3:]
            return k, mov, R, t, cen, w, com
        ka, ma, Ra, ta, ca, wa, coma = pose(sa)
        kb, mb, Rb, tb, cb, wb, comb = pose(sb)
        mv = lambda R, v: np.einsum("nij,nj->ni", R, v)
        pa = ca + ta + mv(Ra, P - ca)          # world_point on each side
        pb = cb + tb + mv(Rb, P - cb)
        ua, ub = mv(Ra, A), mv(Rb, A)
        cga = ca + ta + mv(Ra, coma - ca)
        cgb = cb + tb + mv(Rb, comb - cb)
        ra = np.where(ma[:, None], pa - cga, 0.0)
        rb = np.where(mb[:, None], pb - cgb, 0.0)
        C = np.hstack([pa - pb, _cross(ua, ub)])
        gam_p = _cross(wa, _cross(wa, ra)) - _cross(wb, _cross(wb, rb))
        dua, dub = _cross(wa, ua), _cross(wb, ub)
        gam_a = (_cross(_cross(wa, dua), ub) + 2 * _cross(dua, dub)
                 + _cross(ua, _cross(wb, dub)))
        G = np.hstack([gam_p, gam_a])
        Sk = lambda v: np.stack([np.stack([np.zeros(len(v)), -v[:, 2], v[:, 1]], 1),
                                 np.stack([v[:, 2], np.zeros(len(v)), -v[:, 0]], 1),
                                 np.stack([-v[:, 1], v[:, 0], np.zeros(len(v))], 1)], 1)
        Sua, Sub = Sk(ua), Sk(ub)
        Ba, Bb = Sub @ Sua, -(Sua @ Sub)
        Sra, Srb = Sk(ra), Sk(rb)
        out = {}
        for i, j in enumerate(js):
            ent = []
            if ma[i]:
                k = ka[i]
                ent += [(slice(0, 3), 6 * k, _EYE3), (slice(0, 3), 6 * k + 3, -Sra[i]),
                        (slice(3, 6), 6 * k + 3, Ba[i])]
            if mb[i]:
                k = kb[i]
                ent += [(slice(0, 3), 6 * k, -_EYE3), (slice(0, 3), 6 * k + 3, Srb[i]),
                        (slice(3, 6), 6 * k + 3, Bb[i])]
            out[id(j)] = (C[i], G[i], ent)
        return out

    @staticmethod
    def _pivot_fill(piv, j, Gb):
        C, g, ent = piv[id(j)]
        for rows, col, block in ent:
            Gb[rows, col: col + 3] = block
        return C, g

    def _pivot_rows(self, j, x, u, Gb, idx):
        """Pivot: same point (3 rows) and same axis (ua x ub, 3 rows)."""
        out_p, out_a, gam_p, gam_a = 0, 0, np.zeros(3), np.zeros(3)
        pts, axes, oms = [], [], []
        for body in (j.a, j.b):
            q = self.m.pose(x, body)
            pts.append(body.world_point(q, j.point))
            axes.append(body.rot(q) @ j.axis)
            k = idx.get(id(body))
            oms.append(u[6 * k + 3: 6 * k + 6] if k is not None else np.zeros(3))
        (pa, pb), (ua, ub), (wa, wb) = pts, axes, oms
        C = np.concatenate([pa - pb, _cross(ua, ub)])
        for s, body, p, w in ((1.0, j.a, pa, wa), (-1.0, j.b, pb, wb)):
            k = idx.get(id(body))
            r = p - (self.cog(x, body) if k is not None else p)
            if k is not None:
                Gb[0:3, 6 * k: 6 * k + 3] = s * _EYE3
                Gb[0:3, 6 * k + 3: 6 * k + 6] = -s * _skew(r)
            gam_p += s * _cross(w, _cross(w, r))
        ka, kb = idx.get(id(j.a)), idx.get(id(j.b))
        if ka is not None:
            Gb[3:6, 6 * ka + 3: 6 * ka + 6] = _skew(ub) @ _skew(ua)
        if kb is not None:
            Gb[3:6, 6 * kb + 3: 6 * kb + 6] = -_skew(ua) @ _skew(ub)
        dua, dub = _cross(wa, ua), _cross(wb, ub)
        gam_a = (_cross(_cross(wa, dua), ub) + 2 * _cross(dua, dub)
                 + _cross(ua, _cross(wb, dub)))
        return C, np.concatenate([gam_p, gam_a])

    def accelerations(self, x, u, t):
        """(a, lambda, G): accelerations (cog, angular) and the joint
        multipliers, from M a + G^T nu = F, G a = rhs, solved through
        (G M^-1 G^T) nu (least squares: over-constrained loops are fine)."""
        R, cog, Iw, F = self._blocks(x, u, t)
        C, Gm, gamma = self._kin(x, u, t)
        nb = len(self.bodies)
        if nb:
            mass = self._arrays()["mass"]
            Ii = self._inverse_inertias(R)

            def Mi(V):                         # M^-1 V, block by block
                V = np.asarray(V, float)
                W = V.reshape((nb, 6) + V.shape[1:])
                out = np.empty_like(W)
                out[:, :3] = W[:, :3] / mass.reshape((nb, 1) + (1,) * (V.ndim - 1))
                out[:, 3:] = np.einsum("nij,nj...->ni...", Ii, W[:, 3:])
                return out.reshape(V.shape)
        else:
            Mi = lambda V: np.zeros_like(np.asarray(V, float))
        if Gm.shape[0]:
            dC = Gm @ u - self._motor_speeds(t)
            rhs_c = -gamma - 2 * self.k * dC - self.k ** 2 * C + self.motor_rates(t)
            MiGt = Mi(Gm.T)
            A = Gm @ MiGt
            nu = np.linalg.lstsq(A, Gm @ Mi(F) - rhs_c, rcond=1e-12)[0]
            a = Mi(F) - MiGt @ nu
        else:
            nu = np.zeros(0)
            a = Mi(F)
        return a, -nu, Gm

    def _motor_speeds(self, t, h=1e-5):
        out = []
        for j, n, drv in self._rows():
            if drv:
                f = self.motors[j.name][1]
                out.append([(f(t + h) - f(t - h)) / (2 * h)])
            else:
                out.append(np.zeros(n))
        return np.concatenate(out) if out else np.zeros(0)

    def _C_geom(self, x):
        out = []
        for j, n, drv in self._rows():
            out.append([self.m.continuous_value(j, x)] if drv
                       else _solver._joint_residual(self.m, j, x))
        return np.concatenate([np.atleast_1d(v) for v in out]) if out else np.zeros(0)

    def step(self, x, u, t, dt):
        """Runge-Kutta 4 on (pose, velocities); the pose moves in the cog
        coordinates (``displace``). Exact under constant forces.
        ``self.method = "symplectic"``: semi-implicit Euler (velocities, then
        the pose with the new ones), one evaluation a step — for stiff
        contacts at a small step, where it tracks a fine reference closely;
        ``"rk2"``: the midpoint rule."""
        a1, lam, _ = self.accelerations(x, u, t)
        method = getattr(self, "method", "rk4")
        if method == "symplectic":             # semi-implicit Euler: one evaluation
            u_new = u + dt * a1
            return self.displace(x, dt * u_new), u_new, a1, lam
        if method == "rk2":                     # midpoint: two evaluations
            um = u + 0.5 * dt * a1
            am = self.accelerations(self.displace(x, 0.5 * dt * u), um, t + 0.5 * dt)[0]
            return self.displace(x, dt * um), u + dt * am, a1, lam
        u2 = u + 0.5 * dt * a1
        a2 = self.accelerations(self.displace(x, 0.5 * dt * u), u2, t + 0.5 * dt)[0]
        u3 = u + 0.5 * dt * a2
        a3 = self.accelerations(self.displace(x, 0.5 * dt * u2), u3, t + 0.5 * dt)[0]
        u4 = u + dt * a3
        a4 = self.accelerations(self.displace(x, dt * u3), u4, t + dt)[0]
        x_new = self.displace(x, dt / 6.0 * (u + 2 * u2 + 2 * u3 + u4))
        u_new = u + dt / 6.0 * (a1 + 2 * a2 + 2 * a3 + a4)
        return x_new, u_new, a1, lam

    def energy(self, x, u):
        """(kinetic, potential) in J (potential zero at the origin)."""
        ke = pe = 0.0
        for k, b in enumerate(self.bodies):
            R = self._R(x, b)
            Iw = R @ self.Iref[id(b)] @ R.T
            v, w = u[6 * k: 6 * k + 3], u[6 * k + 3: 6 * k + 6]
            m = self.mass[id(b)]
            ke += 0.5 * m * v @ v + 0.5 * w @ Iw @ w
            pe -= m * self.g @ self.cog(x, b)
        return ke, pe

    def reactions(self, lam):
        """{joint name: multipliers} (a speed motor: its torque / force under
        the key "<name> (moteur)")."""
        out, at = {}, 0
        for j, n, drv in self._rows():
            out[j.name + (" (moteur)" if drv else "")] = lam[at: at + n]
            at += n
        return out

    def run(self, seconds, dt=1e-3, x0=None, u0=None, every=1, t0=0.0, progress=None):
        """Simulate; returns rows {t, x, u, ke, pe, lam} every ``every``
        steps. ``x0`` defaults to the drawn pose (placed on the joints and
        on the speed motors' laws at t0). ``progress(i, steps)`` is called
        now and then; when it returns False the run stops and returns None."""
        x = np.zeros(self.m.n) if x0 is None else np.array(x0, float)
        u = np.zeros(self.n) if u0 is None else np.array(u0, float)
        x = self.settle(x, t0)
        u = self.project_velocity(x, u, t0)
        if self.impacts is not None:
            self.impacts.dt = dt
        rows = []
        t = t0
        steps = int(round(seconds / dt))
        for i in range(steps + 1):
            if i % every == 0 or i == steps:
                ke, pe = self.energy(x, u)
                rows.append(dict(t=t, x=x.copy(), u=u.copy(), ke=ke, pe=pe))
            if i == steps:
                break
            if progress is not None and i % 100 == 0 and progress(i, steps) is False:
                return None
            x, u, a, lam = self.step(x, u, t, dt)
            if rows and rows[-1]["t"] == t:
                rows[-1]["lam"] = lam
            t += dt
        return rows

    def settle(self, x, t, iters=30):
        """Pose on the joints (and the speed motors' values at ``t``):
        Gauss-Newton in the cog coordinates, least motion."""
        for _ in range(iters):
            C = self.C(x, t)
            if np.linalg.norm(C) < 1e-12:
                break
            Gm = self.G(x, t)
            du = -np.linalg.lstsq(Gm, C, rcond=None)[0]
            x = self.displace(x, du)
        return x

    def project_velocity(self, x, u, t):
        """Velocities that respect the joints (and the speed motors' laws):
        the nearest, in the sense of the kinetic energy."""
        Gm = self.G(x, t)
        if not Gm.shape[0]:
            return u
        target = np.zeros(Gm.shape[0])
        at = 0
        for j, n, drv in self._rows():
            if drv:
                f = self.motors[j.name][1]
                target[at] = (f(t + 1e-5) - f(t - 1e-5)) / 2e-5
            at += n
        M, _ = self.M_and_forces(x, u, t)
        Mi = np.linalg.pinv(M)
        A = Gm @ Mi @ Gm.T
        mu = np.linalg.lstsq(A, target - Gm @ u, rcond=None)[0]
        return u + Mi @ Gm.T @ mu
