"""Contact: parts pushed by others, each on its own pivot (or slider) — a
Geneva wheel on the frame, a pawl with play on its rocker, a cam follower.

The pushed parts are not solved with the joints: once they are solved,
they are placed by this rule (``Mechanism.followers``), quasi-statically.
Each keeps the angle it had on its carrier (the part its pivot holds it
to); when a pushing part would go into it, it turns just enough, the way
the push goes, for the two to touch without going through. So it stands
still while nothing pushes it, then follows the push: the dwell and the
step of an intermittent drive. Pushed parts may push each other (a pawl
and the wheel it drives): they are settled together, a few rounds at each
sub-step. The real surfaces of the parts are used (``contact.Surfaces``),
the motion in sub-steps short enough that nothing jumps through a part.
"""
import math
import numpy as np
from .model import rotation


def rotvec(R):
    """Rotation vector of rotation matrix ``R`` (inverse of ``rotation``)."""
    c = max(-1.0, min(1.0, 0.5 * (float(np.trace(R)) - 1.0)))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-6:                       # half a turn: from R + I
        M = 0.5 * (R + np.eye(3))
        k = int(np.argmax(np.diag(M)))
        a = M[:, k] / math.sqrt(max(M[k, k], 1e-12))
        return a * th
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (th / (2.0 * math.sin(th)))


class Pushed:
    """``body`` on ``joint`` (a pivot or a slider) with its carrier (the
    joint's other part), pushed by ``pushers`` (bodies)."""

    def __init__(self, body, joint, pushers):
        self.body, self.joint = body, joint
        self.carrier = joint.a if joint.b is body else joint.b
        self.kind = "slider" if joint.kind == "slider" else "pivot"
        self.point = np.asarray(joint.point, float)       # reference pose
        ax = np.asarray(joint.axis, float)
        self.axis = ax / max(np.linalg.norm(ax), 1e-12)
        self.sign = 1.0 if joint.b is body else -1.0       # body turns +v about +axis
        self.pushers = list(pushers)
        self.value = 0.0

    def pose_of(self, mech, x, v):
        """Pose (6) of the body at angle (or travel) ``v`` on its carrier
        as posed in ``x``."""
        s = self.sign * v
        if self.kind == "pivot":
            Ra, d = rotation(self.axis * s), np.zeros(3)
        else:
            Ra, d = np.eye(3), self.axis * s
        C, F = self.carrier, self.body
        qc = mech.pose(x, C)
        RC = C.rot(qc)
        tC = qc[:3] if not C.fixed else np.zeros(3)
        cC = C.center if not C.fixed else np.zeros(3)
        RF = RC @ Ra
        q = np.zeros(6)
        q[:3] = cC + tC + RC @ (self.point + d - cC) - F.center + RF @ (F.center - self.point)
        q[3:] = rotvec(RF)
        return q

    def lever(self, pt, mech, x):
        if self.kind != "pivot" or pt is None:
            return 1.0
        qc = mech.pose(x, self.carrier)
        c = self.carrier.world_point(qc, self.point)
        a = self.carrier.rot(qc) @ self.axis
        d = pt - c
        return max(float(np.linalg.norm(d - (d @ a) * a)), 1e-4)


class PushGroup:
    """Every pushed part of a mechanism, settled together (one follower)."""

    dynamic = False
    free_dof = 0

    def __init__(self, members, name="contact"):
        # carriers first: a part pushed on a pushed part comes after it
        order, left = [], list(members)
        while left:
            ready = [m for m in left
                     if id(m.carrier) not in {id(o.body) for o in left if o is not m}]
            ready = ready or left[:1]
            for m in ready:
                order.append(m)
                left.remove(m)
        self.members = order
        self.bodies = [m.body for m in order]
        self.name = name
        self._x = None
        self.pushes = 0
        self.jams = 0

    def reset(self):
        self._x = None
        for m in self.members:
            m.value = 0.0

    def _gap(self, mech, S, x, m, v, reach):
        qa = m.pose_of(mech, x, v)
        best, pt = math.inf, None
        for p in m.pushers:
            if not (S.has(m.body) and S.has(p)):
                continue
            r = S.pair(m.body, p, qa, mech.pose(x, p), reach, resting=False)
            if r is not None and r[0] < best:
                best, pt = r[0], r[1]
        return best, pt

    def _free(self, mech, S, x, m, reach, tol):
        """``m``'s value nearest its own where nothing goes into it, found
        the way the push goes; (value, pushed)."""
        v = m.value
        g, pt = self._gap(mech, S, x, m, v, reach)
        if g >= -tol:
            return v, False
        step = 0.5 * S.h / m.lever(pt, mech, x)
        gp = self._gap(mech, S, x, m, v + step, reach)[0]
        gm = self._gap(mech, S, x, m, v - step, reach)[0]
        sgn = 1.0 if gp >= gm else -1.0
        worst = g
        for k in range(1, 120):
            w = v + sgn * k * step
            gk = self._gap(mech, S, x, m, w, reach)[0]
            if gk >= -tol:
                lo, hi = v + sgn * (k - 1) * step, w
                for _ in range(8):
                    mid = 0.5 * (lo + hi)
                    if self._gap(mech, S, x, m, mid, reach)[0] >= -tol:
                        hi = mid
                    else:
                        lo = mid
                return hi, True
            if gk < worst - 0.5 * S.h:          # deeper: not the way out
                break
            worst = min(worst, gk)
        self.jams += 1
        return v, True                          # held from both sides: stays (the check says so)

    def _write(self, mech, x):
        for m in self.members:
            i = m.body.index
            x[6 * i: 6 * i + 6] = m.pose_of(mech, x, m.value)
        return x

    def place(self, mech, x):
        x = np.array(x, float)
        S = getattr(mech, "surfaces", None)
        if S is None or not S.data:
            return self._write(mech, x)
        reach, tol = 2.0 * S.h, S.tol
        prev = self._x if self._x is not None and len(self._x) == len(x) else None
        n = 1
        if prev is not None:                     # no point moves more than h / 2 a sub-step
            mine = {id(b) for b in self.bodies}
            for p in {id(p): p for m in self.members for p in m.pushers + [m.carrier]}.values():
                if p.fixed or id(p) in mine:
                    continue
                corners = [b.center + (b.R if b.R is not None else np.eye(3)) @ (np.array(s) * b.half)
                           for b in p.boxes
                           for s in ((1, 1, 1), (-1, -1, -1), (1, -1, 1), (-1, 1, -1))] or [p.center]
                qa, qb = mech.pose(prev, p), mech.pose(x, p)
                move = max(float(np.linalg.norm(p.world_point(qb, c) - p.world_point(qa, c)))
                           for c in corners)
                n = max(n, int(math.ceil(move / (0.5 * S.h))))
            n = min(n, 400)
        for s in range(1, n + 1):
            xs = x.copy() if prev is None or s == n else prev + (x - prev) * (s / n)
            xs = self._write(mech, xs)
            for _ in range(4):                   # pushed parts pushing each other
                moved = False
                for m in self.members:
                    v, pushed = self._free(mech, S, xs, m, reach, tol)
                    if pushed:
                        self.pushes += 1
                    if v != m.value:
                        m.value = v
                        moved = True
                        xs = self._write(mech, xs)
                if not moved:
                    break
        x = self._write(mech, x)
        self._x = x.copy()
        return x
