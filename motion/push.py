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
        joint.placed = True                     # the solver leaves it to the contact
        self.carrier = joint.a if joint.b is body else joint.b
        self.kind = "slider" if joint.kind == "slider" else "pivot"
        self.point = np.asarray(joint.point, float)       # reference pose
        ax = np.asarray(joint.axis, float)
        self.axis = ax / max(np.linalg.norm(ax), 1e-12)
        self.sign = 1.0 if joint.b is body else -1.0       # body turns +v about +axis
        # the carrier among the pushers: what stops the part on it (the ends
        # of a slot its pin runs in) becomes its stops, found once; rubbing
        # along the slot's sides or round its hub is not a push
        self.on_carrier = any(p is self.carrier for p in pushers)
        self.pushers = [p for p in pushers if p is not self.carrier]
        lim = getattr(joint, "limits", None)
        self.limits = None
        if lim:
            lo, hi = sorted(lim)
            self.limits = (lo, hi) if joint.b is body else (-hi, -lo)
        self.value = 0.0
        self.cog = None                      # reference pose; set by the group

    def height(self, mech, x, v):
        """Height (z) of its centre of gravity at ``v``: what its weight
        lowers when nothing holds it."""
        q = self.pose_of(mech, x, v)
        return float(self.body.world_point(q, self.cog)[2])

    def clamp(self, v):
        if self.limits is None:
            return v
        return min(max(v, self.limits[0]), self.limits[1])

    def find_stops(self, mech, S, x):
        """Stops on the carrier: turned (or slid) both ways from as drawn
        until it really goes into the carrier (deeper than 1.5 sampling
        steps: rubbing is not a stop)."""
        if not self.on_carrier or not (S.has(self.body) and S.has(self.carrier)):
            return
        P = S.data[self.body.name]["P"]
        d = P - self.point
        r = float(np.max(np.linalg.norm(d - np.outer(d @ self.axis, self.axis), axis=1))) \
            if self.kind == "pivot" else 1.0
        step = (1.0 * S.h) / max(r, 1e-4)
        span = math.pi if self.kind == "pivot" else 0.5 * float(np.ptp(P @ self.axis))
        qc = mech.pose(x, self.carrier)
        base = S.overlap_count(self.body, self.carrier, self.pose_of(mech, x, 0.0), qc)
        margin = max(4, int(0.1 * base))
        stops = []
        for sgn in (-1.0, 1.0):
            v, last = 0.0, 0.0
            while abs(v) < span:
                v += sgn * step
                n = S.overlap_count(self.body, self.carrier, self.pose_of(mech, x, v), qc)
                if n > base + margin:                    # runs into the carrier
                    break
                last = v
            stops.append(last - sgn * step if last != 0.0 else 0.0)   # just before it
        lo, hi = stops
        if self.limits is not None:
            lo, hi = max(lo, self.limits[0]), min(hi, self.limits[1])
        self.limits = (lo, hi)
        self.on_carrier = False

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
        # who gives way first: a part with play on a moving carrier (a pawl)
        # turns until its stop before it drives a part on the frame (the
        # wheel) — kept stable, carriers still before what they carry
        rank = {id(m): k for k, m in enumerate(order)}
        order.sort(key=lambda m: (m.carrier.fixed, rank[id(m)]))
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

    @staticmethod
    def _box(body, q):
        """World bounds of a body's boxes at pose q (a quick test before
        the surfaces)."""
        lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
        R = body.rot(q)
        for b in body.boxes:
            Rb = b.R if b.R is not None else np.eye(3)
            c = body.world_point(q, b.center)
            ext = np.abs(R @ Rb) @ np.asarray(b.half, float)
            lo, hi = np.minimum(lo, c - ext), np.maximum(hi, c + ext)
        return lo, hi

    def _gap(self, mech, S, x, m, v, reach):
        qa = m.pose_of(mech, x, v)
        best, pt = math.inf, None
        la, ha = self._box(m.body, qa) if m.body.boxes else (None, None)
        for p in m.pushers:
            if not (S.has(m.body) and S.has(p)):
                continue
            if la is not None and p.boxes:
                lb, hb = self._box(p, mech.pose(x, p))
                if np.any(lb - reach > ha) or np.any(la - reach > hb):
                    continue                    # boxes apart: nothing to touch
            # what touches as drawn (two pawls on one hub) is the assembly,
            # not a push
            # (as tight as the sampling allows: a roller 1 mm off still pushes)
            r = S.pair(m.body, p, qa, mech.pose(x, p), reach, resting=0.75)
            if r is not None and r[0] < best:
                best, pt = r[0], r[1]
        return best, pt

    def _free(self, mech, S, x, m, reach, tol):
        """``m``'s value nearest its own where nothing goes into it, found
        the way the push goes, within its stops; (value, pushed)."""
        v = m.value
        g, pt = self._gap(mech, S, x, m, v, reach)
        if g >= -tol:
            return v, False
        step = 0.5 * S.h / m.lever(pt, mech, x)
        gp = self._gap(mech, S, x, m, v + step, reach)[0]
        gm = self._gap(mech, S, x, m, v - step, reach)[0]
        sgn = 1.0 if gp >= gm else -1.0
        worst = g
        # pushed out to touching (not to the tolerance: the check would
        # see it in by a hair)
        for k in range(1, 120):
            w = v + sgn * k * step
            gk = self._gap(mech, S, x, m, w, reach)[0]
            if m.clamp(w) != w:                 # on its stop: it pushes back instead
                return m.clamp(w), True
            if gk >= 0.0:
                lo, hi = v + sgn * (k - 1) * step, w
                for _ in range(8):
                    mid = 0.5 * (lo + hi)
                    if self._gap(mech, S, x, m, mid, reach)[0] >= 0.0:
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
        if self._x is None:                      # as drawn: the stops on the carriers
            x0 = self._write(mech, x.copy())
            for m in self.members:
                m.find_stops(mech, S, x0)
                if m.cog is None:                # its own, else the middle of its surface
                    com = getattr(m.body, "com", None)
                    m.cog = (np.asarray(com, float) if com is not None
                             else S.data[m.body.name]["P"].mean(axis=0)
                             if S.has(m.body) else m.body.center)
        prev = self._x if self._x is not None and len(self._x) == len(x) else None
        n = 1
        if prev is not None:                     # no point moves more than 0.8 h a sub-step
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
                n = max(n, int(math.ceil(move / (0.8 * S.h))))
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
        if self.gravity:
            x = self._write(mech, x)
            for _ in range(2):                    # pushed parts resting on each other
                for m in self.members:
                    if not m.carrier.fixed:
                        continue                  # a pawl rides its carrier: held by it
                    self._fall(mech, S, x, m, reach, tol)
                    x = self._write(mech, x)
        x = self._write(mech, x)
        self._x = x.copy()
        return x

    gravity = True
    fall_steps = 12                               # a little at each position, not a drop

    def _fall(self, mech, S, x, m, reach, tol):
        """Let ``m`` go down under its weight until it rests on something
        (a pushing part, another pushed part) or on its stop."""
        g, pt = self._gap(mech, S, x, m, m.value, reach)
        step = 0.5 * S.h / m.lever(pt if pt is not None else
                                   m.body.world_point(m.pose_of(mech, x, m.value), m.cog),
                                   mech, x)
        h0 = m.height(mech, x, m.value)
        sgn = -1.0 if m.height(mech, x, m.value + step) > h0 else 1.0
        if abs(m.height(mech, x, m.value + sgn * step) - h0) < 1e-7:
            return                                # balanced (its weight on the axis)
        v = m.value
        for _ in range(self.fall_steps):
            w = m.clamp(v + sgn * step)
            if w == v or m.height(mech, x, w) >= m.height(mech, x, v):
                break
            if self._gap(mech, S, x, m, w, reach)[0] < 0.0:
                break                             # it lands on something
            v = w
        m.value = v
