"""Roller chain on two sprockets, with a slack strand that hangs.

The chain lies in the x-z plane of the mechanism (z up, sprocket axes along
y). The driving sprocket pulls the upper strand, which stays straight
(tangent to both pitch circles); the lower strand gets whatever length is
left and hangs under its own weight: a catenary leaving the driving
sprocket and reaching the driven one, tangent to both pitch circles.

Lengths are counted in pitches, as a chain counts them: on a sprocket of
z teeth a pitch is 2 pi / z of a turn (its chord is exactly the pitch),
on the straight strand and along the catenary a pitch is its length. A
link moves one pitch when the driving sprocket turns one tooth, so the
pins on a sprocket stay in its tooth gaps.

Every link is a part of the mechanism placed by this rule once the joints
are solved (``Mechanism.followers``): it is drawn, animated and checked for
clearance with the rest of the model like any part.
"""
import math
import numpy as np

AXIS = np.array([0.0, -1.0, 0.0])          # positive angle turns x towards z


def pitch_radius(pitch, teeth):
    """Pitch circle radius: a chord of 2 pi / z is one pitch."""
    return pitch / (2.0 * math.sin(math.pi / teeth))


def _ang(v):
    return math.atan2(v[1], v[0])


def _outer_tangent(c1, r1, c2, r2):
    """Upper outer tangent of two circles (2D): contact angles on 1 and 2
    (from the centres), for a strand above the line of centres when c2 is
    to the right of c1."""
    d = c2 - c1
    L = float(np.hypot(*d))
    base = _ang(d)
    a = base + math.pi / 2 + math.asin((r2 - r1) / L)
    a2 = a
    return a, a2


class Chain:
    """Chain on sprocket ``s1`` (driving, pivot joint ``j1``, ``z1`` teeth)
    and ``s2`` (driven, ``j2``, ``z2``; the chain turns it: ``j2`` gets no
    coupling, its angle is set from the chain). ``c1``/``c2``: sprocket centres in
    their bodies' reference pose; ``links``: number of pitches (even);
    ``y``: mid-plane of the chain along the axes."""

    def __init__(self, mech, j1, b1, c1, z1, j2, b2, c2, z2, pitch, links, y=0.0,
                 roller=None, width=None, name="chaîne"):
        self.j1, self.b1, self.c1, self.z1 = j1, b1, np.asarray(c1, float), int(z1)
        self.j2, self.b2, self.c2, self.z2 = j2, b2, np.asarray(c2, float), int(z2)
        self.p, self.N, self.y = float(pitch), int(links), float(y)
        self.r1, self.r2 = pitch_radius(pitch, z1), pitch_radius(pitch, z2)
        self.roller = roller or 0.62 * pitch          # roller diameter
        self.width = width or 0.62 * pitch            # inner width
        self.name = name
        self._guess = None
        self.state = None
        self._anchor = None                        # (n_top, aT1, aT2) as drawn
        self.free_dof = 1                          # the driven sprocket's turn
        g = self.geometry(0.0, self._centres(mech, np.zeros(mech.n)))
        self.ref = g
        self.bodies = []
        for k in range(self.N):
            P, Q = g["pins"][k], g["pins"][(k + 1) % self.N]
            mid = 0.5 * (P + Q)
            b = mech.body(f"{name} {k + 1:03d}", self._w(mid))
            b.boxes.append(self._box(P, Q, k))
            b.color = (70, 74, 82) if k % 2 else (150, 154, 162)
            b.pins = (k, (k + 1) % self.N)
            self.bodies.append(b)
            mech.no_contact.add(frozenset((id(b), id(b1))))
            mech.no_contact.add(frozenset((id(b), id(b2))))
        mech.followers.append(self)

    # -- 2D <-> 3D (x, z plane at y) ------------------------------------
    def _w(self, p):
        return np.array([p[0], self.y, p[1]], float)

    def _box(self, P, Q, k):
        from .model import Box
        d = Q - P
        a = _ang(d)
        R = np.array([[math.cos(a), 0, -math.sin(a)], [0, 1, 0], [math.sin(a), 0, math.cos(a)]])
        outer = k % 2 == 0
        half_w = 0.5 * self.width + (0.12 * self.p if outer else 0.06 * self.p)
        return Box(self._w(0.5 * (P + Q)), (0.5 * self.p + 0.5 * self.roller, half_w,
                                           0.42 * self.roller), R, name="maillon")

    def _centres(self, mech, x):
        w1 = self.b1.world_point(mech.pose(x, self.b1), self.c1)
        w2 = self.b2.world_point(mech.pose(x, self.b2), self.c2)
        return np.array([w1[0], w1[2]]), np.array([w2[0], w2[2]])

    # -- the shape ------------------------------------------------------
    def geometry(self, theta1, centres):
        """Pins (2D) and strands for driving-sprocket angle ``theta1``."""
        S1, S2 = centres
        r1, r2, p, N, z1, z2 = self.r1, self.r2, self.p, self.N, self.z1, self.z2
        aT1, aT2 = _outer_tangent(S1, r1, S2, r2)
        T1 = S1 + r1 * np.array([math.cos(aT1), math.sin(aT1)])
        T2 = S2 + r2 * np.array([math.cos(aT2), math.sin(aT2)])
        n_top = float(np.hypot(*(T1 - T2))) / p
        sol = self._slack(S1, S2, aT1, aT2, n_top)
        aB1, aB2, c, u0, v0, Lc, taut = sol
        A1 = (aB1 - aT1) % (2 * math.pi)             # wrap on 1, CCW from T1
        A2 = (aT2 - aB2) % (2 * math.pi)             # wrap on 2, CCW from B2
        n1, n2 = A1 * z1 / (2 * math.pi), A2 * z2 / (2 * math.pi)
        nc = N - n_top - n1 - n2
        B1 = S1 + r1 * np.array([math.cos(aB1), math.sin(aB1)])
        B2 = S2 + r2 * np.array([math.cos(aB2), math.sin(aB2)])
        marks = np.cumsum([0, n_top, n1, nc, n2])
        # pitches travelled. The pins on the driving sprocket stay in its
        # tooth gaps: when the upper strand gets longer (centres moved
        # apart) the chain is pulled round the driven sprocket, not slipped
        # on the driving one.
        if self._anchor is None:
            self._anchor = (n_top, aT1, aT2)
        n0, a10, a20 = self._anchor
        phase = theta1 * z1 / (2 * math.pi) + (n_top - n0) - (aT1 - a10) * z1 / (2 * math.pi)
        theta2 = phase * 2 * math.pi / z2 + (aT2 - a20)   # driven sprocket

        def at(u):
            u = u % N
            if u < marks[1]:                           # top: T2 -> T1
                return T2 + (T1 - T2) * (u / n_top)
            if u < marks[2]:                           # around 1
                a = aT1 + (u - marks[1]) * 2 * math.pi / z1
                return S1 + r1 * np.array([math.cos(a), math.sin(a)])
            if u < marks[3]:                           # slack strand
                f = (u - marks[2]) / nc
                if taut:
                    return B1 + (B2 - B1) * f
                s_a = math.sinh((B1[0] - u0) / c)
                s = s_a + f * Lc / c
                xx = u0 + c * math.asinh(s)
                return np.array([xx, v0 + c * math.cosh((xx - u0) / c)])
            a = aB2 + (u - marks[3]) * 2 * math.pi / z2    # around 2
            return S2 + r2 * np.array([math.cos(a), math.sin(a)])

        pins = [at(k + phase) for k in range(N)]
        # sag: largest distance below the straight line B1-B2
        sag = 0.0
        if not taut:
            d = (B2 - B1) / np.linalg.norm(B2 - B1)
            nrm = np.array([d[1], -d[0]])
            for f in np.linspace(0, 1, 41):
                q = at(marks[2] + f * nc)
                sag = max(sag, float((q - B1) @ nrm))
        return dict(pins=pins, u=[(k + phase) % N for k in range(N)], marks=marks,
                    S=(S1, S2), theta2=theta2, T1=T1, T2=T2, B1=B1, B2=B2, aB1=aB1, aB2=aB2,
                    n_top=n_top, n1=n1, n2=n2, nc=nc, slack_len=Lc, catenary=(c, u0, v0),
                    sag=sag, taut=taut, span=float(np.hypot(*(S2 - S1))),
                    chord=float(np.hypot(*(B2 - B1))))

    def _slack(self, S1, S2, aT1, aT2, n_top):
        """Departure angle on 1, arrival angle on 2 and the catenary
        (parameter c, vertex u0, v0) of the hanging strand: tangent to both
        pitch circles, its length what the chain has left."""
        r1, r2, p, N, z1, z2 = self.r1, self.r2, self.p, self.N, self.z1, self.z2

        def length_left(a1, a2):
            A1 = (a1 - aT1) % (2 * math.pi)
            A2 = (aT2 - a2) % (2 * math.pi)
            return (N - n_top - A1 * z1 / (2 * math.pi) - A2 * z2 / (2 * math.pi)) * p

        def curve(a1, c):
            B1 = S1 + r1 * np.array([math.cos(a1), math.sin(a1)])
            slope = math.cos(a1) / -math.sin(a1)      # travel (-sin a, cos a)
            u0 = B1[0] - c * math.asinh(slope)
            v0 = B1[1] - c * math.cosh((B1[0] - u0) / c)
            return B1, u0, v0

        def F(v):
            try:
                return _F(v)
            except (OverflowError, ValueError, ZeroDivisionError):
                return np.full(3, 1e6)

        def _F(v):
            a1, a2, lc = v
            c = math.exp(min(lc, 30.0))
            B1, u0, v0 = curve(a1, c)
            B2 = S2 + r2 * np.array([math.cos(a2), math.sin(a2)])
            on = v0 + c * math.cosh((B2[0] - u0) / c) - B2[1]
            slope2 = math.cos(a2) / -math.sin(a2)
            tang = math.sinh((B2[0] - u0) / c) - slope2
            L = c * (math.sinh((B2[0] - u0) / c) - math.sinh((B1[0] - u0) / c))
            return np.array([on, tang, (L - length_left(a1, a2)) / p])

        # too tight: the straight bottom tangent already takes all the length
        a_lo = _ang(S2 - S1) - math.pi / 2 - math.asin((r2 - r1) / float(np.hypot(*(S2 - S1))))
        B1s = S1 + r1 * np.array([math.cos(a_lo), math.sin(a_lo)])
        B2s = S2 + r2 * np.array([math.cos(a_lo), math.sin(a_lo)])
        left = length_left(a_lo, a_lo)
        straight = float(np.hypot(*(B2s - B1s)))
        if left <= straight * (1 + 1e-9):
            return a_lo, a_lo, 1.0, 0.0, 0.0, straight, True
        start = np.array([a_lo - 0.05, a_lo + 0.05, math.log(max(straight, 1e-3))])
        v = self._newton(F, self._guess if self._guess is not None else start)
        if np.linalg.norm(F(v)) > 1e-8:                 # lost: start again
            v = self._newton(F, start)
        if np.linalg.norm(F(v)) > 1e-6:                 # (nearly) no slack left
            return a_lo, a_lo, 1.0, 0.0, 0.0, straight, True
        self._guess = v
        a1, a2, lc = v
        c = math.exp(lc)
        B1, u0, v0 = curve(a1, c)
        return a1, a2, c, u0, v0, length_left(a1, a2), False

    @staticmethod
    def _newton(F, v):
        v = np.array(v, float)
        for _ in range(60):
            f = F(v)
            if np.linalg.norm(f) < 1e-12:
                break
            J = np.zeros((3, 3))
            for i in range(3):
                h = 1e-7
                dv = np.zeros(3); dv[i] = h
                J[:, i] = (F(v + dv) - F(v - dv)) / (2 * h)
            step = np.linalg.lstsq(J, -f, rcond=None)[0]
            t = 1.0
            while t > 1e-4 and np.linalg.norm(F(v + t * step)) >= np.linalg.norm(f):
                t *= 0.5
            v = v + t * step
        return v

    # -- follower -------------------------------------------------------
    def place(self, mech, x):
        """Chain at rest for the pose ``x`` (quasi-static)."""
        theta = mech.continuous_value(self.j1, x)
        g = self.geometry(theta, self._centres(mech, x))
        self.state = g
        return self._write(mech, x, g["pins"], g["theta2"])

    def _write(self, mech, x, pins, theta2):
        x = np.array(x, float)
        ref = self.ref["pins"]
        for b in self.bodies:
            i, k = b.pins
            P0, Q0 = ref[i], ref[k]
            P, Q = pins[i], pins[k]
            a = _ang(Q - P) - _ang(Q0 - P0)
            q = np.zeros(6)
            q[:3] = self._w(0.5 * (P + Q)) - self._w(0.5 * (P0 + Q0))
            q[3:] = AXIS * a
            x[6 * b.index: 6 * b.index + 6] = q
        # the driven sprocket turns with the chain (on its carrier)
        qa = mech.pose(x, self.j2.a)
        i = self.b2.index
        x[6 * i + 3: 6 * i + 6] = qa[3:] + self.j2.axis * theta2
        return x

    # -- dynamics ---------------------------------------------------------
    # The links on the sprockets and in the upper strand follow the
    # sprockets (the strand is under tension: straight). The pins of the
    # slack strand are point masses: weight, damping, one pitch between
    # neighbours (position-based dynamics, Verlet velocities), kept off the
    # sprockets. A pin that leaves the driving sprocket keeps the speed it
    # had; one that reaches the driven sprocket is guided into its tooth gap
    # over its last pitch. Time goes on in sub-steps of ``h`` seconds, the
    # sprockets interpolated between two video frames.
    dynamic = False
    damping = 0.4          # 1/s: speed lost per second (exp)
    gravity = 9.81         # m/s^2, along -z
    h = 1.0 / 1000
    iterations = 4

    def reset(self):
        self._dyn = None

    def advance(self, mech, x, dt):
        """Chain after ``dt`` seconds, the joints being at ``x`` now.
        ``dt`` <= 0 (or a first call) starts from the chain at rest."""
        theta = mech.continuous_value(self.j1, x)
        cen = self._centres(mech, x)
        d = getattr(self, "_dyn", None)
        if d is None or dt <= 0:
            g = self.geometry(theta, cen)
            self._dyn = dict(P=np.array(g["pins"]), V=np.zeros((self.N, 2)), theta=theta,
                             cen=cen, kin=self._kinematic(g), g=g)
            self.state = self._dyn_state(g, self._dyn["P"])
            return self._write(mech, x, list(self._dyn["P"]), g["theta2"])
        n = max(1, int(math.ceil(dt / self.h)))
        hs = dt / n
        th0, c0 = d["theta"], d["cen"]
        P, V, kin_prev = d["P"], d["V"], d["kin"]
        damp = math.exp(-self.damping * hs)
        for s in range(1, n + 1):
            f = s / n
            th = th0 + f * (theta - th0)
            cs = (c0[0] + f * (cen[0] - c0[0]), c0[1] + f * (cen[1] - c0[1]))
            g = self.geometry(th, cs)
            T = np.array(g["pins"])
            kin, w = self._kinematic(g), self._blend(g)
            X = P.copy()
            free = ~kin
            born = free & kin_prev                 # just left the sprocket
            V[born] = (T[born] - P[born]) / hs
            V[free, 1] -= self.gravity * hs
            X[free] = P[free] + V[free] * hs
            X[kin] = T[kin]
            touched = self._project(X, kin, w, T, g)
            # velocities: the free pins keep their own speed, less what
            # would stretch a link (projected like the positions) — taking
            # (X - P) / h instead would brake the swing at every sub-step
            Vk = (T - P) / hs
            Vn = np.where(free[:, None], V, Vk)
            self._project_velocity(X, Vn, kin, g)
            Vn[touched] = (X[touched] - P[touched]) / hs
            V = np.where(free[:, None], Vn * damp, Vk)
            P, kin_prev = X, kin
        self._dyn = dict(P=P, V=V, theta=theta, cen=cen, kin=kin_prev, g=g)
        self.state = self._dyn_state(g, P)
        return self._write(mech, x, list(P), g["theta2"])

    def _kinematic(self, g):
        m2, m3 = g["marks"][2], g["marks"][3]
        return np.array([not (m2 < u < m3) for u in g["u"]]) if not g["taut"] \
            else np.ones(self.N, bool)

    def _blend(self, g):
        """Weight pulling a free pin to its tooth gap over its last pitch
        before the driven sprocket (0 .. 1)."""
        m3 = g["marks"][3]
        return np.array([min(1.0, max(0.0, u - (m3 - 1.0))) ** 2 for u in g["u"]])

    def _project(self, X, kin, w, T, g):
        """One pitch between neighbours, solved for the whole slack strand at
        once (its constraints form a tridiagonal system: a few Newton steps
        make every link exact), then the pins kept off the sprockets and
        the arriving pin guided to its tooth gap."""
        N, p = self.N, self.p
        free = [i for i in np.argsort(g["u"]) if not kin[i]]
        if not free:
            return np.zeros(N, bool)
        run = [(free[0] - 1) % N] + free + [(free[-1] + 1) % N]   # anchors at the ends
        wv = np.array([0.0] + [1.0] * len(free) + [0.0])
        m = len(run) - 1                                          # constraints
        for _ in range(self.iterations):
            Xr = X[run]
            D = Xr[1:] - Xr[:-1]
            L = np.hypot(D[:, 0], D[:, 1])
            nrm = D / L[:, None]
            c = L - p
            if np.abs(c).max() < 1e-10:
                break
            A = np.zeros((m, m))
            A[np.arange(m), np.arange(m)] = wv[:-1] + wv[1:]
            off = -wv[1:-1] * np.einsum("ij,ij->i", nrm[:-1], nrm[1:])
            A[np.arange(m - 1), np.arange(1, m)] = off
            A[np.arange(1, m), np.arange(m - 1)] = off
            lam = np.linalg.solve(A, -c)
            dX = np.zeros_like(Xr)
            dX[:-1] -= wv[:-1, None] * nrm * lam[:, None]
            dX[1:] += wv[1:, None] * nrm * lam[:, None]
            X[run] = Xr + dX
        S = g["S"]
        lim = ((S[0], self.r1 * math.cos(math.pi / self.z1)),
               (S[1], self.r2 * math.cos(math.pi / self.z2)))
        touched = np.zeros(N, bool)
        for i in free:
            for cen, r in lim:                    # off the sprockets
                d = X[i] - cen
                L = math.hypot(d[0], d[1])
                if 1e-12 < L < r:
                    X[i] = cen + d * (r / L)
                    touched[i] = True
            if w[i] > 0:
                X[i] += w[i] * (T[i] - X[i])
                touched[i] = True
        return touched

    def _project_velocity(self, X, V, kin, g):
        """Remove from the free pins' velocities what would change a link
        length (same tridiagonal system as the positions)."""
        N = self.N
        free = [i for i in np.argsort(g["u"]) if not kin[i]]
        if not free:
            return
        run = [(free[0] - 1) % N] + free + [(free[-1] + 1) % N]
        wv = np.array([0.0] + [1.0] * len(free) + [0.0])
        m = len(run) - 1
        Xr, Vr = X[run], V[run]
        D = Xr[1:] - Xr[:-1]
        nrm = D / np.hypot(D[:, 0], D[:, 1])[:, None]
        cdot = np.einsum("ij,ij->i", nrm, Vr[1:] - Vr[:-1])
        A = np.zeros((m, m))
        A[np.arange(m), np.arange(m)] = wv[:-1] + wv[1:]
        off = -wv[1:-1] * np.einsum("ij,ij->i", nrm[:-1], nrm[1:])
        A[np.arange(m - 1), np.arange(1, m)] = off
        A[np.arange(1, m), np.arange(m - 1)] = off
        mu = np.linalg.solve(A, -cdot)
        dV = np.zeros_like(Vr)
        dV[:-1] -= wv[:-1, None] * nrm * mu[:, None]
        dV[1:] += wv[1:, None] * nrm * mu[:, None]
        V[run] = Vr + dV

    def _dyn_state(self, g, P):
        st = dict(g)
        st["pins"] = list(P)
        if not g["taut"]:
            B1, B2 = g["B1"], g["B2"]
            dd = (B2 - B1) / np.linalg.norm(B2 - B1)
            nrm = np.array([dd[1], -dd[0]])
            kin = self._kinematic(g)
            st["sag"] = max([float((P[i] - B1) @ nrm) for i in range(self.N) if not kin[i]]
                            or [0.0])
        return st

    def measures(self):
        g = self.state or self.ref
        if g["taut"]:
            return ["chaîne tendue (pas de mou)"]
        return [f"flèche {g['sag'] * 1000:.1f} mm ({100 * g['sag'] / g['span']:.1f} % de l'entraxe)",
                f"entraxe {g['span'] * 1000:.0f} mm"]
