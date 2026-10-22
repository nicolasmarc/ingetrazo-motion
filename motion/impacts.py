"""Shocks and resting contacts between parts (dynamics).

The parts are seen through their boxes (the ones Motion fits to every
group: a domino, a plank, a block are exact; a curved part is rounded up
to its box). Each box carries points on its corners and along its edges;
a point of one part that enters a box of another part pushes the two
apart, at that point:

    normal:      f_n = k d - c v_n           (never pulling, f_n >= 0)
    tangential:  f_t = -min(mu f_n, c_t |v_t|) v_t / |v_t|   (Coulomb)

d is how deep the point is in the box (towards its nearest face), v the
velocity of the point on the first part relative to the second one. The
stiffness k is set from the masses (a contact frequency, 400 Hz by
default: penetrations stay well under a tenth of a millimetre for parts
of a few grams falling a few centimetres) and the damping c from the
restitution of the two materials (e = 1 a perfect rebound, e = 0 none),
so the dynamics stays a smooth set of forces the step integrates.
Friction and restitution come from the parts' materials (the means of the
two), else 0.4 and 0.4.

This is a penalty method: quick and steady for blocks, plates, dominoes,
stops; it does not replace a finite-element impact study, and the parts do
not deform. Fixed parts (the frame, a table) take part as obstacles.
"""
import math
import numpy as np

_FRICTION, _RESTITUTION = 0.4, 0.4


def _box_samples(half, per_edge=3):
    """Corners and ``per_edge`` points inside every edge, in box axes."""
    s = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], float)
    pts = [s * half]
    for ax in range(3):
        o = [a for a in range(3) if a != ax]
        for i in (-1, 1):
            for j in (-1, 1):
                for t in np.linspace(-1, 1, per_edge + 2)[1:-1]:
                    p = np.zeros(3)
                    p[ax] = t * half[ax]
                    p[o[0]] = i * half[o[0]]
                    p[o[1]] = j * half[o[1]]
                    pts.append(p[None])
    return np.vstack(pts)


def _damping_ratio(e):
    e = min(max(float(e), 1e-3), 0.999)
    le = math.log(e)
    return -le / math.sqrt(math.pi ** 2 + le * le)


def _zeta(e):
    le = np.log(np.clip(e, 1e-3, 0.999))
    return -le / np.sqrt(math.pi ** 2 + le * le)


class Impacts:
    """Contact forces between the parts of ``dyn.m`` (a :class:`Dynamics`).

    ``frequency``: contact frequency (Hz) that sets the stiffness;
    ``dt``: the time step it will be integrated with (caps the friction
    damping so that the step stays stable); ``per_edge``: points along each
    edge of a box."""

    def __init__(self, dyn, frequency=400.0, dt=1e-4, per_edge=3, skip=()):
        self.dyn = dyn
        m = dyn.m
        self.freq = float(frequency)
        self.dt = float(dt)
        moving = {id(b): k for k, b in enumerate(dyn.bodies)}
        fixed = [b for b in [m.ground] + list(getattr(m, "bodies", [])) if b.fixed]
        joined = set()
        for j in m.joints:            # parts held by a joint do not knock each other
            for a in (getattr(j, "a", None),):
                b = getattr(j, "b", None)
                if a is not None and b is not None:
                    joined.add(frozenset((id(a), id(b))))
        self.joined = joined | {frozenset(map(id, p)) for p in skip}
        # one row per box
        self.box_body, self.box_k, self.box_c, self.box_R, self.box_h = [], [], [], [], []
        self.box_pts = []
        for body in fixed + list(dyn.bodies):
            for bx in body.boxes:
                self.box_body.append(body)
                self.box_k.append(moving.get(id(body), -1))
                self.box_c.append(bx.center)
                self.box_R.append(bx.R)
                self.box_h.append(bx.half)
                self.box_pts.append(bx.center + _box_samples(bx.half, per_edge) @ bx.R.T)
        self.nb = len(self.box_body)
        self.box_k = np.array(self.box_k, int)
        self.box_c = np.array(self.box_c, float).reshape(-1, 3)
        self.box_R = np.array(self.box_R, float).reshape(-1, 3, 3)
        self.box_h = np.array(self.box_h, float).reshape(-1, 3)
        self.npts = len(self.box_pts[0]) if self.box_pts else 0
        self.box_pts = np.array(self.box_pts, float).reshape(self.nb, self.npts, 3)
        ids = [id(b) for b in self.box_body]
        same = np.array([[ids[i] == ids[k] for k in range(self.nb)] for i in range(self.nb)], bool)
        both_fixed = (self.box_k[:, None] < 0) & (self.box_k[None, :] < 0)
        held = np.array([[frozenset((ids[i], ids[k])) in self.joined for k in range(self.nb)]
                         for i in range(self.nb)], bool)
        self.allowed = ~(same | both_fixed | held)
        # materials
        fr = [getattr(b, "friction", None) for b in self.box_body]
        rs = [getattr(b, "restitution", None) for b in self.box_body]
        self.box_mu = np.array([_FRICTION if v is None else v for v in fr], float)
        self.box_e = np.array([_RESTITUTION if v is None else v for v in rs], float)
        self.box_m = np.array([dyn.mass[id(b)] if k >= 0 else np.inf
                               for b, k in zip(self.box_body, self.box_k)], float)
        self.count = 0              # contact points at the last call
        self.deepest = 0.0          # deepest penetration seen (m)

    # -- the parts' boxes at pose x ------------------------------------------
    def _world(self, x, Rb):
        C = self.box_c.copy()
        R = self.box_R.copy()
        P = self.box_pts.copy()
        mv = self.box_k >= 0
        if mv.any():
            k = self.box_k[mv]
            bodies = self.dyn.bodies
            idx = np.array([b.index for b in bodies], int)[k]
            q = np.asarray(x, float).reshape(-1, 6)[idx]
            c0 = np.array([b.center for b in bodies], float).reshape(-1, 3)[k]
            t = c0 + q[:, :3]
            Rk = Rb[k]
            C[mv] = t + np.einsum("nij,nj->ni", Rk, self.box_c[mv] - c0)
            R[mv] = Rk @ self.box_R[mv]
            P[mv] = t[:, None, :] + np.einsum("nij,nsj->nsi", Rk, self.box_pts[mv] - c0[:, None, :])
        return C, R, P

    def forces(self, x, u, F, Rb=None, cogs=None):
        """Add the contact forces and torques (about each part's centre of
        gravity) to ``F``."""
        self.count = 0
        if self.nb < 2:
            return
        if Rb is None:
            Rb, cogs = self.dyn.poses(x)
        C, R, P = self._world(x, Rb)
        ext = np.abs(R) @ self.box_h[:, :, None]          # AABB half sizes
        ext = ext[:, :, 0]
        lo, hi = C - ext, C + ext
        ov = np.all((lo[:, None, :] <= hi[None, :, :]) & (lo[None, :, :] <= hi[:, None, :]), axis=2)
        ov &= self.allowed
        ia, ib = np.nonzero(ov)          # points of box ia inside box ib
        if not len(ia):
            return
        pts = P[ia]                                   # (n, s, 3)
        loc = np.einsum("nij,nsi->nsj", R[ib], pts - C[ib][:, None, :])
        h = self.box_h[ib][:, None, :]
        depth = h - np.abs(loc)
        inside = np.all(depth > 0, axis=2)
        if not inside.any():
            return
        pn, ps = np.nonzero(inside)
        d3 = depth[pn, ps]
        ax = np.argmin(d3, axis=1)
        d = d3[np.arange(len(ax)), ax]
        a_box, b_box = ia[pn], ib[pn]
        sign = np.sign(loc[pn, ps, ax])
        sign[sign == 0] = 1.0
        n = R[b_box, :, ax] * sign[:, None]           # out of box b, towards a
        p = pts[pn, ps]
        # velocities of the point on each part (fixed parts: at rest)
        dyn = self.dyn
        ka, kb = self.box_k[a_box], self.box_k[b_box]
        nbod = len(dyn.bodies)
        U = u.reshape(nbod, 6) if nbod else np.zeros((0, 6))

        def vel(k):
            ok = k >= 0
            kk = np.where(ok, k, 0)
            r = np.where(ok[:, None], p - cogs[kk], 0.0) if nbod else np.zeros_like(p)
            v = (U[kk, :3] + np.cross(U[kk, 3:], r)) if nbod else np.zeros_like(p)
            return np.where(ok[:, None], v, 0.0), r

        va, ra = vel(ka)
        vb, rb = vel(kb)
        vrel = va - vb
        vn = np.einsum("ij,ij->i", vrel, n)
        vt = vrel - vn[:, None] * n
        # stiffness and damping, shared among the points of one box pair
        pair = a_box * self.nb + b_box
        _, inv, cnt = np.unique(pair, return_inverse=True, return_counts=True)
        share = cnt[inv].astype(float)
        ma, mb = self.box_m[a_box], self.box_m[b_box]
        fa, fb = np.isinf(ma), np.isinf(mb)
        ma_, mb_ = np.where(fa, 1.0, ma), np.where(fb, 1.0, mb)
        meff = np.where(fa, mb_, np.where(fb, ma_, ma_ * mb_ / (ma_ + mb_)))
        w = 2 * math.pi * self.freq
        k = meff * w * w / share
        e = 0.5 * (self.box_e[a_box] + self.box_e[b_box])
        zeta = _zeta(e)
        c = 2 * zeta * np.sqrt(k * meff / share)
        fn = np.maximum(k * d - c * vn, 0.0)
        mu = 0.5 * (self.box_mu[a_box] + self.box_mu[b_box])
        ct = 0.5 * meff / share / self.dt               # the most a step can take
        vts = np.linalg.norm(vt, axis=1)
        ft_mag = np.minimum(mu * fn, ct * vts)
        tdir = np.where(vts[:, None] > 1e-12, vt / np.maximum(vts, 1e-12)[:, None], 0.0)
        f = fn[:, None] * n - ft_mag[:, None] * tdir    # on part a
        G = np.zeros((max(len(dyn.bodies), 1), 6))
        oa, ob = ka >= 0, kb >= 0
        np.add.at(G, ka[oa], np.hstack([f[oa], np.cross(ra[oa], f[oa])]))
        np.add.at(G, kb[ob], -np.hstack([f[ob], np.cross(rb[ob], f[ob])]))
        F[:6 * len(dyn.bodies)] += G[:len(dyn.bodies)].reshape(-1)
        self.count = len(f)
        self.deepest = max(self.deepest, float(d.max()))
