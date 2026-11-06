"""Beads: small spheres poured among the parts (Motion 0.10).

A *source* lets beads out at a point, at a rate, of a diameter (or a
range of diameters) and a density; they fall, roll, pile up, run down a
gutter or through a funnel, ride a conveyor, and end in a *bin* (a part
that counts the beads that come to it; they stay in it).

The beads follow the parts, the parts do not feel the beads (one way): the
mechanism is simulated first, then the beads on its motion — so they can
be poured again on the same motion without simulating it again.

The beads are moved the way position-based dynamics moves particles
(Müller et al. 2007; Macklin et al. 2014, "Unified particle physics"):
each step they are carried by their speed and gravity, then pushed apart
where they overlap each other or a part, a few rounds, and their speed is
what they moved. Unlike springs it holds at steps of a millisecond or two
(springs stiff enough for a pile need a hundred times smaller), and a pile
comes to rest: a granular flow, not a liquid.

A part's surface is read from its signed distance grid (``sdf``), in the
part's own frame at its pose of the moment (interpolated between the
mechanism's frames), with the speed of the surface there (a turning wheel
throws the beads, a conveyor's belt carries them).
"""

import math

import numpy as np

from . import sdf

G = np.array([0.0, 0.0, -9.81])


class Source:
    """Where and how beads come in (document data: ``doc.bead_source``)."""

    def __init__(self, d):
        self.name = d.get("name", "Source")
        self.point = np.asarray(d.get("point", (0, 0, 0)), float)
        di = np.asarray(d.get("direction", (0, 0, -1)), float)
        self.direction = di / max(np.linalg.norm(di), 1e-12)
        dia = d.get("diametre", 0.006)
        if isinstance(dia, (list, tuple)):
            self.dmin, self.dmax = float(dia[0]), float(dia[1])
        else:
            self.dmin = float(dia)
            self.dmax = max(float(d.get("diametre_max") or dia), self.dmin)
        self.rate = float(d.get("debit", 100.0))              # beads a second
        self.count = int(d.get("nombre", 500))                # at most
        self.start = float(d.get("debut", 0.0))
        self.stop = float(d.get("fin", 1e9))
        self.density = float(d.get("densite", 2500.0))        # kg/m³ (glass)
        self.spread = float(d.get("largeur", 0.0))            # m: the mouth's diameter
        self.speed = float(d.get("vitesse", 0.0))             # m/s, along the direction
        self.color = tuple(d.get("couleur", (0.85, 0.65, 0.15)))
        self.born = 0


class Beads:
    """Beads among the parts ``parts`` of ``mech``: [(the part's name in
    the document, its body in the mechanism (the frame for a fixed part),
    its triangles (n, 3, 3) in the drawn position, turned outward)]."""

    iterations = 4            # rounds of pushing apart a step
    relax = 1.5               # over-relaxation of the averaged pushes
    friction = 0.4            # beads on parts and on each other
    bead_friction = 0.3
    rest = 0.0                # bounce kept (0: none, as sand)
    max_beads = 20000

    def __init__(self, mech, parts, sources, sinks=(), conveyors=None, gravity=True,
                 seed=1, cell=None):
        self.m = mech
        self.sources = [s if isinstance(s, Source) else Source(s) for s in sources]
        self.sinks = set(sinks)
        self.conveyors = dict(conveyors or {})               # name: (direction, speed)
        self.g = G if gravity else np.zeros(3)
        self.rng = np.random.default_rng(seed)
        rmax = max([s.dmax for s in self.sources] + [0.002]) / 2
        self.rmax = rmax
        c = cell or max(rmax / 3, 2e-4)
        self.parts = []
        for name, b, T in parts:
            if T is None or not len(T):
                continue
            T = np.asarray(T, float).reshape(-1, 3, 3)
            grid = sdf.DistanceGrid(T, c, rmax + 3 * c)
            self.parts.append(dict(name=name, body=b, grid=grid,
                                   lo=T.reshape(-1, 3).min(0) - rmax - 2 * c,
                                   hi=T.reshape(-1, 3).max(0) + rmax + 2 * c,
                                   box=(T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0))))
        self.X = np.zeros((0, 3))
        self.V = np.zeros((0, 3))
        self.r = np.zeros(0)
        self.w = np.zeros(0)                                 # 1 / mass
        self.col = np.zeros(0, np.int32)                     # the source it came from
        self.uid = np.zeros(0, np.int64)
        self.next_uid = 0
        self.taken = {}                                      # bin name: beads come to it
        self._counted = {}
        self.lost = 0
        lo = [p["lo"] for p in self.parts] or [np.zeros(3) - 1]
        hi = [p["hi"] for p in self.parts] or [np.zeros(3) + 1]
        self.floor = min(float(np.min([l[2] for l in lo])), min(s.point[2] for s in self.sources)
                         if self.sources else 0.0) - 1.0

    # -- sources ------------------------------------------------------------
    def _pour(self, t, dt):
        for k, s in enumerate(self.sources):
            if t < s.start or t > s.stop or s.born >= s.count:
                continue
            want = min(s.count, int(math.floor((t - s.start) * s.rate)))
            n = min(want - s.born, self.max_beads - len(self.X))
            if n <= 0:
                continue
            r = 0.5 * self.rng.uniform(s.dmin, s.dmax, n)
            # spread over the mouth: a disc across the direction
            u = np.cross(s.direction, [1.0, 0, 0] if abs(s.direction[0]) < 0.9 else [0, 1.0, 0])
            u /= np.linalg.norm(u)
            v = np.cross(s.direction, u)
            a = self.rng.uniform(0, 2 * math.pi, n)
            rad = 0.5 * max(s.spread - 2 * s.dmax, 0.0) * np.sqrt(self.rng.uniform(0, 1, n))
            P = s.point + np.outer(rad * np.cos(a), u) + np.outer(rad * np.sin(a), v)
            # only where there is room: the others wait for the next step
            ok = self._free(P, r)
            if not ok.any():
                continue
            P, r = P[ok], r[ok]
            n = len(P)
            V = np.tile(s.direction * s.speed, (n, 1))
            m = s.density * 4.0 / 3.0 * math.pi * r ** 3
            self.X = np.vstack([self.X, P])
            self.V = np.vstack([self.V, V])
            self.r = np.concatenate([self.r, r])
            self.w = np.concatenate([self.w, 1.0 / m])
            self.col = np.concatenate([self.col, np.full(n, k, np.int32)])
            self.uid = np.concatenate([self.uid, self.next_uid + np.arange(n)])
            self.next_uid += n
            s.born += n

    def _free(self, P, r):
        """New beads that would not overlap a bead nor one another."""
        ok = np.ones(len(P), bool)
        if len(self.X):
            near = self.X[np.linalg.norm(self.X - P.mean(0), axis=1)
                          < np.ptp(P, axis=0).max() + 4 * self.rmax] if len(P) else self.X
            if len(near):
                d = np.linalg.norm(P[:, None] - near[None], axis=2)
                ok &= d.min(1) > r + self.rmax
        for k in range(len(P)):
            if ok[k] and k:
                prev = np.nonzero(ok[:k])[0]
                if len(prev) and np.min(np.linalg.norm(P[prev] - P[k], axis=1) - r[prev] - r[k]) < 0:
                    ok[k] = False
        return ok

    # -- the parts at a moment ------------------------------------------------
    @staticmethod
    def _pose_at(frames, t):
        """Index and fraction of the mechanism's frames around time t."""
        ts = frames["t"]
        k = int(np.clip(np.searchsorted(ts, t, "right") - 1, 0, len(ts) - 2))
        f = (t - ts[k]) / max(ts[k + 1] - ts[k], 1e-12)
        return k, float(np.clip(f, 0.0, 1.0))

    def _part_contacts(self, Xp, x0, x1, f, dt_frame):
        """For each bead near a part: (bead, depth, normal, surface speed,
        part name) from the parts' grids at the pose x0→x1 (fraction f)."""
        out = []
        for p in self.parts:
            b = p["body"]
            q0 = self.m.pose(x0, b)
            q1 = self.m.pose(x1, b)
            q = q0 + f * (q1 - q0)
            c = np.asarray(b.center, float)
            R = b.rot(q)
            L = (Xp - c - q[:3]) @ R + c                     # into the part's drawn frame
            near = np.nonzero(np.all((L >= p["lo"]) & (L <= p["hi"]), axis=1))[0]
            if p["name"] in self.sinks and len(near):
                # a bin counts the beads come into it (within its box), once each
                blo, bhi = p["box"]
                inn = near[np.all((L[near] >= blo) & (L[near] <= bhi), axis=1)]
                seen = self._counted.setdefault(p["name"], set())
                seen.update(self.uid[inn].tolist())
                self.taken[p["name"]] = len(seen)
            if not len(near):
                continue
            d, N = p["grid"].query(L[near])
            ov = self.r[near] - d
            hit = np.isfinite(ov) & (ov > -0.25 * self.r[near])
            if not hit.any():
                continue
            i = near[hit]
            n = N[hit] @ R.T
            # the surface's speed where the bead touches it
            S = L[near[hit]] - N[hit] * d[hit][:, None]
            if b.fixed:
                vs = np.zeros((len(i), 3))
            else:
                R0, R1 = b.rot(q0), b.rot(q1)
                w0 = c + q0[:3] + (S - c) @ R0.T
                w1 = c + q1[:3] + (S - c) @ R1.T
                vs = (w1 - w0) / max(dt_frame, 1e-12)
            if p["name"] in self.conveyors:
                di, sp = self.conveyors[p["name"]]
                vs = vs + np.asarray(di, float) * float(sp)
            out.append((i, ov[hit], n, vs, p["name"]))
        return out

    # -- bead on bead ----------------------------------------------------------
    def _pairs(self, X, reach):
        """Pairs (i < j) of beads closer than r_i + r_j + reach."""
        n = len(X)
        if n < 2:
            return np.zeros(0, int), np.zeros(0, int)
        cell = 2 * self.rmax + reach
        I = np.floor(X / cell).astype(np.int64)
        I -= I.min(0)
        span = I.max(0) + 3
        key = (I[:, 0] * span[1] + I[:, 1]) * span[2] + I[:, 2]
        o = np.argsort(key, kind="stable")
        ks = key[o]
        A, B = [], []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if (dx, dy, dz) < (0, 0, 0):
                        continue                              # half the neighbours
                    nk = key + (dx * span[1] + dy) * span[2] + dz
                    lo = np.searchsorted(ks, nk, "left")
                    hi = np.searchsorted(ks, nk, "right")
                    cnt = hi - lo
                    tot = int(cnt.sum())
                    if not tot:
                        continue
                    i = np.repeat(np.arange(n), cnt)
                    j = o[np.repeat(lo, cnt) + np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt)]
                    if (dx, dy, dz) == (0, 0, 0):
                        keep = i < j
                        i, j = i[keep], j[keep]
                    A.append(i)
                    B.append(j)
        if not A:
            return np.zeros(0, int), np.zeros(0, int)
        i, j = np.concatenate(A), np.concatenate(B)
        d = np.linalg.norm(X[i] - X[j], axis=1)
        keep = d < self.r[i] + self.r[j] + reach
        return i[keep], j[keep]

    # -- a step ------------------------------------------------------------------
    def step(self, t, dt, frames):
        self._pour(t, dt)
        n = len(self.X)
        if not n:
            return
        k, f = self._pose_at(frames, t + dt)
        x0, x1 = frames["x"][k], frames["x"][k + 1]
        dtf = frames["t"][k + 1] - frames["t"][k]
        X0 = self.X
        V = self.V + dt * self.g
        P = X0 + dt * V
        contacts = self._part_contacts(P, x0, x1, f, dtf)
        read = P.copy()                       # where the parts' depths were read
        vmax = float(np.max(np.linalg.norm(V, axis=1)))
        bi, bj = self._pairs(P, reach=2 * dt * vmax + 0.1 * self.rmax)
        wi, wj = self.w[bi], self.w[bj]
        sa = (wi / np.maximum(wi + wj, 1e-30))[:, None]
        sb = (wj / np.maximum(wi + wj, 1e-30))[:, None]
        for _ in range(self.iterations):
            dP = np.zeros_like(P)
            cnt = np.zeros(n)
            # the parts: pushed out along the surface's normal, and held by
            # friction against the surface's own motion (a belt carries)
            for i, ov, nrm, vs, name in contacts:
                depth = ov - np.einsum("ij,ij->i", P[i] - read[i], nrm)
                on = depth > 0
                if not on.any():
                    continue
                ii, nn = i[on], nrm[on]
                c = depth[on][:, None] * nn
                slip = (P[ii] + c - X0[ii]) - vs[on] * dt
                slip = slip - np.einsum("ij,ij->i", slip, nn)[:, None] * nn
                sl = np.linalg.norm(slip, axis=1)
                lim = self.friction * depth[on]
                fr = np.where(sl > lim, lim / np.maximum(sl, 1e-15), 1.0)[:, None] * slip
                np.add.at(dP, ii, c - fr)
                np.add.at(cnt, ii, 1.0)
            # bead on bead: apart along the line of centres, by their masses
            if len(bi):
                dv = P[bi] - P[bj]
                dist = np.linalg.norm(dv, axis=1)
                ov = self.r[bi] + self.r[bj] - dist
                on = ov > 0
                if on.any():
                    a, b2 = bi[on], bj[on]
                    nn = dv[on] / np.maximum(dist[on], 1e-12)[:, None]
                    c = ov[on][:, None] * nn
                    rel = (P[a] - X0[a]) - (P[b2] - X0[b2])
                    rel = rel - np.einsum("ij,ij->i", rel, nn)[:, None] * nn
                    rl = np.linalg.norm(rel, axis=1)
                    lim = self.bead_friction * ov[on]
                    fr = np.where(rl > lim, lim / np.maximum(rl, 1e-15), 1.0)[:, None] * rel
                    np.add.at(dP, a, sa[on] * (c - fr))
                    np.add.at(dP, b2, -sb[on] * (c - fr))
                    np.add.at(cnt, a, 1.0)
                    np.add.at(cnt, b2, 1.0)
            moved = cnt > 0
            if not moved.any():
                break
            P[moved] += self.relax * dP[moved] / np.maximum(cnt[moved], 1.0)[:, None]
        V = (P - X0) / dt
        # a bead's speed is what it moved: capped so one bad push cannot
        # throw it (a bead caught between two parts closing)
        sp = np.linalg.norm(V, axis=1)
        cap = 5.0 + 2.0 * vmax
        V = np.where((sp > cap)[:, None], V * (cap / np.maximum(sp, 1e-12))[:, None], V)
        self.V, self.X = V, P
        fallen = self.X[:, 2] < self.floor
        self.lost += int(fallen.sum())
        if fallen.any():
            self._drop(fallen)

    def _drop(self, gone):
        keep = ~gone
        self.X, self.V, self.r, self.w = self.X[keep], self.V[keep], self.r[keep], self.w[keep]
        self.col, self.uid = self.col[keep], self.uid[keep]

    # -- a whole run on the mechanism's frames -------------------------------------
    def run(self, rows, dt=1.0 / 600, progress=None):
        """Beads along the mechanism's frames ``rows`` (each with "t" and
        "x"): per frame, (positions float32, radii float32, source index)."""
        frames = dict(t=np.array([r["t"] for r in rows], float),
                      x=[np.asarray(r["x"], float) for r in rows])
        if len(frames["t"]) < 2:
            return [None] * len(rows)
        T0, T1 = frames["t"][0], frames["t"][-1]
        out = [None] * len(rows)
        t = T0
        kf = 0
        steps = int(math.ceil((T1 - T0) / dt))
        for s in range(steps + 1):
            while kf < len(rows) and frames["t"][kf] <= t + 1e-9:
                out[kf] = (self.X.astype(np.float32), self.r.astype(np.float32),
                           self.col.copy(), dict(self.taken), self.lost)
                kf += 1
            if s == steps:
                break
            self.step(t, dt, frames)
            t += dt
            if progress is not None and s % 10 == 0:
                self.live_t = t
                if progress(s, steps) is False:
                    return None
        while kf < len(rows):
            out[kf] = (self.X.astype(np.float32), self.r.astype(np.float32),
                       self.col.copy(), dict(self.taken), self.lost)
            kf += 1
        return out
