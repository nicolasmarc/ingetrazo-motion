"""Collisions on the parts' real surfaces (not their boxes).

Each part's faces are sampled into points about ``h`` apart (the
vertices, plus points spread over every triangle by its area). At a pose,
the points of one part are carried into the drawn position of the other
and looked up in that part's k-d tree: two surfaces that touch or cross
come within ``h / 2`` of each other; the smallest distance found is the
clearance (to about ``h / 2``).

What touches in the drawn position is the joint, not a fault:

- parts linked by a gear coupling are not checked against each other
  (their teeth mesh);
- for every other pair, the points of each part that already lie against
  the other in the drawn position (an axle in its bore, a ball in its
  socket, a part resting on another) are left out of that pair's check;
  the rest of both parts is checked.
"""
import math
import numpy as np

class GridTree:
    """Nearest-point queries on a fixed point set, numpy only (the same
    answers as scipy's cKDTree for what Motion asks): IngeTrazo's Windows
    build has no scipy, and without these queries the collisions fell back
    to the parts' bounding boxes (a shell's box swallowing the legs).

    ``query(X, k=1, distance_upper_bound=r)``: (distance, index), inf and
    ``len(P)`` where nothing lies within r; ``query(X, k=k)``: the k nearest
    (distances and indices, (m, k))."""

    def __init__(self, P):
        self.P = np.asarray(P, float).reshape(-1, 3)
        self.n = len(self.P)
        self._grids = {}
        if self.n:
            lo, hi = self.P.min(0), self.P.max(0)
            self.lo, self.hi = lo, hi
            span = float(np.max(hi - lo)) or 1.0
            self._knn_cell = span / max(1.0, (self.n / 8.0) ** (1 / 3))

    def _grid(self, cell):
        g = self._grids.get(cell)
        if g is None:
            C = np.floor((self.P - self.lo) / cell).astype(np.int64)
            key = self._key(C)
            order = np.argsort(key, kind="stable")
            g = self._grids[cell] = (key[order], order)
            if len(self._grids) > 8:
                self._grids.pop(next(iter(self._grids)))
        return g

    @staticmethod
    def _key(C):
        C = C + (1 << 20)                          # room for negative cells
        return (C[..., 0] << 42) | (C[..., 1] << 21) | C[..., 2]

    LIMIT = 3_000_000          # candidate pairs held at once

    def _candidates(self, X, cell, R):
        """Chunks of (first query, query index, point index) for every
        point in the cube of (2R + 1)^3 cells around each query; queries are
        split so that no chunk holds more than LIMIT pairs."""
        skey, order = self._grid(cell)
        C = np.floor((X - self.lo) / cell).astype(np.int64)
        rng = np.arange(-R, R + 1)
        off = np.stack(np.meshgrid(rng, rng, rng, indexing="ij"), -1).reshape(-1, 3)
        K = self._key(C[:, None, :] + off[None, :, :])              # (m, o)
        A = np.searchsorted(skey, K, "left")
        B = np.searchsorted(skey, K, "right")
        per_q = (B - A).sum(axis=1)
        s0 = 0
        while s0 < len(X):
            csum = np.cumsum(per_q[s0:])
            s1 = s0 + max(1, int(np.searchsorted(csum, self.LIMIT, "right")))
            a, cnt = A[s0:s1].ravel(), (B[s0:s1] - A[s0:s1]).ravel()
            tot = int(cnt.sum())
            qi = np.repeat(np.repeat(np.arange(s1 - s0), len(off)), cnt)
            start = np.repeat(a, cnt)
            local = np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt)
            yield s0, s1, qi, order[start + local]
            s0 = s1

    def query(self, X, k=1, distance_upper_bound=np.inf):
        X = np.asarray(X, float)
        single = X.ndim == 1
        X = X.reshape(-1, 3)
        m, k = len(X), int(k)
        rmax = float(distance_upper_bound)
        D = np.full((m, k), np.inf)
        I = np.full((m, k), self.n, int)
        if self.n and m:
            span = float(np.max(np.ptp(self.P, axis=0))) or 1.0
            # start about where the k-th neighbour of a surface point is,
            # then double for the queries still short of k within r: the
            # cube of 3 x 3 x 3 cells of side r holds the whole ball of r
            r = span * math.sqrt(min(k, self.n) / self.n)
            r = min(r, rmax) if np.isfinite(rmax) else r
            todo = np.arange(m)
            while len(todo):
                Dt = np.full((len(todo), k), np.inf)
                It = np.full((len(todo), k), self.n, int)
                for s0, s1, qi, pi in self._candidates(X[todo], r, 1):
                    if not len(qi):
                        continue
                    diff = X[todo[s0:s1]][qi] - self.P[pi]
                    dd = np.sqrt(np.einsum("ij,ij->i", diff, diff))
                    if k == 1:                           # the nearest: no sorting
                        st = np.flatnonzero(np.r_[True, qi[1:] != qi[:-1]])
                        mn = np.minimum.reduceat(dd, st)
                        grp = np.repeat(np.arange(len(st)), np.diff(np.r_[st, len(qi)]))
                        hit = np.flatnonzero(dd == mn[grp])
                        firsts = hit[np.r_[True, grp[hit][1:] != grp[hit][:-1]]]
                        Dt[s0 + qi[firsts], 0] = dd[firsts]
                        It[s0 + qi[firsts], 0] = pi[firsts]
                        continue
                    o = np.lexsort((dd, qi))
                    qi, pi, dd = qi[o], pi[o], dd[o]
                    starts = np.searchsorted(qi, np.arange(s1 - s0), "left")
                    rank = np.arange(len(qi)) - starts[qi]
                    sel = rank < k
                    Dt[s0 + qi[sel], rank[sel]] = dd[sel]
                    It[s0 + qi[sel], rank[sel]] = pi[sel]
                # past the box of the points by a whole span, all are seen
                gap = np.linalg.norm(np.maximum(np.maximum(self.lo - X[todo], X[todo] - self.hi), 0.0),
                                     axis=1)
                last = (r >= rmax) | (r > 2 * span + 2 * gap)
                fin = (Dt[:, min(k, self.n) - 1] <= r) | last
                if np.isfinite(rmax):
                    far = Dt >= rmax
                    Dt[far], It[far] = np.inf, self.n
                D[todo[fin]], I[todo[fin]] = Dt[fin], It[fin]
                todo = todo[~fin]
                r = min(2 * r, rmax) if np.isfinite(rmax) else 2 * r
        if k == 1:
            D, I = D[:, 0], I[:, 0]
        return (D[0], I[0]) if single else (D, I)


try:
    from scipy.spatial import cKDTree as _Tree
except Exception:                                  # IngeTrazo's Windows build
    _Tree = GridTree


class TriGrid:
    """The triangles near a point, by a grid: each triangle is listed in
    every cell that comes within ``reach`` of it, so the triangles of a
    point's own cell are all those closer than ``reach`` (and some more).
    Big faces (a frame's flat sides) are found whatever their centre."""

    def __init__(self, T, reach):
        T = np.asarray(T, float).reshape(-1, 3, 3)
        self.T, self.reach = T, float(reach)
        c = self.cell = float(reach)
        self.lo = T.reshape(-1, 3).min(0) - 2 * c
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        n /= np.maximum(np.linalg.norm(n, axis=1), 1e-30)[:, None]
        lo = np.floor((T.min(1) - reach - self.lo) / c).astype(np.int64)
        hi = np.floor((T.max(1) + reach - self.lo) / c).astype(np.int64)
        keys, tris = [], []
        span = hi - lo + 1
        cnt = np.prod(span, axis=1)
        csum = np.cumsum(cnt)
        k = 0
        while k < len(T):                         # by chunks of 2 M cells at most
            e = max(k + 1, int(np.searchsorted(csum, (csum[k - 1] if k else 0) + 2_000_000, "right")))
            sl = slice(k, e)
            k = e
            tot = int(cnt[sl].sum())
            ti = np.repeat(np.arange(sl.start, sl.stop), cnt[sl])
            loc = np.arange(tot) - np.repeat(np.cumsum(cnt[sl]) - cnt[sl], cnt[sl])
            sp = span[ti]
            ix = loc % sp[:, 0]
            iy = (loc // sp[:, 0]) % sp[:, 1]
            iz = loc // (sp[:, 0] * sp[:, 1])
            C = lo[ti] + np.stack([ix, iy, iz], 1)
            ctr = self.lo + (C + 0.5) * c
            # only the cells the plane of the triangle passes near
            off = np.abs(np.einsum("ij,ij->i", ctr - T[ti, 0], n[ti]))
            ok = off <= reach + 0.87 * c
            keys.append(GridTree._key(C[ok]))
            tris.append(ti[ok])
        key = np.concatenate(keys) if keys else np.zeros(0, np.int64)
        tri = np.concatenate(tris) if tris else np.zeros(0, int)
        o = np.argsort(key, kind="stable")
        self.key, self.tri = key[o], tri[o]

    def near_mask(self, X):
        """Points X with some triangle within reach (or a little more)."""
        X = np.asarray(X, float).reshape(-1, 3)
        K = GridTree._key(np.floor((X - self.lo) / self.cell).astype(np.int64))
        return np.searchsorted(self.key, K, "right") > np.searchsorted(self.key, K, "left")

    def nearest(self, X, k=3):
        """For points X (m, 3): (keep mask, triangle indices (kept, k)) of
        the k nearest triangles, for the points with any within reach."""
        X = np.asarray(X, float).reshape(-1, 3)
        K = GridTree._key(np.floor((X - self.lo) / self.cell).astype(np.int64))
        A = np.searchsorted(self.key, K, "left")
        B = np.searchsorted(self.key, K, "right")
        cnt = B - A
        keep = cnt > 0
        if not keep.any():
            return keep, np.zeros((0, k), int)
        out = np.zeros((int(keep.sum()), k), int)
        A, cnt, Xk = A[keep], cnt[keep], X[keep]
        csum = np.cumsum(cnt)
        s0 = 0
        while s0 < len(Xk):                       # by chunks of 1 M pairs at most
            s1 = max(s0 + 1, int(np.searchsorted(csum, (csum[s0 - 1] if s0 else 0) + 1_000_000, "right")))
            out[s0:s1] = self._nearest(A[s0:s1], cnt[s0:s1], Xk[s0:s1], k)
            s0 = s1
        return keep, out

    def _nearest(self, A, cnt, Xk, k):
        tot = int(cnt.sum())
        qi = np.repeat(np.arange(len(Xk)), cnt)
        local = np.arange(tot) - np.repeat(np.cumsum(cnt) - cnt, cnt)
        ti = self.tri[np.repeat(A, cnt) + local]
        T = self.T[ti]
        cp = closest_on_triangles(Xk[qi], T[:, 0], T[:, 1], T[:, 2])
        dd = np.linalg.norm(Xk[qi] - cp, axis=1)
        o = np.lexsort((dd, qi))
        qi, ti = qi[o], ti[o]
        starts = np.searchsorted(qi, np.arange(len(Xk)), "left")
        rank = np.arange(len(qi)) - starts[qi]
        out = np.repeat(ti[starts][:, None], k, axis=1)   # short lists: the nearest again
        sel = rank < k
        out[qi[sel], rank[sel]] = ti[sel]
        return out


def available():
    return _Tree is not None


def surface_points(T, h):
    """Points on the triangles ``T`` (n, 3, 3), about ``h`` apart."""
    T = np.asarray(T, float).reshape(-1, 3, 3)
    if not len(T):
        return np.zeros((0, 3))
    a, b, c = T[:, 0], T[:, 1], T[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n = np.floor(area / (h * h)).astype(int)              # about one per h^2
    out = [T.reshape(-1, 3)]
    if n.sum():
        rng = np.random.default_rng(0)                    # the same every time
        idx = np.repeat(np.arange(len(T)), n)
        u, v = rng.random(len(idx)), rng.random(len(idx))
        flip = u + v > 1
        u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
        out.append(a[idx] + u[:, None] * (b - a)[idx] + v[:, None] * (c - a)[idx])
    P = np.concatenate(out)
    # one point per cell of 0.7 h: the vertices of fine meshes add nothing
    key = np.floor(P / (0.7 * h)).astype(np.int64)
    _, keep = np.unique(key, axis=0, return_index=True)
    return P[np.sort(keep)]


def closest_on_triangles(P, A, B, C):
    """Closest points to P on triangles (A, B, C), all (n, 3), row by row
    (Ericson, Real-Time Collision Detection 5.1.5, vectorised)."""
    ab, ac, ap = B - A, C - A, P - A
    d1, d2 = _dot(ab, ap), _dot(ac, ap)
    bp = P - B
    d3, d4 = _dot(ab, bp), _dot(ac, bp)
    cp = P - C
    d5, d6 = _dot(ab, cp), _dot(ac, cp)
    va = d3 * d6 - d5 * d4
    vb = d5 * d2 - d1 * d6
    vc = d1 * d4 - d3 * d2
    out = np.empty_like(P)
    done = np.zeros(len(P), bool)

    def put(mask, val):
        m = mask & ~done
        out[m] = val[m]
        done[:] |= m

    put((d1 <= 0) & (d2 <= 0), A)                                     # vertex A
    put((d3 >= 0) & (d4 <= d3), B)                                    # vertex B
    with np.errstate(divide="ignore", invalid="ignore"):
        v = d1 / (d1 - d3)
        put((vc <= 0) & (d1 >= 0) & (d3 <= 0), A + v[:, None] * ab)   # edge AB
        put((d6 >= 0) & (d5 <= d6), C)                                # vertex C
        w = d2 / (d2 - d6)
        put((vb <= 0) & (d2 >= 0) & (d6 <= 0), A + w[:, None] * ac)   # edge AC
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        put((va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0),
            B + w[:, None] * (C - B))                                 # edge BC
        den = 1.0 / (va + vb + vc)
        v, w = vb * den, vc * den
        put(np.ones(len(P), bool), A + v[:, None] * ab + w[:, None] * ac)  # inside
    return out


def _dot(a, b):
    return np.einsum("ij,ij->i", a, b)


def spacing(boxes_extent):
    """A sampling step for a model of the given size (m): 1/300 of it,
    between 0.2 mm and 5 mm."""
    return float(np.clip(boxes_extent / 300.0, 0.0002, 0.005))


POINT_BUDGET = 150_000       # sample points for a whole mechanism


def budget_spacing(h, area, budget=POINT_BUDGET):
    """``h``, widened when ``area`` (m², every part) would need more than
    ``budget`` points at that step."""
    if area > 0 and area / (h * h) > budget:
        return float(np.sqrt(area / budget))
    return float(h)


class Surfaces:
    """The sampled surfaces of a mechanism's parts, and the checks."""

    walk_reach = 1.5      # sampling steps a point may have moved and still
                          # find its nearest triangle by walking round the last
    grid_cell = 0.5       # sampling steps: the distance grids' cells (0.10)
    grid_band = 2.5       # sampling steps the grids are exact each side of a surface
    use_grids = True

    def __init__(self, mech, clouds, h):
        """``clouds``: {body name: [(group name, points (n, 3), triangles
        (m, 3, 3)) in the drawn position]}; the triangles (turned outward)
        give the exact, signed distance where the points come close."""
        self.h = float(h)
        self.tol = PENETRATION
        self.mech = mech
        self.data = {}
        for name, items in clouds.items():
            items = [(g, np.asarray(P, float), np.asarray(T, float).reshape(-1, 3, 3))
                     for g, P, T in items if len(P)]
            if not items:
                continue
            P = np.concatenate([P for _, P, _ in items])
            lab = np.concatenate([np.full(len(P), k) for k, (_, P, _) in enumerate(items)])
            T = np.concatenate([T for _, _, T in items])
            n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
            ok = np.linalg.norm(n, axis=1) > 1e-20
            T, n = T[ok], n[ok]
            n /= np.linalg.norm(n, axis=1)[:, None]
            self.data[name] = dict(P=P, lab=lab, groups=[g for g, _, _ in items],
                                   tree=_Tree(P), lo=P.min(0), hi=P.max(0),
                                   T=T, N=n, ttree=_Tree(T.mean(axis=1)),
                                   tsize=float(np.sqrt(np.max(np.sum(
                                       (T - T.mean(axis=1)[:, None]) ** 2, axis=2)))) if len(T) else 0.0)
        self._skip = {}
        self._last = {}
        self._gears = set()
        for j in mech.joints:
            if j.kind == "coupling":
                for ja, jb in ((j.ja, j.jb),):
                    self._gears.add(frozenset((ja.b.name, jb.b.name)))
                    self._gears.add(frozenset((ja.a.name, jb.b.name)))
                    self._gears.add(frozenset((ja.b.name, jb.a.name)))

    def has(self, body):
        return body.name in self.data

    def _resting(self, a, b, factor=1.5):
        """Points of a and of b that lie against the other in the drawn
        position, within ``factor`` sampling steps (masks)."""
        key = (a.name, b.name, factor)
        if key not in self._skip:
            A, B = self.data[a.name], self.data[b.name]
            r = factor * self.h
            if _box_gap(A["lo"], A["hi"], B["lo"], B["hi"]) > r:
                ma, mb = np.zeros(len(A["P"]), bool), np.zeros(len(B["P"]), bool)
            else:
                ma = B["tree"].query(A["P"], k=1, distance_upper_bound=r)[0] < r
                mb = A["tree"].query(B["P"], k=1, distance_upper_bound=r)[0] < r
            self._skip[key] = (ma, mb)
        return self._skip[key]

    def pair(self, a, b, qa, qb, reach, resting=True):
        """Smallest distance (< reach) between the free points of a and b
        at poses qa, qb: (distance, point a, point b, group a, group b) or
        None. ``resting=False``: the points lying against each other as
        drawn count too (a contact that pushes)."""
        if frozenset((a.name, b.name)) in self._gears:
            return None
        A, B = self.data[a.name], self.data[b.name]
        if resting:
            ma, mb = self._resting(a, b, 1.5 if resting is True else float(resting))
        else:
            ma, mb = np.zeros(len(A["P"]), bool), np.zeros(len(B["P"]), bool)
        # b's free points, carried into a's drawn position
        Pb = B["P"][~mb]
        if not len(Pb):
            return None
        W = _apply(b, qb, Pb)                       # world now
        Lb = _unapply(a, qa, W)                     # in a's drawn frame
        lo, hi = A["lo"] - reach, A["hi"] + reach
        inside = np.all((Lb >= lo) & (Lb <= hi), axis=1)
        if not inside.any():
            return None
        idx_b = np.nonzero(~mb)[0][inside]
        d, ia = A["tree"].query(Lb[inside], k=1, distance_upper_bound=reach)
        ok = np.isfinite(d) & ~ma[np.minimum(ia, len(ma) - 1)]
        if not ok.any():
            return None
        d = np.where(ok, d, np.inf)
        # the exact, signed distance where it matters: every point closer
        # than h, and the nearest few (for the clearance)
        fin = np.nonzero(np.isfinite(d))[0]
        few = fin[np.argsort(d[fin])[:64]]
        near = np.union1d(np.nonzero(d < self.h)[0], few)
        if len(near):
            d[near] = self._signed(A, Lb[inside][near])
        k = int(np.argmin(d))
        pa = _apply(a, qa, A["P"][ia[k]][None])[0]
        pb = W[np.nonzero(inside)[0][k]]
        return (float(d[k]), pa, pb, A["groups"][A["lab"][ia[k]]],
                B["groups"][B["lab"][idx_b[k]]])


    def drawn_overlap(self, a, b, factor=0.75):
        """Mask of b's points that touch a (within ``factor`` steps) or lie
        inside it as drawn: the assembly as modelled (a shaft through its
        bore, a roller drawn a hair into its slot), not a contact."""
        key = ("drawn", a.name, b.name, factor)
        if key not in self._skip:
            A, B = self.data[a.name], self.data[b.name]
            mb = self._resting(a, b, factor)[1].copy()
            r = 1.5 * self.h
            box = np.all((B["P"] >= A["lo"] - r) & (B["P"] <= A["hi"] + r), axis=1)
            if box.any():
                ix = np.nonzero(box & ~mb)[0]
                if len(ix):
                    mb[ix[self._inside_many(A, B["P"][ix])]] = True
                # and what the contact test itself sees inside, as drawn
                ix = np.nonzero(box & ~mb)[0]
                if len(ix) and len(A["T"]):
                    Q = B["P"][ix]
                    d, _ = A["tree"].query(Q, k=1, distance_upper_bound=r)
                    near = np.isfinite(d)
                    if near.any():
                        ix, Q = ix[near], Q[near]
                        k = min(8, len(A["T"]))
                        _, it = A["ttree"].query(Q, k=k)
                        it = np.atleast_2d(it).reshape(len(Q), -1)
                        T = A["T"][it.ravel()]
                        Qr = np.repeat(Q, it.shape[1], axis=0)
                        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
                        dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1)
                        j = np.argmin(dist, axis=1)
                        rows = np.arange(len(Q))
                        cpb = cp.reshape(len(Q), -1, 3)[rows, j]
                        N = A["N"][it[rows, j]]
                        mb[ix[np.einsum("ij,ij->i", Q - cpb, N) < 0.0]] = True
            self._skip[key] = mb
        return self._skip[key]

    def _signed_in(self, A, Q):
        """Signed depths of points Q (in a's drawn frame) in a's solid:
        > 0 inside, < 0 out, nan further than a few steps."""
        out = np.full(len(Q), np.nan)
        if not len(Q) or not len(A["T"]):
            return out
        g = A.setdefault("tgrid", {}).get(4.0 * self.h)
        if g is None:
            g = A["tgrid"][4.0 * self.h] = TriGrid(A["T"], 4.0 * self.h)
        keep, it = g.nearest(Q, 3)
        ix = np.nonzero(keep)[0]
        if len(ix):
            Qk = Q[ix]
            T = A["T"][it.ravel()]
            Qr = np.repeat(Qk, it.shape[1], axis=0)
            cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
            dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Qk), -1)
            j = np.argmin(dist, axis=1)
            rows = np.arange(len(Qk))
            cpb = cp.reshape(len(Qk), -1, 3)[rows, j]
            N = A["N"][it[rows, j]]
            dj = dist[rows, j]
            sd = np.where(np.einsum("ij,ij->i", Qk - cpb, N) < 0.0, dj, -dj)
            out[ix] = np.where(dj <= 4.0 * self.h, sd, np.nan)   # further: not known
        return out

    def drawn_depths(self, a, b):
        """Signed depths of b's points in a as drawn (> 0 inside, < 0
        out, nan: further than a few steps)."""
        key = ("depth", a.name, b.name)
        if key not in self._skip:
            self._skip[key] = self._signed_in(self.data[a.name], self.data[b.name]["P"])
        return self._skip[key]

    def depths_at(self, a, b, qa, qb, idx):
        """Signed depths of b's points ``idx`` in a, the parts at qa, qb."""
        A, B = self.data[a.name], self.data[b.name]
        return self._signed_in(A, _unapply(a, qa, _apply(b, qb, B["P"][idx])))

    def hard_hub(self, a, b, axes):
        """Round the pivots holding a and b (``axes``: [(point,
        direction)] as drawn), b's points nearer the axis than what touches
        or overlaps there as drawn: the shaft in its bore, the hub, whose
        rub is the joint itself."""
        key = ("hub", a.name, b.name, tuple((tuple(p), tuple(d)) for p, d in axes))
        if key not in self._skip:
            B = self.data[b.name]
            dep = np.nan_to_num(self.drawn_depths(a, b), nan=-1.0)
            mb = np.zeros(len(B["P"]), bool)
            ext = float(np.linalg.norm(B["hi"] - B["lo"]))
            for p, d in axes:
                p, d = np.asarray(p, float), np.asarray(d, float)
                d = d / max(np.linalg.norm(d), 1e-12)
                v = B["P"] - p
                rad = np.linalg.norm(v - np.outer(v @ d, d), axis=1)
                hub = (dep > -0.75 * self.h) & (rad < 0.2 * ext)
                if hub.any():
                    mb |= rad < float(rad[hub].max()) + 2.0 * self.h
            self._skip[key] = mb
        return self._skip[key]

    def drawn_inside(self, a, b, tol=0.25):
        """b's points drawn inside a by more than ``tol`` steps: an
        interference of the model, not a contact — until they come out."""
        return np.nan_to_num(self.drawn_depths(a, b), nan=-1.0) > tol * self.h

    def hard_overlap(self, a, b, axes, tol=0.25):
        """:meth:`hard_hub` and :meth:`drawn_inside` together."""
        return self.hard_hub(a, b, axes) | self.drawn_inside(a, b, tol)

    def _mask(self, a, b, resting):
        if isinstance(resting, tuple) and resting[0] == "hard":
            return self.hard_overlap(a, b, resting[1])
        if isinstance(resting, tuple):           # ("drawn", axes of the joints)
            return self.joint_overlap(a, b, resting[1])
        if resting == "drawn":
            return self.drawn_overlap(a, b)
        return self._resting(a, b, float(resting))[1]

    def joint_overlap(self, a, b, axes):
        """:meth:`drawn_overlap`, and round the axes of the pivots holding
        a and b together (``axes``: [(point, direction)] as drawn) every
        point of b nearer the axis than what overlaps there as drawn (the
        shaft and its bore, the hub), so that turning brings no new rub
        in; further out (the end of a slot) still counts."""
        key = ("joint", a.name, b.name, tuple((tuple(p), tuple(d)) for p, d in axes))
        if key not in self._skip:
            B = self.data[b.name]
            mb = self.drawn_overlap(a, b).copy()
            mb_in = mb & ~self._resting(a, b, 0.75)[1]       # inside, not only touching
            ext = float(np.linalg.norm(B["hi"] - B["lo"]))
            for p, d in axes:
                p, d = np.asarray(p, float), np.asarray(d, float)
                d = d / max(np.linalg.norm(d), 1e-12)
                v = B["P"] - p
                rad = np.linalg.norm(v - np.outer(v @ d, d), axis=1)
                # the hub: what overlaps close to the axis (a pin drawn into
                # the side of its slot further out is not the hub)
                hub = mb_in & (rad < 0.2 * ext)
                if not hub.any():
                    continue
                R = float(rad[hub].max()) + 2.0 * self.h
                mb |= rad < R
            self._skip[key] = mb
        return self._skip[key]

    def candidates(self, a, b, qa, qb, resting=None, margin=None, sparse=True):
        """The points of b that may touch a in the next few steps (within
        ``margin`` of its surface) and their nearest triangles of a:
        (point indices into b's points, triangle indices (n, k)), for
        :meth:`penetrations_of` to use step after step."""
        A, B = self.data[a.name], self.data[b.name]
        idx = self._sparse(B) if sparse else np.arange(len(B["P"]))
        if isinstance(resting, np.ndarray):
            idx = idx[~resting[idx]]
        elif resting:
            idx = idx[~self._mask(a, b, resting)[idx]]
        r = margin if margin is not None else 4.0 * self.h
        none = (np.zeros(0, int), np.zeros((0, 1), int))
        if not len(idx) or not len(A["T"]):
            return none
        L = _unapply(a, qa, _apply(b, qb, B["P"][idx]))
        box = np.all((L >= A["lo"] - r) & (L <= A["hi"] + r), axis=1)
        if not box.any():
            return none
        idx, L = idx[box], L[box]
        grids = A.setdefault("tgrid", {})
        reach = 4.0 * self.h        # what is further is found at the next search
        g = grids.get(reach)
        if g is None:
            g = grids[reach] = TriGrid(A["T"], reach)
        keep, it = g.nearest(L, 3)
        return idx[keep], it

    def near_points(self, a, b, qa, qb, mask, margin):
        """b's points (all of them, but those in ``mask``) within about
        ``margin`` of a's surface: the few that may touch it in the next
        steps, searched once while the parts move by less than that."""
        A, B = self.data[a.name], self.data[b.name]
        idx = np.nonzero(~mask)[0] if mask is not None else np.arange(len(B["P"]))
        if not len(idx) or not len(A["T"]):
            return np.zeros(0, int)
        L = _unapply(a, qa, _apply(b, qb, B["P"][idx]))
        box = np.all((L >= A["lo"] - margin) & (L <= A["hi"] + margin), axis=1)
        idx, L = idx[box], L[box]
        if not len(idx):
            return idx
        grids = A.setdefault("tgrid", {})
        g = grids.get(("coarse", margin))
        if g is None:
            g = grids[("coarse", margin)] = TriGrid(A["T"], margin)
        return idx[g.near_mask(L)]

    def nearest_now(self, a, b, qa, qb, idx, full=False, moved=0.0, reach=None):
        """For b's points ``idx`` (found near a a few steps ago): those
        within ``reach`` (or 4 steps) and their nearest triangles of a as
        they are now — as a block lands, its corners' nearest face changes
        from the side to the top of the plate. ``moved``: how far the parts
        may have gone since the last call; a point further than that from
        mattering is not looked at."""
        A, B = self.data[a.name], self.data[b.name]
        if not len(idx):
            return idx, np.zeros((0, 3), int)
        g = A.setdefault("tgrid", {}).get(4.0 * self.h)
        if g is None:
            g = A["tgrid"][4.0 * self.h] = TriGrid(A["T"], 4.0 * self.h)
        key = (a.name, b.name)
        st = self._last.get(key)
        if st is None:
            n = len(B["P"])
            # nearest triangle (-1: never seen, -2: far), distance then,
            # how far the parts have gone since
            st = self._last[key] = [np.full(n, -1, np.int64), np.zeros(n), np.zeros(n)]
        last, dist0, since = st
        since += moved
        need = 4.0 * self.h if reach is None else float(reach) + 0.5 * self.h
        best = last[idx].copy()
        if full:
            best[best == -2] = -1
        # what cannot have come within reach is left as it was
        look = (best == -1) | ((best >= 0) & (dist0[idx] - since[idx] < need))
        if full:
            look |= best >= 0
        sel = np.nonzero(look)[0]
        if not len(sel):
            return idx[:0], np.zeros((0, 3), int)
        ids = idx[sel]
        L = _unapply(a, qa, _apply(b, qb, B["P"][ids]))
        b_s = best[sel]
        # each point remembers its nearest triangle and looks round it (the
        # parts move by a fraction of a step); new points, or points left
        # alone for a while, ask the whole grid
        walk = (b_s >= 0) & (since[ids] < self.walk_reach * self.h)
        d_s = np.full(len(sel), np.inf)
        if walk.any():
            ring = self._ring(A)
            Lk = L[walk]
            b_k = b_s[walk]
            for _ in range(3):                    # walk at most three rings a call
                R = ring[b_k]                     # (n, K), -1 padding
                ok = R >= 0
                rows, cols = np.nonzero(ok)
                T = A["T"][R[rows, cols]]
                cp = closest_on_triangles(Lk[rows], T[:, 0], T[:, 1], T[:, 2])
                D = np.full(R.shape, np.inf)
                D[rows, cols] = np.linalg.norm(Lk[rows] - cp, axis=1)
                j = np.argmin(D, axis=1)
                nb = R[np.arange(len(R)), j]
                dk = D[np.arange(len(R)), j]
                changed = nb != b_k
                b_k = nb
                if not changed.any():
                    break
            b_s[walk], d_s[walk] = b_k, dk
        grid = ~walk
        if grid.any():
            keep, it = g.nearest(L[grid], 3)
            bn = np.full(int(grid.sum()), -2, np.int64)
            bn[keep] = it[:, 0]
            T = A["T"][np.maximum(bn, 0)]
            cp = closest_on_triangles(L[grid], T[:, 0], T[:, 1], T[:, 2])
            dn = np.where(bn >= 0, np.linalg.norm(L[grid] - cp, axis=1), 4.0 * self.h)
            b_s[grid], d_s[grid] = bn, dn
        b_s[d_s > 4.0 * self.h] = -2
        d_s = np.minimum(d_s, 4.0 * self.h)
        last[ids], dist0[ids], since[ids] = b_s, d_s, 0.0
        ok = (b_s >= 0) & (d_s < need)
        return ids[ok], np.repeat(b_s[ok][:, None], 3, axis=1)

    def _ring(self, A):
        """For each triangle, the triangles sharing a corner with it (itself
        included), padded with -1: where its nearest point can go next."""
        R = A.get("ring")
        if R is None:
            T = A["T"]
            V = np.round(T.reshape(-1, 3) / max(self.h * 1e-4, 1e-12)).astype(np.int64)
            _, vid = np.unique(V, axis=0, return_inverse=True)
            vid = vid.reshape(-1, 3)
            tri = np.repeat(np.arange(len(T)), 3)
            v = vid.ravel()
            order = np.argsort(v, kind="stable")
            v_s, t_s = v[order], tri[order]
            starts = np.searchsorted(v_s, np.arange(vid.max() + 1), "left")
            ends = np.searchsorted(v_s, np.arange(vid.max() + 1), "right")
            K = 24
            R = np.full((len(T), K), -1, np.int64)
            for i in range(len(T)):
                nb = np.unique(np.concatenate([t_s[starts[c]:ends[c]] for c in vid[i]]))
                if len(nb) > K:                   # a fan's centre: the nearest
                    c = T[nb].mean(axis=1)
                    nb = nb[np.argsort(np.linalg.norm(c - T[i].mean(axis=0), axis=1))[:K]]
                R[i, :len(nb)] = nb
            A["ring"] = R
        return R

    def _sparse(self, B, f=2.5):
        """Fewer points for the dynamics: the mesh vertices first, then one
        point in every cell of f sampling steps."""
        if "Pd" not in B:
            key = np.floor(B["P"] / (f * self.h)).astype(np.int64)
            _, first = np.unique(key, axis=0, return_index=True)
            B["Pd"] = np.sort(first)
        return B["Pd"]

    def penetrations_of(self, a, b, qa, qb, idx, it):
        """:meth:`penetrations` for the candidate points ``idx`` of b and
        their triangles ``it`` of a (found by :meth:`candidates`)."""
        empty = (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0))
        if not len(idx):
            return empty
        A, B = self.data[a.name], self.data[b.name]
        W = _apply(b, qb, B["P"][idx])
        Q = _unapply(a, qa, W)
        T = A["T"][it.ravel()]
        Qr = np.repeat(Q, it.shape[1], axis=0)
        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
        dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1)
        j = np.argmin(dist, axis=1)
        rows = np.arange(len(Q))
        cpb = cp.reshape(len(Q), -1, 3)[rows, j]
        N = A["N"][it[rows, j]]
        dj = dist[rows, j]
        inside = (np.einsum("ij,ij->i", Q - cpb, N) < 0.0) & (dj < 2.5 * self.h)
        if not inside.any():
            return empty
        return W[inside], N[inside] @ a.rot(qa).T, dj[inside]

    def distance_grid(self, body):
        """The part's signed distance grid (0.10), made on first use."""
        if not self.use_grids:
            return None
        A = self.data.get(body.name)
        if A is None or not len(A["T"]):
            return None
        if "grid" not in A:
            from . import sdf
            A["grid"] = sdf.DistanceGrid(A["T"], self.grid_cell * self.h, self.grid_band * self.h)
        return A["grid"]

    def save_grids(self):
        """Keep the distance grids' nodes for the next simulation."""
        for A in self.data.values():
            g = A.get("grid")
            if g is not None:
                g.save()

    def gaps_by_grid(self, a, b, qa, qb, idx, reach):
        """``gaps_of`` read from a's distance grid: no triangle to find,
        and a normal that turns smoothly round edges."""
        empty = (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0))
        if not len(idx):
            return empty
        G = self.distance_grid(a)
        B = self.data[b.name]
        W = _apply(b, qb, B["P"][idx])
        Q = _unapply(a, qa, W)
        dq, Nq = G.query(Q)
        # where it may touch, the exact distance to the nearest triangles
        # of the cell's corners (the triangles' own normal, as gaps_of)
        with np.errstate(invalid="ignore"):
            close = np.nonzero(np.abs(dq) < reach + G.cell)[0]
        if len(close):
            de, Ne, ok = G.exact(Q, close)
            dq[close[ok]] = de[ok]
            Nq[close[ok]] = Ne[ok]
        self.last_gaps = dq                         # every point read: its gap
        depth = -dq
        reach = min(reach, G.band - G.cell)
        with np.errstate(invalid="ignore"):
            keep = np.isfinite(depth) & (depth > -reach) & (depth < 2.5 * self.h)
        if not keep.any():
            return empty
        self.last_ids = np.asarray(idx)[keep]
        return W[keep], Nq[keep] @ a.rot(qa).T, depth[keep]

    def gaps_of(self, a, b, qa, qb, idx, it, reach):
        """Signed gaps of the candidate points of b to a's surface, those
        closer than ``reach``: (world points, world normals out of a,
        depths: > 0 inside, < 0 a gap still open)."""
        empty = (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0))
        if not len(idx):
            return empty
        A, B = self.data[a.name], self.data[b.name]
        W = _apply(b, qb, B["P"][idx])
        Q = _unapply(a, qa, W)
        T = A["T"][it.ravel()]
        Qr = np.repeat(Q, it.shape[1], axis=0)
        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
        dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1)
        j = np.argmin(dist, axis=1)
        rows = np.arange(len(Q))
        cpb = cp.reshape(len(Q), -1, 3)[rows, j]
        N = A["N"][it[rows, j]]
        dj = dist[rows, j]
        inside = np.einsum("ij,ij->i", Q - cpb, N) < 0.0
        depth = np.where(inside, dj, -dj)
        keep = (depth > -reach) & (depth < 2.5 * self.h)
        if not keep.any():
            return empty
        self.last_ids = np.asarray(idx)[keep]      # which of b's points
        return W[keep], N[keep] @ a.rot(qa).T, depth[keep]

    def penetrations(self, a, b, qa, qb, resting=None):
        """Points of b gone into a at poses qa, qb (dynamics): (world points
        (n, 3), world normals out of a (n, 3), depths (n,)). Only points
        within 1.5 sampling steps of a's surface are seen — the contact
        stiffness keeps them there. ``resting`` (steps): leave out the
        points that touch as drawn (two parts on one hub)."""
        A, B = self.data[a.name], self.data[b.name]
        Pb = B["P"]
        if resting:
            Pb = Pb[~self._mask(a, b, resting)]
        empty = (np.zeros((0, 3)), np.zeros((0, 3)), np.zeros(0))
        if not len(Pb) or not len(A["T"]):
            return empty
        W = _apply(b, qb, Pb)
        L = _unapply(a, qa, W)
        r = 1.5 * self.h
        box = np.all((L >= A["lo"] - r) & (L <= A["hi"] + r), axis=1)
        if not box.any():
            return empty
        W, L = W[box], L[box]
        d, _ = A["tree"].query(L, k=1, distance_upper_bound=r)
        near = np.isfinite(d)
        if not near.any():
            return empty
        W, Q = W[near], L[near]
        k = min(8, len(A["T"]))
        _, it = A["ttree"].query(Q, k=k)
        it = np.atleast_2d(it).reshape(len(Q), -1)
        T = A["T"][it.ravel()]
        Qr = np.repeat(Q, it.shape[1], axis=0)
        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
        dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1)
        j = np.argmin(dist, axis=1)
        rows = np.arange(len(Q))
        tri = it[rows, j]
        cpb = cp.reshape(len(Q), -1, 3)[rows, j]
        N = A["N"][tri]
        inside = np.einsum("ij,ij->i", Q - cpb, N) < 0.0
        if not inside.any():
            return empty
        R = a.rot(qa)
        return W[inside], N[inside] @ R.T, dist[rows, j][inside]

    def overlap_count(self, a, b, qa, qb):
        """How many sampled points of a lie inside b (just under its
        surface) at poses qa, qb: steady while a turns about a hub of b,
        rising when it runs into something (a pin into the end of its slot)."""
        A, B = self.data[a.name], self.data[b.name]
        W = _apply(a, qa, A["P"])
        L = _unapply(b, qb, W)
        reach = 2.0 * self.h
        near = np.all((L >= B["lo"] - reach) & (L <= B["hi"] + reach), axis=1)
        if not near.any():
            return 0
        Q = L[near]
        d, _ = B["tree"].query(Q, k=1, distance_upper_bound=reach)
        Q = Q[np.isfinite(d)]
        if not len(Q):
            return 0
        return int(np.count_nonzero(self._inside_many(B, Q)))

    def _signed(self, A, Q, k=16):
        """Signed distance of points Q (in A's drawn frame) to A's surface:
        < 0 inside A. Inside or not is told by counting the faces crossed
        along three rays (x, y, z; two of three decide), which holds even
        where some faces of the model are turned the wrong way."""
        k = min(k, len(A["T"]))
        _, it = A["ttree"].query(Q, k=k)
        it = np.atleast_2d(it).reshape(len(Q), -1)
        T = A["T"][it.ravel()]
        Qr = np.repeat(Q, it.shape[1], axis=0)
        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
        best = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1).min(axis=1)
        out = best.copy()
        near = np.nonzero(best < self.h)[0]         # near the surface: which side?
        if len(near):
            # all at once: two parts going through each other put tens of
            # thousands of points here, one by one froze the window
            ins = self._inside_many(A, Q[near])
            out[near[ins]] = -best[near[ins]]
        return out

    def _grid(self, A, axis):
        key = "grid%d" % axis
        if key not in A:
            u, v = [c for c in range(3) if c != axis]
            T = A["T"]
            cell = max(4 * self.h, 1e-6)
            lo = np.floor(T[:, :, [u, v]].min(axis=1) / cell).astype(np.int64)
            hi = np.floor(T[:, :, [u, v]].max(axis=1) / cell).astype(np.int64)
            na, nb = hi[:, 0] - lo[:, 0] + 1, hi[:, 1] - lo[:, 1] + 1
            cnt = na * nb
            t = np.repeat(np.arange(len(T)), cnt)
            k = np.arange(len(t)) - np.repeat(np.cumsum(cnt) - cnt, cnt)
            ca = lo[t, 0] + k // nb[t]
            cb = lo[t, 1] + k % nb[t]
            order = np.lexsort((cb, ca))
            t, ca, cb = t[order], ca[order], cb[order]
            cut = np.nonzero(np.diff(ca) | np.diff(cb))[0] + 1
            starts = np.concatenate([[0], cut])
            ends = np.concatenate([cut, [len(t)]])
            grid = {(int(ca[i]), int(cb[i])): t[i:j] for i, j in zip(starts, ends)}
            A[key] = (cell, grid)
        return A[key]

    def _inside_many(self, A, Q):
        """``_inside`` for every point of Q (n, 3) at once: the same three
        rays, the (point, triangle) pairs of each grid cell side by side."""
        Q = np.asarray(Q, float).reshape(-1, 3)
        votes = np.zeros(len(Q), np.int64)
        for axis in range(3):
            u, v = [c for c in range(3) if c != axis]
            cell, grid = self._grid(A, axis)
            cu = np.floor(Q[:, u] / cell).astype(np.int64)
            cv = np.floor(Q[:, v] / cell).astype(np.int64)
            lists = [grid.get((int(a), int(b))) for a, b in zip(cu, cv)]
            cnt = np.array([0 if ix is None else len(ix) for ix in lists], np.int64)
            if not cnt.sum():
                continue
            pi = np.repeat(np.arange(len(Q)), cnt)
            ti = np.concatenate([ix for ix in lists if ix is not None and len(ix)])
            q = Q[pi]
            T = A["T"][ti]
            a, b, c = T[:, 0], T[:, 1], T[:, 2]
            x0, y0 = a[:, u] - q[:, u], a[:, v] - q[:, v]
            x1, y1 = b[:, u] - q[:, u], b[:, v] - q[:, v]
            x2, y2 = c[:, u] - q[:, u], c[:, v] - q[:, v]
            d = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
            ok = np.abs(d) > 1e-30
            d = np.where(ok, d, 1.0)
            l1 = (-x0 * (y2 - y0) + y0 * (x2 - x0)) / d
            l2 = (-(x1 - x0) * y0 + (y1 - y0) * x0) / d
            l0 = 1 - l1 - l2
            hit = ok & (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            z = l0 * a[:, axis] + l1 * b[:, axis] + l2 * c[:, axis]
            n = np.bincount(pi[hit & (z > q[:, axis])], minlength=len(Q))
            votes += n % 2
        return votes >= 2

    def _inside(self, A, q):
        votes = 0
        for axis in range(3):
            u, v = [c for c in range(3) if c != axis]
            cell, grid = self._grid(A, axis)
            ix = grid.get((int(np.floor(q[u] / cell)), int(np.floor(q[v] / cell))))
            if ix is None:
                continue
            T = A["T"][ix]
            a, b, c = T[:, 0], T[:, 1], T[:, 2]
            # where the ray q + t e_axis (t > 0) meets each triangle's plane,
            # inside its projection on (u, v)
            x0, y0 = a[:, u] - q[u], a[:, v] - q[v]
            x1, y1 = b[:, u] - q[u], b[:, v] - q[v]
            x2, y2 = c[:, u] - q[u], c[:, v] - q[v]
            d = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
            ok = np.abs(d) > 1e-30
            d = np.where(ok, d, 1.0)
            l1 = (-x0 * (y2 - y0) + y0 * (x2 - x0)) / d
            l2 = (-(x1 - x0) * y0 + (y1 - y0) * x0) / d
            l0 = 1 - l1 - l2
            hit = ok & (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            z = l0 * a[:, axis] + l1 * b[:, axis] + l2 * c[:, axis]
            n = int(np.count_nonzero(hit & (z > q[axis])))
            votes += n % 2
        return votes >= 2


PENETRATION = 0.0001          # m: deeper than this is a collision (facets)


def _box_gap(alo, ahi, blo, bhi):
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(blo - ahi, alo - bhi))))


def _apply(body, q, P):
    """Drawn-position points of ``body`` -> world at pose q."""
    if body.fixed:
        return P
    R = body.rot(q)
    c = body.center
    return (P - c) @ R.T + c + q[:3]


def _unapply(body, q, W):
    if body.fixed:
        return W
    R = body.rot(q)
    c = body.center
    return (W - c - q[:3]) @ R + c
