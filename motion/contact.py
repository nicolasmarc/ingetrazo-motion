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
import numpy as np

try:
    from scipy.spatial import cKDTree as _Tree
except Exception:                                  # pragma: no cover
    _Tree = None


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


class Surfaces:
    """The sampled surfaces of a mechanism's parts, and the checks."""

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
        self._gears = set()
        for j in mech.joints:
            if j.kind == "coupling":
                for ja, jb in ((j.ja, j.jb),):
                    self._gears.add(frozenset((ja.b.name, jb.b.name)))
                    self._gears.add(frozenset((ja.a.name, jb.b.name)))
                    self._gears.add(frozenset((ja.b.name, jb.a.name)))

    def has(self, body):
        return body.name in self.data

    def _resting(self, a, b):
        """Points of a and of b that lie against the other in the drawn
        position (masks)."""
        key = (a.name, b.name)
        if key not in self._skip:
            A, B = self.data[a.name], self.data[b.name]
            r = 1.5 * self.h
            if _box_gap(A["lo"], A["hi"], B["lo"], B["hi"]) > r:
                ma, mb = np.zeros(len(A["P"]), bool), np.zeros(len(B["P"]), bool)
            else:
                ma = B["tree"].query(A["P"], k=1, distance_upper_bound=r)[0] < r
                mb = A["tree"].query(B["P"], k=1, distance_upper_bound=r)[0] < r
            self._skip[key] = (ma, mb)
        return self._skip[key]

    def pair(self, a, b, qa, qb, reach):
        """Smallest distance (< reach) between the free points of a and b
        at poses qa, qb: (distance, point a, point b, group a, group b) or
        None."""
        if frozenset((a.name, b.name)) in self._gears:
            return None
        A, B = self.data[a.name], self.data[b.name]
        ma, mb = self._resting(a, b)
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


    def _signed(self, A, Q, k=16):
        """Signed distance of points Q (in A's drawn frame) to A's surface:
        < 0 inside A."""
        k = min(k, len(A["T"]))
        _, it = A["ttree"].query(Q, k=k)
        it = np.atleast_2d(it).reshape(len(Q), -1)
        T = A["T"][it.ravel()]
        Qr = np.repeat(Q, it.shape[1], axis=0)
        cp = closest_on_triangles(Qr, T[:, 0], T[:, 1], T[:, 2])
        dist = np.linalg.norm(Qr - cp, axis=1).reshape(len(Q), -1)
        side = _dot(Qr - cp, A["N"][it.ravel()]).reshape(len(Q), -1)
        best = dist.min(axis=1)
        tie = dist <= best[:, None] * (1 + 1e-6) + 1e-12
        outside = np.where(tie, side, -np.inf).max(axis=1) >= -1e-12
        return np.where(outside, best, -best)


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
