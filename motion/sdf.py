"""Signed distance grids of the parts' surfaces (Motion 0.10).

A part is rigid: the distance to its surface, signed (< 0 inside), is the
same in its own frame all along a simulation. It is kept on the nodes of a
grid of cells about half the sampling step, and a point's depth and the
surface's normal there are read by interpolation between the eight corners
of its cell — a few array operations for thousands of points, where
finding each point's nearest triangles took most of a step.

The nodes are worked out the first time a point comes near them (a part
only ever touches the others in a few places: the Socle of the Geneva
mechanism would have 4 million nodes, a simulation reads some thousands),
then kept: keys sorted, values beside, the new ones in a small second
list merged now and then.

Inside or not is told the way ``contact.Surfaces._inside_many`` tells it —
the faces crossed along three rays, two of three decide — so a model with
some faces turned the wrong way still has its inside.
"""

import hashlib
import os
import time

import numpy as np

from . import contact

CACHE_MB = 300           # the grids kept on disk between simulations, at most

_BIAS = 1 << 20          # cell indices shifted to be >= 0 in the keys
_SPAN = 1 << 21

_CORNERS = np.array([(dx, dy, dz) for dx in (0, 1) for dy in (0, 1) for dz in (0, 1)], np.int64)


def _key(I):
    I = np.asarray(I, np.int64) + _BIAS
    return (I[..., 0] * _SPAN + I[..., 1]) * _SPAN + I[..., 2]


def _unkey(K):
    z = K % _SPAN
    y = (K // _SPAN) % _SPAN
    x = K // (_SPAN * _SPAN)
    return np.stack([x, y, z], -1) - _BIAS


class DistanceGrid:
    """Signed distance to the triangles ``T`` (n, 3, 3), turned outward,
    on the nodes of a grid of ``cell``, exact within ``band`` of the
    surface, ± band beyond."""

    def __init__(self, T, cell, band):
        T = np.asarray(T, float).reshape(-1, 3, 3)
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        ok = np.linalg.norm(n, axis=1) > 1e-20
        self.T = T = T[ok]
        self.cell = float(cell)
        self.band = float(band)
        self.lo = np.floor(T.reshape(-1, 3).min(0) / self.cell) * self.cell - 2 * self.cell
        self.box_lo, self.box_hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
        self.sig = hashlib.sha1(np.ascontiguousarray(T, np.float64).tobytes()
                                + np.array([self.cell, self.band]).tobytes()).hexdigest()[:24]
        self.tri = contact.TriGrid(T, self.band)
        self._cols = [self._columns(axis) for axis in range(3)]
        n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
        self.N = n / np.linalg.norm(n, axis=1)[:, None]
        self.keys = np.zeros(0, np.int64)
        self.d = np.zeros(0, np.float32)
        self.t = np.zeros(0, np.int32)            # each node's nearest triangle (-1: none)
        self.new_keys = np.zeros(0, np.int64)
        self.new_d = np.zeros(0, np.float32)
        self.new_t = np.zeros(0, np.int32)
        self.saved = 0
        self._load()
        self.computed = 0

    # -- the nodes, worked out when first needed ---------------------------
    def _columns(self, axis):
        """The triangles over each column of nodes along ``axis``."""
        T, c = self.T, self.cell
        u, v = [a for a in range(3) if a != axis]
        lo_u = np.floor((T[:, :, u].min(1) - self.lo[u]) / c).astype(np.int64)
        hi_u = np.ceil((T[:, :, u].max(1) - self.lo[u]) / c).astype(np.int64)
        lo_v = np.floor((T[:, :, v].min(1) - self.lo[v]) / c).astype(np.int64)
        hi_v = np.ceil((T[:, :, v].max(1) - self.lo[v]) / c).astype(np.int64)
        nb = hi_v - lo_v + 1
        cnt = (hi_u - lo_u + 1) * nb
        keys, tris = [], []
        csum = np.cumsum(cnt)
        k = 0
        while k < len(T):                     # by chunks of 4 M entries at most
            e = max(k + 1, int(np.searchsorted(csum, (csum[k - 1] if k else 0) + 4_000_000, "right")))
            sl = slice(k, e)
            k = e
            ti = np.repeat(np.arange(sl.start, sl.stop), cnt[sl])
            loc = np.arange(int(cnt[sl].sum())) - np.repeat(np.cumsum(cnt[sl]) - cnt[sl], cnt[sl])
            keys.append((lo_u[ti] + loc // nb[ti] + _BIAS) * _SPAN + (lo_v[ti] + loc % nb[ti] + _BIAS))
            tris.append(ti)
        key = np.concatenate(keys)
        tri = np.concatenate(tris)
        o = np.argsort(key, kind="stable")
        return key[o], tri[o]

    def _inside(self, nodes):
        """Nodes inside the closed surface (two of three rays)."""
        votes = np.zeros(len(nodes), np.int8)
        T, c = self.T, self.cell
        for axis in range(3):
            u, v = [a for a in range(3) if a != axis]
            ckey, ctri = self._cols[axis]
            nk = (nodes[:, u] + _BIAS) * _SPAN + (nodes[:, v] + _BIAS)
            A = np.searchsorted(ckey, nk, "left")
            cnt = np.searchsorted(ckey, nk, "right") - A
            if not cnt.sum():
                continue
            pi = np.repeat(np.arange(len(nodes)), cnt)
            ti = ctri[np.repeat(A, cnt) + np.arange(int(cnt.sum())) - np.repeat(np.cumsum(cnt) - cnt, cnt)]
            a, b, cc = T[ti, 0], T[ti, 1], T[ti, 2]
            qu = self.lo[u] + nodes[pi, u] * c
            qv = self.lo[v] + nodes[pi, v] * c
            x0, y0 = a[:, u] - qu, a[:, v] - qv
            x1, y1 = b[:, u] - qu, b[:, v] - qv
            x2, y2 = cc[:, u] - qu, cc[:, v] - qv
            det = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
            good = np.abs(det) > 1e-30
            det = np.where(good, det, 1.0)
            l1 = (-x0 * (y2 - y0) + y0 * (x2 - x0)) / det
            l2 = (-(x1 - x0) * y0 + (y1 - y0) * x0) / det
            l0 = 1 - l1 - l2
            hit = good & (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
            z = l0 * a[:, axis] + l1 * b[:, axis] + l2 * cc[:, axis]
            qz = self.lo[axis] + nodes[pi, axis] * c
            n = np.bincount(pi[hit & (z > qz)], minlength=len(nodes))
            votes += (n % 2 == 1).astype(np.int8)
        return votes >= 2

    def _work_out(self, K):
        """Signed distances at the nodes of keys K (new, unique)."""
        I = _unkey(K)
        X = self.lo + I * self.cell
        out = np.full(len(K), np.nan, np.float32)
        # beyond the band the distance is the band's (sign kept): a point
        # far inside reads deep, far outside reads clear
        dist = np.full(len(K), self.band)
        tri = np.full(len(K), -1, np.int32)
        keep, it = self.tri.nearest(X, k=1)
        if keep.any():
            Tk = self.T[it[:, 0]]
            cp = contact.closest_on_triangles(X[keep], Tk[:, 0], Tk[:, 1], Tk[:, 2])
            dist[keep] = np.minimum(np.linalg.norm(X[keep] - cp, axis=1), self.band)
            tri[keep] = it[:, 0]
        ins = np.zeros(len(K), bool)
        inbox = np.all((X >= self.box_lo) & (X <= self.box_hi), axis=1)
        if inbox.any():
            ins[inbox] = self._inside(I[inbox])
        out[:] = np.where(ins, -dist, dist)
        self.computed += len(K)
        return out, tri

    def _lookup(self, K):
        """Values and nearest triangles at keys K (any shape), working out
        the missing ones."""
        flat = K.ravel()
        val = np.full(len(flat), np.nan, np.float32)
        tri = np.full(len(flat), -1, np.int32)
        found = np.zeros(len(flat), bool)
        for keys, d, t in ((self.keys, self.d, self.t), (self.new_keys, self.new_d, self.new_t)):
            if not len(keys):
                continue
            p = np.minimum(np.searchsorted(keys, flat), len(keys) - 1)
            hit = (keys[p] == flat) & ~found
            val[hit] = d[p[hit]]
            tri[hit] = t[p[hit]]
            found |= hit
        if not found.all():
            miss, inv = np.unique(flat[~found], return_inverse=True)
            vals, tris = self._work_out(miss)
            val[~found] = vals[inv]
            tri[~found] = tris[inv]
            nk = np.concatenate([self.new_keys, miss])
            nd = np.concatenate([self.new_d, vals])
            nt = np.concatenate([self.new_t, tris])
            o = np.argsort(nk, kind="stable")
            self.new_keys, self.new_d, self.new_t = nk[o], nd[o], nt[o]
            if len(self.new_keys) > max(4096, len(self.keys) // 8):
                mk = np.concatenate([self.keys, self.new_keys])
                md = np.concatenate([self.d, self.new_d])
                mt = np.concatenate([self.t, self.new_t])
                o = np.argsort(mk, kind="stable")
                self.keys, self.d, self.t = mk[o], md[o], mt[o]
                self.new_keys = np.zeros(0, np.int64)
                self.new_d = np.zeros(0, np.float32)
                self.new_t = np.zeros(0, np.int32)
        return val.reshape(K.shape), tri.reshape(K.shape)

    # -- reading ----------------------------------------------------------
    def query(self, Q):
        """Signed distance (± band beyond the band) and outward unit normal
        (the distance's slope) at points Q (m, 3) of the part's frame."""
        Q = np.asarray(Q, float).reshape(-1, 3)
        if not len(Q):
            self.last_tris = np.zeros((0, 8), np.int32)
            return np.zeros(0), np.zeros((0, 3))
        # well outside the part's box: clear of it, nothing to work out
        gap = np.linalg.norm(np.maximum(np.maximum(self.box_lo - Q, Q - self.box_hi), 0.0), axis=1)
        far = gap > self.band + 2 * self.cell
        if far.any():
            d = gap.copy()
            N = np.zeros((len(Q), 3))
            tris = np.full((len(Q), 8), -1, np.int32)
            if (~far).any():
                d[~far], N[~far] = self.query(Q[~far])
                tris[~far] = self.last_tris
            self.last_tris = tris
            return d, N
        g = (Q - self.lo) / self.cell
        I = np.floor(g).astype(np.int64)
        f = g - I
        v, tri = self._lookup(_key(I[:, None, :] + _CORNERS[None]))
        self.last_tris = tri                                   # (m, 8)
        v = v.astype(float).reshape(-1, 2, 2, 2)
        fx, fy, fz = f[:, 0, None, None], f[:, 1, None], f[:, 2]
        vx = v[:, 0] * (1 - fx) + v[:, 1] * fx                 # (m, 2, 2): y, z
        vy = vx[:, 0] * (1 - fy) + vx[:, 1] * fy               # (m, 2): z
        d = vy[:, 0] * (1 - fz) + vy[:, 1] * fz
        gz = vy[:, 1] - vy[:, 0]
        gy = vx[:, 1] - vx[:, 0]
        gy = gy[:, 0] * (1 - fz) + gy[:, 1] * fz
        gx = v[:, 1] - v[:, 0]
        gx = gx[:, 0] * (1 - fy) + gx[:, 1] * fy
        gx = gx[:, 0] * (1 - fz) + gx[:, 1] * fz
        G = np.stack([gx, gy, gz], 1) / self.cell
        nrm = np.linalg.norm(G, axis=1)
        N = G / np.where(nrm > 1e-12, nrm, 1.0)[:, None]
        return d, N

    def exact(self, Q, rows):
        """For points Q[rows]: the exact signed distance and the normal of
        the nearest of the triangles nearest to their cell's corners (the
        interpolation rounds the inside of an edge by a fraction of a
        cell: enough to let a part sink in a little more, Genève)."""
        # the eight corners mostly share their nearest triangle: each
        # triangle once per point (a third of a simulation's time went
        # into the same closest point worked out up to eight times)
        tri = np.sort(self.last_tris[rows], axis=1)
        dup = np.zeros(tri.shape, bool)
        dup[:, 1:] = tri[:, 1:] == tri[:, :-1]
        t = np.where(dup, -1, tri).ravel()
        ok = t >= 0
        Qk = Q[rows][np.repeat(np.arange(len(rows)), 8)[ok]]
        dist = np.full(len(t), np.inf)
        cp = np.zeros((len(t), 3))
        Tk = self.T[t[ok]]
        cp[ok] = contact.closest_on_triangles(Qk, Tk[:, 0], Tk[:, 1], Tk[:, 2])
        dist[ok] = np.linalg.norm(Qk - cp[ok], axis=1)
        tri = t.reshape(-1, 8)
        dist = dist.reshape(-1, 8)
        j = np.argmin(dist, axis=1)
        r = np.arange(len(rows))
        best = dist[r, j]
        tb = tri[r, j]
        cpb = cp.reshape(-1, 8, 3)[r, j]
        N = self.N[np.maximum(tb, 0)]
        inside = np.einsum("ij,ij->i", Q[rows] - cpb, N) < 0.0
        return np.where(inside, -best, best), N, np.isfinite(best)

    # -- kept between simulations -----------------------------------------
    @staticmethod
    def cache_dir():
        base = os.environ.get("LOCALAPPDATA") or os.path.join(os.path.expanduser("~"), ".cache")
        return os.path.join(base, "ingetrazo-motion", "grids")

    def _path(self):
        return os.path.join(self.cache_dir(), self.sig + ".npz")

    def _load(self):
        """The nodes worked out by an earlier simulation of the same part."""
        try:
            with np.load(self._path()) as z:
                k, d, t = z["keys"], z["d"], z["t"]
            if len(k) == len(d) == len(t):
                self.keys, self.d, self.t = k.astype(np.int64), d.astype(np.float32), t.astype(np.int32)
                self.saved = len(k)
                os.utime(self._path())
        except Exception:
            pass

    def save(self):
        """Keep the nodes for the next simulation (when it has new ones)."""
        if len(self.keys) + len(self.new_keys) <= self.saved:
            return
        try:
            if len(self.new_keys):
                mk = np.concatenate([self.keys, self.new_keys])
                md = np.concatenate([self.d, self.new_d])
                mt = np.concatenate([self.t, self.new_t])
                o = np.argsort(mk, kind="stable")
                self.keys, self.d, self.t = mk[o], md[o], mt[o]
                self.new_keys = np.zeros(0, np.int64)
                self.new_d = np.zeros(0, np.float32)
                self.new_t = np.zeros(0, np.int32)
            os.makedirs(self.cache_dir(), exist_ok=True)
            tmp = self._path() + ".tmp.npz"
            np.savez(tmp, keys=self.keys, d=self.d, t=self.t)
            os.replace(tmp, self._path())
            self.saved = len(self.keys)
            _trim(self.cache_dir())
        except Exception:
            pass


def _trim(folder):
    """The oldest grids out once the folder holds more than CACHE_MB."""
    try:
        files = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".npz")]
        files.sort(key=lambda f: os.path.getmtime(f))
        total = sum(os.path.getsize(f) for f in files)
        while files and total > CACHE_MB * 1e6:
            f = files.pop(0)
            total -= os.path.getsize(f)
            os.remove(f)
    except Exception:
        pass
