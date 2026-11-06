"""Closed test shapes (triangles turned outward) for the bead tests."""
import math
import numpy as np


def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    c = lambda i: np.where([i & 1, i & 2, i & 4], hi, lo)
    T = []
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        p = [c(i) for i in q]
        T += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return np.array(T)


def _ring(R, z, n, c=(0.0, 0.0)):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.stack([c[0] + R * np.cos(a), c[1] + R * np.sin(a), np.full(n, z)], 1)


def funnel(r_top, r_bottom, z_top, z_bottom, spout=0.0, thick=0.002, n=64, c=(0.0, 0.0)):
    """A funnel: a cone open at both ends (and a straight spout of length
    ``spout`` under it), a wall ``thick`` thick, closed solid."""
    zs = [z_top, z_bottom] + ([z_bottom - spout] if spout > 0 else [])
    rs = [r_top, r_bottom] + ([r_bottom] if spout > 0 else [])
    inner = [_ring(r, z, n, c) for r, z in zip(rs, zs)]
    outer = [_ring(r + thick, z, n, c) for r, z in zip(rs, zs)]
    T = []

    def band(A, B, flip):
        for k in range(n):
            a0, a1, b0, b1 = A[k], A[(k + 1) % n], B[k], B[(k + 1) % n]
            t1, t2 = [a0, b0, b1], [a0, b1, a1]
            if flip:
                t1, t2 = t1[::-1], t2[::-1]
            T.extend([t1, t2])
    for i in range(len(zs) - 1):
        band(outer[i], outer[i + 1], False)       # outer wall, normals out
        band(inner[i], inner[i + 1], True)        # inner wall, normals in (towards the axis)
    band(inner[0], outer[0], True)                # top rim
    band(inner[-1], outer[-1], False)             # bottom rim
    T = np.array(T)
    o = T.reshape(-1, 3).mean(0)
    vol = np.einsum("ij,ij->i", T[:, 0] - o, np.cross(T[:, 1] - o, T[:, 2] - o)).sum()
    return T[:, ::-1] if vol < 0 else T
