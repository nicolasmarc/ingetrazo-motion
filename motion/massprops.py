"""Mass properties of closed triangle meshes: volume, centre of gravity,
inertia tensor (about the centre of gravity, world axes).

Each triangle (a, b, c) and the origin make a tetrahedron; its signed
volume and second moments add up to the solid's (divergence theorem), so
the mesh need not be convex — only closed and turned outward.
"""
import numpy as np

# second moments of the unit tetrahedron (0, e1, e2, e3): int x_i x_j dV
_C = np.array([[2.0, 1.0, 1.0], [1.0, 2.0, 1.0], [1.0, 1.0, 2.0]]) / 120.0


def of_triangles(T, density=1.0):
    """(mass, centre (3,), inertia (3, 3) about the centre, volume) of the
    solid bounded by triangles ``T`` (n, 3, 3), outward."""
    T = np.asarray(T, float).reshape(-1, 3, 3)
    if not len(T):
        return 0.0, np.zeros(3), np.zeros((3, 3)), 0.0
    # shift to a local origin: better conditioned far from the world origin
    o = T.reshape(-1, 3).mean(axis=0)
    a, b, c = T[:, 0] - o, T[:, 1] - o, T[:, 2] - o
    det = np.einsum("ij,ij->i", a, np.cross(b, c))           # 6 x tetra volume
    vol = det.sum() / 6.0
    if abs(vol) < 1e-18:
        return 0.0, o.copy(), np.zeros((3, 3)), 0.0
    centre = (det[:, None] * (a + b + c)).sum(axis=0) / 24.0 / vol
    # covariance int x x^T dV, tetra by tetra: det A C A^T, A = [a b c]
    A = np.stack([a, b, c], axis=2)                          # (n, 3, 3) columns
    cov = np.einsum("n,nij,jk,nlk->il", det, A, _C, A)
    cov_c = cov - vol * np.outer(centre, centre)              # about the centre
    I = np.trace(cov_c) * np.eye(3) - cov_c
    return density * vol, centre + o, density * I, vol


def combine(items):
    """Mass, centre and inertia (about the centre) of several pieces
    [(mass, centre, inertia about own centre), ...] — parallel axes."""
    m = sum(x[0] for x in items)
    if m <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    c = sum(x[0] * np.asarray(x[1]) for x in items) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in items:
        d = np.asarray(ci) - c
        I += Ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
    return m, c, I
