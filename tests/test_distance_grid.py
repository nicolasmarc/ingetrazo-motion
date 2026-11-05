"""Motion 0.10: the parts' signed distance grids."""

import os, sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import sdf                                       # noqa: E402
from test_mass import box                                    # noqa: E402


def _exact_box(Q, half):
    q = np.abs(Q) - half
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0)


def test_signed_distance_of_a_box(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    half = np.array([0.02, 0.01, 0.005])
    T = box(tuple(-half), tuple(half)).reshape(-1, 3, 3)
    G = sdf.DistanceGrid(T, 0.0005, 0.0025)
    Q = np.random.default_rng(1).uniform(-half - 0.003, half + 0.003, (5000, 3))
    d, N = G.query(Q)
    ex = _exact_box(Q, half)
    near = np.abs(ex) < 0.002
    err = np.abs(d[near] - ex[near])
    assert np.percentile(err, 99) < 1e-4 and np.all(np.sign(d[near & (np.abs(ex) > 3e-4)])
                                                    == np.sign(ex[near & (np.abs(ex) > 3e-4)]))
    # near the surface, the exact distance from the corners' triangles
    # (of the points just read)
    rows = np.nonzero(near)[0]
    de, Ne, ok = G.exact(Q, rows)
    assert ok.all() and np.max(np.abs(de - ex[near])) < 1e-9
    # the normal on the big face points out of it
    d1, N1 = G.query(np.array([[0.0, 0.0, 0.0052]]))
    assert abs(d1[0] - 0.0002) < 2e-5 and N1[0] @ (0, 0, 1) > 0.99


def test_faces_turned_the_wrong_way_keep_the_inside(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    T = box((-0.01, -0.01, -0.01), (0.01, 0.01, 0.01)).reshape(-1, 3, 3).copy()
    T[:2] = T[:2, ::-1]                                       # one face turned
    G = sdf.DistanceGrid(T, 0.0005, 0.0025)
    d, _ = G.query(np.array([[0.0, 0.0, 0.009], [0.0, 0.0, 0.011]]))
    assert d[0] < 0 < d[1]


def test_the_nodes_are_kept_for_the_next_simulation(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    T = box((-0.01, -0.01, -0.01), (0.01, 0.01, 0.01)).reshape(-1, 3, 3)
    Q = np.random.default_rng(2).uniform(-0.012, 0.012, (500, 3))
    G = sdf.DistanceGrid(T, 0.0005, 0.0025)
    d, _ = G.query(Q)
    assert G.computed > 0
    G.save()
    G2 = sdf.DistanceGrid(T, 0.0005, 0.0025)
    d2, _ = G2.query(Q)
    assert G2.computed == 0 and np.allclose(d, d2, equal_nan=True)
