"""The numpy nearest-point search (IngeTrazo's Windows build has no scipy)
answers as scipy's k-d tree does."""
import numpy as np
import pytest

from motion.contact import GridTree


def _points(rng):
    sph = lambda v: v / np.linalg.norm(v, axis=1)[:, None]
    return np.vstack([np.c_[rng.random((3000, 2)), np.zeros(3000)],
                      sph(rng.normal(size=(3000, 3))) * 0.3 + 0.5])


def _brute(P, X, k):
    d = np.linalg.norm(X[:, None, :] - P[None, :, :], axis=2)
    i = np.argsort(d, axis=1)[:, :k]
    return np.take_along_axis(d, i, 1), i


def test_nearest_within_a_radius():
    rng = np.random.default_rng(0)
    P = _points(rng)
    X = np.vstack([rng.random((800, 3)) * 1.2 - 0.1, P[:400] + rng.normal(size=(400, 3)) * 0.002])
    G = GridTree(P)
    for r in (0.002, 0.02, 0.1):
        d, i = G.query(X, k=1, distance_upper_bound=r)
        bd, bi = _brute(P, X, 1)
        bd, bi = bd[:, 0], bi[:, 0]
        inside = bd < r
        assert np.allclose(d[inside], bd[inside]) and np.all(np.isinf(d[~inside]))
        assert np.all(i[~inside] == len(P))
        assert np.allclose(np.linalg.norm(P[i[inside]] - X[inside], axis=1), bd[inside])


def test_k_nearest():
    rng = np.random.default_rng(1)
    P = _points(rng)
    X = P[:300] + rng.normal(size=(300, 3)) * 0.003
    d, i = GridTree(P).query(X, k=16)
    bd, _ = _brute(P, X, 16)
    assert d.shape == (300, 16) and np.allclose(d, bd)


def test_contact_works_without_scipy(monkeypatch):
    from motion import contact
    monkeypatch.setattr(contact, "_Tree", GridTree)
    assert contact.available()
