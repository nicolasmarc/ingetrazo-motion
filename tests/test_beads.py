"""Motion 0.10: beads poured among the parts."""
import os, sys

import numpy as np

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from motion import model, particles                        # noqa: E402
from shapes import box, funnel                              # noqa: E402


def _rows(seconds, x=np.zeros(0)):
    return [dict(t=t, x=x) for t in np.arange(0, seconds + 1e-9, 1 / 30)]


def _run(tris, sources, seconds, **kw):
    m = model.Mechanism()
    B = particles.Beads(m, [(n, m.ground, T) for n, T in tris.items()], sources, **kw)
    return B, B.run(_rows(seconds), dt=1 / 600)


def _tray(w=0.03, h=0.03, at=(0.0, 0.0, 0.0)):
    T = np.concatenate([box((-w, -w, -0.01), (w, w, 0.0)),
                        box((-w - 0.005, -w, 0.0), (-w, w, h)), box((w, -w, 0.0), (w + 0.005, w, h)),
                        box((-w - 0.005, -w - 0.005, 0.0), (w + 0.005, -w, h)),
                        box((-w - 0.005, w, 0.0), (w + 0.005, w + 0.005, h))])
    return T + np.asarray(at, float)


def test_beads_pile_up_and_rest_in_a_tray(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    tris = {"bac": _tray()}
    B, out = _run(tris, [dict(point=(0, 0, 0.04), diametre=0.006, debit=400, nombre=120,
                              largeur=0.04)], 2.5)
    X, r = out[-1][0], out[-1][1]
    assert len(X) == 120 and B.lost == 0
    assert (X[:, 2] - r).min() > -1e-4                      # on the plate, not in it
    assert np.linalg.norm(B.V, axis=1).max() < 0.05        # at rest
    D = np.linalg.norm(X[:, None] - X[None], axis=2) + np.eye(len(X))
    assert (0.006 - D).max() < 2e-4                        # beads do not sink in each other
    assert X[:, 2].max() > 0.008                            # in layers, not a single one


def test_a_funnel_lets_them_through_onto_a_belt_into_a_sink(tmp_path, monkeypatch):
    monkeypatch.setenv("LOCALAPPDATA", str(tmp_path))
    tris = {"entonnoir": funnel(0.05, 0.012, 0.15, 0.10, spout=0.015),
            "tapis": box((-0.03, -0.04, 0.03), (0.25, 0.04, 0.04)),
            "bac": _tray(0.06, 0.04, (0.32, 0.0, -0.02))}
    B, out = _run(tris, [dict(point=(0, 0, 0.16), diametre=0.006, debit=200, nombre=150,
                              largeur=0.06)], 4.0,
                  sinks={"bac"}, conveyors={"tapis": ((1, 0, 0), 0.25)})
    # through the funnel's spout (nothing left in it), carried by the belt
    k = 45                                                    # 1.5 s
    X = out[k][0]
    on_belt = (X[:, 2] > 0.035) & (X[:, 2] < 0.06)
    assert on_belt.sum() > 50
    assert (X[:, 2] > 0.09).sum() < 15
    # the belt's beads go its way, into the bin, and stay there (counted)
    assert out[-1][3].get("bac", 0) >= 130, out[-1][3]
    X = out[-1][0]
    assert ((X[:, 0] > 0.26) & (X[:, 0] < 0.38) & (X[:, 2] < 0.03)).sum() >= 130
    assert not any(((x[:, 2] > 0.04) & (np.hypot(x[:, 0], x[:, 1]) < 0.05)
                    & (x[:, 2] < 0.15) & (np.hypot(x[:, 0], x[:, 1]) > 0.06)).any() for x, *_ in out[1:])


def test_a_narrow_spout_lets_fewer_through():
    def through(spout):
        tris = {"entonnoir": funnel(0.05, spout, 0.15, 0.10, spout=0.015),
                "plateau": box((-0.2, -0.2, -0.01), (0.2, 0.2, 0.0))}
        B, out = _run(tris, [dict(point=(0, 0, 0.16), diametre=0.006, debit=300, nombre=150,
                                  largeur=0.06)], 1.5)
        return int((out[-1][0][:, 2] < 0.08).sum())
    wide, narrow = through(0.014), through(0.0078)
    assert wide > 2 * narrow, (wide, narrow)


def test_beads_follow_a_moving_part():
    # a plate turning about z under a few beads carries them round
    m = model.Mechanism()
    b = m.body("plateau", center=(0, 0, 0))
    T = box((-0.1, -0.1, -0.01), (0.1, 0.1, 0.0))
    rows = []
    for t in np.arange(0, 1.5 + 1e-9, 1 / 30):
        x = np.zeros(6)
        x[5] = 1.0 * max(t - 0.5, 0.0)                    # 1 rad/s from 0.5 s
        rows.append(dict(t=t, x=x))
    B = particles.Beads(m, [("plateau", b, T)], [dict(point=(0.06, 0, 0.01), diametre=0.006,
                                                     debit=100, nombre=1)])
    out = B.run(rows, dt=1 / 600)
    p0 = out[15][0][0]
    p1 = out[-1][0][0]
    a0, a1 = np.arctan2(p0[1], p0[0]), np.arctan2(p1[1], p1[0])
    assert abs((a1 - a0) - 1.0) < 0.15, (a0, a1)          # turned with the plate (1 rad)
