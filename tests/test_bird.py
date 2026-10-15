"""Flapping bird: one crank, two wings of two segments, closed forms."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import samples, solver, collide

TOL = 1e-8
_cache = {}


def _bird():
    if "m" not in _cache:
        m, info = samples.oiseau()
        _cache.update(m=m, info=info, sweep=solver.sweep(m, 25))
    return _cache["m"], _cache["info"], _cache["sweep"]


def _pt(m, body, x, p):
    return body.world_point(m.pose(x, body), p)


def test_one_dof_all_round():
    m, info, sw = _bird()
    for k in (0, 7, 13, 20):
        assert solver.dof(m, sw[k]["x"]) == 1


def test_full_turn_matches_closed_form():
    m, info, sw = _bird()
    assert all(s["ok"] for s in sw)
    for s in sw:                                   # 0 .. 360 deg, past +-180
        ref = samples.oiseau_reference(info, s["value"])
        slider = info["bodies"]["coulisseau"]
        assert abs(_pt(m, slider, s["x"], info["Q0"])[2] - ref["zQ"]) < TOL
        for tag, w in info["wings"].items():
            for key, body in (("A", w["arm"]), ("E", w["arm"]), ("C", w["hand"]),
                              ("tip", w["hand"])):
                assert np.linalg.norm(_pt(m, body, s["x"], w[key]) - ref[tag][key]) < TOL


def test_wings_are_mirror_images():
    m, info, sw = _bird()
    D, G = info["wings"]["D"], info["wings"]["G"]
    for s in sw[::3]:
        a = _pt(m, D["hand"], s["x"], D["tip"])
        b = _pt(m, G["hand"], s["x"], G["tip"])
        assert np.linalg.norm(a * (-1, 1, 1) - b) < TOL


def test_stroke_ends_at_slider_ends():
    # the arm angle depends only on the slider height, which is extreme at
    # 90 deg (Lc + r) and 270 deg (Lc - r): the stroke ends are there
    m, info, sw = _bird()
    r, Lc = info["r"], info["Lc"]
    hi = samples.oiseau_reference(info, math.pi / 2)
    lo = samples.oiseau_reference(info, 3 * math.pi / 2)
    assert abs(hi["zQ"] - (Lc + r)) < 1e-15 and abs(lo["zQ"] - (Lc - r)) < 1e-15
    scan = [samples.oiseau_reference(info, t)["D"]["arm"]
            for t in np.linspace(0, 2 * math.pi, 3601)]
    assert abs(min(scan) - hi["D"]["arm"]) < 1e-6      # slider up -> wing down
    assert abs(max(scan) - lo["D"]["arm"]) < 1e-6
    x = None
    for t in np.linspace(0, math.pi / 2, 7):
        x, ok, _ = solver.solve(m, t, x)
    w = info["wings"]["D"]
    E = _pt(m, w["arm"], x, w["E"])
    ang = math.atan2(E[2] - w["S"][2], E[0] - w["S"][0])
    assert abs(ang - hi["D"]["arm"]) < TOL


def test_hand_amplifies_the_beat():
    m, info, sw = _bird()
    refs = [samples.oiseau_reference(info, t) for t in np.linspace(0, 2 * math.pi, 361)]
    arm = [r["D"]["arm"] for r in refs]
    hand = [r["D"]["hand"] for r in refs]
    k = (max(hand) - min(hand)) / (max(arm) - min(arm))
    assert 1.3 < k < 1.8, k


def test_full_turn_closes():
    m, info, sw = _bird()
    w = info["wings"]["D"]
    a = _pt(m, w["hand"], sw[0]["x"], w["tip"])
    b = _pt(m, w["hand"], sw[-1]["x"], w["tip"])
    assert np.linalg.norm(a - b) < TOL


def test_no_collision_and_smallest_gap():
    m, info, sw = _bird()
    rep = collide.validate(m, 13, jeu_min=0.0015)
    assert rep["ok"], rep["problems"]
    assert abs(rep["worst"]["distance"] - 0.002) < 1e-9     # hand lever / tie rod


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
