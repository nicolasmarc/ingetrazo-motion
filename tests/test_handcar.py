"""Hand-pumped rail car: beam, rod, crank, gears, rolling axles."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import samples, solver, collide

TOL = 1e-8
_c = {}


def _car():
    if "m" not in _c:
        m, info = samples.draisine()
        _c.update(m=m, info=info, sw=solver.sweep(m, 25))
    return _c["m"], _c["info"], _c["sw"]


def test_one_dof():
    m, info, sw = _car()
    for k in (0, 9, 17):
        assert solver.dof(m, sw[k]["x"]) == 1


def test_three_turns_match_closed_form():
    m, info, sw = _car()
    J, B = info["joints"], info["bodies"]
    for s in sw:
        assert s["ok"], s
        x, ref = s["x"], samples.draisine_reference(info, s["value"])
        off = np.array([m.continuous_value(J["voie"], x), 0, 0])
        K = B["arbre"].world_point(m.pose(x, B["arbre"]), info["K"]) - off
        Bw = B["balancier"].world_point(m.pose(x, B["balancier"]), info["B"]) - off
        assert np.linalg.norm(K - ref["K"]) < TOL
        assert np.linalg.norm(Bw - ref["B"]) < TOL
        assert abs(m.continuous_value(J["balancier"], x) - ref["beam"]) < TOL
        assert abs(m.continuous_value(J["arriere"], x) - ref["axle"]) < TOL
        assert abs(m.continuous_value(J["avant"], x) - ref["axle"]) < TOL
        assert abs(off[0] - ref["travel"]) < TOL


def test_travel_per_turn():
    m, info, sw = _car()
    s = m.continuous_value(info["joints"]["voie"], sw[-1]["x"])
    expect = 3 * 2 * math.pi * info["R"] * info["ratio"]    # 3 turns x 2 x 2 pi R
    assert abs(s - expect) < TOL


def test_axles_roll_without_slipping():
    m, info, sw = _car()
    J = info["joints"]
    for s in sw[3:20:4]:
        x = s["x"]
        x2, ok, _ = solver.solve(m, s["value"] + 1e-6, x)
        travel = m.continuous_value(J["voie"], x)
        for name, O in (("arriere", info["Or"]), ("avant", info["Of"])):
            wheel = info["bodies"][name]
            contact = np.array([O[0] + travel, 0.5, 0.0])     # on the rail
            mat = wheel.local_point(m.pose(x, wheel), contact)
            moved = wheel.world_point(m.pose(x2, wheel), mat) - contact
            assert np.linalg.norm(moved) < 1e-10


def test_beam_stroke_ends():
    # crank and rod in line: |B - C| = L + rc or L - rc
    m, info, sw = _car()
    P, C, db, L, rc = info["P"], info["C"], info["db"], info["L"], info["rc"]
    ends = []
    side = samples._side(P, C, info["B"])         # beam end side of line P-C
    for d in (L + rc, L - rc):
        Bx = samples._circle_cut(P, db, C, d, side)
        v, v0 = Bx - P, info["B"] - P
        ends.append(math.remainder(math.atan2(v[2], v[0]) - math.atan2(v0[2], v0[0]),
                                   2 * math.pi))
    scan = [samples.draisine_reference(info, t)["beam"]
            for t in np.linspace(0, 2 * math.pi, 20001)]
    assert abs(min(scan) - min(ends)) < 1e-6 and abs(max(scan) - max(ends)) < 1e-6


def test_clean_run_and_smallest_gap():
    m, info, sw = _car()
    rep = collide.validate(m, 25, jeu_min=info["jeu_min"])
    assert rep["ok"], rep["problems"]
    w = rep["worst"]
    assert (w["box_a"], w["box_b"]) == ("support d'arbre", "roue dentée")
    assert abs(w["distance"] - 0.005) < 1e-9


def test_crank_through_the_shaft_is_caught():
    # the first draft: shaft across the rod's plane
    m, info = samples.draisine()
    shaft = info["bodies"]["arbre"]
    shaft.boxes[0].center = info["C"].copy()
    shaft.boxes[0].half = np.array([0.025, 0.26, 0.025])
    rep = collide.validate(m, 13)
    assert any("arbre de manivelle / bielle" in p for p in rep["problems"])


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
