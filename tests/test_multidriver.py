"""Several motors on one sweep: speeds and phases, against closed forms."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import solver, collide, document as doc

CCW = (0, -1.0, 0)


def _cut(c0, r0, c1, r1, side):
    d = c1 - c0
    L = math.hypot(d[0], d[2])
    a = (r0 * r0 - r1 * r1 + L * L) / (2 * L)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    u = d / L
    n = np.array([-u[2], 0.0, u[0]])
    return c0 + a * u + side * h * n


def _fourbar(m, dx, tag):
    """Crank 1, coupler 3.5, rocker 3, ground 4 (crank-rocker), at x+dx."""
    O2, O4 = np.array([dx, 0, 0.0]), np.array([dx + 4, 0, 0.0])
    B = O2 + np.array([1.0, 0, 0])
    C = _cut(O4, 3.0, B, 3.5, 1.0)
    crank = m.body(f"manivelle{tag}", (O2 + B) / 2, [Box((O2 + B) / 2, (0.5, .05, .05))])
    coupler = m.body(f"bielle{tag}", (B + C) / 2, [])
    rocker = m.body(f"balancier{tag}", (C + O4) / 2, [])
    p = m.pivot(m.ground, crank, O2, CCW, name=f"moteur{tag}")
    m.pivot(crank, coupler, B, CCW)
    m.pivot(coupler, rocker, C, CCW)
    m.pivot(rocker, m.ground, O4, CCW)
    side = 1.0 if (B - O4)[0] * (C - O4)[2] - (B - O4)[2] * (C - O4)[0] >= 0 else -1.0
    return dict(p=p, O2=O2, O4=O4, C=C, coupler=coupler, side=side)


def _two():
    m = Mechanism()
    A = _fourbar(m, 0.0, "A")
    B = _fourbar(m, 10.0, "B")
    m.drive(A["p"], 0.0, 2 * math.pi)                         # one turn
    m.add_driver(B["p"], math.pi / 2, math.pi / 2 + 4 * math.pi)  # 2x, +90 deg
    return m, A, B


def _ref(f, th):
    Bp = f["O2"] + np.array([math.cos(th), 0, math.sin(th)])
    return _cut(f["O4"], 3.0, Bp, 3.5, f["side"])


def test_two_motors_speed_and_phase():
    m, A, B = _two()
    assert len(m.drivers) == 2 and m.driver[0] is A["p"]
    assert solver.dof(m) == 2                                # two free cranks
    sw = solver.sweep(m, 25)
    for s in sw:
        assert s["ok"], s
        t = s["t"]
        thA, thB = 2 * math.pi * t, math.pi / 2 + 4 * math.pi * t
        for f, th in ((A, thA), (B, thB)):
            C = f["coupler"].world_point(m.pose(s["x"], f["coupler"]), f["C"])
            assert np.linalg.norm(C - _ref(f, th)) < 1e-8
        assert abs(m.continuous_value(B["p"], s["x"]) - thB) < 1e-8


def test_targets_follow_the_leading_motor():
    m, A, B = _two()
    tg = m.driver_targets(math.pi)                           # A at half way
    assert abs(tg[0] - math.pi) < 1e-15 and abs(tg[1] - (math.pi / 2 + 2 * math.pi)) < 1e-15


def test_single_motor_api_unchanged():
    m, A, B = _two()
    m.drive(A["p"], 0, 1.0)
    assert len(m.drivers) == 1 and m.driver == (A["p"], 0.0, 1.0)
    m.add_driver(A["p"], 0, 2.0)                             # same joint: replaced
    assert len(m.drivers) == 1 and m.driver[2] == 2.0


def test_document_keeps_several_motors():
    d = doc.empty()
    d, f = doc.add_body(d, ["g0"], True)
    d, a = doc.add_body(d, ["g1"], False)
    d, b = doc.add_body(d, ["g2"], False)
    d, ja = doc.add_joint(d, "pivot", f, a, point=(0, 0, 0), axis=(0, 1, 0))
    d, jb = doc.add_joint(d, "pivot", f, b, point=(5, 0, 0), axis=(0, 1, 0))
    d = doc.set_driver(d, ja, 0, 1.0)
    d = doc.add_driver(d, jb, 0.5, 2.5)
    assert [x["joint"] for x in d["drivers"]] == [ja, jb]
    d = doc.add_driver(d, jb, 0, 3.0)                        # updated, not doubled
    assert [x["hi"] for x in d["drivers"]] == [1.0, 3.0]
    corners = lambda c: [np.array(c) + np.array(s) * 0.1 for s in
                         ((-1, -1, -1), (1, 1, 1), (1, -1, 1), (-1, 1, -1))]
    pts = {"g0": corners((0, 0, -1)), "g1": corners((1, 0, 0)), "g2": corners((6, 0, 0))}
    m, problems = doc.build(d, pts.get)
    assert not problems and len(m.drivers) == 2
    d = doc.remove_driver(d, ja)
    assert [x["joint"] for x in d["drivers"]] == [jb]
    d = doc.remove(d, jb)
    assert d["drivers"] == [] and d["driver"] is None
    old = {"bodies": [], "joints": [], "driver": {"joint": "x", "lo": 0, "hi": 1}}
    assert doc.normalized(old)["drivers"] == [{"joint": "x", "lo": 0, "hi": 1}]


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
