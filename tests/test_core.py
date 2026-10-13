import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import solver, collide


def _drop_leaf():
    m = Mechanism()
    leaf = m.body("leaf", center=(0.3, 0, 0),
                  boxes=[Box((0.3, 0, 0), (0.3, 0.2, 0.009))])
    m.ground.boxes = [Box((-0.2, 0, 0), (0.2, 0.2, 0.009))]
    hinge = m.pivot(m.ground, leaf, (0, 0, 0), (0, 1, 0),
                    limits=(0, math.pi / 2))
    m.drive(hinge, 0, math.pi / 2)
    return m, leaf, hinge


def test_dropleaf_tip_follows_circle():
    m, leaf, hinge = _drop_leaf()
    tip = np.array([0.6, 0, 0])
    for s in solver.sweep(m, 19):
        ang = s["value"]
        p = leaf.world_point(m.pose(s["x"], leaf), tip)
        assert s["ok"]
        assert abs(p[0] - 0.6 * math.cos(ang)) < 1e-8
        assert abs(p[2] + 0.6 * math.sin(ang)) < 1e-8 or \
            abs(p[2] - 0.6 * math.sin(ang)) < 1e-8
        assert abs(math.hypot(p[0], p[2]) - 0.6) < 1e-8   # radius kept


def test_dropleaf_dof_is_one():
    m, *_ = _drop_leaf()
    assert solver.dof(m) == 1


def test_drawer_slider():
    m = Mechanism()
    d = m.body("drawer", center=(0, 0.25, 0),
               boxes=[Box((0, 0.25, 0), (0.2, 0.25, 0.08))])
    s = m.slider(m.ground, d, (0, 0, 0), (0, 1, 0), limits=(0, 0.4))
    m.drive(s, 0, 0.4)
    assert solver.dof(m) == 1
    for st in solver.sweep(m, 11):
        assert st["ok"]
        p = d.world_point(m.pose(st["x"], d), np.array([0, 0.5, 0]))
        assert abs(p[1] - (0.5 + st["value"])) < 1e-9
        assert abs(p[0]) < 1e-9 and abs(p[2]) < 1e-9


def _fourbar():
    m = Mechanism()
    # ground pivots O2=(0,0) O4=(4,0); crank 1, coupler 3.5, rocker 3
    crank = m.body("crank", (0.5, 0, 0), [Box((0.5, 0, 0), (0.5, .05, .05))])
    coupler = m.body("coupler", (1.0, 1.75, 0), [Box((1.0, 1.75, 0), (.05, .05, .05))])
    rocker = m.body("rocker", (3.5, 0.5, 0), [Box((3.5, 0.5, 0), (.05, .05, .05))])
    # reference config: crank along +x at angle 0 -> B=(1,0); solve C
    # coupler 3.5 from B, rocker 3 from O4=(4,0): C = (4 - 3cos a, 3 sin a)
    # |C-B|=3.5 -> choose a so it holds
    a = math.acos((3**2 + 3**2 - 3.5**2) / (2 * 3 * 3)) if False else None
    # solve circle intersection numerically for the drawn pose
    B = np.array([1.0, 0]); O4 = np.array([4.0, 0])
    d = np.linalg.norm(O4 - B)
    aa = (3.5**2 - 3.0**2 + d**2) / (2 * d)
    h = math.sqrt(3.5**2 - aa**2)
    C = B + aa * (O4 - B) / d + h * np.array([0, 1.0])
    m.pivot(m.ground, crank, (0, 0, 0), (0, 0, 1), name="O2")
    m.pivot(crank, coupler, (B[0], B[1], 0), (0, 0, 1), name="B")
    m.pivot(coupler, rocker, (C[0], C[1], 0), (0, 0, 1), name="C")
    m.pivot(rocker, m.ground, (4, 0, 0), (0, 0, 1), name="O4")
    m.drive(m.joints[0], 0, math.radians(60))
    return m, coupler, rocker, B, C


def test_fourbar_closed_loop():
    m, coupler, rocker, B, C = _fourbar()
    assert solver.dof(m) == 1
    for st in solver.sweep(m, 13):
        assert st["ok"], st
        th = st["value"]
        Bw = np.array([math.cos(th), math.sin(th), 0.0])
        # analytic: C on circle(B,3.5) ∩ circle(O4,3)
        Cw = coupler.world_point(m.pose(st["x"], coupler),
                                 np.array([C[0], C[1], 0.0]))
        assert abs(np.linalg.norm(Cw - Bw) - 3.5) < 1e-8     # bar lengths kept
        assert abs(np.linalg.norm(Cw - np.array([4, 0, 0])) - 3.0) < 1e-8
        Cr = rocker.world_point(m.pose(st["x"], rocker),
                                np.array([C[0], C[1], 0.0]))
        assert np.linalg.norm(Cw - Cr) < 1e-8                 # joint closed


def test_interference_blocks_drawer_at_stop():
    m = Mechanism()
    d = m.body("drawer", (0, 0.25, 0), [Box((0, 0.25, 0), (0.2, 0.25, 0.08))])
    j = m.slider(m.ground, d, (0, 0, 0), (0, 1, 0), limits=(0, 0.4))
    # shelf board in front of the cabinet, 0.30 in front of the drawer face
    m.ground.boxes = [Box((0, 0.8, 0), (0.3, 0.05, 0.3))]
    m.drive(j, 0, 0.4)
    first = None
    for st in solver.sweep(m, 41):
        if collide.interferences(m, st["x"], ignore_joined=False) and first is None:
            first = st["value"]
    # drawer front is at y=0.5; board starts at y=0.75 -> hit after 0.25 travel
    assert first is not None and abs(first - 0.25) <= 0.4 / 40 + 1e-9


def test_limit_violation_reported():
    m, leaf, hinge = _drop_leaf()
    x, ok, _ = solver.solve(m, math.radians(100))
    assert ok
    v = collide.limit_violations(m, x)
    assert v and v[0][0] == hinge.name


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
