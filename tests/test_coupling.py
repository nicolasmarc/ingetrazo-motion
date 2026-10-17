"""Coupling joint: gears, rolling without slip, against exact ratios."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import solver, collide

CCW = (0, -1.0, 0)


def _gears(z1=12, z2=30, m=0.002):
    mech = Mechanism()
    d = m * (z1 + z2) / 2
    g1 = mech.body("pignon", (0, 0, 0), [Box((0, 0, 0), (m * z1 / 2, 0.005, m * z1 / 2))])
    g2 = mech.body("roue", (d, 0, 0), [Box((d, 0, 0), (m * z2 / 2, 0.005, m * z2 / 2))])
    p1 = mech.pivot(mech.ground, g1, (0, 0, 0), CCW, name="axe1")
    p2 = mech.pivot(mech.ground, g2, (d, 0, 0), CCW, name="axe2")
    mech.couple(p1, p2, -z1 / z2, name="engrènement")
    mech.drive(p1, 0, 3 * 2 * math.pi)                 # three turns
    return mech, p1, p2, z1, z2


def test_gear_ratio_over_several_turns():
    mech, p1, p2, z1, z2 = _gears()
    assert mech.multi_turn()
    assert solver.dof(mech) == 1
    sw = solver.sweep(mech, 13)                        # 90 deg a step
    for s in sw:
        assert s["ok"], s
        th = s["value"]
        assert abs(mech.continuous_value(p1, s["x"]) - th) < 1e-9
        assert abs(mech.continuous_value(p2, s["x"]) + th * z1 / z2) < 1e-9
        w = -th * z1 / z2                              # wrapped value agrees
        assert abs(math.remainder(mech.joint_value(p2, s["x"]) - w, 2 * math.pi)) < 1e-9


def _rolling(R=0.25):
    mech = Mechanism()
    car = mech.body("chariot", (0, 0, R), [Box((0, 0, R + 0.1), (0.3, 0.2, 0.02))])
    wheel = mech.body("roue", (0, 0, R), [Box((0, 0, R), (R * 0.7, 0.02, R * 0.7))])
    s = mech.slider(mech.ground, car, (0, 0, R), (1, 0, 0), name="voie")
    p = mech.pivot(car, wheel, (0, 0, R), CCW, name="essieu")
    mech.couple(p, s, -R, name="roulement")          # rolls towards +x
    mech.drive(s, 0.0, 4 * math.pi * R)              # two turns of the wheel
    return mech, wheel, s, p, R


def test_rolling_without_slip():
    mech, wheel, s, p, R = _rolling()
    assert solver.dof(mech) == 1
    sw = solver.sweep(mech, 17)
    for st in sw:
        assert st["ok"]
        x = st["x"]
        assert abs(mech.continuous_value(p, x) + st["value"] / R) < 1e-9
        # the wheel's material point now touching the rail has no speed
        x2, ok, _ = solver.solve(mech, st["value"] + 1e-6, x)
        bottom = np.array([st["value"], 0, 0.0])
        mat = wheel.local_point(mech.pose(x, wheel), bottom)
        moved = wheel.world_point(mech.pose(x2, wheel), mat) - bottom
        assert np.linalg.norm(moved) < 1e-10, moved        # slipping: ~1e-6


def test_rolling_the_wrong_way_slips():
    mech, wheel, s, p, R = _rolling()
    mech.joints[-1].ratio = +R                             # wrong sign
    x, _, _ = solver.solve(mech, 0.1)
    x2, _, _ = solver.solve(mech, 0.1 + 1e-6, x)
    bottom = np.array([0.1, 0, 0.0])
    mat = wheel.local_point(mech.pose(x, wheel), bottom)
    moved = wheel.world_point(mech.pose(x2, wheel), mat) - bottom
    assert abs(np.linalg.norm(moved) - 2e-6) < 1e-8


def test_coupling_does_not_join_parts():
    mech, *_ = _gears()
    assert collide._joined(mech) and all(
        "coupling" not in {j.kind for j in js} for js in collide._joined(mech).values())


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
