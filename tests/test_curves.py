"""The curves (motion.curves) against cases worked by hand: the force in
the pivot of an arm hanging at rest is its weight, a swinging pendulum
keeps its total energy, and the joint's angle follows the motor's travel."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
from motion import curves, dynamics as D, solver
from motion.model import Mechanism
from test_dynamics import rod, G


def _poses(dyn, rows):
    """The rows as the panel keeps them (see Panel.simulation)."""
    n = len(rows)
    return [dict(t=i / max(n - 1, 1), value=r["t"], x=r["x"], ke=r["ke"], pe=r["pe"],
                 couples={}, reac={k: list(v) for k, v in dyn.reactions(r["lam"]).items()}
                 if "lam" in r else {})
            for i, r in enumerate(rows)]


def _arm(start_deg):
    L, mass = 0.5, 2.0
    m = Mechanism()
    a = math.radians(start_deg)                        # from hanging straight down
    tip = (L * math.sin(a), 0, -L * math.cos(a))
    b = rod(m, "bras", (0, 0, 0), tip, mass)
    j = m.pivot(m.ground, b, (0, 0, 0), (0, 1, 0))
    return m, b, j, mass


def test_force_in_the_pivot_of_a_hanging_arm_is_its_weight():
    m, b, j, mass = _arm(0.0)
    dyn = D.Dynamics(m)
    rows = _poses(dyn, dyn.run(0.5, dt=0.005, every=10))
    assert ("effort", "Effort dans la liaison (N)") in curves.quantities(m, j.name, True)
    xs, ys, xlab = curves.series(m, rows, "effort", j.name, True)
    f = ys[0][1]
    assert xlab == "temps (s)" and abs(xs[-1] - 0.5) < 1e-9
    ok = np.isfinite(f)
    assert ok.sum() >= len(f) - 1
    assert np.allclose(f[ok], mass * G, rtol=1e-3), f[ok]


def test_swinging_pendulum_keeps_its_energy_and_its_height_moves():
    m, b, j, mass = _arm(60.0)
    dyn = D.Dynamics(m)
    rows = _poses(dyn, dyn.run(1.0, dt=0.002, every=10))
    xs, ys, _ = curves.series(m, rows, "energie", None, True)
    names = [lab for lab, _ in ys]
    assert names == ["cinétique", "potentielle", "totale"]
    ke, pe, tot = (y for _, y in ys)
    assert pe.min() == 0.0 and ke.max() > 0.1
    assert np.ptp(tot) < 1e-3 * tot.max(), np.ptp(tot)
    _, h, _ = curves.series(m, rows, "hauteur", "bras", True)
    hz = h[0][1]
    assert abs(hz[0] - (-250 * math.cos(math.radians(60)))) < 1e-6      # mm
    assert hz.min() < hz[0] - 50                                          # it swings down
    # the angle's speed is the derivative of the angle
    _, v, _ = curves.series(m, rows, "vitesse", j.name, True)
    assert abs(v[0][1]).max() > 50                                        # °/s


def test_kinematic_curve_follows_the_motor():
    m, b, j, mass = _arm(0.0)
    m.drive(j, 0.0, math.radians(90))
    rows = solver.sweep(m, 11)
    qs = dict(curves.quantities(m, j.name, False))
    assert set(qs) == {"valeur"}                         # no forces without dynamics
    xs, ys, xlab = curves.series(m, rows, "valeur", j.name, False)
    assert xlab == "course du moteur (%)"
    assert np.allclose(xs, np.linspace(0, 100, 11))
    assert np.allclose(np.abs(ys[0][1]), np.linspace(0, 90, 11), atol=1e-6)


def test_ticks_and_csv():
    assert curves.nice_ticks(0, 9.3) == [0, 2, 4, 6, 8]
    assert curves.nice_ticks(-0.013, 0.012)[0] == -0.01
    text = curves.to_csv(np.array([0, 0.5]), [("a", np.array([1.0, 2.0])),
                                             ("b", np.array([3.0, np.nan]))], "temps (s)", "E (J)")
    assert text.splitlines() == ["temps (s);E (J) a;E (J) b", "0;1;3", "0.5;2;nan"]
