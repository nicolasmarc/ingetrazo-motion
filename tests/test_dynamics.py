"""Rigid-body dynamics (motion.dynamics) against cases worked by hand:
free fall and a tumbling body, pendulum periods (small and large swing,
elliptic integral), a double pendulum's energy, a block sliding down a
slope, a flywheel under a torque, geared inertia, the torque a motor must
give to hold an arm, and a four-bar's power balance."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import dynamics as D

G = 9.81


def rod(m, name, p, q, mass, w=0.02):
    """A uniform rod body from p to q: mass, centre and inertia set."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    c = 0.5 * (p + q)
    L = float(np.linalg.norm(q - p))
    d = (q - p) / L
    b = m.body(name, c, [Box(c, (L / 2, w / 2, w / 2))])
    b.mass, b.com = mass, c
    b.inertia = mass * L * L / 12 * (np.eye(3) - np.outer(d, d))
    return b


def agm(a, b):
    for _ in range(60):
        a, b = 0.5 * (a + b), math.sqrt(a * b)
    return a


def test_free_fall_is_exact():
    m = Mechanism()
    b = rod(m, "caillou", (0, 0, 1), (0.1, 0, 1), 1.0)
    dyn = D.Dynamics(m)
    u0 = np.array([0.3, 0.0, 2.0, 0, 0, 0])
    rows = dyn.run(0.6, dt=0.01, u0=u0)
    for r in rows[::10]:
        t = r["t"]
        c = dyn.cog(r["x"], b)
        assert np.allclose(c, [0.05 + 0.3 * t, 0, 1 + 2.0 * t - 0.5 * G * t * t], atol=1e-9), (t, c)


def test_tumbling_body_keeps_its_angular_momentum():
    # an asymmetric block spinning about a tilted axis, no gravity: the
    # angular momentum (world) and the kinetic energy stay put
    m = Mechanism()
    b = m.body("pavé", (0, 0, 0), [Box((0, 0, 0), (0.1, 0.05, 0.02))])
    b.mass, b.com = 3.0, np.zeros(3)
    b.inertia = np.diag([3.0 / 12 * (0.1 ** 2 + 0.04 ** 2) * 1, 3.0 / 12 * (0.04 + 0.04 ** 2),
                         3.0 / 12 * (0.04 + 0.01)])
    dyn = D.Dynamics(m, gravity=(0, 0, 0))
    u0 = np.array([0, 0, 0, 3.0, 0.2, 1.0])
    rows = dyn.run(1.0, dt=0.002, u0=u0, every=50)

    def L(r):
        R = dyn._R(r["x"], b)
        return R @ b.inertia @ R.T @ r["u"][3:]
    L0 = L(rows[0])
    for r in rows:
        assert np.linalg.norm(L(r) - L0) < 1e-5 * np.linalg.norm(L0)
        assert abs(r["ke"] - rows[0]["ke"]) < 1e-5 * rows[0]["ke"]


def pendulum(theta0):
    m = Mechanism()
    th = math.radians(theta0)
    tip = 0.5 * np.array([math.sin(th), 0, -math.cos(th)])
    b = rod(m, "balancier", (0, 0, 0), tip, 2.0)
    m.pivot(m.ground, b, (0, 0, 0), (0, 1, 0))
    return m, b


def period(dyn, b, rows):
    xs = np.array([dyn.cog(r["x"], b)[0] for r in rows])
    ts = np.array([r["t"] for r in rows])
    cr = [ts[i] - xs[i] * (ts[i + 1] - ts[i]) / (xs[i + 1] - xs[i])
          for i in range(len(xs) - 1) if (xs[i] > 0) != (xs[i + 1] > 0)]
    return 2 * float(np.mean(np.diff(cr)))


def test_pendulum_periods_small_and_large_swing():
    L, mass = 0.5, 2.0
    for theta0 in (5.0, 90.0, 150.0):
        m, b = pendulum(theta0)
        dyn = D.Dynamics(m)
        T0 = 2 * math.pi * math.sqrt((mass * L * L / 3) / (mass * G * L / 2))
        k = math.sin(math.radians(theta0) / 2)
        T = T0 / agm(1.0, math.sqrt(1 - k * k))          # 2 K(k) / pi = 1 / agm
        rows = dyn.run(2.2 * T, dt=0.004)
        assert abs(period(dyn, b, rows) / T - 1) < 2e-4, theta0
        E = [r["ke"] + r["pe"] for r in rows]
        assert max(E) - min(E) < 1e-4 * mass * G * L


def test_double_pendulum_keeps_its_energy():
    m = Mechanism()
    a = rod(m, "bras 1", (0, 0, 0), (0.3, 0, 0), 1.0)
    b = rod(m, "bras 2", (0.3, 0, 0), (0.3, 0, 0.25), 0.6)
    m.pivot(m.ground, a, (0, 0, 0), (0, 1, 0))
    m.pivot(a, b, (0.3, 0, 0), (0, 1, 0))
    dyn = D.Dynamics(m)
    rows = dyn.run(2.0, dt=0.002, every=25)
    E = np.array([r["ke"] + r["pe"] for r in rows])
    assert np.ptp(E) < 2e-4 * (1.0 + 0.6) * G * 0.3
    assert max(r["ke"] for r in rows) > 1.0                  # it did swing hard


def test_block_slides_down_a_slope():
    alpha = math.radians(30)
    d = np.array([math.cos(alpha), 0, -math.sin(alpha)])
    m = Mechanism()
    b = m.body("bloc", (0, 0, 0), [Box((0, 0, 0), (0.05, 0.05, 0.05))])
    b.mass, b.com, b.inertia = 1.5, np.zeros(3), 1.5 * 0.01 / 6 * np.eye(3)
    m.slider(m.ground, b, (0, 0, 0), d)
    dyn = D.Dynamics(m)
    rows = dyn.run(0.8, dt=0.005, every=20)
    for r in rows:
        s = dyn.cog(r["x"], b) @ d
        assert abs(s - 0.5 * G * math.sin(alpha) * r["t"] ** 2) < 1e-8


def test_flywheel_under_a_torque_and_geared_inertia():
    # disc on a vertical axis (weight does not turn it), constant torque
    m = Mechanism()
    r1, r2, mass1, mass2 = 0.1, 0.2, 2.0, 3.0
    w1 = m.body("volant", (0, 0, 0), [Box((0, 0, 0), (r1, r1, 0.01))])
    w1.mass, w1.com, w1.inertia = mass1, np.zeros(3), np.diag([1, 1, 2]) * mass1 * r1 * r1 / 4
    w2 = m.body("roue", (0.3, 0, 0), [Box((0.3, 0, 0), (r2, r2, 0.01))])
    w2.mass, w2.com, w2.inertia = mass2, np.array([0.3, 0, 0]), np.diag([1, 1, 2]) * mass2 * r2 * r2 / 4
    j1 = m.pivot(m.ground, w1, (0, 0, 0), (0, 0, 1))
    j2 = m.pivot(m.ground, w2, (0.3, 0, 0), (0, 0, 1))
    tau = 0.8
    I1, I2 = mass1 * r1 * r1 / 2, mass2 * r2 * r2 / 2
    # alone
    dyn = D.Dynamics(m, motors={j1.name: ("couple", lambda t: tau)})
    rows = dyn.run(0.5, dt=0.005, every=20)
    for r in rows:
        assert abs(m.continuous_value(j1, r["x"]) - 0.5 * tau / I1 * r["t"] ** 2) < 1e-8
    # geared 1 : -1/2 (the wheel turns half as fast, the other way)
    m.couple(j1, j2, -0.5)
    dyn = D.Dynamics(m, motors={j1.name: ("couple", lambda t: tau)})
    rows = dyn.run(0.5, dt=0.005, every=20)
    alpha = tau / (I1 + 0.25 * I2)
    for r in rows:
        assert abs(m.continuous_value(j1, r["x"]) - 0.5 * alpha * r["t"] ** 2) < 1e-5


def test_motor_torque_holding_an_arm():
    # a horizontal arm raised slowly by a speed motor: the torque it gives
    # is the weight's moment m g d cos(theta) (plus I alpha = 0)
    L, mass, w = 0.6, 1.2, 0.2                               # rad/s
    m = Mechanism()
    b = rod(m, "bras", (0, 0, 0), (L, 0, 0), mass)
    j = m.pivot(m.ground, b, (0, 0, 0), (0, -1, 0))          # + lifts the tip
    dyn = D.Dynamics(m, motors={j.name: ("vitesse", lambda t: w * t)})
    rows = dyn.run(2.0, dt=0.01, every=10)
    for r in rows[1:-1]:
        th = w * r["t"]
        tq = dyn.reactions(r["lam"])[j.name + " (moteur)"][0]   # torque the motor gives
        assert abs(tq - mass * G * L / 2 * math.cos(th)) < 1e-4, (th, tq)


def test_four_bar_power_balance():
    # crank driven at constant speed: motor torque x speed = d(E)/dt
    A, B, C, Dd = (np.array(p, float) for p in ((0, 0, 0), (0, 0, 0.06), (0.2, 0, 0.09), (0.2, 0, 0)))
    m = Mechanism()
    crank = rod(m, "manivelle", A, B, 0.2)
    coupler = rod(m, "bielle", B, C, 0.5)
    rocker = rod(m, "balancier", Dd, C, 0.3)
    jA = m.pivot(m.ground, crank, A, (0, -1, 0))
    m.pivot(crank, coupler, B, (0, -1, 0))
    m.pivot(coupler, rocker, C, (0, -1, 0))
    m.pivot(m.ground, rocker, Dd, (0, -1, 0))
    w = 2 * math.pi                                         # one turn per second
    dyn = D.Dynamics(m, motors={jA.name: ("vitesse", lambda t: w * t)})
    dt = 0.002
    rows = dyn.run(1.0, dt=dt)
    E = np.array([r["ke"] + r["pe"] for r in rows])
    P = np.array([dyn.reactions(r["lam"])[jA.name + " (moteur)"][0] * w if "lam" in r else np.nan
                  for r in rows])
    dE = np.gradient(E, dt)
    ok = ~np.isnan(P)
    ok[:3] = ok[-3:] = False
    err = np.abs(P[ok] - dE[ok]).max() / np.abs(dE[ok]).max()
    assert err < 0.01, err


def test_double_pendulum_example_loses_energy_only_to_friction():
    from motion import samples
    m, info = samples.pendule_double(duree=2.0)
    sim = info["simulation"]
    rows = D.Dynamics(m, **sim["params"]).run(2.0, dt=sim["dt"], every=10)
    E = np.array([r["ke"] + r["pe"] for r in rows])
    assert np.all(np.diff(E) < 1e-9)                       # friction only takes
    m, info = samples.pendule_double(duree=2.0, amortissement=0.0)
    rows = D.Dynamics(m, **info["simulation"]["params"]).run(2.0, dt=info["simulation"]["dt"],
                                                            every=10)
    E = np.array([r["ke"] + r["pe"] for r in rows])
    assert np.ptp(E) < 1e-3 * abs(E[0])                    # none without it
    assert all(b.mass > 0.05 for b in m.bodies)            # oak arms, from their volume


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
