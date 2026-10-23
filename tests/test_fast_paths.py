"""The faster paths give the same numbers as the plain ones they replace."""
import math
import numpy as np

from motion import dynamics, samples
from motion.dynamics import Dynamics, _unwrap, log_rotation, log_rotations, unwraps
from motion.model import Mechanism, rotations


def test_rotation_logs_and_unwraps_match_one_by_one():
    rng = np.random.default_rng(1)
    for _ in range(50):
        W = rng.normal(size=(40, 3)) * rng.choice([0, 1e-13, 1e-6, 0.5, 3.14159, 9.0], size=(40, 1))
        R = rotations(W)
        assert np.allclose(log_rotations(R), [log_rotation(r) for r in R], atol=1e-14)
        near = W + rng.normal(size=(40, 3)) * rng.choice([0, 1e-13, 1, 7], size=(40, 1))
        V = log_rotations(R)
        assert np.allclose(unwraps(V, near), [_unwrap(v, n) for v, n in zip(V, near)], atol=1e-12)


def test_all_pivots_at_once_match_one_by_one():
    for build in (samples.pendule_double, samples.chaise_pliante, samples.oiseau):
        m, _ = build()
        for b in m.bodies:
            if b.mass is None:
                b.mass, b.com, b.inertia = 1.0, b.center.copy(), np.eye(3) * 0.01
        d = Dynamics(m)
        idx = {id(b): k for k, b in enumerate(d.bodies)}
        rng = np.random.default_rng(0)
        for _ in range(10):
            x, u = rng.normal(size=m.n) * 0.3, rng.normal(size=d.n)
            piv = d._pivots_batch(x, u, idx)
            for j, n, drv in d._rows():
                if j.kind == "pivot" and not drv:
                    G1, G2 = np.zeros((n, d.n)), np.zeros((n, d.n))
                    C1, g1 = d._pivot_rows(j, x, u, G1, idx)
                    C2, g2 = d._pivot_fill(piv, j, G2)
                    assert np.allclose(C1, C2, atol=1e-14) and np.allclose(g1, g2, atol=1e-12)
                    assert np.allclose(G1, G2, atol=1e-14)


def test_semi_implicit_step_exchanges_ball_speeds():
    """The shocks' integrator (semi-implicit Euler, 0.1 ms) on the head-on
    balls: the struck ball leaves at (1 + e)/2 of the speed, within 0.5 %."""
    from test_impacts import _ball
    m = Mechanism()
    r, e = 0.0125, 0.97
    _ball(m, "A", (0, 0, 0), r, e=e)
    _ball(m, "B", (2 * r + 0.002, 0, 0), r, e=e)
    u0 = np.zeros(12)
    u0[0] = 0.5
    dyn = Dynamics(m, gravity=(0, 0, 0), contacts=dict(dt=1e-4))
    dyn.method = "symplectic"
    rows = dyn.run(0.03, dt=1e-4, u0=u0, every=300)
    vb = rows[-1]["u"][6]
    assert abs(vb / (0.5 * (1 + e) / 2) - 1) < 0.005, vb


def test_block_inverse_mass_matches_the_dense_one():
    m, _ = samples.pendule_double()
    d = Dynamics(m)
    rng = np.random.default_rng(3)
    x, u = rng.normal(size=m.n) * 0.2, rng.normal(size=d.n)
    a, lam, G = d.accelerations(x, u, 0.0)
    M, F = d.M_and_forces(x, u, 0.0)
    Mi = np.linalg.pinv(M, rcond=1e-12)
    A = G @ Mi @ G.T
    C, Gm, gamma = d._kin(x, u, 0.0)
    rhs = -gamma - 2 * d.k * (Gm @ u - d._motor_speeds(0.0)) - d.k ** 2 * C + d.motor_rates(0.0)
    nu = np.linalg.lstsq(A, Gm @ Mi @ F - rhs, rcond=1e-12)[0]
    assert np.allclose(a, Mi @ (F - G.T @ nu), atol=1e-9)
