"""Linéaire annulaire, ponctuelle, appui plan and hélicoïdale (vis-écrou),
kinematics and dynamics, against cases worked by hand."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import dynamics as D, solver
from test_joints import block

G = 9.81


def test_degrees_of_freedom():
    for make, dof in ((lambda m, b: m.annular(m.ground, b, (0, 0, 0), (1, 0, 0)), 4),
                      (lambda m, b: m.point_plane(m.ground, b, (0, 0, 0), (0, 0, 1)), 5),
                      (lambda m, b: m.planar(m.ground, b, (0, 0, 0), (0, 0, 1)), 3),
                      (lambda m, b: m.screw(m.ground, b, (0, 0, 0), (0, 0, 1), 0.002), 1)):
        m = Mechanism()
        b = block(m, "pièce", (0.1, 0.02, 0.03), 1.0)
        make(m, b)
        assert solver.dof(m) == dof, (m.joints[0].kind, solver.dof(m))


def slope(alpha):
    return np.array([math.cos(alpha), 0, -math.sin(alpha)])


def test_annular_ball_slides_down_its_bore_and_spins_freely():
    # a ball whose centre is held on a sloping line: it slides like a bead
    # on a wire, g sin(alpha), whatever it turns about
    a = math.radians(25)
    m = Mechanism()
    b = block(m, "rotule", (0, 0, 0), 0.3)
    m.annular(m.ground, b, (0, 0, 0), slope(a))
    dyn = D.Dynamics(m)
    w = np.array([2.0, -1.0, 4.0])
    rows = dyn.run(0.6, dt=0.004, u0=np.concatenate([[0, 0, 0], w]), every=15)
    for r in rows:
        c = dyn.cog(r["x"], b)
        s = 0.5 * G * math.sin(a) * r["t"] ** 2
        assert np.linalg.norm(c - s * slope(a)) < 1e-8, (r["t"], c)
        assert np.allclose(r["u"][3:], w, atol=1e-8)          # a cube: spins on


def test_point_on_a_plane_slides_and_drifts():
    # a point on an inclined plane: g sin(alpha) down the slope, any
    # sideways speed it had stays
    a = math.radians(40)
    n = np.array([math.sin(a), 0, math.cos(a)])
    m = Mechanism()
    b = block(m, "palpeur", (0, 0, 0), 0.5)
    m.point_plane(m.ground, b, (0, 0, 0), n)
    dyn = D.Dynamics(m)
    rows = dyn.run(0.5, dt=0.004, u0=np.array([0, 0.3, 0, 0, 0, 0]), every=25)
    for r in rows:
        t = r["t"]
        c = dyn.cog(r["x"], b)
        want = 0.5 * G * math.sin(a) * t * t * slope(a) + np.array([0, 0.3 * t, 0])
        assert np.linalg.norm(c - want) < 1e-8, (t, c, want)


def test_planar_slides_down_the_plane_and_turns_about_its_normal():
    a = math.radians(20)
    n = np.array([math.sin(a), 0, math.cos(a)])
    m = Mechanism()
    b = block(m, "patin", (0, 0, 0), 1.2)
    m.planar(m.ground, b, (0, 0, 0), n)
    dyn = D.Dynamics(m)
    rows = dyn.run(0.5, dt=0.004, u0=np.concatenate([[0, 0, 0], 3.0 * n]), every=25)
    for r in rows:
        t = r["t"]
        c = dyn.cog(r["x"], b)
        assert np.linalg.norm(c - 0.5 * G * math.sin(a) * t * t * slope(a)) < 1e-8
        assert np.allclose(r["u"][3:], 3.0 * n, atol=1e-8)


def test_screw_nut_falls_slowed_by_its_spin():
    # a nut on a vertical screw: z = p theta / 2 pi, so
    # (m + I k^2) z'' = -m g with k = 2 pi / p
    mass, I, p = 0.2, 4e-5, 0.004
    m = Mechanism()
    b = block(m, "écrou", (0, 0, 0), mass, inertia=(I, I, I))
    j = m.screw(m.ground, b, (0, 0, 0), (0, 0, 1), p)
    dyn = D.Dynamics(m)
    k = 2 * math.pi / p
    acc = mass * G / (mass + I * k * k)
    rows = dyn.run(0.5, dt=0.002, every=25)
    for r in rows:
        z = dyn.cog(r["x"], b)[2]
        assert abs(z + 0.5 * acc * r["t"] ** 2) < 1e-8, (r["t"], z)
        assert abs(m.continuous_value(j, r["x"]) - z * k) < 1e-6


def test_screw_driven_lifts_and_the_motor_pays_for_it():
    # a lead screw turned at w lifts a carriage p w / 2 pi; the torque is
    # the weight's m g p / 2 pi (ideal screw, no friction)
    mass, p, w = 3.0, 0.005, 4 * math.pi
    m = Mechanism()
    b = block(m, "chariot", (0, 0, 0), mass)
    j = m.screw(m.ground, b, (0, 0, 0), (0, 0, 1), p)
    dyn = D.Dynamics(m, motors={j.name: ("vitesse", lambda t: w * t)})
    rows = dyn.run(1.0, dt=0.005, every=20)
    for r in rows[1:-1]:
        assert abs(m.slide_value(j, r["x"]) - p * w * r["t"] / (2 * math.pi)) < 1e-9
        tq = dyn.reactions(r["lam"])[j.name + " (moteur)"][0]
        assert abs(tq - mass * G * p / (2 * math.pi)) < 1e-6, tq


def test_eccentric_drives_a_leg_through_a_ball_in_a_bore():
    # the spider's leg drive in miniature: a wheel turning about x carries
    # a bore (along x) e off its centre; the leg hangs from the frame at H
    # by a yaw pivot (z) and a pitch pivot (y); a ball on the leg, D from
    # H, sits in the bore (linéaire annulaire). The ball goes round with
    # the bore in y, z and slides along x: the leg's yaw follows
    # tan(psi) = -y / -(x - D) with x from |ball - H| = D.
    e, D = 0.007, 0.03
    H = np.array([D, 0, -e])
    m = Mechanism()
    wheel = block(m, "roue", (0, 0, 0), 0.05)
    chape = block(m, "pivot", H, 0.01)
    leg = block(m, "patte", H + (0.01, 0, 0), 0.02)
    jw = m.pivot(m.ground, wheel, (0, 0, 0), (1, 0, 0))
    jy = m.pivot(m.ground, chape, H, (0, 0, 1))
    m.pivot(chape, leg, H, (0, 1, 0))
    jb = m.annular(wheel, leg, (0, 0, -e), (1, 0, 0))
    m.drive(jw, 0, 2 * math.pi)
    assert solver.dof(m) == 1
    x = np.zeros(m.n)
    for th in np.linspace(0, 2 * math.pi, 25)[1:]:
        x, ok, _ = solver.solve(m, th, x)
        assert ok and np.abs(solver._joint_residual(m, jb, x)).max() < 1e-9
        y, z = e * math.sin(th), -e * math.cos(th)          # the bore, turned about x
        vx = -math.sqrt(D * D - y * y - (z + e) ** 2)
        psi = math.atan2(-y, -vx)
        assert abs(m.joint_value(jy, x) - psi) < 1e-7, (th, m.joint_value(jy, x), psi)


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
