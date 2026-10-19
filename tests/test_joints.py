"""The other standard joints and the joint modifiers, kinematics and
dynamics, against cases worked by hand: pivot glissant (cylindrical),
rotule (ball: a conical pendulum), encastrement (lock), liaison élastique
(mass on a flexible mount), springs and dampers on joints (torsion and
linear oscillators), stops."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box
from motion import dynamics as D, solver

G = 9.81


def block(m, name, c, mass, half=(0.02, 0.02, 0.02), inertia=None):
    c = np.asarray(c, float)
    b = m.body(name, c, [Box(c, half)])
    h = 2 * np.asarray(half)
    b.mass, b.com = mass, c
    b.inertia = (np.diag(inertia) if inertia is not None else
                 mass / 12 * np.diag([h[1] ** 2 + h[2] ** 2, h[0] ** 2 + h[2] ** 2,
                                      h[0] ** 2 + h[1] ** 2]))
    return b


def rod(m, name, p, q, mass):
    p, q = np.asarray(p, float), np.asarray(q, float)
    c, L = 0.5 * (p + q), float(np.linalg.norm(q - p))
    d = (q - p) / L
    b = m.body(name, c, [Box(c, (0.01, 0.01, 0.01))])
    b.mass, b.com = mass, c
    b.inertia = mass * L * L / 12 * (np.eye(3) - np.outer(d, d)) + 1e-9 * np.eye(3)
    return b


def test_degrees_of_freedom():
    for make, dof in ((lambda m, b: m.cylindrical(m.ground, b, (0, 0, 0), (1, 0, 0)), 2),
                      (lambda m, b: m.ball(m.ground, b, (0, 0, 0)), 3),
                      (lambda m, b: m.lock(m.ground, b), 0),
                      (lambda m, b: m.elastic(m.ground, b, (0, 0, 0)), 0)):
        m = Mechanism()
        b = block(m, "pièce", (0.1, 0, 0), 1.0)
        make(m, b)
        assert solver.dof(m) == dof


def test_cylindrical_falls_along_its_axis_and_keeps_spinning():
    m = Mechanism()
    b = block(m, "manchon", (0, 0, 0), 0.5)
    m.cylindrical(m.ground, b, (0, 0, 0), (0, 0, 1))
    dyn = D.Dynamics(m)
    u0 = np.array([0, 0, 0, 0, 0, 12.0])                     # spinning about the axis
    rows = dyn.run(0.5, dt=0.005, u0=u0, every=20)
    for r in rows:
        t = r["t"]
        assert abs(dyn.cog(r["x"], b)[2] + 0.5 * G * t * t) < 1e-8
        assert np.allclose(r["u"][3:], [0, 0, 12.0], atol=1e-6)
        assert abs(m.slide_value(m.joints[0], r["x"]) + 0.5 * G * t * t) < 1e-8


def test_ball_joint_conical_pendulum():
    # uniform rod hanging from a ball joint, turning steadily at an angle
    # theta from the vertical: w^2 = 3 g / (2 L cos theta)
    L, th = 0.4, math.radians(35)
    tip = L * np.array([math.sin(th), 0, -math.cos(th)])
    m = Mechanism()
    b = rod(m, "tige", (0, 0, 0), tip, 0.8)
    m.ball(m.ground, b, (0, 0, 0))
    dyn = D.Dynamics(m)
    w = math.sqrt(3 * G / (2 * L * math.cos(th)))
    c = dyn.cog(np.zeros(m.n), b)
    u0 = np.concatenate([np.cross([0, 0, w], c), [0, 0, w]])
    rows = dyn.run(2.0, dt=0.002, u0=u0, every=25)
    for r in rows:
        z = dyn.cog(r["x"], b)[2]
        assert abs(z - c[2]) < 2e-4 * L, (r["t"], z, c[2])  # stays at its height
    # and it went round: about 2 s / period turns
    turns = w * 2.0 / (2 * math.pi)
    assert turns > 1.5


def test_lock_makes_one_body():
    m = Mechanism()
    a = block(m, "caisse", (0, 0, 0), 2.0)
    b = block(m, "couvercle", (0, 0, 0.05), 0.5)
    m.lock(a, b)
    dyn = D.Dynamics(m)
    rows = dyn.run(0.4, dt=0.005, u0=np.array([0, 0, 0, 0, 0, 3.0] + [0] * 6), every=20)
    for r in rows:
        # the lid keeps its place on the box, the pair falls freely
        ca, cb = dyn.cog(r["x"], a), dyn.cog(r["x"], b)
        assert abs(np.linalg.norm(cb - ca) - 0.05) < 1e-7
        cm = (2.0 * ca + 0.5 * cb) / 2.5
        assert abs(cm[2] - (0.01 - 0.5 * G * r["t"] ** 2)) < 1e-6


def test_torsion_spring_period():
    I, k = 0.002, 0.5                                         # kg m^2, N m/rad
    m = Mechanism()
    b = block(m, "volant", (0, 0, 0), 1.0, inertia=(I, I, I))
    j = m.pivot(m.ground, b, (0, 0, 0), (0, 0, 1))           # weight along the axis
    j.spring = (k, 0.3)                                       # released 0.3 rad off
    rows = D.Dynamics(m).run(1.2, dt=0.002)
    th = np.array([m.continuous_value(j, r["x"]) - 0.3 for r in rows])
    t = np.array([r["t"] for r in rows])
    assert np.allclose(th, -0.3 * np.cos(math.sqrt(k / I) * t), atol=1e-6)


def test_mass_on_a_vertical_spring():
    mass, k = 0.8, 200.0
    m = Mechanism()
    b = block(m, "masse", (0, 0, 0), mass)
    j = m.slider(m.ground, b, (0, 0, 0), (0, 0, 1))
    j.spring = (k, 0.0)                                       # free length at the drawn pose
    rows = D.Dynamics(m).run(1.0, dt=0.002, every=5)
    w = math.sqrt(k / mass)
    s0 = -mass * G / k                                        # where it would rest
    for r in rows:
        s = m.slide_value(j, r["x"])
        assert abs(s - s0 * (1 - math.cos(w * r["t"]))) < 1e-7


def test_elastic_mount():
    # a mass on a flexible mount: it sags m g / k and bobs at sqrt(k / m)
    mass, k = 1.5, 3.0e4
    m = Mechanism()
    b = block(m, "moteur", (0, 0, 0), mass)
    j = m.elastic(m.ground, b, (0, 0, 0), k_lin=k, k_rot=50.0, c_lin=0.0, c_rot=0.0)
    rows = D.Dynamics(m).run(0.3, dt=0.0005, every=4)
    w = math.sqrt(k / mass)
    for r in rows:
        z = dyn_z = r["x"][2]
        assert abs(z - (-mass * G / k) * (1 - math.cos(w * r["t"]))) < 1e-7, (r["t"], z)


def test_slider_stop_holds_the_block():
    alpha = math.radians(30)
    d = np.array([math.cos(alpha), 0, -math.sin(alpha)])
    m = Mechanism()
    b = block(m, "chariot", (0, 0, 0), 1.0)
    j = m.slider(m.ground, b, (0, 0, 0), d, limits=(None, 0.10))   # stop 100 mm down
    rows = D.Dynamics(m).run(1.5, dt=0.001, every=10)
    s = np.array([m.slide_value(j, r["x"]) for r in rows])
    assert s.max() < 0.10 + 0.002                             # pressed in less than 2 mm
    assert abs(s[-1] - 0.10) < 0.0005                          # resting on it
    assert np.linalg.norm(rows[-1]["u"]) < 1e-3


def test_pivot_stop_and_damper():
    m = Mechanism()
    b = rod(m, "abattant", (0, 0, 0), (0.5, 0, 0), 2.0)      # a flap, horizontal
    j = m.pivot(m.ground, b, (0, 0, 0), (0, -1, 0), limits=(-math.pi / 4, None))
    j.damper = 0.2
    rows = D.Dynamics(m).run(2.0, dt=0.002, every=25)
    th = [m.joint_value(j, r["x"]) for r in rows]
    assert min(th) > -math.pi / 4 - 0.03                      # caught by the stop at 45°
    assert abs(th[-1] + math.pi / 4) < 0.005                  # and resting on it
    assert np.linalg.norm(rows[-1]["u"]) < 0.01
