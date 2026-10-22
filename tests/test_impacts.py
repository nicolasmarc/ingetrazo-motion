"""Shocks and resting contacts between free parts (motion.impacts)."""
import math
import numpy as np
from motion.model import Mechanism, Box, rotation
from motion.dynamics import Dynamics

DT = 2e-4


def _table(m, half=(0.5, 0.5, 0.01)):
    m.ground.boxes.append(Box((0, 0, -half[2]), half))


def _block(m, name, c, half, rho=700.0, R=None, mu=0.4, e=0.4):
    R = np.eye(3) if R is None else R
    c = np.asarray(c, float)
    b = m.body(name, c, [Box(c, half, R)])
    s = 2 * np.asarray(half, float)
    b.mass = rho * float(np.prod(s))
    b.com = c.copy()
    b.inertia = R @ (b.mass / 12 * np.diag([s[1] ** 2 + s[2] ** 2, s[0] ** 2 + s[2] ** 2,
                                             s[0] ** 2 + s[1] ** 2])) @ R.T
    b.friction, b.restitution = mu, e
    return b


def test_block_rests_on_the_table():
    m = Mechanism()
    _table(m)
    _block(m, "bloc", (0, 0, 0.02), (0.02, 0.02, 0.02))
    dyn = Dynamics(m, contacts=dict(dt=DT))
    rows = dyn.run(0.3, dt=DT, every=500)
    x = rows[-1]["x"]
    assert abs(x[2]) < 1e-4                      # sinks less than 0.1 mm
    assert np.linalg.norm(x[:2]) < 1e-5 and np.linalg.norm(x[3:6]) < 1e-3
    assert dyn.impacts.deepest < 1e-4


def test_rebound_follows_the_restitution():
    m = Mechanism()
    _table(m)
    e = 0.5
    _block(m, "bloc", (0, 0, 0.12), (0.01, 0.01, 0.01), e=e)   # falls 0.11 m
    dyn = Dynamics(m, contacts=dict(dt=DT))
    rows = dyn.run(0.35, dt=DT, every=5)
    z = np.array([r["x"][2] for r in rows]) + 0.12 - 0.01
    k = int(np.argmin(z[: len(z) // 2 + 10]))     # first touch
    top = z[k:].max()
    assert 0.5 * e * e * 0.11 < top < 1.6 * e * e * 0.11


def test_friction_stops_a_sliding_block():
    m = Mechanism()
    _table(m)
    mu = 0.4
    _block(m, "bloc", (0, 0, 0.01), (0.02, 0.02, 0.01), mu=mu)
    u0 = np.zeros(6)
    u0[0] = 0.5
    dyn = Dynamics(m, contacts=dict(dt=DT))
    rows = dyn.run(0.25, dt=DT, u0=u0, every=1250)
    s = rows[-1]["x"][0]
    expect = 0.5 ** 2 / (2 * mu * 9.81)
    assert abs(s - expect) < 0.15 * expect
    assert np.linalg.norm(rows[-1]["u"][:3]) < 1e-2


def test_a_falling_domino_knocks_the_next_one_down():
    H, W, E = 0.048, 0.024, 0.0075
    m = Mechanism()
    _table(m)
    for i in range(3):
        _block(m, f"D{i + 1}", (0.030 * i, 0, H / 2), (E / 2, W / 2, H / 2), rho=720)
    x0 = np.zeros(m.n)
    tilt = math.radians(15)
    e0, c = np.array([E / 2, 0, 0]), np.array([0, 0, H / 2])
    x0[0:3] = e0 + rotation([0, tilt, 0]) @ (c - e0) - c
    x0[3:6] = [0, tilt, 0]
    dyn = Dynamics(m, contacts=dict(dt=DT))
    rows = dyn.run(0.6, dt=DT, x0=x0, every=3000)
    ang = [math.degrees(np.linalg.norm(rows[-1]["x"][6 * k + 3: 6 * k + 6])) for k in range(3)]
    assert ang[0] > 60 and ang[1] > 60 and ang[2] > 80, ang      # the last one lies flat
    assert dyn.impacts.deepest < 1e-3


def test_parts_held_by_a_joint_do_not_knock():
    m = Mechanism()
    _table(m)
    b = _block(m, "volet", (0, 0, 0.05), (0.005, 0.05, 0.05))
    m.pivot(m.ground, b, (0, 0, 0.0), (0, 1, 0), None, "charnière")
    m.ground.boxes.append(Box((0, 0, 0.05), (0.02, 0.02, 0.02)))   # overlaps the flap
    dyn = Dynamics(m, contacts=dict(dt=DT))
    x = np.zeros(m.n)
    M, F = dyn.M_and_forces(x, np.zeros(dyn.n), 0.0)
    assert dyn.impacts.count == 0
    assert np.allclose(F[3:6], 0.0) and np.allclose(F[:3], b.mass * dyn.g)


def test_shocks_are_stored_in_the_document():
    from motion import document as doc
    d = doc.set_simulation(doc.empty(), True, 2.5, chocs=True)
    assert d["simulation"] == {"active": True, "duree": 2.5, "chocs": True}
    d = doc.set_simulation(d, True, 2.5)                  # left as it was
    assert d["simulation"]["chocs"] is True
    assert doc.empty()["simulation"].get("chocs", False) is False


def test_validation_counts_shocks_as_no_fault():
    from motion import collide
    H, W, E = 0.048, 0.024, 0.0075
    m = Mechanism()
    _table(m)
    for i in range(2):                 # two dominoes leaning on each other
        _block(m, f"D{i + 1}", (0.006 * i, 0, H / 2), (E / 2, W / 2, H / 2))
    poses = [dict(t=0.0, value=0.0, x=np.zeros(m.n), ok=True, res=0.0)]
    assert not collide.validate(m, jeu_min=0.001, poses=poses)["ok"]
    assert collide.validate(m, jeu_min=0.001, poses=poses, contacts=True)["ok"]


def test_a_large_face_is_tiled_for_drawing():
    from motion import render
    m = Mechanism()
    _table(m)
    _block(m, "D1", (0, 0, 0.004), (0.024, 0.012, 0.004))     # a domino lying flat
    sh = render.Shaded(m)
    ground = [it for it in sh.items if it[0] is m.ground][0]
    tiles = [f for f in ground[2] if f.dtype == render.TILE]
    assert len(tiles) > 100                     # 1 m x 1 m cut about 24 mm across
    part = [it for it in sh.items if it[0].name == "D1"][0]
    assert len(part[2]) == 6                    # the parts themselves stay whole


def test_moving_parts_hidden_without_touching_the_saved_state():
    """The IngeTrazo view of the video hides the moving parts: the scene
    version takes a value of its own meanwhile, then comes back."""
    from types import SimpleNamespace
    from motion.panel import MotionPanel
    sc = SimpleNamespace(version=41)
    vp = SimpleNamespace(scene=sc, _epoch_memo=1, _epoch_same=2)
    me = SimpleNamespace(app=SimpleNamespace(viewport=vp))
    MotionPanel._refresh_view_cache(me, True)
    assert sc.version < 0 and vp._epoch_memo is None
    first = sc.version
    MotionPanel._refresh_view_cache(me, False)
    assert sc.version == 41
    MotionPanel._refresh_view_cache(me, True)
    assert sc.version < 0 and sc.version != first   # never the same twice
    MotionPanel._refresh_view_cache(me, False)
    assert sc.version == 41
