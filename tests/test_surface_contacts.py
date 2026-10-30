"""Dynamics with contacts on the parts' real surfaces (every part touches
every other by default, no material needed): a block dropped on a plate
lands, bounces and rests on it; a block drawn a hair into its plate (an
interference of the model) stays put instead of being thrown off."""
import math, os, sys
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from motion import document as doc, dynamics
from test_mass import box


def _drop(z0, seconds, hard=False, dt=1e-4):
    tris = {"plateau": box((-0.05, -0.05, -0.01), (0.05, 0.05, 0.0)),
            "bloc": box((-0.015, -0.015, z0), (0.015, 0.015, z0 + 0.03))}
    d = doc.empty()
    d, _ = doc.add_body(d, ["plateau"], True, "plateau")
    d, _ = doc.add_body(d, ["bloc"], False, "bloc")
    d = doc.set_simulation(d, True, seconds, True)
    m, problems = doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                            triangles_of_group=tris.get)
    assert not problems, problems
    dyn = dynamics.Dynamics(m, contacts=dict(dt=1e-4))
    dyn.method = "symplectic"
    dyn.impacts.hard = hard
    assert [(a.name, b.name) for a, b, _ in dyn.impacts.mesh_pairs] == [("plateau", "bloc")]
    rows = dyn.run(seconds, dt=dt, every=max(1, int(round(0.01 / dt))))
    b = m.bodies[0]
    z = [float(b.world_point(m.pose(r["x"], b), (0, 0, z0))[2]) for r in rows]
    return dyn, rows, z


def test_a_block_lands_and_rests_on_its_plate():
    dyn, rows, z = _drop(0.01, 0.12)
    # free fall until it meets the plate (10 mm in 45 ms)
    k = next(i for i, r in enumerate(rows) if abs(r["t"] - 0.04) < 1e-9)
    assert abs(z[k] - (0.01 - 0.5 * 9.81 * 0.04 ** 2)) < 1e-4, z[k]
    assert abs(z[-1]) < 2e-4, z[-1]                 # resting on it, not sunk
    assert dyn.impacts.deepest < 1e-3
    assert abs(dyn.mass[id(dyn.bodies[0])] - 700.0 * 0.03 ** 3) < 1e-9   # no material: 700 kg/m3


def test_an_interference_of_the_drawing_is_the_assembly():
    dyn, rows, z = _drop(-0.0005, 0.03)             # drawn 0.5 mm into the plate
    v = max(float(np.linalg.norm(r["u"][:3])) for r in rows)
    assert v < 0.05, v                              # not shot out of it


def test_contacts_as_constraints_hold_with_a_long_step():
    # unilateral contacts: no stiffness to resolve, a step of 1 ms; the
    # block lands without bouncing and rests exactly on the plate
    dyn, rows, z = _drop(0.01, 0.15, hard=True, dt=1e-3)
    k = next(i for i, r in enumerate(rows) if abs(r["t"] - 0.04) < 1e-9)
    assert abs(z[k] - (0.01 - 0.5 * 9.81 * 0.04 ** 2)) < 5e-4, z[k]
    assert max(z[k + 2:]) < 2e-4, z                # no bounce
    assert abs(z[-1]) < 1e-4, z[-1]
    assert float(np.linalg.norm(rows[-1]["u"])) < 1e-3


def test_mass_of_the_real_solid():
    from motion.dynamics import mesh_properties
    m, c, I = mesh_properties(box((0, 0, 0), (0.2, 0.1, 0.05)), 1000.0)
    assert abs(m - 1.0) < 1e-12
    assert np.allclose(c, (0.1, 0.05, 0.025))
    assert np.allclose(I, np.diag([0.0125, 0.0425, 0.05]) / 12)
    # a ring, not its box: a quarter of the mass is not there
    ring = np.concatenate([box((-0.05, -0.05, 0), (0.05, -0.025, 0.01)),
                           box((-0.05, 0.025, 0), (0.05, 0.05, 0.01)),
                           box((-0.05, -0.025, 0), (-0.025, 0.025, 0.01)),
                           box((0.025, -0.025, 0), (0.05, 0.025, 0.01))])
    m, c, I = mesh_properties(ring, 1000.0)
    assert abs(m - 1000 * 0.01 * (0.1 ** 2 - 0.05 ** 2)) < 1e-9
    assert np.allclose(c, (0, 0, 0.005), atol=1e-12)


def test_triangles_near_a_point_whatever_their_size():
    from motion.contact import TriGrid, closest_on_triangles
    T = box((-0.1, -0.1, -0.01), (0.1, 0.1, 0.0)).reshape(-1, 3, 3)   # 12 big faces
    g = TriGrid(T, 0.004)
    X = np.random.default_rng(1).random((500, 3)) * [0.2, 0.2, 0.02] - [0.1, 0.1, 0.012]
    keep, it = g.nearest(X)
    n = len(T)
    cp = closest_on_triangles(np.repeat(X, n, 0), *[np.tile(T[:, i], (len(X), 1)) for i in range(3)])
    D = np.linalg.norm(np.repeat(X, n, 0) - cp, axis=1).reshape(len(X), n)
    best = D.min(axis=1)
    assert not np.any((best < 0.004) & ~keep)
    assert np.allclose(D[np.nonzero(keep)[0], it[:, 0]], best[keep])


def test_dominoes_fall_one_on_the_next():
    # three dominoes on a plate, the first drawn leaning: no material, no
    # contact to declare; each pushes the next down, friction holds their feet
    def rot_y(T, ang, pivot):
        c, s = math.cos(ang), math.sin(ang)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        return (T.reshape(-1, 3) - pivot) @ R.T + pivot
    tris = {"plateau": box((-0.05, -0.05, -0.01), (0.15, 0.05, 0.0))}
    for i in range(3):
        x0 = 0.025 * i
        T = box((x0, -0.01, 0.0), (x0 + 0.006, 0.01, 0.04)).reshape(-1, 3)
        if i == 0:
            T = rot_y(T, math.radians(12), np.array([x0 + 0.006, 0, 0]))
        tris[f"d{i}"] = T.reshape(-1, 3, 3)
    d = doc.empty()
    d, _ = doc.add_body(d, ["plateau"], True, "plateau")
    for i in range(3):
        d, _ = doc.add_body(d, [f"d{i}"], False, f"d{i}")
    d = doc.set_simulation(d, True, 0.6, True)
    m, problems = doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                            triangles_of_group=tris.get)
    assert not problems, problems
    dyn = dynamics.Dynamics(m, contacts=dict(dt=1e-3))
    dyn.method = "symplectic"
    assert dyn.impacts.surfaces_only
    dyn.impacts.hard = True
    rows = dyn.run(0.6, dt=1e-3, every=50)
    bs = {b.name: b for b in dyn.bodies}

    def tilt(r, i):
        b = bs[f"d{i}"]
        z = b.rot(m.pose(r["x"], b)) @ np.array([0, 0, 1.0])
        return math.degrees(math.atan2(z[0], z[2]))

    first = [next((r["t"] for r in rows if tilt(r, i) > 20), None) for i in range(3)]
    assert None not in first and first[0] < first[1] < first[2], first
    assert all(tilt(rows[-1], i) > 55 for i in range(3)), [tilt(rows[-1], i) for i in range(3)]
    # the feet stay where they stood (friction), the last lies on the plate
    for i in range(1, 3):
        b = bs[f"d{i}"]
        foot = b.world_point(m.pose(rows[-1]["x"], b), (0.025 * i + 0.006, 0, 0))
        assert abs(foot[0] - (0.025 * i + 0.006)) < 5e-3 and abs(foot[2]) < 1e-3, foot
