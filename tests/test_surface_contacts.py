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


def _drop(z0, seconds):
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
    assert [(a.name, b.name) for a, b, _ in dyn.impacts.mesh_pairs] == [("plateau", "bloc")]
    rows = dyn.run(seconds, dt=1e-4, every=100)
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
