"""Contact: a part pushed by another (motion.push). A flap on its pivot is
pushed by an arm turning about another axis: it stands still until the arm
reaches it, then turns just enough never to be gone through, and stays
where it was left when the arm goes back."""
import math, os, sys
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from motion import collide, solver, document as doc, push
from motion.model import rotation
from test_mass import box


def _build():
    # frame, an arm standing at x = 0 (driven, pivot at its foot) and a
    # flap standing at x = 0.10 (pivot at its foot), axes along y: the arm
    # leans over towards the flap, meets its top and lays it over
    tris = {"bati": box((-0.05, -0.02, -0.06), (0.20, 0.02, -0.05)),
            "bras": box((-0.004, -0.005, 0.0), (0.004, 0.005, 0.13)),
            "volet": box((0.096, -0.005, 0.0), (0.104, 0.005, 0.05))}
    d = doc.empty()
    for name, fixed in (("bati", True), ("bras", False), ("volet", False)):
        d, _ = doc.add_body(d, [name], fixed, name)
    d, pa = doc.add_joint(d, "pivot", "bati", "bras", point=(0, 0, 0), axis=(0, 1, 0))
    d, pv = doc.add_joint(d, "pivot", "bati", "volet", point=(0.10, 0, 0), axis=(0, 1, 0))
    d, c = doc.add_joint(d, "contact", "volet", "bras")
    d = doc.add_driver(d, pa, 0.0, math.radians(80))
    m, problems = doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                            triangles_of_group=tris.get)
    return m, problems, d


def test_rotvec_is_the_inverse_of_rotation():
    rng = np.random.default_rng(3)
    for _ in range(50):
        w = rng.normal(size=3)
        w *= rng.uniform(0, 3.1) / np.linalg.norm(w)
        assert np.allclose(push.rotvec(rotation(w)), w, atol=1e-9)
    assert np.allclose(push.rotvec(np.eye(3)), 0)


def test_a_flap_pushed_by_an_arm():
    m, problems, d = _build()
    assert not problems, problems
    assert m.followers and isinstance(m.followers[0], push.PushGroup)
    rows = solver.sweep(m, 31)
    flap = next(j for j in m.joints if j.name.startswith("pivot") and j.b.name == "volet")
    arm = next(j for j in m.joints if j.b.name == "bras")
    a = [m.joint_value(arm, r["x"]) for r in rows]
    f = [m.joint_value(flap, r["x"]) for r in rows]
    # the arm reaches up past the flap's foot: the flap stands still first,
    # then is laid over the same way the arm turns, never going back
    assert abs(f[0]) < 1e-9 and abs(f[3]) < 1e-9
    assert f[-1] > math.radians(30), [round(math.degrees(v), 1) for v in f]
    assert all(f[k + 1] >= f[k] - 1e-9 for k in range(len(f) - 1))
    # it starts moving only once the arm reaches its top (about 63 deg)
    first = next(k for k in range(len(f)) if f[k] > 1e-6)
    assert math.radians(55) < a[first] < math.radians(70), math.degrees(a[first])
    # never gone through
    rep = collide.validate(m, 31, 0.0)
    assert not [p for p in rep["problems"] if "collision" in p], rep["problems"][:5]


def test_a_contact_needs_a_single_pivot():
    tris = {"bati": box((-0.05, -0.02, -0.06), (0.20, 0.02, -0.05)),
            "bras": box((-0.004, -0.005, 0.0), (0.004, 0.005, 0.13)),
            "volet": box((0.096, -0.005, 0.0), (0.104, 0.005, 0.05))}
    d = doc.empty()
    for name, fixed in (("bati", True), ("bras", False), ("volet", False)):
        d, _ = doc.add_body(d, [name], fixed, name)
    d, pa = doc.add_joint(d, "pivot", "bati", "bras", point=(0, 0, 0), axis=(0, 1, 0))
    d, _ = doc.add_joint(d, "pivot", "bati", "volet", point=(0.10, 0, 0), axis=(0, 1, 0))
    d, _ = doc.add_joint(d, "slider", "bati", "volet", point=(0.10, 0, 0), axis=(1, 0, 0))
    d, _ = doc.add_joint(d, "contact", "volet", "bras")
    d = doc.add_driver(d, pa, 0.0, 1.0)
    m, problems = doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                            triangles_of_group=tris.get)
    assert any("contact" in p for p in problems), problems
