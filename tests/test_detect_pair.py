"""A joint found from the two parts' shapes (no edge to pick): a shaft in
a bore, a ball in its socket, flat slides."""
import math
import numpy as np

from motion import detect


def _faces_soft(faces):
    """All neighbours softened: one smooth region."""
    return detect.regions_from_faces(faces, [(i, i + 1) for i in range(len(faces) - 1)])


def _cylinder(r, z0, z1, n=32, inward=False, c=(0, 0, 0)):
    c = np.asarray(c, float)
    out = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        p = [c + (r * math.cos(a0), r * math.sin(a0), z0), c + (r * math.cos(a1), r * math.sin(a1), z0),
             c + (r * math.cos(a1), r * math.sin(a1), z1), c + (r * math.cos(a0), r * math.sin(a0), z1)]
        out.append(np.array(p[::-1] if inward else p))
    return out


def _sphere(r, c=(0, 0, 0), n=16, inward=False):
    c = np.asarray(c, float)
    f = lambda t, u: c + r * np.array([math.sin(t) * math.cos(u), math.sin(t) * math.sin(u), math.cos(t)])
    out = []
    for i in range(n):
        for j in range(2 * n):
            t0, t1 = math.pi * i / n, math.pi * (i + 1) / n
            u0, u1 = math.pi * j / n, math.pi * (j + 1) / n
            q = np.array([f(t0, u0), f(t1, u0), f(t1, u1), f(t0, u1)])
            out.append(q[::-1] if inward else q)
    return out


def test_shaft_in_bore_gives_the_axis():
    shaft = detect.make_part("a", "arbre", _faces_soft(_cylinder(0.01, -0.05, 0.05)))
    hub = detect.make_part("b", "moyeu", _faces_soft(_cylinder(0.0101, -0.02, 0.02, inward=True)))
    x = detect.between(hub, shaft, "pivot")
    assert x is not None and abs(abs(x.axis[2]) - 1) < 1e-6
    assert np.linalg.norm(x.point[:2]) < 1e-6


def test_ball_in_socket_gives_the_centre():
    c = (0.1, 0.02, 0.3)
    ball = detect.make_part("a", "rotule", _faces_soft(_sphere(0.0125, c)))
    cup = detect.make_part("b", "coupelle", _faces_soft(_sphere(0.0127, c, inward=True)))
    assert ball.spheres and not ball.cylinders and ball.spheres[0].convex
    assert cup.spheres and not cup.spheres[0].convex
    x = detect.between(cup, ball, "ball")
    assert x is not None and np.linalg.norm(x.point - c) < 1e-5


def test_nothing_in_common_says_so():
    a = detect.make_part("a", "a", _faces_soft(_cylinder(0.01, 0, 0.01)))
    b = detect.make_part("b", "b", _faces_soft(_sphere(0.02, (0.5, 0, 0))))
    assert detect.between(a, b, "pivot") is None and detect.between(a, b, "ball") is None


def test_a_round_slide():
    rod = detect.make_part("a", "tige", _faces_soft(_cylinder(0.008, -0.1, 0.1)))
    sleeve = detect.make_part("b", "douille", _faces_soft(_cylinder(0.0081, -0.01, 0.01, inward=True)))
    x = detect.between(sleeve, rod, "slider")
    assert x is not None and x.kind == "slider" and abs(abs(x.axis[2]) - 1) < 1e-6


def test_scan_proposes_the_ball_joint_and_applies_it():
    from motion import document as doc
    c = (0.0, 0.0, 0.1)
    import test_detect_pair as T
    base = detect.make_part("base", "socle", list(T._faces_soft(T._sphere(0.0127, c, inward=True)))
                            + detect.regions_from_faces([np.array([(-.1, -.1, 0), (.1, -.1, 0), (.1, .1, 0), (-.1, .1, 0.0)])], []))
    ball = detect.make_part("ball", "rotule", T._faces_soft(T._sphere(0.0125, c)))
    props, frame = detect.scan([base, ball])
    assert [x.kind for x in props] == ["ball"]
    d, made = doc.apply_detection(doc.empty(), {"base": ("socle", ["base"]), "ball": ("rotule", ["ball"])},
                                  "base", props)
    assert d["joints"][0]["kind"] == "ball" and np.allclose(d["joints"][0]["point"], c, atol=1e-5)
