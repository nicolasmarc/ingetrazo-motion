"""Automatic joint detection (motion.detect) on assemblies written by an
independent CAD kernel (tests/data/detect_*.step, CadQuery/OpenCascade),
then the proposals turned into a working mechanism (motion.document)."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
from motion import detect, stepread, solver, document as doc

DATA = os.path.join(HERE, "data")
FACTS = json.load(open(os.path.join(DATA, "detect_facts.json")))


def _scan(name):
    asm = stepread.read(os.path.join(DATA, f"detect_{name}.step"))
    parts = [detect.part_from_step(p) for p in asm.parts]
    props, frame = detect.scan(parts)
    pts = {p.name: p.triangles.reshape(-1, 3) for p in asm.parts}
    return asm, parts, props, frame, pts


def _mechanism(name, travel):
    asm, parts, props, frame, pts = _scan(name)
    d, made = doc.apply_detection(doc.empty(), {p.name: (p.name, [p.name]) for p in asm.parts},
                                  frame, props)
    first = next(j for j in d["joints"] if j["kind"] != "coupling")
    d = doc.add_driver(d, first["name"], 0.0, travel)
    m, problems = doc.build(d, lambda u: pts.get(u))
    return m, problems, d, made


def test_cylinder_fit_shaft_and_bore():
    r, n, h = 0.01, 32, 0.02
    def ring(sign):
        faces = []
        for k in range(n):
            a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
            P = [(r * math.cos(a0), r * math.sin(a0), 0), (r * math.cos(a1), r * math.sin(a1), 0),
                 (r * math.cos(a1), r * math.sin(a1), h), (r * math.cos(a0), r * math.sin(a0), h)]
            faces.append(np.array(P if sign > 0 else P[::-1]))
        soft = [(k, (k + 1) % n) for k in range(n)]
        return detect.regions_from_faces(faces, soft)
    for sign, convex in ((1, True), (-1, False)):
        (reg,) = ring(sign)
        c = detect.fit_cylinder(reg)
        assert c is not None and c.convex is convex
        assert abs(abs(c.axis[2]) - 1) < 1e-9 and abs(c.radius - r) < 0.01 * r
        assert abs(c.t1 - c.t0 - h) < 1e-12 and c.span >= 355


def test_four_bar_four_pivots_on_the_frame():
    asm, parts, props, frame, _ = _scan("quadrilatere")
    f = FACTS["quadrilatere"]
    assert frame == "Socle"
    piv = {frozenset((x.a, x.b)): x for x in props if x.kind == "pivot"}
    assert len(props) == 4 and len(piv) == 4
    where = {("Socle", "Manivelle"): f["A"], ("Manivelle", "Bielle"): f["B"],
             ("Bielle", "Balancier"): f["C"], ("Socle", "Balancier"): f["D"]}
    for pair, xy in where.items():
        x = piv[frozenset(pair)]
        assert abs(abs(x.axis[2]) - 1) < 1e-6
        assert np.allclose(x.point[:2], np.array(xy) / 1000, atol=1e-6), (pair, x.point)
        assert "Ø 10.0 mm" in x.detail and "jeu 0.10 mm" in x.detail


def test_four_bar_moves_like_a_crank_rocker():
    m, problems, d, made = _mechanism("quadrilatere", 2 * math.pi)
    assert not problems and len(made) == 4
    F = solver.sweep(m, 37)
    assert all(r["ok"] for r in F)
    assert solver.dof(m, F[5]["x"]) == 1
    # the rocker swings between the two closed-form extremes of a crank-rocker
    f = FACTS["quadrilatere"]
    A, B, C, D = (np.array(f[k], float) / 1000 for k in "ABCD")
    a, b, c, d0 = (np.linalg.norm(B - A), np.linalg.norm(C - B), np.linalg.norm(D - C),
                   np.linalg.norm(D - A))
    rock = next(j for j in m.joints if j.kind == "pivot" and "Balancier" in (j.a.name, j.b.name)
                and "Socle" in (j.a.name, j.b.name))
    ang = [m.continuous_value(rock, r["x"]) for r in F]
    swing = max(ang) - min(ang)
    # extremes: crank and coupler in line (b + a, b - a)
    def rocker_angle(L):
        return math.acos((c * c + d0 * d0 - L * L) / (2 * c * d0))
    th = abs(rocker_angle(b + a) - rocker_angle(b - a))
    assert abs(swing - th) < 0.02 * th, (swing, th)


def test_gears_found_with_their_ratio():
    asm, parts, props, frame, _ = _scan("engrenages")
    assert frame == "Platine"
    g = [x for x in props if x.kind == "gears"]
    assert len(g) == 1 and g[0].extra["ratio"] == -20 / 30
    assert "20 et 30 dents" in g[0].detail
    m, problems, d, made = _mechanism("engrenages", 2 * math.pi)
    assert not problems and any(j["kind"] == "coupling" for j in d["joints"])
    F = solver.sweep(m, 13)
    pv = [j for j in m.joints if j.kind == "pivot"]
    x = F[-1]["x"]
    assert abs(m.continuous_value(pv[1], x) / m.continuous_value(pv[0], x) + 20 / 30) < 1e-6
    assert solver.dof(m, F[3]["x"]) == 1


def test_slide_along_the_channel():
    asm, parts, props, frame, _ = _scan("coulisse")
    assert frame == "Rail" and [x.kind for x in props] == ["slider"]
    assert abs(abs(props[0].axis[0]) - 1) < 1e-9          # along x
    m, problems, d, made = _mechanism("coulisse", 0.05)
    F = solver.sweep(m, 5)
    assert all(r["ok"] for r in F) and solver.dof(m, F[2]["x"]) == 1


def test_hinge_pin_goes_with_the_leaf_it_is_pressed_in():
    asm, parts, props, frame, _ = _scan("charniere")
    assert frame == "Aile fixe" and len(props) == 1
    x = props[0]
    assert x.kind == "pivot" and {x.a, x.b} == {"Aile fixe", "Aile mobile"}
    assert x.extra["merge"] == {"Axe": "Aile fixe"} and "jeu 0.10 mm" in x.detail
    assert abs(abs(x.axis[1]) - 1) < 1e-9                   # the hinge axis (y)
    m, problems, d, made = _mechanism("charniere", 2.0)
    assert not problems and made == ["pivot Aile fixe–Aile mobile"]
    assert d["bodies"][0]["groups"] == ["Aile fixe", "Axe"] and d["bodies"][0]["fixed"]
    F = solver.sweep(m, 9)
    assert all(r["ok"] for r in F) and solver.dof(m, F[3]["x"]) == 1


def test_nothing_proposed_for_parts_that_do_not_fit():
    # the gears without their plate: two toothed wheels, no shaft, no bore fit
    asm, parts, props, frame, _ = _scan("engrenages")
    alone = [p for p in parts if p.name != "Platine"]
    props2, _ = detect.scan(alone)
    assert props2 == []
    # a shaft much smaller than the bore is not a pivot
    asm, parts, props, frame, _ = _scan("quadrilatere")
    for p in parts:
        for c in p.cylinders:
            if c.convex:
                c.radius *= 0.7
    assert [x for x in detect.scan(parts)[0] if x.kind == "pivot"] == []


def test_teeth_counted_on_an_outline():
    for z in (9, 17, 40):
        pts = []
        for k in range(z):
            for da, r in ((-0.35, 0.018), (-0.15, 0.022), (0.15, 0.022), (0.35, 0.018)):
                t = 2 * math.pi * (k + da) / z
                pts.append((r * math.cos(t), r * math.sin(t), 0.0))
        res = detect.count_teeth(np.array(pts), np.zeros(3), np.array([0, 0, 1.0]))
        assert res is not None and res[0] == z


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
