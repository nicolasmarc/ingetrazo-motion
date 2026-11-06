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


def test_own_axis_of_a_part_without_its_shaft():
    # a toothed wheel: the centre of its teeth; a link: its round shape
    # nearest the part it turns in (pin or bore); a bar with round ends is no wheel
    asm, parts, props, frame, _ = _scan("engrenages")
    pivots = [x for x in props if x.kind == "pivot"]
    for p in parts:
        if p.name == "Platine":
            continue
        pt, ax, kind, det = detect.own_axis(p)
        assert kind == "gear" and "dents" in det, (p.name, det)
        ref = next(x for x in pivots if p.key in (x.a, x.b))
        d = pt - ref.point
        assert abs(abs(ax @ ref.axis) - 1) < 1e-6
        assert np.linalg.norm(d - (d @ ref.axis) * ref.axis) < 1e-6
    asm, parts, props, frame, _ = _scan("quadrilatere")
    by = {p.name: p for p in parts}
    for name in ("Manivelle", "Balancier"):
        pt, ax, kind, det = detect.own_axis(by[name], near=by["Socle"].points)
        assert kind in ("bore", "round"), (name, det)
        ref = next(x for x in props if x.kind == "pivot" and {x.a, x.b} == {by[name].key, frame})
        d = pt - ref.point
        assert np.linalg.norm(d - (d @ ref.axis) * ref.axis) < 1e-6, name
    assert detect.own_axis(by["Bielle"])[2] != "gear"


def test_teeth_of_a_helical_wheel():
    # 21 teeth twisted by one tooth pitch across the face (a helical wheel,
    # Nicolas's Geneva model): the whole outline is a full circle, the
    # thin slices still show the teeth
    z, rt, rr, width = 21, 0.0494, 0.0446, 0.018
    pts = []
    for h in np.linspace(0, width, 60):
        twist = (2 * math.pi / z) * h / width
        for a in np.linspace(0, 2 * math.pi, 1500, endpoint=False):
            r = rr + (rt - rr) * (0.5 + 0.5 * math.cos(z * (a - twist)))
            pts.append((r * math.cos(a), h, r * math.sin(a)))
    P = np.array(pts) + (0.114, 0.0, 0.062)
    ax = np.array([0.0, 1.0, 0.0])
    centre = np.array([0.114, 0.0, 0.062])
    assert detect.count_teeth(P, centre, ax) is None or \
        not detect._regular_teeth(P, centre, ax, detect.count_teeth(P, centre, ax)[0])
    t = detect.teeth_of(P, centre, ax)
    assert t is not None and t[0] == z, t
    part = detect.Part(key="g", name="g", points=P, lo=P.min(0), hi=P.max(0))
    pt, axis, kind, det = detect.own_axis(part)
    assert kind == "gear" and abs(abs(axis @ ax) - 1) < 1e-9
    assert np.linalg.norm((pt - centre) - ((pt - centre) @ ax) * ax) < 2e-4


def test_a_boss_seen_in_the_sections():
    # a plate with a round boss, all of one smooth piece (no cylinder of
    # its own to fit): the boss is a circle in the cuts across its axis
    n, R, c = 48, 0.007, np.array([-0.026, 0.0, 0.152])
    T = []
    ring = [c + R * np.array([math.cos(a), 0, math.sin(a)])
            for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    for i in range(n):                                  # the boss: a tube 10 mm long
        a, b = ring[i], ring[(i + 1) % n]
        a2, b2 = a + (0, 0.010, 0), b + (0, 0.010, 0)
        T += [(a, b, b2), (a, b2, a2)]
    plate = [(-0.05, 0.0, 0.10), (0.07, 0.0, 0.10), (0.07, 0.0, 0.20), (-0.05, 0.0, 0.20)]
    T += [(plate[0], plate[1], plate[2]), (plate[0], plate[2], plate[3])]
    T = np.array(T, float)
    found = detect.round_outlines(T, np.array([0.0, 1.0, 0.0]))
    assert found, "no circle"
    centre, r = found[0]
    assert abs(r - R) < 2e-4 and np.hypot(centre[0] - c[0], centre[2] - c[2]) < 1e-4
    part = detect.Part(key="c", name="c", points=T.reshape(-1, 3),
                       lo=T.reshape(-1, 3).min(0), hi=T.reshape(-1, 3).max(0))
    pt, ax, kind, det = detect.own_axis(part, triangles=T)
    assert abs(abs(ax[1]) - 1) < 1e-9 and np.hypot(pt[0] - c[0], pt[2] - c[2]) < 1e-4, det


def test_an_arm_bolted_through_a_slot_is_a_lock():
    # a wheel on its pin carries an arm on the same pin; a bolt of the arm,
    # 20 mm off the axis, goes through an arc slot of the wheel: the bolt
    # is tightened, the arm and the wheel are one part (not a pivot)
    import math as _m
    from motion.detect import Cylinder, Part, Proposal, _clamped
    Y = np.array([0.0, 1.0, 0.0])
    pin = Proposal("pivot", "roue", "bras", np.zeros(3), Y, 0.8, "")
    bolt = Cylinder(np.array([0.02, 0.0, 0.0]), Y, 0.003, -0.004, 0.004, True, 360.0, 0.0)
    arm = Part("bras", "bras", cylinders=[bolt],
               points=np.array([[0.02 + 0.003 * _m.cos(a), 0.0, 0.003 * _m.sin(a)]
                                for a in np.linspace(0, 2 * _m.pi, 24)]))
    # the slot's two walls, 3.5 mm from the bolt's axis on either side (radially)
    walls = [[r * _m.cos(a), y, r * _m.sin(a)] for r in (0.0165, 0.0235)
             for a in np.radians(np.linspace(-30, 30, 13)) for y in (-0.002, 0.002)]
    wheel = Part("roue", "roue", points=np.array(walls))
    for p in (arm, wheel):
        p.lo, p.hi = p.points.min(0), p.points.max(0)
    got = _clamped(wheel, arm, pin)
    assert got and got[1].kind == "lock" and got[0] is pin
    # no slot round the bolt (it stands in the air): still a pivot
    wheel.points = np.array([w for w in walls if w[0] < 0.01] + [[-0.02, 0, 0]])
    assert _clamped(wheel, arm, pin) is None


def _toothed(z, r_tip, r_root, centre=(0.0, 0.0, 0.0), inner=False, rim=None, ys=(-0.004, 0.004)):
    """Points of a toothed outline about y (``inner``: the teeth point in,
    with an outer rim of radius ``rim``)."""
    pts = []
    c = np.asarray(centre, float)
    for k in range(z):
        for da, hi in ((-0.35, False), (-0.15, True), (0.15, True), (0.35, False)):
            t = 2 * math.pi * (k + da) / z
            r = (r_tip if hi else r_root) if not inner else (r_root if hi else r_tip)
            for y in ys:
                pts.append(c + (r * math.cos(t), y, r * math.sin(t)))
    if rim:
        for t in np.linspace(0, 2 * math.pi, 90, endpoint=False):
            for y in ys:
                pts.append(c + (rim * math.cos(t), y, rim * math.sin(t)))
    return np.array(pts)


def test_internal_teeth_of_a_ring_gear():
    P = _toothed(40, 0.038, 0.042, inner=True, rim=0.05)
    got = detect.inner_teeth_of(P, np.zeros(3), np.array([0.0, 1.0, 0.0]))
    assert got is not None and got[0] == 40 and abs(got[1] - 0.038) < 1e-3
    # an external gear has no internal teeth
    assert detect.inner_teeth_of(_toothed(20, 0.022, 0.018), np.zeros(3), np.array([0.0, 1.0, 0.0])) is None


def test_planet_in_a_ring_on_a_crank_is_a_planetary_train():
    # Cardan's circles: ring 40 teeth held, planet 20 teeth on a crank pin
    # 20 mm off the ring's axis: the planet turns -2 times the crank on it
    from motion.detect import Cylinder, Part, Proposal, _planetary
    Y = np.array([0.0, 1.0, 0.0])
    ring = Part("ring", "couronne", cylinders=[Cylinder(np.zeros(3), Y, 0.05, -0.004, 0.004, True, 360, 0)],
                points=_toothed(40, 0.038, 0.042, inner=True, rim=0.05))
    planet = Part("planet", "satellite", points=_toothed(20, 0.022, 0.018, centre=(0.02, 0, 0)))
    crank = Part("crank", "manivelle", points=np.array([[0, 0.01, 0], [0.02, 0.01, 0]]))
    frame = Part("frame", "bâti", points=np.array([[0, 0.02, 0]]))
    for p in (ring, planet, crank, frame):
        p.lo, p.hi = p.points.min(0), p.points.max(0)
    pin = Proposal("pivot", "crank", "planet", np.array([0.02, 0, 0]), Y, 0.9, "")
    main = Proposal("pivot", "frame", "crank", np.zeros(3), Y, 0.9, "")
    got = _planetary([ring, planet, crank, frame], [pin, main], 0.2)
    assert len(got) == 1 and got[0].kind == "planetary"
    x = got[0]
    assert x.extra["ratio"] == -2.0 and x.extra["carrier"] == "crank" and x.extra["planet"] == "planet"
    # not meshing (the pin 5 mm further out): nothing
    pin.point = np.array([0.025, 0, 0])
    planet.points = _toothed(20, 0.022, 0.018, centre=(0.025, 0, 0))
    assert not _planetary([ring, planet, crank, frame], [pin, main], 0.2)


def test_planetary_train_turns_the_planet_twice_back_on_its_crank():
    # the planet's pin on its crank's circle moves on a straight line
    from motion.detect import Proposal
    Y = np.array([0.0, 1.0, 0.0])
    d = doc.empty()
    pins = {"bâti": [[0, 0.02, 0]], "manivelle": [[0, 0.01, 0], [0.02, 0.01, 0]],
            "satellite": [[0.02, 0, 0.0]], "couronne": [[0.05, 0, 0]]}
    main = Proposal("pivot", "bâti", "manivelle", np.zeros(3), Y, 0.9, "")
    pin = Proposal("pivot", "manivelle", "satellite", np.array([0.02, 0, 0]), Y, 0.9, "")
    lock = Proposal("lock", "bâti", "couronne", np.zeros(3), Y, 0.6, "")
    pl = Proposal("planetary", "couronne", "satellite", np.zeros(3), Y, 0.8, "",
                  dict(pivot_carrier=main, pivot_planet=pin, ratio=-2.0, ring="couronne",
                       carrier="manivelle", planet="satellite"))
    d, made = doc.apply_detection(d, {k: (k, [k]) for k in pins}, "bâti", [main, pin, lock, pl])
    cpl = next(j for j in d["joints"] if j["kind"] == "coupling")
    d = doc.add_driver(d, cpl["ja"], 0.0, 2 * math.pi)
    m, problems = doc.build(d, lambda u: np.array(pins[u]))
    F = solver.sweep(m, 37)
    sat = next(b for b in m.bodies if b.name == "satellite")
    # a point of the planet on its pitch circle (radius 20 mm), at the
    # ring's centre at the start, half a turn from where the planet meets
    # the ring: on the ring's diameter across, a straight line
    P = np.array([sat.world_point(m.pose(f["x"], sat), np.zeros(3)) for f in F])
    assert all(f["ok"] for f in F)
    assert np.max(np.abs(P[:, 0])) < 1e-6 and np.ptp(P[:, 2]) > 0.079


def test_a_shoulder_screw_threaded_in_its_hole_is_a_lock():
    # a shoulder screw (shoulder Ø 5, thread Ø 4 not seen as a cylinder)
    # screwed into a plate's Ø 5 hole: one part with the plate — and a
    # wheel turning on the shoulder pivots, as before (0.10)
    from motion.detect import Cylinder, Part, _filled_bore
    Y = np.array([0.0, 1.0, 0.0])
    hole = Cylinder(np.array([0.0, 0.0035, 0.0]), Y, 0.0025, -0.0035, 0.0035, False, 360.0, 0.0)
    plate_pts = [[0.0025 * math.cos(a), y, 0.0025 * math.sin(a)]
                 for a in np.linspace(0, 2 * math.pi, 24, endpoint=False) for y in (0.0, 0.007)]
    plate_pts += [[x, y, z] for x in (-0.02, 0.02) for y in (0.0, 0.007) for z in (-0.02, 0.02)]
    plate = Part("plaque", "plaque", cylinders=[hole], points=np.array(plate_pts))
    shoulder = Cylinder(np.array([0.0, -0.006, 0.0]), Y, 0.0025, -0.006, 0.006, True, 360.0, 0.0)
    # the thread: a helix of points at r 2 mm, from the shoulder into the hole
    t = np.linspace(0, 2 * math.pi * 9, 300)
    thread = np.column_stack([0.002 * np.cos(t), 0.0066 * t / t[-1], 0.002 * np.sin(t)])
    ring = [[0.0025 * math.cos(a), y, 0.0025 * math.sin(a)]
            for a in np.linspace(0, 2 * math.pi, 24, endpoint=False) for y in (-0.012, 0.0)]
    screw = Part("vis", "vis", cylinders=[shoulder], points=np.vstack([thread, ring]))
    for p in (plate, screw):
        p.lo, p.hi = p.points.min(0), p.points.max(0)
    got = _filled_bore(screw, plate, 0.05)
    assert got is not None and got.kind == "lock" and "vissé" in got.detail
    # the same screw with a pin through the plate's hole, recognised as a
    # cylinder: the fit test has spoken, no lock
    screw.cylinders = [shoulder, Cylinder(np.array([0.0, 0.0035, 0.0]), Y, 0.0024, -0.0035, 0.0035,
                                          True, 360.0, 0.0)]
    got = _filled_bore(screw, plate, 0.05)
    assert got is None or got.kind != "lock"
