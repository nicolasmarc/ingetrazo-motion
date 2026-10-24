"""Large parts (a 2.4 m box stack, as in Nicolas's test file of 24/10/2026):
the video export froze on it. The sampled surfaces stay under a budget of
points, the inside test runs on every point at once, and a moving part
tied to nothing fixed is reported."""
import math, os, sys, time
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from motion import contact, collide, document as doc
from test_mass import box
from test_contact import mechanism

W, D, H = 2.365, 1.630, 0.338            # his three stacked boxes (m)


def _stack():
    return {"bas": box((0, 0, 0), (W, D, H)),
            "milieu": box((0, 0, H), (W, D, 2 * H)),
            "haut": box((0, 0, 2 * H), (W, D, 3 * H))}


def test_inside_test_on_many_points_matches_one_by_one():
    tris = {"caisse": box((0, 0, 0), (0.3, 0.2, 0.1)), "bloc": box((0.5, 0, 0), (0.6, 0.1, 0.1))}
    m, _ = mechanism(tris, [("caisse", ["caisse"], True), ("bloc", ["bloc"], False)],
                     [("slider", "caisse", "bloc", (0.5, 0.05, 0.05), (1, 0, 0), None)],
                     [("glissière1", 0.0, 0.1)])
    S = m.surfaces
    A = S.data["caisse"]
    Q = np.random.default_rng(1).uniform((-0.05, -0.05, -0.05), (0.35, 0.25, 0.15), (400, 3))
    many = S._inside_many(A, Q)
    one = np.array([S._inside(A, q) for q in Q])
    assert (many == one).all()
    assert many.any() and not many.all()


def test_large_parts_stay_under_the_point_budget():
    tris = _stack()
    m, _ = mechanism(tris, [("bas", ["bas"], True), ("milieu", ["milieu"], False),
                            ("haut", ["haut"], False)],
                     [("lock", "bas", "milieu", (0, 0, H), (0, 0, 1), None),
                      ("pivot", "milieu", "haut", (W, D / 2, 2 * H), (0, 1, 0), None)],
                     [("pivot1", 0.0, math.pi / 2)])
    n = sum(len(v["P"]) for v in m.surfaces.data.values())
    assert n <= 1.2 * contact.POINT_BUDGET, n
    # small parts keep their fine step
    assert contact.budget_spacing(0.001, 0.05) == 0.001


def test_parts_going_through_each_other_are_checked_quickly():
    # the top box turns down into the middle one (the case that froze):
    # each step used to test tens of thousands of points one by one
    tris = _stack()
    m, problems = mechanism(tris, [("bas", ["bas"], True), ("milieu", ["milieu"], False),
                                   ("haut", ["haut"], False)],
                            [("lock", "bas", "milieu", (0, 0, H), (0, 0, 1), None),
                             ("pivot", "milieu", "haut", (W, D / 2, 2 * H), (0, -1, 0), None)],
                            [("pivot1", 0.0, math.pi / 2)])
    assert not problems, problems
    t0 = time.time()
    rep = collide.validate(m, 21, 0.005)
    took = time.time() - t0
    assert any("milieu" in p and "haut" in p for p in rep["problems"]), rep["problems"]
    assert took < 60, took


def test_a_moving_part_tied_to_nothing_fixed_is_reported():
    # his file: fixed1, part1, part2 and only a pivot part1 <-> part2
    tris = _stack()
    bodies = [("fixed1", ["bas"], True), ("part1", ["milieu"], False), ("part2", ["haut"], False)]
    joints = [("pivot", "part1", "part2", (W, D / 2, 2 * H), (0, 1, 0), None)]
    m, problems = mechanism(tris, bodies, joints, [("pivot1", 0.0, math.pi / 2)])
    loose = [p for p in problems if "aucune pièce fixe, sa position" in p]
    assert len(loose) == 2 and loose[0].startswith("part1"), problems
    # tied to the fixed part: nothing to say
    m, problems = mechanism(tris, bodies, joints + [("lock", "fixed1", "part1", (0, 0, H),
                                                     (0, 0, 1), None)],
                            [("pivot1", 0.0, math.pi / 2)])
    assert not problems, problems
    # in dynamics a part may fall freely (a domino): no warning
    d = doc.empty()
    for name, uids, fixed in bodies:
        d, _ = doc.add_body(d, uids, fixed, name)
    d = doc.set_simulation(d, active=True)
    _, problems = doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                            triangles_of_group=tris.get)
    assert not [p for p in problems if "aucune pièce fixe, sa position" in p], problems
