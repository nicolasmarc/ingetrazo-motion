"""Collisions on the real surfaces (motion.contact): exact closest points,
a hollow casing that boxes cannot see into, a gap kept and a gap closed,
parts resting against each other at their joint."""
import math, os, sys
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from motion import contact, collide, document as doc
from test_mass import box


def test_closest_points_on_triangles_against_brute_force():
    rng = np.random.default_rng(3)
    n = 400
    A, B, C = (rng.normal(size=(n, 3)) for _ in range(3))
    P = 2 * rng.normal(size=(n, 3))
    got = contact.closest_on_triangles(P, A, B, C)
    # brute force: a fine barycentric grid
    g = np.linspace(0, 1, 161)
    u, v = np.meshgrid(g, g)
    keep = u + v <= 1
    u, v = u[keep], v[keep]
    for i in range(0, n, 20):
        Q = A[i] + u[:, None] * (B[i] - A[i]) + v[:, None] * (C[i] - A[i])
        best = np.linalg.norm(Q - P[i], axis=1).min()
        assert abs(np.linalg.norm(got[i] - P[i]) - best) < 2e-2 * max(best, 0.1)
        assert np.linalg.norm(got[i] - P[i]) <= best + 1e-12


def hollow(lo, hi, wall):
    """A closed box with a box-shaped hole inside (outward on both skins)."""
    outer = box(lo, hi)
    inner = box(np.add(lo, wall), np.subtract(hi, wall))[:, ::-1]   # turned inward
    return np.concatenate([outer, inner])


def mechanism(tris, bodies, joints, drivers):
    d = doc.empty()
    for name, uids, fixed in bodies:
        d, _ = doc.add_body(d, uids, fixed, name)
    for kind, a, b, p, ax, lim in joints:
        d, _ = doc.add_joint(d, kind, a, b, point=p, axis=ax, limits=lim)
    for j, lo, hi in drivers:
        d = doc.add_driver(d, j, lo, hi)
    return doc.build(d, lambda u: tris[u].reshape(-1, 3), lambda u: u,
                     triangles_of_group=tris.get)


def test_rod_inside_a_hollow_casing():
    # a rod slides inside a casing (walls 5 mm): the casing's box swallows
    # it, its surfaces do not; pushed on, it meets the far wall
    tris = {"carter": hollow((0, 0, 0), (0.2, 0.06, 0.06), 0.005),
            "tige": box((0.02, 0.025, 0.025), (0.08, 0.035, 0.035))}
    m, problems = mechanism(tris, [("carter", ["carter"], True), ("tige", ["tige"], False)],
                            [("slider", "carter", "tige", (0.05, 0.03, 0.03), (1, 0, 0), None)],
                            [("glissière1", 0.0, 0.1)])
    assert not problems and m.surfaces is not None
    rep = collide.validate(m, 21, 0.02)              # gaps read up to 20 mm away
    hit = [r["t"] for r in rep["steps"] if r["hits"]]
    # the rod's end (x = 0.08 + s) meets the wall at x = 0.195: s = 0.115 > 0.1
    assert not hit, rep["problems"]
    assert all("jeu" in p for p in rep["problems"])
    gap = rep["worst"]
    assert abs(gap["distance"] - 0.015) < 1e-6, gap          # at the end: 195 - 180 mm
    assert gap["box_a"] == "carter" and gap["box_b"] == "tige"
    # (its box holds the rod whole: boxes could only skip the pair)
    bx = m.ground.boxes[0]
    assert np.all(np.abs(np.array([0.05, 0.03, 0.03]) - bx.center) < bx.half)
    # pushed 130 mm: it goes through the wall from 115 mm on
    m, _ = mechanism(tris, [("carter", ["carter"], True), ("tige", ["tige"], False)],
                     [("slider", "carter", "tige", (0.05, 0.03, 0.03), (1, 0, 0), None)],
                     [("glissière1", 0.0, 0.13)])
    rep = collide.validate(m, 27, 0.0)
    hit = [round(r["value"], 4) for r in rep["steps"] if r["hits"]]
    assert hit and abs(hit[0] - 0.12) < 1e-9, hit            # first step past 115 mm


def test_resting_contact_is_the_joint():
    # a lid on a box, hinged along an edge: lying on it is no collision,
    # opening is free; turned the other way it goes into the box
    tris = {"caisse": box((0, 0, 0), (0.3, 0.2, 0.1)),
            "couvercle": box((0, 0, 0.1), (0.3, 0.2, 0.11))}
    for hi, bad in ((math.radians(100), False), (math.radians(-20), True)):
        m, _ = mechanism(tris, [("caisse", ["caisse"], True), ("couvercle", ["couvercle"], False)],
                         [("pivot", "caisse", "couvercle", (0.0, 0.1, 0.1), (0, -1, 0), None)],
                         [("pivot1", 0.0, hi)])
        rep = collide.validate(m, 11, 0.0)
        assert bool(rep["problems"]) == bad, (hi, rep["problems"])
        if bad:
            assert rep["steps"][0]["hits"] == []                 # not at rest


def test_gap_kept_by_a_sliding_face():
    # a block slides along a wall 0.12 mm away: no collision, the gap read
    # exactly (points alone are 0.8 mm apart here)
    tris = {"mur": box((0, 0, 0), (0.3, 0.01, 0.1)),
            "bloc": box((0.0, 0.01012, 0.0), (0.05, 0.04, 0.05))}
    m, _ = mechanism(tris, [("mur", ["mur"], True), ("bloc", ["bloc"], False)],
                     [("slider", "mur", "bloc", (0.0, 0.02, 0.02), (1, 0, 0), None)],
                     [("glissière1", 0.0, 0.2)])
    rep = collide.validate(m, 9, 0.0)
    assert rep["ok"], rep["problems"]
    # resting against the wall at the start: the same face sliding along is
    # left out (it is the guide); a wall 3 mm off is read exactly
    tris["bloc"] = box((0.0, 0.013, 0.0), (0.05, 0.04, 0.05))
    m, _ = mechanism(tris, [("mur", ["mur"], True), ("bloc", ["bloc"], False)],
                     [("slider", "mur", "bloc", (0.0, 0.02, 0.02), (1, 0, 0), None)],
                     [("glissière1", 0.0, 0.2)])
    rep = collide.validate(m, 9, 0.004)
    assert abs(rep["worst"]["distance"] - 0.003) < 1e-9
    assert any("jeu 3.0 mm" in p for p in rep["problems"])


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
