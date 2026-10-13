"""Minimum clearance between parts, checked against closed forms."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box, rotation
from motion import collide, solver, samples


def _obb(c, half, R=None):
    return (np.asarray(c, float), np.eye(3) if R is None else R,
            np.asarray(half, float))


def test_face_to_face_gap():
    d, pa, pb = collide.box_distance(_obb((0, 0, 0), (1, 1, 1)),
                                     _obb((2.007, 0.3, -0.2), (1, 1, 1)))
    assert abs(d - 0.007) < 1e-12
    assert abs(pa[0] - 1) < 1e-12 and abs(pb[0] - 1.007) < 1e-12


def test_corner_to_corner_is_exact():
    # gap (a, a, a) between opposite corners: true distance a*sqrt(3);
    # the separating-axis bound alone would only say a
    a = 0.01
    A = _obb((0, 0, 0), (1, 1, 1))
    B = _obb((2 + a, 2 + a, 2 + a), (1, 1, 1))
    d, *_ = collide.box_distance(A, B)
    assert abs(d - a * math.sqrt(3)) < 1e-12
    assert abs(-collide._sat(A, B) - a) < 1e-12


def test_crossed_edges():
    # A turned 45 deg about y: top edge along y at z = sqrt(2)
    # B turned 45 deg about x: bottom edge along x at z = zc - sqrt(2)
    zc = 3.0
    A = _obb((0, 0, 0), (1, 1, 1), rotation((0, math.pi / 4, 0)))
    B = _obb((0, 0, zc), (1, 1, 1), rotation((math.pi / 4, 0, 0)))
    d, pa, pb = collide.box_distance(A, B)
    assert abs(d - (zc - 2 * math.sqrt(2))) < 1e-12
    assert np.linalg.norm(pa - (0, 0, math.sqrt(2))) < 1e-12


def test_corner_to_tilted_face():
    # B turned by th about z: its face normal n = (cos th, sin th, 0);
    # corner (1, 1, 1) of A is the closest point to that face
    th = math.radians(30)
    n = np.array([math.cos(th), math.sin(th), 0])
    gap = 0.02
    h = 0.5
    cB = np.array([1, 1, 0]) + n * (gap + h)
    B = _obb(cB, (h, 3, 3), rotation((0, 0, th)))
    d, pa, pb = collide.box_distance(_obb((0, 0, 0), (1, 1, 1)), B)
    # the whole edge x = y = 1 is equidistant; the gap is exact
    assert abs(d - gap) < 1e-12


def test_overlap_is_negative():
    d, *_ = collide.box_distance(_obb((0, 0, 0), (1, 1, 1)),
                                 _obb((1.9, 0, 0), (1, 1, 1)))
    assert abs(d + 0.1) < 1e-12


def test_drawer_gap_closes_linearly():
    m = Mechanism()
    d = m.body("tiroir", (0, 0.25, 0), [Box((0, 0.25, 0), (0.2, 0.25, 0.08))])
    j = m.slider(m.ground, d, (0, 0, 0), (0, 1, 0), limits=(0, 0.2))
    m.ground.boxes = [Box((0, 0.8, 0), (0.3, 0.05, 0.3), name="tablette")]
    m.drive(j, 0, 0.2)
    for st in solver.sweep(m, 11):
        c = collide.clearances(m, st["x"])[0]
        assert abs(c["distance"] - (0.25 - st["value"])) < 1e-9
    rep = collide.validate(m, 11, jeu_min=0.06)
    assert not rep["ok"]
    assert any("jeu" in p and p.startswith("100 %") for p in rep["problems"])
    assert collide.validate(m, 11, jeu_min=0.04)["ok"]


def test_leaf_corner_towards_wall():
    # leaf 0.6 x 0.018 hinged at x=0, turning down; wall face at x = 0.7
    m = Mechanism()
    leaf = m.body("abattant", (0.3, 0, 0), [Box((0.3, 0, 0), (0.3, 0.2, 0.009))])
    m.ground.boxes = [Box((0.75, 0, -0.5), (0.05, 0.5, 0.5), name="mur")]
    hinge = m.pivot(m.ground, leaf, (0, 0, 0), (0, 1, 0))
    m.drive(hinge, 0, math.radians(80))
    for st in solver.sweep(m, 17):
        th = abs(st["value"])
        expect = 0.7 - (0.6 * math.cos(th) + 0.009 * math.sin(th))
        c = [c for c in collide.clearances(m, st["x"]) if c["b"] == "abattant"][0]
        assert abs(c["distance"] - expect) < 1e-9, (th, c["distance"], expect)


def test_joined_parts_touching_when_drawn_are_the_joint():
    # drop-leaf: leaf and table top touch along the hinge -> not a clash,
    # but a block under the table top on the leaf body IS checked
    m = Mechanism()
    leaf = m.body("abattant", (0.3, 0, 0),
                  [Box((0.3, 0, 0), (0.3, 0.2, 0.009)),
                   Box((0.05, 0, -0.03), (0.02, 0.02, 0.02), name="taquet")])
    m.ground.boxes = [Box((-0.2, 0, 0), (0.2, 0.2, 0.009), name="plateau"),
                      Box((-0.1, 0, -0.08), (0.02, 0.2, 0.02), name="ceinture")]
    m.pivot(m.ground, leaf, (0, 0, 0), (0, 1, 0))
    skip = collide._skipped_box_pairs(m, m.ground, leaf, True)
    assert skip == {(0, 0)}
    assert collide._skipped_box_pairs(m, m.ground, leaf, "body") == \
        {(0, 0), (0, 1), (1, 0), (1, 1)}


def test_chair_smallest_gap_is_the_side_play():
    m, info = samples.chaise_pliante()
    rep = collide.validate(m, 41, jeu_min=0.004)
    assert rep["ok"], rep["problems"]
    assert abs(rep["worst"]["distance"] - 0.005) < 1e-9
    bad = collide.validate(m, 11, jeu_min=0.006)
    assert not bad["ok"]
    pairs = {(c["a"], c["b"]) for st in bad["steps"]
             for c in collide.clearance_violations(m, st["x"], 0.006)}
    assert ("pied_dossier", "pied_assise") in pairs
    assert ("pied_assise", "assise") in pairs


def test_chair_seat_to_backrest_board_folded():
    for recul in (0.03, 0.04, 0.05):
        m, info = samples.chaise_pliante(recul_dossier=recul)
        x, ok, _ = solver.solve(m, info["phi_fold"])
        assert ok
        g, seat = m.ground, info["bodies"]["assise"]
        board = [b for b in g.boxes if b.name == "dossier"][0]
        d, *_ = collide.box_distance(
            collide.world_obb(g, m.pose(x, g), board),
            collide.world_obb(seat, m.pose(x, seat), seat.boxes[0]))
        assert abs(d - (recul - info["seat_thickness"] / 2)) < 1e-9


def test_chair_flush_board_reported_by_validate():
    m, info = samples.chaise_pliante(recul_dossier=0.02)
    rep = collide.validate(m, 21)
    assert any("collision pied_dossier / assise" in p for p in rep["problems"])


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
