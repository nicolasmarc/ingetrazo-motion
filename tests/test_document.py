"""Document data -> mechanism (what the panel stores), no Qt needed."""
import json, math, sys, os, itertools
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import document as doc, solver, collide
from motion.model import rotation


def _corners(c, half, R=np.eye(3)):
    return [np.asarray(c) + R @ (np.array(s) * half)
            for s in itertools.product((-1, 1), repeat=3)]


def test_fit_box_recovers_a_turned_box():
    R = rotation((0.2, -0.5, 0.9))
    pts = _corners((1, 2, 3), np.array([0.3, 0.1, 0.02]), R)
    bx = doc.fit_box(pts)
    assert np.linalg.norm(bx.center - (1, 2, 3)) < 1e-12
    assert np.allclose(sorted(bx.half), [0.02, 0.1, 0.3], atol=1e-12)


def test_drop_leaf_from_groups_and_an_edge():
    groups = {"g-table": _corners((-0.2, 0, 0), np.array([0.2, 0.2, 0.009])),
              "g-leaf": _corners((0.3, 0, 0), np.array([0.3, 0.2, 0.009]))}
    d = doc.empty()
    d, fixed = doc.add_body(d, ["g-table"], True)
    d, leaf = doc.add_body(d, ["g-leaf"], False)
    hinge_edge = (np.array([0, -0.2, 0.0]), np.array([0, 0.2, 0.0]))
    d, piv = doc.add_joint(d, "pivot", fixed, leaf, point=hinge_edge[0],
                           axis=hinge_edge[1] - hinge_edge[0],
                           limits=(0, math.pi / 2))
    d = doc.set_driver(d, piv, 0, math.pi / 2)
    d = json.loads(json.dumps(d))                      # survives the .igz
    m, problems = doc.build(d, groups.get)
    assert not problems, problems
    assert solver.dof(m) == 1
    rep = collide.validate(m, 11)
    assert rep["ok"], rep["problems"]
    leaf_b = m.bodies[0]
    tr = doc.trajectory(m, rep["steps"], leaf_b, (0.6, 0, 0))
    assert np.allclose(np.hypot(tr[:, 0], tr[:, 2]), 0.6, atol=1e-8)


def test_group_moved_to_another_body():
    d = doc.empty()
    d, a = doc.add_body(d, ["u1", "u2"], False)
    d, b = doc.add_body(d, ["u2"], False)
    assert d["bodies"][0]["groups"] == ["u1"] and d["bodies"][1]["groups"] == ["u2"]
    d, j = doc.add_joint(d, "slider", a, b, point=(0, 0, 0), axis=(0, 0, 2))
    d = doc.set_driver(d, j, 0, 0.1)
    d = doc.remove(d, a)
    assert d["joints"] == [] and d["driver"] is None


def test_missing_pieces_are_reported():
    d = doc.empty()
    d, a = doc.add_body(d, ["gone"], False)
    m, problems = doc.build(d, lambda uid: None)
    assert any("aucune pièce fixe" in p for p in problems)
    assert any("aucun groupe" in p for p in problems)


def test_rail_and_rollers_from_selected_edges():
    # quarter arc drawn as 12 segments, then a straight; two roller marks
    R, H = 0.38, 2.0
    pts = [np.array([0, 0, 0.0]), np.array([0, 0, H])]
    for k in range(1, 13):
        a = k * math.pi / 24
        pts.append(np.array([R - R * math.cos(a), 0, H + R * math.sin(a)]))
    pts.append(np.array([R + 2.0, 0, H + R]))
    segs = [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    segs = segs[::-1]                                   # any order
    segs += [((-0.05, 0, 0.0), (0.0, 0, 0.0)), ((0.0, 0, 0.5), (0.05, 0, 0.5))]
    rng = np.random.default_rng(3)
    for _ in range(6):                                  # selection order free
        order = rng.permutation(len(segs))
        sh = [segs[i] if rng.random() < .5 else segs[i][::-1] for i in order]
        rail, rollers = doc.rail_and_rollers(sh)
        assert len(rail) == len(pts) and len(rollers) == 2
        assert np.allclose(rail[0], (0, 0, 0)) and np.allclose(rail[-1], pts[-1])
        assert np.allclose(rollers[0], (0, 0, 0)) and np.allclose(rollers[1], (0, 0, 0.5))
    path = doc.path_from_points(rail)
    assert abs(path.length - (H + 12 * 2 * R * math.sin(math.pi / 48) + 2.0)) < 1e-9


def test_nearest_body():
    boxes = {"a": [doc.fit_box(_corners((0, 0, 0), np.array([1, 1, 1])))],
             "b": [doc.fit_box(_corners((5, 0, 0), np.array([1, 1, 1])))]}
    assert doc.nearest_body((3.9, 0, 0), boxes) == "b"
    assert doc.nearest_body((3.9, 0, 0), boxes, exclude=("b",)) == "a"


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
