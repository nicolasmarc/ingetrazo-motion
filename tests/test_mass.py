"""Materials and mass properties: closed forms, and solids whose volume,
centre of gravity and inertia were computed by an independent CAD kernel
(OpenCascade GProp, tests/data/masse_facts.json)."""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))
from motion import massprops, materials, stepread, document as doc

FACTS = json.load(open(os.path.join(HERE, "data", "masse_facts.json")))


def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    c = lambda i: np.where([i & 1, i & 2, i & 4], hi, lo)
    T = []
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        p = [c(i) for i in q]
        T += [[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
    return np.array(T)


def test_box_closed_form():
    a, b, h, rho = 0.2, 0.1, 0.05, 710.0                 # an oak block
    m, c, I, v = massprops.of_triangles(box((1, 2, 3), (1 + a, 2 + b, 3 + h)), rho)
    assert abs(m - rho * a * b * h) < 1e-9
    assert np.allclose(c, (1 + a / 2, 2 + b / 2, 3 + h / 2), atol=1e-12)
    assert np.allclose(np.diag(I), [m / 12 * (b * b + h * h), m / 12 * (a * a + h * h),
                                    m / 12 * (a * a + b * b)], rtol=1e-9)
    assert np.allclose(I - np.diag(np.diag(I)), 0, atol=1e-12)


def test_two_halves_make_the_whole():
    whole = massprops.of_triangles(box((0, 0, 0), (0.4, 0.1, 0.1)), 7850)
    halves = [massprops.of_triangles(box((x, 0, 0), (x + 0.2, 0.1, 0.1)), 7850)[:3]
              for x in (0.0, 0.2)]
    m, c, I = massprops.combine(halves)
    assert abs(m - whole[0]) < 1e-9 and np.allclose(c, whole[1]) and np.allclose(I, whole[2])


def test_solids_against_an_independent_kernel():
    for name, f in FACTS.items():
        a = stepread.read(os.path.join(HERE, "data", f"masse_{name}.step"), tol=2e-5)
        T = np.concatenate([p.triangles for p in a.parts])
        m, c, I, v = massprops.of_triangles(T, 1.0)
        Iref = np.array(f["inertia"])
        planar = name == "equerre"
        tol = 1e-9 if planar else 3e-3                   # curved faces: facets of 0.02 mm
        assert abs(v / f["volume"] - 1) < tol, (name, v, f["volume"])
        assert np.linalg.norm(c - f["centre"]) < (1e-9 if planar else 1e-5), name
        assert np.abs(I - Iref).max() < tol * np.abs(Iref).max() + 1e-15, name


def test_materials_named_in_part_names():
    g = materials.guess
    assert g("Pied chêne") == "chene" and g("Plateau Hetre massif") == "hetre"
    assert g("Steel_brushed") == "acier" and g("Axe inox") == "inox"
    assert g("Contreplaqué 18") == "contreplaque" and g("Panneau MDF") == "mdf"
    assert g("Bague laiton") == "laiton" and g("Patin POM") == "pom"
    assert g("Pignon") is None and g("Pinces") is None     # "pin" only as a word
    lib = materials.library({"matieres": {"chene": {"density": 750},
                                          "doc:iroko": {"name": "Iroko", "density": 640}}})
    assert lib["chene"]["density"] == 750 and lib["chene"]["name"] == "Chêne"
    assert lib["doc:iroko"]["density"] == 640 and lib["hetre"]["density"] == 720


def test_document_masses_per_body():
    tri = {"a": box((0, 0, 0), (0.1, 0.1, 0.1)), "b": box((0.1, 0, 0), (0.2, 0.1, 0.1)),
           "c": box((0, 0, 0.1), (0.1, 0.1, 0.2))}
    d = doc.empty()
    d, _ = doc.add_body(d, ["a", "b"], True, "bâti")
    d, _ = doc.add_body(d, ["c"], False, "bras")
    d = doc.set_material(d, ["a"], "acier")
    d = doc.set_material(d, ["b"], "chene")
    ms = doc.mass_properties(d, tri.get)
    assert abs(ms["bâti"]["mass"] - (7.85 + 0.71)) < 1e-9
    assert ms["bâti"]["materials"] == ["Acier", "Chêne"] and ms["bâti"]["missing"] == []
    xc = (7.85 * 0.05 + 0.71 * 0.15) / 8.56
    assert abs(ms["bâti"]["com"][0] - xc) < 1e-12
    assert ms["bras"]["mass"] == 0 and ms["bras"]["missing"] == ["c"]
    d = doc.add_material(d, "doc:iroko", {"name": "Iroko", "density": 640})
    d = doc.set_material(d, ["c"], "doc:iroko")
    m, problems = doc.build(d, lambda u: tri[u].reshape(-1, 3), triangles_of_group=tri.get)
    arm = m.bodies[0]
    assert abs(arm.mass - 0.64) < 1e-12 and np.allclose(arm.com, (0.05, 0.05, 0.15))


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
