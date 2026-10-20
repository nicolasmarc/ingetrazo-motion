"""STEP reader, against files written by an independent CAD kernel.

tests/data/*.step and facts.json come from tools/make_step_fixtures.py
(CadQuery / OpenCascade): volumes, bounding boxes and placements there are
the kernel's, not ours.
"""
import json, math, os, sys
from collections import Counter
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import stepread

DATA = os.path.join(os.path.dirname(__file__), "data")
FACTS = json.load(open(os.path.join(DATA, "facts.json")))
FINE = dict(tol=0.00002, max_angle=math.radians(4))


def _read(name, **kw):
    return stepread.read(os.path.join(DATA, name + ".step"), **kw)


def _open_edges(T):
    P = np.round(T.reshape(-1, 3) * 1e9).astype(np.int64)
    keys = {}
    ids = np.array([keys.setdefault(tuple(q), len(keys)) for q in P]).reshape(-1, 3)
    ec = Counter()
    for a, b, c in ids:
        if len({a, b, c}) < 3:
            continue
        for e in ((a, b), (b, c), (c, a)):
            ec[tuple(sorted(e))] += 1
    return sum(1 for v in ec.values() if v != 2)


def _centroid(T):
    v = np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])) / 6
    return (v[:, None] * T.sum(axis=1) / 4).sum(0) / v.sum()


def test_parse_complex_strings_comments():
    txt = """ISO-10303-21; HEADER; ENDSEC; DATA;
    /* a comment; with a semicolon */
    #1 = PRODUCT('it''s','name;x','',(#2));
    #2 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );
    #3 = MEASURE_REPRESENTATION_ITEM('',LENGTH_MEASURE(1.5E-02),#2);
    ENDSEC; END-ISO-10303-21;"""
    e = stepread.parse(txt)
    assert e[1]["PRODUCT"][0] == "it's" and e[1]["PRODUCT"][1] == "name;x"
    assert set(e[2]) == {"LENGTH_UNIT", "NAMED_UNIT", "SI_UNIT"}
    assert e[2]["SI_UNIT"] == ["MILLI", "METRE"]
    assert e[3]["MEASURE_REPRESENTATION_ITEM"][1] == ("LENGTH_MEASURE", [0.015])


def test_block_is_exact():
    asm = _read("bloc")
    assert asm.unit == 1e-3 and len(asm.parts) == 1
    T = asm.parts[0].triangles
    assert len(T) == 12
    assert abs(stepread.volume(T) * 1e9 - FACTS["bloc"]["volume"]) < 1e-6
    lo, hi = T.reshape(-1, 3).min(0) * 1e3, T.reshape(-1, 3).max(0) * 1e3
    assert np.allclose(lo, FACTS["bloc"]["bbox"][0]) and np.allclose(hi, FACTS["bloc"]["bbox"][1])


def test_volumes_planes_cylinders_cones_spheres_tori_bsplines():
    for name in ("platine", "pignon", "galet", "raccord"):
        T = _read(name, **FINE).parts[0].triangles
        v, ref = stepread.volume(T) * 1e9, FACTS[name]["volume"]
        assert abs(v - ref) / ref < 0.003, (name, v, ref)


def test_closed_parts_are_watertight():
    for name in ("bloc", "platine", "pignon", "galet"):
        T = _read(name).parts[0].triangles
        assert _open_edges(T) == 0, name


def test_bounding_box_of_a_plate_with_boss():
    T = _read("platine").parts[0].triangles * 1e3
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    ref = np.array(FACTS["platine"]["bbox"])
    assert np.abs(lo - ref[0]).max() < 0.25 and np.abs(hi - ref[1]).max() < 0.25


def test_finer_tolerance_converges():
    ref = FACTS["galet"]["volume"]
    errs = [abs(stepread.volume(_read("galet", tol=t).parts[0].triangles) * 1e9 - ref) / ref
            for t in (0.0004, 0.0001, 0.00002)]
    assert errs[0] > errs[1] > errs[2]


def test_assembly_tree_placements_colours():
    asm = _read("assemblage", **FINE)
    by = {p.path[-1]: p for p in asm.parts}
    assert set(by) == {"socle", "levier_g", "levier_d"}
    assert all(p.path[0] == "mecanisme" for p in asm.parts)
    assert np.allclose(by["socle"].color, (0.6, 0.6, 0.6), atol=1e-6)
    c_local = np.array(FACTS["assemblage"]["lever_center_local"])
    for name in ("levier_g", "levier_d"):
        f = FACTS["assemblage"]["parts"][name]
        a = math.radians(f["angle"])
        R = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
        expect = R @ c_local + np.array(f["center"])
        got = _centroid(by[name].triangles * 1e3)
        assert np.linalg.norm(got - expect) < 0.02, (name, got, expect)
        v = stepread.volume(by[name].triangles) * 1e9
        assert abs(v - f["volume"]) / f["volume"] < 0.003


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)


# -- faces that came out wrong on a real model (a mechanical spider) ----------------

def test_plate_with_rows_of_holes_lined_up():
    # a plate with two rows of rounded holes on the same lines: a cut from a
    # hole to the outline must not graze the corners of the next hole
    from motion import stepread as S
    def rounded(x0, y0, w, h, r, n=6):
        pts = []
        for cx, cy, a0 in ((x0 + w - r, y0 + r, -90), (x0 + w - r, y0 + h - r, 0),
                           (x0 + r, y0 + h - r, 90), (x0 + r, y0 + r, 180)):
            for k in range(n + 1):
                t = np.radians(a0 + 90 * k / n)
                pts.append((cx + r * np.cos(t), cy + r * np.sin(t)))
        return np.array(pts)
    outer = np.array([(-60, -10), (60, -10), (60, 10), (-60, 10)], float)
    holes = [rounded(x, y, 12, 7, 1.5) for x in range(-56, 45, 15) for y in (-8, 1)]
    holes += [np.array([(x + 0.8 * np.cos(t), 0.8 * np.sin(t))
                        for t in np.linspace(0, 2 * np.pi, 13)[:-1]])
              for x in np.arange(-42.5, 33, 15)]                 # in the gaps, on the axis
    uvs = [outer] + [h[::-1] for h in holes]
    class Flat:
        curved = False
    tris, pts = S._triangulate_uv(uvs, Flat())
    T = pts[np.asarray(tris)]
    area = 0.5 * np.abs((T[:, 1, 0] - T[:, 0, 0]) * (T[:, 2, 1] - T[:, 0, 1])
                        - (T[:, 2, 0] - T[:, 0, 0]) * (T[:, 1, 1] - T[:, 0, 1])).sum()
    want = abs(S._area2(outer)) / 2 - sum(abs(S._area2(h)) / 2 for h in holes)
    assert abs(area - want) < 1e-9 * want, (area, want)


def test_closed_bspline_surface_face_across_its_seam():
    # a square tube as a degree-1 b-spline closed in u; a face on it that
    # straddles the seam (u = 0) comes out as the strip it is, not the
    # whole tube the other way round
    from motion import stepread as S
    ring = [(1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1), (1, 0)]
    pts = {}
    grid = []
    for i, (x, y) in enumerate(ring):
        row = []
        for j, z in enumerate((0.0, 2.0)):
            pts[100 * i + j + 1] = np.array([x, y, z], float)
            row.append(100 * i + j + 1)
        grid.append(row)
    class Rd:
        def point(self, ref):
            return pts[int(ref)]
    n = len(ring)
    a = {"B_SPLINE_SURFACE_WITH_KNOTS": ["", 1, 1, grid, "UNSPECIFIED", "T", "F", "F",
                                         [2] + [1] * (n - 2) + [2], [2, 2],
                                         list(range(n)), [0.0, 1.0], "UNSPECIFIED"]}
    Sf = S._BSplineSurface(Rd(), a)
    assert Sf.closed_u and not Sf.closed_v
    # the face: from the middle of the side at y = -1 ... round x = 1 ...
    # to the middle of the side at y = +1, z from 0.5 to 1.5
    side = [(0.0, -1.0), (1, -1), (1, 0), (1, 1), (0.0, 1.0)]
    lo = [np.array([x, y, 0.5]) for x, y in side]
    hi = [np.array([x, y, 1.5]) for x, y in side[::-1]]
    L = np.array(lo + hi)
    uv = Sf.to_uv_loop(L)
    assert np.ptp(uv[:, 0]) < 0.5 * (Sf.u1 - Sf.u0) * Sf.su        # one side of the seam
    tris, P = S._triangulate_uv([uv], Sf)
    T = P[np.asarray(tris)]
    area = 0.5 * np.abs((T[:, 1, 0] - T[:, 0, 0]) * (T[:, 2, 1] - T[:, 0, 1])
                        - (T[:, 2, 0] - T[:, 0, 0]) * (T[:, 1, 1] - T[:, 0, 1])).sum()
    # 4 knot spans round x = 1 (uv is scaled by the mean length per span),
    # 1 high
    assert abs(area - 4.0 * Sf.su * 1.0) < 1e-6, area
    mid = Sf.from_uv(T.mean(axis=1))
    assert np.all(mid[:, 0] > -1e-9), mid          # all on the x >= 0 half


def _mesh(Sf, loops, tol=0.002):
    """The reader's own steps for one face: loops -> uv -> triangles."""
    from motion import stepread as S
    if hasattr(Sf, "on_axis"):
        loops = [S._split_axis_points(L, Sf) for L in loops]
    uvs = [Sf.to_uv_loop(L) for L in loops]
    tris, P = S._triangulate_uv(uvs, Sf)
    offs = np.cumsum([0] + [len(u) for u in uvs])
    bnd = [list(range(offs[k], offs[k + 1])) for k in range(len(uvs))]
    P, T = S._refine(P, tris, Sf, tol, math.radians(10), bnd)
    X = Sf.from_uv(np.asarray(P))
    X[:offs[-1]] = np.concatenate(loops)
    T3 = X[np.asarray(T)]
    return 0.5 * np.linalg.norm(np.cross(T3[:, 1] - T3[:, 0], T3[:, 2] - T3[:, 0]), axis=1).sum()


def test_half_cone_through_its_apex():
    # a countersink's half: from the apex out to half a circle and back.
    # The file's reference radius is far away (866 at the placement): the
    # angle must be scaled where the face is, and the apex met twice
    from motion import stepread as S
    half = math.radians(60)
    r_ref, z_ref = 866.0, 500.0                      # radius 866 at z = 500 ...
    ax = (np.array([0, 0, z_ref]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]))
    # ... so the apex is at z = 500 - 866 / tan 60 = 0
    apex = np.array([0, 0, z_ref - r_ref / math.tan(half)])
    h = 0.8
    R = h * math.tan(half)
    arc = [np.array([R * math.cos(t), R * math.sin(t), apex[2] + h]) for t in np.linspace(0, math.pi, 25)]
    loop = np.array([apex] + arc)
    Sf = S._Cone(ax, r_ref, half, [loop])
    want = math.pi * R * math.hypot(R, h) / 2
    got = _mesh(Sf, [loop])
    assert abs(got - want) < 0.01 * want, (got, want)


def test_surface_of_revolution_follows_its_profile():
    # a quarter circle turned about z (a rounded rim): the area is the
    # profile's own, not that of the line between its ends
    from motion import stepread as S
    r0, a = 5.0, 2.0
    prof = np.array([[r0 + a * math.cos(t), 0.0, a * math.sin(t)]
                     for t in np.linspace(0, math.pi / 2, 200)])
    t = np.linspace(0, 2 * math.pi, 97)[:-1]
    loops = [np.column_stack([(r0 + a) * np.cos(t), (r0 + a) * np.sin(t), 0 * t]),
             np.column_stack([r0 * np.cos(-t), r0 * np.sin(-t), a + 0 * t])]
    Sf = S._Revolution(np.zeros(3), np.array([0, 0, 1.0]), prof, loops)
    want = 2 * math.pi * a * (r0 * math.pi / 2 + a)          # Pappus
    got = _mesh(Sf, loops, tol=0.01)
    # facets a little long and thin over the bend: a few % over at most
    assert abs(got - want) < 0.05 * want, (got, want)
    # without the profile (boundary points only) it would be the cone
    # between the two circles, far off
    cone = math.pi * (r0 + r0 + a) * math.hypot(a, a)
    assert abs(got - want) < 0.5 * abs(cone - want)
