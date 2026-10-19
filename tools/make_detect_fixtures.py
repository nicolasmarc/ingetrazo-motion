"""STEP assemblies for the joint-detection tests (tests/data/detect_*.step),
written by an independent CAD kernel (CadQuery / OpenCascade). Test data
only; the tests read the committed files.

    pip install cadquery && python tools/make_detect_fixtures.py
"""
import json, math
from pathlib import Path
import cadquery as cq

OUT = Path(__file__).resolve().parents[1] / "tests" / "data"
facts = {}


def bar(p, q, z0, t, w=20.0):
    """Flat link from p to q (mm), thickness t from z0."""
    L = math.dist(p, q)
    ang = math.degrees(math.atan2(q[1] - p[1], q[0] - p[0]))
    return (cq.Workplane("XY").workplane(offset=z0).center((p[0] + q[0]) / 2, (p[1] + q[1]) / 2)
            .slot2D(L + w, w, ang).extrude(t))


def hole(wp, c, d, z0, h):
    return wp.cut(cq.Workplane("XY").workplane(offset=z0).center(*c).circle(d / 2).extrude(h))


def pin(wp, c, d, z0, h):
    return wp.union(cq.Workplane("XY").workplane(offset=z0).center(*c).circle(d / 2).extrude(h))


# 1. crank-rocker four-bar, axes along z, pins integral to the links
A, B, C, D = (0, 0), (0, 60), (200, 90), (200, 0)
socle = cq.Workplane("XY").center(100, 0).rect(260, 40).extrude(10)
socle = hole(hole(socle, A, 10.2, 0, 10), D, 10.2, 0, 10)
crank = pin(hole(bar(A, B, 10, 8), B, 10.2, 10, 8), A, 10, 0, 10)
rocker = pin(hole(bar(D, C, 10, 8), C, 10.2, 10, 8), D, 10, 0, 10)
coupler = pin(pin(bar(B, C, 18, 8), B, 10, 11, 7), C, 10, 11, 7)   # pins end 1 mm above the base
asm = cq.Assembly(name="Quadrilatere")
for name, s, col in (("Socle", socle, (0.4, 0.4, 0.45)), ("Manivelle", crank, (0.9, 0.5, 0.2)),
                     ("Bielle", coupler, (0.3, 0.5, 0.9)), ("Balancier", rocker, (0.2, 0.7, 0.3))):
    asm.add(s, name=name, color=cq.Color(*col))
asm.save(str(OUT / "detect_quadrilatere.step"))
facts["quadrilatere"] = {"A": A, "B": B, "C": C, "D": D}

# 2. a block sliding in a U channel (exact contact on three faces), along x
rail = (cq.Workplane("XY").box(300, 60, 10, centered=(True, True, False))
        .union(cq.Workplane("XY").center(0, 25).box(300, 10, 30, centered=(True, True, False)))
        .union(cq.Workplane("XY").center(0, -25).box(300, 10, 30, centered=(True, True, False))))
block = cq.Workplane("XY").workplane(offset=10).box(60, 40, 30, centered=(True, True, False))
asm = cq.Assembly(name="Coulisse")
asm.add(rail, name="Rail")
asm.add(block, name="Chariot")
asm.save(str(OUT / "detect_coulisse.step"))


# 3. two spur gears (polyline teeth) on shafts fixed to a plate
def gear(z, rp, bore, thick, z0, centre):
    m = 2 * rp / z
    ra, rf = rp + m, rp - m
    pts = []
    for k in range(z):
        a = 2 * math.pi * k / z
        for da, r in ((-0.35, rf), (-0.15, ra), (0.15, ra), (0.35, rf)):
            t = a + da * 2 * math.pi / z
            pts.append((centre[0] + r * math.cos(t), centre[1] + r * math.sin(t)))
    g = cq.Workplane("XY").workplane(offset=z0).polyline(pts).close().extrude(thick)
    return hole(g, centre, bore, z0, thick)


z1, z2, mod = 20, 30, 2.0
c1, c2 = (0, 0), (mod * (z1 + z2) / 2, 0)
plate = cq.Workplane("XY").center(25, 0).rect(140, 90).extrude(8)
plate = pin(pin(plate, c1, 8, 8, 14), c2, 8, 8, 14)
asm = cq.Assembly(name="Engrenages")
asm.add(plate, name="Platine")
asm.add(gear(z1, mod * z1 / 2, 8.1, 8, 10, c1), name="Pignon")
asm.add(gear(z2, mod * z2 / 2, 8.1, 8, 10, c2), name="Roue")
asm.save(str(OUT / "detect_engrenages.step"))
facts["engrenages"] = {"z1": z1, "z2": z2, "c1": c1, "c2": c2}

# 4. a hinge: two leaves and a separate pin, press-fitted in the left
#    leaf's knuckles (bore = pin), free in the right one's (0.2 mm play)
def leaf(x0, x1, knuckles, bore):
    w = cq.Workplane("XY").center((x0 + x1) / 2, 0).rect(abs(x1 - x0), 60).extrude(3)
    edge = x1 if x1 < 0 else x0                 # the side at the axis
    for z0 in knuckles:
        k = (cq.Workplane("XZ").workplane(offset=-z0 - 15).center(0, 4.5).circle(4.5)
             .extrude(15))
        tab = (cq.Workplane("XY").center(edge / 2, z0 + 7.5).rect(abs(edge), 15).extrude(3))
        w = w.union(k).union(tab)
    return w.cut(cq.Workplane("XZ").workplane(offset=-40).center(0, 4.5).circle(bore / 2)
                 .extrude(80))


left = leaf(-50, -4.6, (-30, 0), 6.0)
right = leaf(4.6, 40, (-15, 15), 6.2)
pin_ = cq.Workplane("XZ").workplane(offset=-30).center(0, 4.5).circle(3.0).extrude(60)
asm = cq.Assembly(name="Charniere")
asm.add(left, name="Aile fixe")
asm.add(right, name="Aile mobile")
asm.add(pin_, name="Axe")
asm.save(str(OUT / "detect_charniere.step"))

(OUT / "detect_facts.json").write_text(json.dumps(facts, indent=1))
print("ok")
