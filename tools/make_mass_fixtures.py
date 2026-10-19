"""STEP solids and their mass properties computed by an independent CAD
kernel (CadQuery / OpenCascade GProp), for the mass tests
(tests/data/masse_*.step, masse_facts.json). Unit density, metres.

    pip install cadquery && python tools/make_mass_fixtures.py
"""
import json
from pathlib import Path
import cadquery as cq
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

OUT = Path(__file__).resolve().parents[1] / "tests" / "data"
facts = {}


def props(shape):
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, g)
    c = g.CentreOfMass()
    M = g.MatrixOfInertia()
    k = 1e-3                                   # mm -> m
    return dict(volume=g.Mass() * k ** 3, centre=[c.X() * k, c.Y() * k, c.Z() * k],
                inertia=[[M.Value(i, j) * k ** 5 for j in (1, 2, 3)] for i in (1, 2, 3)])


solids = {
    # an L bracket, off the origin, planes only
    "equerre": (cq.Workplane("XY").polyline([(0, 0), (80, 0), (80, 10), (10, 10), (10, 60), (0, 60)])
                .close().extrude(25).translate((30, -20, 15))),
    # a tube (annular end faces: faces with holes), axis along x
    "tube": cq.Workplane("YZ").circle(20).circle(15).extrude(120).translate((-40, 10, 5)),
    # a stepped shaft with a bore, tilted
    "arbre": (cq.Workplane("XY").circle(12).extrude(30).faces(">Z").workplane().circle(8).extrude(40)
              .faces(">Z").workplane().hole(6, 50)
              .rotate((0, 0, 0), (1, 1, 0), 35).translate((5, 5, 5))),
}
for name, wp in solids.items():
    cq.exporters.export(wp, str(OUT / f"masse_{name}.step"))
    facts[name] = props(wp.val())
(OUT / "masse_facts.json").write_text(json.dumps(facts, indent=1))
print({k: round(v["volume"] * 1e9) for k, v in facts.items()})
