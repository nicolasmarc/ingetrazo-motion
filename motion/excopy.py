"""Copy a ready-made example into the document (pure, no Qt).

The examples are built in code and drawn over the view; copied, each part
becomes real IngeTrazo groups (shaded, saved in the .igz, editable) and the
joints, motors and dynamics settings become the document's mechanism, as
if they had been made by hand in the panel. The example itself stays as it
is (its tests check it against closed-form results).

Left out: the two chains (their links follow a chain calculation of their
own, which joints between parts cannot express) and the handcar (its crank
and rod are drawn in one plane, which the example allows on its boxes but
a real model would rightly report as a collision at every step).
"""
import math
import numpy as np

from . import document as doc
from .lang import T as _T

NOT_COPYABLE = ("chaine", "chaine_souple", "draisine")

# a part without a colour of its own: frame grey, then warm wood tones
_GREY = (140, 140, 146)
_TONES = ((196, 106, 36), (150, 90, 40), (180, 140, 80), (120, 80, 50), (210, 160, 100))


def box_faces(box):
    """The 6 faces (corner arrays, reference pose) of a ``model.Box``."""
    c, R, h = box.center, box.R, box.half
    k = c + (np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                       for sz in (-1, 1)]) * h) @ R.T
    return [k[list(f)] for f in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                                 (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))]


def parts(mech, info):
    """[(body name, fixed, [(group name, faces, rgb 0-255)])]: one group per
    box (so the copied part keeps the example's collision boxes), or one
    group with the example's drawing when it has one (gears, wheels)."""
    polys = (info or {}).get("polys") or {}
    out = []
    tone = 0
    for body in [mech.ground] + list(mech.bodies):
        if body.color is not None:
            col = tuple(body.color)
        elif body is mech.ground:
            col = _GREY
        else:
            col, tone = _TONES[tone % len(_TONES)], tone + 1
        if body.name in polys and len(polys[body.name]):
            solids = _solids([np.asarray(f, float) for f in polys[body.name]])
            groups = [(body.name if len(solids) == 1 else f"{body.name} {k + 1}", F, col)
                      for k, F in enumerate(solids)]
        else:
            groups = []
            for k, bx in enumerate(body.boxes):
                nm = bx.name or body.name
                if len(body.boxes) > 1:
                    nm = f"{body.name} – {nm}" if bx.name and bx.name != body.name \
                        else f"{body.name} {k + 1}"
                groups.append((nm, box_faces(bx), tuple(bx.color) if bx.color else col))
        if groups:
            out.append((body.name, body is mech.ground, groups))
    return out


def _solids(faces, tol=1e-9):
    """The faces split into the closed pieces they form (shared corners):
    one group each, so each piece gets its own collision box (a crank
    shaft and its gear, not one box around both)."""
    key = lambda p: tuple(np.round(np.asarray(p) / tol).astype(np.int64))
    parent = list(range(len(faces)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    owner = {}
    for i, f in enumerate(faces):
        for p in f:
            k = key(p)
            if k in owner:
                a, b = find(i), find(owner[k])
                if a != b:
                    parent[a] = b
            else:
                owner[k] = i
    out = {}
    for i, f in enumerate(faces):
        out.setdefault(find(i), []).append(f)
    return list(out.values())


def _rail_points(path, step=0.005, max_angle=math.radians(2.0)):
    """A guide's rail as a polyline: lines as they are, arcs in short chords."""
    pts = [np.asarray(path.start, float)]
    for kind, s0, L, d in path.pieces:
        if kind == "line":
            pts.append(d[0] + L * d[1])
            continue
        R, angle = d[4], d[5]
        n = max(2, int(math.ceil(max(angle / max_angle, L / step))))
        for i in range(1, n + 1):
            pts.append(path.point(s0 + L * i / n))
    return [list(map(float, p)) for p in pts]


def _vec(v):
    return [float(x) for x in np.asarray(v, float)]


def _limits(lim):
    return [float(lim[0]), float(lim[1])] if lim is not None else None


def data(mech, info, uids):
    """The document's mechanism for the copied example. ``uids``: {body
    name: [group uids]} of the groups made from :func:`parts`."""
    info = info or {}
    d = doc.empty()
    for body in [mech.ground] + list(mech.bodies):
        if uids.get(body.name):
            d["bodies"].append({"name": body.name, "groups": [str(u) for u in uids[body.name]],
                                "fixed": body is mech.ground})
    bodies = {b["name"] for b in d["bodies"]}
    # a joint may share a part's name (the bird's « manivelle »): in the
    # document the list is keyed by name, so the joint gets its kind added
    rename = {}
    for j in mech.joints:
        nm = j.name or j.kind
        if nm in bodies or nm in rename.values():
            nm = f"{nm} ({_T(_KIND_WORD.get(j.kind, j.kind))})"
        rename[id(j)] = nm
    sim = info.get("simulation") or {}
    damping = (sim.get("params") or {}).get("damping") or {}
    for j in mech.joints:
        rec = {"kind": j.kind, "name": rename[id(j)], "a": j.a.name, "b": j.b.name,
               "limits": _limits(getattr(j, "limits", None))}
        if j.kind == "coupling":
            rec.update(ja=rename[id(j.ja)], jb=rename[id(j.jb)], ratio=float(j.ratio))
        else:
            rec["point"] = _vec(j.point)
            if j.kind == "guide":
                rec["rail"] = _rail_points(j.path)
            elif hasattr(j, "axis"):
                rec["axis"] = _vec(j.axis)
            if j.kind == "screw":
                rec["pitch"] = float(j.pitch)
            if j.kind == "elastic":
                rec.update(k_lin=float(j.k_lin), k_rot=float(j.k_rot),
                           c_lin=float(j.c_lin), c_rot=float(j.c_rot))
            if getattr(j, "spring", None):
                rec["spring"] = [float(x) for x in j.spring]
            c = float(getattr(j, "damper", 0.0) or 0.0) + float(damping.get(j.name, 0.0))
            if c:
                rec["damper"] = c
            if getattr(j, "slide_limits", None):
                rec["slide_limits"] = _limits(j.slide_limits)
        d["joints"].append(rec)
    d["drivers"] = [{"joint": rename[id(jj)], "lo": float(lo), "hi": float(hi)}
                    for jj, lo, hi in mech.drivers]
    d["driver"] = dict(d["drivers"][0]) if d["drivers"] else None
    d["couples"] = {rename[id(j)]: float(v) for j in mech.joints
                    for n, v in (getattr(mech, "torques", None) or {}).items() if n == j.name}
    if sim:
        d["simulation"] = {"active": True, "duree": float(sim.get("duree", 4.0)),
                           "chocs": bool((sim.get("params") or {}).get("contacts"))}
    if info.get("jeu_min") is not None:
        d["jeu_min"] = float(info["jeu_min"])
    _materials(d, mech, uids)
    return doc.normalized(d)


_KIND_WORD = {"pivot": "pivot", "slider": "glissière", "guide": "galet",
              "cylindrical": "pivot glissant", "ball": "rotule", "lock": "encastrement",
              "elastic": "élastique", "annular": "linéaire annulaire",
              "pointplane": "ponctuelle", "planar": "appui plan", "screw": "hélicoïdale",
              "coupling": "engrenage"}


def _materials(d, mech, uids):
    """Parts that have a mass in the example (the dynamics ones) keep it:
    the library material of the same density, else one of the document."""
    from . import massprops, materials
    lib = materials.library(None)
    for body in mech.bodies:
        if not body.mass or not uids.get(body.name):
            continue
        vol = sum(abs(massprops.of_triangles(_tris(bx), 1.0)[0]) for bx in body.boxes) \
            if body.boxes else 0.0
        if vol <= 0:
            continue
        rho = body.mass / vol
        key = next((k for k, r in lib.items() if abs(r["density"] - rho) <= 0.02 * rho), None)
        if key is None:
            key = f"exemple-{len(d['matieres']) + 1}"
            d["matieres"][key] = {"name": _T("Matière de l'exemple"), "family": "autre",
                                  "density": round(rho, 1)}
        for u in uids[body.name]:
            d["materiau"][str(u)] = key


def _tris(box):
    F = box_faces(box)
    return np.array([t for f in F for t in ((f[0], f[1], f[2]), (f[0], f[2], f[3]))])
