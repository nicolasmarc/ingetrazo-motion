"""The mechanism as stored in the IngeTrazo document (pure, no Qt).

One JSON-safe dict (``app.set_document_data``), metres and radians::

    {"format": 1,
     "bodies": [{"name": "assise", "groups": ["<uid>", ...], "fixed": false}],
     "joints": [{"kind": "pivot" | "slider" | "guide", "name": "...",
                 "a": "<body>", "b": "<body>",
                 "point": [x, y, z], "axis": [x, y, z],      # pivot, slider
                 "rail": [[x, y, z], ...],                   # guide polyline
                 "limits": [lo, hi] | null},
                {"kind": "coupling", "name": "...", "a": "<body>", "b": "<body>",
                 "ja": "<joint>", "jb": "<joint>", "ratio": -0.5}],  # jb = ratio x ja
     "drivers": [{"joint": "<name>", "lo": 0.0, "hi": 1.57}, ...],
     "driver": <the first of "drivers"> | null,      # kept for 0.1-0.5 files
     "jeu_min": 0.005, "pas": 51,
     "materiau": {"<uid>": "chene", ...},           # material of each group
     "matieres": {"<key>": {"name": ..., "density": ...}}}   # the document's own

A body is one or more IngeTrazo groups moving together; every body marked
``fixed`` is merged into the frame. Its collision boxes are fitted to the
groups' points (one box per group).
"""
import copy
import numpy as np
from .model import Mechanism, Box, Path, unit

FORMAT = 1


def empty():
    return {"format": FORMAT, "bodies": [], "joints": [], "drivers": [],
            "driver": None, "jeu_min": 0.005, "pas": 51, "materiau": {}, "matieres": {}}


def normalized(data):
    """A complete, independent copy (missing keys filled in). A file with
    the single ``driver`` of 0.1-0.5 gets it as its one motor."""
    out = empty()
    if isinstance(data, dict):
        for k in out:
            if k in data and data[k] is not None:
                out[k] = copy.deepcopy(data[k])
    if isinstance(data, dict) and "drivers" not in data and out["driver"]:
        out["drivers"] = [copy.deepcopy(out["driver"])]
    out["driver"] = copy.deepcopy(out["drivers"][0]) if out["drivers"] else None
    return out


def unique_name(data, stem):
    used = {b["name"] for b in data["bodies"]} | {j["name"] for j in data["joints"]}
    k = 1
    while f"{stem}{k}" in used:
        k += 1
    return f"{stem}{k}"


# -- editing ----------------------------------------------------------------

def add_body(data, uids, fixed, name=None):
    """New body from groups ``uids``; groups already used elsewhere are
    taken out of their previous body (a group belongs to one body)."""
    d = normalized(data)
    uids = [str(u) for u in uids]
    for b in d["bodies"]:
        b["groups"] = [u for u in b["groups"] if u not in uids]
    name = name or unique_name(d, "fixe" if fixed else "pièce")
    d["bodies"].append({"name": name, "groups": uids, "fixed": bool(fixed)})
    return d, name


def add_joint(data, kind, a, b, point=None, axis=None, rail=None,
              limits=None, name=None):
    d = normalized(data)
    stem = {"pivot": "pivot", "slider": "glissière", "guide": "galet"}[kind]
    j = {"kind": kind, "name": name or unique_name(d, stem), "a": a, "b": b,
         "limits": list(limits) if limits is not None else None}
    if kind == "guide":
        j["rail"] = [list(map(float, p)) for p in rail]
        j["point"] = list(map(float, point))
    else:
        j["point"] = list(map(float, point))
        j["axis"] = list(map(float, unit(axis)))
    d["joints"].append(j)
    return d, j["name"]


def remove(data, name):
    """Remove a body (and its joints) or a joint."""
    d = normalized(data)
    d["bodies"] = [b for b in d["bodies"] if b["name"] != name]
    d["joints"] = [j for j in d["joints"]
                   if j["name"] != name and name not in (j["a"], j["b"])]
    names = {j["name"] for j in d["joints"]}            # couplings of a gone joint
    d["joints"] = [j for j in d["joints"] if j["kind"] != "coupling"
                   or (j.get("ja") in names and j.get("jb") in names)]
    names = {j["name"] for j in d["joints"]}
    d["drivers"] = [x for x in d["drivers"] if x["joint"] in names]
    return normalized(d)


def add_coupling(data, ja, jb, ratio, name=None):
    """``jb`` turns ``ratio`` times ``ja`` (gears: -z1/z2)."""
    d = normalized(data)
    rec = {j["name"]: j for j in d["joints"]}
    a, b = rec[ja], rec[jb]
    j = {"kind": "coupling", "name": name or unique_name(d, "engrenage"),
         "a": a["b"], "b": b["b"], "ja": ja, "jb": jb, "ratio": float(ratio), "limits": None}
    d["joints"].append(j)
    return d, j["name"]


def apply_detection(data, parts, frame, proposals):
    """Bodies and joints from the detection (``motion.detect``):
    ``parts`` = {key: (name, [group uids])}, ``frame`` = key of the fixed
    part, ``proposals`` = the ticked ones. One body per part involved;
    joints keyed on the frame side first. Returns (data, [joint names])."""
    d = normalized(data)
    names = {}
    merge = {}                                   # pin -> the part it is fixed in
    for x in proposals:
        merge.update(getattr(x, "extra", {}).get("merge", {}))
    used = ({k for x in proposals for k in (x.a, x.b)} | {frame}) - set(merge)
    for key in [frame] + sorted((k for k in used if k != frame), key=lambda k: parts[k][0]):
        if key not in parts:
            continue
        nm, uids = parts[key]
        uids = list(uids) + [u for pin, host in merge.items() if host == key
                             for u in parts[pin][1]]
        base = nm
        taken = {b["name"] for b in d["bodies"]} | {j["name"] for j in d["joints"]}
        k = 2
        while nm in taken:
            nm, k = f"{base} {k}", k + 1
        d, nm = add_body(d, uids, key == frame, nm)
        names[key] = nm
    made, by_prop = [], {}
    for x in proposals:
        if x.kind not in ("pivot", "slider"):
            continue
        a, b = (x.a, x.b) if x.b != frame else (x.b, x.a)
        stem = "pivot" if x.kind == "pivot" else "glissière"
        jn = f"{stem} {names[a]}–{names[b]}"
        if jn in {b_["name"] for b_ in d["bodies"]} | {j["name"] for j in d["joints"]}:
            jn = unique_name(d, jn + " ")
        d, jn = add_joint(d, x.kind, names[a], names[b], point=x.point, axis=x.axis, name=jn)
        made.append(jn)
        by_prop[id(x)] = (jn, a, b, np.asarray(x.axis, float))
    for x in proposals:
        if x.kind != "gears":
            continue
        pa, pb = x.extra["pivot_a"], x.extra["pivot_b"]
        if id(pa) not in by_prop or id(pb) not in by_prop:
            continue                       # a gear's pivot was not kept
        ja, a0, a1, ax_a = by_prop[id(pa)]
        jb, b0, b1, ax_b = by_prop[id(pb)]
        # angles are of the joint's second part about its axis, seen from
        # its first: turn the gear ratio into a ratio of those values
        f = x.extra["frame"]
        r = x.extra["ratio"]
        r *= (1 if a0 == f else -1) * (1 if b0 == f else -1)
        if ax_a @ ax_b < 0:
            r = -r
        d, cn = add_coupling(d, ja, jb, r, name=unique_name(d, "engrenage "))
        made.append(cn)
    return d, made


def set_material(data, uids, key):
    """Groups ``uids`` are made of material ``key`` (None: forget it)."""
    d = normalized(data)
    for u in uids:
        if key is None:
            d["materiau"].pop(str(u), None)
        else:
            d["materiau"][str(u)] = key
    return d


def add_material(data, key, props):
    """A material of the document's own (or a library one overridden)."""
    from .materials import FIELDS
    d = normalized(data)
    d["matieres"][key] = {f: props[f] for f in FIELDS if f in props and props[f] is not None}
    return d


def mass_properties(data, triangles_of_group):
    """{body name: dict(mass, com, inertia, volume, materials, missing)}
    from each group's closed triangles (world, reference pose) and its
    material; groups without a material are listed in ``missing`` and left
    out of the mass."""
    from . import massprops, materials
    d = normalized(data)
    lib = materials.library(d)
    out = {}
    for b in d["bodies"]:
        pieces, mats, missing, vol = [], set(), [], 0.0
        for u in b["groups"]:
            T = triangles_of_group(u)
            if T is None or not len(T):
                continue
            key = d["materiau"].get(u)
            rec = lib.get(key) if key else None
            m, c, I, v = massprops.of_triangles(T, rec["density"] if rec else 1.0)
            vol += abs(v)
            if rec is None:
                missing.append(u)
                continue
            mats.add(rec["name"])
            if m < 0:                         # faces turned inward
                m, I = -m, -I
            pieces.append((m, c, I))
        m, c, I = massprops.combine(pieces) if pieces else (0.0, None, None)
        out[b["name"]] = dict(mass=m, com=c, inertia=I, volume=vol,
                              materials=sorted(mats), missing=missing)
    return out


def set_driver(data, joint, lo, hi):
    """``joint`` becomes the only motor."""
    d = normalized(data)
    d["drivers"] = [{"joint": joint, "lo": float(lo), "hi": float(hi)}]
    return normalized(d)


def add_driver(data, joint, lo, hi):
    """One more motor (or new travel for an existing one). All motors run
    on the same 0..100 %, each from its ``lo`` to its ``hi``."""
    d = normalized(data)
    rec = {"joint": joint, "lo": float(lo), "hi": float(hi)}
    for k, x in enumerate(d["drivers"]):
        if x["joint"] == joint:
            d["drivers"][k] = rec
            break
    else:
        d["drivers"].append(rec)
    return normalized(d)


def remove_driver(data, joint):
    d = normalized(data)
    d["drivers"] = [x for x in d["drivers"] if x["joint"] != joint]
    return normalized(d)


# -- geometry helpers ---------------------------------------------------------

def fit_box(points, name=""):
    """Bounding box of ``points``: the smaller of the axis-aligned box and
    the box along the principal directions."""
    P = np.asarray(points, float).reshape(-1, 3)
    if len(P) == 0:
        return None
    best = None
    frames = [np.eye(3)]
    if len(P) >= 3:
        c = P - P.mean(axis=0)
        _, _, vt = np.linalg.svd(c, full_matrices=False)
        R = vt.T
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1
        frames.append(R)
    for R in frames:
        loc = P @ R
        lo, hi = loc.min(axis=0), loc.max(axis=0)
        half = np.maximum((hi - lo) / 2, 1e-6)
        vol = float(np.prod(half))
        if best is None or vol < best[0] - 1e-15:
            best = (vol, R @ ((lo + hi) / 2), half, R)
    return Box(best[1], best[2], best[3], name)


def chains(segments, tol=1e-6, max_turn=45.0):
    """Smooth polylines from a list of segments ((p0, p1), ...): two
    segments continue each other when they share an end and turn by less
    than ``max_turn`` degrees (a rail drawn as lines and arc segments is
    smooth, a roller mark drawn across it is not). Longest first."""
    segs = [(np.asarray(a, float), np.asarray(b, float)) for a, b in segments]
    left = list(range(len(segs)))
    cos_max = np.cos(np.radians(max_turn))
    out = []

    def same(p, q):
        return np.linalg.norm(p - q) <= tol

    def smooth(d0, d1):
        n0, n1 = np.linalg.norm(d0), np.linalg.norm(d1)
        return n0 > 0 and n1 > 0 and (d0 @ d1) / (n0 * n1) >= cos_max

    while left:
        i = left.pop(0)
        line = [segs[i][0], segs[i][1]]
        grown = True
        while grown:
            grown = False
            for k in list(left):
                a, b = segs[k]
                for end in (-1, 0):
                    tip, prev = line[end], line[end + 1 if end == 0 else -2]
                    if same(a, tip):
                        nxt = b
                    elif same(b, tip):
                        nxt = a
                    else:
                        continue
                    if not smooth(tip - prev, nxt - tip):
                        continue
                    if end == -1:
                        line.append(nxt)
                    else:
                        line.insert(0, nxt)
                    left.remove(k)
                    grown = True
                    break
        out.append(line)
    out.sort(key=lambda l: -sum(np.linalg.norm(l[i + 1] - l[i])
                                for i in range(len(l) - 1)))
    return out


def rail_and_rollers(segments):
    """Selection for a guide: the longest chain is the rail (run from its
    lowest end), every other segment marks one roller (its point nearest
    to the rail); rollers come in rail order, whatever the selection order."""
    cs = chains(segments)
    if not cs:
        raise ValueError("sélectionner les arêtes du rail")
    rail = cs[0]
    a, b = rail[0], rail[-1]                  # runs from its lowest end
    if (b[2], b[0], b[1]) < (a[2], a[0], a[1]):
        rail = rail[::-1]
    path = path_from_points(rail)
    rollers = []
    for other in cs[1:]:
        best = min(other, key=lambda p: np.linalg.norm(p - path.closest(p)[1]))
        rollers.append(best)
    rollers.sort(key=lambda p: path.closest(p)[0])     # in rail order
    return rail, rollers


def path_from_points(points):
    pts = [np.asarray(p, float) for p in points]
    path = Path(pts[0])
    for p in pts[1:]:
        if np.linalg.norm(p - path.end) > 1e-12:
            path.line_to(p)
    return path


def nearest_body(point, boxes_by_body, exclude=()):
    """Name of the body whose boxes come nearest ``point``."""
    p = np.asarray(point, float)
    best = (np.inf, None)
    for name, boxes in boxes_by_body.items():
        if name in exclude:
            continue
        for bx in boxes:
            loc = (p - bx.center) @ bx.R
            d = float(np.linalg.norm(loc - np.clip(loc, -bx.half, bx.half)))
            if d < best[0]:
                best = (d, name)
    return best[1]


# -- building the mechanism -----------------------------------------------------

def body_boxes(data, points_of_group):
    """{body name: [Box per group]} from the groups' world points."""
    out = {}
    for b in data["bodies"]:
        boxes = []
        for uid in b["groups"]:
            pts = points_of_group(uid)
            if pts is not None and len(pts):
                bx = fit_box(pts, name=str(uid))
                if bx is not None:
                    boxes.append(bx)
        out[b["name"]] = boxes
    return out


def build(data, points_of_group=lambda uid: None, names_of_group=None,
          triangles_of_group=None):
    """(Mechanism, problems). ``points_of_group(uid)`` gives the world
    points of a group (None when the group is gone); with
    ``triangles_of_group`` the parts get their mass, centre of gravity and
    inertia from the materials."""
    d = normalized(data)
    problems = []
    m = Mechanism()
    boxes = body_boxes(d, points_of_group)
    if names_of_group:
        for name, bxs in boxes.items():
            for bx in bxs:
                bx.name = names_of_group(bx.name) or bx.name
    bodies = {}
    fixed = [b for b in d["bodies"] if b["fixed"]]
    m.ground.name = fixed[0]["name"] if len(fixed) == 1 else "bâti"
    for b in fixed:
        m.ground.boxes.extend(boxes.get(b["name"], []))
        bodies[b["name"]] = m.ground
    for b in d["bodies"]:
        if b["fixed"]:
            continue
        bxs = boxes.get(b["name"], [])
        c = np.mean([bx.center for bx in bxs], axis=0) if bxs else np.zeros(3)
        bodies[b["name"]] = m.body(b["name"], c, bxs)
        if not bxs:
            problems.append(f"{b['name']} : aucun groupe trouvé dans le modèle")
    if not fixed:
        problems.append("aucune pièce fixe : ajouter une « pièce fixe »")
    joints = {}
    for j in d["joints"]:
        a, b = bodies.get(j["a"]), bodies.get(j["b"])
        if a is None or b is None:
            problems.append(f"{j['name']} : pièce inconnue")
            continue
        if a is b:
            problems.append(f"{j['name']} : relie une pièce à elle-même")
            continue
        lim = tuple(j["limits"]) if j.get("limits") else None
        if j["kind"] == "coupling":
            continue                      # after every other joint
        if j["kind"] == "pivot":
            joints[j["name"]] = m.pivot(a, b, j["point"], j["axis"], lim, j["name"])
        elif j["kind"] == "slider":
            joints[j["name"]] = m.slider(a, b, j["point"], j["axis"], lim, j["name"])
        else:
            joints[j["name"]] = m.guide(a, b, j["point"], path_from_points(j["rail"]),
                                        lim, j["name"])
    for j in d["joints"]:
        if j["kind"] != "coupling":
            continue
        ja, jb = joints.get(j.get("ja")), joints.get(j.get("jb"))
        if ja is None or jb is None:
            problems.append(f"{j['name']} : liaison couplée inconnue")
            continue
        joints[j["name"]] = m.couple(ja, jb, j["ratio"], j["name"])
    for drv in d["drivers"]:
        if drv["joint"] in joints:
            m.add_driver(joints[drv["joint"]], drv["lo"], drv["hi"])
        else:
            problems.append(f"moteur {drv['joint']} : liaison inconnue")
    if not m.drivers and m.bodies:
        problems.append("aucun moteur : choisir une liaison et « Moteur 0–100 % »")
    m.masses = {}
    if triangles_of_group is not None:
        m.masses = mass_properties(d, triangles_of_group)
        for name, body in bodies.items():
            mp = m.masses.get(name)
            if mp and mp["mass"] > 0 and body is not m.ground:
                body.mass, body.com, body.inertia = mp["mass"], mp["com"], mp["inertia"]
    return m, problems


def trajectory(mech, rows, body, point):
    """World positions of ``point`` (reference pose) of ``body`` along the
    solved ``rows`` (from ``solver.sweep`` or ``collide.validate``)."""
    return np.array([body.world_point(mech.pose(r["x"], body), point) for r in rows])
