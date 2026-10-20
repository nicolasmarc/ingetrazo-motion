"""Interference and clearance between bodies (oriented boxes).

- ``obb_overlap``: separating-axis test (15 axes).
- ``box_distance``: EXACT distance between two separated boxes, from the
  closest features (vertex against box, edge against edge); when they
  overlap, a negative value = the smallest push-out depth found on the
  separating axes.
- Joined bodies: the box pairs that already touch in the drawn
  configuration ARE the joint (hinge knuckle, pin in its slot, drawer in
  its runners) and are skipped; every other box pair of the two bodies is
  still checked. ``ignore_joined="body"`` skips the whole pair (V0.1),
  ``False`` checks everything.
"""
import itertools
import numpy as np
from .lang import T as _T

_CORNERS = np.array(list(itertools.product((-1, 1), repeat=3)), float)
_EDGES = [(i, j) for i in range(8) for j in range(i + 1, 8)
          if np.count_nonzero(_CORNERS[i] != _CORNERS[j]) == 1]


def world_obb(body, q, box):
    R = body.rot(q) @ box.R
    c = body.world_point(q, box.center)
    return c, R, box.half


def obb_overlap(A, B, margin=0.0):
    """True if two OBBs (centre, R, half) overlap by more than ``margin``
    (margin < 0 -> require a clearance of -margin)."""
    return _sat(A, B) > margin


def _sat(A, B):
    """Smallest overlap over the separating axes (< 0: separated by at
    least that much along some axis)."""
    ca, Ra, ha = A
    cb, Rb, hb = B
    t = cb - ca
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for k in range(3):
            c = np.cross(Ra[:, i], Rb[:, k])
            n = np.linalg.norm(c)
            if n > 1e-9:
                axes.append(c / n)
    best = np.inf
    for ax in axes:
        ra = sum(ha[i] * abs(Ra[:, i] @ ax) for i in range(3))
        rb = sum(hb[i] * abs(Rb[:, i] @ ax) for i in range(3))
        best = min(best, ra + rb - abs(t @ ax))
    return best


def _corners(obb):
    c, R, h = obb
    return c + (_CORNERS * h) @ R.T


def _points_to_box(P, obb):
    """Closest points of box ``obb`` to the points ``P`` (N, 3)."""
    c, R, h = obb
    local = (P - c) @ R
    return c + np.clip(local, -h, h) @ R.T


def _seg_seg(p1, q1, p2, q2):
    """Closest points between segment pairs (arrays (N, 3)), clamped."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.einsum("ij,ij->i", d1, d1)
    e = np.einsum("ij,ij->i", d2, d2)
    f = np.einsum("ij,ij->i", d2, r)
    c = np.einsum("ij,ij->i", d1, r)
    b = np.einsum("ij,ij->i", d1, d2)
    den = a * e - b * b
    a_ = np.where(a > 1e-300, a, 1.0)
    e_ = np.where(e > 1e-300, e, 1.0)
    s = np.where(den > 1e-14 * a_ * e_,
                 np.clip((b * f - c * e) / np.where(den > 0, den, 1), 0, 1), 0.0)
    t = (b * s + f) / e_
    lo, hi = t < 0, t > 1
    t = np.clip(t, 0, 1)
    s = np.where(lo, np.clip(-c / a_, 0, 1), s)
    s = np.where(hi, np.clip((b - c) / a_, 0, 1), s)
    return p1 + d1 * s[:, None], p2 + d2 * t[:, None]


def box_distance(A, B):
    """(distance, point_on_A, point_on_B). Exact when the boxes are apart;
    when they overlap: (-depth, centre_A, centre_B)."""
    depth = _sat(A, B)
    if depth >= 0:
        return -float(depth), A[0], B[0]
    ca, cb = _corners(A), _corners(B)
    cand = []
    pb = _points_to_box(ca, B)                      # vertices of A vs B
    cand.append((ca, pb))
    pa = _points_to_box(cb, A)                      # vertices of B vs A
    cand.append((pa, cb))
    ia = np.array([e[0] for e in _EDGES]); ja = np.array([e[1] for e in _EDGES])
    I, J = np.meshgrid(np.arange(12), np.arange(12), indexing="ij")
    I, J = I.ravel(), J.ravel()
    x1, x2 = _seg_seg(ca[ia[I]], ca[ja[I]], cb[ia[J]], cb[ja[J]])
    cand.append((x1, x2))
    best = (np.inf, None, None)
    for pa_, pb_ in cand:
        d = np.linalg.norm(pa_ - pb_, axis=1)
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), pa_[k], pb_[k])
    return best


# -- pairs of bodies -------------------------------------------------------

def _joined(mech):
    out = {}
    for j in mech.joints:
        if j.kind != "coupling":           # a ratio joins no parts together
            out.setdefault(frozenset((j.a.name, j.b.name)), []).append(j)
    return out


def _skipped_box_pairs(mech, a, b, mode, tol=1e-9):
    """Box index pairs of bodies a, b that are not checked."""
    if mode is False:
        return set()
    if frozenset((a.name, b.name)) not in _joined(mech):
        return set()
    allp = {(i, k) for i in range(len(a.boxes)) for k in range(len(b.boxes))}
    if mode == "body":
        return allp
    z = np.zeros(6)
    return {(i, k) for i, k in allp
            if _sat(world_obb(a, z, a.boxes[i]), world_obb(b, z, b.boxes[k])) > -tol}


def _pairs(mech, ignore_joined):
    bodies = [mech.ground] + mech.bodies
    key = (str(ignore_joined), len(mech.joints),
           tuple(len(b.boxes) for b in bodies))
    memo = getattr(mech, "_skip_memo", None)
    if memo is None or memo[0] != key:
        memo = (key, {})
        try:
            mech._skip_memo = memo
        except Exception:
            pass
    fb = {id(b) for b in getattr(mech, "follower_bodies", lambda: [])()}
    nc = getattr(mech, "no_contact", set())
    for i, a in enumerate(bodies):
        for b in bodies[i + 1:]:
            if (id(a) in fb and id(b) in fb) or frozenset((id(a), id(b))) in nc:
                continue                  # links of one chain, link on its sprocket
            if a.boxes and b.boxes:
                k = (id(a), id(b))
                if k not in memo[1]:
                    memo[1][k] = _skipped_box_pairs(mech, a, b, ignore_joined)
                yield a, b, memo[1][k]


def _world_boxes(body, q):
    """[(obb, aabb_lo, aabb_hi)] of a body's boxes at pose ``q``."""
    out = []
    for bx in body.boxes:
        c, R, h = world_obb(body, q, bx)
        e = np.abs(R) @ h
        out.append(((c, R, h), c - e, c + e))
    return out


def _aabb_gap(a, b):
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(b[1] - a[2], a[1] - b[2]))))


def _surfaces(mech):
    S = getattr(mech, "surfaces", None)
    return S if S is not None and S.data else None


def _mesh_pairs(mech, x, reach):
    """Pairs checked on their real surfaces: [(a, b, record or None)]."""
    S = _surfaces(mech)
    if S is None:
        return []
    out = []
    fb = {id(b) for b in getattr(mech, "follower_bodies", lambda: [])()}
    nc = getattr(mech, "no_contact", set())
    bodies = [mech.ground] + mech.bodies
    joined = _joined(mech)
    for i, a in enumerate(bodies):
        if not S.has(a):
            continue
        for b in bodies[i + 1:]:
            if not S.has(b) or frozenset((id(a), id(b))) in nc or (a.fixed and b.fixed):
                continue
            if id(a) in fb and id(b) in fb:
                continue
            r = S.pair(a, b, mech.pose(x, a), mech.pose(x, b), reach)
            if r is not None:
                d, pa, pb, ga, gb = r
                out.append(dict(a=a.name, b=b.name, box_a=ga, box_b=gb, pa=pa, pb=pb,
                                distance=d if d < -S.tol else max(d, 0.0), mesh=True,
                                joined=frozenset((a.name, b.name)) in joined))
    return out


def _mesh_reach(mech, jeu_min=0.0):
    S = _surfaces(mech)
    return max(jeu_min + S.h, 6 * S.h) if S is not None else 0.0


def _box_only(mech, a, b):
    S = _surfaces(mech)
    return S is None or not (S.has(a) and S.has(b))


def interferences(mech, x, margin=1e-6, ignore_joined=True):
    """List of (body_a, body_b) pairs that overlap at configuration ``x``."""
    hits = [(c["a"], c["b"]) for c in _mesh_pairs(mech, x, _mesh_reach(mech))
            if c["distance"] < 0]
    cache = {}
    for a, b, skip in _pairs(mech, ignore_joined):
        if not _box_only(mech, a, b):
            continue
        for body in (a, b):
            if id(body) not in cache:
                cache[id(body)] = _world_boxes(body, mech.pose(x, body))
        WA, WB = cache[id(a)], cache[id(b)]
        if any(_aabb_gap(A, B) <= 0 and obb_overlap(A[0], B[0], margin)
               for i, A in enumerate(WA) for k, B in enumerate(WB)
               if (i, k) not in skip):
            hits.append((a.name, b.name))
    return hits


def clearances(mech, x, ignore_joined=True, reach=None):
    """Smallest gap for every pair of bodies at configuration ``x``:
    list of dicts (a, b, box_a, box_b, distance, pa, pb), smallest first.
    distance < 0 means the boxes overlap (by about -distance).
    Box pairs are taken nearest first by their axis-aligned bounds, and the
    exact distance is only computed while it can still beat the best."""
    out = list(_mesh_pairs(mech, x, reach if reach is not None else _mesh_reach(mech)))
    cache = {}
    for a, b, skip in _pairs(mech, ignore_joined):
        if not _box_only(mech, a, b):
            continue
        for body in (a, b):
            if id(body) not in cache:
                cache[id(body)] = _world_boxes(body, mech.pose(x, body))
        WA, WB = cache[id(a)], cache[id(b)]
        cand = sorted((_aabb_gap(A, B), i, k) for i, A in enumerate(WA)
                      for k, B in enumerate(WB) if (i, k) not in skip)
        best = None
        for gap, i, k in cand:
            if best is not None and gap >= best["distance"]:
                break                              # the rest is further away
            d, pa, pb = box_distance(WA[i][0], WB[k][0])
            if best is None or d < best["distance"]:
                best = dict(a=a.name, b=b.name, box_a=a.boxes[i].name or str(i),
                            box_b=b.boxes[k].name or str(k), distance=d, pa=pa, pb=pb)
        if best is not None:
            out.append(best)
    return sorted(out, key=lambda r: r["distance"])


def _box_hits(mech, x, ignore_joined, margin=1e-6):
    hits, cache = [], {}
    for a, b, skip in _pairs(mech, ignore_joined):
        if not _box_only(mech, a, b):
            continue
        for body in (a, b):
            if id(body) not in cache:
                cache[id(body)] = _world_boxes(body, mech.pose(x, body))
        WA, WB = cache[id(a)], cache[id(b)]
        if any(_aabb_gap(A, B) <= 0 and obb_overlap(A[0], B[0], margin)
               for i, A in enumerate(WA) for k, B in enumerate(WB)
               if (i, k) not in skip):
            hits.append((a.name, b.name))
    return hits


def clearance_violations(mech, x, jeu_min, ignore_joined=True):
    """Pairs closer than ``jeu_min`` (overlaps included)."""
    return [c for c in clearances(mech, x, ignore_joined)
            if c["distance"] < jeu_min - 1e-12]


def limit_violations(mech, x, tol=1e-6):
    out = []
    for j in mech.joints:
        if j.limits is None:
            continue
        v = mech.joint_value(j, x)
        lo, hi = j.limits
        if (lo is not None and v < lo - tol) or (hi is not None and v > hi + tol):
            out.append((j.name, v, j.limits))
    return out


def validate(mech, steps=51, jeu_min=0.0, ignore_joined=True, poses=None):
    """Run the driver 0..100 % and check everything at each step.

    Returns a dict: ``steps`` (per step: t, value, ok, res, hits, limits,
    clearance = smallest gap record), ``worst`` (smallest gap over the run,
    with its t), ``ok`` (converged, no interference, no limit, gap >=
    jeu_min everywhere) and ``problems`` (readable strings).
    ``poses``: rows {t (0..1), value, x, ok, res} computed elsewhere (a
    dynamic simulation) instead of the driver's sweep."""
    from .solver import sweep
    rows, worst, problems = [], None, []
    for st in (poses if poses is not None else sweep(mech, steps)):
        x = st["x"]
        cl = clearances(mech, x, ignore_joined, reach=_mesh_reach(mech, jeu_min))
        hits = [(c["a"], c["b"]) for c in cl if c.get("mesh") and c["distance"] < 0]
        if _surfaces(mech) is None:
            hits = interferences(mech, x, ignore_joined=ignore_joined)
        else:
            hits += [h for h in _box_hits(mech, x, ignore_joined)]
        row = dict({k: v for k, v in st.items() if k not in ("x",)},
                   t=st["t"], value=st["value"], x=x, ok=st["ok"], res=st["res"],
                   hits=hits,
                   limits=limit_violations(mech, x),
                   clearance=cl[0] if cl else None)
        rows.append(row)
        pct = round(100 * st["t"])
        if not st["ok"]:
            problems.append(_T("{p} % : position non atteinte (résidu {r})", p=pct, r=f"{st['res']:.2e}"))
        for a, b in row["hits"]:
            problems.append(_T("{p} % : collision {a} / {b}", p=pct, a=a, b=b))
        if poses is None:                 # in a simulation the stops act: not a fault
            for name, v, lim in row["limits"]:
                problems.append(_T("{p} % : butée dépassée {name} = {v}", p=pct, name=name, v=f"{v:.4g}"))
        # parts joined together: right next to their joint (within 2 sampling
        # steps) the gap means nothing — a pin in its bore; their collisions
        # and their gaps further off count
        if _surfaces(mech) is not None:
            near = 2 * _surfaces(mech).h
            cl = [c for c in cl if not c.get("joined") or c["distance"] < 0
                  or c["distance"] >= near]
        row["clearance"] = cl[0] if cl else None
        if cl:
            c = cl[0]
            if worst is None or c["distance"] < worst["distance"]:
                worst = dict(c, t=st["t"])
            for c in cl:
                if 0 <= c["distance"] < jeu_min - 1e-12:
                    problems.append(_T("{p} % : jeu {d} mm < {j} mm entre {a} ({ga}) et {b} ({gb})",
                                      p=pct, d=f"{c['distance'] * 1000:.1f}",
                                      j=f"{jeu_min * 1000:.1f}", a=c['a'], ga=c['box_a'],
                                      b=c['b'], gb=c['box_b']))
    return dict(steps=rows, worst=worst, ok=not problems, problems=problems)
