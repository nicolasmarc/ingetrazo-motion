"""Automatic joint detection: scan the parts and propose joints.

Each part is described by its *regions*: a planar face, or a smooth
surface (faces joined by soft edges — how IngeTrazo, and the STEP import,
mesh a curved face). A smooth region whose normals are all square to one
direction and whose points lie on a circle around it is a cylinder: a
shaft (normals pointing out) or a bore (pointing in).

Proposals, between two parts:

* **pivot** — a shaft inside a bore of the other part: parallel and
  coaxial, the bore as large as the shaft or slightly larger, overlapping
  along the axis. Several coaxial fits of one pair make one pivot.
* **slider** — two or more flat contacts (opposite normals, coplanar,
  overlapping) whose normals are not parallel: the part can only slide
  along the line square to all of them (a dovetail, a T-slot, a drawer
  runner).

and, between two pivots on the same part:

* **gears** — two toothed parts turning on parallel axes whose tip circles
  overlap by about one tooth depth: a coupling of ratio -z1/z2 (teeth
  counted on the outline).

The frame proposed is the part the most joints lean on (ties: the
largest). Every proposal has a score 0..1 (how clean the fit is); nothing
is applied until the user ticks it. Pure numpy, no Qt.
"""
import math
from dataclasses import dataclass, field
import numpy as np


# -- regions -------------------------------------------------------------------

@dataclass
class Region:
    kind: str                         # "plane" | "smooth"
    points: np.ndarray                # (n, 3) vertices
    normals: np.ndarray               # (k, 3) unit normals (one per facet)
    areas: np.ndarray                 # (k,)
    centres: np.ndarray = None        # (k, 3) facet centres
    polys: list = None                # planar: [outer polygon (m, 3)]


@dataclass
class Cylinder:
    point: np.ndarray                 # on the axis (middle of the extent)
    axis: np.ndarray
    radius: float
    t0: float                         # extent along the axis from ``point``
    t1: float
    convex: bool                      # shaft (True) or bore (False)
    span: float                       # angle covered, degrees
    rms: float                        # circle fit residual / radius


@dataclass
class Sphere:
    centre: np.ndarray
    radius: float
    convex: bool                      # ball (True) or socket (False)
    rms: float


@dataclass
class Plane:
    normal: np.ndarray
    offset: float                     # normal . x
    poly: np.ndarray                  # (m, 3)
    area: float


@dataclass
class Part:
    key: object                       # the caller's id (group uid)
    name: str
    cylinders: list = field(default_factory=list)
    planes: list = field(default_factory=list)
    spheres: list = field(default_factory=list)
    points: np.ndarray = None
    lo: np.ndarray = None
    hi: np.ndarray = None
    volume: float = None              # for choosing the frame (else the box)


@dataclass
class Proposal:
    kind: str                         # "pivot" | "slider" | "gears"
    a: object                         # part keys (gears: the two gear parts)
    b: object
    point: np.ndarray = None
    axis: np.ndarray = None
    score: float = 0.0
    detail: str = ""
    extra: dict = field(default_factory=dict)


def _unit(v):
    v = np.asarray(v, float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-15 else v


def _basis(a):
    t = np.array([1.0, 0, 0]) if abs(a[0]) < 0.9 else np.array([0, 1.0, 0])
    u = _unit(np.cross(a, t))
    return u, np.cross(a, u)


def polygon_normal_area(P):
    P = np.asarray(P, float)
    n = np.zeros(3)
    for i in range(len(P)):
        n += np.cross(P[i], P[(i + 1) % len(P)])
    a = 0.5 * np.linalg.norm(n)
    return (_unit(n), a)


def polygons_normal_area(polys):
    """:func:`polygon_normal_area` of many polygons at once: (n, 3) unit
    normals, (n,) areas (polygons of one size are done together)."""
    n = len(polys)
    N, A = np.zeros((n, 3)), np.zeros(n)
    by_len = {}
    for i, P in enumerate(polys):
        by_len.setdefault(len(P), []).append(i)
    for L, idx in by_len.items():
        if L == 0:
            continue
        Q = np.array([np.asarray(polys[i], float) for i in idx]).reshape(len(idx), L, 3)
        R = np.roll(Q, -1, axis=1)
        c = np.stack([Q[..., 1] * R[..., 2] - Q[..., 2] * R[..., 1],
                      Q[..., 2] * R[..., 0] - Q[..., 0] * R[..., 2],
                      Q[..., 0] * R[..., 1] - Q[..., 1] * R[..., 0]], axis=-1).sum(axis=1)
        ln = np.sqrt(np.einsum("ij,ij->i", c, c))
        N[idx] = np.where(ln[:, None] > 1e-15, c / np.where(ln > 1e-15, ln, 1.0)[:, None], c)
        A[idx] = 0.5 * ln
    return N, A


def regions_from_faces(faces, soft_pairs):
    """Regions from polygons: ``faces`` = list of (m, 3) arrays,
    ``soft_pairs`` = (i, j) indices of faces sharing a soft edge."""
    parent = list(range(len(faces)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    soft = set()
    for i, j in soft_pairs:
        soft.update((i, j))
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj
    groups = {}
    for i in range(len(faces)):
        groups.setdefault(find(i), []).append(i)
    out = []
    allN, allA = polygons_normal_area([np.asarray(f, float) for f in faces])
    for idx in groups.values():
        polys = [np.asarray(faces[i], float) for i in idx]
        normals = allN[idx]
        areas = allA[idx]
        pts = np.vstack(polys)
        centres = np.array([P.mean(axis=0) for P in polys])
        if len(idx) == 1 and idx[0] not in soft:
            out.append(Region("plane", pts, normals, areas, centres, polys))
        else:
            out.append(Region("smooth", pts, normals, areas, centres))
    return out


def fit_cylinder(reg, tol_angle=3.0, tol_fit=0.02):
    """A Cylinder if the smooth region is one, else None."""
    if reg.kind != "smooth" or len(reg.normals) < 3:
        return None
    N, A = reg.normals, reg.areas
    M = (N * A[:, None]).T @ N
    w, V = np.linalg.eigh(M)
    a = V[:, 0]
    if np.max(np.abs(N @ a)) > math.sin(math.radians(tol_angle)):
        return None
    if w[1] < 1e-3 * w[2]:                      # all normals one way: a plane
        return None
    u, v = _basis(a)
    P = reg.points
    x, y = P @ u, P @ v
    # algebraic circle fit: x^2 + y^2 + D x + E y + F = 0
    Am = np.column_stack([x, y, np.ones_like(x)])
    b = -(x * x + y * y)
    (D, E, F), *_ = np.linalg.lstsq(Am, b, rcond=None)
    cx, cy = -D / 2, -E / 2
    r2 = cx * cx + cy * cy - F
    if r2 <= 0:
        return None
    R = math.sqrt(r2)
    res = np.sqrt((x - cx) ** 2 + (y - cy) ** 2) - R
    rms = float(np.sqrt(np.mean(res ** 2))) / R
    if rms > tol_fit:
        return None
    t = P @ a
    c0 = cx * u + cy * v                         # axis point at t = 0
    # shaft: the facets face away from the axis; bore: towards it
    C = reg.centres
    rad = C - c0
    rad -= np.outer(rad @ a, a)
    convex = float(np.sum(A * np.einsum("ij,ij->i", N, rad))) > 0
    ang_p = np.arctan2(y - cy, x - cx)
    # angle covered: the circle less its gaps (a gap = much wider than the
    # usual spacing of the points around it)
    ang = np.unique(np.round(np.sort(ang_p), 9))
    if len(ang) < 3:
        return None
    gaps = np.diff(np.concatenate([ang, [ang[0] + 2 * np.pi]]))
    med = float(np.median(gaps))
    span = 360.0 - math.degrees(float(gaps[gaps > 3 * med].sum()))
    t0, t1 = float(t.min()), float(t.max())
    mid = 0.5 * (t0 + t1)
    return Cylinder(c0 + mid * a, a, R, t0 - mid, t1 - mid, bool(convex), span, rms)


def fit_sphere(reg, tol_fit=0.02):
    """A Sphere if the smooth region is part of one (its normals spread in
    every direction, its points at one distance from a centre), else None."""
    if reg.kind != "smooth" or len(reg.points) < 8 or len(reg.normals) < 4:
        return None
    N, A = reg.normals, reg.areas
    w = np.linalg.eigvalsh((N * A[:, None]).T @ N)
    if w[0] < 0.02 * w[2]:                      # a cylinder or a cone, not a ball
        return None
    P = reg.points
    Am = np.column_stack([2 * P, np.ones(len(P))])
    b = np.sum(P * P, axis=1)
    sol, *_ = np.linalg.lstsq(Am, b, rcond=None)
    c = sol[:3]
    r2 = sol[3] + c @ c
    if r2 <= 0:
        return None
    R = math.sqrt(r2)
    rms = float(np.sqrt(np.mean((np.linalg.norm(P - c, axis=1) - R) ** 2))) / R
    if rms > tol_fit:
        return None
    convex = float(np.sum(A * np.einsum("ij,ij->i", N, reg.centres - c))) > 0
    return Sphere(c, R, bool(convex), rms)


def make_part(key, name, regions, volume=None):
    p = Part(key, name, volume=volume)
    pts = []
    for r in regions:
        pts.append(r.points)
        if r.kind == "plane":
            n = _unit(np.sum(r.normals * r.areas[:, None], axis=0))
            p.planes.append(Plane(n, float(n @ r.points.mean(axis=0)), r.polys[0],
                                  float(r.areas.sum())))
        else:
            c = fit_cylinder(r)
            if c is not None:
                p.cylinders.append(c)
            else:
                sph = fit_sphere(r)
                if sph is not None:
                    p.spheres.append(sph)
    p.points = np.vstack(pts) if pts else np.zeros((0, 3))
    p.lo, p.hi = (p.points.min(0), p.points.max(0)) if len(p.points) else (np.zeros(3),) * 2
    return p


# -- matching ------------------------------------------------------------------

def _near(p, q, gap):
    return bool(np.all(p.lo - gap <= q.hi) and np.all(q.lo - gap <= p.hi))


def _pivots(p, q, size):
    """Shaft of one in a bore of the other, coaxial."""
    found = []
    for s_part, h_part in ((p, q), (q, p)):
        for s in s_part.cylinders:
            if not s.convex:
                continue
            for h in h_part.cylinders:
                if h.convex:
                    continue
                if abs(abs(s.axis @ h.axis) - 1) > 1 - math.cos(math.radians(1.5)):
                    continue
                d = h.point - s.point
                off = np.linalg.norm(d - (d @ s.axis) * s.axis)
                gap = h.radius - s.radius
                tol = max(1e-4, 0.02 * s.radius, 2e-4 * size)
                if off > tol or gap < -tol or gap > max(5e-4, 0.12 * s.radius):
                    continue
                # overlap along the axis
                th = d @ s.axis
                sign = 1.0 if s.axis @ h.axis > 0 else -1.0
                h0, h1 = sorted((th + sign * h.t0, th + sign * h.t1))
                lo, hi = max(s.t0, h0), min(s.t1, h1)
                ov = hi - lo
                if ov <= 0.05 * s.radius:
                    continue
                fit = (1 - min(1.0, off / tol) * 0.5) * (1 - min(1.0, max(gap, 0) / (0.12 * s.radius)) * 0.5)
                cover = min(1.0, min(s.span, h.span) / 300.0)
                score = fit * min(1.0, 0.3 + ov / (2 * s.radius)) * (0.6 + 0.4 * cover)
                found.append(dict(point=s.point + 0.5 * (lo + hi) * s.axis, axis=s.axis,
                                  radius=s.radius, gap=gap, overlap=ov, score=score,
                                  shaft=s_part.key))
    if not found:
        return None
    # one pivot per pair: the coaxial fits together (main axis = best fit)
    best = max(found, key=lambda f: f["score"])
    same = [f for f in found if abs(abs(f["axis"] @ best["axis"]) - 1) < 1e-3
            and np.linalg.norm(np.cross(f["point"] - best["point"], best["axis"]))
            < max(1e-4, 0.05 * best["radius"])]
    score = min(1.0, sum(f["score"] for f in same))
    ov = sum(f["overlap"] for f in same)
    pts = np.array([f["point"] for f in same])
    det = (f"Ø {2000 * best['radius']:.1f} mm, jeu {1000 * max(best['gap'], 0):.2f} mm, "
           f"portée {1000 * ov:.1f} mm" + (f" ({len(same)} appuis)" if len(same) > 1 else ""))
    return Proposal("pivot", p.key, q.key, pts.mean(axis=0), _unit(best["axis"]), score, det,
                    dict(radius=best["radius"], fits=len(same), gap=max(best["gap"], 0.0),
                         shaft=best["shaft"]))


def _ball(p, q, size):
    """A ball of one in a spherical socket of the other: same centre, the
    socket as large as the ball or slightly larger."""
    best = None
    for b_part, s_part in ((p, q), (q, p)):
        for b in b_part.spheres:
            if not b.convex:
                continue
            for s in s_part.spheres:
                if s.convex:
                    continue
                off = float(np.linalg.norm(b.centre - s.centre))
                gap = s.radius - b.radius
                tol = max(1e-4, 0.03 * b.radius, 2e-4 * size)
                if off > tol or gap < -tol or gap > max(5e-4, 0.12 * b.radius):
                    continue
                score = (1 - 0.5 * min(1.0, off / tol)) * (1 - 0.5 * min(1.0, max(gap, 0) / (0.12 * b.radius)))
                if best is None or score > best[0]:
                    best = (score, 0.5 * (b.centre + s.centre), b.radius, max(gap, 0.0))
    if best is None:
        return None
    score, c, r, gap = best
    return Proposal("ball", p.key, q.key, c, None, score,
                    f"Ø {2000 * r:.1f} mm, jeu {1000 * gap:.2f} mm", dict(radius=r, gap=gap))


def merge_parts(key, name, parts):
    """One Part from several (a body made of several groups)."""
    m = Part(key, name)
    for q in parts:
        m.cylinders += q.cylinders
        m.planes += q.planes
        m.spheres += q.spheres
    pts = [q.points for q in parts if q.points is not None and len(q.points)]
    m.points = np.vstack(pts) if pts else np.zeros((0, 3))
    m.lo, m.hi = (m.points.min(0), m.points.max(0)) if len(m.points) else (np.zeros(3),) * 2
    vols = [q.volume for q in parts if q.volume]
    m.volume = sum(vols) if vols else None
    return m


def between(p, q, kind, size=None):
    """The joint of ``kind`` between two parts, found from their shapes:
    "pivot" / "cylindrical" (a shaft in a bore), "slider" (flat contacts),
    "ball" (a ball in a socket). A Proposal, or None."""
    if size is None:
        pts = np.vstack([x.points for x in (p, q) if len(x.points)])
        size = float(np.linalg.norm(pts.max(0) - pts.min(0))) or 1.0
    if kind in ("pivot", "cylindrical"):
        return _pivots(p, q, size)
    if kind == "slider":
        sl = _slider(p, q, size)
        if sl is None:                  # a round slide: a shaft in a bore, sliding
            pv = _pivots(p, q, size)
            if pv is not None:
                return Proposal("slider", p.key, q.key, pv.point, pv.axis, pv.score, pv.detail)
        return sl
    if kind == "ball":
        return _ball(p, q, size)
    return None


def _axles(parts, pivots):
    """A separate pin through two parts (a hinge pin, a shoulder bolt) gives
    two pivots on one axis. The pin goes with the part it fits tightest
    (press fit) and the two parts get one pivot between them."""
    by = {p.key: p for p in parts}
    out, gone = [], set()
    for P in parts:
        mine = [x for x in pivots if P.key in (x.a, x.b)]
        if len(mine) < 2 or not P.cylinders or any(not c.convex for c in P.cylinders):
            continue
        ax = mine[0].axis
        if any(abs(abs(x.axis @ ax) - 1) > 1e-3
               or np.linalg.norm(np.cross(x.point - mine[0].point, ax)) > 1e-4 for x in mine):
            continue
        others = [(x, x.b if x.a == P.key else x.a) for x in mine]
        if P.volume and any(by[o].volume and abs(by[o].volume) < abs(P.volume) for _, o in others):
            continue                                  # the "pin" is the bigger part
        host_x, host = min(others, key=lambda t: t[0].extra.get("gap", 0.0))
        for x, o in others:
            if o == host:
                continue
            out.append(Proposal("pivot", host, o, 0.5 * (x.point + host_x.point), x.axis,
                                min(x.score, host_x.score),
                                x.detail + f" — axe « {P.name} » avec « {by[host].name} »",
                                dict(x.extra, merge={P.key: host})))
        gone.update(id(x) for x in mine)
    return [x for x in pivots if id(x) not in gone] + out


def _overlap_2d(P, Q, n):
    """Overlap area estimate of two coplanar polygons (bounding rectangles
    in the plane)."""
    u, v = _basis(n)
    a = np.column_stack([P @ u, P @ v])
    b = np.column_stack([Q @ u, Q @ v])
    lo = np.maximum(a.min(0), b.min(0))
    hi = np.minimum(a.max(0), b.max(0))
    d = hi - lo
    return float(d[0] * d[1]) if np.all(d > 0) else 0.0


def _slider(p, q, size):
    tol = max(1e-4, 2e-4 * size)
    contacts = []
    if p.planes and q.planes:
        # facing and coplanar, all pairs at once; the overlap only for those
        Np = np.array([a.normal for a in p.planes])
        Nq = np.array([b.normal for b in q.planes])
        Op = np.array([a.offset for a in p.planes])
        Oq = np.array([b.offset for b in q.planes])
        cand = np.nonzero((Np @ Nq.T <= -math.cos(math.radians(1.0)))
                          & (np.abs(Op[:, None] + Oq[None, :]) <= tol))
        for i, k in zip(*cand):
            a, b = p.planes[i], q.planes[k]
            ov = _overlap_2d(a.poly, b.poly, a.normal)
            if ov > 1e-8:
                contacts.append((a.normal, ov))
    if len(contacts) < 2:
        return None
    # the most different pair of contact normals sets the direction
    best, dirn = 0.0, None
    for i in range(len(contacts)):
        for j in range(i + 1, len(contacts)):
            c = np.cross(contacts[i][0], contacts[j][0])
            if np.linalg.norm(c) > best:
                best, dirn = np.linalg.norm(c), _unit(c)
    if best < math.sin(math.radians(25)):
        return None
    if any(abs(n @ dirn) > math.sin(math.radians(2)) for n, _ in contacts):
        return None                                     # a face across the way: blocked
    area = sum(ov for _, ov in contacts)
    centre = 0.5 * (np.maximum(p.lo, q.lo) + np.minimum(p.hi, q.hi))
    score = min(1.0, 0.5 + 0.1 * len(contacts)) * min(1.0, best / math.sin(math.radians(60)))
    return Proposal("slider", p.key, q.key, centre, dirn, score,
                    f"{len(contacts)} appuis plans, {1e4 * area:.1f} cm²")


def count_teeth(points, centre, axis, min_teeth=6, max_depth=0.35):
    """Teeth of a gear-like outline around ``axis``: (z, tip radius, root
    radius) or None."""
    u, v = _basis(axis)
    d = points - centre
    d = d - np.outer(d @ axis, axis)
    r = np.hypot(d @ u, d @ v)
    ang = np.arctan2(d @ v, d @ u)
    keep = r > 0.6 * r.max()                     # the outline, not the bore
    r, ang = r[keep], ang[keep]
    nb = 1440
    prof = np.zeros(nb)
    k = ((ang + np.pi) / (2 * np.pi) * nb).astype(int) % nb
    np.maximum.at(prof, k, r)
    filled = prof > 0
    if filled.sum() < 4 * min_teeth:
        return None
    # outline vertices only: fill the bins between them
    idx = np.nonzero(filled)[0]
    prof = np.interp(np.arange(nb), idx, prof[idx], period=nb)
    rmax, rmin = prof.max(), prof.min()
    if rmax <= 0 or (rmax - rmin) / rmax < 0.02 or (rmax - rmin) / rmax > max_depth:
        return None
    mid = 0.5 * (rmax + rmin)
    above = prof > mid
    z = int(np.sum(above & ~np.roll(above, 1)))
    if z < min_teeth:
        return None
    return z, float(rmax), float(rmin)


def teeth_of(points, centre, axis, min_teeth=6):
    """(z, tip radius, root radius) of a toothed wheel about ``axis``, or
    None. The whole outline first; then thin slices across the axis, one
    after the other: the teeth of a helical wheel twist along it, so all
    its slices together fill the tip circle (Nicolas's Geneva wheels)."""
    P = np.asarray(points, float)
    t = count_teeth(P, centre, axis, min_teeth)
    if t and _regular_teeth(P, centre, axis, t[0]):
        return t
    h = (P - centre) @ axis
    lo, hi = float(h.min()), float(h.max())
    L = hi - lo
    if L <= 0:
        return None
    found = []
    for f in (0.5, 0.35, 0.65, 0.2, 0.8, 0.1, 0.9):
        S = P[np.abs(h - (lo + f * L)) <= max(0.025 * L, 1e-5)]
        if len(S) < 8 * min_teeth:
            continue
        r = count_teeth(S, centre, axis, min_teeth, max_depth=0.45)
        if r and _regular_teeth(S, centre, axis, r[0]):
            found.append(r)
    if not found:
        return None
    zs = [r[0] for r in found]
    z = max(set(zs), key=zs.count)
    tips = [r[1] for r in found if r[0] == z]
    roots = [r[2] for r in found if r[0] == z]
    return z, float(max(tips)), float(min(roots))


def _regular_teeth(points, centre, axis, z, share=0.45):
    """The outline around ``axis`` repeats ``z`` times, evenly: most of its
    ups and downs at that one frequency (a bar with two round ends is
    not a wheel of 9 teeth)."""
    u, v = _basis(axis)
    d = points - centre
    d = d - np.outer(d @ axis, axis)
    r = np.hypot(d @ u, d @ v)
    ang = np.arctan2(d @ v, d @ u)
    keep = r > 0.6 * r.max()
    r, ang = r[keep], ang[keep]
    nb = 1440
    prof = np.zeros(nb)
    k = ((ang + np.pi) / (2 * np.pi) * nb).astype(int) % nb
    np.maximum.at(prof, k, r)
    idx = np.nonzero(prof > 0)[0]
    if len(idx) < 4 * z:
        return False
    prof = np.interp(np.arange(nb), idx, prof[idx], period=nb)
    F = np.abs(np.fft.rfft(prof - prof.mean())) ** 2
    total = F[1:].sum()
    if total <= 0:
        return False
    near = F[max(z - 1, 1):z + 2].sum() + F[2 * z - 1:2 * z + 2].sum()   # the teeth and their
    return near / total >= share                                         # sharper shape


def own_axis(part, min_teeth=6, near=None):
    """The axis a part turns about by itself, read from its shape alone
    (no shaft needed in a bore): a toothed wheel's (the centre of its
    teeth), else its largest bore, else its largest round outside (a
    shaft, a wheel, a pulley). (point, axis, kind, detail) or None; the
    point sits halfway along the part on that axis. ``near``: points of
    the part it turns in (the frame): of two bores, the one nearest it."""
    P = part.points if part.points is not None else np.zeros((0, 3))
    if len(P) < 8:
        return None
    round_ = [c for c in part.cylinders if c.span >= 180.0]

    def mid(c, ax):
        t = (P - c) @ ax
        return c + 0.5 * (t.min() + t.max()) * ax

    # a toothed wheel: its own axes first (bores), then the directions of
    # the points (a flat wheel: the thin one), each recentred on the outline
    cands = [(np.asarray(c.point, float), _unit(c.axis)) for c in
             sorted(round_, key=lambda c: (c.convex, -c.radius))]
    C = P.mean(axis=0)
    w, V = np.linalg.eigh(np.cov((P - C).T))
    cands += [(C, np.eye(3)[k]) for k in range(3)]          # drawn square to the axes
    cands += [(C, _unit(V[:, k])) for k in range(3)]
    for c, ax in cands:
        for centre in (c, None):
            if centre is None:                  # the middle of the outline
                u, v = _basis(ax)
                d = P - c
                pu, pv = d @ u, d @ v
                centre = c + u * 0.5 * (pu.max() + pu.min()) + v * 0.5 * (pv.max() + pv.min())
            t = teeth_of(P, centre, ax, min_teeth)
            if t:
                z, rt, rr = t
                return (mid(centre, ax), ax, "gear",
                        f"denture {z} dents, Ø tête {2000 * rt:.1f} mm")
    Q = None
    if near is not None and len(near):
        Q = np.asarray(near, float)
        Q = Q[:: max(1, len(Q) // 4000)]

    def gap(c):                               # from the axis to the other part
        if Q is None:
            return 0.0
        ax = _unit(c.axis)
        d = Q - c.point
        return float(np.min(np.linalg.norm(d - np.outer(d @ ax, ax), axis=1)))

    if not round_:
        return None
    # the round shape nearest the part it turns in (its pin or its bore
    # is there); else a bore before a round outside, the largest first
    c = min(round_, key=lambda c: (round(gap(c), 4), c.convex, -c.radius, -c.span))
    ax = _unit(c.axis)
    what = "forme ronde" if c.convex else "alésage"
    return (mid(np.asarray(c.point, float), ax), ax, "round" if c.convex else "bore",
            f"{what} Ø {2000 * c.radius:.1f} mm")


def _gears(parts, pivots, size):
    by = {p.key: p for p in parts}
    out = []
    for i in range(len(pivots)):
        for j in range(i + 1, len(pivots)):
            A, B = pivots[i], pivots[j]
            common = {A.a, A.b} & {B.a, B.b}
            if len(common) != 1:
                continue
            f = common.pop()
            ga = A.b if A.a == f else A.a
            gb = B.b if B.a == f else B.a
            if ga == gb or abs(abs(A.axis @ B.axis) - 1) > 1e-3:
                continue
            d = B.point - A.point
            dist = np.linalg.norm(d - (d @ A.axis) * A.axis)
            ta = teeth_of(by[ga].points, A.point, A.axis)
            tb = teeth_of(by[gb].points, B.point, B.axis)
            if not ta or not tb:
                continue
            (za, ra, fa), (zb, rb, fb) = ta, tb
            depth = 0.5 * ((ra - fa) + (rb - fb))
            # meshing: the tip circles overlap by about one tooth depth
            ok = abs((ra + rb) - dist - depth) < 0.6 * depth + 2e-4 * size
            if not ok:
                continue
            ratio = -za / zb
            out.append(Proposal("gears", ga, gb, None, A.axis, 0.8,
                                f"{za} et {zb} dents, rapport {za}/{zb}",
                                dict(pivot_a=A, pivot_b=B, ratio=ratio, frame=f)))
    return out


def scan(parts, size=None):
    """Proposals for ``parts`` (list of :class:`Part`) and the key of the
    part proposed as the frame. ``size``: model size (tolerances)."""
    if size is None:
        pts = np.vstack([p.points for p in parts if len(p.points)]) if parts else np.zeros((1, 3))
        size = float(np.linalg.norm(pts.max(0) - pts.min(0))) or 1.0
    gap = max(5e-4, 2e-3 * size)
    props = []
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            p, q = parts[i], parts[j]
            if not _near(p, q, gap):
                continue
            pv = _pivots(p, q, size)
            if pv:
                props.append(pv)
            sl = _slider(p, q, size)
            if sl and not pv:
                props.append(sl)
            bl = _ball(p, q, size) if not pv else None
            if bl:
                props.append(bl)
    pivots = _axles(parts, [x for x in props if x.kind == "pivot"])
    props = pivots + [x for x in props if x.kind != "pivot"]
    props += _gears(parts, pivots, size)
    # frame: the part the most joints lean on, then the largest
    deg = {p.key: 0 for p in parts}
    for x in props:
        if x.kind != "gears":
            deg[x.a] += 1
            deg[x.b] += 1
    vol = {p.key: abs(p.volume) if p.volume else float(np.prod(np.maximum(p.hi - p.lo, 1e-9)))
           for p in parts}
    frame = max(deg, key=lambda k: (deg[k], vol[k])) if deg else None
    props.sort(key=lambda x: ({"pivot": 0, "slider": 1, "ball": 2, "gears": 3}[x.kind], -x.score))
    return props, frame


def part_from_step(part, key=None):
    """:class:`Part` of a part read by :mod:`motion.stepread`."""
    from .stepread import volume
    return make_part(part.name if key is None else key, part.name,
                     regions_from_step_part(part), volume(part.triangles))


def regions_from_step_part(part):
    """Regions of a part read by :mod:`motion.stepread` (one per face)."""
    out = []
    for f in part.faces:
        if f.loops is not None and len(f.loops) and len(f.loops[0]) >= 3:
            P = part.world(np.asarray(f.loops[0], float))
            n, a = polygon_normal_area(P)
            out.append(Region("plane", P, n[None], np.array([a]), P.mean(axis=0)[None], [P]))
        elif len(f.triangles):
            T = part.world(f.triangles.reshape(-1, 3)).reshape(-1, 3, 3)
            nrm = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
            ar = 0.5 * np.linalg.norm(nrm, axis=1)
            ok = ar > 1e-16
            if ok.sum() < 3:
                continue
            T, nrm, ar = T[ok], nrm[ok], ar[ok]
            out.append(Region("smooth", T.reshape(-1, 3), nrm / (2 * ar[:, None]), ar,
                              T.mean(axis=1)))
    return out
