"""Position solver: Levenberg-Marquardt on joint constraints.

Works for open chains and closed loops alike (four-bar, scissor...).
"""
import math
import numpy as np
from .model import perp_basis, cross3


ROWS = {"guide": 3, "coupling": 1, "ball": 3,      # residual rows (default 6)
        "annular": 3, "pointplane": 1, "planar": 4, "screw": 7}


def rows_of(j):
    return ROWS.get(j.kind, 6)


def _joint_residual(mech, j, x):
    if j.kind == "coupling":
        return np.array([mech.continuous_value(j.jb, x)
                         - j.ratio * mech.continuous_value(j.ja, x)])
    qa, qb = mech.pose(x, j.a), mech.pose(x, j.b)
    if j.kind == "guide":                     # point on the rail (rank 2)
        p = j.a.local_point(qa, j.b.world_point(qb, j.point))
        return p - j.path.closest(p)[1]
    Ra, Rb = j.a.rot(qa), j.b.rot(qb)          # one rotation per part
    p = np.asarray(j.point, float)
    pa = p if j.a.fixed else j.a.center + qa[:3] + Ra @ (p - j.a.center)
    pb = p if j.b.fixed else j.b.center + qb[:3] + Rb @ (p - j.b.center)
    if j.kind == "ball":
        return pa - pb                                       # same point
    if j.kind in ("lock", "elastic"):                        # one piece (kinematics)
        Rr = Ra.T @ Rb
        return np.concatenate([pb - pa, [Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0],
                                         Rr[1, 0] - Rr[0, 1]]])
    ua, ub = Ra @ j.axis, Rb @ j.axis
    if j.kind == "annular":                                  # ball centre on the bore's axis
        d = pb - pa
        return d - ua * (ua @ d)
    if j.kind == "pointplane":                               # point on the plane
        return np.array([ua @ (pb - pa)])
    if j.kind == "planar":                                   # on the plane, normals together
        return np.concatenate([[ua @ (pb - pa)], cross3(ua, ub)])
    if j.kind == "screw":                                    # cylindrical + advance per turn
        d = pb - pa
        ang = float((mech.pose(x, j.b)[3:] - mech.pose(x, j.a)[3:]) @ j.axis)
        return np.concatenate([d - ua * (ua @ d), cross3(ua, ub),
                               [ua @ d - j.pitch * ang / (2 * np.pi)]])
    if j.kind == "pivot":
        return np.concatenate([pa - pb, cross3(ua, ub)])   # same point, axis
    d = pb - pa
    if j.kind == "cylindrical":                              # on the axis, parallel
        return np.concatenate([d - ua * (ua @ d), cross3(ua, ub)])
    Rr = Ra.T @ Rb                            # no relative rotation
    return np.concatenate([d - ua * (ua @ d),                # stays on the line
                           [Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0],
                            Rr[1, 0] - Rr[0, 1]]])


def _driver_residual(mech, x, driver_value, k=0, targets=None):
    """Residual of motor ``k`` (target from the leading motor's value)."""
    drv = mech.drivers[k]
    j = drv[0]
    target = (targets or mech.driver_targets(driver_value))[k]
    if mech.multi_turn(drv):                  # several turns: count them
        return mech.continuous_value(j, x) - target
    d = mech.joint_value(j, x) - target
    if j.kind in ("pivot", "cylindrical", "screw"):   # angles: a full turn is no error
        d = math.atan2(math.sin(d), math.cos(d))
    return d


def residual(mech, x, driver_value=None):
    r = [_joint_residual(mech, j, x) for j in mech.joints]
    if driver_value is not None:
        tg = mech.driver_targets(driver_value)
        r.append([_driver_residual(mech, x, driver_value, k, tg)
                  for k in range(len(mech.drivers))])
    return np.concatenate(r) if r else np.zeros(0)


def _joint_bodies(j):
    return j.bodies() if j.kind == "coupling" else [j.a, j.b]


def jacobian(mech, x, driver_value=None, eps=1e-7):
    """Central differences, body by body: a body's 6 columns only touch the
    joints (and motors) that body takes part in."""
    rows, at = [], 0
    for j in mech.joints:
        n = rows_of(j)
        rows.append((j, slice(at, at + n)))
        at += n
    nd = len(mech.drivers) if driver_value is not None else 0
    J = np.zeros((at + nd, x.size))
    tg = mech.driver_targets(driver_value) if nd else None
    fb = {id(b) for b in getattr(mech, "follower_bodies", lambda: [])()}
    by_body = {}
    for j, sl in rows:
        for o in {id(o) for o in _joint_bodies(j)}:
            by_body.setdefault(o, []).append((j, sl))
    drv_by = {}
    for k in range(nd):
        for o in {id(o) for o in _joint_bodies(mech.drivers[k][0])}:
            drv_by.setdefault(o, []).append(k)
    for body in mech.bodies:
        if id(body) in fb:
            continue
        mine = by_body.get(id(body), [])
        drv = drv_by.get(id(body), [])
        for c in range(6 * body.index, 6 * body.index + 6):
            xp = x.copy(); xp[c] += eps
            xm = x.copy(); xm[c] -= eps
            for j, sl in mine:
                J[sl, c] = (_joint_residual(mech, j, xp)
                            - _joint_residual(mech, j, xm)) / (2 * eps)
            for k in drv:
                J[at + k, c] = (_driver_residual(mech, xp, driver_value, k, tg)
                                - _driver_residual(mech, xm, driver_value, k, tg)) / (2 * eps)
    return J


def solve(mech, driver_value, x0=None, tol=1e-10, max_iter=100):
    """Return (x, converged, residual_norm). Follower parts (chain links)
    are left out of the iteration and placed once the joints are solved."""
    x = np.zeros(mech.n) if x0 is None else np.array(x0, float)
    if getattr(mech, "followers", None):
        fb = {id(b) for b in mech.follower_bodies()}
        act = np.array([c for b in mech.bodies if id(b) not in fb
                        for c in range(6 * b.index, 6 * b.index + 6)], int)
        xs, ok, nr = _solve(mech, driver_value, x, tol, max_iter, act)
        return mech.place(xs), ok, nr
    return _solve(mech, driver_value, x, tol, max_iter, None)


def _solve(mech, driver_value, x, tol, max_iter, act):
    lam = 1e-3
    r = residual(mech, x, driver_value)
    for _ in range(max_iter):
        nr = np.linalg.norm(r)
        if nr < tol:
            return x, True, nr
        J = jacobian(mech, x, driver_value)
        if act is not None:
            J = J[:, act]
        g = J.T @ r
        H = J.T @ J
        while True:
            try:
                d = np.linalg.solve(H + lam * np.eye(H.shape[0]), -g)
                if act is None:
                    dx = d
                else:
                    dx = np.zeros(x.size)
                    dx[act] = d
            except np.linalg.LinAlgError:
                lam *= 10
                if lam > 1e12:
                    return x, False, nr
                continue
            xn = x + dx
            rn = residual(mech, xn, driver_value)
            if np.linalg.norm(rn) < nr:
                x, r = xn, rn
                lam = max(lam / 5, 1e-12)
                break
            lam *= 5
            if lam > 1e12:
                return x, False, nr
    nr = np.linalg.norm(r)
    return x, nr < tol * 100, nr


def dof(mech, x=None, tol=1e-6):
    """Mobility at configuration ``x``: free coordinates minus constraint rank."""
    x = np.zeros(mech.n) if x is None else x
    J = jacobian(mech, x)
    s = np.linalg.svd(J, compute_uv=False)
    rank = int((s > tol * max(s.max(), 1.0)).sum()) if s.size else 0
    # chain links are placed by a rule, not free
    fol = getattr(mech, "followers", [])
    return (mech.n - rank - 6 * len(mech.follower_bodies() if fol else [])
            - sum(getattr(f, "free_dof", 0) for f in fol))


def sweep(mech, steps=51, progress=None):
    """Solve 0..100 % of the driver. Returns list of dicts with pose x.
    ``progress(i, steps)`` after each step; False stops (None returned)."""
    j, lo, hi = mech.driver
    out, x = [], None
    # every motor that is a pivot is followed in steps of at most 0.4 rad,
    # whatever ``steps`` (several turns in few steps would lose count)
    sub = 1
    if steps > 1:
        for jj, l_, h_ in mech.drivers:
            if jj.kind in ("pivot", "cylindrical", "screw"):
                sub = max(sub, int(math.ceil(abs(h_ - l_) / (steps - 1) / 0.4)))
    v_prev = lo
    for i in range(steps):
        t = i / (steps - 1)
        v = lo + t * (hi - lo)
        if x is not None:
            for k in range(1, sub):
                x, _, _ = solve(mech, v_prev + (v - v_prev) * k / sub, x)
        x, ok, nr = solve(mech, v, x)
        v_prev = v
        out.append({"t": t, "value": v, "x": x.copy(), "ok": ok, "res": nr})
        if progress is not None and progress(i + 1, steps) is False:
            return None
    return out
