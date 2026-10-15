"""Position solver: Levenberg-Marquardt on joint constraints.

Works for open chains and closed loops alike (four-bar, scissor...).
"""
import math
import numpy as np
from .model import perp_basis


def _joint_residual(mech, j, x):
    qa, qb = mech.pose(x, j.a), mech.pose(x, j.b)
    if j.kind == "guide":                     # point on the rail (rank 2)
        p = j.a.local_point(qa, j.b.world_point(qb, j.point))
        return p - j.path.closest(p)[1]
    Ra, Rb = j.a.rot(qa), j.b.rot(qb)
    pa = j.a.world_point(qa, j.point)
    pb = j.b.world_point(qb, j.point)
    ua, ub = Ra @ j.axis, Rb @ j.axis
    if j.kind == "pivot":
        return np.concatenate([pa - pb, np.cross(ua, ub)])   # same point, axis
    d = pb - pa
    Rr = Ra.T @ Rb                            # no relative rotation
    return np.concatenate([d - ua * (ua @ d),                # stays on the line
                           [Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0],
                            Rr[1, 0] - Rr[0, 1]]])


def _driver_residual(mech, x, driver_value):
    j = mech.driver[0]
    d = mech.joint_value(j, x) - driver_value
    if j.kind == "pivot":                     # angles: a full turn is no error
        d = math.atan2(math.sin(d), math.cos(d))
    return d


def residual(mech, x, driver_value=None):
    r = [_joint_residual(mech, j, x) for j in mech.joints]
    if driver_value is not None:
        r.append([_driver_residual(mech, x, driver_value)])
    return np.concatenate(r) if r else np.zeros(0)


def jacobian(mech, x, driver_value=None, eps=1e-7):
    """Central differences, body by body: a body's 6 columns only touch the
    joints that body takes part in (the rest of the column is zero)."""
    rows, at = [], 0
    for j in mech.joints:
        n = 3 if j.kind == "guide" else 6
        rows.append((j, slice(at, at + n)))
        at += n
    m_rows = at + (1 if driver_value is not None else 0)
    J = np.zeros((m_rows, x.size))
    for body in mech.bodies:
        mine = [(j, sl) for j, sl in rows if j.a is body or j.b is body]
        drv = (driver_value is not None
               and body in (mech.driver[0].a, mech.driver[0].b))
        for k in range(6 * body.index, 6 * body.index + 6):
            xp = x.copy(); xp[k] += eps
            xm = x.copy(); xm[k] -= eps
            for j, sl in mine:
                J[sl, k] = (_joint_residual(mech, j, xp)
                            - _joint_residual(mech, j, xm)) / (2 * eps)
            if drv:
                J[-1, k] = (_driver_residual(mech, xp, driver_value)
                            - _driver_residual(mech, xm, driver_value)) / (2 * eps)
    return J


def solve(mech, driver_value, x0=None, tol=1e-10, max_iter=100):
    """Return (x, converged, residual_norm)."""
    x = np.zeros(mech.n) if x0 is None else np.array(x0, float)
    lam = 1e-3
    r = residual(mech, x, driver_value)
    for _ in range(max_iter):
        nr = np.linalg.norm(r)
        if nr < tol:
            return x, True, nr
        J = jacobian(mech, x, driver_value)
        g = J.T @ r
        H = J.T @ J
        while True:
            try:
                dx = np.linalg.solve(H + lam * np.eye(x.size), -g)
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
    return mech.n - rank


def sweep(mech, steps=51):
    """Solve 0..100 % of the driver. Returns list of dicts with pose x."""
    j, lo, hi = mech.driver
    out, x = [], None
    for i in range(steps):
        t = i / (steps - 1)
        v = lo + t * (hi - lo)
        x, ok, nr = solve(mech, v, x)
        out.append({"t": t, "value": v, "x": x.copy(), "ok": ok, "res": nr})
    return out
