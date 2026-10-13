"""Position solver: Levenberg-Marquardt on joint constraints.

Works for open chains and closed loops alike (four-bar, scissor...).
"""
import numpy as np
from .model import perp_basis


def residual(mech, x, driver_value=None):
    r = []
    for j in mech.joints:
        qa, qb = mech.pose(x, j.a), mech.pose(x, j.b)
        Ra, Rb = j.a.rot(qa), j.b.rot(qb)
        pa = j.a.world_point(qa, j.point)
        pb = j.b.world_point(qb, j.point)
        ua, ub = Ra @ j.axis, Rb @ j.axis
        if j.kind == "pivot":
            r.extend(pa - pb)                 # same point
            r.extend(np.cross(ua, ub))        # same axis
        else:
            d = pb - pa
            r.extend(d - ua * (ua @ d))       # stays on the line
            Rr = Ra.T @ Rb                    # no relative rotation
            r.extend([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0],
                      Rr[1, 0] - Rr[0, 1]])
    if driver_value is not None:
        j = mech.driver[0]
        r.append(mech.joint_value(j, x) - driver_value)
    return np.array(r, float)


def jacobian(mech, x, driver_value=None, eps=1e-7):
    r0 = residual(mech, x, driver_value)
    J = np.zeros((r0.size, x.size))
    for k in range(x.size):
        xp = x.copy(); xp[k] += eps
        xm = x.copy(); xm[k] -= eps
        J[:, k] = (residual(mech, xp, driver_value)
                   - residual(mech, xm, driver_value)) / (2 * eps)
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
