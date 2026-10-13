"""Interference between bodies: oriented boxes, separating-axis test."""
import numpy as np


def world_obb(body, q, box):
    R = body.rot(q)
    c = body.world_point(q, box.center)
    return c, R, box.half


def obb_overlap(A, B, margin=0.0):
    """True if two OBBs (centre, R, half) overlap by more than ``margin``
    (margin < 0 -> require a clearance of -margin)."""
    ca, Ra, ha = A
    cb, Rb, hb = B
    t = cb - ca
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for k in range(3):
            c = np.cross(Ra[:, i], Rb[:, k])
            if np.linalg.norm(c) > 1e-9:
                axes.append(c / np.linalg.norm(c))
    for ax in axes:
        ra = sum(ha[i] * abs(Ra[:, i] @ ax) for i in range(3))
        rb = sum(hb[i] * abs(Rb[:, i] @ ax) for i in range(3))
        if abs(t @ ax) > ra + rb - margin:
            return False
    return True


def interferences(mech, x, margin=1e-6, ignore_joined=True):
    """List of (body_a, body_b) pairs that overlap at configuration ``x``."""
    joined = set()
    if ignore_joined:
        for j in mech.joints:
            joined.add(frozenset((j.a.name, j.b.name)))
    bodies = [mech.ground] + mech.bodies
    hits = []
    for i, a in enumerate(bodies):
        for b in bodies[i + 1:]:
            if frozenset((a.name, b.name)) in joined:
                continue
            qa, qb = mech.pose(x, a), mech.pose(x, b)
            if any(obb_overlap(world_obb(a, qa, ba), world_obb(b, qb, bb),
                               margin)
                   for ba in a.boxes for bb in b.boxes):
                hits.append((a.name, b.name))
    return hits


def limit_violations(mech, x, tol=1e-6):
    out = []
    for j in mech.joints:
        if j.limits is None:
            continue
        v = mech.joint_value(j, x)
        if v < j.limits[0] - tol or v > j.limits[1] + tol:
            out.append((j.name, v, j.limits))
    return out
