"""Guide / rail joint and the sectional garage door, against closed forms."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion.model import Mechanism, Box, Path
from motion import samples, solver, collide

TOL = 1e-8


def test_rail_geometry():
    H, R, Lh = 2.0, 0.38, 2.3
    r = samples.rail_porte(H, R, Lh)
    assert abs(r.length - (H + math.pi / 2 * R + Lh)) < 1e-12
    a = math.pi / 4
    p = r.point(H + a * R)
    assert np.linalg.norm(p - (R - R * math.cos(a), 0, H + R * math.sin(a))) < 1e-12
    assert np.linalg.norm(r.end - (R + Lh, 0, H + R)) < 1e-12
    for s in (H, H + math.pi / 2 * R):                  # tangent continuity
        assert np.linalg.norm(r.tangent(s - 1e-9) - r.tangent(s + 1e-9)) < 1e-6
    # closest point of a point 5 cm off the curve, towards the centre
    q = (R - (R - 0.05) * math.cos(a), 0, H + (R - 0.05) * math.sin(a))
    s, c = r.closest(q)
    assert abs(s - (H + a * R)) < 1e-12 and abs(np.linalg.norm(c - q) - 0.05) < 1e-12


def _trammel(L=1.0, lam=0.3):
    """Rod whose ends run in a vertical and a horizontal rail (two rails
    per end, y = ±0.1, to hold it in its plane)."""
    m = Mechanism()
    z0 = L * math.sin(math.radians(60))
    A, B = np.array([0, 0, z0]), np.array([L * math.cos(math.radians(60)), 0, 0])
    rod = m.body("tige", (A + B) / 2, [])
    for y in (-0.1, 0.1):
        Y = np.array([0, y, 0])
        m.guide(m.ground, rod, A + Y, Path(Y + (0, 0, 0.01)).line_to(Y + (0, 0, 2 * L)),
                name=f"A{y}")
        m.guide(m.ground, rod, B + Y, Path(Y + (0.01, 0, 0)).line_to(Y + (2 * L, 0, 0)),
                name=f"B{y}")
    m.drive(m.joints[0], z0 - 0.01, 0.2 * L - 0.01)   # arc length = z - 0.01
    return m, rod, A, B, L, lam


def test_trammel_traces_ellipse():
    m, rod, A, B, L, lam = _trammel()
    x, ok, _ = solver.solve(m, 0.5)
    assert ok and solver.dof(m, x) == 1
    P0 = A + lam * (B - A)
    for st in solver.sweep(m, 15):
        assert st["ok"], st
        z = st["value"] + 0.01
        Bw = rod.world_point(m.pose(st["x"], rod), B)
        assert abs(Bw[0] - math.sqrt(L * L - z * z)) < TOL      # |AB| = L
        P = rod.world_point(m.pose(st["x"], rod), P0)
        # point at fraction lam from A: x = lam*xB, z = (1-lam)*z_A
        assert abs((P[0] / (lam * L)) ** 2 + (P[2] / ((1 - lam) * L)) ** 2 - 1) < TOL
        M = rod.world_point(m.pose(st["x"], rod), (A + B) / 2)
        assert abs(np.linalg.norm(M) - L / 2) < TOL             # midpoint circle


def _rollers(m, info, x, side=1):
    out = []
    for lev in range(info["n"] + 1):
        body = info["panels"][min(lev, info["n"] - 1)]
        p = np.array([0, side * info["largeur"] / 2, lev * info["h"]])
        out.append(body.world_point(m.pose(x, body), p))
    return out


def test_door_closed_and_open():
    m, info = samples.porte_sectionnelle()
    h, R, H = info["h"], info["R"], info["H"]
    x, ok, _ = solver.solve(m, 0.0)
    assert ok
    for k, p in enumerate(_rollers(m, info, x)):
        assert np.linalg.norm(p - (0, 1.2, k * h)) < TOL
    x, ok, _ = solver.solve(m, info["s_open"], x0=solver.sweep(m, 11)[-2]["x"])
    assert ok
    for k, p in enumerate(_rollers(m, info, x, -1)):
        assert np.linalg.norm(p - (R + k * h, -1.2, H + R)) < TOL


def test_door_two_rollers_on_the_curve():
    m, info = samples.porte_sectionnelle()
    h, R, H, n = info["h"], info["R"], info["H"], info["n"]
    s0 = H - (n - 1) * h                       # roller n-1 at the curve start
    x = None
    for v in np.linspace(0, s0, 8):            # follow the motion up to it
        x, ok, _ = solver.solve(m, v, x)
    assert ok
    a = 2 * math.asin(h / (2 * R))             # chord h on radius R
    top = _rollers(m, info, x)[-1]
    assert np.linalg.norm(top - (R - R * math.cos(a), 1.2, H + R * math.sin(a))) < TOL
    assert abs(m.joint_value(info["rollers"][-1], x) - (H + a * R)) < TOL


def test_door_matches_independent_marching():
    m, info = samples.porte_sectionnelle()
    for t in (0.3, 0.6, 0.9):
        x, ok, _ = solver.solve(m, t * info["s_open"])
        assert ok and solver.dof(m, x) == 1
    for st in solver.sweep(m, 21):
        assert st["ok"], st
        s_ref, p_ref = samples.porte_reference(info, st["value"])
        for side in (1, -1):
            pts = _rollers(m, info, st["x"], side)
            for k, (p, q) in enumerate(zip(pts, p_ref)):
                assert np.linalg.norm(p - q - (0, (side - 1) * 1.2, 0)) < TOL
                if k:
                    assert abs(np.linalg.norm(p - pts[k - 1]) - info["h"]) < TOL
        for j, s in zip(info["rollers"], s_ref):
            assert abs(m.joint_value(j, st["x"]) - s) < TOL


def test_door_lintel_clearance():
    m, info = samples.porte_sectionnelle()
    rep = collide.validate(m, 21, jeu_min=0.005)
    assert rep["ok"], rep["problems"]
    w = rep["worst"]
    assert {w["a"], w["b"]} == {"bâti", "panneau4"} and w["t"] == 0.0
    assert abs(w["distance"] - info["jeu_linteau"]) < 1e-12
    assert not collide.validate(m, 3, jeu_min=0.015)["ok"]


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
