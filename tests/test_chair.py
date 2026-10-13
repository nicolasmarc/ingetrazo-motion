"""Folding chair with crossed legs, checked against closed-form geometry."""
import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import samples, solver, collide

TOL = 1e-8


def _pt(m, body, x, p):
    return body.world_point(m.pose(x, body), p)


def _dist_to_line(p, a, b):
    d = (b - a) / np.linalg.norm(b - a)
    v = p - a
    return float(np.linalg.norm(v - d * (v @ d)))


def test_chair_open_dimensions():
    m, info = samples.chaise_pliante()
    seat = info["bodies"]["assise"].boxes[0]
    top = seat.center[2] + seat.half[2]
    assert abs(top - 0.45) < 1e-12                       # seat height 45 cm
    assert abs(2 * seat.half[0] - 0.40) < 1e-12          # depth 40 cm
    assert abs(2 * seat.half[1] - 0.40) < 1e-12          # width 40 cm
    assert info["Fd"][2] == 0 and info["Fa"][2] == 0     # both feet on floor
    # crossing lies on both leg axes
    assert _dist_to_line(info["P"], info["Fd"], info["K"]) < 1e-12
    assert _dist_to_line(info["P"], info["Fa"], info["S0"]) < 1e-12
    assert info["Fa"][0] < info["P"][0] < info["Fd"][0]  # legs really cross
    x, ok, _ = solver.solve(m, 0.0)
    assert ok and np.abs(x).max() < 1e-12                # drawn = 0 %


def test_chair_has_one_dof():
    m, info = samples.chaise_pliante()
    for t in (0.0, 0.5, 0.97):
        x, ok, _ = solver.solve(m, t * info["phi_fold"])
        assert ok and solver.dof(m, x) == 1


def test_chair_follows_closed_form():
    m, info = samples.chaise_pliante()
    seat, pa = info["bodies"]["assise"], info["bodies"]["pied_assise"]
    slot = info["joints"]["lumiere"]
    for st in solver.sweep(m, 41):
        assert st["ok"], st
        ref = samples.chaise_reference(info, st["value"])
        S = _pt(m, seat, st["x"], info["S0"])
        S_leg = _pt(m, pa, st["x"], info["S0"])
        Q = _pt(m, seat, st["x"], info["K"])
        assert np.linalg.norm(S - ref["S"]) < TOL
        assert np.linalg.norm(S - S_leg) < TOL            # front pivot closed
        assert np.linalg.norm(Q - ref["Q"]) < TOL
        assert abs(m.joint_value(slot, st["x"]) - ref["s"]) < TOL
        assert abs(np.linalg.norm(Q - S) - info["L"]) < TOL
        ang = math.atan2(Q[2] - S[2], Q[0] - S[0])
        assert abs(math.remainder(ang - ref["seat_angle"], 2 * math.pi)) < TOL
        assert abs(Q[1]) < TOL and abs(S[1]) < TOL        # stays planar


def test_chair_folds_flat():
    m, info = samples.chaise_pliante()
    x, ok, _ = solver.solve(m, info["phi_fold"])
    assert ok
    pa, seat = info["bodies"]["pied_assise"], info["bodies"]["assise"]
    Fa = _pt(m, pa, x, info["Fa"])
    S = _pt(m, pa, x, info["S0"])
    # the seat-carrying leg lies on the other leg's axis
    assert _dist_to_line(Fa, info["Fd"], info["K"]) < TOL
    assert _dist_to_line(S, info["Fd"], info["K"]) < TOL
    # seat parallel to the backrest
    Q = _pt(m, seat, x, info["K"])
    v = (Q - S) / np.linalg.norm(Q - S)
    assert np.linalg.norm(np.cross(v, info["u"])) < TOL
    assert not collide.interferences(m, x)


def test_chair_slot_travel_maximum():
    m, info = samples.chaise_pliante()
    P, K, u, r, L = info["P"], info["K"], info["u"], info["r"], info["L"]
    s_max = samples._s_max(K, u, P, r, L)
    Qs = K + s_max * u
    assert abs(np.linalg.norm(Qs - P) - (r + L)) < 1e-12
    # angle where pivot, front axis and pin are aligned
    phi_star = samples._angle(info["S0"] - P, Qs - P)
    assert 0 < phi_star < info["phi_fold"]
    x, ok, _ = solver.solve(m, phi_star)
    assert ok
    assert abs(m.joint_value(info["joints"]["lumiere"], x) - s_max) < 1e-7
    travel = [m.joint_value(info["joints"]["lumiere"], st["x"])
              for st in solver.sweep(m, 81)]
    assert max(travel) <= s_max + 1e-9
    assert info["slot_length"] >= s_max                 # slot long enough
    assert min(travel) >= -1e-9                         # open = slot end stop


def test_chair_sweep_clean():
    m, info = samples.chaise_pliante()
    for st in solver.sweep(m, 41):
        assert not collide.interferences(m, st["x"]), st["t"]
        assert not collide.limit_violations(m, st["x"]), st["t"]


def test_chair_flush_backrest_board_is_caught():
    # board right behind the upright: the seat's rear edge hits it mid-fold
    m, info = samples.chaise_pliante(recul_dossier=0.02)
    hits = [st["t"] for st in solver.sweep(m, 41)
            if ("pied_dossier", "assise") in collide.interferences(m, st["x"])]
    assert hits and 0.0 < min(hits) and max(hits) < 1.0


def test_chair_short_slot_reports_limit():
    m, info = samples.chaise_pliante(longueur_lumiere=0.30)
    bad = [st["t"] for st in solver.sweep(m, 41)
           if collide.limit_violations(m, st["x"])]
    assert bad and all(b > 0 for b in bad)


def test_chair_other_dimensions():
    m, info = samples.chaise_pliante(hauteur=0.47, profondeur=0.42,
                                     largeur=0.45)
    seat = info["bodies"]["assise"].boxes[0]
    assert abs(seat.center[2] + seat.half[2] - 0.47) < 1e-12
    assert abs(2 * seat.half[0] - 0.42) < 1e-12
    assert abs(2 * seat.half[1] - 0.45) < 1e-12
    body = info["bodies"]["assise"]
    for st in solver.sweep(m, 21):
        assert st["ok"]
        ref = samples.chaise_reference(info, st["value"])
        assert np.linalg.norm(_pt(m, body, st["x"], info["K"]) - ref["Q"]) < TOL


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
