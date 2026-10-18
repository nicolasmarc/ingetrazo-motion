"""Roller chain with a hanging slack strand (motion.chain, samples.chaine),
checked against closed forms and an independent catenary solution."""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from motion import samples, solver, collide
from motion.model import Mechanism
from motion import chain as C

P8 = 0.0127                                      # 08B, 1/2 in


def _pair(z1, z2, centre, links, h=0.25):
    m = Mechanism()
    g = m.ground
    s1 = m.body("menant", (0, 0, h))
    s2 = m.body("mené", (centre, 0, h))
    j1 = m.pivot(g, s1, (0, 0, h), C.AXIS)
    j2 = m.pivot(g, s2, (centre, 0, h), C.AXIS)
    m.drive(j1, 0.0, 2 * math.pi)
    ch = C.Chain(m, j1, s1, (0, 0, h), z1, j2, s2, (centre, 0, h), z2, P8, links)
    return m, ch


def test_pitch_radius():
    # 17 teeth, 1/2 in: catalogue pitch diameter 69.12 mm
    assert abs(2 * C.pitch_radius(P8, 17) - 0.06912) < 2e-5
    assert abs(2 * C.pitch_radius(P8, 25) - 0.10134) < 2e-5


def test_every_link_is_one_pitch():
    m, ch = _pair(17, 25, 0.42, 88)
    for th in (0.0, 0.37, 2.0, 5.1):
        g = ch.geometry(th, ch._centres(m, np.zeros(m.n)))
        P = np.array(g["pins"])
        d = np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1) / P8
        assert len(P) == 88
        assert d.min() > 0.998 and d.max() < 1.002, (d.min(), d.max())


def test_slack_strand_is_tangent_and_hangs():
    m, ch = _pair(17, 25, 0.42, 88)
    g = ch.ref
    c, u0, v0 = g["catenary"]
    for a, B in ((g["aB1"], g["B1"]), (g["aB2"], g["B2"])):
        slope = math.sinh((B[0] - u0) / c)
        travel = np.array([-math.sin(a), math.cos(a)])          # along the circle
        assert abs(math.atan(slope) - math.atan2(travel[1], travel[0])) < 1e-9
    assert g["B1"][1] > v0 + c - 1e-12 and g["sag"] > 0.03      # hangs below


def _independent_sag(z, D, links):
    """Equal sprockets at one height: symmetric catenary v = c cosh(x/c)
    centred between them, tangent to both pitch circles, total chain
    length = links. Bisection on the departure angle — a different set of
    unknowns from motion.chain."""
    r = C.pitch_radius(P8, z)
    n_top = D / P8

    def total(a):                       # a: departure angle on the left circle
        xB = -D / 2 + r * math.cos(a)
        slope = math.cos(a) / -math.sin(a)
        c = xB / math.asinh(slope)
        wrap = ((a - math.pi / 2) % (2 * math.pi)) * z / (2 * math.pi)
        return n_top + 2 * wrap + 2 * c * math.sinh(-xB / c) / P8, c, xB

    lo, hi = -math.pi / 2 - 1.2, -math.pi / 2 - 1e-6
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if total(mid)[0] > links:      # departs too early: too much chain
            lo = mid
        else:
            hi = mid
    _, c, xB = total(0.5 * (lo + hi))
    return c * (math.cosh(xB / c) - 1)


def test_sag_matches_an_independent_catenary():
    for D, N in ((0.40, 84), (0.40, 85), (0.60, 116)):
        m, ch = _pair(20, 20, D, N)
        ref = _independent_sag(20, D, N)
        assert abs(ch.ref["sag"] - ref) < 2e-5, (D, N, ch.ref["sag"], ref)


def test_too_short_is_taut():
    m, ch = _pair(17, 25, 0.43, 88)
    assert ch.ref["taut"] and "tendue" in ch.measures()[0]


def test_pins_stay_in_the_tooth_gaps():
    m, info = samples.chaine(tension=0.0)
    ch = info["chain"]
    F = solver.sweep(m, 31)

    def seats(body, centre, z, x, arc):
        q = m.pose(x, body)
        R = body.rot(q)
        out = []
        for p in ch.state["pins"]:
            w = np.array([p[0], 0.0, p[1]])
            if abs(np.linalg.norm(w - body.world_point(q, centre)) - arc) < 1e-9:
                loc = R.T @ (w - body.world_point(q, centre))
                a = math.atan2(loc[2], loc[0]) * z / (2 * math.pi)
                out.append(a - round(a))
        return out
    for f in F[::5]:
        ch.place(m, f["x"])
        d1 = seats(info["chain"].b1, ch.c1, info["z1"], f["x"], ch.r1)
        d2 = seats(info["chain"].b2, ch.c2, info["z2"], f["x"], ch.r2)
        assert d1 and d2
        assert np.ptp(d1) < 1e-6 and np.ptp(d2) < 1e-6       # one seat phase each
    # and the driven sprocket turns z1/z2 of the driving one
    x = F[-1]["x"]
    assert abs(m.continuous_value(info["j2"], x)
               - m.continuous_value(info["j1"], x) * info["z1"] / info["z2"]) < 1e-9


def test_example_tensioner_takes_up_the_slack():
    m, info = samples.chaine()
    rep = collide.validate(m, 21, info["jeu_min"])
    assert rep["ok"], rep["problems"][:5]
    ch = info["chain"]
    sags = []
    for f in rep["steps"]:
        ch.place(m, f["x"])
        sags.append(ch.state["sag"])
    assert all(b < a for a, b in zip(sags, sags[1:]))
    assert 0.09 < sags[0] / 0.420 < 0.11                      # about 10 % loose
    assert 0.01 < sags[-1] / ch.state["span"] < 0.04
    assert solver.dof(m, rep["steps"][5]["x"]) == 2              # two motors


# -- dynamics -------------------------------------------------------------------

def _run(m, ch, theta_of, seconds, fps=30, kick=None):
    """Advance the chain in time; returns per frame (max pin distance from
    the chain at rest, longest and shortest link / pitch)."""
    x = solver.solve(m, 0.0)[0]
    ch.dynamic = True
    ch.advance(m, x, 0.0)
    if kick is not None:
        kick(ch)
    out = []
    for i in range(1, int(seconds * fps) + 1):
        xx = x.copy()
        b = ch.b1.index
        xx[6 * b + 3: 6 * b + 6] = ch.j1.axis * theta_of(i / fps)
        xx = ch.place(m, xx)
        rest = np.array(ch.state["pins"])
        ch.advance(m, xx, 1.0 / fps)
        P = np.array(ch.state["pins"])
        d = np.linalg.norm(np.roll(P, -1, axis=0) - P, axis=1) / ch.p
        out.append((float(np.linalg.norm(P - rest, axis=1).max()), d.max(), d.min()))
    return np.array(out)


def test_a_chain_running_steadily_keeps_its_rest_shape():
    # a chain moving along its own catenary needs no sideways force: at a
    # steady speed the dynamic chain sits on the shape at rest
    m, info = samples.chaine(tension=0.0)
    ch = info["chain"]
    w = 2 * math.pi * 1.2                                     # 1.2 turn/s
    # speed ramps up over the first second (angle w (t^2 / 2), then w (t - 1/2))
    r = _run(m, ch, lambda t: w * (0.5 * t * t if t < 1 else t - 0.5), 6.0)
    assert r[-60:, 0].max() < 0.002, r[-60:, 0].max()         # within 2 mm
    assert r[:, 1].max() < 1.005 and r[:, 2].min() > 0.995     # links one pitch


def test_a_disturbed_strand_swings_then_settles():
    m, info = samples.chaine(tension=0.0)
    ch = info["chain"]
    ch.damping = 0.8

    def kick(c):                                               # a tap on the strand
        free = ~c._kinematic(c._dyn["g"])
        c._dyn["V"][free, 0] += 0.8                            # sideways
    r = _run(m, ch, lambda t: 0.0, 8.0, kick=kick)
    assert r[:15, 0].max() > 0.01                              # it moved (> 10 mm)
    assert r[-15:, 0].max() < 0.25 * r[:15, 0].max()           # and calmed down
    assert r[:, 1].max() < 1.005 and r[:, 2].min() > 0.995


def test_supple_example_is_not_quasi_static():
    m, info = samples.chaine_souple()
    ch = info["chain"]
    assert solver.dof(m) == 1 and collide.validate(m, 31, info["jeu_min"])["ok"]
    al = info["allure"]
    assert al(0.0) == 0.0 and abs(al(1.0) - 1.0) < 1e-12 and al(0.95) == 1.0   # stops, settles
    n, fps = 14 * 30, 30
    x, dev = None, []
    j, lo, hi = m.driver
    for i in range(n):
        x, ok, _ = solver.solve(m, lo + al(i / (n - 1)) * (hi - lo), x)
        rest = np.array(ch.state["pins"])
        x = ch.advance(m, x, 0.0 if i == 0 else 1.0 / fps)
        dev.append(float(np.linalg.norm(np.array(ch.state["pins"]) - rest, axis=1).max()))
    assert max(dev) > 0.02                                     # lags and swings: > 20 mm


def test_sway_frequency_matches_string_theory():
    # a shallow hanging strand swings like a string: its first antisymmetric
    # mode is f = sqrt(H / mu) / L = sqrt(g c) / L (H = mu g c, c the
    # catenary parameter, L the span) — the sag does not change that mode
    m, info = samples.chaine(tension=0.0, entraxe=0.4245)      # sag 14 mm, 3.4 %
    ch = info["chain"]
    ch.damping = 0.0
    x = solver.solve(m, 0.0)[0]
    ch.dynamic = True
    ch.advance(m, x, 0.0)
    g = ch._dyn["g"]
    free = [i for i in np.argsort(g["u"]) if not ch._kinematic(g)[i]]
    for k, i in enumerate(free):                               # antisymmetric push
        ch._dyn["V"][i, 1] += 0.02 * math.sin(2 * math.pi * (k + 1) / (len(free) + 1))
    q, fps, zs = free[len(free) // 4], 400, []
    for _ in range(4 * fps):
        ch.advance(m, x, 1.0 / fps)
        zs.append(ch.state["pins"][q][1])
    zs = np.array(zs) - np.mean(zs)
    F = np.abs(np.fft.rfft(zs * np.hanning(len(zs))))
    k = int(np.argmax(F[3:])) + 3
    a, b, c = F[k - 1], F[k], F[k + 1]
    f_sim = (k + 0.5 * (a - c) / (a - 2 * b + c)) * fps / len(zs)
    f_th = math.sqrt(9.81 * g["catenary"][0]) / g["chord"]
    assert abs(f_sim / f_th - 1) < 0.03, (f_sim, f_th)


if __name__ == "__main__":
    for k, fn in list(globals().items()):
        if k.startswith("test_"):
            fn(); print("ok", k)
