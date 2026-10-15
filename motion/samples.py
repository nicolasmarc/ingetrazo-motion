"""Ready-made mechanisms, built from a few dimensions (metres).

Each builder returns ``(mechanism, info)``: ``info`` holds the key points
and joints, so tests and the panel can check the result against
closed-form geometry.

Plane convention for the planar examples: x = forward, z = up, the joint
axes run along y (across the piece of furniture).
"""
import math
import numpy as np
from .model import Mechanism, Box, bar, unit

Y = np.array([0, 1.0, 0])
CCW = (0, -1.0, 0)        # pivot axis: positive angle turns x towards z


def _p(x, z, y=0.0):
    return np.array([x, y, z], float)


def chaise_pliante(hauteur=0.45, profondeur=0.40, largeur=0.40,
                   hauteur_dossier=0.40, inclinaison_dossier=11.0,
                   epaisseur_assise=0.02, retrait_axes=0.03, debord_pieds=0.02,
                   section=(0.04, 0.02), jeu_lateral=0.005, recul_dossier=0.04,
                   jeu_entretoise=0.005,
                   longueur_lumiere=None):
    """Folding chair with crossed legs (planar scissor).

    - ``pied_dossier`` (the reference, held still): one rigid part per
      side = leg + backrest upright. It stands on the floor at the FRONT,
      rises backwards to the rear of the seat, then carries the backrest.
    - ``pied_assise``: stands on the floor at the rear, rises forwards,
      crosses ``pied_dossier`` (pivot ``croisement``) and carries the front
      of the seat (pivot ``avant_assise``).
    - ``assise``: articulated at the front, its rear pin slides in a slot
      (``lumiere``) of the backrest upright, through a small ``coulisseau``
      (pin = pivot ``axe_arriere`` + slider ``lumiere``).

    ``jeu_entretoise``: gap between a stretcher (offset from its leg's
    face) and the other leg once folded flat.
    ``recul_dossier``: from the upright axis to the front face of the
    backrest board (the seat's rear edge swings behind the upright while
    folding). ``hauteur`` is the top of the seat. Driver = opening angle at the
    crossing: 0 % = open (as drawn), 100 % = folded flat.
    """
    H, D, W = hauteur, profondeur, largeur
    w, ty = section                               # member width, thickness
    zs = H - epaisseur_assise / 2                 # joint axes: seat mid-plane
    K = _p(retrait_axes, zs)                      # rear pin, open position
    S0 = _p(D - retrait_axes, zs)                 # front seat pivot
    Fd = _p(D + debord_pieds, 0)                  # foot of pied_dossier
    Fa = _p(-debord_pieds, 0)                     # foot of pied_assise
    a = math.radians(inclinaison_dossier)
    u = np.array([-math.sin(a), 0, math.cos(a)])  # backrest / slot direction
    T = K + hauteur_dossier * u                   # top of backrest
    # crossing = intersection of the two leg axes (in the xz plane)
    d1, d2 = K - Fd, S0 - Fa
    A = np.array([[d1[0], -d2[0]], [d1[2], -d2[2]]])
    t = np.linalg.solve(A, (Fa - Fd)[[0, 2]])
    P = Fd + t[0] * d1

    y_as = W / 2 + jeu_lateral + ty / 2           # pied_assise, inside
    y_do = y_as + ty + jeu_lateral                # pied_dossier, outside
    L = float(np.linalg.norm(S0 - K))             # pin-to-pivot on the seat
    r = float(np.linalg.norm(S0 - P))
    if longueur_lumiere is None:                  # slot just long enough
        longueur_lumiere = _s_max(K, u, P, r, L) + 0.01

    m = Mechanism()
    g = m.ground
    g.name = "pied_dossier"
    nback = np.array([-math.cos(a), 0, -math.sin(a)])   # behind the backrest
    for sgn in (-1, 1):
        y = sgn * y_do * Y
        g.boxes.append(bar(Fd + y, K + y, w, ty, name="montant bas"))
        g.boxes.append(bar(K + y, T + y, w, ty, name="montant dossier"))
    # backrest board, behind the uprights, upper part of the backrest
    c0 = K + 0.55 * hauteur_dossier * u + nback * (recul_dossier + 0.0075)
    c1 = K + 0.95 * hauteur_dossier * u + nback * (recul_dossier + 0.0075)
    g.boxes.append(bar(c0, c1, 0.015, 2 * y_do + ty, name="dossier"))
    # front stretcher, in front of the leg axis, near the floor
    nf = unit(np.cross(Y, unit(K - Fd)))          # in-plane normal
    nf = nf if nf[0] > 0 else -nf
    q0 = Fd + 0.12 * (K - Fd) + nf * (w / 2 + jeu_entretoise + 0.015)
    g.boxes.append(bar(q0 - (y_do + ty / 2) * Y, q0 + (y_do + ty / 2) * Y,
                       0.03, 0.03, side=K - Fd, name="entretoise avant"))

    pa = m.body("pied_assise", center=(Fa + S0) / 2)
    for sgn in (-1, 1):
        y = sgn * y_as * Y
        pa.boxes.append(bar(Fa + y, S0 + y + 0.02 * unit(S0 - Fa), w, ty,
                            name="pied"))
    nr = unit(np.cross(Y, unit(S0 - Fa)))
    nr = nr if nr[0] < 0 else -nr                 # rear face
    q1 = Fa + 0.12 * (S0 - Fa) + nr * (w / 2 + jeu_entretoise + 0.015)
    pa.boxes.append(bar(q1 - (y_as - ty / 2) * Y, q1 + (y_as - ty / 2) * Y,
                        0.03, 0.03, side=S0 - Fa, name="entretoise arrière"))

    seat = m.body("assise", center=_p(D / 2, zs),
                  boxes=[Box(_p(D / 2, zs), (D / 2, W / 2, epaisseur_assise / 2),
                             name="assise")])
    pin = m.body("coulisseau", center=K,
                 boxes=[Box(K + sgn * (W / 2 + y_do + ty / 2) / 2 * Y,
                            (0.006, (y_do + ty / 2 - W / 2) / 2, 0.006),
                            name="axe arrière") for sgn in (-1, 1)])

    cross = m.pivot(g, pa, P, CCW, name="croisement")
    front = m.pivot(pa, seat, S0, CCW, name="avant_assise")
    rear = m.pivot(seat, pin, K, CCW, name="axe_arriere")
    slot = m.slider(g, pin, K, u, limits=(0.0, longueur_lumiere), name="lumiere")
    phi_fold = _angle(S0 - Fa, K - Fd)
    cross.limits = (0.0, phi_fold)
    m.drive(cross, 0.0, phi_fold)
    g.color, pa.color = (150, 100, 60), (196, 140, 84)
    seat.color, pin.color = (226, 190, 140), (90, 95, 105)
    info = dict(P=P, K=K, S0=S0, Fd=Fd, Fa=Fa, T=T, u=u, L=L, r=r,
                phi_fold=phi_fold, recul_dossier=recul_dossier, slot_length=longueur_lumiere,
                seat_thickness=epaisseur_assise, member_width=w,
                joints=dict(croisement=cross, avant_assise=front,
                            axe_arriere=rear, lumiere=slot),
                bodies=dict(pied_dossier=g, pied_assise=pa, assise=seat,
                            coulisseau=pin))
    info["mesures"] = lambda mech, x: [
        f"course dans la lumière {1000 * mech.joint_value(slot, x):.1f} mm"]
    return m, info


def _angle(v_from, v_to):
    """Signed angle (xz plane, positive x -> z) turning ``v_from`` onto ``v_to``."""
    a0 = math.atan2(v_from[2], v_from[0])
    a1 = math.atan2(v_to[2], v_to[0])
    return (a1 - a0 + math.pi) % (2 * math.pi) - math.pi


def _s_max(K, u, P, r, L):
    """Largest slot travel: pin furthest along ``u`` when pivot, front axis
    and pin are aligned, |K + s u - P| = r + L (larger root)."""
    d = K - P
    b = u @ d
    return float(-b + math.sqrt(b * b - d @ d + (r + L) ** 2))


def chaise_reference(info, phi):
    """Closed-form position at opening angle ``phi`` (radians):
    front pivot S, rear pin Q, slot travel s and seat direction angle."""
    P, K, S0, u, L = info["P"], info["K"], info["S0"], info["u"], info["L"]
    c, s_ = math.cos(phi), math.sin(phi)
    v = S0 - P
    S = P + np.array([c * v[0] - s_ * v[2], 0, s_ * v[0] + c * v[2]])
    d = S - K
    b = u @ d
    s = b + math.sqrt(max(b * b - d @ d + L * L, 0.0))
    Q = K + s * u
    return dict(S=S, Q=Q, s=s, seat_angle=math.atan2(Q[2] - S[2], Q[0] - S[0]))


def rail_porte(hauteur, rayon, longueur_horizontale, y=0.0):
    """Track of a sectional door (centre line of the rollers): vertical
    from the floor to ``hauteur``, quarter circle of ``rayon`` turning
    into the garage (+x), then horizontal."""
    from .model import Path
    zt = hauteur
    return (Path(_p(0, 0, y)).line_to(_p(0, zt, y))
            .arc(_p(rayon, zt, y), math.pi / 2, (0, 1.0, 0))
            .line_to(_p(rayon + longueur_horizontale, zt + rayon, y)))


def porte_sectionnelle(hauteur=2.0, largeur=2.4, panneaux=4, rayon=0.38,
                       epaisseur=0.04, jeu_linteau=0.01, retombee=0.10,
                       longueur_horizontale=None):
    """Sectional garage door: ``panneaux`` equal panels hinged together,
    one roller per side at every joint plus the bottom and top edges,
    all running in two identical tracks (``rail_porte``) at y = ±largeur/2.

    The panel's inner face is on the roller line (x = 0), the outer face at
    x = -epaisseur; the lintel (fixed) starts ``jeu_linteau`` outside it and
    comes down ``retombee`` below the top of the opening.
    Driver = position of the bottom roller along the track: 0 % = closed,
    100 % = bottom roller at the end of the curve (door horizontal).
    """
    n, h = int(panneaux), hauteur / panneaux
    if longueur_horizontale is None:
        longueur_horizontale = hauteur + 0.3
    m = Mechanism()
    g = m.ground
    g.name = "bâti"
    zt = hauteur
    g.boxes.append(Box(_p(-epaisseur - jeu_linteau - 0.1, zt - retombee + 0.2),
                       (0.1, largeur / 2 + 0.2, 0.2), name="linteau"))
    rails = {sgn: rail_porte(hauteur, rayon, longueur_horizontale, sgn * largeur / 2)
             for sgn in (-1, 1)}
    panels, hinges, rollers = [], [], []
    for k in range(n):
        z0 = k * h
        c = _p(-epaisseur / 2, z0 + h / 2)
        panels.append(m.body(f"panneau{k + 1}", center=c,
                             boxes=[Box(c, (epaisseur / 2, largeur / 2, h / 2),
                                        name=f"panneau{k + 1}")]))
    for k in range(1, n):
        hinges.append(m.pivot(panels[k - 1], panels[k], _p(0, k * h), (0, 1.0, 0),
                              name=f"charnière{k}"))
    for lev in range(n + 1):
        body = panels[min(lev, n - 1)]
        for sgn in (1, -1):
            j = m.guide(g, body, _p(0, lev * h, sgn * largeur / 2), rails[sgn],
                        name=f"galet{lev}{'D' if sgn > 0 else 'G'}")
            if sgn > 0:
                rollers.append(j)
    s_open = hauteur + math.pi / 2 * rayon
    m.drive(rollers[0], 0.0, s_open)
    g.color = (150, 150, 150)
    for b in panels:
        b.color = (226, 226, 220)
    info = dict(rail=rails[1], rollers=rollers, panels=panels, hinges=hinges,
                h=h, n=n, R=rayon, H=hauteur, s_open=s_open,
                largeur=largeur, jeu_linteau=jeu_linteau)
    return m, info


def porte_reference(info, s0):
    """Independent positions of the rollers (one side) for the bottom
    roller at arc length ``s0``: each next roller is the point further
    along the track at chord distance h (bisection, convex track)."""
    rail, h = info["rail"], info["h"]
    s = [float(s0)]
    for _ in range(info["n"]):
        a = s[-1]
        pa = rail.point(a)
        lo, hi = a, a + h * math.pi / 2 + 1e-9
        hi = min(hi, rail.length)
        for _ in range(200):
            mid = (lo + hi) / 2
            if np.linalg.norm(rail.point(mid) - pa) < h:
                lo = mid
            else:
                hi = mid
        s.append((lo + hi) / 2)
    return s, [rail.point(v) for v in s]


# -- flapping bird ------------------------------------------------------------

def _circle_cut(c0, r0, c1, r1, side):
    """Intersection of circles (xz plane) on ``side`` (+1/-1) of c0->c1."""
    d = c1 - c0
    L = math.hypot(d[0], d[2])
    a = (r0 * r0 - r1 * r1 + L * L) / (2 * L)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    u = d / L
    n = np.array([-u[2], 0.0, u[0]])
    return c0 + a * u + side * h * n


def _side(c0, c1, p):
    d, v = c1 - c0, p - c0
    return 1.0 if d[0] * v[2] - d[2] * v[0] >= 0 else -1.0


def oiseau(manivelle=0.03, bielle=0.09, ecart_biellettes=0.018,
           epaule=(0.07, 0.135), levier=0.052, bras=0.20, main=0.24,
           tirant_sous_epaule=0.02, levier_main=0.04, angle_levier_main=120.0,
           corde=0.16):
    """Flapping bird, both wings driven by one crank (metres).

    The crank (axis y, turning a full revolution) drives a vertical slider
    through a connecting rod; two short rods take the slider to the lever
    of each wing (``bras``, hinged at the shoulder). Each wing has a second
    segment (``main``, hinged at the elbow) held by a tie rod (``tirant``)
    from a point of the body just under the shoulder: the hand folds and
    spreads as the arm beats, amplifying the stroke (about x1.5).

    x = across (wings), z = up, y = fore-aft (+y = head). Driver = crank
    angle, 0 -> 360 deg (0 % = as drawn, wings level).
    """
    r, Lc, dq = manivelle, bielle, ecart_biellettes
    zQ = math.sqrt(Lc * Lc - r * r)
    O = _p(0, 0)
    P0 = _p(r, 0)                                  # crank pin, angle 0
    Q0 = _p(0, zQ)                                 # slider pin
    a_lm = math.radians(angle_levier_main)
    m = Mechanism()
    g = m.ground
    g.name = "corps"
    T = 0.006                                      # plate thickness (y)

    def slab(p0, p1, w, y, name):
        return bar(np.asarray(p0, float) + y * Y, np.asarray(p1, float) + y * Y, w, T,
                   name=name)

    # body: back plate, slider guide, head, tail, tie-rod clevises
    g.boxes += [Box(_p(0, 0.06, 0.034), (0.085, T / 2, 0.11), name="flasque"),
                slab(_p(0, 0.035), _p(0, 0.17), 0.012, 0.024, "glissière"),
                Box(_p(0, 0.17, 0.20), (0.03, 0.05, 0.03), name="tête"),
                Box(_p(0, 0.20, 0.235), (0.006, 0.02, 0.006), name="bec"),
                bar(_p(0, 0.13, 0.034), _p(0, 0.17, 0.15), 0.035, 0.035, side=(1, 0, 0),
                    name="cou"),
                Box(_p(0, 0.10, -0.16), (0.07, 0.11, 0.004), name="queue")]
    crank = m.body("manivelle", O, [slab(O, P0 + 0.012 * unit(P0 - O), 0.03, 0.0, "manivelle")])
    rod = m.body("bielle", (P0 + Q0) / 2, [slab(P0, Q0, 0.012, -0.008, "bielle")])
    slider = m.body("coulisseau", Q0, [Box(Q0 + 0.012 * Y, (0.03, 0.004, 0.012),
                                           name="coulisseau")])
    m.drive(m.pivot(g, crank, O, CCW, name="manivelle"), 0.0, 2 * math.pi)
    g.color = (200, 46, 46)
    crank.color = slider.color = (52, 92, 178)
    rod.color = (60, 60, 66)
    m.pivot(crank, rod, P0, Y, name="maneton")
    m.pivot(rod, slider, Q0, Y, name="pied de bielle")
    m.slider(g, slider, Q0, (0, 0, 1.0), name="coulisse")

    wings = {}
    for sgn, tag in ((1, "D"), (-1, "G")):
        S = _p(sgn * epaule[0], epaule[1])
        A = S + _p(-sgn * levier, 0)                         # lever, inward
        Qs = Q0 + _p(sgn * dq, 0)
        E = S + _p(sgn * bras, 0)
        tip = E + _p(sgn * main, 0)
        C = E + levier_main * _p(sgn * math.cos(a_lm), math.sin(a_lm))
        B = S + _p(0, -tirant_sous_epaule)
        # rod: slider -> arm lever
        bl = m.body(f"biellette{tag}", (Qs + A) / 2, [slab(Qs, A, 0.01, -0.016, "biellette")])
        ribs_a = [(k * bras / 4, corde * (0.80 + 0.06 * k)) for k in (1, 2, 3, 4)]
        ribs_h = [(k * main / 5, corde * (1.10 - 0.17 * k)) for k in (1, 2, 3, 4, 5)]
        arm = m.body(f"bras{tag}", (S + E) / 2,
                     [slab(A, S, 0.014, -0.024, "levier"),
                      bar(S - 0.01 * Y, E - 0.01 * Y, 0.012, 0.012, name="longeron")]
                     + _lattice(S, sgn, ribs_a, 0.006, "nervure"))
        hand = m.body(f"main{tag}", (E + tip) / 2,
                      [slab(E, C, 0.012, -0.040, "levier main"),
                       bar(E - 0.01 * Y, tip - 0.01 * Y, 0.01, 0.01, name="longeron")]
                      + _lattice(E, sgn, ribs_h, 0.005, "plume"))
        arm.color = hand.color = (240, 240, 236)
        tie = m.body(f"tirant{tag}", (B + C) / 2, [slab(B, C, 0.008, -0.048, "tirant")])
        bl.color = tie.color = (60, 60, 66)
        g.boxes.append(Box(B - 0.054 * Y, (0.008, 0.005, 0.008), name=f"chape{tag}"))
        g.boxes.append(Box(S + 0.02 * Y, (0.012, 0.008, 0.012), name=f"palier{tag}"))
        m.pivot(slider, bl, Qs, Y, name=f"tête biellette{tag}")
        m.pivot(bl, arm, A, Y, name=f"levier{tag}")
        m.pivot(g, arm, S, Y, name=f"épaule{tag}")
        m.pivot(arm, hand, E, Y, name=f"coude{tag}")
        m.pivot(hand, tie, C, Y, name=f"poignet{tag}")
        m.pivot(tie, g, B, Y, name=f"attache tirant{tag}")
        wings[tag] = dict(S=S, A=A, Qs=Qs, E=E, tip=tip, C=C, B=B, arm=arm, hand=hand,
                          sgn=sgn, Lr=float(np.linalg.norm(A - Qs)),
                          Lt=float(np.linalg.norm(C - B)),
                          sideA=_side(S, Qs, A), sideC=_side(E, B, C))
    info = dict(r=r, Lc=Lc, dq=dq, Q0=Q0, P0=P0, wings=wings, levier=levier,
                bras=bras, main=main, levier_main=levier_main, a_lm=a_lm,
                bodies=dict(manivelle=crank, bielle=rod, coulisseau=slider),
                traces=[(wings[t]["hand"], wings[t]["tip"]) for t in ("D", "G")])

    def mesures(mech, x):
        w = wings["D"]
        E = w["arm"].world_point(mech.pose(x, w["arm"]), w["E"])
        T_ = w["hand"].world_point(mech.pose(x, w["hand"]), w["tip"])
        a = math.degrees(math.atan2(E[2] - w["S"][2], E[0] - w["S"][0]))
        h = math.degrees(math.atan2(T_[2] - E[2], T_[0] - E[0]))
        return [f"bras {a:+.1f}°", f"main {h:+.1f}°", f"pliage {h - a:+.1f}°"]

    info["mesures"] = mesures
    info["vue"] = (90.0, 8.0)          # face on: the plane of the beat
    info["jeu_min"] = 0.0015           # links stacked 2 mm apart (washers)
    return m, info


def _lattice(root, sgn, ribs, t, name):
    """Wing lattice behind a spar starting at ``root``: ribs (x along the
    spar, chord) running back along +y, and a trailing edge joining their
    ends."""
    out, ends = [], []
    for k, (d, chord) in enumerate(ribs):
        p0 = root + _p(sgn * d, 0) + 0.004 * Y
        p1 = p0 + chord * Y
        out.append(bar(p0, p1, t, t, side=(1, 0, 0), name=name))
        ends.append(p1)
    first = root + _p(sgn * 0.03, 0) + (0.004 + 0.6 * ribs[0][1]) * Y
    for a, b in zip([first] + ends[:-1], ends):
        out.append(bar(a, b, t, t, side=(0, 0, 1.0), name="bord de fuite"))
    return out


def oiseau_reference(info, theta):
    """Closed form for crank angle ``theta``: slider height, then for each
    wing the arm angle (circle cut at the lever) and the hand (circle cut
    at the wrist). Angles from +x (right) / -x (left), positive = up."""
    r, Lc, dq = info["r"], info["Lc"], info["dq"]
    zq = r * math.sin(theta) + math.sqrt(Lc * Lc - (r * math.cos(theta)) ** 2)
    out = {"zQ": zq}
    for tag, w in info["wings"].items():
        sgn = w["sgn"]
        Qs = _p(sgn * dq, zq)
        A = _circle_cut(w["S"], info["levier"], Qs, w["Lr"], w["sideA"])
        lever_dir = (A - w["S"]) / info["levier"]
        spar = -lever_dir                                    # lever is opposite the spar
        E = w["S"] + info["bras"] * spar
        C = _circle_cut(E, info["levier_main"], w["B"], w["Lt"], w["sideC"])
        # hand spar: lever direction turned back by the drawn lever angle
        v = (C - E) / info["levier_main"]
        ang = math.atan2(v[2], sgn * v[0]) - info["a_lm"]
        hand = np.array([sgn * math.cos(ang), 0, math.sin(ang)])
        out[tag] = dict(A=A, E=E, C=C, tip=E + info["main"] * hand,
                        arm=math.atan2(spar[2], sgn * spar[0]), hand=ang)
    return out
