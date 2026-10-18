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


# -- hand-pumped rail car (draisine à balancier) ---------------------------------

def _prism(center, radius, y0, y1, n=28, outline=None):
    """Faces (reference pose) of a prism along y: a polygon in the xz plane
    around ``center`` (``outline``: list of (angle, radius), else a regular
    n-gon), from y0 to y1."""
    c = np.asarray(center, float)
    if outline is None:
        outline = [(2 * math.pi * k / n, radius) for k in range(n)]
    ring = [c + np.array([r * math.cos(a), 0, r * math.sin(a)]) for a, r in outline]
    A = [p + np.array([0, y0 - c[1], 0]) for p in ring]
    B = [p + np.array([0, y1 - c[1], 0]) for p in ring]
    faces = [np.array(A), np.array(B[::-1])]
    for k in range(len(ring)):
        k2 = (k + 1) % len(ring)
        faces.append(np.array([A[k], A[k2], B[k2], B[k]]))
    return faces


def _gear_outline(z, module, phase=0.0):
    """Tooth outline (angle, radius): z trapezoid teeth, addendum = module,
    dedendum = 1.25 module; a tooth centred on ``phase``."""
    rp = module * z / 2
    ra, rf = rp + module, rp - 1.25 * module
    p = 2 * math.pi / z
    out = []
    for k in range(z):
        a = phase + k * p
        out += [(a - 0.5 * p, rf), (a - 0.27 * p, rf), (a - 0.14 * p, ra),
                (a + 0.14 * p, ra), (a + 0.27 * p, rf)]
    return out


def _box_faces(box):
    c, R, h = box.center, box.R, box.half
    k = c + (np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                       for sz in (-1, 1)]) * h) @ R.T
    return [k[list(f)] for f in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                                 (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3))]


def draisine(rayon_roue=0.25, empattement=1.10, voie=1.0, hauteur_pivot=1.45,
             bras_bielle=0.43, demi_balancier=0.85, manivelle=0.13,
             dents_roue=40, dents_pignon=20, module=0.008, tours=3,
             longueur_voie=17.0):
    """Hand-pumped rail car (metres). The walking beam (``balancier``) rocks
    on top of the A-frame; a connecting rod takes it to the crank of an
    intermediate shaft whose gear (``dents_roue`` teeth) drives a pinion
    (``dents_pignon``) on the rear axle; both axles roll without slipping
    on a metre-gauge track (``voie``). x = along the track, z = up.

    Driver = crank angle, ``tours`` whole turns (constant speed): each turn
    of the crank is one up-and-down of the beam and moves the car by
    2 pi R x dents_roue / dents_pignon.
    """
    R, gr = rayon_roue, dents_roue / dents_pignon
    Or = _p(-empattement / 2, R)                    # rear axle
    Of = _p(empattement / 2, R)                     # front axle
    dc = module * (dents_roue + dents_pignon) / 2   # centre distance
    a_c = math.radians(60)
    C = Or + _p(dc * math.cos(a_c), dc * math.sin(a_c))   # crank shaft
    P = _p(0, hauteur_pivot)                        # beam pivot
    B = P + _p(-bras_bielle, 0)                     # rod on the beam
    K = C + _p(manivelle, 0)                        # crank pin (angle 0)
    L = float(np.linalg.norm(B - K))
    zdeck = 0.62
    m = Mechanism()
    g = m.ground
    g.name = "voie"
    polys = {}
    # track: rails in 1 m lengths, sleepers every 0.6 m
    x0 = -3.0
    for k in range(int(longueur_voie)):
        for y in (-voie / 2, voie / 2):
            g.boxes.append(Box(_p(x0 + k + 0.5, -0.04, y), (0.5, 0.03, 0.04), name="rail",
                               color=(176, 180, 188)))
    for k in range(int(longueur_voie / 0.6)):
        g.boxes.append(Box(_p(x0 + 0.3 + 0.6 * k, -0.11, 0), (0.09, voie / 2 + 0.25, 0.03),
                           name="traverse", color=(112, 86, 62)))
    g.color = (120, 110, 100)

    car = m.body("châssis", _p(0, zdeck))
    W = voie / 2 + 0.12                            # side beams outside the wheels
    car.boxes += [Box(_p(0, 0.56, s * W), (0.9, 0.04, 0.06), name="longeron")
                  for s in (-1, 1)]
    car.boxes += [Box(_p(xx, 0.56, 0), (0.04, W - 0.04, 0.05), name="traverse châssis")
                  for xx in (-0.85, 0.85)]
    # deck with an opening for the rod
    car.boxes += [Box(_p(0.315, zdeck + 0.02, 0), (0.535, W + 0.02, 0.02), name="plancher"),
                  Box(_p(-0.735, zdeck + 0.02, 0), (0.115, W + 0.02, 0.02), name="plancher"),
                  Box(_p(-0.42, zdeck + 0.02, 0.10 + (W - 0.08) / 2 + 0.01),
                      (0.2, (W - 0.08) / 2, 0.02), name="plancher"),
                  Box(_p(-0.42, zdeck + 0.02, -(0.10 + (W - 0.08) / 2 + 0.01)),
                      (0.2, (W - 0.08) / 2, 0.02), name="plancher")]
    # A-frame: two legs each side, from the deck to the pivot
    for s in (-1, 1):
        for xx in (-0.38, 0.38):
            car.boxes.append(bar(_p(xx, zdeck + 0.04, s * 0.09), P + _p(0, -0.03, s * 0.09),
                                 0.05, 0.04, name="chevalet"))
    # bearings: axle boxes on the side beams, hangers of the crank shaft
    for O in (Or, Of):
        for s in (-1, 1):
            car.boxes.append(Box(O + _p(0, 0.13, s * W), (0.06, 0.12, 0.05), name="palier"))
    for yy in (0.06, 0.24):                       # overhung crank: shaft on one side
        car.boxes.append(bar(C + _p(0, 0, yy), _p(C[0], zdeck, yy), 0.05, 0.03,
                             name="support d'arbre"))
    car.color = (214, 124, 48)

    def axle(name, O, gear=False):
        body = m.body(name, O)
        body.boxes.append(Box(O, (0.035, W + 0.04, 0.035), name="essieu"))
        faces = _box_faces(body.boxes[-1])
        for s in (-1, 1):
            yc = s * voie / 2
            body.boxes.append(Box(O + _p(0, 0, yc), (0.15, 0.04, 0.15), name="roue"))
            faces += _prism(O + _p(0, 0, yc), R, yc - 0.04, yc + 0.04)
            yi = yc - s * 0.04                     # flange on the inside
            faces += _prism(O + _p(0, 0, yi), R + 0.03, min(yi, yi - s * 0.015),
                            max(yi, yi - s * 0.015))
            yo = yc + s * 0.041                    # spokes on the outside face
            for k in range(4):
                a = k * math.pi / 4
                d = _p(math.cos(a), math.sin(a)) * (R - 0.03)
                faces += _box_faces(bar(O + _p(0, 0, yo) - d, O + _p(0, 0, yo) + d,
                                        0.03, 0.006))
        if gear:                                   # pinion, a tooth towards C
            u = C - O
            ph = math.atan2(u[2], u[0])
            body.boxes.append(Box(O + _p(0, 0, 0.10), (0.045, 0.02, 0.045), name="pignon"))
            faces += _prism(O + _p(0, 0, 0.10), 0, 0.08, 0.12,
                            outline=_gear_outline(dents_pignon, module, ph))
        polys[name] = faces
        body.color = (70, 72, 78)
        return body

    rear = axle("essieu arrière", Or, gear=True)
    front = axle("essieu avant", Of)
    shaft = m.body("arbre de manivelle", C,
                   [Box(C + _p(0, 0, 0.145), (0.025, 0.115, 0.025), name="arbre"),
                    bar(C - 0.0 * Y, K, 0.05, 0.03, name="manivelle"),
                    Box(C + _p(0, 0, 0.10), (0.10, 0.02, 0.10), name="roue dentée")])
    u = Or - C
    ph = math.atan2(u[2], u[0]) + math.pi / dents_roue          # a gap towards O
    polys["arbre de manivelle"] = (_box_faces(shaft.boxes[0]) + _box_faces(shaft.boxes[1])
                                   + _prism(C + _p(0, 0, 0.10), 0, 0.08, 0.12,
                                            outline=_gear_outline(dents_roue, module, ph)))
    shaft.color = (150, 152, 160)
    rod = m.body("bielle", (B + K) / 2, [bar(K, B, 0.04, 0.03, name="bielle")])
    rod.color = (90, 92, 100)
    beam = m.body("balancier", P,
                  [bar(P + _p(-demi_balancier, 0), P + _p(demi_balancier, 0), 0.07, 0.06,
                       name="balancier")]
                  + [Box(P + _p(sx * demi_balancier, 0), (0.025, 0.45, 0.025), name="poignée")
                     for sx in (-1, 1)])
    beam.color = (196, 106, 36)

    track = m.slider(g, car, _p(0, zdeck), (1, 0, 0), name="voie")
    pr = m.pivot(car, rear, Or, CCW, name="essieu arrière")
    pf = m.pivot(car, front, Of, CCW, name="essieu avant")
    pc = m.pivot(car, shaft, C, CCW, name="arbre")
    pb = m.pivot(car, beam, P, CCW, name="pivot du balancier")
    m.pivot(shaft, rod, K, CCW, name="maneton")
    m.pivot(rod, beam, B, CCW, name="tête de bielle")
    m.couple(pc, pr, -gr, name="engrenage")
    m.couple(pr, track, -R, name="roulement arrière")
    m.couple(pf, track, -R, name="roulement avant")
    m.drive(pc, 0.0, tours * 2 * math.pi)
    info = dict(R=R, ratio=gr, C=C, P=P, B=B, K=K, L=L, rc=manivelle, db=bras_bielle,
                hb=demi_balancier, Or=Or, Of=Of, sideB=_side(P, K, B),
                joints=dict(voie=track, arriere=pr, avant=pf, arbre=pc, balancier=pb),
                bodies=dict(chassis=car, arriere=rear, avant=front, arbre=shaft,
                            bielle=rod, balancier=beam),
                polys=polys, suivre="châssis", vue=(-62.0, 18.0), jeu_min=0.003, sol=False,
                traces=[(beam, P + _p(demi_balancier, 0)), (beam, P + _p(-demi_balancier, 0))])

    def mesures(mech, x):
        b = math.degrees(mech.continuous_value(pb, x))
        s = mech.continuous_value(track, x)
        return [f"balancier {b:+.1f}°", f"avance {s:.2f} m",
                f"roues {math.degrees(-mech.continuous_value(pr, x)) / 360:.2f} tr"]

    info["mesures"] = mesures
    return m, info


def draisine_reference(info, theta):
    """Closed form for crank angle ``theta`` (car frame): crank pin,
    beam end of the rod (circle cut), beam angle, axle angle (gear) and
    car travel (rolling)."""
    C, P, rc, db, L = info["C"], info["P"], info["rc"], info["db"], info["L"]
    K = C + _p(rc * math.cos(theta), rc * math.sin(theta))
    B = _circle_cut(P, db, K, L, info["sideB"])
    v0 = info["B"] - P
    v = B - P
    beam = math.atan2(v[2], v[0]) - math.atan2(v0[2], v0[0])
    beam = math.atan2(math.sin(beam), math.cos(beam))
    axle = -info["ratio"] * theta
    return dict(K=K, B=B, beam=beam, axle=axle, travel=-info["R"] * axle)


# -- roller chain with a slack strand (chaîne détendue) ----------------------------

def _sprocket_outline(r, z, roller, pitch, seat0):
    """Sprocket outline (angle, radius) around its centre: a seat for a
    roller at every pin position (seat0 + 2 pi k / z), teeth between."""
    rs = 0.53 * roller
    ra = r + 0.32 * pitch
    ds = math.asin(min(rs / r, 0.99))
    half = math.pi / z
    out = []
    for k in range(z):
        a0 = seat0 + 2 * math.pi * k / z
        for t in np.linspace(-half, half, 13)[:-1]:
            d = abs(t)
            if d < ds:
                rr = r * math.cos(d) - math.sqrt(max(rs * rs - (r * math.sin(d)) ** 2, 0.0))
            else:
                f = (d - ds) / (half - ds)
                rr = r * math.cos(ds) + (ra - r * math.cos(ds)) * min(1.0, 1.6 * f)
            out.append((a0 + t, rr))
    return out


def _stadium(P, Q, rad, n=6):
    """(angle, radius) outline around the midpoint of P-Q (x-z) of a link
    plate: two half circles of radius ``rad`` round the pins."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    mid = 0.5 * (P + Q)
    a = math.atan2(Q[1] - P[1], Q[0] - P[0])
    pts = []
    for C, a0 in ((Q, a - math.pi / 2), (P, a + math.pi / 2)):
        for k in range(n + 1):
            t = a0 + math.pi * k / n
            pts.append(C + rad * np.array([math.cos(t), math.sin(t)]))
    return mid, [(math.atan2(p[1] - mid[1], p[0] - mid[0]), float(np.hypot(*(p - mid))))
                 for p in pts]


def chaine(pas=0.0127, dents_menant=17, dents_mene=25, maillons=88, entraxe=0.420,
           tension=0.0047, tours=3, hauteur=0.25, excentrique=0.0, rapport_excentrique=2):
    """Roller chain (pitch ``pas``, ISO 08B by default: 1/2 in) on two
    sprockets, with a tensioner (metres). The driving sprocket (crank
    handle, ``tours`` turns) pulls the upper strand; the lower strand hangs.
    A second motor moves the driven sprocket's bearing by ``tension``: the
    chain goes from loose (about 10 % sag) to well tensioned. x = along the
    chain, z = up, the chain in the plane y = 0.

    ``excentrique`` > 0: no tensioner motor; the bearing is pushed back and
    forth by an eccentric geared to the crank (``rapport_excentrique`` turns
    per crank turn) through a short rod, so the centre distance breathes by
    twice that amount — the slack strand swings (see :func:`chaine_souple`).
    """
    from .chain import Chain, AXIS
    roller, width = 0.00851, 0.00775            # 08B roller and inner width
    S1 = _p(0, hauteur)
    S2 = _p(entraxe, hauteur)
    m = Mechanism()
    g = m.ground
    g.name = "bâti"
    yb = 0.07                                    # frame behind the chain
    g.boxes += [Box(_p(entraxe / 2, 0.01, 0.04), (entraxe / 2 + 0.16, 0.10, 0.01), name="socle"),
                Box(_p(0, (hauteur + 0.04) / 2, yb), (0.025, 0.015, (hauteur - 0.0) / 2),
                    name="montant"),
                Box(_p(entraxe, (hauteur - 0.02) / 2 + 0.01, yb), (0.025, 0.015, (hauteur - 0.06) / 2),
                    name="montant"),
                Box(_p(entraxe + 0.005, hauteur, yb), (0.045, 0.015, 0.03), name="coulisse")]
    g.color = (92, 96, 104)
    polys = {}
    t = 0.0015                                   # plate thickness

    drive = m.body("pignon menant", S1, [Box(S1, (0.045, 0.005, 0.045), name="pignon")])
    drive.boxes += [bar(S1 + _p(0, 0, -0.03), S1 + _p(0.085, 0, -0.03), 0.018, 0.008,
                        name="manivelle"),
                    Box(S1 + _p(0.085, 0, -0.055), (0.009, 0.025, 0.009), name="poignée"),
                    Box(S1 + _p(0, 0, 0.0325), (0.012, 0.0325, 0.012), name="arbre")]
    drive.color = (196, 106, 36)
    car = m.body("tendeur", S2, [Box(S2 + _p(0, 0, 0.05), (0.02, 0.008, 0.02), name="palier")])
    car.color = (120, 124, 132)
    driven = m.body("pignon mené", S2, [Box(S2, (0.06, 0.005, 0.06), name="pignon"),
                                        Box(S2 + _p(0, 0, 0.022), (0.012, 0.022, 0.012), name="arbre")])
    driven.color = (196, 106, 36)

    j1 = m.pivot(g, drive, S1, AXIS, name="arbre menant")
    js = m.slider(g, car, S2, (1, 0, 0), name="tendeur")
    j2 = m.pivot(car, driven, S2, AXIS, name="arbre mené")
    m.drive(j1, 0.0, tours * 2 * math.pi)
    if excentrique > 0:
        E = _p(entraxe + 0.13, hauteur)               # eccentric shaft, behind the bearing
        pin = E + _p(-excentrique, 0)
        Kb = S2 + _p(0.035, 0)                         # rod end on the bearing block
        exc = m.body("excentrique", E, [Box(E + _p(0, 0, 0.05), (0.018, 0.012, 0.018),
                                            name="excentrique")])
        exc.color = (196, 106, 36)
        rod = m.body("biellette", (pin + Kb) / 2, [bar(Kb + _p(0, 0, 0.05), pin + _p(0, 0, 0.05),
                                                       0.012, 0.006, name="biellette")])
        rod.color = (150, 152, 160)
        car.boxes.append(Box(Kb + _p(-0.0175, 0, 0.05), (0.0175, 0.008, 0.008), name="chape"))
        g.boxes.append(Box(E + _p(0, 0, 0.07), (0.02, 0.012, 0.03), name="palier d'excentrique"))
        je = m.pivot(g, exc, E, AXIS, name="excentrique")
        m.pivot(exc, rod, pin, AXIS, name="maneton")
        m.pivot(rod, car, Kb, AXIS, name="chape")
        m.couple(j1, je, rapport_excentrique, name="renvoi")
    else:
        m.add_driver(js, 0.0, tension)
    ch = Chain(m, j1, drive, S1, dents_menant, j2, driven, S2, dents_mene, pas, maillons,
               roller=roller, width=width)

    # drawings: sprockets with a seat at every pin, plates, rollers, pins
    a0 = ch._anchor
    seat1 = a0[1] - a0[0] * 2 * math.pi / dents_menant
    seat2 = a0[2]
    hub = lambda c, r, y0, y1: _prism(c, r, y0, y1, n=20)
    polys["pignon menant"] = (
        _prism(S1, 0, -0.9 * width / 2, 0.9 * width / 2,
               outline=_sprocket_outline(ch.r1, dents_menant, roller, pas, seat1))
        + hub(S1, 0.012, -0.03, 0.065)
        + _box_faces(drive.boxes[1]) + _box_faces(drive.boxes[2]))
    polys["pignon mené"] = (
        _prism(S2, 0, -0.9 * width / 2, 0.9 * width / 2,
               outline=_sprocket_outline(ch.r2, dents_mene, roller, pas, seat2))
        + hub(S2, 0.012, -0.012, 0.044))
    for k, b in enumerate(ch.bodies):
        P, Q = ch.ref["pins"][k], ch.ref["pins"][(k + 1) % maillons]
        mid, out = _stadium(P, Q, 0.55 * roller)
        c3 = _p(mid[0], mid[1])
        yi = width / 2 + (t if k % 2 else 3 * t)          # inner / outer links
        faces = []
        for s in (-1, 1):
            y0 = s * (yi - t)
            faces += _prism(c3, 0, min(y0, s * yi), max(y0, s * yi), outline=out)
        Pw = _p(P[0], P[1])
        faces += _prism(Pw, roller / 2, -width / 2, width / 2, n=10)          # roller
        faces += _prism(Pw, 0.0022, -(width / 2 + 4.2 * t), width / 2 + 4.2 * t, n=6)  # pin
        polys[b.name] = faces
    polys["tendeur"] = [f for bx in car.boxes for f in _box_faces(bx)]

    info = dict(chain=ch, j1=j1, j2=j2, tendeur=js, pas=pas, z1=dents_menant, z2=dents_mene,
                polys=polys, vue=(-28.0, 14.0), jeu_min=0.002, sol=True,
                traces=[(ch.bodies[0], _p(*ch.ref["pins"][0]))])

    def mesures(mech, x):
        ch.place(mech, x)
        out = ch.measures()
        out.append(f"pignon mené {math.degrees(mech.continuous_value(j2, x)) / 360:.2f} tr")
        return out

    info["mesures"] = mesures
    return m, info


def chaine_souple(pas=0.0127, dents_menant=17, dents_mene=25, maillons=88, entraxe=0.420,
                  excentrique=0.0025, rapport_excentrique=3, tours=12, amortissement=0.4):
    """The chain with dynamics: the slack strand has weight and inertia.
    The driven sprocket's bearing is moved by an eccentric (``rapport_excentrique``
    times per crank turn, stroke 2 x ``excentrique``), so the slack length breathes and the strand
    swings, lags and settles; the crank is turned by hand (starts from rest,
    uneven speed, stops) — see ``info["allure"]``. ``amortissement``: 1/s.
    In a video the chain is computed in time; elsewhere (slider, validation)
    it is shown at rest."""
    m, info = chaine(pas, dents_menant, dents_mene, maillons, entraxe, tours=tours,
                     excentrique=excentrique, rapport_excentrique=rapport_excentrique)
    ch = info["chain"]
    ch.dynamic = True
    ch.damping = amortissement

    # hand cranking over the run u in [0, 1]: start from rest, uneven speed
    # (faster on the push, slower on the pull), stop at 85 %, then the
    # chain settles
    n = 4000
    us = np.linspace(0.0, 1.0, n)
    run = 0.85
    speed = np.where(us < run,
                     np.minimum(1.0, us / 0.07) ** 2 * (1 + 0.45 * np.sin(2 * math.pi * 2 * tours * us / run)),
                     0.0)
    speed[(us > run - 0.02) & (us < run)] *= np.linspace(1, 0, ((us > run - 0.02) & (us < run)).sum())
    cum = np.concatenate([[0.0], np.cumsum(0.5 * (speed[1:] + speed[:-1]) * np.diff(us))])
    cum /= cum[-1]
    info["allure"] = lambda u: float(np.interp(u, us, cum))
    info["dynamique"] = True
    info["vue"] = (-24.0, 12.0)
    info["traces"] = []                 # a trace would show the chain at rest

    def mesures(mech, x):
        out = ch.measures()
        out.append(f"dynamique, amortissement {ch.damping:g}/s")
        return out
    info["mesures"] = mesures
    return m, info

