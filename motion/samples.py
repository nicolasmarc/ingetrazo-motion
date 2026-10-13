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
    q0 = Fd + 0.12 * (K - Fd) + nf * (w / 2 + 0.015)
    g.boxes.append(Box(q0, (0.015, y_do + ty / 2, 0.015), name="entretoise avant"))

    pa = m.body("pied_assise", center=(Fa + S0) / 2)
    for sgn in (-1, 1):
        y = sgn * y_as * Y
        pa.boxes.append(bar(Fa + y, S0 + y + 0.02 * unit(S0 - Fa), w, ty,
                            name="pied"))
    nr = unit(np.cross(Y, unit(S0 - Fa)))
    nr = nr if nr[0] < 0 else -nr                 # rear face
    q1 = Fa + 0.12 * (S0 - Fa) + nr * (w / 2 + 0.015)
    pa.boxes.append(Box(q1, (0.015, y_as - ty / 2, 0.015), name="entretoise arriere"))

    seat = m.body("assise", center=_p(D / 2, zs),
                  boxes=[Box(_p(D / 2, zs), (D / 2, W / 2, epaisseur_assise / 2),
                             name="assise")])
    pin = m.body("coulisseau", center=K,
                 boxes=[Box(K + (W / 2 + 0.01) * Y, (0.006, 0.01, 0.006), name="axe"),
                        Box(K - (W / 2 + 0.01) * Y, (0.006, 0.01, 0.006), name="axe")])

    cross = m.pivot(g, pa, P, CCW, name="croisement")
    front = m.pivot(pa, seat, S0, CCW, name="avant_assise")
    rear = m.pivot(seat, pin, K, CCW, name="axe_arriere")
    slot = m.slider(g, pin, K, u, limits=(0.0, longueur_lumiere), name="lumiere")
    phi_fold = _angle(S0 - Fa, K - Fd)
    cross.limits = (0.0, phi_fold)
    m.drive(cross, 0.0, phi_fold)
    info = dict(P=P, K=K, S0=S0, Fd=Fd, Fa=Fa, T=T, u=u, L=L, r=r,
                phi_fold=phi_fold, recul_dossier=recul_dossier, slot_length=longueur_lumiere,
                seat_thickness=epaisseur_assise, member_width=w,
                joints=dict(croisement=cross, avant_assise=front,
                            axe_arriere=rear, lumiere=slot),
                bodies=dict(pied_dossier=g, pied_assise=pa, assise=seat,
                            coulisseau=pin))
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
