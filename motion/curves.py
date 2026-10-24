"""Curves of the motion (pure numpy, no Qt): what can be plotted for the
item chosen in the list, and its values along the computed steps.

Rows are those Motion already computed: the dynamic simulation's poses
(``value`` = time in s, ``x``, ``ke``/``pe``, ``couples``, ``reac``) or the
kinematic steps (``t`` 0..1 of the motor's travel, ``x``).
"""
import math
import numpy as np

# joints whose first three constraint rows hold a point (or a line) together:
# their multipliers are the force the joint carries, in N
_FORCE_KINDS = ("pivot", "ball", "lock", "cylindrical", "slider", "annular", "screw", "guide")
_ANGLE_KINDS = ("pivot", "cylindrical", "screw")


def quantities(mech, name, dynamic):
    """[(key, label)] for the joint or part ``name`` (None: the whole
    mechanism). Labels are French; the caller translates them."""
    out = []
    j = next((j for j in mech.joints if j.name == name), None)
    b = next((b for b in [mech.ground] + list(mech.bodies) if b.name == name), None)
    if j is not None and j.kind != "coupling":
        unit = "°" if j.kind in _ANGLE_KINDS else "mm"
        out.append(("valeur", "Position de la liaison ({u})".replace("{u}", unit)))
        if dynamic:
            out.append(("vitesse", "Vitesse de la liaison ({u}/s)".replace("{u}", unit)))
            if j.kind in _FORCE_KINDS:
                out.append(("effort", "Effort dans la liaison (N)"))
            if any(d[0] is j for d in mech.drivers):
                out.append(("couple", "Couple du moteur (N·m)" if j.kind in _ANGLE_KINDS
                            else "Force du moteur (N)"))
    elif b is not None and not b.fixed:
        out.append(("hauteur", "Hauteur du centre de gravité (mm)"))
        if dynamic:
            out.append(("vitesse_piece", "Vitesse du centre de gravité (m/s)"))
    if dynamic and name is None:
        out.append(("energie", "Énergie du mécanisme (J)"))
    return out


def _cog(mech, b, x):
    c = getattr(b, "com", None)
    return b.world_point(mech.pose(x, b), b.center if c is None else c)


def series(mech, rows, key, name, dynamic):
    """(xs, [(label, ys)], x label) — several curves for the energy (kinetic,
    potential, total), one otherwise. Raises ValueError when not available."""
    if not rows:
        raise ValueError("rien de calculé")
    if dynamic:
        xs = np.array([r["value"] for r in rows], float)
        xlab = "temps (s)"
    else:
        xs = 100.0 * np.array([r["t"] for r in rows], float)
        xlab = "course du moteur (%)"
    j = next((j for j in mech.joints if j.name == name), None)
    b = next((b for b in [mech.ground] + list(mech.bodies) if b.name == name), None)
    if key in ("valeur", "vitesse"):
        f = (lambda v: math.degrees(v)) if j.kind in _ANGLE_KINDS else (lambda v: 1000.0 * v)
        val = np.array([f(mech.continuous_value(j, r["x"])) for r in rows], float)
        if key == "valeur":
            return xs, [("", val)], xlab
        return xs, [("", _derivative(xs, val))], xlab
    if key == "effort":
        ys = []
        for r in rows:
            lam = (r.get("reac") or {}).get(j.name)
            ys.append(float(np.linalg.norm(np.asarray(lam, float)[:3])) if lam is not None else np.nan)
        return xs, [("", np.array(ys, float))], xlab
    if key == "couple":
        ys = np.array([(r.get("couples") or {}).get(j.name, np.nan) for r in rows], float)
        return xs, [("", ys)], xlab
    if key == "hauteur":
        ys = np.array([1000.0 * _cog(mech, b, r["x"])[2] for r in rows], float)
        return xs, [("", ys)], xlab
    if key == "vitesse_piece":
        P = np.array([_cog(mech, b, r["x"]) for r in rows], float)
        V = np.vstack([_derivative(xs, P[:, k]) for k in range(3)]).T
        return xs, [("", np.linalg.norm(V, axis=1))], xlab
    if key == "energie":
        ke = np.array([r.get("ke", np.nan) for r in rows], float)
        pe = np.array([r.get("pe", np.nan) for r in rows], float)
        pe = pe - np.nanmin(pe)                       # potential from the lowest point
        return xs, [("cinétique", ke), ("potentielle", pe), ("totale", ke + pe)], xlab
    raise ValueError(key)


def _derivative(xs, ys):
    """Central differences (one-sided at the ends)."""
    if len(xs) < 2:
        return np.zeros_like(ys)
    return np.gradient(ys, xs)


def nice_ticks(lo, hi, n=5):
    """Round tick values covering lo..hi (about n of them)."""
    if not np.isfinite(lo) or not np.isfinite(hi):
        return []
    if hi - lo < 1e-12:
        lo, hi = lo - 1.0, hi + 1.0
    raw = (hi - lo) / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = min((k * mag for k in (1, 2, 2.5, 5, 10) if k * mag >= raw), default=10 * mag)
    start = math.ceil(lo / step) * step
    return [start + i * step for i in range(int((hi - start) / step + 1e-9) + 1)]


def to_csv(xs, curves, xlab, ylab):
    """The curves as CSV text (semicolons, decimal points)."""
    head = [xlab] + [f"{ylab} {lab}".strip() for lab, _ in curves]
    lines = [";".join(head)]
    for i, x in enumerate(xs):
        lines.append(";".join([f"{x:.6g}"] + [f"{ys[i]:.6g}" for _, ys in curves]))
    return "\n".join(lines) + "\n"
