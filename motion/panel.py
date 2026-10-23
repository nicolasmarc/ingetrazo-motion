"""IngeTrazo side panel: define a mechanism on the model, play it, check it.

How it is used (all from the current selection):

- Pièce fixe / Pièce mobile: the selected groups become one body.
- Pivot / Glissière: ONE selected edge (or construction line) gives the
  axis (pivot) or the direction (slider); the two nearest bodies are joined.
- Guide / rail: select the rail's edges (lines and arc segments, smooth)
  plus one short mark per roller, drawn across the rail where the roller
  is; each roller is joined to the nearest moving body.
- Moteur 0–100 %: makes the joint chosen in the list the driver, with the
  travel typed under it.
- Valider: runs 0..100 % and reports convergence, collisions, stops and
  the smallest clearance against the minimum gap.
- Trajectoire: traces the end points of the selected edges (else the joint
  points) through the motion.

The animation is drawn OVER the model (an overlay): the model's groups do
not move, so playing never adds undo steps. The mechanism itself is kept in
the document (``app.set_document_data``), one undo step per edit.
"""
import math
import numpy as np
from PySide6.QtCore import QPointF, QSize, Qt, QTimer
from PySide6.QtGui import QColor, QPen
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QComboBox,
                               QDoubleSpinBox, QFormLayout, QGridLayout,
                               QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit,
                               QSlider, QSpinBox, QToolButton, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from . import collide, document as doc, icons, samples, solver
from .lang import T as _T

SOURCES = (("model", "Modèle (document)"),
           ("chaise", "Exemple : chaise pliante"),
           ("porte", "Exemple : porte sectionnelle"),
           ("oiseau", "Exemple : oiseau à battement d'ailes"),
           ("draisine", "Exemple : draisine à balancier"),
           ("chaine", "Exemple : chaîne détendue et tendeur"),
           ("chaine_souple", "Exemple : chaîne souple (dynamique)"),
           ("pendule_double", "Exemple : pendule double (dynamique)"))

SAMPLE_NAMES = {"chaise": "Chaise pliante en X", "porte": "Porte sectionnelle",
                "oiseau": "Oiseau à battement d'ailes",
                "draisine": "Draisine à balancier",
                "chaine": "Chaîne détendue et tendeur",
                "chaine_souple": "Chaîne souple (dynamique)",
                "pendule_double": "Pendule double (dynamique)"}

KIND_NAMES = {"fixed": "Pièce fixe", "moving": "Pièce mobile", "pivot": "Pivot",
              "slider": "Glissière", "guide": "Guide / rail",
              "cylindrical": "Pivot glissant", "ball": "Rotule", "lock": "Encastrement",
              "elastic": "Liaison élastique", "annular": "Linéaire annulaire",
              "pointplane": "Ponctuelle", "planar": "Appui plan", "screw": "Hélicoïdale",
              "coupling": "Couplage"}


def _detail(text):
    """A detection's detail line in the panel's language."""
    import re
    from .lang import english
    if not text or not english():
        return text
    for fr, en in ((r"\bjeu\b", "gap"), (r"\bportée\b", "bearing length"),
                   (r"\bappuis plans\b", "flat contacts"), (r"\bappuis\b", "contacts"),
                   (r"\bdents\b", "teeth"), (r"\brapport\b", "ratio"), (r"\bet\b", "and"),
                   (r"— axe « (.*?) » avec « (.*?) »", r"— pin « \1 » with « \2 »")):
        text = re.sub(fr, en, text)
    return text


TIPS = {
    "fixed": "Pièce fixe (Fixed part) — les groupes sélectionnés forment le bâti.",
    "moving": "Pièce mobile (Moving part) — les groupes sélectionnés bougent ensemble.",
    "pivot": "Pivot — sélectionner UNE arête ou ligne de construction sur l'axe.",
    "slider": "Glissière (Slider) — sélectionner UNE arête dans le sens du coulissement.",
    "guide": "Guide / rail — sélectionner les arêtes du rail + un petit trait en travers à chaque galet.",
    "driver": "Moteur 0–100 % (Driver) — la liaison choisie dans la liste, avec la course indiquée.",
    "validate": "Valider (Validate) — de 0 à 100 % : collisions, butées, jeu minimal.",
    "trajectory": "Trajectoire (Trajectory) — trace les extrémités des arêtes sélectionnées.",
    "cylindrical": "Pivot glissant (Cylindrical) — UNE arête sur l'axe : la pièce tourne et coulisse.",
    "ball": "Rotule (Ball) — des arêtes autour du centre (un petit cercle) : toutes les rotations.",
    "lock": "Encastrement (Lock) — des arêtes au point de liaison : les deux pièces n'en font qu'une.",
    "elastic": "Liaison élastique (Elastic) — des arêtes au point de liaison : un silentbloc, "
               "rigide en cinématique, ressorts et amortisseurs en dynamique.",
    "annular": "Linéaire annulaire (Annular) — UNE arête sur l'axe de l'alésage, plus un cercle "
               "de la sphère : la sphère glisse dans l'alésage et tourne en tous sens.",
    "pointplane": "Ponctuelle (Point–plane) — UNE arête le long de la normale, partant du point "
                  "de contact : le point reste sur le plan.",
    "planar": "Appui plan (Planar) — UNE arête le long de la normale au plan : la pièce glisse "
              "sur le plan et tourne autour de la normale.",
    "screw": "Hélicoïdale (Screw) — UNE arête sur l'axe, puis le pas : la pièce avance d'un pas "
             "par tour (pas négatif : filet à gauche).",
}


# -- reading the IngeTrazo scene (kept here, in one place) ----------------------

def _xyz(p):
    return np.array([p.x(), p.y(), p.z()], float)


def _placements(group):
    """(mesh, (rot, trans) | None) for the group and its nested placements."""
    try:
        from core.group import iter_placements, np_affine
    except Exception:                      # outside IngeTrazo (tests)
        yield group.mesh, None
        return
    for g, m in iter_placements(group):
        yield g.mesh, (None if m is None else np_affine(m))


def _positions(vertices):
    """(n, 3) array of vertex positions, read in one pass."""
    return np.array([(p.x(), p.y(), p.z()) for p in (v.position for v in vertices)],
                    float).reshape(-1, 3)


def group_points(group):
    out = []
    for mesh, aff in _placements(group):
        if not mesh.vertices:
            continue
        P = _positions(mesh.vertices)
        out.append(P if aff is None else P @ aff[0].T + aff[1])
    return np.vstack(out) if out else np.zeros((0, 3))


def _mesh_triangles(mesh):
    """(n, 3, 3) triangles of one mesh. Triangles and convex quads without
    holes (most of an imported STEP model) are indexed directly; any other
    face goes through IngeTrazo's own triangulation (holes, concave)."""
    verts = mesh.vertices
    if not verts:
        return np.zeros((0, 3, 3))
    P = _positions(verts)
    index = {id(v): i for i, v in enumerate(verts)}
    tris, quads, quad_faces, other = [], [], [], []
    for f in mesh.faces:
        loop = f.loop
        if f.hole_loops or len(loop) not in (3, 4):
            other.append(f)
            continue
        try:
            ids = [index[id(v)] for v in loop]
        except KeyError:
            other.append(f)
            continue
        if len(ids) == 3:
            tris.append(ids)
        else:
            quads.append(ids)
            quad_faces.append(f)
    parts = []
    if tris:
        parts.append(P[np.array(tris)])
    if quads:
        Q = P[np.array(quads)]                                  # (m, 4, 3)
        n = np.cross(Q[:, 2] - Q[:, 0], Q[:, 3] - Q[:, 1])
        e = np.roll(Q, -1, axis=1) - Q                          # edges i -> i+1
        turn = np.einsum("mij,mj->mi", np.cross(e, np.roll(e, -1, axis=1)), n)
        convex = (turn > 0).all(axis=1) | (turn < 0).all(axis=1)
        C = Q[convex]
        if len(C):
            parts.append(C[:, [0, 1, 2]])
            parts.append(C[:, [0, 2, 3]])
        if not convex.all():
            other += [f for f, ok in zip(quad_faces, convex) if not ok]
    tri = []
    for f in other:
        try:
            for a, b, c in f.triangulate():
                tri.append(((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()),
                            (c.x(), c.y(), c.z())))
        except Exception:
            continue
    if tri:
        parts.append(np.array(tri, float))
    return np.concatenate(parts) if parts else np.zeros((0, 3, 3))


def group_triangles(group):
    """World triangles (n, 3, 3) of a group's faces, holes cut (IngeTrazo's
    own triangulation), nested placements included."""
    out = []
    for mesh, aff in _placements(group):
        T = _mesh_triangles(mesh)
        if len(T):
            if aff is not None:
                T = T @ aff[0].T + aff[1]
            out.append(T)
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def group_material_names(group):
    """Names that may say what a group is made of: its own name, its paint,
    the paints of its faces."""
    names = [getattr(group, "name", "")]
    mat = getattr(group, "material", None) or {}
    names += [mat.get("mat", ""), mat.get("name", "")]
    seen = set()
    for f in list(getattr(group.mesh, "faces", []))[:400]:
        n = (getattr(f, "attrs", None) or {}).get("mat")
        if n and n not in seen:
            seen.add(n)
            names.append(n)
    return names


# the overlay of a moving part: at most this many strokes (a STEP part can
# bring tens of thousands of tessellation edges, drawn at every repaint)
OVERLAY_MAX_SEGMENTS = 1500


def _lighter(S, cap=None):
    """At most ``cap`` segments of (M, 2, 3), the longest first (they carry
    the shape; the short ones are the facets of curved faces)."""
    cap = OVERLAY_MAX_SEGMENTS if cap is None else cap
    if len(S) <= cap:
        return S
    L = np.einsum("ij,ij->i", S[:, 1] - S[:, 0], S[:, 1] - S[:, 0])
    keep = np.argpartition(-L, cap - 1)[:cap]
    return S[np.sort(keep)]


def group_segments(group):
    out = []
    for mesh, aff in _placements(group):
        shown = [e for e in mesh.edges if not getattr(e, "hidden", False)]
        sharp = [e for e in shown if not getattr(e, "soft", False)]
        segs = [((a.x(), a.y(), a.z()), (b.x(), b.y(), b.z()))
                for a, b in ((e.v0.position, e.v1.position) for e in (sharp or shown))]
        if not segs:
            continue
        S = np.array(segs, float)
        if aff is not None:
            S = S @ aff[0].T + aff[1]
        out.append(S)
    return np.vstack(out) if out else np.zeros((0, 2, 3))


class _Grip(QWidget):
    """A thin bar under the list: drag it to set the list's height."""

    def __init__(self, target, done=None):
        super().__init__()
        self.target, self.done = target, done
        self.setFixedHeight(9)
        self.setCursor(Qt.SizeVerCursor)
        self._y0 = self._h0 = None

    def paintEvent(self, ev):
        from PySide6.QtGui import QPainter
        p = QPainter(self)
        c = self.palette().mid().color()
        p.setPen(QPen(c, 1))
        w, y = self.width(), self.height() // 2
        for dx in (-6, -2, 2, 6):                    # four dots in the middle
            p.drawEllipse(QPointF(w / 2 + dx, y), 0.9, 0.9)
        p.end()

    def mousePressEvent(self, ev):
        self._y0 = ev.globalPosition().y()
        self._h0 = self.target.height()

    def mouseMoveEvent(self, ev):
        if self._y0 is None:
            return
        h = int(max(60, min(2000, self._h0 + ev.globalPosition().y() - self._y0)))
        self.target.setFixedHeight(h)

    def mouseReleaseEvent(self, ev):
        if self._y0 is not None and self.done is not None:
            self.done(self.target.height())
        self._y0 = None

    def mouseDoubleClickEvent(self, ev):            # back to the usual height
        self.target.setFixedHeight(90)
        if self.done is not None:
            self.done(90)


def _make_groups(parts):
    """IngeTrazo groups for ``excopy.parts``: [(body name, group)]."""
    from core.group import Group
    from core.mesh import Mesh
    out = []
    for body, fixed, groups in parts:
        for name, faces, col in groups:
            pos = np.vstack(faces)
            mesh = Mesh()
            mesh.add_faces_bulk(pos, np.array([len(f) for f in faces], dtype=np.int64),
                                np.ones(len(faces), dtype=np.int64))
            try:
                from core.orient import orient_outward
                orient_outward(mesh)
            except Exception:
                pass
            g = Group(mesh, name=name)
            g.material = {"color": tuple(c / 255.0 for c in col)}
            out.append((body, g))
    return out


def groups_by_uid(scene):
    f = getattr(scene, "groups_by_uid", None)
    if callable(f):
        return f()
    return {g.uid: g for g in getattr(scene, "groups", [])}


def selection(scene):
    """(groups, segments) of the current selection."""
    groups, segs = [], []
    for ent in list(getattr(scene, "selection", ()) or ()):
        if hasattr(ent, "mesh") and hasattr(ent, "uid"):
            groups.append(ent)
        elif hasattr(ent, "v0") and hasattr(ent, "v1"):          # Edge
            segs.append((_xyz(ent.v0.position), _xyz(ent.v1.position)))
        elif hasattr(ent, "a") and hasattr(ent, "b"):            # construction line
            try:
                segs.append((_xyz(ent.a), _xyz(ent.b)))
            except Exception:
                pass
    return groups, segs


# -- the panel ------------------------------------------------------------------------

class MotionPanel(QWidget):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.source = "model"
        self.data = doc.empty()
        self.mech = None
        self.sample_info = None
        self.problems = []
        self.frames = None          # validate() rows, or None
        self.report = None
        self.x = None               # pose shown now
        self.t = 0.0
        self.traces = []            # (body, point) followed by Trajectory
        self.trace_pts = []
        self.body_segments = {}     # body name -> (M, 2, 3) reference segments
        self._build_ui()
        self.timer = QTimer(self)
        self.timer.setInterval(40)
        self.timer.timeout.connect(self._tick)
        self.reload()

    # ---- UI -------------------------------------------------------------------------
    def _build_ui(self):
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        self.src = QComboBox()
        for key, text in SOURCES:
            self.src.addItem(_T(text), key)
        self.src.currentIndexChanged.connect(self._source_changed)
        srow = QHBoxLayout()
        srow.addWidget(self.src, 1)
        self.step_btn = QToolButton()
        self.step_btn.setText(_T("Importer STEP…"))
        self.step_btn.setToolTip(_T("Assemblage STEP (AP203/214/242) : une pièce = un groupe, "
                                   "en une seule étape d'annulation."))
        self.step_btn.clicked.connect(self._import_step_dialog)
        srow.addWidget(self.step_btn)
        self.scan_btn = QToolButton()
        self.scan_btn.setIcon(icons.icon("scan", self, 20))
        self.scan_btn.setText(_T("Détecter…"))
        self.scan_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.scan_btn.setToolTip(_T("Analyse les pièces (la sélection, sinon tout le modèle) et "
                                   "propose les liaisons : arbres dans des alésages (pivots), "
                                   "appuis plans (glissières), engrenages. Rien n'est appliqué "
                                   "avant validation."))
        self.scan_btn.clicked.connect(self._scan_dialog)
        srow.addWidget(self.scan_btn)
        lay.addLayout(srow)

        grid = QGridLayout()
        grid.setSpacing(2)
        self.buttons = {}
        from PySide6.QtWidgets import QSizePolicy
        for i, name in enumerate(icons.NAMES):
            b = QToolButton()
            b.setIcon(icons.icon(name, self, 24))
            b.setIconSize(QSize(24, 24))
            b.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            b.setText(icons.label(name))
            b.setToolTip(_T(TIPS[name]))
            b.setAutoRaise(True)
            b.setMinimumWidth(10)
            b.setFixedHeight(52)
            # every button fills its cell: the icons and their labels sit on
            # the same centre lines, column by column, row by row
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            b.clicked.connect(lambda _c=False, n=name: self._action(n))
            grid.addWidget(b, i // 4, i % 4)
            self.buttons[name] = b
        for c in range(4):
            grid.setColumnStretch(c, 1)
            grid.setColumnMinimumWidth(c, 10)
        self.buttons["trajectory"].setCheckable(True)
        lay.addLayout(grid)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels([_T("Nom"), _T("Type"), _T("Détail")])
        self.tree.setRootIsDecorated(False)
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tree.itemSelectionChanged.connect(self._item_selected)
        # a click (not the list being refilled) shows the item in the model
        self.tree.itemClicked.connect(lambda it, _c: self._highlight(it.data(0, Qt.UserRole)))
        # The list keeps the height the user gives it with the grip under it
        # (drag down: taller; the panel scrolls). A splitter could only share
        # the room already there, none on a laptop screen.
        self.tree.setFixedHeight(self._list_height())
        lay.addWidget(self.tree)
        self.grip = _Grip(self.tree, self._save_list_height)
        self.grip.setToolTip(_T("Glisser pour agrandir la liste"))
        lay.addWidget(self.grip)

        row = QHBoxLayout()
        self.delete_btn = QToolButton()
        self.delete_btn.setText(_T("Supprimer"))
        self.delete_btn.clicked.connect(self._delete)
        row.addWidget(self.delete_btn)
        row.addStretch(1)
        self.copy_btn = QToolButton()
        self.copy_btn.setText(_T("Copier dans le modèle"))
        self.copy_btn.setToolTip(_T("Crée les pièces de l'exemple dans le document (vrais groupes, "
                                   "ombrés, enregistrés dans le fichier) avec leurs liaisons et "
                                   "moteurs, à modifier comme un mécanisme fait à la main. "
                                   "L'exemple lui-même ne change pas."))
        self.copy_btn.clicked.connect(self.copy_example)
        self.copy_btn.setVisible(False)
        row.addWidget(self.copy_btn)
        lay.addLayout(row)

        mrow = QHBoxLayout()
        mrow.addWidget(QLabel(_T("Matériau")))
        self.mat_combo = QComboBox()
        self.mat_combo.setToolTip(_T("Matériau de la pièce choisie dans la liste (ou des groupes "
                                    "sélectionnés) : masse, centre de gravité et inertie en "
                                    "découlent."))
        self.mat_combo.activated.connect(self._material_chosen)
        mrow.addWidget(self.mat_combo, 1)
        self.mat_new = QToolButton()
        self.mat_new.setText(_T("Nouveau…"))
        self.mat_new.setToolTip(_T("Ajouter un matériau (ou corriger une valeur) pour ce document."))
        self.mat_new.clicked.connect(self._material_dialog)
        mrow.addWidget(self.mat_new)
        lay.addLayout(mrow)
        self.mass_label = QLabel("")
        self.mass_label.setWordWrap(True)
        lay.addWidget(self.mass_label)

        box = QGroupBox(_T("Liaison choisie"))
        form = QFormLayout(box)
        self.joint_form = form
        self.combo_a, self.combo_b = QComboBox(), QComboBox()
        form.addRow(_T("Pièce A"), self.combo_a)
        form.addRow(_T("Pièce B"), self.combo_b)
        self.lim_on = QCheckBox(_T("Butées"))
        self.lim_lo, self.lim_hi = self._spin(), self._spin()
        lr = QHBoxLayout()
        lr.addWidget(self.lim_on)
        lr.addWidget(self.lim_lo)
        lr.addWidget(self.lim_hi)
        form.addRow(lr)
        self.lim_row = lr
        self.drv_lo, self.drv_hi = self._spin(), self._spin()
        dr = QHBoxLayout()
        dr.addWidget(self.drv_lo)
        dr.addWidget(QLabel("→"))
        dr.addWidget(self.drv_hi)
        self.unit = QLabel("")
        dr.addWidget(self.unit)
        form.addRow(_T("Course moteur"), dr)
        self.drv_row = dr
        self.motor_on = QCheckBox(_T("Moteur (plusieurs possibles : même 0–100 %, chacun sa course)"))
        self.motor_on.setToolTip(_T("Coché : cette liaison est un moteur. Vitesse = course ; "
                                   "déphasage = valeur de départ."))
        form.addRow(self.motor_on)
        self.k_spin, self.c_spin, self.tq_spin = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for w_, hi_ in ((self.k_spin, 1e9), (self.c_spin, 1e7), (self.tq_spin, 1e6)):
            w_.setRange(-hi_ if w_ is self.tq_spin else 0.0, hi_)
            w_.setDecimals(3)
        kr = QHBoxLayout()
        self.k_label, self.c_label = QLabel(_T("Ressort")), QLabel(_T("Amort."))
        kr.addWidget(self.k_label)
        kr.addWidget(self.k_spin)
        kr.addWidget(self.c_label)
        kr.addWidget(self.c_spin)
        form.addRow(kr)
        self.k_row = kr
        self.k_spin.setToolTip(_T("Raideur (N·m/rad pour un pivot, N/m pour une glissière ou une "
                                 "liaison élastique), au repos dans la position dessinée. 0 : aucun."))
        self.c_spin.setToolTip(_T("Amortissement visqueux (N·m·s/rad ou N·s/m). Dynamique seulement."))
        tr = QHBoxLayout()
        self.tq_on = QCheckBox(_T("Moteur en couple"))
        self.tq_on.setToolTip(_T("Dynamique : un couple (N·m) ou une force (N) constant ; le "
                                "mécanisme répond (au lieu d'imposer la course)."))
        tr.addWidget(self.tq_on)
        tr.addWidget(self.tq_spin)
        form.addRow(tr)
        self.tq_row = tr
        self.apply_btn = QToolButton()
        self.apply_btn.setText(_T("Appliquer"))
        self.apply_btn.clicked.connect(self._apply_joint)
        form.addRow(self.apply_btn)
        lay.addWidget(box)

        play = QHBoxLayout()
        self.play_btn = QToolButton()
        self.play_btn.setText("▶")
        self.play_btn.setToolTip(_T("Lecture / pause"))
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self._play)
        play.addWidget(self.play_btn)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.valueChanged.connect(self._scrub)
        play.addWidget(self.slider, 1)
        self.pct = QLabel("0 %")
        self.pct.setMinimumWidth(42)
        play.addWidget(self.pct)
        lay.addLayout(play)
        self.loop = QCheckBox(_T("Aller-retour"))
        self.loop.setChecked(True)
        prow = QHBoxLayout()
        prow.addWidget(self.loop)
        prow.addStretch(1)
        self.video_btn = QToolButton()
        self.video_btn.setText(_T("Exporter la vidéo…"))
        self.video_btn.setToolTip(_T("Vidéo en écran partagé : la vue IngeTrazo à gauche, "
                                    "l'animation ombrée et les mesures à droite."))
        self.video_btn.clicked.connect(self._export_dialog)
        prow.addWidget(self.video_btn)
        lay.addLayout(prow)

        val = QHBoxLayout()
        val.addWidget(QLabel(_T("Jeu mini")))
        self.jeu = QDoubleSpinBox()
        self.jeu.setRange(0, 100)
        self.jeu.setDecimals(1)
        self.jeu.setSuffix(" mm")
        self.jeu.setValue(5.0)
        val.addWidget(self.jeu)
        val.addWidget(QLabel(_T("Pas")))
        self.pas = QSpinBox()
        self.pas.setRange(3, 401)
        self.pas.setValue(51)
        val.addWidget(self.pas)
        lay.addLayout(val)
        drow = QHBoxLayout()
        self.dyn_on = QCheckBox(_T("Dynamique"))
        self.dyn_on.setToolTip(_T("Masses (matériaux), pesanteur, ressorts, amortisseurs, butées, "
                                 "moteurs en couple : Valider, la lecture et la vidéo suivent la "
                                 "simulation dans le temps ; les moteurs en vitesse font leur "
                                 "course en « durée » et le rapport donne leur couple."))
        self.dyn_on.toggled.connect(self._dynamics_toggled)
        drow.addWidget(self.dyn_on)
        drow.addWidget(QLabel(_T("durée")))
        self.dur = QDoubleSpinBox()
        self.dur.setRange(0.1, 600)
        self.dur.setDecimals(1)
        self.dur.setSuffix(" s")
        self.dur.setValue(4.0)
        self.dur.editingFinished.connect(lambda: self._dynamics_toggled(self.dyn_on.isChecked()))
        drow.addWidget(self.dur)
        self.chocs_on = QCheckBox(_T("Chocs"))
        self.chocs_on.setToolTip(_T("Chocs et appuis entre pièces libres (dominos, blocs, butées "
                                    "posées) : les boîtes des pièces se repoussent, avec le "
                                    "frottement et le rebond de leurs matériaux. Les pièces "
                                    "tenues par une liaison ne se heurtent pas."))
        self.chocs_on.toggled.connect(lambda on: self._dynamics_toggled(self.dyn_on.isChecked()))
        drow.addWidget(self.chocs_on)
        drow.addStretch(1)
        lay.addLayout(drow)

        self.out = QPlainTextEdit()
        self.out.setReadOnly(True)
        self.out.setMaximumBlockCount(500)
        lay.addWidget(self.out, 1)
        self.status = QLabel("")
        self.status.setWordWrap(True)
        lay.addWidget(self.status)

    @staticmethod
    def _spin():
        s = QDoubleSpinBox()
        s.setRange(-1e6, 1e6)
        s.setDecimals(1)
        return s

    def _list_height(self):
        try:
            from PySide6.QtCore import QSettings
            v = int(QSettings("IngeTrazo", "motion").value("hauteur_liste", 90))
        except Exception:
            v = 90
        return max(60, min(v, 2000))

    def _save_list_height(self, h):
        try:
            from PySide6.QtCore import QSettings
            QSettings("IngeTrazo", "motion").setValue("hauteur_liste", int(h))
        except Exception:
            pass

    def say(self, text):
        self.status.setText(text)
        win = getattr(self.app, "window", None)
        bar = getattr(win, "statusBar", None)
        try:
            if callable(bar):
                bar().showMessage(text, 6000)
        except Exception:
            pass

    # ---- data ---------------------------------------------------------------------
    def _doc_key(self):
        """What the mechanism depends on: the scene's version (geometry) and
        the document data. None while Motion itself hides the moving parts
        for a picture (the version then takes a value of its own)."""
        import json
        sc = self._scene()
        ver = getattr(sc, "version", None)
        if getattr(self, "_real_version", None) is not None or (isinstance(ver, int) and ver < 0):
            return None
        try:
            data = json.dumps(self.app.document_data(default=None), sort_keys=True)
        except Exception:
            data = None
        return (ver, data, self.source)

    def reload(self):
        """The document changed (edit, undo, New, Open). IngeTrazo also sends
        this signal when it only redraws (a version change seen at paint
        time, e.g. while a video is rendered): then nothing is rebuilt."""
        if getattr(self, "_quiet", 0):
            return
        key = self._doc_key()
        if key is None:
            return
        if key == getattr(self, "_last_key", None) and self.mech is not None:
            return
        self._last_key = key
        if self.source == "model":
            self.data = doc.normalized(self.app.document_data(default=None))
            self.jeu.blockSignals(True)
            self.jeu.setValue(1000 * self.data.get("jeu_min", 0.005))
            self.jeu.blockSignals(False)
            self.pas.setValue(int(self.data.get("pas", 51)))
            sim = self.data.get("simulation") or {}
            for w_ in (self.dyn_on, self.dur, self.chocs_on):
                w_.blockSignals(True)
            self.dyn_on.setChecked(bool(sim.get("active")))
            self.dur.setValue(float(sim.get("duree", 4.0)))
            self.chocs_on.setChecked(bool(sim.get("chocs")))
            for w_ in (self.dyn_on, self.dur, self.chocs_on):
                w_.blockSignals(False)
        self._invalidate()
        self._fill_tree()

    def _store(self, data, message=""):
        data["jeu_min"] = self.jeu.value() / 1000
        data["pas"] = self.pas.value()
        self.data = data
        self.app.set_document_data(data)    # one undo step; reload follows
        self._invalidate()
        self._fill_tree()
        if message:
            self.say(message)

    def _invalidate(self):
        self.mech = None
        self.frames = None
        self.report = None
        self.x = None
        self.trace_pts = []

    def _scene(self):
        return self.app.scene

    def _boxes(self):
        by = groups_by_uid(self._scene())
        return doc.body_boxes(self.data, lambda u: group_points(by[u]) if u in by else None)

    def mechanism(self):
        """The current mechanism (built once per change)."""
        if self.mech is not None:
            return self.mech
        builders = {"chaise": samples.chaise_pliante, "porte": samples.porte_sectionnelle,
                    "oiseau": samples.oiseau, "draisine": samples.draisine,
                    "chaine": samples.chaine, "chaine_souple": samples.chaine_souple,
                    "pendule_double": samples.pendule_double}
        self.sample_info = None
        if self.source in builders:
            self.mech, self.sample_info = builders[self.source]()
            self.problems = []
        else:
            by = groups_by_uid(self._scene())
            self.mech, self.problems = doc.build(
                self.data, lambda u: group_points(by[u]) if u in by else None,
                lambda u: getattr(by.get(u), "name", None),
                triangles_of_group=lambda u: self._triangles(by, u))
            self.body_segments = {}
            for b in self.data["bodies"]:
                if b["fixed"]:
                    continue                     # never drawn: the model shows it
                segs = [group_segments(by[u]) for u in b["groups"] if u in by]
                segs = [s for s in segs if len(s)]
                if segs:
                    self.body_segments[b["name"]] = _lighter(np.vstack(segs))
            if getattr(self, "_masses_pending", False):     # built before the timer:
                self._masses_pending = False               # show them now
                self._show_masses(getattr(self.mech, "masses", {}) or {})
        if self.source != "model":
            self.body_segments = {}
        return self.mech

    def _triangles(self, by, uid):
        """A group's triangles, cached while its geometry is unchanged."""
        g = by.get(uid)
        if g is None:
            return None
        cache = self.__dict__.setdefault("_tri_cache", {})
        faces = getattr(g.mesh, "faces", [])
        try:
            v0 = faces[0].vertices[0] if faces else None
            key = (id(g.mesh), len(faces), None if v0 is None else (v0.x(), v0.y(), v0.z()),
                   repr(getattr(g, "xform", None)))
        except Exception:
            key = None
        hit = cache.get(uid)
        if hit is not None and key is not None and hit[0] == key:
            return hit[1]
        T = group_triangles(g)
        cache[uid] = (key, T)
        return T

    def _guess_materials(self, data, uids):
        """Groups named after their matter ('Pied chêne', 'Steel') get it,
        unless they already have one."""
        from . import materials
        by = groups_by_uid(self._scene())
        for u in uids:
            if str(u) in data.get("materiau", {}) or u not in by:
                continue
            key = materials.guess(*group_material_names(by[u]))
            if key:
                data = doc.set_material(data, [u], key)
        return data

    def _fill_tree(self):
        self.tree.blockSignals(True)
        self.tree.clear()
        if self.source == "model":
            bodies = [(b["name"], "fixed" if b["fixed"] else "moving",
                       _T("{n} groupe(s)", n=len(b['groups']))) for b in self.data["bodies"]]
            joints = [(j["name"], j["kind"], f"{j['a']} ↔ {j['b']}")
                      for j in self.data["joints"]]
            motors = [x["joint"] for x in self.data["drivers"]]
        else:
            m = self.mechanism()
            bodies = [(m.ground.name, "fixed", "")] + [(b.name, "moving", "")
                                                       for b in m.bodies]
            joints = [(j.name, j.kind, f"{j.a.name} ↔ {j.b.name}") for j in m.joints]
            motors = [d[0].name for d in m.drivers]
        # Masses need the whole mechanism (every face of every part): never
        # inside the document-changed signal, or opening a big model freezes
        # the window. Shown at once when known, else computed right after.
        masses = self._known_masses()
        if self.mech is None and self.source == "model" and self.data["bodies"]:
            self._schedule_masses()
        if masses:
            bodies = [(n, k, self._mass_text(masses.get(n)) or dt) for n, k, dt in bodies]
        kinds = {k: _T(v) for k, v in KIND_NAMES.items()}
        for name, kind, detail in bodies + joints:
            it = QTreeWidgetItem([name, kinds[kind], detail])
            it.setIcon(0, icons.icon(kind, self.tree, 16))
            it.setData(0, Qt.UserRole, name)
            if name in motors:
                it.setIcon(1, icons.icon("driver", self.tree, 16))
                it.setText(1, kinds[kind] + _T(" · moteur"))
            self.tree.addTopLevelItem(it)
        for c in range(3):
            self.tree.resizeColumnToContents(c)
        self.tree.blockSignals(False)
        editable = self.source == "model"
        from .excopy import NOT_COPYABLE
        self.copy_btn.setVisible(not editable and self.source not in NOT_COPYABLE)
        for n in ("fixed", "moving", "pivot", "slider", "guide",
                  "cylindrical", "ball", "lock", "elastic",
                  "annular", "pointplane", "planar", "screw"):
            self.buttons[n].setEnabled(editable)
        self.delete_btn.setEnabled(editable)
        self._fill_materials()
        self.mat_combo.setEnabled(editable)
        self.mat_new.setEnabled(editable)
        self.apply_btn.setEnabled(editable)
        names = [b["name"] for b in self.data["bodies"]] if editable else []
        for c in (self.combo_a, self.combo_b, self.lim_on, self.lim_lo, self.lim_hi):
            c.setEnabled(editable)
        for c in (self.combo_a, self.combo_b):
            c.blockSignals(True)
            c.clear()
            c.addItems(names)
            c.blockSignals(False)
        self._show_fields(None)
        self._first_motor = motors[0] if motors else None
        if motors and (self.mech is not None or self.source != "model"):
            self._select_first_motor()           # show the first motor's travel

    def _select_first_motor(self):
        name = getattr(self, "_first_motor", None)
        hit = self.tree.findItems(name, Qt.MatchExactly) if name else []
        if hit and self.tree.currentItem() is None:
            self.tree.setCurrentItem(hit[0])

    # ---- materials ------------------------------------------------------------------
    def _fill_materials(self):
        from . import materials
        lib = materials.library(self.data if self.source == "model" else None)
        self.mat_combo.blockSignals(True)
        self.mat_combo.clear()
        self.mat_combo.addItem(_T("— non défini —"), None)
        order = {"bois": 0, "métal": 1, "plastique": 2, "autre": 3}
        for k in sorted(lib, key=lambda k: (order.get(lib[k].get("family"), 9), lib[k]["name"])):
            r = lib[k]
            self.mat_combo.addItem(f"{_T(r['name'])} — {r['density']:g} kg/m³", k)
        self.mat_combo.blockSignals(False)
        self._show_material(self._chosen())

    def _body_record(self, name):
        return next((b for b in self.data["bodies"] if b["name"] == name), None)

    def _show_material(self, name):
        """Combo and mass line for the body ``name`` (if it is one)."""
        if self.source != "model":
            self.mass_label.setText("")
            return
        b = self._body_record(name) if name else None
        if b is None:
            self.mass_label.setText("")
            return
        keys = {self.data["materiau"].get(u) for u in b["groups"]}
        self.mat_combo.blockSignals(True)
        if len(keys) == 1:
            i = self.mat_combo.findData(keys.pop())
            self.mat_combo.setCurrentIndex(max(i, 0))
        else:
            self.mat_combo.setCurrentIndex(-1)
        self.mat_combo.blockSignals(False)
        mp = self._known_masses().get(name)
        if not mp:
            self.mass_label.setText("")
            return
        if mp["mass"] > 0:
            c = mp["com"] * 1000
            txt = _T("{name} : {m} kg, volume {v} cm³ — centre de gravité ({x} ; {y} ; {z}) mm",
                    name=name, m=f"{mp['mass']:.3g}", v=f"{mp['volume'] * 1e6:.4g}",
                    x=f"{c[0]:.1f}", y=f"{c[1]:.1f}", z=f"{c[2]:.1f}")
        else:
            txt = _T("{name} : volume {v} cm³ — choisir un matériau",
                    name=name, v=f"{mp['volume'] * 1e6:.4g}")
        if mp["missing"] and mp["mass"] > 0:
            txt += _T(" ; {n} groupe(s) sans matériau", n=len(mp['missing']))
        self.mass_label.setText(txt)

    def _material_chosen(self, _i=None):
        if self.source != "model":
            return
        key = self.mat_combo.currentData()
        name = self._chosen()
        b = self._body_record(name) if name else None
        if b is not None:
            uids = b["groups"]
        else:
            groups, _ = selection(self._scene())
            uids = [g.uid for g in groups]
        if not uids:
            self.say(_T("Choisir une pièce dans la liste, ou sélectionner des groupes."))
            return
        d = doc.set_material(self.data, uids, key)
        label = self.mat_combo.currentText().split(" — ")[0]
        self._store(d, _T("{label} : {n} groupe(s).", label=label, n=len(uids)))
        if name:
            hit = self.tree.findItems(name, Qt.MatchExactly)
            if hit:
                self.tree.setCurrentItem(hit[0])

    def _material_dialog(self):
        from PySide6.QtWidgets import QDialog, QDialogButtonBox, QLineEdit
        from . import materials
        dlg = QDialog(self)
        dlg.setWindowTitle(_T("Matériau du document"))
        form = QFormLayout(dlg)
        name = QLineEdit()
        base = QComboBox()
        lib = materials.library(self.data)
        base.addItem(_T("(nouveau)"), None)
        for k in sorted(lib, key=lambda k: _T(lib[k]["name"])):
            base.addItem(_T(lib[k]["name"]), k)
        rho, E, mu, e = QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox(), QDoubleSpinBox()
        for w, lo, hi, dec, suf in ((rho, 1, 25000, 0, " kg/m³"), (E, 0, 1000, 2, " GPa"),
                                    (mu, 0, 2, 2, ""), (e, 0, 1, 2, "")):
            w.setRange(lo, hi)
            w.setDecimals(dec)
            w.setSuffix(suf)
        rho.setValue(700)

        def load(_i=None):
            k = base.currentData()
            if k is None:
                return
            r = lib[k]
            name.setText(_T(r["name"]))
            rho.setValue(r["density"])
            E.setValue(r.get("young") or 0)
            mu.setValue(r.get("friction") or 0)
            e.setValue(r.get("restitution") or 0)
        base.currentIndexChanged.connect(load)
        form.addRow(_T("Partir de"), base)
        form.addRow(_T("Nom"), name)
        form.addRow(_T("Masse volumique"), rho)
        form.addRow(_T("Module d'Young"), E)
        form.addRow(_T("Frottement (sec)"), mu)
        form.addRow(_T("Restitution (chocs)"), e)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Cancel).setText(_T("Annuler"))
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QDialog.Accepted or not name.text().strip():
            return
        key = base.currentData()
        if key is None or _T(lib[key]["name"]) != name.text().strip():
            key = "doc:" + materials._plain(name.text().strip()).replace(" ", "-")
        fam = lib[base.currentData()]["family"] if base.currentData() else "autre"
        d = doc.add_material(self.data, key, dict(
            name=name.text().strip(), family=fam, density=rho.value(),
            young=E.value() or None, friction=mu.value(), restitution=e.value()))
        self._store(d, _T("Matériau « {name} » : {rho} kg/m³.", name=name.text().strip(),
                         rho=f"{rho.value():g}"))

    def _known_masses(self):
        """Masses of the mechanism already built, without building it."""
        return (getattr(self.mech, "masses", None) or {}) if self.mech is not None else {}

    def _schedule_masses(self):
        if getattr(self, "_masses_pending", False):
            return
        self._masses_pending = True
        QTimer.singleShot(0, self._compute_masses)

    def _compute_masses(self):
        """Build the mechanism once the window is drawn, then fill the masses."""
        self._masses_pending = False
        if self.source != "model" or not self.data["bodies"]:
            return
        if self.mech is None:
            self.say(_T("Calcul des masses…"))
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            masses = getattr(self.mechanism(), "masses", {}) or {}
        except Exception:
            masses = {}
        finally:
            QApplication.restoreOverrideCursor()
        self.say("")
        self._show_masses(masses)

    def _show_masses(self, masses):
        """Masses in the list and under the material, once they are known."""
        for i in range(self.tree.topLevelItemCount()):
            it = self.tree.topLevelItem(i)
            txt = self._mass_text(masses.get(it.data(0, Qt.UserRole)))
            if txt:
                it.setText(2, txt)
        self.tree.resizeColumnToContents(2)
        if self.tree.currentItem() is None:
            self._select_first_motor()
        else:
            self._show_material(self._chosen())

    @staticmethod
    def _mass_text(mp):
        if not mp:
            return ""
        mats = " + ".join(_T(x) for x in mp["materials"]) if mp["materials"] else ""
        if mp["missing"] and not mp["materials"]:
            return _T("matériau ?")
        txt = f"{mats} · {mp['mass']:.3g} kg" if mats else f"{mp['mass']:.3g} kg"
        if mp["missing"]:
            txt += _T(" (+{n} sans matériau)", n=len(mp['missing']))
        return txt

    def _chosen(self):
        it = self.tree.currentItem()
        return it.data(0, Qt.UserRole) if it is not None else None

    def _joint_record(self, name):
        for j in self.data["joints"]:
            if j["name"] == name:
                return j
        return None

    def _is_angle(self, kind):
        return kind in ("pivot", "cylindrical", "screw")

    def _to_ui(self, kind, v):
        return math.degrees(v) if self._is_angle(kind) else 1000 * v

    def _from_ui(self, kind, v):
        return math.radians(v) if self._is_angle(kind) else v / 1000

    # what each kind of joint can take: stops, motor travel, spring and
    # damper, torque motor (the rest of the box is hidden)
    FIELDS = {"pivot": "lmkct", "slider": "lmkct", "cylindrical": "lmkct", "screw": "lmkct",
              "guide": "lm", "ball": "c", "elastic": "kc"}

    def _show_fields(self, kind):
        """Only the fields that mean something for this kind of joint
        (``None``: a part, or nothing chosen)."""
        f = self.FIELDS.get(kind, "") if kind else ""
        form = self.joint_form
        ab = kind is not None and kind != "coupling"
        for w in (self.combo_a, self.combo_b):
            form.setRowVisible(w, ab)
        form.setRowVisible(self.lim_row, "l" in f)
        form.setRowVisible(self.drv_row, "m" in f)
        form.setRowVisible(self.motor_on, "m" in f)
        form.setRowVisible(self.k_row, "k" in f or "c" in f)
        for w in (self.k_label, self.k_spin):
            w.setVisible("k" in f)
        for w in (self.c_label, self.c_spin):
            w.setVisible("c" in f)
        form.setRowVisible(self.tq_row, "t" in f)
        form.setRowVisible(self.apply_btn, kind is not None)
        if kind == "elastic":
            self.k_label.setText(_T("Raideur"))
            self.k_spin.setToolTip(_T("Raideur en translation (N/m) ; en rotation : 1 % de cette "
                                     "valeur en N·m/rad."))
        else:
            self.k_label.setText(_T("Ressort"))

    def _highlight(self, name):
        """Show in the model what was clicked in the list: a part's groups
        (or, for a joint, both its parts' groups) become IngeTrazo's
        selection; a joint's point gets a ring in the view."""
        self._hl_point, self._hl_bodies, self._hl_name = None, set(), name
        if not name:
            return
        if self.source == "model":
            bodies = {b["name"]: b for b in self.data["bodies"]}
            rec = self._joint_record(name)
            names = [rec["a"], rec["b"]] if rec is not None else ([name] if name in bodies else [])
            if rec is not None and rec.get("point") is not None:
                self._hl_point = np.asarray(rec["point"], float)
            by = groups_by_uid(self._scene())
            groups = [by[u] for n in names if n in bodies for u in bodies[n]["groups"] if u in by]
            sel = getattr(self._scene(), "selection", None)
            if sel is not None and groups:
                try:
                    sel.clear()
                    sel.update(groups)
                except Exception:
                    pass
            self._hl_bodies = set(names)
        else:
            m = self.mechanism()
            j = next((j for j in m.joints if j.name == name), None)
            if j is not None:
                self._hl_bodies = {j.a.name, j.b.name}
                if j.kind != "coupling":
                    self._hl_point = np.asarray(j.point, float)
            else:
                self._hl_bodies = {name}
        try:
            self.app.viewport.update()
        except Exception:
            pass

    def _item_selected(self):
        name = self._chosen()
        if getattr(self, "_hl_point", None) is not None or getattr(self, "_hl_bodies", None):
            if name not in getattr(self, "_hl_bodies", set()) | {getattr(self, "_hl_name", None)}:
                self._hl_point, self._hl_bodies = None, set()   # stale highlight
        self._show_material(name)
        if self.source != "model":
            j = next((j for j in self.mechanism().joints if j.name == name), None)
            self._show_fields(j.kind if j is not None else None)
            if j is not None:
                self._show_travel(j.kind, j, None)
            return
        j = self._joint_record(name)
        self._show_fields(j["kind"] if j is not None else None)
        if j is None:
            return
        self.combo_a.setCurrentText(j["a"])
        self.combo_b.setCurrentText(j["b"])
        lim = j.get("limits")
        self.lim_on.setChecked(bool(lim))
        if lim:
            self.lim_lo.setValue(self._to_ui(j["kind"], lim[0]))
            self.lim_hi.setValue(self._to_ui(j["kind"], lim[1]))
        mj = next((k for k in self.mechanism().joints if k.name == name), None)
        self._show_travel(j["kind"], mj, j)
        if j["kind"] == "elastic":
            self.k_spin.setValue(j.get("k_lin", 1e5))
            self.c_spin.setValue(j.get("c_lin", 50.0))
        else:
            self.k_spin.setValue((j.get("spring") or [0.0])[0])
            self.c_spin.setValue(j.get("damper") or 0.0)
        tq = self.data.get("couples", {}).get(j["name"])
        self.tq_on.setChecked(tq is not None)
        self.tq_spin.setValue(tq or 0.0)

    def _show_travel(self, kind, mj, rec):
        self.unit.setText("°" if self._is_angle(kind) else "mm")
        m = self.mechanism()
        mine = None
        if self.source != "model":
            mine = next(((lo, hi) for jj, lo, hi in m.drivers if jj is mj), None)
        elif rec is not None:
            mine = next(((x["lo"], x["hi"]) for x in self.data["drivers"]
                         if x["joint"] == rec["name"]), None)
        self.motor_on.setChecked(mine is not None)
        if mine is not None:
            lo, hi = mine
        else:
            lo = m.joint_value(mj, np.zeros(m.n)) if mj is not None else 0.0
            if kind == "pivot":
                hi = lo + math.pi / 2
            elif kind == "guide" and mj is not None:
                hi = mj.path.length
            else:
                hi = lo + 0.1
        self.drv_lo.setValue(self._to_ui(kind, lo))
        self.drv_hi.setValue(self._to_ui(kind, hi))

    # ---- actions --------------------------------------------------------------------
    def _action(self, name):
        try:
            getattr(self, "_do_" + name)()
        except Exception as exc:          # never break the app; say why
            self.say(f"{icons.label(name)} : {exc}")
        self.app.viewport.update()

    def _do_fixed(self, fixed=True):
        groups, _ = selection(self._scene())
        if not groups:
            self.say(_T("Sélectionner un ou plusieurs groupes."))
            return
        d, name = doc.add_body(self.data, [g.uid for g in groups], fixed)
        d = self._guess_materials(d, [g.uid for g in groups])
        self._store(d, _T("{name} : {n} groupe(s).", name=name, n=len(groups)))

    def _do_moving(self):
        self._do_fixed(False)

    def _two_bodies(self, point):
        boxes = self._boxes()
        if len(boxes) < 2:
            raise ValueError(_T("définir d'abord au moins deux pièces"))
        b1 = doc.nearest_body(point, boxes)
        b2 = doc.nearest_body(point, boxes, exclude=(b1,))
        fixed = {b["name"] for b in self.data["bodies"] if b["fixed"]}
        return (b2, b1) if b1 not in fixed and b2 in fixed else (b1, b2)

    def _one_segment(self):
        _, segs = selection(self._scene())
        if len(segs) != 1:
            raise ValueError(_T("sélectionner UNE arête (ou ligne de construction)"))
        p0, p1 = segs[0]
        if np.linalg.norm(p1 - p0) < 1e-9:
            raise ValueError(_T("arête de longueur nulle"))
        return p0, p1

    def _other_joint(self, kind):
        self._action(kind)

    def _do_cylindrical(self):
        self._do_pivot("cylindrical")

    def _point_joint(self, kind):
        _, segs = selection(self._scene())
        if not segs:
            raise ValueError(_T("sélectionner des arêtes au point de liaison "
                               "(un cercle autour du centre, ou une arête)"))
        P = np.array([p for sg in segs for p in sg])
        c = 0.5 * (P.min(axis=0) + P.max(axis=0))
        a, b = self._two_bodies(c)
        d, name = doc.add_joint(self.data, kind, a, b, point=c)
        self._store(d, f"{name} : {a} ↔ {b}.")

    def _do_ball(self):
        self._point_joint("ball")

    def _do_lock(self):
        self._point_joint("lock")

    def _do_elastic(self):
        self._point_joint("elastic")

    def _do_annular(self):
        _, segs = selection(self._scene())
        if not segs:
            raise ValueError(_T("sélectionner UNE arête sur l'axe de l'alésage "
                               "(et un cercle de la sphère)"))
        lens = [np.linalg.norm(q - p) for p, q in segs]
        k = int(np.argmax(lens)) if len(segs) > 1 else 0
        p0, p1 = segs[k]
        ax = (p1 - p0) / max(lens[k], 1e-12)
        rest = [p for i, sg in enumerate(segs) if i != k for p in sg]
        if rest:                       # the ball's centre, from its circle
            P = np.array(rest)
            c = 0.5 * (P.min(axis=0) + P.max(axis=0))
        else:
            c = 0.5 * (p0 + p1)
        a, b = self._two_bodies(c)
        d, name = doc.add_joint(self.data, "annular", a, b, point=c, axis=ax)
        self._store(d, f"{name} : {a} ↔ {b}.")

    def _do_pointplane(self):
        self._do_pivot("pointplane")

    def _do_planar(self):
        self._do_pivot("planar")

    def ask_pitch(self):
        """Pitch of a new hélicoïdale (mm per turn)."""
        from PySide6.QtWidgets import QInputDialog
        v, ok = QInputDialog.getDouble(self, _T("Hélicoïdale"), _T("Pas (mm par tour) :"),
                                       1.0, -1000.0, 1000.0, 3)
        return v / 1000 if ok and v else None

    def _do_screw(self):
        p0, p1 = self._one_segment()
        pitch = self.ask_pitch()
        if not pitch:
            return
        a, b = self._two_bodies((p0 + p1) / 2)
        d, name = doc.add_joint(self.data, "screw", a, b, point=(p0 + p1) / 2,
                                axis=p1 - p0, pitch=pitch)
        self._store(d, _T("{name} : {a} ↔ {b}, pas {p} mm.", name=name, a=a, b=b,
                         p=f"{1000 * pitch:g}"))

    def _dynamics_toggled(self, on):
        if self.source != "model":
            return
        d = doc.set_simulation(self.data, on, self.dur.value(), self.chocs_on.isChecked())
        self._store(d, _T("Dynamique : simulation dans le temps.") if on else _T("Dynamique : arrêtée."))

    def _do_pivot(self, kind="pivot"):
        p0, p1 = self._one_segment()
        a, b = self._two_bodies((p0 + p1) / 2)
        d, name = doc.add_joint(self.data, kind, a, b,
                                point=p0 if kind in ("slider", "pointplane", "planar")
                                else (p0 + p1) / 2, axis=p1 - p0)
        self._store(d, f"{name} : {a} ↔ {b}.")

    def _do_slider(self):
        self._do_pivot("slider")

    def _do_guide(self):
        _, segs = selection(self._scene())
        if len(segs) < 2:
            raise ValueError(_T("sélectionner le rail et un trait par galet"))
        rail, rollers = doc.rail_and_rollers(segs)
        if not rollers:
            raise ValueError(_T("aucun galet : tracer un petit trait en travers du rail"))
        boxes = self._boxes()
        fixed = [b["name"] for b in self.data["bodies"] if b["fixed"]]
        if not fixed:
            raise ValueError(_T("définir d'abord la pièce fixe qui porte le rail"))
        a = doc.nearest_body(np.mean(rail, axis=0), {k: boxes[k] for k in fixed}) or fixed[0]
        d, names = self.data, []
        for r in rollers:
            b = doc.nearest_body(r, boxes, exclude=fixed)
            if b is None:
                raise ValueError(_T("aucune pièce mobile près du galet"))
            d, n = doc.add_joint(d, "guide", a, b, point=r, rail=rail)
            names.append(n)
        self._store(d, _T("{n} galet(s) sur le rail de {a}.", n=len(names), a=a))

    def _do_driver(self):
        name = self._chosen()
        if self.source != "model":
            m = self.mechanism()
            j = next((j for j in m.joints if j.name == name), m.driver[0])
            m.add_driver(j, self._from_ui(j.kind, self.drv_lo.value()),
                         self._from_ui(j.kind, self.drv_hi.value()))
            self.frames = None
            self.report = None
            self._fill_tree()
            self.say(_T("Moteur : {j} ({n} moteur(s)).", j=j.name, n=len(m.drivers)))
            return
        j = self._joint_record(name)
        if j is None:
            if len(self.data["joints"]) == 1:
                j = self.data["joints"][0]
                self._show_travel(j["kind"], None, j)
            else:
                raise ValueError(_T("choisir une liaison dans la liste"))
        d = doc.add_driver(self.data, j["name"], self._from_ui(j["kind"], self.drv_lo.value()),
                           self._from_ui(j["kind"], self.drv_hi.value()))
        self._store(d, _T("Moteur : {j} ({n} moteur(s)).", j=j['name'], n=len(d['drivers'])))

    def _apply_joint(self):
        j = self._joint_record(self._chosen())
        if j is None or self.source != "model":
            return
        d = doc.normalized(self.data)
        rec = next(k for k in d["joints"] if k["name"] == j["name"])
        rec["a"], rec["b"] = self.combo_a.currentText(), self.combo_b.currentText()
        rec["limits"] = ([self._from_ui(j["kind"], self.lim_lo.value()),
                          self._from_ui(j["kind"], self.lim_hi.value())]
                         if self.lim_on.isChecked() else None)
        if self.motor_on.isChecked():
            d = doc.add_driver(d, j["name"], self._from_ui(j["kind"], self.drv_lo.value()),
                               self._from_ui(j["kind"], self.drv_hi.value()))
        else:
            d = doc.remove_driver(d, j["name"])
        k, c = self.k_spin.value(), self.c_spin.value()
        if j["kind"] == "elastic":
            d = doc.set_joint_params(d, j["name"], k_lin=k or 1e5, k_rot=(k or 1e5) * 0.01,
                                     c_lin=c, c_rot=c * 0.01)
        elif j["kind"] in ("pivot", "slider", "cylindrical", "ball", "screw"):
            d = doc.set_joint_params(d, j["name"], spring=[k, 0.0] if k else None,
                                     damper=c or None)
        d = doc.set_torque(d, j["name"], self.tq_spin.value() if self.tq_on.isChecked() else None)
        self._store(d, _T("{name} mis à jour.", name=j['name']))

    def _delete(self):
        name = self._chosen()
        if name and self.source == "model":
            self._store(doc.remove(self.data, name), _T("{name} supprimé.", name=name))

    def simulation(self):
        """The example's dynamic simulation (weight, inertia; no motor
        needed), as poses at 30 per second; None for a kinematic one."""
        sim = (self.sample_info or {}).get("simulation")
        if not sim and self.source == "model":
            ms = getattr(self.mechanism(), "simulation", {}) or {}
            if ms.get("active"):
                m = self.mechanism()
                T = float(ms.get("duree", 4.0))
                motors = {jj.name: ("vitesse", (lambda t, lo=lo, hi=hi:
                                                 lo + (hi - lo) * min(max(t / T, 0.0), 1.0)))
                          for jj, lo, hi in m.drivers}
                for name, v in (getattr(m, "torques", {}) or {}).items():
                    motors[name] = ("couple", (lambda t, v=v: v))
                sim = dict(duree=T, dt=0.002, params=dict(motors=motors))
                if ms.get("chocs"):            # stiff contacts: a finer step
                    # semi-implicit Euler at 0.1 ms: one evaluation a step,
                    # and it follows a much finer reference better than
                    # Runge-Kutta at 0.2 ms did (Newton's cradle, 2nd shock)
                    sim["dt"] = 1e-4
                    sim["method"] = "symplectic"
                    sim["params"]["contacts"] = dict(dt=1e-4)
        if not sim:
            return None
        if getattr(self, "_sim_running", False):
            return None                        # asked again while it runs
        if getattr(self, "_sim_cache", (None,))[0] is not self.mech:
            from . import dynamics
            m = self.mechanism()
            dyn = dynamics.Dynamics(m, **sim.get("params", {}))
            dyn.method = sim.get("method", "rk4")
            dt = sim.get("dt", 0.002)
            every = max(1, int(round(1.0 / (30 * dt))))
            steps = int(round(sim["duree"] / dt))
            prog = self._sim_progress(steps)
            self._sim_running = True
            try:
                rows = dyn.run(sim["duree"], dt=dt, every=every, progress=prog)
            finally:
                self._sim_running = False
                if prog is not None:
                    prog.close()
            if rows is None:                   # cancelled: not again until asked
                self._sim_cache = (self.mech, None)
                self.say(_T("Simulation interrompue : ▶ ou « Valider » la relance."))
                return None
            n = len(rows)
            motor_rows = [nm for nm, (kind, _f) in dyn.motors.items() if kind == "vitesse"]

            def torques(r):
                if "lam" not in r:
                    return {}
                rx = dyn.reactions(r["lam"])
                return {nm: float(rx[nm + " (moteur)"][0]) for nm in motor_rows
                        if nm + " (moteur)" in rx}
            poses = [dict(t=i / max(n - 1, 1), value=r["t"], x=r["x"], ok=True,
                          res=float(np.linalg.norm(dyn.C(r["x"], r["t"]))),
                          ke=r["ke"], pe=r["pe"], couples=torques(r))
                     for i, r in enumerate(rows)]
            if len(poses) > 1 and not poses[-1]["couples"]:
                poses[-1]["couples"] = poses[-2]["couples"]
            self._sim_cache = (self.mech, poses)
            self._sim_assumed = list(dyn.assumed)
            self._sim_impacts = (dyn.impacts.deepest if dyn.impacts is not None else None)
        return self._sim_cache[1]

    def has_simulation(self):
        """A dynamic simulation is set (without running it)."""
        if (self.sample_info or {}).get("simulation"):
            return True
        if self.source != "model":
            return False
        return bool((getattr(self.mechanism(), "simulation", {}) or {}).get("active"))

    def rerun_simulation(self):
        """Forget a cancelled run, so the next request computes again."""
        if getattr(self, "_sim_cache", (None, 0))[1] is None:
            self._sim_cache = (None, None)

    def _busy(self, label):
        """A progress window (shown after 0.4 s) whose callback ``(i, n)``
        keeps the window answering; None when the panel is not shown."""
        if not self.isVisible():
            return None
        from PySide6.QtWidgets import QProgressDialog
        dlg = QProgressDialog(label, _T("Annuler"), 0, 100, self)
        dlg.setWindowModality(Qt.WindowModal)
        dlg.setMinimumDuration(400)

        def step(i, n):
            dlg.setMaximum(max(int(n), 1))
            dlg.setValue(int(i))
            QApplication.processEvents()
            return not dlg.wasCanceled()
        step.close = dlg.close
        return step

    def _sim_progress(self, steps):
        """A progress window for a long run (shocks: thousands of steps),
        so the window keeps answering; None for a short one or headless."""
        if steps < 2000 or not self.isVisible():
            return None
        from PySide6.QtWidgets import QProgressDialog
        dlg = QProgressDialog(_T("Simulation dynamique…"), _T("Annuler"), 0, steps, self)
        dlg.setWindowModality(Qt.WindowModal)
        dlg.setMinimumDuration(400)

        def step(i, n):
            dlg.setValue(i)
            QApplication.processEvents()
            return not dlg.wasCanceled()
        step.close = dlg.close
        return step

    def _chocs(self):
        sim = (self.sample_info or {}).get("simulation")
        if sim:
            return bool(sim.get("params", {}).get("contacts"))
        return bool((getattr(self.mechanism(), "simulation", {}) or {}).get("chocs"))

    def _do_validate(self):
        m = self.mechanism()
        self.rerun_simulation()
        poses = self.simulation()
        if m.driver is None and poses is None:
            raise ValueError(_T("pas de moteur"))
        if self.source == "model" and (
                abs(self.data.get("jeu_min", 0.005) - self.jeu.value() / 1000) > 1e-12
                or int(self.data.get("pas", 51)) != self.pas.value()):
            # the gap and the steps belong to the document: stored (one undo
            # step), so they come back with the file
            self._store(doc.normalized(self.data))
            m = self.mechanism()
            poses = self.simulation()
        prog = self._busy(_T("Validation…"))
        try:
            rep = collide.validate(m, self.pas.value(), self.jeu.value() / 1000,
                                   poses=poses, contacts=poses is not None and self._chocs(),
                                   progress=prog)
        finally:
            if prog is not None:
                prog.close()
        if rep is None:
            self.say(_T("Validation interrompue."))
            return
        self.report = rep
        self.frames = self.report["steps"]
        self._write_report()
        if self.buttons["trajectory"].isChecked():
            self._compute_traces()
        self._scrub(self.slider.value())

    def _write_report(self):
        rep, lines = self.report, []
        lines += [f"⚠ {p}" for p in self.problems]
        w = rep["worst"]
        if rep["ok"]:
            lines.append(_T("✔ Mécanisme valide de 0 à 100 %."))
        else:
            lines.append(_T("✖ {n} problème(s) :", n=len(rep['problems'])))
            lines += ["  " + p for p in rep["problems"][:60]]
            if len(rep["problems"]) > 60:
                lines.append("  …")
        if w is not None:
            lines.append(_T("Jeu le plus faible : {d} mm entre {a} ({ga}) et {b} ({gb}) à {p} %.",
                           d=f"{w['distance'] * 1000:.1f}", a=w['a'], ga=w['box_a'],
                           b=w['b'], gb=w['box_b'], p=round(100 * w['t'])))
        # the most frequent count over a few poses: a single pose can mislead
        # (a roller at the very end of its rail reads 0; after whole turns of
        # a crank a part's rotation parameters reach 2π and it reads 5 or 7
        # on the rail car instead of 1)
        m, F = self.mechanism(), self.frames
        counts = [solver.dof(m, F[round(f * (len(F) - 1))]["x"])
                  for f in (0.1, 0.3, 0.5, 0.7, 0.9)]
        dof = max(set(counts), key=lambda c: (counts.count(c), -c))
        lines.append(_T("Degrés de liberté (sans moteur) : {n}", n=dof))
        st = rep["steps"]
        if st and "ke" in st[0]:
            e0, e1 = st[0]["ke"] + st[0]["pe"], st[-1]["ke"] + st[-1]["pe"]
            lines.append(_T("Simulation dynamique de {t} s : énergie {e0} → {e1} mJ.",
                           t=f"{st[-1]['value']:.1f}", e0=f"{1000 * e0:.0f}", e1=f"{1000 * e1:.0f}"))
            names = sorted({k for r in st for k in (r.get("couples") or {})})
            for nm in names:
                vals = [r["couples"][nm] for r in st if nm in (r.get("couples") or {})]
                k = int(np.argmax(np.abs(vals)))
                lines.append(_T("Couple moteur {nm} : maxi {mx} N·m (à {t} s), moyen {mean} N·m.",
                               nm=nm, mx=f"{abs(vals[k]):.3g}", t=f"{st[k]['value']:.2f}",
                               mean=f"{np.mean(np.abs(vals)):.3g}"))
            on_stop = {}
            for r in st:
                for name, v, lim in r.get("limits") or []:
                    on_stop.setdefault(name, r["value"])
            for name, t0 in on_stop.items():
                lines.append(_T("{name} : en butée à partir de {t} s.", name=name, t=f"{t0:.2f}"))
            deep = getattr(self, "_sim_impacts", None)
            if deep is not None and self._chocs():
                lines.append(_T("Chocs entre pièces : enfoncement maxi {d} mm (pièces indéformables).",
                               d=f"{deep * 1000:.2f}"))
            assumed = getattr(self, "_sim_assumed", [])
            if assumed:
                lines.append(_T("ⓘ sans matériau, masse estimée (boîtes, 700 kg/m³) : {names}.",
                               names=", ".join(assumed)))
        ms = getattr(m, "masses", {}) or {}
        if ms:
            known = {n: mp for n, mp in ms.items() if mp["mass"] > 0}
            if known:
                tot = sum(mp["mass"] for mp in known.values())
                lines.append(_T("Masse totale : {m} kg — ", m=f"{tot:.3g}")
                             + ", ".join(f"{n} {mp['mass']:.3g} kg" for n, mp in known.items()))
            miss = sum(len(mp["missing"]) for mp in ms.values())
            if miss:
                lines.append(_T("ⓘ {n} groupe(s) sans matériau : masse non comptée.", n=miss))
        self.out.setPlainText("\n".join(lines))

    def _do_trajectory(self):
        if not self.buttons["trajectory"].isChecked():
            self.traces, self.trace_pts = [], []
            return
        m = self.mechanism()
        _, segs = selection(self._scene())
        self.traces = []
        moving = {b.name: b for b in m.bodies}
        if segs and self.source == "model":
            boxes = {k: v for k, v in self._boxes().items() if k in moving}
            for p0, p1 in segs:
                for p in (p0, p1):
                    b = doc.nearest_body(p, boxes)
                    if b:
                        self.traces.append((moving[b], p))
        elif (getattr(self, "sample_info", None) or {}).get("traces"):
            self.traces = [(b, np.asarray(p, float)) for b, p in self.sample_info["traces"]]
        else:
            for j in m.joints:
                if j.kind == "coupling":
                    continue
                for body in (j.a, j.b):
                    if not body.fixed:
                        self.traces.append((body, np.asarray(j.point, float)))
                        break
        self._compute_traces()
        self.say(_T("Trajectoire de {n} point(s).", n=len(self.traces)))

    def _compute_traces(self):
        m = self.mechanism()
        if self.frames is None:
            self.frames = solver.sweep(m, self.pas.value())
        self.trace_pts = [doc.trajectory(m, self.frames, b, p) for b, p in self.traces]

    # ---- playback -------------------------------------------------------------------
    def _source_changed(self):
        self.source = self.src.currentData()
        self.timer.stop()
        self.play_btn.setChecked(False)
        self.traces, self.trace_pts = [], []
        self.buttons["trajectory"].setChecked(False)
        self.out.clear()
        self.reload()
        jeu = (self.sample_info or {}).get("jeu_min")     # an example's own gap
        if jeu is not None:
            self.jeu.setValue(1000 * jeu)
        self.slider.setValue(0)
        self._scrub(0)

    # ---- copy an example into the document -----------------------------------------
    def copy_example(self, confirm=None):
        """The current example as real groups and the document's mechanism
        (two undo steps: the groups, then the mechanism). ``confirm``: the
        answer to « replace the mechanism already in the document? »
        (None: ask)."""
        from . import excopy
        builders = {"chaise": samples.chaise_pliante, "porte": samples.porte_sectionnelle,
                    "oiseau": samples.oiseau, "pendule_double": samples.pendule_double}
        key = self.source
        if key not in builders:
            return None
        current = doc.normalized(self.app.document_data(default=None))
        if current["bodies"]:
            if confirm is None:
                from PySide6.QtWidgets import QMessageBox
                confirm = QMessageBox.question(
                    self, _T("Copier dans le modèle"),
                    _T("Le document a déjà un mécanisme. Le remplacer par celui de l'exemple ? "
                       "(Ses groupes restent dans le modèle.)")) == QMessageBox.Yes
            if not confirm:
                return None
        m, info = builders[key]()
        made = _make_groups(excopy.parts(m, info))
        from core.history import SnapshotImport
        vp = self.app.viewport

        def mutate(scene):
            scene.groups.extend(g for _, g in made)

        vp.history.execute(SnapshotImport(mutate))
        uids = {}
        for body, g in made:
            uids.setdefault(body, []).append(g.uid)
        data = excopy.data(m, info, uids)
        self.src.blockSignals(True)
        self.src.setCurrentIndex(0)                 # back to the document
        self.src.blockSignals(False)
        self.source = "model"
        self.traces, self.trace_pts = [], []
        self.app.set_document_data(data)            # one undo step; reload follows
        notify = getattr(vp, "notify_scene_changed", None)
        if callable(notify):
            notify()
        self.reload()
        vp.update()
        self.say(_T("Exemple copié : {n} pièce(s), {j} liaison(s) — modifiables.",
                    n=len(data["bodies"]), j=len(data["joints"])))
        return data

    # ---- STEP import -----------------------------------------------------------------
    def import_step(self, path, tol=0.0002):
        """Add the parts of a STEP assembly to the model; they are left
        selected, ready to become fixed or moving parts."""
        from . import stepimport
        asm, groups = stepimport.import_into(self.app, path, tol)
        sel = getattr(self._scene(), "selection", None)
        if sel is not None:
            sel.clear()
            sel.update(groups)
        warn = "; ".join(f"{k} ×{n}" for k, n in asm.warnings.items())
        self.say(_T("STEP : {n} pièce(s) importée(s)", n=len(groups))
                 + (_T(" — non lu : {w}", w=warn) if warn else "") + ".")
        return asm, groups

    # ---- automatic joint detection ------------------------------------------------
    def _scan_groups(self):
        sc = self._scene()
        sel = [g for g in (getattr(sc, "selection", None) or ())
               if hasattr(g, "mesh") and hasattr(g, "uid")]
        groups = sel if len(sel) >= 2 else [g for g in getattr(sc, "groups", [])
                                            if not getattr(g, "hidden", False)
                                            and not getattr(g, "billboard", False)]
        return groups

    @staticmethod
    def _detect_part(g):
        """detect.Part of an IngeTrazo group: its faces, joined through soft
        edges into smooth regions."""
        from . import detect
        try:
            from core.group import world_mesh
            mesh = world_mesh(g)
        except Exception:
            mesh = g.mesh
        faces, index = [], {}
        vol = 0.0
        for f in mesh.faces:
            P = np.array([(v.x(), v.y(), v.z()) for v in f.vertices], float)
            if len(P) < 3:
                continue
            index[id(f)] = len(faces)
            faces.append(P)
            n, a = detect.polygon_normal_area(P)
            for H in getattr(f, "holes", []) or []:
                H = np.array([(v.x(), v.y(), v.z()) for v in H], float)
                if len(H) >= 3:
                    a -= detect.polygon_normal_area(H)[1]
            vol += float(n @ P.mean(axis=0)) * a / 3.0
        soft = []
        for e in mesh.edges:
            fs = getattr(e, "faces", ())
            if getattr(e, "soft", False) and len(fs) == 2:
                i, j = index.get(id(fs[0])), index.get(id(fs[1]))
                if i is not None and j is not None:
                    soft.append((i, j))
        regions = detect.regions_from_faces(faces, soft)
        return detect.make_part(g.uid, str(getattr(g, "name", "pièce")), regions, vol)

    def scan_model(self, groups=None):
        """(parts {uid: (name, [uid])}, proposals, frame uid) for ``groups``
        (default: the selection, else every visible group)."""
        from . import detect
        groups = self._scan_groups() if groups is None else list(groups)
        parts = [self._detect_part(g) for g in groups]
        props, frame = detect.scan(parts)
        names = {p.key: (p.name, [p.key]) for p in parts}
        return names, props, frame

    def apply_scan(self, parts, frame, proposals):
        """Bodies and joints for the ticked ``proposals`` (one undo step)."""
        if self.source != "model":
            self.set_source("model")
        d, made = doc.apply_detection(self.data, parts, frame, proposals)
        d = self._guess_materials(d, [u for b in d["bodies"] for u in b["groups"]])
        n = {k: sum(1 for x in proposals if x.kind == k) for k in ("pivot", "slider", "gears")}
        msg = _T("Détection : {p} pivot(s), {s} glissière(s), {g} engrenage(s) ; pièce fixe : {f}.",
                p=n['pivot'], s=n['slider'], g=n['gears'], f=parts[frame][0])
        if not d["drivers"]:
            msg += _T(" Choisir un moteur dans la liste.")
        self._store(d, msg)
        return made

    def _scan_dialog(self):
        from PySide6.QtWidgets import (QApplication, QDialog, QDialogButtonBox,
                                       QMessageBox, QTreeWidgetItem)
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            parts, props, frame = self.scan_model()
        except Exception as exc:
            QApplication.restoreOverrideCursor()
            QMessageBox.warning(self, _T("Détecter les liaisons"), f"{type(exc).__name__} : {exc}")
            return
        QApplication.restoreOverrideCursor()
        if not props:
            QMessageBox.information(self, _T("Détecter les liaisons"),
                                    _T("{n} pièce(s) analysée(s) : aucune liaison reconnue "
                                      "(arbre dans un alésage, appuis plans, engrenage).",
                                      n=len(parts)))
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(_T("Liaisons proposées"))
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel(_T("{n} pièce(s) analysée(s). Cocher les liaisons à créer :",
                               n=len(parts))))
        fr = QHBoxLayout()
        fr.addWidget(QLabel(_T("Pièce fixe")))
        combo = QComboBox()
        keys = sorted(parts, key=lambda k: parts[k][0])
        for k in keys:
            combo.addItem(parts[k][0], k)
        combo.setCurrentIndex(keys.index(frame))
        fr.addWidget(combo, 1)
        lay.addLayout(fr)
        tree = QTreeWidget()
        tree.setHeaderLabels([_T("Liaison"), _T("Pièces"), _T("Détail"), _T("Confiance")])
        tree.setRootIsDecorated(False)
        label = {"pivot": _T("Pivot"), "slider": _T("Glissière"), "gears": _T("Engrenage")}
        icon = {"pivot": "pivot", "slider": "slider", "gears": "coupling"}
        items = []
        for x in props:
            it = QTreeWidgetItem([label[x.kind], f"{parts[x.a][0]} ↔ {parts[x.b][0]}",
                                  _detail(x.detail), f"{round(100 * x.score)} %"])
            it.setIcon(0, icons.icon(icon[x.kind], tree, 16))
            it.setCheckState(0, Qt.Checked if x.score >= 0.5 else Qt.Unchecked)
            tree.addTopLevelItem(it)
            items.append((it, x))
        for c in range(4):
            tree.resizeColumnToContents(c)
        lay.addWidget(tree, 1)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Ok).setText(_T("Créer les liaisons"))
        bb.button(QDialogButtonBox.Cancel).setText(_T("Annuler"))
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        dlg.resize(720, 420)
        if dlg.exec() != QDialog.Accepted:
            return
        chosen = [x for it, x in items if it.checkState(0) == Qt.Checked]
        if chosen:
            self.apply_scan(parts, combo.currentData(), chosen)

    def _import_step_dialog(self):
        from PySide6.QtWidgets import QApplication, QFileDialog, QMessageBox
        path, _ = QFileDialog.getOpenFileName(self, _T("Importer un assemblage STEP"), "",
                                              "STEP (*.step *.stp *.STEP *.STP)")
        if not path:
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.import_step(path)
        except Exception as exc:
            QMessageBox.warning(self, _T("Importer STEP"), f"{type(exc).__name__} : {exc}")
        finally:
            QApplication.restoreOverrideCursor()

    def set_source(self, key):
        i = [k for k, _ in SOURCES].index(key)
        self.src.setCurrentIndex(i)

    def _scrub(self, v):
        self.t = v / 1000.0
        self.pct.setText(f"{round(100 * self.t)} %")
        try:
            self._pose_at(self.t)
            self._dynamics(v)
        except Exception as exc:
            self.say(_T("Lecture : {e}", e=exc))
        self.app.viewport.update()

    def _dynamics(self, v):
        """Chains with dynamics: while playing, they go on in time (one
        timer tick); a jump of the slider starts them again at rest."""
        m = self.mechanism()
        dyn = [f for f in getattr(m, "followers", []) if getattr(f, "dynamic", False)]
        if not dyn or self.x is None:
            return
        last = getattr(self, "_last_v", None)
        going = self.timer.isActive() and last is not None and abs(v - last) <= 20
        self._last_v = v
        for f in dyn:
            self.x = f.advance(m, self.x, self.timer.interval() / 1000.0 if going else 0.0)

    def _pose_at(self, t):
        m = self.mechanism()
        if self.frames is None and self.simulation() is not None:
            self.frames = self.simulation()
        if m.driver is None and self.frames:
            k = int(round(t * (len(self.frames) - 1)))
            self.x = self.frames[k]["x"]
            return
        if m.driver is None:
            self.x = np.zeros(m.n)
            return
        if self.frames:
            k = int(round(t * (len(self.frames) - 1)))
            self.x = self.frames[k]["x"]
            return
        j, lo, hi = m.driver
        x, ok, _ = solver.solve(m, lo + t * (hi - lo), self.x)
        self.x = x
        if not ok:
            self.say(_T("{p} % : position non atteinte.", p=round(100 * t)))

    def _play(self, on):
        self.play_btn.setText("⏸" if on else "▶")
        if on:
            self.rerun_simulation()
            m = self.mechanism()
            if m.driver is not None and self.frames is None:
                self.frames = solver.sweep(m, self.pas.value())
            self._dir = 1
            self.timer.start()
        else:
            self.timer.stop()

    def _tick(self):
        if self.full_turn() or (self.mechanism().driver is not None
                                and self.mechanism().multi_turn()):   # a crank keeps turning
            self.slider.setValue((self.slider.value() + 10) % 1000)
            return
        v = self.slider.value() + self._dir * 10
        if v > 1000 or v < 0:
            if self.loop.isChecked():
                self._dir = -self._dir
                v = max(0, min(1000, v))
            else:
                v = 0
        self.slider.setValue(v)

    # ---- drawing over the viewport ----------------------------------------------------
    def paint(self, viewport, painter, wire=True):
        m = self.mech
        if m is None or not (self.isVisible() or getattr(self, "_force_paint", False)):
            return
        x = self.x if self.x is not None else np.zeros(m.n)
        acc = icons.accent()
        hits = set()
        if self.frames:
            k = int(round(self.t * (len(self.frames) - 1)))
            for a, b in self.frames[k].get("hits", []):
                hits.update((a, b))
        bodies = [m.ground] + m.bodies if wire else []
        if self.source == "model" and not np.any(x):
            bodies = []                          # drawn pose: the model shows the parts
        for body in bodies:
            if body.fixed and self.source == "model":
                continue                         # the model shows the frame
            q = m.pose(x, body)
            color = QColor(220, 40, 40) if body.name in hits else (
                QColor(140, 140, 140) if body.fixed else acc)
            segs = self.body_segments.get(body.name)
            if segs is not None and len(segs):
                if body.fixed:
                    continue                     # the model already shows it
                pts = segs.reshape(-1, 3)
            else:
                pts = _box_segments(body, q)
                if not len(pts):
                    continue
                self._draw_segments(painter, pts, color,
                                    3.0 if body.name in getattr(self, "_hl_bodies", ()) else 1.4)
                continue
            R = body.rot(q)
            w = body.center + q[:3] + (pts - body.center) @ R.T
            self._draw_segments(painter, w, color,
                                3.0 if body.name in getattr(self, "_hl_bodies", ()) else 1.6)
        # joints
        pts = []
        for j in m.joints:
            if j.kind == "coupling":
                continue
            body = j.b if not j.b.fixed else j.a
            pts.append(body.world_point(m.pose(x, body), j.point))
        if pts:
            px, py, ok = self.app.world_to_pixels(np.array(pts))
            painter.setPen(QPen(acc, 1.6))
            painter.setBrush(QColor(255, 255, 255))
            for X, Y, o in zip(px, py, ok):
                if o:
                    painter.drawEllipse(QPointF(X, Y), 3.5, 3.5)
        # the joint picked in the list: a ring where it is
        hp = getattr(self, "_hl_point", None)
        if hp is not None:
            jn = self._chosen()
            mj = next((j for j in m.joints if j.name == jn and j.kind != "coupling"), None)
            if mj is not None:
                body = mj.b if not mj.b.fixed else mj.a
                hp = body.world_point(m.pose(x, body), mj.point)
            px, py, ok = self.app.world_to_pixels(np.array([hp]))
            if ok[0]:
                painter.setBrush(Qt.NoBrush)
                painter.setPen(QPen(acc, 3.0))
                painter.drawEllipse(QPointF(px[0], py[0]), 11, 11)
                painter.setPen(QPen(acc, 1.5))
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    painter.drawLine(QPointF(px[0] + 14 * dx, py[0] + 14 * dy),
                                     QPointF(px[0] + 20 * dx, py[0] + 20 * dy))
        # centres of gravity
        cg = [(b, b.world_point(m.pose(x, b), b.com)) for b in m.bodies
              if getattr(b, "com", None) is not None]
        if cg:
            px, py, ok = self.app.world_to_pixels(np.array([w for _, w in cg]))
            for X, Y, o in zip(px, py, ok):
                if o:
                    _draw_cog(painter, QPointF(X, Y), 6.0)
        # trajectories
        pen = QPen(acc, 1.4, Qt.DashLine)
        for tr in self.trace_pts:
            if len(tr) < 2:
                continue
            px, py, ok = self.app.world_to_pixels(tr)
            painter.setPen(pen)
            for i in range(len(tr) - 1):
                if ok[i] and ok[i + 1]:
                    painter.drawLine(QPointF(px[i], py[i]), QPointF(px[i + 1], py[i + 1]))

    # ---- video export ------------------------------------------------------------------
    def _settings(self):
        from PySide6.QtCore import QSettings
        return QSettings("IngeTrazo", "motion")

    def _ffmpeg_hint(self):
        try:
            v = self._settings().value("ffmpeg", "")
            return str(v) if v else None
        except Exception:
            return None

    def _export_dialog(self):
        from PySide6.QtWidgets import (QDialog, QDialogButtonBox, QFileDialog,
                                       QMessageBox, QProgressDialog)
        from . import video
        m = self.mechanism()
        sim = self.simulation()
        if m.driver is None and sim is None:
            self.say(_T("Exporter la vidéo : pas de moteur."))
            return
        dlg = QDialog(self)
        dlg.setWindowTitle(_T("Exporter la vidéo"))
        form = QFormLayout(dlg)
        dur = QDoubleSpinBox()
        dur.setRange(2, 120)
        dur.setValue(8)
        dur.setSuffix(" s")
        form.addRow(_T("Durée"), dur)
        res = QComboBox()
        for text, wh in (("1920 × 1080", (1920, 1080)), ("1280 × 720", (1280, 720))):
            res.addItem(text, wh)
        form.addRow(_T("Résolution"), res)
        back = QCheckBox(_T("Aller-retour (0 → 100 → 0 %)"))
        back.setChecked(True)
        form.addRow(back)
        slow = QDoubleSpinBox()
        slow.setRange(1.0, 20.0)
        slow.setDecimals(1)
        slow.setPrefix("× ")
        slow.setValue(1.0)
        slow.setToolTip(_T("Simulation dynamique : la vidéo la montre ce nombre de fois "
                           "plus lentement (1 : en temps réel)."))
        if sim is not None:                # the simulation sets the length
            form.addRow(_T("Ralenti"), slow)
            back.setVisible(False)
            form.setRowVisible(dur, False)
        enc = QLabel()
        form.addRow("Format", enc)
        pick = QToolButton()
        pick.setText(_T("Indiquer ffmpeg.exe…"))
        form.addRow(pick)

        def refresh():
            kind, exe = video.mp4_backend(self._ffmpeg_hint())
            enc.setText({"qt": "MP4 (Qt Multimedia)", "ffmpeg": f"MP4 (ffmpeg : {exe})"}.get(
                kind, _T("AVI Motion-JPEG — pour du MP4, indiquer ffmpeg.exe")))
            pick.setVisible(kind != "qt")

        def choose():
            f, _ = QFileDialog.getOpenFileName(dlg, "ffmpeg", "", "ffmpeg (ffmpeg*)")
            if f:
                self._settings().setValue("ffmpeg", f)
            refresh()

        pick.clicked.connect(choose)
        refresh()
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QDialog.Accepted:
            return
        kind, _ = video.mp4_backend(self._ffmpeg_hint())
        ext, filt = (".mp4", _T("Vidéo MP4 (*.mp4)")) if kind else (".avi", _T("Vidéo AVI (*.avi)"))
        path, _ = QFileDialog.getSaveFileName(self, _T("Exporter la vidéo"), "motion" + ext, filt)
        if not path:
            return
        prog = QProgressDialog(_T("Rendu de la vidéo…"), _T("Annuler"), 0, 100, self)
        prog.setWindowModality(Qt.WindowModal)
        prog.setMinimumDuration(0)

        def step(i, n):
            prog.setMaximum(n)
            prog.setValue(i)
            return not prog.wasCanceled()

        try:
            out, label = self.export_video(path, dur.value(), res.currentData(),
                                           back.isChecked(), progress=step,
                                           ralenti=slow.value())
        except Exception as exc:
            prog.close()
            QMessageBox.warning(self, _T("Exporter la vidéo"), str(exc))
            return
        prog.close()
        self.say(_T("Vidéo enregistrée : {out} — {label}.", out=out, label=label) if out
                 else _T("Export annulé."))

    def _moving_groups(self):
        """IngeTrazo groups of the moving bodies (model source)."""
        if self.source != "model":
            return []
        by = groups_by_uid(self._scene())
        return [by[u] for b in self.data["bodies"] if not b["fixed"]
                for u in b["groups"] if u in by]

    def _refresh_view_cache(self, hidden=None):
        # Best effort, viewport internals: forget the memoised placement
        # epoch so a group's ``hidden`` flag is seen. The draw batches key on
        # the scene version too (IngeTrazo 0.5.9): while the moving parts
        # are hidden the version takes a value of its own, never a real one
        # (negative), and comes back after — no undo step, the document
        # stays saved. If these names change, moving parts simply stay
        # drawn in place.
        vp = self.app.viewport
        for name in ("_epoch_memo", "_epoch_same"):
            if hasattr(vp, name):
                try:
                    setattr(vp, name, None)
                except Exception:
                    pass
        sc = getattr(vp, "scene", None)
        if sc is None or not isinstance(getattr(sc, "version", None), int):
            return
        if hidden:
            self._real_version = sc.version
            self._hide_count = getattr(self, "_hide_count", 0) + 1
            sc.version = -self._hide_count
        elif hidden is False and getattr(self, "_real_version", None) is not None:
            sc.version = self._real_version
            self._real_version = None

    def _viewport_image(self, width, shaded=None, x=None, hits=None):
        """The IngeTrazo view as an image: IngeTrazo renders the model
        (moving parts taken out for the shot), Motion draws the moving
        parts at pose ``x`` on top, then the joints and trajectories."""
        vp = self.app.viewport
        img = None
        render = getattr(vp, "render_image", None)
        hide = [g for g in self._moving_groups() if not g.hidden] if shaded is not None else []
        if callable(render):
            self._quiet = getattr(self, "_quiet", 0) + 1
            try:
                for g in hide:
                    g.hidden = True
                if hide:
                    self._refresh_view_cache(True)
                img = render(width)
            except Exception:
                img = None
            finally:
                for g in hide:
                    g.hidden = False
                if hide:
                    self._refresh_view_cache(False)
                self._quiet -= 1
        lw, lh = max(vp.width(), 1), max(vp.height(), 1)
        rendered = img is not None and not img.isNull()
        if not rendered:                                 # no GL here
            from PySide6.QtGui import QImage
            img = QImage(width, max(1, int(round(width * lh / lw))), QImage.Format_RGB32)
            img.fill(QColor(236, 236, 234))
        from PySide6.QtGui import QPainter
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.scale(img.width() / lw, img.height() / lh)
        self._force_paint = True
        try:
            if shaded is not None and x is not None:
                from . import render as _render
                cam = getattr(vp, "camera", None)
                yaw = float(getattr(cam, "yaw", 0.0))
                pitch = float(getattr(cam, "pitch", 0.0))
                fwd = _render.view_axes(yaw, pitch)[2]
                names = None
                if self.source == "model" and rendered:
                    names = {b.name for b in self.mechanism().bodies}
                if hits is None:
                    hits = {n for pair in collide.interferences(self.mechanism(), x)
                            for n in pair}
                _render.draw_projected(shaded, p, x, self.app.world_to_pixels, fwd,
                                       names, hits)
                self.paint(vp, p, wire=False)
            else:
                self.paint(vp, p)
        finally:
            self._force_paint = False
            p.end()
        return img

    def _polys(self):
        """Face polygons of each body's groups (reference pose)."""
        if self.source != "model":
            return {}
        by = groups_by_uid(self._scene())
        out = {}
        for b in self.data["bodies"]:
            faces = []
            for u in b["groups"]:
                g = by.get(u)
                if g is None:
                    continue
                for mesh, aff in _placements(g):
                    if not mesh.vertices:
                        continue
                    V = _positions(mesh.vertices)            # read once
                    if aff is not None:
                        V = V @ aff[0].T + aff[1]
                    index = {id(v): i for i, v in enumerate(mesh.vertices)}
                    for f in mesh.faces:
                        try:
                            faces.append(V[[index[id(v)] for v in f.loop]])
                        except KeyError:
                            P = np.array([_xyz(v.position) for v in f.loop])
                            faces.append(P if aff is None else P @ aff[0].T + aff[1])
            if faces:
                out[b["name"]] = faces
        return out

    def full_turn(self):
        """Driver is a pivot doing a whole revolution (a crank)."""
        m = self.mechanism()
        if m.driver is None or self.has_simulation():
            return False
        j, lo, hi = m.driver
        return j.kind == "pivot" and abs(abs(hi - lo) - 2 * math.pi) < 1e-6

    def export_video(self, path, seconds=8.0, size=(1920, 1080), aller_retour=True,
                     fps=30, progress=None, cycles=None, ralenti=1.0):
        """Render the motion to ``path`` (split screen). Returns (file, kind
        label), or (None, None) when ``progress(i, n)`` returned False.
        A crank (full-turn driver) runs at constant speed for ``cycles``
        turns (default: one turn per 2 s); anything else is eased, there
        and back when ``aller_retour``. A dynamic simulation plays in real
        time, or ``ralenti`` times slower (its poses interpolated)."""
        from . import render, video
        if self.play_btn.isChecked():          # no playing behind the frames
            self.play_btn.setChecked(False)
        self.timer.stop()
        m = self.mechanism()
        poses = self.simulation()
        if m.driver is None and poses is None:
            raise ValueError(_T("pas de moteur"))
        # checked once, at the validation steps: the readings of a frame
        # (smallest gap, collisions) are those of its nearest step
        rows = self.report["steps"] if self.report else None
        if rows is None or len(rows) < 3:
            prog = self._busy(_T("Validation…"))
            try:
                rep = collide.validate(m, max(self.pas.value(), 21),
                                       self.jeu.value() / 1000, poses=poses,
                                       contacts=poses is not None and self._chocs(),
                                       progress=prog)
            finally:
                if prog is not None:
                    prog.close()
            if rep is None:
                return None, None                 # cancelled
            rows = rep["steps"]
        sweep_rows = rows
        slow = max(float(ralenti or 1.0), 1.0)
        if poses is not None:                     # a simulation plays in real time
            seconds = (poses[-1]["value"] - poses[0]["value"]) * slow
        n = max(2, int(round(seconds * fps)))
        hold = min(int(0.5 * fps), n // 6)
        ease = lambda u: 0.5 - 0.5 * math.cos(math.pi * u)
        allure = (self.sample_info or {}).get("allure")
        if poses is not None:
            tl = [i / (n - 1) for i in range(n)]
        elif allure:                              # an example's own timing
            tl = [allure(i / (n - 1)) for i in range(n)]
        elif m.multi_turn():                      # several turns: once, steadily
            tl = [i / (n - 1) for i in range(n)]
        elif self.full_turn():
            cycles = cycles or max(1, round(seconds / 2.0))
            tl = [(i * cycles / n) % 1.0 for i in range(n)]
        elif aller_retour:
            run = (n - 3 * hold) // 2
            tl = ([0.0] * hold + [ease(i / max(run - 1, 1)) for i in range(run)] + [1.0] * hold
                  + [ease(1 - i / max(run - 1, 1)) for i in range(run)])
        else:
            run = n - 2 * hold
            tl = [0.0] * hold + [ease(i / max(run - 1, 1)) for i in range(run)]
        tl += [tl[-1]] * (n - len(tl))
        cam = getattr(self.app.viewport, "camera", None)
        yaw = float(getattr(cam, "yaw", math.radians(-45)))
        pitch = float(getattr(cam, "pitch", math.radians(25)))
        vue = (self.sample_info or {}).get("vue")         # a view of its own
        if vue:
            yaw, pitch = math.radians(vue[0]), math.radians(vue[1])
        polys = self._polys() or (self.sample_info or {}).get("polys") or {}
        shaded = render.Shaded(m, polys)
        follow = None                                      # camera follows a part
        fname = (self.sample_info or {}).get("suivre")
        if fname:
            follow = next((b for b in m.bodies if b.name == fname), None)
        W, H = size
        u = H / 1080.0
        pane = ((W - 48 * u) / 2 - 8, H - 180 * u - 8)
        scale, centre = shaded.fit([r["x"] for r in sweep_rows], yaw, pitch, *pane,
                                   follow=follow)
        floor = (abs(math.degrees(pitch)) > 12
                 and (self.sample_info or {}).get("sol", True))
        cam_target = getattr(cam, "target", None)
        traces = self.trace_pts if self.buttons["trajectory"].isChecked() else []
        j, lo, hi = m.driver if m.driver is not None else (None, 0.0, 1.0)
        unit = (lambda v: f"{math.degrees(v):.1f}°") if (j is not None and j.kind == "pivot") else \
            (lambda v: f"{1000 * v:.0f} mm")
        if j is not None and m.multi_turn():
            unit = lambda v: _T("{v} tr", v=f"{v / (2 * math.pi):.2f}")
        name = _T(SAMPLE_NAMES.get(self.source, "Mécanisme"))
        writer = video.open_writer(path, W, H, fps, self._ffmpeg_hint())
        saved_x, saved_t = self.x, self.t
        x = None
        dyn = [f for f in getattr(m, "followers", []) if getattr(f, "dynamic", False)]
        for f in dyn:                             # chains computed in time
            f.reset()
        try:
            for i, t in enumerate(tl):
                if progress is not None and not progress(i, len(tl)):
                    writer.close()
                    return None, None
                f = t * (len(sweep_rows) - 1)
                k = min(int(f), len(sweep_rows) - 2)
                a = f - k
                if poses is not None:                 # the simulation's own pose
                    if slow > 1.0:                    # slow motion: between two poses
                        x = (1 - a) * sweep_rows[k]["x"] + a * sweep_rows[k + 1]["x"]
                    else:
                        x = sweep_rows[int(round(f))]["x"]
                    ok = True
                else:
                    guess = (1 - a) * sweep_rows[k]["x"] + a * sweep_rows[k + 1]["x"]
                    x, ok, _ = solver.solve(m, lo + t * (hi - lo), guess)
                for f in dyn:
                    x = f.advance(m, x, 0.0 if i == 0 else 1.0 / fps)
                self.x, self.t = x, t
                near = rows[int(round(t * (len(rows) - 1)))]
                hits = {nm for pair in near.get("hits", []) for nm in pair}
                cl = [near["clearance"]] if near.get("clearance") else []
                shown = lo + t * (hi - lo)
                info = [_T("temps {t} s", t=f"{near['value']:.2f}")] if poses is not None else []
                if poses is not None and slow > 1.0:
                    info.append(_T("ralenti ×{k}", k=f"{slow:g}"))
                for nm, v in (near.get("couples") or {}).items():
                    info.append(_T("couple {nm} {v} N·m", nm=nm, v=f"{v:+.3g}"))
                for (jj, l_, h_), v in zip(m.drivers, m.driver_targets(shown) if m.drivers else []):
                    if jj.kind == "pivot":
                        txt = (_T("{v} tr", v=f"{v / (2 * math.pi):.2f}") if m.multi_turn((jj, l_, h_))
                               else f"{math.degrees(v):.1f}°")
                    else:
                        txt = f"{1000 * v:.0f} mm"
                    info.append(f"{jj.name} : {txt}")
                extra = (self.sample_info or {}).get("mesures")
                if extra:
                    try:
                        info += extra(m, x, near)
                    except TypeError:
                        info += extra(m, x)
                if cl:
                    info.append(_T("jeu mini {d} mm", d=f"{1000 * max(cl[0]['distance'], 0):.1f}"))
                names = sorted(hits)
                if len(names) > 4:                    # a line, not a paragraph
                    names = names[:3] + [_T("+{n} autres", n=len(names) - 3)]
                info.append(_T("collision : ") + " / ".join(names) if hits
                            else _T("aucune collision"))
                if not ok:
                    info.append(_T("position non atteinte"))
                shift = np.zeros(3)
                if follow is not None:
                    shift = follow.world_point(m.pose(x, follow), follow.center) - follow.center
                    if cam_target is not None:
                        try:
                            cam.target = type(cam_target)(cam_target.x() + shift[0],
                                                          cam_target.y() + shift[1],
                                                          cam_target.z() + shift[2])
                        except Exception:
                            pass
                left = self._viewport_image(int(pane[0] * 1.5), shaded, x, hits)
                frame = render.compose(
                    W, H, left,
                    lambda p, r: shaded.draw(p, r, x, yaw, pitch, scale, centre + shift,
                                             hits, traces, floor=floor),
                    f"{name} — IngeTrazo Motion", info, t)
                writer.add(frame)
            if progress is not None:
                progress(len(tl), len(tl))
            out = writer.close()
        finally:
            self.x, self.t = saved_x, saved_t
            if cam_target is not None:
                try:
                    cam.target = cam_target
                except Exception:
                    pass
            self.app.viewport.update()
        return out, writer.label

    def _draw_segments(self, painter, pts, color, width):
        px, py, ok = self.app.world_to_pixels(pts)
        painter.setPen(QPen(color, width))
        px, py, ok = np.asarray(px, float), np.asarray(py, float), np.asarray(ok, bool)
        n = len(pts) // 2 * 2
        keep = ok[0:n:2] & ok[1:n:2]
        if not keep.any():
            return
        from PySide6.QtCore import QLineF
        a = np.stack([px[0:n:2], py[0:n:2], px[1:n:2], py[1:n:2]], axis=1)[keep]
        painter.drawLines([QLineF(*r) for r in a.tolist()])     # one call


def _draw_cog(painter, c, r):
    """The usual centre-of-gravity mark: a circle in quarters, two filled."""
    from PySide6.QtCore import QRectF
    painter.setPen(QPen(QColor(30, 30, 30), 1.2))
    painter.setBrush(QColor(255, 255, 255))
    painter.drawEllipse(c, r, r)
    painter.setBrush(QColor(30, 30, 30))
    rect = QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r)
    painter.drawPie(rect, 0, 90 * 16)
    painter.drawPie(rect, 180 * 16, 90 * 16)
    painter.setBrush(Qt.NoBrush)


_BOX_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7),
              (0, 4), (1, 5), (2, 6), (3, 7)]


def _box_segments(body, q):
    """World end points (pairs) of the wireframe of a body's boxes."""
    out = []
    for bx in body.boxes:
        c, R, h = collide.world_obb(body, q, bx)
        k = collide._corners((c, R, h))
        for i, j in _BOX_EDGES:
            out += [k[i], k[j]]
    return np.array(out) if out else np.zeros((0, 3))


def setup(app):
    panel = MotionPanel(app)
    # Taller than a laptop screen once every row is shown: in a scroll area
    # the panel keeps its rows whole and scrolls, instead of being squeezed
    # (spin boxes cut in half at the bottom of the window).
    from PySide6.QtWidgets import QFrame, QScrollArea
    scroll = QScrollArea()
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QFrame.NoFrame)
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    scroll.setWidget(panel)
    # room for the scroll bar: the panel never gets narrower than its rows
    scroll.setMinimumWidth(panel.minimumSizeHint().width()
                           + scroll.verticalScrollBar().sizeHint().width() + 2)
    dock = app.add_panel("Motion", scroll, start_hidden=False)
    app.add_overlay(panel.paint)
    app.on_document_changed(panel.reload)

    def show(source=None):
        if source:
            panel.set_source(source)
        app.show_panel(dock)
        panel.mechanism()
        panel._scrub(panel.slider.value())

    sub = app.add_menu("Motion")
    if sub is not None:
        sub.addAction(_T("Panneau Motion"), lambda: show())
        sub.addAction(_T("Importer un assemblage STEP…"),
                      lambda: (show(), panel._import_step_dialog()))
        for key, text in SOURCES[1:]:
            sub.addAction(_T(text), lambda _c=False, k=key: show(k))
    else:
        app.add_menu_action("Motion", lambda: show(),
                            tip=_T("Mécanismes : pivots, glissières, rails, validation."))
    return panel
