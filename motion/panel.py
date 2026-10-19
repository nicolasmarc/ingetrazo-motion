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
from PySide6.QtWidgets import (QAbstractItemView, QCheckBox, QComboBox,
                               QDoubleSpinBox, QFormLayout, QGridLayout,
                               QGroupBox, QHBoxLayout, QLabel, QPlainTextEdit,
                               QSlider, QSpinBox, QToolButton, QTreeWidget,
                               QTreeWidgetItem, QVBoxLayout, QWidget)

from . import collide, document as doc, icons, samples, solver

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

TIPS = {
    "fixed": "Pièce fixe (Fixed part) — les groupes sélectionnés forment le bâti.",
    "moving": "Pièce mobile (Moving part) — les groupes sélectionnés bougent ensemble.",
    "pivot": "Pivot — sélectionner UNE arête ou ligne de construction sur l'axe.",
    "slider": "Glissière (Slider) — sélectionner UNE arête dans le sens du coulissement.",
    "guide": "Guide / rail — sélectionner les arêtes du rail + un petit trait en travers à chaque galet.",
    "driver": "Moteur 0–100 % (Driver) — la liaison choisie dans la liste, avec la course indiquée.",
    "validate": "Valider (Validate) — de 0 à 100 % : collisions, butées, jeu minimal.",
    "trajectory": "Trajectoire (Trajectory) — trace les extrémités des arêtes sélectionnées.",
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


def group_points(group):
    out = []
    for mesh, aff in _placements(group):
        if not mesh.vertices:
            continue
        P = np.array([_xyz(v.position) for v in mesh.vertices])
        out.append(P if aff is None else P @ aff[0].T + aff[1])
    return np.vstack(out) if out else np.zeros((0, 3))


def group_triangles(group):
    """World triangles (n, 3, 3) of a group's faces, holes cut (IngeTrazo's
    own triangulation), nested placements included."""
    out = []
    for mesh, aff in _placements(group):
        tri = []
        for f in mesh.faces:
            try:
                for a, b, c in f.triangulate():
                    tri.append((_xyz(a), _xyz(b), _xyz(c)))
            except Exception:
                continue
        if tri:
            T = np.array(tri)
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


def group_segments(group):
    out = []
    for mesh, aff in _placements(group):
        segs = [(_xyz(e.v0.position), _xyz(e.v1.position)) for e in mesh.edges
                if not getattr(e, "hidden", False)]
        if not segs:
            continue
        S = np.array(segs)
        if aff is not None:
            S = S @ aff[0].T + aff[1]
        out.append(S)
    return np.vstack(out) if out else np.zeros((0, 2, 3))


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
            self.src.addItem(text, key)
        self.src.currentIndexChanged.connect(self._source_changed)
        srow = QHBoxLayout()
        srow.addWidget(self.src, 1)
        self.step_btn = QToolButton()
        self.step_btn.setText("Importer STEP…")
        self.step_btn.setToolTip("Assemblage STEP (AP203/214/242) : une pièce = un groupe, "
                                 "en une seule étape d'annulation.")
        self.step_btn.clicked.connect(self._import_step_dialog)
        srow.addWidget(self.step_btn)
        self.scan_btn = QToolButton()
        self.scan_btn.setIcon(icons.icon("scan", self, 20))
        self.scan_btn.setText("Détecter…")
        self.scan_btn.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.scan_btn.setToolTip("Analyse les pièces (la sélection, sinon tout le modèle) et "
                                 "propose les liaisons : arbres dans des alésages (pivots), "
                                 "appuis plans (glissières), engrenages. Rien n'est appliqué "
                                 "avant validation.")
        self.scan_btn.clicked.connect(self._scan_dialog)
        srow.addWidget(self.scan_btn)
        lay.addLayout(srow)

        grid = QGridLayout()
        grid.setSpacing(2)
        self.buttons = {}
        for i, name in enumerate(icons.NAMES):
            b = QToolButton()
            b.setIcon(icons.icon(name, self, 24))
            b.setIconSize(QSize(24, 24))
            b.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)
            b.setText(icons.LABELS[name][0])
            b.setToolTip(TIPS[name])
            b.setAutoRaise(True)
            b.setMinimumWidth(10)
            b.clicked.connect(lambda _c=False, n=name: self._action(n))
            grid.addWidget(b, i // 4, i % 4)
            self.buttons[name] = b
        self.buttons["trajectory"].setCheckable(True)
        lay.addLayout(grid)

        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Nom", "Type", "Détail"])
        self.tree.setRootIsDecorated(False)
        self.tree.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tree.itemSelectionChanged.connect(self._item_selected)
        lay.addWidget(self.tree, 1)

        row = QHBoxLayout()
        self.delete_btn = QToolButton()
        self.delete_btn.setText("Supprimer")
        self.delete_btn.clicked.connect(self._delete)
        row.addWidget(self.delete_btn)
        row.addStretch(1)
        lay.addLayout(row)

        mrow = QHBoxLayout()
        mrow.addWidget(QLabel("Matériau"))
        self.mat_combo = QComboBox()
        self.mat_combo.setToolTip("Matériau de la pièce choisie dans la liste (ou des groupes "
                                  "sélectionnés) : masse, centre de gravité et inertie en "
                                  "découlent.")
        self.mat_combo.activated.connect(self._material_chosen)
        mrow.addWidget(self.mat_combo, 1)
        self.mat_new = QToolButton()
        self.mat_new.setText("Nouveau…")
        self.mat_new.setToolTip("Ajouter un matériau (ou corriger une valeur) pour ce document.")
        self.mat_new.clicked.connect(self._material_dialog)
        mrow.addWidget(self.mat_new)
        lay.addLayout(mrow)
        self.mass_label = QLabel("")
        self.mass_label.setWordWrap(True)
        lay.addWidget(self.mass_label)

        box = QGroupBox("Liaison choisie")
        form = QFormLayout(box)
        self.combo_a, self.combo_b = QComboBox(), QComboBox()
        form.addRow("Pièce A", self.combo_a)
        form.addRow("Pièce B", self.combo_b)
        self.lim_on = QCheckBox("Butées")
        self.lim_lo, self.lim_hi = self._spin(), self._spin()
        lr = QHBoxLayout()
        lr.addWidget(self.lim_on)
        lr.addWidget(self.lim_lo)
        lr.addWidget(self.lim_hi)
        form.addRow(lr)
        self.drv_lo, self.drv_hi = self._spin(), self._spin()
        dr = QHBoxLayout()
        dr.addWidget(self.drv_lo)
        dr.addWidget(QLabel("→"))
        dr.addWidget(self.drv_hi)
        self.unit = QLabel("")
        dr.addWidget(self.unit)
        form.addRow("Course moteur", dr)
        self.motor_on = QCheckBox("Moteur (plusieurs possibles : même 0–100 %, chacun sa course)")
        self.motor_on.setToolTip("Coché : cette liaison est un moteur. Vitesse = course ; "
                                 "déphasage = valeur de départ.")
        form.addRow(self.motor_on)
        self.apply_btn = QToolButton()
        self.apply_btn.setText("Appliquer")
        self.apply_btn.clicked.connect(self._apply_joint)
        form.addRow(self.apply_btn)
        lay.addWidget(box)

        play = QHBoxLayout()
        self.play_btn = QToolButton()
        self.play_btn.setText("▶")
        self.play_btn.setToolTip("Lecture / pause")
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
        self.loop = QCheckBox("Aller-retour")
        self.loop.setChecked(True)
        prow = QHBoxLayout()
        prow.addWidget(self.loop)
        prow.addStretch(1)
        self.video_btn = QToolButton()
        self.video_btn.setText("Exporter la vidéo…")
        self.video_btn.setToolTip("Vidéo en écran partagé : la vue IngeTrazo à gauche, "
                                  "l'animation ombrée et les mesures à droite.")
        self.video_btn.clicked.connect(self._export_dialog)
        prow.addWidget(self.video_btn)
        lay.addLayout(prow)

        val = QHBoxLayout()
        val.addWidget(QLabel("Jeu mini"))
        self.jeu = QDoubleSpinBox()
        self.jeu.setRange(0, 100)
        self.jeu.setDecimals(1)
        self.jeu.setSuffix(" mm")
        self.jeu.setValue(5.0)
        val.addWidget(self.jeu)
        val.addWidget(QLabel("Pas"))
        self.pas = QSpinBox()
        self.pas.setRange(3, 401)
        self.pas.setValue(51)
        val.addWidget(self.pas)
        lay.addLayout(val)

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
    def reload(self):
        """The document changed (edit, undo, New, Open)."""
        if self.source == "model":
            self.data = doc.normalized(self.app.document_data(default=None))
            self.jeu.blockSignals(True)
            self.jeu.setValue(1000 * self.data.get("jeu_min", 0.005))
            self.jeu.blockSignals(False)
            self.pas.setValue(int(self.data.get("pas", 51)))
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
                segs = [group_segments(by[u]) for u in b["groups"] if u in by]
                segs = [s for s in segs if len(s)]
                if segs:
                    self.body_segments[b["name"]] = np.vstack(segs)
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
                       f"{len(b['groups'])} groupe(s)") for b in self.data["bodies"]]
            joints = [(j["name"], j["kind"], f"{j['a']} ↔ {j['b']}")
                      for j in self.data["joints"]]
            motors = [x["joint"] for x in self.data["drivers"]]
        else:
            m = self.mechanism()
            bodies = [(m.ground.name, "fixed", "")] + [(b.name, "moving", "")
                                                       for b in m.bodies]
            joints = [(j.name, j.kind, f"{j.a.name} ↔ {j.b.name}") for j in m.joints]
            motors = [d[0].name for d in m.drivers]
        masses = {}
        if self.source == "model" and self.data["bodies"]:
            try:
                masses = getattr(self.mechanism(), "masses", {}) or {}
            except Exception:
                masses = {}
        if masses:
            bodies = [(n, k, self._mass_text(masses.get(n)) or dt) for n, k, dt in bodies]
        kinds = {"fixed": "Pièce fixe", "moving": "Pièce mobile", "pivot": "Pivot",
                 "slider": "Glissière", "guide": "Guide / rail",
                 "coupling": "Couplage"}
        for name, kind, detail in bodies + joints:
            it = QTreeWidgetItem([name, kinds[kind], detail])
            it.setIcon(0, icons.icon(kind, self.tree, 16))
            it.setData(0, Qt.UserRole, name)
            if name in motors:
                it.setIcon(1, icons.icon("driver", self.tree, 16))
                it.setText(1, kinds[kind] + " · moteur")
            self.tree.addTopLevelItem(it)
        for c in range(3):
            self.tree.resizeColumnToContents(c)
        self.tree.blockSignals(False)
        editable = self.source == "model"
        for n in ("fixed", "moving", "pivot", "slider", "guide"):
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
        if motors:                               # show the first motor's travel
            hit = self.tree.findItems(motors[0], Qt.MatchExactly)
            if hit:
                self.tree.setCurrentItem(hit[0])

    # ---- materials ------------------------------------------------------------------
    def _fill_materials(self):
        from . import materials
        lib = materials.library(self.data if self.source == "model" else None)
        self.mat_combo.blockSignals(True)
        self.mat_combo.clear()
        self.mat_combo.addItem("— non défini —", None)
        order = {"bois": 0, "métal": 1, "plastique": 2, "autre": 3}
        for k in sorted(lib, key=lambda k: (order.get(lib[k].get("family"), 9), lib[k]["name"])):
            r = lib[k]
            self.mat_combo.addItem(f"{r['name']} — {r['density']:g} kg/m³", k)
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
        mp = (getattr(self.mechanism(), "masses", {}) or {}).get(name)
        if not mp:
            self.mass_label.setText("")
            return
        if mp["mass"] > 0:
            c = mp["com"] * 1000
            txt = (f"{name} : {mp['mass']:.3g} kg, volume {mp['volume'] * 1e6:.4g} cm³ — "
                   f"centre de gravité ({c[0]:.1f} ; {c[1]:.1f} ; {c[2]:.1f}) mm")
        else:
            txt = f"{name} : volume {mp['volume'] * 1e6:.4g} cm³ — choisir un matériau"
        if mp["missing"] and mp["mass"] > 0:
            txt += f" ; {len(mp['missing'])} groupe(s) sans matériau"
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
            self.say("Choisir une pièce dans la liste, ou sélectionner des groupes.")
            return
        d = doc.set_material(self.data, uids, key)
        label = self.mat_combo.currentText().split(" — ")[0]
        self._store(d, f"{label} : {len(uids)} groupe(s).")
        if name:
            hit = self.tree.findItems(name, Qt.MatchExactly)
            if hit:
                self.tree.setCurrentItem(hit[0])

    def _material_dialog(self):
        from PySide6.QtWidgets import QDialog, QDialogButtonBox, QLineEdit
        from . import materials
        dlg = QDialog(self)
        dlg.setWindowTitle("Matériau du document")
        form = QFormLayout(dlg)
        name = QLineEdit()
        base = QComboBox()
        lib = materials.library(self.data)
        base.addItem("(nouveau)", None)
        for k in sorted(lib, key=lambda k: lib[k]["name"]):
            base.addItem(lib[k]["name"], k)
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
            name.setText(r["name"])
            rho.setValue(r["density"])
            E.setValue(r.get("young") or 0)
            mu.setValue(r.get("friction") or 0)
            e.setValue(r.get("restitution") or 0)
        base.currentIndexChanged.connect(load)
        form.addRow("Partir de", base)
        form.addRow("Nom", name)
        form.addRow("Masse volumique", rho)
        form.addRow("Module d'Young", E)
        form.addRow("Frottement (sec)", mu)
        form.addRow("Restitution (chocs)", e)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Cancel).setText("Annuler")
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        form.addRow(bb)
        if dlg.exec() != QDialog.Accepted or not name.text().strip():
            return
        key = base.currentData()
        if key is None or lib[key]["name"] != name.text().strip():
            key = "doc:" + materials._plain(name.text().strip()).replace(" ", "-")
        fam = lib[base.currentData()]["family"] if base.currentData() else "autre"
        d = doc.add_material(self.data, key, dict(
            name=name.text().strip(), family=fam, density=rho.value(),
            young=E.value() or None, friction=mu.value(), restitution=e.value()))
        self._store(d, f"Matériau « {name.text().strip()} » : {rho.value():g} kg/m³.")

    @staticmethod
    def _mass_text(mp):
        if not mp:
            return ""
        mats = " + ".join(mp["materials"]) if mp["materials"] else ""
        if mp["missing"] and not mp["materials"]:
            return "matériau ?"
        txt = f"{mats} · {mp['mass']:.3g} kg" if mats else f"{mp['mass']:.3g} kg"
        if mp["missing"]:
            txt += f" (+{len(mp['missing'])} sans matériau)"
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
        return kind == "pivot"

    def _to_ui(self, kind, v):
        return math.degrees(v) if self._is_angle(kind) else 1000 * v

    def _from_ui(self, kind, v):
        return math.radians(v) if self._is_angle(kind) else v / 1000

    def _item_selected(self):
        name = self._chosen()
        self._show_material(name)
        if self.source != "model":
            j = next((j for j in self.mechanism().joints if j.name == name), None)
            if j is not None:
                self._show_travel(j.kind, j, None)
            return
        j = self._joint_record(name)
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
            self.say(f"{icons.LABELS[name][0]} : {exc}")
        self.app.viewport.update()

    def _do_fixed(self, fixed=True):
        groups, _ = selection(self._scene())
        if not groups:
            self.say("Sélectionner un ou plusieurs groupes.")
            return
        d, name = doc.add_body(self.data, [g.uid for g in groups], fixed)
        d = self._guess_materials(d, [g.uid for g in groups])
        self._store(d, f"{name} : {len(groups)} groupe(s).")

    def _do_moving(self):
        self._do_fixed(False)

    def _two_bodies(self, point):
        boxes = self._boxes()
        if len(boxes) < 2:
            raise ValueError("définir d'abord au moins deux pièces")
        b1 = doc.nearest_body(point, boxes)
        b2 = doc.nearest_body(point, boxes, exclude=(b1,))
        fixed = {b["name"] for b in self.data["bodies"] if b["fixed"]}
        return (b2, b1) if b1 not in fixed and b2 in fixed else (b1, b2)

    def _one_segment(self):
        _, segs = selection(self._scene())
        if len(segs) != 1:
            raise ValueError("sélectionner UNE arête (ou ligne de construction)")
        p0, p1 = segs[0]
        if np.linalg.norm(p1 - p0) < 1e-9:
            raise ValueError("arête de longueur nulle")
        return p0, p1

    def _do_pivot(self, kind="pivot"):
        p0, p1 = self._one_segment()
        a, b = self._two_bodies((p0 + p1) / 2)
        d, name = doc.add_joint(self.data, kind, a, b, point=p0 if kind == "slider"
                                else (p0 + p1) / 2, axis=p1 - p0)
        self._store(d, f"{name} : {a} ↔ {b}.")

    def _do_slider(self):
        self._do_pivot("slider")

    def _do_guide(self):
        _, segs = selection(self._scene())
        if len(segs) < 2:
            raise ValueError("sélectionner le rail et un trait par galet")
        rail, rollers = doc.rail_and_rollers(segs)
        if not rollers:
            raise ValueError("aucun galet : tracer un petit trait en travers du rail")
        boxes = self._boxes()
        fixed = [b["name"] for b in self.data["bodies"] if b["fixed"]]
        if not fixed:
            raise ValueError("définir d'abord la pièce fixe qui porte le rail")
        a = doc.nearest_body(np.mean(rail, axis=0), {k: boxes[k] for k in fixed}) or fixed[0]
        d, names = self.data, []
        for r in rollers:
            b = doc.nearest_body(r, boxes, exclude=fixed)
            if b is None:
                raise ValueError("aucune pièce mobile près du galet")
            d, n = doc.add_joint(d, "guide", a, b, point=r, rail=rail)
            names.append(n)
        self._store(d, f"{len(names)} galet(s) sur le rail de {a}.")

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
            self.say(f"Moteur : {j.name} ({len(m.drivers)} moteur(s)).")
            return
        j = self._joint_record(name)
        if j is None:
            if len(self.data["joints"]) == 1:
                j = self.data["joints"][0]
                self._show_travel(j["kind"], None, j)
            else:
                raise ValueError("choisir une liaison dans la liste")
        d = doc.add_driver(self.data, j["name"], self._from_ui(j["kind"], self.drv_lo.value()),
                           self._from_ui(j["kind"], self.drv_hi.value()))
        self._store(d, f"Moteur : {j['name']} ({len(d['drivers'])} moteur(s)).")

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
        self._store(d, f"{j['name']} mis à jour.")

    def _delete(self):
        name = self._chosen()
        if name and self.source == "model":
            self._store(doc.remove(self.data, name), f"{name} supprimé.")

    def simulation(self):
        """The example's dynamic simulation (weight, inertia; no motor
        needed), as poses at 30 per second; None for a kinematic one."""
        sim = (self.sample_info or {}).get("simulation")
        if not sim:
            return None
        if getattr(self, "_sim_cache", (None,))[0] is not self.mech:
            from . import dynamics
            m = self.mechanism()
            dyn = dynamics.Dynamics(m, **sim.get("params", {}))
            dt = sim.get("dt", 0.002)
            every = max(1, int(round(1.0 / (30 * dt))))
            rows = dyn.run(sim["duree"], dt=dt, every=every)
            n = len(rows)
            poses = [dict(t=i / max(n - 1, 1), value=r["t"], x=r["x"], ok=True,
                          res=float(np.linalg.norm(dyn.C(r["x"], r["t"]))),
                          ke=r["ke"], pe=r["pe"]) for i, r in enumerate(rows)]
            self._sim_cache = (self.mech, poses)
        return self._sim_cache[1]

    def _do_validate(self):
        m = self.mechanism()
        poses = self.simulation()
        if m.driver is None and poses is None:
            raise ValueError("pas de moteur")
        if self.source == "model":
            self.data["jeu_min"] = self.jeu.value() / 1000
            self.data["pas"] = self.pas.value()
        self.report = collide.validate(m, self.pas.value(), self.jeu.value() / 1000,
                                       poses=poses)
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
            lines.append("✔ Mécanisme valide de 0 à 100 %.")
        else:
            lines.append(f"✖ {len(rep['problems'])} problème(s) :")
            lines += ["  " + p for p in rep["problems"][:60]]
            if len(rep["problems"]) > 60:
                lines.append("  …")
        if w is not None:
            lines.append(f"Jeu le plus faible : {w['distance'] * 1000:.1f} mm entre "
                         f"{w['a']} ({w['box_a']}) et {w['b']} ({w['box_b']}) "
                         f"à {round(100 * w['t'])} %.")
        # the most frequent count over a few poses: a single pose can mislead
        # (a roller at the very end of its rail reads 0; after whole turns of
        # a crank a part's rotation parameters reach 2π and it reads 5 or 7
        # on the rail car instead of 1)
        m, F = self.mechanism(), self.frames
        counts = [solver.dof(m, F[round(f * (len(F) - 1))]["x"])
                  for f in (0.1, 0.3, 0.5, 0.7, 0.9)]
        dof = max(set(counts), key=lambda c: (counts.count(c), -c))
        lines.append(f"Degrés de liberté (sans moteur) : {dof}")
        st = rep["steps"]
        if st and "ke" in st[0]:
            e0, e1 = st[0]["ke"] + st[0]["pe"], st[-1]["ke"] + st[-1]["pe"]
            lines.append(f"Simulation dynamique de {st[-1]['value']:.1f} s : énergie "
                         f"{1000 * e0:.0f} → {1000 * e1:.0f} mJ (frottement dans les liaisons).")
        ms = getattr(m, "masses", {}) or {}
        if ms:
            known = {n: mp for n, mp in ms.items() if mp["mass"] > 0}
            if known:
                tot = sum(mp["mass"] for mp in known.values())
                lines.append(f"Masse totale : {tot:.3g} kg — "
                             + ", ".join(f"{n} {mp['mass']:.3g} kg" for n, mp in known.items()))
            miss = sum(len(mp["missing"]) for mp in ms.values())
            if miss:
                lines.append(f"ⓘ {miss} groupe(s) sans matériau : masse non comptée.")
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
        self.say(f"Trajectoire de {len(self.traces)} point(s).")

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
        self.say(f"STEP : {len(groups)} pièce(s) importée(s)"
                 + (f" — non lu : {warn}" if warn else "") + ".")
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
        msg = (f"Détection : {n['pivot']} pivot(s), {n['slider']} glissière(s), "
               f"{n['gears']} engrenage(s) ; pièce fixe : {parts[frame][0]}.")
        if not d["drivers"]:
            msg += " Choisir un moteur dans la liste."
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
            QMessageBox.warning(self, "Détecter les liaisons", f"{type(exc).__name__} : {exc}")
            return
        QApplication.restoreOverrideCursor()
        if not props:
            QMessageBox.information(self, "Détecter les liaisons",
                                    f"{len(parts)} pièce(s) analysée(s) : aucune liaison "
                                    "reconnue (arbre dans un alésage, appuis plans, engrenage).")
            return
        dlg = QDialog(self)
        dlg.setWindowTitle("Liaisons proposées")
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel(f"{len(parts)} pièce(s) analysée(s). Cocher les liaisons à créer :"))
        fr = QHBoxLayout()
        fr.addWidget(QLabel("Pièce fixe"))
        combo = QComboBox()
        keys = sorted(parts, key=lambda k: parts[k][0])
        for k in keys:
            combo.addItem(parts[k][0], k)
        combo.setCurrentIndex(keys.index(frame))
        fr.addWidget(combo, 1)
        lay.addLayout(fr)
        tree = QTreeWidget()
        tree.setHeaderLabels(["Liaison", "Pièces", "Détail", "Confiance"])
        tree.setRootIsDecorated(False)
        label = {"pivot": "Pivot", "slider": "Glissière", "gears": "Engrenage"}
        icon = {"pivot": "pivot", "slider": "slider", "gears": "coupling"}
        items = []
        for x in props:
            it = QTreeWidgetItem([label[x.kind], f"{parts[x.a][0]} ↔ {parts[x.b][0]}",
                                  x.detail, f"{round(100 * x.score)} %"])
            it.setIcon(0, icons.icon(icon[x.kind], tree, 16))
            it.setCheckState(0, Qt.Checked if x.score >= 0.5 else Qt.Unchecked)
            tree.addTopLevelItem(it)
            items.append((it, x))
        for c in range(4):
            tree.resizeColumnToContents(c)
        lay.addWidget(tree, 1)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Ok).setText("Créer les liaisons")
        bb.button(QDialogButtonBox.Cancel).setText("Annuler")
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
        path, _ = QFileDialog.getOpenFileName(self, "Importer un assemblage STEP", "",
                                              "STEP (*.step *.stp *.STEP *.STP)")
        if not path:
            return
        QApplication.setOverrideCursor(Qt.WaitCursor)
        try:
            self.import_step(path)
        except Exception as exc:
            QMessageBox.warning(self, "Importer STEP", f"{type(exc).__name__} : {exc}")
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
            self.say(f"Lecture : {exc}")
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
            self.say(f"{round(100 * t)} % : position non atteinte.")

    def _play(self, on):
        self.play_btn.setText("⏸" if on else "▶")
        if on:
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
        for body in bodies:
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
                self._draw_segments(painter, pts, color, 1.4)
                continue
            R = body.rot(q)
            w = body.center + q[:3] + (pts - body.center) @ R.T
            self._draw_segments(painter, w, color, 1.6)
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
        if m.driver is None:
            self.say("Exporter la vidéo : pas de moteur.")
            return
        dlg = QDialog(self)
        dlg.setWindowTitle("Exporter la vidéo")
        form = QFormLayout(dlg)
        dur = QDoubleSpinBox()
        dur.setRange(2, 120)
        dur.setValue(8)
        dur.setSuffix(" s")
        form.addRow("Durée", dur)
        res = QComboBox()
        for text, wh in (("1920 × 1080", (1920, 1080)), ("1280 × 720", (1280, 720))):
            res.addItem(text, wh)
        form.addRow("Résolution", res)
        back = QCheckBox("Aller-retour (0 → 100 → 0 %)")
        back.setChecked(True)
        form.addRow(back)
        enc = QLabel()
        form.addRow("Format", enc)
        pick = QToolButton()
        pick.setText("Indiquer ffmpeg.exe…")
        form.addRow(pick)

        def refresh():
            kind, exe = video.mp4_backend(self._ffmpeg_hint())
            enc.setText({"qt": "MP4 (Qt Multimedia)", "ffmpeg": f"MP4 (ffmpeg : {exe})"}.get(
                kind, "AVI Motion-JPEG — pour du MP4, indiquer ffmpeg.exe"))
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
        ext, filt = (".mp4", "Vidéo MP4 (*.mp4)") if kind else (".avi", "Vidéo AVI (*.avi)")
        path, _ = QFileDialog.getSaveFileName(self, "Exporter la vidéo", "motion" + ext, filt)
        if not path:
            return
        prog = QProgressDialog("Rendu de la vidéo…", "Annuler", 0, 100, self)
        prog.setWindowModality(Qt.WindowModal)
        prog.setMinimumDuration(0)

        def step(i, n):
            prog.setMaximum(n)
            prog.setValue(i)
            return not prog.wasCanceled()

        try:
            out, label = self.export_video(path, dur.value(), res.currentData(),
                                           back.isChecked(), progress=step)
        except Exception as exc:
            prog.close()
            QMessageBox.warning(self, "Exporter la vidéo", str(exc))
            return
        prog.close()
        self.say(f"Vidéo enregistrée : {out} — {label}." if out else "Export annulé.")

    def _moving_groups(self):
        """IngeTrazo groups of the moving bodies (model source)."""
        if self.source != "model":
            return []
        by = groups_by_uid(self._scene())
        return [by[u] for b in self.data["bodies"] if not b["fixed"]
                for u in b["groups"] if u in by]

    def _refresh_view_cache(self):
        # Best effort, viewport internals: forget the memoised placement
        # epoch so a group's ``hidden`` flag is seen without bumping the
        # document version (no undo step, the document stays saved). If
        # these names change, moving parts simply stay drawn in place.
        vp = self.app.viewport
        for name in ("_epoch_memo", "_epoch_same"):
            if hasattr(vp, name):
                try:
                    setattr(vp, name, None)
                except Exception:
                    pass

    def _viewport_image(self, width, shaded=None, x=None, hits=None):
        """The IngeTrazo view as an image: IngeTrazo renders the model
        (moving parts taken out for the shot), Motion draws the moving
        parts at pose ``x`` on top, then the joints and trajectories."""
        vp = self.app.viewport
        img = None
        render = getattr(vp, "render_image", None)
        hide = [g for g in self._moving_groups() if not g.hidden] if shaded is not None else []
        if callable(render):
            try:
                for g in hide:
                    g.hidden = True
                if hide:
                    self._refresh_view_cache()
                img = render(width)
            except Exception:
                img = None
            finally:
                for g in hide:
                    g.hidden = False
                if hide:
                    self._refresh_view_cache()
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
                    for f in mesh.faces:
                        P = np.array([_xyz(v) for v in f.vertices])
                        if aff is not None:
                            P = P @ aff[0].T + aff[1]
                        faces.append(P)
            if faces:
                out[b["name"]] = faces
        return out

    def full_turn(self):
        """Driver is a pivot doing a whole revolution (a crank)."""
        m = self.mechanism()
        if m.driver is None or self.simulation() is not None:
            return False
        j, lo, hi = m.driver
        return j.kind == "pivot" and abs(abs(hi - lo) - 2 * math.pi) < 1e-6

    def export_video(self, path, seconds=8.0, size=(1920, 1080), aller_retour=True,
                     fps=30, progress=None, cycles=None):
        """Render the motion to ``path`` (split screen). Returns (file, kind
        label), or (None, None) when ``progress(i, n)`` returned False.
        A crank (full-turn driver) runs at constant speed for ``cycles``
        turns (default: one turn per 2 s); anything else is eased, there
        and back when ``aller_retour``."""
        from . import render, video
        m = self.mechanism()
        poses = self.simulation()
        if m.driver is None and poses is None:
            raise ValueError("pas de moteur")
        # checked once, at the validation steps: the readings of a frame
        # (smallest gap, collisions) are those of its nearest step
        rows = self.report["steps"] if self.report else None
        if rows is None or len(rows) < 3:
            rows = collide.validate(m, max(self.pas.value(), 21),
                                    self.jeu.value() / 1000, poses=poses)["steps"]
        sweep_rows = rows
        if poses is not None:                     # a simulation plays in real time
            seconds = poses[-1]["value"]
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
            unit = lambda v: f"{v / (2 * math.pi):.2f} tr"
        name = SAMPLE_NAMES.get(self.source, "Mécanisme")
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
                    x, ok = sweep_rows[int(round(f))]["x"], True
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
                info = [f"temps {near['value']:.2f} s"] if poses is not None else []
                for (jj, l_, h_), v in zip(m.drivers, m.driver_targets(shown) if m.drivers else []):
                    if jj.kind == "pivot":
                        txt = (f"{v / (2 * math.pi):.2f} tr" if m.multi_turn((jj, l_, h_))
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
                    info.append(f"jeu mini {1000 * max(cl[0]['distance'], 0):.1f} mm")
                info.append("collision : " + " / ".join(sorted(hits)) if hits else "aucune collision")
                if not ok:
                    info.append("position non atteinte")
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
        for i in range(0, len(pts) - 1, 2):
            if ok[i] and ok[i + 1]:
                painter.drawLine(QPointF(px[i], py[i]), QPointF(px[i + 1], py[i + 1]))


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
    dock = app.add_panel("Motion", panel, start_hidden=False)
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
        sub.addAction("Panneau Motion", lambda: show())
        sub.addAction("Importer un assemblage STEP…",
                      lambda: (show(), panel._import_step_dialog()))
        sub.addAction("Exemple : chaise pliante", lambda: show("chaise"))
        sub.addAction("Exemple : porte sectionnelle", lambda: show("porte"))
        sub.addAction("Exemple : oiseau à battement d'ailes", lambda: show("oiseau"))
        sub.addAction("Exemple : draisine à balancier", lambda: show("draisine"))
        sub.addAction("Exemple : chaîne détendue et tendeur", lambda: show("chaine"))
        sub.addAction("Exemple : chaîne souple (dynamique)", lambda: show("chaine_souple"))
        sub.addAction("Exemple : pendule double (dynamique)", lambda: show("pendule_double"))
    else:
        app.add_menu_action("Motion", lambda: show(),
                            tip="Mécanismes : pivots, glissières, rails, validation.")
    return panel
