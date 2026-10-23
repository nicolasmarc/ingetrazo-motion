"""The panel inside a real IngeTrazo window (offscreen).

Needs PySide6 and an IngeTrazo source checkout: set INGETRAZO_SRC, or put
the clone next to this repository as ``../ingetrazo-upstream`` or
``../ingetrazo``. Skipped (not failed) when either is missing.
"""
import math, os, shutil, sys, tempfile
from pathlib import Path
import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MOTION_LANG", "fr")        # the texts these tests read
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
REPO = HERE.parent


def _src():
    for c in (os.environ.get("INGETRAZO_SRC"), REPO.parent / "ingetrazo-upstream",
              REPO.parent / "ingetrazo"):
        if c and (Path(c) / "views" / "extension_api.py").is_file():
            return Path(c)
    return None


SRC = _src()
try:
    import PySide6  # noqa: F401
    HAVE_QT = True
except ImportError:
    HAVE_QT = False
SKIP = None if (SRC and HAVE_QT) else "IngeTrazo source or PySide6 not available"

_state = {}


def _window():
    """One main window with the extension installed from a temp folder."""
    if "win" in _state:
        return _state["win"]
    sys.path.insert(0, str(SRC))
    os.chdir(SRC)
    plugins = Path(tempfile.mkdtemp()) / "plugins"
    plugins.mkdir()
    shutil.copytree(REPO / "motion", plugins / "motion",
                    ignore=shutil.ignore_patterns("__pycache__"))
    from core import extensions
    extensions.plugin_dirs = lambda: [plugins]
    from PySide6.QtWidgets import QApplication
    _state["qapp"] = QApplication.instance() or QApplication([])
    from views.main_window import MainWindow
    win = MainWindow()
    win.resize(1400, 900)
    win.show()
    dock = next(d for d in win._extension_docks if d.objectName() == "extension_motion")
    panel = dock.widget()
    if hasattr(panel, "setWidgetResizable"):     # 0.5.9+: a scroll area on short screens
        panel = panel.widget()
    _state.update(win=win, panel=panel, dock=dock)
    return win


def _box_group(name, lo, hi):
    """A closed box group (6 faces, their edges come with them)."""
    from PySide6.QtGui import QVector3D as V
    from core.group import Group
    from core.mesh import Mesh
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    m = Mesh()
    c = [V(*np.where([i & 1, i & 2, i & 4], hi, lo)) for i in range(8)]
    for f in ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4),
              (1, 5, 7, 3)):
        m.add_face([c[i] for i in f])
    return Group(m, name)


def _select(win, *ents):
    sel = win.viewport.scene.selection
    sel.clear()
    sel.update(ents)


def _reset(win, panel):
    scene = win.viewport.scene
    scene.groups.clear()
    scene.selection.clear()
    panel.set_source("model")
    panel.app.set_document_data(None)


def test_drop_leaf_built_from_the_model():
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    table = _box_group("Plateau", (-0.4, -0.2, -0.009), (0, 0.2, 0.009))
    leaf = _box_group("Abattant", (0.0, -0.2, -0.009), (0.6, 0.2, 0.009))
    scene.groups += [table, leaf]
    # the hinge on the top face, the leaf folds up: a hinge at mid-thickness
    # would swing the leaf's lower corner into the table (the check on the
    # real surfaces sees it)
    hinge = scene.mesh.add_edge(V(0, -0.2, 0.009), V(0, 0.2, 0.009))
    win.viewport.notify_scene_changed()

    _select(win, table); panel._action("fixed")
    _select(win, leaf); panel._action("moving")
    _select(win, hinge); panel._action("pivot")
    data = panel.app.document_data()
    assert [b["name"] for b in data["bodies"]] == ["fixe1", "pièce1"]
    j = data["joints"][0]
    assert j["kind"] == "pivot" and {j["a"], j["b"]} == {"fixe1", "pièce1"}
    assert j["a"] == "fixe1"                                # frame on side A
    # driver: pick the pivot in the list, 0 -> -90 degrees (leaf goes down)
    it = panel.tree.findItems("pivot1", Qt_MatchExactly())[0]
    panel.tree.setCurrentItem(it)
    panel.drv_lo.setValue(0)
    panel.drv_hi.setValue(-90)
    panel._action("driver")
    assert panel.app.document_data()["driver"]["joint"] == "pivot1"
    panel.pas.setValue(19)
    panel._action("validate")
    assert panel.report["ok"], panel.report["problems"]
    assert "valide" in panel.out.toPlainText()
    # scrub to 50 %: the leaf's far edge is at 45 degrees below
    panel.slider.setValue(500)
    m = panel.mechanism()
    body = m.bodies[0]
    tip = body.world_point(m.pose(panel.x, body), (0.6, 0, 0.009))
    assert abs(tip[0] - 0.6 * math.cos(math.pi / 4)) < 1e-6
    assert abs(abs(tip[2] - 0.009) - 0.6 * math.sin(math.pi / 4)) < 1e-6
    # each edit was one undo step: the steps (19) stored by Valider, then
    # the motor
    win._on_undo()
    assert panel.app.document_data()["pas"] == 51
    win._on_undo()
    assert panel.app.document_data()["driver"] is None
    win._on_redo()
    assert panel.app.document_data()["driver"]["joint"] == "pivot1"


def test_two_motors_from_the_panel():
    """Two leaves, two hinges, two motors: the second at half speed."""
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    table = _box_group("Plateau", (-0.4, -0.2, -0.009), (0, 0.2, 0.009))
    leaf1 = _box_group("Abattant1", (0.0, -0.2, -0.009), (0.6, 0.2, 0.009))
    table2 = _box_group("Plateau2", (-0.4, 1.0, -0.009), (0, 1.4, 0.009))
    leaf2 = _box_group("Abattant2", (0.0, 1.0, -0.009), (0.6, 1.4, 0.009))
    scene.groups += [table, leaf1, table2, leaf2]
    h1 = scene.mesh.add_edge(V(0, -0.2, 0.009), V(0, 0.2, 0.009))   # on the top face
    h2 = scene.mesh.add_edge(V(0, 1.0, 0.009), V(0, 1.4, 0.009))
    win.viewport.notify_scene_changed()
    _select(win, table, table2); panel._action("fixed")
    _select(win, leaf1); panel._action("moving")
    _select(win, leaf2); panel._action("moving")
    _select(win, h1); panel._action("pivot")
    _select(win, h2); panel._action("pivot")
    for name, hi in (("pivot1", -90), ("pivot2", -45)):
        panel.tree.setCurrentItem(panel.tree.findItems(name, Qt_MatchExactly())[0])
        panel.drv_lo.setValue(0)
        panel.drv_hi.setValue(hi)
        panel._action("driver")
    data = panel.app.document_data()
    assert [x["joint"] for x in data["drivers"]] == ["pivot1", "pivot2"]
    panel.pas.setValue(11)
    panel._action("validate")
    assert panel.report["ok"], panel.report["problems"]
    panel.slider.setValue(500)
    m = panel.mechanism()
    for body, deg in ((m.bodies[0], 45), (m.bodies[1], 22.5)):
        tip = body.world_point(m.pose(panel.x, body), (0.6, body.center[1], 0.009))
        assert abs(math.degrees(math.atan2(abs(tip[2] - 0.009), tip[0])) - deg) < 1e-5
    # untick «Moteur» on the second one: back to one motor (one undo step)
    panel.tree.setCurrentItem(panel.tree.findItems("pivot2", Qt_MatchExactly())[0])
    assert panel.motor_on.isChecked()
    panel.motor_on.setChecked(False)
    panel._apply_joint()
    assert [x["joint"] for x in panel.app.document_data()["drivers"]] == ["pivot1"]
    win._on_undo()
    assert len(panel.app.document_data()["drivers"]) == 2


def test_panel_on_two_curved_rails_built_from_edges():
    """One door panel, two rollers per side, rails drawn as edges (line,
    12 arc segments, line), rollers marked by short strokes."""
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    H, R, h, w = 2.0, 0.38, 0.5, 1.2
    frame = _box_group("Linteau", (-0.25, -w - 0.2, 2.0), (-0.05, w + 0.2, 2.4))
    leaf = _box_group("Panneau", (-0.04, -w, 0.0), (0.0, w, h))
    scene.groups += [frame, leaf]
    rails = {}
    for y in (w, -w):
        pts = [(0, y, 0.0), (0, y, H)]
        pts += [(R - R * math.cos(k * math.pi / 24), y, H + R * math.sin(k * math.pi / 24))
                for k in range(1, 13)]
        pts.append((R + 1.5, y, H + R))
        rails[y] = [scene.mesh.add_edge(V(*pts[i]), V(*pts[i + 1]))
                    for i in range(len(pts) - 1)]
        rails[y] += [scene.mesh.add_edge(V(-0.03, y, z), V(0, y, z)) for z in (0.0, h)]
    win.viewport.notify_scene_changed()
    _select(win, frame); panel._action("fixed")
    _select(win, leaf); panel._action("moving")
    for y in (w, -w):
        _select(win, *rails[y]); panel._action("guide")
    data = panel.app.document_data()
    assert [j["kind"] for j in data["joints"]] == ["guide"] * 4, panel.status.text()
    assert all(j["a"] == "fixe1" and j["b"] == "pièce1" for j in data["joints"])
    first = data["joints"][0]["name"]                 # bottom roller (rail order)
    assert first == "galet1"
    panel.tree.setCurrentItem(panel.tree.findItems(first, Qt_MatchExactly())[0])
    assert abs(panel.drv_lo.value()) < 1e-9           # pre-filled: where it is
    panel.drv_hi.setValue(1500)                  # 1.5 m of travel
    panel._action("driver")
    panel.jeu.setValue(5)
    panel.pas.setValue(16)
    panel._action("validate")
    assert panel.report["ok"], panel.report["problems"]
    assert "Degrés de liberté (sans moteur) : 1" in panel.out.toPlainText()
    w_ = panel.report["worst"]
    # at 100 % the top roller reaches the start of the curve: the panel's
    # outer top corner sits right under the lintel's corner, 10 mm apart
    assert (w_["box_a"], w_["box_b"], w_["t"]) == ("Linteau", "Panneau", 1.0)
    assert abs(w_["distance"] - 0.01) < 1e-6           # float32 positions


def test_video_export_split_screen():
    """Export from the panel, through every writer available here."""
    if SKIP:
        print("skip:", SKIP); return
    import subprocess
    from motion_probe import probe
    win = _window()
    panel = _state["panel"]
    out_dir = Path(tempfile.mkdtemp())
    # model source (the drop-leaf of the first test is still in the scene)
    if not panel.app.document_data():
        test_drop_leaf_built_from_the_model()
    panel.set_source("model")
    assert panel._polys(), "group faces should be used for the shaded view"
    calls = []
    out, label = panel.export_video(str(out_dir / "abattant.mp4"), 1.0, (640, 360),
                                    progress=lambda i, n: calls.append(i) or True)
    assert Path(out).is_file() and calls[-1] == 30
    info = probe(out)
    if info:
        assert info["frames"] == 30 and (info["w"], info["h"]) == (640, 360)
    # example, forced to each writer
    panel.set_source("chaise")
    import importlib                      # the plugin's own copy of motion.video
    video = importlib.import_module(type(panel).__module__.rsplit(".", 1)[0] + ".video")
    for prefer in ("avi", "ffmpeg", "qt"):
        if prefer == "ffmpeg" and not video.find_ffmpeg():
            continue
        if prefer == "qt" and not video.qt_multimedia_available():
            continue
        real = video.open_writer
        video.open_writer = lambda *a, **k: real(*a, prefer=prefer)
        try:
            out, label = panel.export_video(str(out_dir / f"chaise_{prefer}.mp4"), 1.0,
                                            (1280, 720))
        finally:
            video.open_writer = real
        assert out.endswith(".avi" if prefer == "avi" else ".mp4"), out
        info = probe(out)
        if info:
            assert info["frames"] == 30 and info["w"] == 1280, (prefer, info)
    # cancelling stops early and returns nothing
    out, label = panel.export_video(str(out_dir / "stop.mp4"), 1.0, (320, 180),
                                    progress=lambda i, n: i < 5)
    assert out is None


def test_step_assembly_imported_then_animated():
    """A STEP assembly from another CAD kernel becomes groups, then a
    mechanism: the left lever turns about its hole."""
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    n0 = len(scene.groups)
    asm, groups = panel.import_step(str(HERE / "data" / "assemblage.step"))
    assert sorted(g.name for g in groups) == ["levier_d", "levier_g", "socle"]
    assert len(scene.groups) == n0 + 3 and set(scene.selection) == set(groups)
    socle = next(g for g in groups if g.name == "socle")
    assert socle.material["color"][0] == pytest_approx(0.6)
    from motion_probe import probe  # noqa: F401  (keeps the import list honest)
    # extents in metres
    sys.path.insert(0, str(REPO))
    import importlib
    pan = importlib.import_module(type(panel).__module__)
    pts = pan.group_points(socle)
    assert np.allclose(pts.min(0), (-0.05, -0.015, -0.0025), atol=1e-9)
    # faces come in turned outward: the signed volume of each closed part
    # (from IngeTrazo's own faces, triangulated by fan) is positive
    for g in groups:
        vol = 0.0
        for f in g.mesh.faces:
            P = [np.array([v.x(), v.y(), v.z()]) for v in f.vertices]
            for k in range(1, len(P) - 1):
                vol += P[0] @ np.cross(P[k], P[k + 1]) / 6
            for hole in f.holes:
                H = [np.array([v.x(), v.y(), v.z()]) for v in hole]
                for k in range(1, len(H) - 1):
                    vol += H[0] @ np.cross(H[k], H[k + 1]) / 6
        assert vol > 0, (g.name, vol)
    # undo takes the whole import back, redo brings it again
    win._on_undo()
    assert len(scene.groups) == n0
    win._on_redo()
    assert len(scene.groups) == n0 + 3
    # mechanism: socle fixed, left lever on a pivot through its hole
    lev = next(g for g in groups if g.name == "levier_g")
    a = math.radians(30)
    hole = np.array([-25 + (-20) * math.cos(a), (-20) * math.sin(a), 10]) / 1000
    axis = scene.mesh.add_edge(V(*(hole - [0, 0, 0.01])), V(*(hole + [0, 0, 0.01])))
    win.viewport.notify_scene_changed()
    _select(win, socle); panel._action("fixed")
    _select(win, lev); panel._action("moving")
    _select(win, axis); panel._action("pivot")
    panel.tree.setCurrentItem(panel.tree.findItems("pivot1", Qt_MatchExactly())[0])
    panel.drv_lo.setValue(0)
    panel.drv_hi.setValue(90)
    panel._action("driver")
    panel.pas.setValue(7)
    panel._action("validate")
    assert panel.report["ok"], panel.report["problems"]
    m = panel.mechanism()
    x = panel.report["steps"][-1]["x"]
    body = m.bodies[0]
    c0 = np.array([-25 + 0.6487 * math.cos(a), 0.6487 * math.sin(a), 10]) / 1000
    c1 = body.world_point(m.pose(x, body), c0)
    assert abs(np.linalg.norm((c1 - hole)[:2]) - np.linalg.norm((c0 - hole)[:2])) < 1e-9


def pytest_approx(v, tol=1e-6):
    class _A:
        def __eq__(self, other):
            return abs(other - v) < tol
    return _A()


def Qt_MatchExactly():
    from PySide6.QtCore import Qt
    return Qt.MatchExactly


def test_document_data_survives_igz():
    if SKIP:
        print("skip:", SKIP); return
    win = _window()
    from core.scene import Scene
    from formats import igz
    scene = win.viewport.scene
    path = Path(tempfile.mkdtemp()) / "m.igz"
    igz.save_scene(scene, path)
    other = Scene()
    igz.load_into(other, path)
    assert other.plugin_data.get("motion") == scene.plugin_data.get("motion")


def test_examples_validate_in_the_panel():
    if SKIP:
        print("skip:", SKIP); return
    win = _window()
    panel = _state["panel"]
    for key in ("chaise", "porte", "oiseau", "draisine"):
        panel.set_source(key)
        m = panel.mechanism()
        unit = math.degrees if m.driver[0].kind == "pivot" else (lambda v: 1000 * v)
        assert abs(panel.drv_hi.value() - round(unit(m.driver[2]), 1)) < 0.051
        assert panel.full_turn() == (key == "oiseau")      # the bird has a crank
        panel.pas.setValue(21)
        panel._action("validate")
        assert panel.report["ok"], (key, panel.report["problems"])
        panel.buttons["trajectory"].setChecked(True)
        panel._action("trajectory")
        assert panel.trace_pts and all(len(t) == 21 for t in panel.trace_pts)
        panel.buttons["trajectory"].setChecked(False)
        panel._action("trajectory")


def test_screenshot():
    """Panel + overlay rendered into a PNG (catalog screenshot draft)."""
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QImage, QPainter, QColor
    from PySide6.QtWidgets import QApplication
    win = _window()
    panel = _state["panel"]
    panel.set_source("chaise")
    panel.pas.setValue(41)
    panel._action("validate")
    panel.buttons["trajectory"].setChecked(True)
    panel._action("trajectory")
    panel.slider.setValue(450)
    vp = win.viewport
    cam = vp.camera
    try:
        from PySide6.QtGui import QVector3D as V
        view = os.environ.get("MOTION_VIEW", "front")   # front = x-z plane
        cam.set_view(view)
        cam.perspective = False
        cam.target = V(0.12, 0, 0.42)
        cam.distance = 2.6
    except Exception:
        pass
    panel.app.show_panel(_state["dock"])
    QApplication.processEvents()
    out = os.environ.get("MOTION_SHOT")
    if out:
        img = win.grab().toImage()
        # the GL viewport may not render offscreen: draw the overlay ourselves
        frame = QImage(vp.size(), QImage.Format_ARGB32)
        frame.fill(QColor("#f4f4f2"))
        p = QPainter(frame)
        p.setRenderHint(QPainter.Antialiasing)
        panel._force_paint = True
        panel.paint(vp, p)
        p.end()
        pos = vp.mapTo(win, vp.rect().topLeft())
        p = QPainter(img)
        p.drawImage(pos, frame)
        p.end()
        img.save(out)
    assert panel.trace_pts


def test_detection_finds_the_joints_of_imported_assemblies():
    """Détecter… on STEP assemblies from another CAD kernel, read through
    IngeTrazo's own groups (faces + soft edges): a four-bar, gears, a
    slide. The proposals become a working mechanism in one undo step."""
    if SKIP:
        print("skip:", SKIP); return
    sys.path.insert(0, str(REPO))
    win = _window()
    panel = _state["panel"]
    results = {}
    for name in ("quadrilatere", "engrenages", "coulisse"):
        _reset(win, panel)
        scene = win.viewport.scene
        scene.groups[:] = [g for g in scene.groups if getattr(g, "billboard", False)]
        scene.version += 1
        asm, groups = panel.import_step(str(HERE / "data" / f"detect_{name}.step"))
        parts, props, frame = panel.scan_model(groups)
        kinds = sorted(x.kind for x in props)
        results[name] = (parts[frame][0], kinds)
        made = panel.apply_scan(parts, frame, props)
        piv = [j for j in panel.data["joints"] if j["kind"] in ("pivot", "slider")]
        lo, hi = (0.0, 1.0) if piv[0]["kind"] == "pivot" else (0.0, 0.05)
        panel._store(doc_add_driver(panel.data, piv[0]["name"], lo, hi))
        m = panel.mechanism()
        rep = __import__("motion.collide", fromlist=["validate"]).validate(m, 11, 0.0)
        assert all(r["ok"] for r in rep["steps"]), name
        dof = __import__("motion.solver", fromlist=["dof"]).dof(m, rep["steps"][3]["x"])
        assert dof == 1, (name, dof)
        if name == "engrenages":
            pv = [j for j in m.joints if j.kind == "pivot"]
            x = rep["steps"][-1]["x"]
            r = m.continuous_value(pv[1], x) / m.continuous_value(pv[0], x)
            assert abs(abs(r) - 20 / 30) < 1e-6 and r < 0, r
    assert results["quadrilatere"] == ("Socle", ["pivot"] * 4)
    assert results["engrenages"] == ("Platine", ["gears", "pivot", "pivot"])
    assert results["coulisse"] == ("Rail", ["slider"])


def test_materials_give_masses_from_ingetrazo_faces():
    """Mass of imported solids, from IngeTrazo's own faces (planar faces
    with holes included) and the material picked in the panel, against
    OpenCascade's values."""
    if SKIP:
        print("skip:", SKIP); return
    import json
    sys.path.insert(0, str(REPO))
    facts = json.load(open(HERE / "data" / "masse_facts.json"))
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    got = {}
    for name, key, rho, tol in (("equerre", "chene", 710, 1e-6), ("tube", "acier", 7850, 0.015)):   # float32 positions
        asm, groups = panel.import_step(str(HERE / "data" / f"masse_{name}.step"))
        scene.selection.clear()
        scene.selection.update(groups)
        panel._do_fixed(name == "equerre")
        body = panel.data["bodies"][-1]["name"]
        hit = panel.tree.findItems(body, Qt_MatchExactly())
        panel.tree.setCurrentItem(hit[0])
        panel.mat_combo.setCurrentIndex(panel.mat_combo.findData(key))
        panel._material_chosen()
        mp = panel.mechanism().masses[body]
        f = facts[name]
        assert abs(mp["mass"] / (rho * f["volume"]) - 1) < tol, (name, mp["mass"], rho * f["volume"])
        assert np.linalg.norm(mp["com"] - f["centre"]) < 1e-6, name
        got[name] = mp
    assert "kg" in panel.mass_label.text() and "centre de gravité" in panel.mass_label.text()
    # undo takes the material back
    win._on_undo()
    assert panel.data["materiau"].get(panel.data["bodies"][-1]["groups"][0]) is None


def Qt_MatchExactly():
    from PySide6.QtCore import Qt
    return Qt.MatchExactly


def test_dynamics_of_a_drop_leaf_from_the_model():
    """The drop leaf in oak, let go: it falls on its pivot, is caught by a
    stop at 60° with a damper, the report gives the energy; the panel's
    new joints and fields go into the document."""
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    sys.path.insert(0, str(REPO))
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    scene = win.viewport.scene
    table = _box_group("Plateau chêne", (-0.4, -0.2, -0.009), (0, 0.2, 0.009))
    leaf = _box_group("Abattant chêne", (0.0, -0.2, -0.009), (0.6, 0.2, 0.009))
    scene.groups += [table, leaf]
    hinge = scene.mesh.add_edge(V(0, -0.2, 0), V(0, 0.2, 0))
    win.viewport.notify_scene_changed()
    _select(win, table); panel._action("fixed")
    _select(win, leaf); panel._action("moving")
    _select(win, hinge); panel._action("pivot")
    d = panel.app.document_data()
    assert set(d["materiau"].values()) == {"chene"}            # guessed from the names
    it = panel.tree.findItems("pivot1", Qt_MatchExactly())[0]
    panel.tree.setCurrentItem(it)
    panel.lim_on.setChecked(True)
    panel.lim_lo.setValue(-60)                                 # a stay at 60°
    panel.lim_hi.setValue(60)                                  # either way down
    panel.c_spin.setValue(0.5)
    panel._apply_joint()
    j = panel.app.document_data()["joints"][0]
    assert j["damper"] == 0.5 and j["limits"][0] == pytest_approx(-math.pi / 3)
    panel.dur.setValue(2.0)
    panel.dyn_on.setChecked(True)                               # one undo step
    assert panel.app.document_data()["simulation"]["active"]
    panel._action("validate")
    txt = panel.out.toPlainText()
    assert "Simulation dynamique de 2.0 s" in txt, txt
    m = panel.mechanism()
    x = panel.report["steps"][-1]["x"]
    pv = m.joints[0]
    th = m.joint_value(pv, x)
    assert abs(abs(th) - math.pi / 3) < 0.01, th                # resting on its stay
    # a ball joint from the «Autres liaisons» menu: around a small circle
    ring = [scene.mesh.add_edge(V(0.6 + 0.01 * math.cos(a), 0.01 * math.sin(a), 0),
                                V(0.6 + 0.01 * math.cos(a + 0.5), 0.01 * math.sin(a + 0.5), 0))
            for a in np.arange(0, 2 * math.pi, 0.5)]
    win.viewport.notify_scene_changed()
    _select(win, *ring)
    panel._other_joint("ball")
    kinds = [k["kind"] for k in panel.app.document_data()["joints"]]
    assert kinds == ["pivot", "ball"], kinds


def doc_add_driver(data, joint, lo, hi):
    from motion import document
    return document.add_driver(data, joint, lo, hi)


def teardown_module(module=None):
    win = _state.pop("win", None)
    if win is not None:
        win._saved_version = win.viewport.scene.version
        win.close()
        win.deleteLater()


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
    teardown_module()


def test_screw_and_the_other_new_tools_from_the_panel():
    # a nut on a vertical screw (pitch 2 mm) turned two turns rises 4 mm;
    # the other three new tools make their joints from the same kind of
    # selection
    if SKIP:
        print("skip:", SKIP); return
    from PySide6.QtGui import QVector3D as V
    win = _window()
    panel = _state["panel"]
    _reset(win, panel)
    assert len(panel.buttons) == 16
    for n in ("annular", "pointplane", "planar", "screw"):
        assert panel.buttons[n].isEnabled() and panel.buttons[n].text()
    scene = win.viewport.scene
    frame = _box_group("Bâti", (-0.1, -0.1, -0.02), (0.1, 0.1, 0.0))
    nut = _box_group("Écrou", (-0.01, -0.01, 0.05), (0.01, 0.01, 0.06))
    scene.groups += [frame, nut]
    axis = scene.mesh.add_edge(V(0, 0, 0.04), V(0, 0, 0.07))
    win.viewport.notify_scene_changed()
    _select(win, frame); panel._action("fixed")
    _select(win, nut); panel._action("moving")
    panel.ask_pitch = lambda: 0.002
    _select(win, axis); panel._action("screw")
    j = panel.app.document_data()["joints"][0]
    assert j["kind"] == "screw" and abs(j["pitch"] - 0.002) < 1e-12
    it = panel.tree.findItems(j["name"], Qt_MatchExactly())[0]
    panel.tree.setCurrentItem(it)
    panel.drv_lo.setValue(0)
    panel.drv_hi.setValue(720)
    panel._action("driver")
    panel._action("validate")
    assert panel.report["ok"], panel.report["problems"]
    panel.slider.setValue(1000)
    m = panel.mechanism()
    b = m.bodies[0]
    assert abs(m.slide_value(m.joints[0], panel.x) - 0.004) < 1e-7
    # the three others, each replacing the last
    for kind in ("annular", "pointplane", "planar"):
        panel.app.set_document_data(dict(panel.app.document_data(), joints=[], driver=None,
                                         drivers=[]))
        panel.set_source("model")
        _select(win, axis); panel._action(kind)
        js = panel.app.document_data()["joints"]
        assert [x["kind"] for x in js] == [kind], (kind, panel.out.toPlainText())
        from motion import solver
        assert solver.dof(panel.mechanism()) == {"annular": 4, "pointplane": 5, "planar": 3}[kind]


def test_english_panel_and_fields_per_joint_kind():
    """With IngeTrazo in English, Motion speaks English; the joint box shows
    only the fields the chosen joint uses."""
    if SKIP:
        print("skip:", SKIP); return
    import importlib
    win = _window()
    panel = _state["panel"]
    pkg = type(panel).__module__.rsplit(".", 1)[0]
    lang = importlib.import_module(pkg + ".lang")
    lang.set_language("en")
    try:
        other = type(panel)(panel.app)
        texts = {n: b.text() for n, b in other.buttons.items()}
        assert texts["fixed"] == "Fixed part" and texts["annular"] == "Annular"
        assert texts["driver"].startswith("Driver") and texts["validate"] == "Validate"
        assert other.tree.headerItem().text(0) == "Name"
        assert other.step_btn.text() == "Import STEP…"
        assert "shafts in bores" in other.scan_btn.toolTip()
        other.deleteLater()
        rep = importlib.import_module(pkg + ".collide")
        assert "between" in lang.T("{p} % : jeu {d} mm < {j} mm entre {a} ({ga}) et {b} ({gb})",
                                   p=1, d=2, j=3, a="a", ga="g", b="b", gb="h")
    finally:
        lang.set_language(None)
    # fields per kind (French panel)
    test_screw_and_the_other_new_tools_from_the_panel()      # leaves a planar joint
    form = panel.joint_form
    def shown(w):
        return form.isRowVisible(w)
    j = panel.app.document_data()["joints"][0]
    panel.tree.setCurrentItem(panel.tree.findItems(j["name"], Qt_MatchExactly())[0])
    assert j["kind"] == "planar"
    assert not shown(panel.lim_row) and not shown(panel.drv_row) and not shown(panel.tq_row)
    assert not shown(panel.k_row) and shown(panel.combo_a)
    # a part: nothing of the joint box
    b = panel.app.document_data()["bodies"][0]["name"]
    panel.tree.setCurrentItem(panel.tree.findItems(b, Qt_MatchExactly())[0])
    assert not shown(panel.combo_a) and not shown(panel.apply_btn)
    # a pivot: everything
    test_drop_leaf_built_from_the_model()
    panel.tree.setCurrentItem(panel.tree.findItems("pivot1", Qt_MatchExactly())[0])
    for w in (panel.lim_row, panel.drv_row, panel.motor_on, panel.k_row, panel.tq_row):
        assert shown(w)
    assert panel.k_spin.isVisibleTo(panel) and panel.c_spin.isVisibleTo(panel)


def test_examples_copied_into_the_model_validate():
    """« Copier dans le modèle »: real groups, the mechanism in the
    document, the same validation as the example; not offered for chains."""
    if SKIP:
        print("skip:", SKIP); return
    win = _window()
    panel = _state["panel"]
    for key in ("chaise", "porte", "oiseau", "pendule_double"):
        _reset(win, panel)
        panel.set_source(key)
        assert panel.copy_btn.isVisibleTo(panel)
        data = panel.copy_example(confirm=True)
        assert panel.source == "model"
        uids = {u for b in data["bodies"] for u in b["groups"]}
        assert uids <= {g.uid for g in win.viewport.scene.groups}
        assert panel.problems == [], (key, panel.problems)
        if key == "pendule_double":
            m = panel.mechanism()
            assert {round(v["mass"], 4) for v in m.masses.values() if v["mass"]} == {0.0767, 0.0639}
            continue
        panel.pas.setValue(21)
        panel._action("validate")
        assert panel.report["ok"], (key, panel.report["problems"])
    for key in ("chaine", "chaine_souple", "draisine"):
        panel.set_source(key)
        assert not panel.copy_btn.isVisibleTo(panel)
    _reset(win, panel)
