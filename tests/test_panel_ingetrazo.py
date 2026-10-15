"""The panel inside a real IngeTrazo window (offscreen).

Needs PySide6 and an IngeTrazo source checkout: set INGETRAZO_SRC, or put
the clone next to this repository as ``../ingetrazo-upstream`` or
``../ingetrazo``. Skipped (not failed) when either is missing.
"""
import math, os, shutil, sys, tempfile
from pathlib import Path
import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
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
    _state.update(win=win, panel=dock.widget(), dock=dock)
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
    hinge = scene.mesh.add_edge(V(0, -0.2, 0), V(0, 0.2, 0))
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
    tip = body.world_point(m.pose(panel.x, body), (0.6, 0, 0))
    assert abs(tip[0] - 0.6 * math.cos(math.pi / 4)) < 1e-6
    assert abs(abs(tip[2]) - 0.6 * math.sin(math.pi / 4)) < 1e-6
    # each edit was one undo step
    win._on_undo()
    assert panel.app.document_data()["driver"] is None
    win._on_redo()
    assert panel.app.document_data()["driver"]["joint"] == "pivot1"


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
    assert abs(w_["distance"] - 0.01) < 1e-7


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
    for key in ("chaise", "porte", "oiseau"):
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
