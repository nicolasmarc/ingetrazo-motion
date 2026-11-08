"""The realistic video: the same motion, rendered by Blender.

IngeTrazo (0.6 and later) renders a still in Blender (core.render_blender:
finding Blender, exporting the model with its materials, the sky, the
lights and the ground). Motion reuses all of it and adds the motion: the
model is exported once at the drawn pose, each moving part a node of its
own, and ``blender_anim.py`` places them frame after frame. The images
then go through the same writer as the split-screen video."""
import json
import math
import os
import queue
import shutil
import subprocess
import threading
import time
from pathlib import Path

import numpy as np

#: Engines offered, per quality: samples per image (Cycles denoises).
ENGINES = ("eevee", "cycles")
SAMPLES = {"eevee": 32, "cycles": 32}


def _rb():
    try:
        from core import render_blender as rb
    except Exception:
        return None
    import inspect
    try:
        if "serve" not in inspect.signature(rb.write_job).parameters:
            return None                       # older IngeTrazo: no scene builder to reuse
    except (TypeError, ValueError):
        return None
    return rb


def available():
    """IngeTrazo can drive Blender (0.6 and later)."""
    return _rb() is not None


def saved_path():
    rb = _rb()
    if rb is None:
        return ""
    try:
        from PySide6.QtCore import QSettings
        return str(QSettings().value(rb.SETTINGS_KEY, "") or "")
    except Exception:
        return ""


def remember(path):
    """The Blender chosen here is IngeTrazo's too (same setting)."""
    rb = _rb()
    if rb is None:
        return
    from PySide6.QtCore import QSettings
    QSettings().setValue(rb.SETTINGS_KEY, str(path))


def find():
    """IngeTrazo's BlenderFound, or None."""
    rb = _rb()
    if rb is None:
        return None
    try:
        return rb.find_blender(saved_path() or None)
    except Exception:
        return None


def part_matrices(m, x, data_bodies):
    """``{"MOT_<uid>": 4×4}``: each moving group's motion from its drawn
    place at pose ``x`` (rotation about the body's centre, then shift)."""
    by_name = {b.name: b for b in m.bodies}
    out = {}
    for b in data_bodies:
        body = by_name.get(b["name"])
        if b.get("fixed") or body is None:
            continue
        q = m.pose(x, body)
        R = np.asarray(body.rot(q), float)
        c = np.asarray(body.center, float)
        t = c + np.asarray(q[:3], float) - R @ c
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = t
        rows = M.tolist()
        for u in b["groups"]:
            out[node_name(u)] = rows
    return out


def node_name(uid):
    return "MOT_" + str(uid).replace("-", "")[:16]


def document_ambience(app):
    """The document's ambience, sun and lights (IngeTrazo's Render panel)."""
    rb = _rb()
    try:
        raw = app.document_data({}) or {}
    except TypeError:
        raw = app.document_data() or {}
    except Exception:
        raw = {}
    if not isinstance(raw, dict):
        raw = {}
    amb = raw.get("ambience", "day")
    try:
        sun = max(0.1, min(2.0, float(raw.get("sun_scale", 1.0))))
    except (TypeError, ValueError):
        sun = 1.0
    try:
        lights = rb.clean_lights(raw.get("lights", []))
    except Exception:
        lights = []
    return (amb if amb in rb.AMBIENCES else "day"), sun, lights


def write_scene(app, scene, camera, work, moving_uids, width, height, engine,
                frames):
    """job.json, model.glb, render_scene.py, blender_anim.py and anim.json
    in ``work``; returns the job's path. The groups are renamed for the
    export only (their names come back), the face-me figures stay out."""
    rb = _rb()
    from formats.gltf import save_glb
    amb, sun, lights = document_ambience(app)
    renamed, hidden = {}, []
    try:
        for g in scene.groups:
            if g.uid in moving_uids:
                renamed[g] = g.name
                g.name = node_name(g.uid)
            if getattr(g, "billboard", False) and not g.hidden:
                g.hidden = True
                hidden.append(g)
        job = rb.write_job(scene, camera, work, engine=engine, quality=1,
                           width=width, height=height, ground=True,
                           ambience=amb, lights=lights, sun_scale=sun, serve=True)
        # write_job merges the model into one mesh (its figures face the
        # camera): again, one node per group, so that the parts can move
        save_glb(scene, work / "model.glb")
    finally:
        for g, nm in renamed.items():
            g.name = nm
        for g in hidden:
            g.hidden = False
    jd = json.loads(job.read_text(encoding="utf-8"))
    jd["samples"] = SAMPLES.get(engine, 32)
    job.write_text(json.dumps(jd, indent=1), encoding="utf-8")
    shutil.copyfile(rb.render_script(), work / "render_scene.py")
    shutil.copyfile(Path(__file__).with_name("blender_anim.py"), work / "blender_anim.py")
    (work / "anim.json").write_text(json.dumps({"frames": frames}), encoding="utf-8")
    return job


def command(found, job):
    work = job.parent
    return [*found.command, "-b", "--factory-startup",
            "--python", str(work / "blender_anim.py"), "--",
            str(job), str(work / "anim.json"), str(work / "frames")]


def work_dir():
    rb = _rb()
    try:
        base = rb.work_base()
    except Exception:
        import tempfile
        base = Path(tempfile.gettempdir()) / "IngeTrazo" / "render"
    return base / f"motion-{os.getpid()}-{int(time.time())}"


def run(argv, n, progress=None, log=None):
    """Blender to the end; ``progress(done, n)`` after each image (and in
    between, from its samples), False from it stops Blender. Returns True
    when all the images are there, None when cancelled; raises with
    Blender's last lines when it failed."""
    rb = _rb()
    try:
        env = rb.clean_env()
    except Exception:
        env = None
    kw = {}
    if os.name == "nt":
        kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, env=env, text=True,
                            encoding="utf-8", errors="replace", **kw)
    lines = queue.Queue()

    def pump():
        for line in proc.stdout:
            lines.put(line)
        lines.put(None)

    threading.Thread(target=pump, daemon=True).start()
    tail, done, ended = [], 0, False
    while not ended:
        try:
            line = lines.get(timeout=0.1)
        except queue.Empty:
            line = ""
        if line is None:
            ended = True
            break
        if line:
            tail = (tail + [line.rstrip()])[-25:]
            if log is not None:
                log.write(line)
            parts = line.split()
            if parts[:2] == ["MOTION", "frame"]:
                done = int(parts[2])
            else:
                frac = rb.progress_of(line) if hasattr(rb, "progress_of") else None
                if frac is not None and progress is not None:
                    # within an image: a step that moves, not a stuck bar
                    if progress(done + min(frac, 0.99), n) is False:
                        _stop(proc)
                        return None
                    continue
        if progress is not None and progress(done, n) is False:
            _stop(proc)
            return None
    proc.wait()
    if done < n:
        raise RuntimeError("Blender : " + "\n".join(t for t in tail if t.strip())[-1500:])
    return True


def _stop(proc):
    try:
        proc.kill()
    except Exception:
        pass
    try:
        proc.wait(timeout=5)
    except Exception:
        pass


def caption(img, title, sub=None):
    """A discreet title at the bottom left of a rendered image."""
    from PySide6.QtCore import Qt, QRectF
    from PySide6.QtGui import QColor, QFont, QPainter
    p = QPainter(img)
    try:
        p.setRenderHint(QPainter.Antialiasing)
        h = img.height()
        f = QFont()
        f.setPixelSize(max(12, int(h * 0.024)))
        p.setFont(f)
        text = title if not sub else f"{title}   ·   {sub}"
        fm = p.fontMetrics()
        pad = h * 0.012
        w = fm.horizontalAdvance(text) + 2 * pad
        r = QRectF(h * 0.025, h - h * 0.025 - fm.height() - 2 * pad, w, fm.height() + 2 * pad)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 0, 0, 110))
        p.drawRoundedRect(r, pad, pad)
        p.setPen(QColor(255, 255, 255, 230))
        p.drawText(r, Qt.AlignCenter, text)
    finally:
        p.end()
    return img
