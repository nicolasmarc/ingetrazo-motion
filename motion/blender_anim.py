"""Run INSIDE Blender (never imported by Motion): IngeTrazo's own scene
builder (render_scene.py, next to the job) sets the model, its materials,
the camera, the sky, the lights and the ground; instead of one image, each
frame of ``anim.json`` places the moving parts and is rendered to
``<outdir>/f0000.png``…

    blender -b --factory-startup --python blender_anim.py -- job.json anim.json outdir

``anim.json``: ``{"frames": [{"M": {"MOT_<uid>": 4×4 row-major}}, …]}`` —
the motion of each part from where it is drawn (the model's coordinates).
Lines starting with ``MOTION`` tell Motion how far it is."""
import json
import os
import sys
import time
import types

import bpy
from mathutils import Matrix


def _say(*parts):
    print("MOTION", *parts, flush=True)


def _scene_builder(path):
    """render_scene.py's functions, without running its main()."""
    src = open(path, encoding="utf-8").read()
    body = src.rstrip()
    if body.endswith("main()"):
        body = body[: -len("main()")]
    mod = types.ModuleType("render_scene")
    mod.__file__ = path
    exec(compile(body, path, "exec"), mod.__dict__)
    return mod


def _animate(job_path, anim_path, outdir):
    here = os.path.dirname(os.path.abspath(job_path))
    rs = _scene_builder(os.path.join(here, "render_scene.py"))
    frames = json.load(open(anim_path, encoding="utf-8"))["frames"]
    os.makedirs(outdir, exist_ok=True)

    def play(scene, figures):
        # the scene is built (render_scene.main stops here when serving):
        # the parts as exported are the drawn pose; each frame moves them
        parts = {}
        for o in bpy.data.objects:
            key = o.name.split(".")[0]
            if not key.startswith("MOT_"):
                continue
            p = o.parent
            if p is not None and p.name.split(".")[0].startswith("MOT_"):
                continue                      # moves with its parent
            parts.setdefault(key, []).append((o, o.matrix_world.copy()))
        _say("parts", len(parts))
        n = len(frames)
        r = scene.render
        for i, fr in enumerate(frames):
            t0 = time.time()
            for key, M in fr["M"].items():
                for o, base in parts.get(key, ()):
                    o.matrix_world = Matrix(M) @ base
            bpy.context.view_layer.update()
            r.filepath = os.path.join(outdir, f"f{i:04d}.png")
            bpy.ops.render.render(write_still=True)
            _say("frame", i + 1, n, f"{time.time() - t0:.1f}")
        _say("done")

    rs.serve = play
    rs.main()


if __name__ == "__main__":
    argv = sys.argv[sys.argv.index("--") + 1:]
    _animate(argv[0], argv[1], argv[2])
