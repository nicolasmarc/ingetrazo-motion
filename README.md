# Motion — mechanisms for IngeTrazo

Kinematic validation of simple furniture and building mechanisms: drop-leaf,
drawer, folding chair, sectional garage door. Define the parts and joints on
your model, play the motion with a slider, and check it: collisions, joint
stops and the minimum clearance between parts.

*Validation cinématique de mécanismes simples (abattant, tiroir, chaise
pliante, porte sectionnelle) : pièces et liaisons définies sur le modèle,
lecture au curseur, contrôle des collisions, des butées et du jeu minimal.*

Clean-room, from scratch, GPL-3.0-or-later. Pure Python + numpy core, tested
against closed-form results.

![Motion panel](screenshots/motion.png)

## Install

IngeTrazo ▸ Extensions ▸ Manage extensions… ▸ **Motion**, or unzip
`dist/motion-<version>.zip` into the plugins folder (`%APPDATA%\ingetrazo\plugins`),
which gives `plugins\motion\__init__.py`. Restart IngeTrazo; the **Motion**
tab appears in the side tray, and Extensions ▸ Motion has the examples.

## Use

| Icon | Select first | Result |
|---|---|---|
| Fixed part | one or more groups | they form the frame |
| Moving part | one or more groups | they move together |
| Pivot | ONE edge or construction line on the axis | hinge between the two nearest parts |
| Slider | ONE edge along the travel | slide between the two nearest parts |
| Guide/rail | the rail's edges (lines + arc segments) and a short stroke across the rail at each roller | one roller joint per stroke, on the nearest moving part |
| Driver 0–100 % | a joint in the list, and its travel (° or mm) | that joint drives the motion |
| Validate | — | 0 → 100 %: convergence, collisions, stops, gaps under *Jeu mini* |
| Trajectory | edges whose end points to trace (else the joint points) | dashed paths in the view |

**Exporter la vidéo…** (under the playback slider) writes a split-screen
video: the IngeTrazo view on the left, a shaded animation with the readings
on the right. MP4 needs an encoder: Qt Multimedia if your IngeTrazo has it,
else `ffmpeg` (found on the PATH, or point to `ffmpeg.exe` once in the
dialog); without either, the video is saved as AVI (Motion-JPEG).

The animation is drawn over the model: the groups themselves do not move,
so playing adds no undo steps. The mechanism is saved in the `.igz`; every
edit is one undo step. Collision boxes are fitted to each group.

## What the core does

- Rigid bodies with 6-parameter poses; pivot, slider and guide/rail
  (a point on a path of lines and circular arcs) with optional stops.
- Levenberg–Marquardt position solver for open chains and closed loops;
  degrees of freedom from the constraint Jacobian's rank.
- Oriented boxes: separating-axis interference, **exact** box-to-box
  distance (vertex/box and edge/edge), minimum-clearance check along the
  motion. Box pairs of joined parts that already touch when drawn are the
  joint; the rest of the two parts is still checked.
- Examples: folding chair with crossed legs, sectional door (any number of
  panels, two tracks).

## Tests

```
python tests/test_core.py        # V0.1: drop-leaf, drawer, four-bar
python tests/test_chair.py       # folding chair vs closed forms
python tests/test_clearance.py   # exact distances, gaps along a motion
python tests/test_guide.py       # rail, trammel ellipse, sectional door
python tests/test_document.py    # what the panel stores
python tests/test_video.py       # video writers (AVI checked byte by byte)
python tests/test_panel_ingetrazo.py   # the panel in a real IngeTrazo
                                       # window (needs PySide6 and an
                                       # IngeTrazo checkout: INGETRAZO_SRC)
```
All are pytest-compatible. Build the package: `python tools/build_zip.py`
(prints the sha256 for `catalog/motion.toml`).

## Licence

GPL-3.0-or-later — see `LICENSE`.
