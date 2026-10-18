# Changelog

## 0.6.1 — 2026-10-18
- STEP reader: several named solids in one product become several parts (as the STEP export
  extension writes them). `motion/stepread.py` is the same file as
  `stepexport/stepread.py` in the STEP extension (ingetrazo-step), kept in step.
- STEP reader: the component name typed by the designer ("Marche") wins over a solid name
  the CAD program gave on its own ("Solide", "Body1"); accented names decoded.

## 0.6.0 — 2026-10-17

### Added
- **Several motors** (`Mechanism.add_driver`): every motor runs on the same
  0..100 % from its own start to its own end — a different travel is a
  different speed, a different start a phase. Panel: «Moteur» adds a motor
  (or updates its travel), a «Moteur» tick box per joint removes it; every
  motor is marked in the list and read out in the video. Document: a list
  of motors, files with the single motor of 0.1–0.5 still open.
  Tested on two four-bars, one at twice the speed and 90° ahead.
- **STEP assemblies** (`motion.stepread`, pure Python, from ISO 10303-21):
  product tree, placements, units, colours; planes, cylinders, cones,
  spheres, tori, linear extrusions, revolutions and B-spline surfaces
  meshed to a chord tolerance (constrained Delaunay in the face's
  parameters, seams and poles handled), closed parts watertight. Checked
  against files written by an independent CAD kernel (CadQuery /
  OpenCascade): volumes within 0.3 %, exact bounding boxes, placements of
  turned parts, colours.
- **Importer STEP…** (panel and Extensions ▸ Motion): one group per part,
  named and painted, in one undo step; the parts are left selected, ready
  to become fixed or moving parts. 80 parts / 110 000 faces in about 28 s.

### Changed
- Solver: one residual row per motor; the sweep follows every turning
  motor in steps of at most 0.4 rad.

## 0.5.0 — 2026-10-17

### Added
- **Coupling joint** (`Mechanism.couple`): a fixed ratio between two
  joints — gears (two pivots, -z1/z2), rolling without slip (wheel pivot
  and slider, -R), rack and pinion. Pivot angles are counted continuously
  (whole turns), so a driver may run several turns.
- **Hand-pumped rail car** example (`samples.draisine`): walking beam on
  an A-frame, connecting rod, overhung crank on an intermediate shaft,
  40/20 gears to the rear axle, both axles rolling on a metre-gauge track;
  wheels and toothed gears drawn as real outlines. Tested against closed
  forms over three crank turns (crank pin, rod end, beam angle, axle
  angles, travel), no slip at both contact points, beam stroke ends.
  The validation caught the first draft's crank shaft crossing the rod.
- Video: the camera can follow a part (the car), in both panes; colours
  per box (steel rails, wooden sleepers); a multi-turn driver runs once,
  at constant speed.
- Icon «Couplage» (two meshing gears) for the joints list.

### Changed
- Collisions and clearances: box pairs are sorted by their axis-aligned
  bounds and the exact distance is only computed while it can still beat
  the best; joined-pair data is cached — about 20 times faster on the rail
  car (80 s -> 4 s for 25 steps).
- `solver.sweep` follows a pivot in steps of 0.4 rad at most, whatever the
  number of recorded steps.

## 0.4.0 — 2026-10-16

### Added
- **Flapping bird** example (`samples.oiseau`): one crank turning full
  revolutions drives a vertical slider; two rods beat both wings; each wing
  has an arm and a hand, the hand held by a tie rod from the body so it
  folds on the upstroke and spreads on the downstroke (about x1.5).
  Tested against closed forms over a full turn (slider height, both
  circle cuts), mirror symmetry, stroke ends at the slider's ends.
- Cranks: a pivot driver may turn past ±180° (whole revolutions); the
  panel plays it round and round, the video runs it at constant speed.
- Video: the IngeTrazo pane now shows the model rendered by IngeTrazo with
  the moving parts drawn shaded at their pose by Motion (the moving groups
  are left out of that shot only — no undo step, the document is not
  touched); example-specific readings (arm, hand, fold angles; slot
  travel); an example may choose its own view for the right pane; body
  colours.

### Changed
- Solver: the Jacobian is built body by body (only the joints a body takes
  part in), about 6 times faster on 11 bodies.
- Video: smallest gap and collisions are computed once at the validation
  steps (each frame shows its nearest step), about 3 times faster.

## 0.3.0 — 2026-10-15

### Added
- **Export de la vidéo / Video export** (panel button «Exporter la vidéo…»):
  split screen, the IngeTrazo view on the left (model + motion overlay,
  through the viewport's own off-screen render), a shaded animation with
  the readings (driver value, smallest gap, collisions) on the right;
  duration, 1920 × 1080 or 1280 × 720, there-and-back; cancellable.
- `motion.video`: MP4 through Qt Multimedia when present, else an
  `ffmpeg` executable (PATH, or pointed to once in the export dialog),
  else a pure-Python Motion-JPEG AVI so a video is always produced.
- `motion.render`: shaded QPainter view of a mechanism (group faces, or
  boxes) and the split-screen frame.

## 0.2.0 — 2026-10-13

### Added
- **Folding chair** example (`samples.chaise_pliante`): crossed legs, seat
  hinged at the front and sliding at the rear in a slot of the backrest
  upright, backrest rigid with its leg. Seat 45 cm high, 40 × 40 cm by
  default; every dimension is a parameter. Tested against closed forms:
  pivot and pin positions, slot travel and its exact maximum, flat fold.
- **Minimum clearance**: exact distance between oriented boxes,
  `collide.clearances`, `clearance_violations` and `validate` (0–100 %
  run with convergence, collisions, stops and gaps under a minimum).
- **Guide/rail** joint: a point of a part runs on a path of lines and
  circular arcs fixed to another part (`model.Path`, `Mechanism.guide`).
- **Sectional garage door** example (`samples.porte_sectionnelle`).
- Boxes can be turned in the drawn pose (`Box.R`, `model.bar`).
- **IngeTrazo panel**: the 8 tools (drawn in code), parts and joints list,
  driver travel, playback slider, validation report, trajectories, the
  two examples; the mechanism is kept in the document.

### Changed
- Joined parts: only the box pairs that touch in the drawn configuration
  are skipped (they are the joint); other boxes of the two parts are now
  checked. `ignore_joined="body"` gives the 0.1 behaviour.

## 0.1.0
- Rigid bodies, pivot and slider with stops, driver 0–100 %, closed-loop
  solver, degrees of freedom, interference between oriented boxes.
