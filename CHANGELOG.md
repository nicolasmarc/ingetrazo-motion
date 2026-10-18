# Changelog

## 0.7.0 — 2026-10-18

### Added
- **Roller chain with a slack strand** (`motion.chain`): a chain on two sprockets
  (teeth, pitch, number of links). The driving sprocket pulls the upper strand, which
  stays straight; the lower strand gets the length left and hangs as a catenary tangent
  to both pitch circles. Every link is a part (drawn, animated, checked for clearance);
  the pins stay in the tooth gaps, and the chain turns the driven sprocket. Readings:
  sag (and % of the centre distance), centre distance; a chain too short is reported.
  Mechanism parts placed by such a rule: `Mechanism.followers`.
- **Example «chaîne détendue et tendeur»** (`samples.chaine`): 1/2 in chain, 17 and 25
  teeth, 88 links, a crank turned 3 times and a tensioner (second motor) that takes the
  sag from about 10 % to 3 % of the centre distance. Tested against closed forms (pitch
  diameters, every link one pitch, tangency, speed ratio, pins in the tooth gaps) and an
  independent catenary solution.
- **Dynamics, first step: a supple chain** (`Chain.dynamic`, `Chain.advance`): in a
  video (and while playing) the slack strand is computed in time — pins with weight and
  inertia, damping (1/s), every link one pitch at every sub-step (the strand's
  constraints solved at once, a tridiagonal system; velocities projected the same way so
  the swing is not braked by the scheme), kept off the sprockets, guided into the tooth
  gaps. Checked: a chain running steadily keeps its shape at rest (a chain moving along
  its own catenary needs no sideways force), a tapped strand swings then settles, and its
  sway frequency matches string theory sqrt(g c) / L within 3 %.
- **Example «chaîne souple (dynamique)»** (`samples.chaine_souple`): the driven
  sprocket's bearing moved by an eccentric geared to the crank (3 times per turn,
  ±2.5 mm), the crank turned by hand (from rest, uneven speed, then stopped): the slack
  strand lags, whips and settles. An example may give its own timing (`allure`).
- **Détecter… — automatic joints** (`motion.detect`, panel button next to «Importer
  STEP…»): scans the parts (the selection, else the whole model) and proposes the joints:
  a shaft in a bore of another part (coaxial, bore as large or slightly larger, overlapping)
  → pivot, several coaxial fits making one; two or more flat contacts with non-parallel
  normals → slider along the only free direction; two toothed parts on parallel pivots
  whose tip circles mesh → gears, ratio -z1/z2 from the teeth counted on the outline; a
  separate pin through two parts → one pivot, the pin going with the part it is pressed
  in. The frame proposed is the part the most joints lean on (then the largest). Each
  proposal shows Ø, play, bearing length and a confidence; nothing is created until ticked,
  then bodies and joints come in one undo step. Works on IngeTrazo's own faces (curved
  surfaces = faces joined by soft edges), so on drawn models as on STEP imports.
  Tested on four assemblies written by an independent CAD kernel (four-bar, gears, slide,
  hinge with a separate pin): joints in place, the four-bar's rocker swing against its
  closed form, the gear ratio, one degree of freedom each.
- Document: `coupling` joints (gears) are stored and rebuilt.

## 0.6.1 — 2026-10-18
- STEP reader: several named solids in one product become several parts (as the STEP export
  extension writes them). `motion/stepread.py` is the same file as
  `stepexport/stepread.py` in the STEP extension (ingetrazo-step), kept in step.
- STEP reader: the component name typed by the designer ("Marche") wins over a solid name
  the CAD program gave on its own ("Solide", "Body1"); accented names decoded.
- Validation report: degrees of freedom counted over several poses (most frequent value);
  a single mid-run pose read 5 instead of 1 on the rail car after whole crank turns.
- Screenshot: the rail-car example in IngeTrazo.

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
