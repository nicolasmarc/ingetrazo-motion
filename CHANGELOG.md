# Changelog

## 0.8.0 — unreleased

### Fixed
- **Opening a big model no longer freezes the window** (the spider, 133
  parts, ~300 000 faces): the masses were computed inside IngeTrazo's
  document-changed signal. They are now computed right after the window is
  drawn (wait cursor, « Calcul des masses… »), and the faces are read 3.5×
  faster (triangles and convex quads indexed directly; holes and other
  faces still go through IngeTrazo's triangulation, same area and volume).
- **The list of parts and joints can be enlarged**: drag the handle under
  the list to give it the room a mechanism with many joints needs.

### Added
- **Shocks between free parts** (`motion.impacts`, « Chocs » next to
  « Dynamique »): parts not held by a joint push each other back through
  their boxes (corners and points along every edge), with the friction and
  the restitution of their materials; fixed parts are obstacles. A penalty
  method (contact at 400 Hz, step 0.2 ms): dominoes, blocks, loose stops; the
  parts stay rigid. With shocks on, touching between free parts is no
  collision in the report, which gives the deepest overlap instead. Tested: a
  block resting on a table (sinks < 0.1 mm), a rebound of e² h, a block
  sliding v²/2μg before it stops, a domino knocking the next ones down,
  parts held by a pivot not knocking each other.
- **Balls**: a group whose points all lie on a sphere is a ball for the
  shocks — exact contact with another ball or a box, Hertz's law (f ∝ d^1.5).
  Tested: two equal balls exchanging their speed ((1 ± e)/2), a ball resting
  on a table; Newton's cradle, five steel balls on V strings: the last one
  leaves at 36° when the first falls from 40° (e 0.97, μ 0.1).
- **Slow motion** for a dynamic simulation's video (« Ralenti » ×1–20, the
  poses interpolated); a simulation without any motor can now be exported.
- Video: the IngeTrazo half showed the moving parts both where they were
  drawn and where they had moved (hidden parts were not seen hidden by
  IngeTrazo 0.5.9's draw batches); the Motion half let a large face (a table)
  cover the parts lying on it — such faces are now drawn in tiles.
- The dynamics handles many free parts quicker (rotations, masses and
  displacements computed for all the parts at once).
- **Four more standard joints**: linéaire annulaire (a ball in a bore: its
  centre stays on the axis, every rotation free), ponctuelle (a point on a
  plane), appui plan (planar) and hélicoïdale (screw and nut, pitch per turn,
  motor, springs, stops). Kinematics, dynamics, document, icons; the panel's
  grid is 4×4. Tested by hand: bead on a sloping wire, point on a slope with
  a sideways drift, block on a plane spinning about its normal, a nut falling
  on its screw ((m + I k²) z'' = -m g), a lead screw's motor torque
  (m g p / 2π), and an eccentric driving a leg through a ball in a bore.
- **Collisions on the real surfaces** (`motion.contact`): parts built from the
  model are checked by their faces — sampled points, k-d trees, then the exact
  signed distance to the triangles near contact. Resting contacts in the drawn
  position are the joint; gear pairs are not checked. Boxes stay for the
  examples. Tested: a rod inside a hollow casing (its box swallows the rod),
  a hinged lid opening and closing into its box, a face sliding 0.12 mm off a
  wall, a 3 mm gap read exactly.

### Fixed — STEP reading (faces drawn wrong on a real model)
- B-spline surfaces closed round one direction (tubes, ribs): a face across
  the seam was meshed the long way round (a sheet between the ribs); the
  seam is now crossed, bands between two rings are meshed as a strip, and
  surfaces closed the other way are turned round first.
- Planar faces with rows of lined-up holes: the cut to a hole could graze
  the next one, and holes stayed filled. Cuts now miss every edge and
  vertex, ears never step over a cut, and if the area still comes out wrong
  the face is cut in slices instead (never fails).
- The outline of a face is the loop that holds the others (some files flag
  a hole as the outer bound).
- Surfaces of linear extrusion: the swept curve is sampled whole (taken
  between one point and itself it went there and back, and gear flanks
  came out three times too big).
- Surfaces of revolution follow their own profile curve (not the straight
  line between the boundary points), with the right normal.
- Cones scaled by the radius where the face is, not the file's reference
  radius; a loop through the apex (or a sphere's pole) meets it twice.
- Spheres cut by a circle that is not round their axis (a ball on its stem)
  are meshed seen from above the face.
- Facets leaning more than 15° off a small curved surface are split (a
  1.5 mm ball at 0.2 mm was a few flat facets).
- Complex B_SPLINE_CURVE entities (name in REPRESENTATION_ITEM) are read:
  the 8 "unreadable" faces of the model.
- Checked face by face against OpenCascade on a 133-part assembly: from 493
  faces off by more than 3 % to 20, all of them thin tubes and small circles
  within the chord tolerance. Faster b-spline evaluation.

### Added (interface)
- **English**: Motion follows IngeTrazo's language (Window ▸ Language): French
  when IngeTrazo is in French, English otherwise (`MOTION_LANG=fr|en` forces
  one). Panel, tool names and tips, messages, validation report, video
  readings, examples, materials and default joint names.
- The «Selected joint» box shows only what the chosen joint uses (a ball
  joint: its damper; a lock or an annular joint: its two parts; a part:
  nothing).
- PLA in the materials (3D printing).

### Changed
- Video colours: the frame in light grey, each family of moving parts
  («Patte 1»…«Patte 8», «Roue …») in its own clear colour (a part's own
  colour still wins). `render.colours(mech)` gives them.
- Videos of big models keep their largest faces when thinned (every n-th
  face left holes).
- Groups whose faces are turned inward are turned outward for the collision
  check (signed volume).
- The panel tests' hinges sit on the top face: at mid-thickness the leaf's
  corner swings into the table, which the new check reports (rightly).

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
- «Jeu mini» and «Pas» are stored in the document when validating (one undo step):
  they used to be kept only in memory and came back at their old values with the file.
- Checked with IngeTrazo 0.5.9, with and without its experimental Rust engine: a
  model saved and reopened as lazy meshes keeps its mechanism, masses and report; a
  model without Motion parts stays lazy (Motion only touches the groups that are parts).
- **Materials** (`motion.materials`, the panel's «Matériau» row): a library of woods
  (chêne, hêtre, frêne, noyer, pin, épicéa, douglas, contreplaqué, MDF, OSB), metals
  (acier, inox, aluminium, laiton, bronze, fonte) and plastics (POM, PA 6, PEHD, ABS,
  PMMA), glass — density, Young's modulus, dry friction, restitution (reference averages);
  «Nouveau…» adds or corrects one for the document. A material is set per group, saved in
  the .igz, and guessed from the part's or its paint's name («Pied chêne», «Steel»).
- **Mass, centre of gravity, inertia** (`motion.massprops`) of every part from
  IngeTrazo's own faces (holes cut) and its materials: shown in the list and for the
  chosen part, centres of gravity drawn in the view (the usual quartered circle), masses
  and total in the validation report, groups without a material reported. Checked on a
  box (closed form) and on solids whose volume, centre and inertia OpenCascade computed:
  exact for flat faces, within 0.3 % for curved ones at 0.02 mm facets (an import at the
  default 0.2 mm reads small round parts about 1 % light).
- **Dynamics of the whole mechanism** (`motion.dynamics`): every part with its mass,
  centre of gravity and inertia; weight, joint friction, *speed* motors (a law in time:
  the torque or force they must give comes out) and *torque* motors (a torque or force:
  the mechanism answers); the joints' reactions. Newton-Euler at the centres of gravity,
  accelerations and reactions solved together (over-constrained loops accepted),
  Runge-Kutta 4, drift pulled back (Baumgarte); pivots differentiated analytically.
  Checked against hand-worked cases: free fall (exact), a tumbling block (angular
  momentum and energy kept), pendulum periods at 5°, 90° and 150° (elliptic integral,
  within 0.02 %), a double pendulum's energy, a block sliding down a slope, a flywheel and
  a geared pair under a torque, the torque holding an arm (m g d cos θ), a four-bar's
  power balance (motor torque × speed = dE/dt within 1 %).
- **Example «pendule double (dynamique)»**: two oak arms let go, nothing drives them;
  validation, playback and the split-screen video follow the simulation in real time,
  with the energy read out. A validation can take poses computed elsewhere
  (`collide.validate(poses=...)`).
- **The other standard joints**, in the panel's tool grid with the others (twelve
  tools, four per row, each with its icon drawn in code): *pivot glissant*
  (cylindrical: turns and slides, its angle motorised, its slide with its own stops and
  spring), *rotule* (ball), *encastrement* (lock: two parts as one), *liaison élastique*
  (a silent-block: rigid for the kinematics, springs and dampers in every direction in
  the dynamics).
- **Joint modifiers** in «Liaison choisie»: *ressort* (stiffness, at rest in the drawn
  pose), *amortisseur*, *moteur en couple* (a constant torque or force); stops now act in
  the dynamics (a stiff, damped stop at about 40 Hz), reported as «en butée» there.
- **«Dynamique» in the panel** for your own models (with a duration): validation,
  playback and video follow the simulation; speed motors run their travel in that time
  and the report gives each one's maximum and mean torque; parts without a material get
  a mass from their boxes and are listed.
- Checked: degrees of freedom of each joint, a body falling along a cylindrical joint
  while spinning, the conical pendulum on a ball joint (w² = 3g / 2L cos θ, height kept),
  a lid locked on a falling box, a torsion spring's period, a mass on a vertical spring,
  a mass on an elastic mount (sag m g / k, frequency sqrt(k/m)), a block caught by a stop
  on a slope, a flap caught by its stay; a drawn drop leaf in oak falling onto its 60°
  stay in a real IngeTrazo window.

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
