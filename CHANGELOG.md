# Changelog

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
