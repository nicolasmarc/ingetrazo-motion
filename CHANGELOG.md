# Changelog

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
