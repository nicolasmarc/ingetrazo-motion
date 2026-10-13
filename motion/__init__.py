"""Motion — kinematic validation of simple mechanisms (clean-room, GPL-3.0).

Pure Python + numpy core (no Qt): ``motion.model`` (bodies, pivot, slider,
guide/rail), ``motion.solver`` (closed loops, degrees of freedom),
``motion.collide`` (interference, exact minimum clearance, validation),
``motion.samples`` (folding chair, sectional door) and ``motion.document``
(what the panel stores in the .igz). The IngeTrazo panel is loaded lazily
by ``setup``.
"""
__version__ = "0.2.0"


def setup(app):          # pragma: no cover - UI, needs IngeTrazo
    from .panel import setup as _setup
    _setup(app)
