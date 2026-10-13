"""Motion — kinematic validation of simple mechanisms (clean-room, GPL-3.0).

Pure Python + numpy core (no Qt): ``motion.model`` / ``motion.solver`` /
``motion.collide``.  The IngeTrazo UI is loaded lazily by ``setup``.
"""
__version__ = "0.1.0"


def setup(app):          # pragma: no cover - UI, needs IngeTrazo
    from .panel import setup as _setup
    _setup(app)
