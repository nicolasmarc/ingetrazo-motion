"""The 8 Motion icons, drawn with QPainter (no image files).

Every icon is drawn on a 24 x 24 grid, in the text colour of the palette
(``ink``) with one accent colour, so it follows light and dark themes.
"""
import math
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QIcon, QPainter, QPainterPath,
                           QPen, QPixmap, QPolygonF)

# the panel's tools, four per row: parts and the simple joints, the other
# standard joints, then rail, motor, validation, trajectory
NAMES = ("fixed", "moving", "pivot", "slider",
         "cylindrical", "ball", "lock", "elastic",
         "annular", "pointplane", "planar", "screw",
         "guide", "driver", "validate", "trajectory")

LABELS = {
    "fixed": ("Pièce fixe", "Fixed part"),
    "moving": ("Pièce mobile", "Moving part"),
    "pivot": ("Pivot", "Pivot"),
    "slider": ("Glissière", "Slider"),
    "guide": ("Guide / rail", "Guide/rail"),
    "driver": ("Moteur 0–100 %", "Driver 0–100 %"),
    "validate": ("Valider", "Validate"),
    "trajectory": ("Trajectoire", "Trajectory"),
    "coupling": ("Couplage", "Coupling"),
    "scan": ("Détecter", "Detect"),
    "cylindrical": ("Pivot glissant", "Cylindrical"),
    "ball": ("Rotule", "Ball"),
    "lock": ("Encastrement", "Lock"),
    "elastic": ("Élastique", "Elastic"),
    "spring": ("Ressort", "Spring"),
    "annular": ("Lin. annulaire", "Annular"),
    "pointplane": ("Ponctuelle", "Point–plane"),
    "planar": ("Appui plan", "Planar"),
    "screw": ("Hélicoïdale", "Screw"),
}

def label(name):
    """The tool's name in the panel's language."""
    from .lang import english
    fr, en = LABELS[name]
    return en if english() else fr


FALLBACK_ACCENT = QColor(232, 115, 12)       # orange


def accent():
    try:
        from views.theme import accent_color      # IngeTrazo's accent
        c = accent_color()
        if isinstance(c, QColor) and c.isValid():
            return c
    except Exception:
        pass
    return QColor(FALLBACK_ACCENT)


def _pen(color, w=1.7, style=Qt.SolidLine):
    p = QPen(color, w, style)
    p.setCapStyle(Qt.RoundCap)
    p.setJoinStyle(Qt.RoundJoin)
    return p


def _arrow_head(p, tip, ang, color, size=3.6):
    """Filled arrow head at ``tip`` pointing along ``ang`` (radians)."""
    a1, a2 = ang + math.radians(150), ang - math.radians(150)
    poly = QPolygonF([tip,
                      QPointF(tip.x() + size * math.cos(a1), tip.y() + size * math.sin(a1)),
                      QPointF(tip.x() + size * math.cos(a2), tip.y() + size * math.sin(a2))])
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(color))
    p.drawPolygon(poly)
    p.setBrush(Qt.NoBrush)


def _soft(c, alpha=70):
    c = QColor(c)
    c.setAlpha(alpha)
    return c


def _fixed(p, ink, acc):
    p.setPen(_pen(ink))
    p.setBrush(_soft(acc))
    p.drawRect(QRectF(6.5, 6, 11, 9.5))
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(ink, 1.9))
    p.drawLine(QPointF(3, 15.5), QPointF(21, 15.5))
    p.setPen(_pen(ink, 1.2))
    for x in range(6, 23, 4):                     # ground hatching
        p.drawLine(QPointF(x, 15.5), QPointF(x - 3.5, 20.5))


def _moving(p, ink, acc):
    p.save()                                      # ghost of the drawn pose
    p.translate(8, 15)
    p.rotate(-28)
    p.setPen(_pen(ink, 1.1, Qt.DashLine))
    p.drawRect(QRectF(-4.5, -4, 9, 8))
    p.restore()
    p.setPen(_pen(ink))
    p.setBrush(_soft(acc))
    p.drawRect(QRectF(11.5, 11, 9, 8))
    p.setBrush(Qt.NoBrush)
    path = QPainterPath(QPointF(5, 7))
    path.quadTo(QPointF(11, 2.5), QPointF(17.5, 6.5))
    p.setPen(_pen(acc, 1.8))
    p.drawPath(path)
    _arrow_head(p, QPointF(18.6, 7.4), math.radians(40), acc)


def _pivot(p, ink, acc):
    c = QPointF(12, 13)
    p.setPen(_pen(ink, 3.0))
    p.drawLine(QPointF(3.5, 20), c)               # link a
    p.drawLine(c, QPointF(20.5, 9))               # link b
    p.setPen(_pen(acc, 1.6))
    p.drawArc(QRectF(c.x() - 8, c.y() - 8, 16, 16), 100 * 16, 75 * 16)
    a = math.radians(-175)
    _arrow_head(p, QPointF(c.x() + 8 * math.cos(a), c.y() + 8 * math.sin(a)),
                math.radians(90), acc, 3.2)
    p.setPen(_pen(ink, 1.6))
    p.setBrush(_soft(acc, 255))
    p.drawEllipse(c, 3.3, 3.3)
    p.setBrush(QBrush(ink))
    p.setPen(Qt.NoPen)
    p.drawEllipse(c, 1.1, 1.1)
    p.setBrush(Qt.NoBrush)


def _slider(p, ink, acc):
    p.setPen(_pen(ink, 1.9))
    p.drawLine(QPointF(2.5, 17), QPointF(21.5, 17))
    p.drawLine(QPointF(2.5, 14.5), QPointF(2.5, 19.5))
    p.drawLine(QPointF(21.5, 14.5), QPointF(21.5, 19.5))
    p.setPen(_pen(ink))
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(8, 11, 8, 6))
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.6))
    p.drawLine(QPointF(6, 6), QPointF(18, 6))
    _arrow_head(p, QPointF(20, 6), 0.0, acc)
    _arrow_head(p, QPointF(4, 6), math.pi, acc)


def _guide(p, ink, acc):
    path = QPainterPath(QPointF(4, 21.5))
    path.lineTo(QPointF(4, 13))
    path.arcTo(QRectF(4, 4, 18, 18), 180, -90)    # quarter turn, radius 9
    path.lineTo(QPointF(21.5, 4))
    p.setPen(_pen(ink, 2.0))
    p.drawPath(path)
    a = math.radians(225)                         # second roller on the arc
    r2 = QPointF(13 + 9 * math.cos(a), 13 + 9 * math.sin(a))
    r1 = QPointF(4, 18.5)
    p.setPen(_pen(acc, 3.2))                      # the panel riding on them
    p.drawLine(r1, r2)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(acc))
    for r in (r1, r2):
        p.drawEllipse(r, 2.7, 2.7)
    p.setBrush(QBrush(ink))
    for r in (r1, r2):
        p.drawEllipse(r, 1.0, 1.0)
    p.setBrush(Qt.NoBrush)


def _driver(p, ink, acc):
    c, r = QPointF(12, 14), 8.5
    p.setPen(_pen(ink, 1.7))
    p.drawArc(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r), -20 * 16, 220 * 16)
    p.setPen(_pen(ink, 1.1))
    for k in range(6):                            # ticks 0..100 %
        a = math.radians(200 - k * 44)
        p.drawLine(QPointF(c.x() + (r - 2.6) * math.cos(a), c.y() - (r - 2.6) * math.sin(a)),
                   QPointF(c.x() + r * math.cos(a), c.y() - r * math.sin(a)))
    a = math.radians(55)
    tip = QPointF(c.x() + (r - 1.5) * math.cos(a), c.y() - (r - 1.5) * math.sin(a))
    p.setPen(_pen(acc, 2.0))
    p.drawLine(c, tip)
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(acc))
    p.drawEllipse(c, 1.9, 1.9)
    p.setBrush(Qt.NoBrush)
    f = QFont()
    f.setPixelSize(7)
    f.setBold(True)
    p.setFont(f)
    p.setPen(ink)
    p.drawText(QRectF(6, 15.5, 12, 8), Qt.AlignCenter, "%")


def _validate(p, ink, acc):
    p.setPen(_pen(acc, 1.8))
    p.setBrush(_soft(acc, 45))
    p.drawEllipse(QPointF(12, 12), 9, 9)
    p.setBrush(Qt.NoBrush)
    path = QPainterPath(QPointF(7.5, 12.5))
    path.lineTo(QPointF(10.8, 15.8))
    path.lineTo(QPointF(16.8, 8.6))
    p.setPen(_pen(ink, 2.4))
    p.drawPath(path)


def _trajectory(p, ink, acc):
    path = QPainterPath(QPointF(4, 19.5))
    path.cubicTo(QPointF(5, 9), QPointF(13, 15), QPointF(18.5, 6))
    p.setPen(_pen(ink, 1.5, Qt.DotLine))
    p.drawPath(path)
    _arrow_head(p, QPointF(19.6, 4.3), math.radians(-60), acc, 4.0)
    p.setPen(_pen(ink, 1.3))
    p.setBrush(_soft(acc, 255))
    for t in (0.0, 0.5):
        pt = path.pointAtPercent(t)
        p.drawEllipse(pt, 2.3, 2.3)
    p.setBrush(Qt.NoBrush)


def _coupling(p, ink, acc):
    """Two meshing gears: a ratio between two joints."""
    def gear(cx, cy, r, z, col, phase):
        path = QPainterPath()
        for k in range(z * 2 + 1):
            a = phase + k * math.pi / z
            rr = r + (1.4 if k % 2 == 0 else -0.2)
            pt = QPointF(cx + rr * math.cos(a), cy + rr * math.sin(a))
            path.moveTo(pt) if k == 0 else path.lineTo(pt)
        p.setPen(_pen(ink, 1.3))
        p.setBrush(_soft(col, 255) if col is acc else Qt.NoBrush)
        p.drawPath(path)
        p.setBrush(Qt.NoBrush)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(ink))
        p.drawEllipse(QPointF(cx, cy), 1.2, 1.2)
        p.setBrush(Qt.NoBrush)
    gear(8.5, 13.5, 5.6, 9, acc, 0.0)
    gear(17.0, 8.0, 3.6, 6, ink, math.pi / 6)


def _scan(p, ink, acc):
    """A magnifier over a pivot (an axis found by scanning the model)."""
    p.setPen(_pen(ink, 1.3))
    p.drawEllipse(QPointF(7.5, 15.5), 4.2, 4.2)          # a part with a bore
    p.drawLine(QPointF(3.3, 15.5), QPointF(11.7, 15.5))
    p.setPen(_pen(acc, 2.0))
    p.drawEllipse(QPointF(13.5, 9.5), 5.6, 5.6)          # the lens
    p.drawLine(QPointF(17.6, 13.6), QPointF(21.5, 17.5))
    p.setPen(Qt.NoPen)
    p.setBrush(QBrush(acc))
    p.drawEllipse(QPointF(13.5, 9.5), 1.6, 1.6)          # the axis found
    p.setBrush(Qt.NoBrush)


def _cylindrical(p, ink, acc):
    """A shaft through a sleeve, turning and sliding (arrows both ways)."""
    p.setPen(_pen(ink, 1.4))
    p.drawLine(QPointF(3, 12), QPointF(21, 12))                 # the shaft
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(8.5, 8, 7, 8))                           # the sleeve
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.4))
    p.drawArc(QRectF(7, 4, 10, 6), 20 * 16, 140 * 16)          # turns
    _arrow_head(p, QPointF(8.2, 6.0), math.radians(200), acc, 2.6)
    p.drawLine(QPointF(4, 19.5), QPointF(20, 19.5))            # slides
    _arrow_head(p, QPointF(20, 19.5), 0.0, acc, 2.6)
    _arrow_head(p, QPointF(4, 19.5), math.pi, acc, 2.6)


def _ball(p, ink, acc):
    """A ball in its socket, with a stem."""
    p.setPen(_pen(ink, 1.4))
    p.drawArc(QRectF(4.5, 7.5, 15, 15), 200 * 16, 140 * 16)   # the socket
    p.setBrush(_soft(acc, 255))
    p.drawEllipse(QPointF(12, 15), 4.6, 4.6)
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(12, 10.4), QPointF(16.5, 3))            # the stem


def _lock(p, ink, acc):
    """Two parts bolted together."""
    p.setPen(_pen(ink, 1.3))
    p.drawRect(QRectF(3, 8, 9, 10))
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(12, 8, 9, 10))
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(ink, 1.8))
    for y in (10.5, 15.5):
        p.drawLine(QPointF(8, y), QPointF(16, y))              # the bolts


def _zigzag(p, x0, x1, y, n, h):
    pts = [QPointF(x0, y)]
    for i in range(1, 2 * n):
        pts.append(QPointF(x0 + (x1 - x0) * i / (2 * n), y + (h if i % 2 else -h)))
    pts.append(QPointF(x1, y))
    for a, b in zip(pts, pts[1:]):
        p.drawLine(a, b)


def _elastic(p, ink, acc):
    """Two plates held by springs (a flexible mount)."""
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(4, 4), QPointF(4, 20))
    p.setPen(_pen(ink, 1.3))
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(16, 6, 5, 12))
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.3))
    _zigzag(p, 4, 16, 9, 3, 2.0)
    _zigzag(p, 4, 16, 15, 3, 2.0)


def _spring(p, ink, acc):
    """A coil spring between two plates."""
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(5, 3), QPointF(19, 3))
    p.drawLine(QPointF(5, 21), QPointF(19, 21))
    p.setPen(_pen(acc, 1.4))
    pts = [QPointF(12, 3)]
    for i in range(1, 8):
        pts.append(QPointF(12 + (5 if i % 2 else -5), 3 + 18 * i / 8))
    pts.append(QPointF(12, 21))
    for a, b in zip(pts, pts[1:]):
        p.drawLine(a, b)


def _annular(p, ink, acc):
    """A ball in a bore (cut open), free to slide along it."""
    p.setPen(_pen(ink, 1.4))
    p.drawLine(QPointF(3, 8), QPointF(21, 8))                   # the bore's walls
    p.drawLine(QPointF(3, 16), QPointF(21, 16))
    p.setBrush(_soft(acc, 255))
    p.drawEllipse(QPointF(12, 12), 3.8, 3.8)                   # the ball
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.4))
    p.drawLine(QPointF(5, 20.5), QPointF(19, 20.5))
    _arrow_head(p, QPointF(19, 20.5), 0.0, acc, 2.6)
    _arrow_head(p, QPointF(5, 20.5), math.pi, acc, 2.6)


def _pointplane(p, ink, acc):
    """A ball touching a plane at one point."""
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(3, 18), QPointF(21, 18))                 # the plane
    p.setPen(_pen(ink, 1.0))
    for x in (5, 9, 13, 17, 21):
        p.drawLine(QPointF(x, 18), QPointF(x - 3, 21.5))
    p.setPen(_pen(ink, 1.4))
    p.setBrush(_soft(acc, 255))
    p.drawEllipse(QPointF(12, 12.5), 5.2, 5.2)
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(12, 7.3), QPointF(12, 3))                # the stem


def _planar(p, ink, acc):
    """A block lying on a plane, sliding and turning on it."""
    p.setPen(_pen(ink, 1.6))
    p.drawLine(QPointF(3, 17), QPointF(21, 17))
    p.setPen(_pen(ink, 1.0))
    for x in (5, 9, 13, 17, 21):
        p.drawLine(QPointF(x, 17), QPointF(x - 3, 20.5))
    p.setPen(_pen(ink, 1.4))
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(6.5, 10.5, 11, 6.5))
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.4))
    p.drawArc(QRectF(7, 3.5, 10, 6), 20 * 16, 140 * 16)        # turns about the normal
    _arrow_head(p, QPointF(8.2, 5.5), math.radians(200), acc, 2.6)


def _screw(p, ink, acc):
    """A threaded rod through a nut."""
    p.setPen(_pen(ink, 1.3))
    p.drawLine(QPointF(3, 9.5), QPointF(21, 9.5))
    p.drawLine(QPointF(3, 14.5), QPointF(21, 14.5))
    for x in (4, 7, 16, 19):                                    # the thread
        p.drawLine(QPointF(x, 14.5), QPointF(x + 2, 9.5))
    p.setBrush(_soft(acc, 255))
    p.drawRect(QRectF(9, 6.5, 6, 11))                          # the nut
    p.setBrush(Qt.NoBrush)
    p.setPen(_pen(acc, 1.4))
    p.drawArc(QRectF(7, 17, 10, 5), 200 * 16, 140 * 16)
    _arrow_head(p, QPointF(15.8, 20.6), math.radians(-20), acc, 2.6)


_DRAW = dict(annular=_annular, pointplane=_pointplane, planar=_planar, screw=_screw,
             cylindrical=_cylindrical, ball=_ball, lock=_lock, elastic=_elastic,
             spring=_spring, scan=_scan, coupling=_coupling, fixed=_fixed, moving=_moving, pivot=_pivot, slider=_slider,
             guide=_guide, driver=_driver, validate=_validate,
             trajectory=_trajectory)


_FRAME = {}


def _frame(name):
    """Centre and scale that put the drawing of ``name`` in the middle of
    its square, every icon filling the same 20 of its 24 units: drawn once
    large, its inked pixels measured."""
    if name not in _FRAME:
        from PySide6.QtGui import QImage
        n = 192
        img = QImage(n, n, QImage.Format_ARGB32)
        img.fill(Qt.transparent)
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.scale(n / 24.0, n / 24.0)
        _DRAW[name](p, QColor(0, 0, 0), QColor(0, 0, 0))
        p.end()
        import numpy as _np
        ptr = img.constBits()
        a = _np.frombuffer(ptr, _np.uint8, count=n * n * 4).reshape(n, n, 4)[:, :, 3]
        ys, xs = _np.nonzero(a > 40)
        if not len(xs):
            _FRAME[name] = (12.0, 12.0, 1.0)
        else:
            x0, x1 = xs.min() * 24.0 / n, (xs.max() + 1) * 24.0 / n
            y0, y1 = ys.min() * 24.0 / n, (ys.max() + 1) * 24.0 / n
            s = min(20.0 / max(x1 - x0, y1 - y0, 1e-6), 1.6)
            _FRAME[name] = ((x0 + x1) / 2, (y0 + y1) / 2, s)
    return _FRAME[name]


def paint(painter, name, size, ink, acc):
    """Draw icon ``name`` into a ``size`` x ``size`` square at (0, 0),
    centred and at the common size."""
    cx, cy, s = _frame(name)
    painter.save()
    painter.setRenderHint(QPainter.Antialiasing, True)
    painter.setRenderHint(QPainter.TextAntialiasing, True)
    painter.scale(size / 24.0, size / 24.0)
    painter.translate(12.0, 12.0)
    painter.scale(s, s)
    painter.translate(-cx, -cy)
    _DRAW[name](painter, QColor(ink), QColor(acc))
    painter.restore()


def pixmap(name, size=24, ink=None, acc=None, dpr=2.0):
    ink = QColor(30, 30, 30) if ink is None else QColor(ink)
    acc = accent() if acc is None else QColor(acc)
    pm = QPixmap(int(size * dpr), int(size * dpr))
    pm.setDevicePixelRatio(dpr)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    paint(p, name, size, ink, acc)
    p.end()
    return pm


def icon(name, widget=None, size=24):
    """QIcon of ``name``, inked with ``widget``'s text colour."""
    ink = None
    if widget is not None:
        ink = widget.palette().color(widget.foregroundRole())
    ic = QIcon()
    for s in (size, 2 * size):
        ic.addPixmap(pixmap(name, s, ink, dpr=1.0))
    dis = QColor(ink or QColor(30, 30, 30))
    dis.setAlpha(90)
    ic.addPixmap(pixmap(name, size, dis, dis, dpr=1.0), QIcon.Disabled)
    return ic
