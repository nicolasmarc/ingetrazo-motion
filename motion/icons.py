"""The 8 Motion icons, drawn with QPainter (no image files).

Every icon is drawn on a 24 x 24 grid, in the text colour of the palette
(``ink``) with one accent colour, so it follows light and dark themes.
"""
import math
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QIcon, QPainter, QPainterPath,
                           QPen, QPixmap, QPolygonF)

NAMES = ("fixed", "moving", "pivot", "slider", "guide", "driver",
         "validate", "trajectory")

LABELS = {
    "fixed": ("Pièce fixe", "Fixed part"),
    "moving": ("Pièce mobile", "Moving part"),
    "pivot": ("Pivot", "Pivot"),
    "slider": ("Glissière", "Slider"),
    "guide": ("Guide / rail", "Guide/rail"),
    "driver": ("Moteur 0–100 %", "Driver 0–100 %"),
    "validate": ("Valider", "Validate"),
    "trajectory": ("Trajectoire", "Trajectory"),
}

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


_DRAW = dict(fixed=_fixed, moving=_moving, pivot=_pivot, slider=_slider,
             guide=_guide, driver=_driver, validate=_validate,
             trajectory=_trajectory)


def paint(painter, name, size, ink, acc):
    """Draw icon ``name`` into a ``size`` x ``size`` square at (0, 0)."""
    painter.save()
    painter.setRenderHint(QPainter.Antialiasing, True)
    painter.setRenderHint(QPainter.TextAntialiasing, True)
    painter.scale(size / 24.0, size / 24.0)
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
