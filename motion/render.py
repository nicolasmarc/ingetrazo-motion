"""Shaded drawing of a mechanism with QPainter, and the split-screen frame.

No OpenGL: faces are projected (parallel projection), shaded by a fixed
light and painted back to front. Good for videos and previews of furniture
mechanisms; it is not the IngeTrazo renderer.

Camera convention = IngeTrazo's: the eye sits at
target + d (cos p cos y, cos p sin y, sin p) for yaw y and pitch p.
"""
import math
import re
import numpy as np
from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (QBrush, QColor, QFont, QImage, QLinearGradient,
                           QPainter, QPen, QPolygonF)

from . import collide

ACCENT = QColor(232, 115, 12)
# the frame in a light neutral grey, the moving parts in clear, distinct
# colours: one colour per family of parts (« Patte 1 » … « Patte 8 » alike)
FIXED = QColor(196, 198, 202)
MOVING = [QColor(38, 110, 200), QColor(232, 120, 20), QColor(40, 160, 90),
          QColor(205, 50, 60), QColor(140, 80, 190), QColor(20, 160, 170),
          QColor(220, 180, 30), QColor(150, 100, 60)]


def family(name):
    """« Patte 3 », « Patte4 », « Roue 12 » -> « patte », « roue »."""
    return re.sub(r"[\s_\-]*\d+$", "", (name or "").strip()).lower() or name


def colours(mech):
    """{body name: (r, g, b)}: the body's own colour if it has one, else
    grey for the frame and one palette colour per family."""
    out, fam = {}, {}
    for body in [mech.ground] + mech.bodies:
        if getattr(body, "color", None):
            out[body.name] = tuple(body.color)
        elif body.fixed:
            out[body.name] = FIXED.getRgb()[:3]
        else:
            k = fam.setdefault(family(body.name), len(fam))
            out[body.name] = MOVING[k % len(MOVING)].getRgb()[:3]
    return out


HIT = QColor(220, 50, 50)
_BOX_FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]


def view_axes(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    eye = np.array([cp * cy, cp * sy, sp])
    fwd = -eye                                    # looking direction
    right = np.cross(fwd, (0, 0, 1.0))
    n = np.linalg.norm(right)
    right = np.array([1.0, 0, 0]) if n < 1e-9 else right / n
    up = np.cross(right, fwd)
    return right, up, fwd


class Shaded:
    """``polys``: {body name: [ (k, 3) arrays of face corners, reference
    pose ]}; bodies without polygons are drawn from their boxes."""

    def __init__(self, mech, polys=None, max_faces=40000):
        self.m = mech
        self.items = []                           # (body, V, faces, colour)
        polys = polys or {}
        k = 0
        cols = colours(mech)
        for body in [mech.ground] + mech.bodies:
            faces = polys.get(body.name)
            fcols = None
            if faces:
                V = np.vstack(faces)
                idx, o = [], 0
                for f in faces:
                    idx.append(np.arange(o, o + len(f)))
                    o += len(f)
            else:
                V, idx, fcols = self._boxes(body)
            if not len(V):
                continue
            col = QColor(*cols[body.name])
            if not body.fixed:
                k += 1
            self.items.append([body, V, idx, col, fcols])
        total = sum(len(i[2]) for i in self.items)
        if total > max_faces:                     # keep it drawable: the
            # largest faces (the small ones are the fine facets of curved
            # surfaces, hardly seen); every n-th face would leave holes
            areas = []
            for it in self.items:
                V = it[1]
                for f in it[2]:
                    P = V[f]
                    areas.append(0.5 * float(np.linalg.norm(
                        np.cross(P[1:-1] - P[0], P[2:] - P[0]).sum(axis=0))) if len(P) >= 3 else 0.0)
            areas = np.array(areas)
            cut = np.sort(areas)[::-1][max_faces - 1]
            k = 0
            for it in self.items:
                n = len(it[2])
                keep = np.nonzero(areas[k:k + n] >= cut)[0]
                k += n
                it[2] = [it[2][i] for i in keep]
                if it[4] is not None:
                    it[4] = [it[4][i] for i in keep]

    @staticmethod
    def _boxes(body):
        V, idx, cols = [], [], []
        for bx in body.boxes:
            c, R, h = collide.world_obb(body, np.zeros(6), bx)
            k = collide._corners((c, R, h))
            base = len(V) * 8
            V.append(k)
            idx += [np.array(f) + base for f in _BOX_FACES]
            col = QColor(*bx.color) if getattr(bx, "color", None) else None
            cols += [col] * len(_BOX_FACES)
        if not any(cols):
            cols = None
        return (np.vstack(V) if V else np.zeros((0, 3))), idx, cols

    def world(self, body, x):
        q = self.m.pose(x, body)
        V = [it for it in self.items if it[0] is body][0][1]
        if body.fixed:
            return V
        R = body.rot(q)
        return body.center + q[:3] + (V - body.center) @ R.T

    def fit(self, xs, yaw, pitch, w, h, margin=0.08, follow=None):
        """(scale, centre) so every pose in ``xs`` fits a w x h area.
        ``follow``: a body the camera follows — the frame is fitted to the
        moving parts around it (the fixed ones, a long track, are left out)."""
        right, up, _ = view_axes(yaw, pitch)
        pts = []
        for x in xs:
            shift = 0.0
            if follow is not None:
                shift = follow.world_point(self.m.pose(x, follow), follow.center) - follow.center
            for it in self.items:
                if follow is not None and it[0].fixed:
                    continue
                W = self.world(it[0], x) - shift
                pts.append(W[:: max(1, len(W) // 400)])
        P = np.vstack(pts)
        u, v = P @ right, P @ up
        span_u, span_v = np.ptp(u) or 1.0, np.ptp(v) or 1.0
        self.floor_z = float(P[:, 2].min())
        scale = (1 - 2 * margin) * min(w / span_u, h / span_v)
        cu, cv = (u.min() + u.max()) / 2, (v.min() + v.max()) / 2
        centre = cu * right + cv * up
        return scale, centre

    def draw(self, p, rect, x, yaw, pitch, scale, centre, hits=(), traces=(),
             floor=True):
        right, up, fwd = view_axes(yaw, pitch)
        cx, cy = rect.center().x(), rect.center().y()
        light = -fwd + np.array([-0.3, -0.4, 0.8])
        light /= np.linalg.norm(light)

        def to_px(P):
            P = np.asarray(P) - centre
            return cx + scale * (P @ right), cy - scale * (P @ up)

        p.save()
        p.setClipRect(rect)
        p.setRenderHint(QPainter.Antialiasing, True)
        if floor:
            zmin = getattr(self, "floor_z", None)
            if zmin is None:
                zmin = min(self.world(it[0], x)[:, 2].min() for it in self.items)
            ext = max(rect.width(), rect.height()) / scale
            g = np.arange(-ext, ext + 1e-9, max(0.1, round(ext / 12, 1)))
            p.setPen(QPen(QColor(205, 205, 200), 1))
            c0 = centre
            for a in g:
                for P0, P1 in (((c0[0] + a, c0[1] - ext, zmin), (c0[0] + a, c0[1] + ext, zmin)),
                               ((c0[0] - ext, c0[1] + a, zmin), (c0[0] + ext, c0[1] + a, zmin))):
                    (x0, y0), (x1, y1) = to_px(P0), to_px(P1)
                    p.drawLine(QPointF(x0, y0), QPointF(x1, y1))
        polys = []
        for body, V, idx, col, fcols in self.items:
            W = self.world(body, x)
            px, py = to_px(W)
            depth = W @ fwd
            hit = body.name in hits
            for fi, f in enumerate(idx):
                base = HIT if hit else (fcols[fi] if fcols and fcols[fi] else col)
                Pf = W[f]
                if len(f) < 3:
                    continue
                n = np.cross(Pf[1] - Pf[0], Pf[2] - Pf[0])
                nn = np.linalg.norm(n)
                if nn < 1e-15:
                    continue
                shade = 0.55 + 0.45 * abs((n / nn) @ light)
                polys.append((float(depth[f].mean()), px[f], py[f], base, shade))
        polys.sort(key=lambda t: -t[0])
        for _, X, Yp, base, shade in polys:
            c = QColor(int(base.red() * shade), int(base.green() * shade),
                       int(base.blue() * shade))
            p.setPen(QPen(c.darker(140), 0.7))
            p.setBrush(QBrush(c))
            p.drawPolygon(QPolygonF([QPointF(a, b) for a, b in zip(X, Yp)]))
        for tr in traces:
            if len(tr) < 2:
                continue
            X, Yp = to_px(tr)
            p.setPen(QPen(ACCENT, 2, Qt.DashLine))
            for i in range(len(tr) - 1):
                p.drawLine(QPointF(X[i], Yp[i]), QPointF(X[i + 1], Yp[i + 1]))
        p.setPen(QPen(ACCENT, 2))
        p.setBrush(QColor(255, 255, 255))
        for j in self.m.joints:
            if j.kind == "coupling":
                continue
            body = j.b if not j.b.fixed else j.a
            X, Yp = to_px(body.world_point(self.m.pose(x, body), j.point))
            p.drawEllipse(QPointF(X, Yp), 4, 4)
        p.restore()


def draw_projected(shaded, p, x, project, fwd, bodies=None, hits=(), alpha=255):
    """Draw ``shaded``'s faces through an outside projection (IngeTrazo's
    ``world_to_pixels``): ``project(P (N,3)) -> (px, py, in_front)``;
    ``fwd`` = viewing direction (for back-to-front order and shading);
    ``bodies`` = names to draw (None = all)."""
    fwd = np.asarray(fwd, float)
    fwd = fwd / np.linalg.norm(fwd)
    light = -fwd + np.array([-0.3, -0.4, 0.8])
    light /= np.linalg.norm(light)
    polys = []
    for body, V, idx, col, fcols in shaded.items:
        if bodies is not None and body.name not in bodies:
            continue
        W = shaded.world(body, x)
        px, py, ok = project(W)
        depth = W @ fwd
        hit = body.name in hits
        for fi, f in enumerate(idx):
            base = HIT if hit else (fcols[fi] if fcols and fcols[fi] else col)
            if len(f) < 3 or not np.all(ok[f]):
                continue
            Pf = W[f]
            n = np.cross(Pf[1] - Pf[0], Pf[2] - Pf[0])
            nn = np.linalg.norm(n)
            if nn < 1e-15:
                continue
            shade = 0.55 + 0.45 * abs((n / nn) @ light)
            polys.append((float(depth[f].mean()), px[f], py[f], base, shade))
    polys.sort(key=lambda t: -t[0])
    p.save()
    p.setRenderHint(QPainter.Antialiasing, True)
    for _, X, Yp, base, shade in polys:
        c = QColor(int(base.red() * shade), int(base.green() * shade),
                   int(base.blue() * shade), alpha)
        p.setPen(QPen(c.darker(140), 0.7))
        p.setBrush(QBrush(c))
        p.drawPolygon(QPolygonF([QPointF(a, b) for a, b in zip(X, Yp)]))
    p.restore()


def compose(width, height, left, draw_right, title, info_lines, t,
            left_label="IngeTrazo", right_label=None):
    """Split-screen frame: ``left`` (a QImage, the IngeTrazo view) and
    ``draw_right(painter, rect)`` side by side, a title band and a
    progress band."""
    img = QImage(width, height, QImage.Format_RGB32)
    p = QPainter(img)
    p.setRenderHint(QPainter.Antialiasing, True)
    p.setRenderHint(QPainter.TextAntialiasing, True)
    p.setRenderHint(QPainter.SmoothPixmapTransform, True)
    g = QLinearGradient(0, 0, 0, height)
    g.setColorAt(0, QColor(250, 250, 248))
    g.setColorAt(1, QColor(232, 232, 228))
    p.fillRect(0, 0, width, height, g)
    u = height / 1080.0                           # layout unit
    top, bottom, gap = 92 * u, 88 * u, 16 * u
    pane_w = (width - 3 * gap) / 2
    pane_h = height - top - bottom
    lrect = QRectF(gap, top, pane_w, pane_h)
    rrect = QRectF(2 * gap + pane_w, top, pane_w, pane_h)

    f_title = QFont("DejaVu Sans")
    f_title.setPixelSize(max(12, int(34 * u)))
    f_title.setBold(True)
    f_lab = QFont("DejaVu Sans")
    f_lab.setPixelSize(max(10, int(20 * u)))
    f_lab.setBold(True)
    f_info = QFont("DejaVu Sans")
    f_info.setPixelSize(max(10, int(22 * u)))

    p.setPen(QColor(35, 35, 35))
    p.setFont(f_title)
    p.drawText(QRectF(gap, 0, width - 2 * gap, top - 30 * u), Qt.AlignLeft | Qt.AlignVCenter, title)

    if right_label is None:
        from .lang import T
        right_label = T("Animation Motion")
    for rect, label in ((lrect, left_label), (rrect, right_label)):
        p.setPen(QPen(QColor(200, 200, 196), 1))
        p.setBrush(QColor(255, 255, 255))
        p.drawRoundedRect(rect, 10 * u, 10 * u)
        p.setFont(f_lab)
        p.setPen(ACCENT)
        p.drawText(QRectF(rect.left(), rect.top() - 30 * u, rect.width(), 28 * u),
                   Qt.AlignLeft | Qt.AlignBottom, label)

    if left is not None and not left.isNull():
        inner = lrect.adjusted(4, 4, -4, -4)
        s = min(inner.width() / left.width(), inner.height() / left.height())
        w, h = left.width() * s, left.height() * s
        target = QRectF(inner.center().x() - w / 2, inner.center().y() - h / 2, w, h)
        p.save()
        p.setClipRect(inner)
        p.drawImage(target, left)
        p.restore()
    p.save()
    draw_right(p, rrect.adjusted(4, 4, -4, -4))
    p.restore()

    # bottom band: percentage, progress, readings
    yb = height - bottom
    p.setFont(f_info)
    p.setPen(QColor(35, 35, 35))
    p.drawText(QRectF(gap, yb + 8 * u, 160 * u, 34 * u), Qt.AlignLeft | Qt.AlignVCenter,
               f"{round(100 * t):d} %")
    bar = QRectF(gap + 170 * u, yb + 20 * u, pane_w - 170 * u, 10 * u)
    p.setPen(Qt.NoPen)
    p.setBrush(QColor(210, 210, 206))
    p.drawRoundedRect(bar, 5 * u, 5 * u)
    p.setBrush(ACCENT)
    p.drawRoundedRect(QRectF(bar.left(), bar.top(), bar.width() * t, bar.height()), 5 * u, 5 * u)
    p.setPen(QColor(60, 60, 60))
    p.drawText(QRectF(rrect.left(), yb + 4 * u, rrect.width(), bottom - 8 * u),
               Qt.AlignLeft | Qt.AlignVCenter | Qt.TextWordWrap, "   ·   ".join(info_lines))
    p.end()
    return img
