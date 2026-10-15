"""Video writers: the AVI is checked byte by byte, MP4 writers when present."""
import os, struct, sys, tempfile
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
try:
    from PySide6.QtGui import QColor, QImage, QPainter
    from PySide6.QtWidgets import QApplication
    APP = QApplication.instance() or QApplication([])
    from motion import video
except ImportError:                          # no PySide6: nothing to test
    video = None
from motion_probe import probe


def _frames(n, w=320, h=180):
    for i in range(n):
        im = QImage(w, h, QImage.Format_RGB32)
        im.fill(QColor(255, 255, 255))
        p = QPainter(im)
        p.fillRect(10 + 5 * i, 60, 40, 40, QColor(232, 115, 12))
        p.end()
        yield im


def test_avi_structure():
    if video is None:
        return
    path = os.path.join(tempfile.mkdtemp(), "a.mp4")
    w = video.open_writer(path, 320, 180, 25, prefer="avi")
    for im in _frames(12):
        w.add(im)
    out = w.close()
    assert out.endswith(".avi")
    data = open(out, "rb").read()
    assert data[:4] == b"RIFF" and data[8:12] == b"AVI "
    assert struct.unpack("<I", data[4:8])[0] == len(data) - 8
    i = data.index(b"avih")
    usec, _, _, _, frames = struct.unpack("<IIIII", data[i + 8:i + 28])
    assert usec == 40000 and frames == 12
    k = data.index(b"idx1")
    n = struct.unpack("<I", data[k + 4:k + 8])[0] // 16
    assert n == 12
    movi = data.index(b"movi")
    for e in range(n):                       # every index entry -> a JPEG
        _, _, off, size = struct.unpack("<4sIII", data[k + 8 + 16 * e:k + 24 + 16 * e])
        assert data[movi + off:movi + off + 4] == b"00dc"
        jpg = data[movi + off + 8:movi + off + 8 + size]
        assert jpg[:2] == b"\xff\xd8"
    img = QImage.fromData(jpg, "JPG")
    assert (img.width(), img.height()) == (320, 180)
    c = img.pixelColor(10 + 5 * 11 + 20, 80)           # last frame's square
    assert c.red() > 200 and c.blue() < 80
    info = probe(out)
    if info:
        assert info["codec"] == "mjpeg" and info["frames"] == 12


def test_mp4_writers_when_available():
    if video is None:
        return
    for prefer, ok in (("ffmpeg", video.find_ffmpeg()), ("qt", video.qt_multimedia_available())):
        if not ok:
            continue
        path = os.path.join(tempfile.mkdtemp(), "m.mp4")
        w = video.open_writer(path, 320, 180, 30, prefer=prefer)
        for im in _frames(30):
            w.add(im)
        out = w.close()
        assert out == path and os.path.getsize(out) > 0
        info = probe(out)
        if info:
            assert info["frames"] == 30 and (info["w"], info["h"]) == (320, 180)


def test_odd_sizes_are_made_even():
    if video is None:
        return
    path = os.path.join(tempfile.mkdtemp(), "o.mp4")
    w = video.open_writer(path, 321, 181, 30, prefer="avi")
    assert (w.w, w.h) == (320, 180)
    w.close()


if __name__ == "__main__":
    for k, f in list(globals().items()):
        if k.startswith("test_"):
            f(); print("ok", k)
