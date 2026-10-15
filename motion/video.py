"""Write a sequence of QImages to a video file.

Three writers, tried in this order by :func:`open_writer`:

- ``QtWriter`` — Qt Multimedia (QVideoFrameInput + QMediaRecorder), MP4,
  when the PySide6 in use has it (from-source installs with
  PySide6-Addons; the frozen IngeTrazo does not ship it today);
- ``FfmpegWriter`` — an ``ffmpeg`` executable (on the PATH, or one the user
  pointed to once), MP4 / H.264, raw frames piped to its stdin;
- ``AviWriter`` — pure Python, always available: Motion-JPEG in an AVI
  file (plays in VLC and the usual Windows players; not an MP4).

Order: Qt with H.264, ffmpeg (H.264), Qt with MPEG-4, AVI.
"""
import os
import struct
import subprocess
import sys

from PySide6.QtCore import QBuffer, QByteArray, QIODevice
from PySide6.QtGui import QImage


class AviWriter:
    """Motion-JPEG AVI (RIFF AVI with an idx1 index), written in Python."""
    kind = "avi"
    label = "AVI (Motion-JPEG)"

    def __init__(self, path, width, height, fps, quality=92):
        self.path, self.w, self.h, self.fps, self.q = path, width, height, fps, quality
        self.f = open(path, "wb")
        self.index = []                         # (offset in movi, size)
        self._header_placeholder()

    def _header_placeholder(self):
        self.f.write(b"\0" * self._header_size())
        self.movi_start = self.f.tell()         # just after 'movi' fourcc
        self.f.write(b"")

    def _header_size(self):
        return len(self._headers(0, 0))

    def _headers(self, frames, movi_size, riff_size=0):
        def chunk(fourcc, data):
            return fourcc + struct.pack("<I", len(data)) + data + (b"\0" if len(data) % 2 else b"")

        def lst(kind, data):
            return b"LIST" + struct.pack("<I", len(data) + 4) + kind + data

        usec = int(round(1e6 / self.fps))
        maxbytes = max((s for _, s in self.index), default=0) * self.fps
        avih = struct.pack("<IIIIIIIIII4I", usec, maxbytes, 0, 0x10, frames, 0, 1,
                           0, self.w, self.h, 0, 0, 0, 0)
        strh = struct.pack("<4s4sIHHIIIIIIIIhhhh", b"vids", b"MJPG", 0, 0, 0, 0,
                           1, self.fps, 0, frames, 0, 0xFFFFFFFF, 0,
                           0, 0, self.w, self.h)
        strf = struct.pack("<IiiHH4sIiiII", 40, self.w, self.h, 1, 24, b"MJPG",
                           self.w * self.h * 3, 0, 0, 0, 0)
        hdrl = lst(b"hdrl", chunk(b"avih", avih) +
                   lst(b"strl", chunk(b"strh", strh) + chunk(b"strf", strf)))
        return (b"RIFF" + struct.pack("<I", riff_size) + b"AVI " + hdrl +
                b"LIST" + struct.pack("<I", movi_size + 4) + b"movi")

    def add(self, image: QImage):
        if image.width() != self.w or image.height() != self.h:
            raise ValueError("frame size changed")
        ba = QByteArray()
        buf = QBuffer(ba)
        buf.open(QIODevice.WriteOnly)
        image.convertToFormat(QImage.Format_RGB888).save(buf, "JPG", self.q)
        data = bytes(ba)
        off = self.f.tell() - self.movi_start + 4      # from the 'movi' fourcc
        self.f.write(b"00dc" + struct.pack("<I", len(data)) + data)
        if len(data) % 2:
            self.f.write(b"\0")
        self.index.append((off, len(data)))

    def close(self):
        movi_size = self.f.tell() - self.movi_start
        idx = b"".join(b"00dc" + struct.pack("<III", 0x10, off, size)
                       for off, size in self.index)
        self.f.write(b"idx1" + struct.pack("<I", len(idx)) + idx)
        end = self.f.tell()
        self.f.seek(0)
        self.f.write(self._headers(len(self.index), movi_size, end - 8))
        self.f.close()
        return self.path


class FfmpegWriter:
    kind = "mp4"
    label = "MP4 (H.264, ffmpeg)"

    def __init__(self, path, width, height, fps, exe):
        self.path, self.w, self.h = path, width, height
        cmd = [exe, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
               "-movflags", "+faststart", path]
        flags = 0x08000000 if sys.platform == "win32" else 0     # no console window
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE,
                                     creationflags=flags)

    def add(self, image: QImage):
        img = image.convertToFormat(QImage.Format_RGB888)
        stride, row = img.bytesPerLine(), 3 * self.w
        raw = bytes(img.constBits())
        if stride != row:
            raw = b"".join(raw[y * stride:y * stride + row] for y in range(self.h))
        self.proc.stdin.write(raw)

    def close(self):
        self.proc.stdin.close()
        err = self.proc.stderr.read().decode("utf-8", "replace")
        if self.proc.wait() != 0:
            raise RuntimeError(f"ffmpeg : {err.strip()[:300]}")
        return self.path


class QtWriter:
    kind = "mp4"
    label = "MP4 (H.264, Qt Multimedia)"

    def __init__(self, path, width, height, fps):
        from PySide6.QtCore import QEventLoop, QSize, QTimer, QUrl
        from PySide6.QtMultimedia import (QMediaCaptureSession, QMediaFormat,
                                          QMediaRecorder, QVideoFrameInput,
                                          QVideoFrameFormat)
        self._loop_cls, self._timer = QEventLoop, QTimer
        self.path, self.w, self.h, self.fps = path, width, height, fps
        fmt = QVideoFrameFormat(QSize(width, height), QVideoFrameFormat.Format_RGBA8888)
        fmt.setStreamFrameRate(fps)
        self.input = QVideoFrameInput(fmt)
        self.session = QMediaCaptureSession()
        self.session.setVideoFrameInput(self.input)
        self.rec = QMediaRecorder()
        self.session.setRecorder(self.rec)
        mf = QMediaFormat(QMediaFormat.MPEG4)
        codec = _qt_codec()
        mf.setVideoCodec(codec)
        self.label = "MP4 ({}, Qt Multimedia)".format(
            "H.264" if codec == QMediaFormat.VideoCodec.H264 else "MPEG-4")
        self.rec.setMediaFormat(mf)
        self.rec.setQuality(QMediaRecorder.VeryHighQuality)
        self.rec.setVideoFrameRate(fps)
        self.rec.setVideoResolution(QSize(width, height))
        self.rec.setOutputLocation(QUrl.fromLocalFile(os.path.abspath(path)))
        self.n = 0
        self.rec.record()
        self._wait(lambda: self.rec.recorderState() == QMediaRecorder.RecordingState, 5000)
        if self.rec.error() != QMediaRecorder.NoError:
            raise RuntimeError(self.rec.errorString())

    def _wait(self, done, ms):
        loop = self._loop_cls()
        t = self._timer()
        t.timeout.connect(lambda: loop.quit() if done() else None)
        t.start(5)
        guard = self._timer()
        guard.setSingleShot(True)
        guard.timeout.connect(loop.quit)
        guard.start(ms)
        loop.exec()
        t.stop()
        return done()

    def add(self, image: QImage):
        from PySide6.QtMultimedia import QVideoFrame
        frame = QVideoFrame(image.convertToFormat(QImage.Format_RGBA8888))
        us = int(round(1e6 / self.fps))
        frame.setStartTime(self.n * us)
        frame.setEndTime((self.n + 1) * us)
        while not self.input.sendVideoFrame(frame):     # encoder queue full
            self._wait(lambda: False, 5)
        self.n += 1

    def close(self):
        from PySide6.QtMultimedia import QMediaRecorder
        self.rec.stop()
        self._wait(lambda: self.rec.recorderState() == QMediaRecorder.StoppedState, 60000)
        if self.rec.error() != QMediaRecorder.NoError:
            raise RuntimeError(self.rec.errorString())
        return self.path


def _qt_codec():
    """Best MP4 video codec Qt can encode here (H.264, else MPEG-4), or None."""
    try:
        from PySide6.QtMultimedia import QMediaFormat, QVideoFrameInput  # noqa: F401
        fmt = QMediaFormat(QMediaFormat.MPEG4)
        have = fmt.supportedVideoCodecs(QMediaFormat.Encode)
        for c in (QMediaFormat.VideoCodec.H264, QMediaFormat.VideoCodec.MPEG4):
            if c in have:
                return c
    except Exception:
        pass
    return None


def qt_multimedia_available():
    return _qt_codec() is not None


def find_ffmpeg(hint=None):
    """Path of a working ffmpeg: ``hint`` first, then the PATH."""
    if hint and os.path.isfile(hint):
        return hint
    names = ("ffmpeg.exe", "ffmpeg") if sys.platform == "win32" else ("ffmpeg",)
    for d in os.environ.get("PATH", "").split(os.pathsep):
        for n in names:
            c = os.path.join(d.strip('"'), n)
            if os.path.isfile(c) and os.access(c, os.X_OK):
                return c
    return None


def mp4_backend(ffmpeg_hint=None):
    """('qt' | 'ffmpeg' | None, ffmpeg path): H.264 first (Qt, then
    ffmpeg), then Qt's MPEG-4, then nothing (AVI)."""
    codec = _qt_codec()
    try:
        from PySide6.QtMultimedia import QMediaFormat
        h264 = codec == QMediaFormat.VideoCodec.H264
    except Exception:
        h264 = False
    if h264:
        return "qt", None
    exe = find_ffmpeg(ffmpeg_hint)
    if exe:
        return "ffmpeg", exe
    return ("qt", None) if codec is not None else (None, None)


def open_writer(path, width, height, fps, ffmpeg_hint=None, prefer=None):
    """The best writer for ``path``: MP4 when an MP4 encoder exists, else
    AVI next to it (``path`` changed to .avi). ``prefer`` forces one of
    'qt', 'ffmpeg', 'avi' (tests)."""
    width, height = width - width % 2, height - height % 2
    backend, exe = mp4_backend(ffmpeg_hint)
    if prefer:
        backend = prefer
        exe = exe or find_ffmpeg(ffmpeg_hint)
    if backend == "qt":
        try:
            return QtWriter(path, width, height, fps)
        except Exception:
            exe = exe or find_ffmpeg(ffmpeg_hint)
            backend = "ffmpeg" if exe else None
    if backend == "ffmpeg" and exe:
        return FfmpegWriter(path, width, height, fps, exe)
    root, _ = os.path.splitext(path)
    return AviWriter(root + ".avi", width, height, fps)
