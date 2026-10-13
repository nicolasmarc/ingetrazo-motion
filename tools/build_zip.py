"""Build dist/motion-<version>.zip deterministically and print its sha256.

The zip holds ONE folder, ``motion/``, as IngeTrazo installs it
(``plugins/motion/__init__.py``). Same sources -> same bytes -> same hash.
"""
import hashlib, re, sys, zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "motion"


def version():
    m = re.search(r'__version__\s*=\s*"([^"]+)"', (PKG / "__init__.py").read_text("utf-8"))
    return m.group(1)


def build():
    out = ROOT / "dist" / f"motion-{version()}.zip"
    out.parent.mkdir(exist_ok=True)
    files = sorted(p for p in PKG.rglob("*")
                   if p.is_file() and "__pycache__" not in p.parts
                   and p.suffix not in (".pyc", ".pyo"))
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for p in files:
            info = zipfile.ZipInfo(p.relative_to(ROOT).as_posix(), (1980, 1, 1, 0, 0, 0))
            info.external_attr = 0o644 << 16
            info.compress_type = zipfile.ZIP_DEFLATED
            z.writestr(info, p.read_bytes().replace(b"\r\n", b"\n"))
    return out, hashlib.sha256(out.read_bytes()).hexdigest()


if __name__ == "__main__":
    path, sha = build()
    print(path.relative_to(ROOT).as_posix())
    print(sha)
    sys.exit(0)
