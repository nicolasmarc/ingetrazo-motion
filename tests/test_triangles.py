"""The fast triangle reader gives the same surface and volume as IngeTrazo's
own triangulation (triangles, convex and concave quads, a face with a hole)."""
import sys
import numpy as np
import pytest

from test_panel_ingetrazo import SRC, HAVE_QT

pytestmark = pytest.mark.skipif(not (SRC and HAVE_QT),
                                reason="IngeTrazo source or PySide6 not available")


def _reference(mesh):
    tri = [[(p.x(), p.y(), p.z()) for p in t] for f in mesh.faces for t in f.triangulate()]
    return np.array(tri, float).reshape(-1, 3, 3)


def _area_volume(T):
    cr = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    vol = np.einsum("ij,ij->i", T[:, 0], np.cross(T[:, 1], T[:, 2])).sum() / 6
    return 0.5 * np.linalg.norm(cr, axis=1).sum(), vol


def test_fast_triangles_match_ingetrazo():
    sys.path.insert(0, str(SRC))
    from PySide6.QtGui import QVector3D as V
    from core.mesh import Mesh
    from motion.panel import _mesh_triangles
    m = Mesh()
    m.add_face([V(0, 0, 0), V(1, 0, 0), V(0, 1, 0)])                        # triangle
    m.add_face([V(2, 0, 0), V(3, 0, 0), V(3, 1, 0.2), V(2, 1, 0.2)])        # convex quad
    m.add_face([V(5, 0, 1), V(7, 0, 1), V(6, 0.4, 1), V(6, 2, 1)])          # concave (dart)
    m.add_face([V(0, 0, 3), V(4, 0, 3), V(4, 4, 3), V(0, 4, 3)],            # square with a hole
               [[V(1, 1, 3), V(1, 3, 3), V(3, 3, 3), V(3, 1, 3)]])
    m.add_face([V(0, 0, 5), V(2, 0, 5), V(3, 1, 5), V(2, 2, 5), V(0, 2, 5)])  # pentagon
    fast, ref = _mesh_triangles(m), _reference(m)
    assert len(fast) == len(ref)
    a1, v1 = _area_volume(fast)
    a2, v2 = _area_volume(ref)
    assert a1 == pytest.approx(a2, rel=1e-12)
    assert v1 == pytest.approx(v2, rel=1e-12, abs=1e-12)
