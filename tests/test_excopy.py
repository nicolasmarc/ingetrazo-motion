"""« Copier dans le modèle »: an example rebuilt from the document data
moves like the example itself (every part, along the whole travel)."""
import numpy as np
import pytest

from motion import document as doc, excopy, samples, solver

COPYABLE = {"chaise": samples.chaise_pliante, "porte": samples.porte_sectionnelle,
            "oiseau": samples.oiseau, "draisine": samples.draisine,
            "pendule_double": samples.pendule_double}


def _copy(builder):
    m, info = builder()
    pts, uids, n = {}, {}, 0
    for body, fixed, groups in excopy.parts(m, info):
        for gname, faces, col in groups:
            n += 1
            uid = f"g{n}"
            pts[uid] = np.vstack(faces)
            uids.setdefault(body, []).append(uid)
    data = excopy.data(m, info, uids)
    m2, problems = doc.build(data, lambda u: pts.get(u))
    return m, info, data, m2, problems, uids


def _probe_points(m, body):
    c = body.center
    return [c, c + np.array([0.1, 0, 0]), c + np.array([0, 0.1, 0.05])]


@pytest.mark.parametrize("key", [k for k in COPYABLE if k != "pendule_double"])
def test_copied_example_moves_like_the_example(key):
    m, info, data, m2, problems, uids = _copy(COPYABLE[key])
    assert problems == []
    assert len(m2.bodies) == len(m.bodies)
    assert len(m2.joints) == len(m.joints)
    assert len(m2.drivers) == len(m.drivers)
    by2 = {b.name: b for b in m2.bodies}
    rows1 = solver.sweep(m, 9)
    rows2 = solver.sweep(m2, 9)
    assert all(r["ok"] for r in rows1) and all(r["ok"] for r in rows2)
    worst = 0.0
    for r1, r2 in zip(rows1, rows2):
        for b in m.bodies:
            b2 = by2[b.name]
            for p in _probe_points(m, b):
                w1 = b.world_point(m.pose(r1["x"], b), p)
                w2 = b2.world_point(m2.pose(r2["x"], b2), p)
                worst = max(worst, float(np.linalg.norm(w1 - w2)))
    # the door's rails become chords (2°, 5 mm): a fraction of a millimetre
    assert worst < (2e-3 if key == "porte" else 1e-6), worst


def test_names_stay_unique_and_motors_follow():
    m, info, data, m2, problems, uids = _copy(samples.oiseau)
    names = [b["name"] for b in data["bodies"]] + [j["name"] for j in data["joints"]]
    assert len(names) == len(set(names))
    joints = {j["name"] for j in data["joints"]}
    assert all(d["joint"] in joints for d in data["drivers"])


def test_pendulum_keeps_its_masses_and_dynamics():
    m, info, data, m2, problems, uids = _copy(samples.pendule_double)
    assert data["simulation"]["active"] is True
    assert data["simulation"]["duree"] == pytest.approx(info["simulation"]["duree"])
    assert all(j.get("damper") == pytest.approx(0.0005) for j in data["joints"])
    keys = {data["materiau"][u] for b in m.bodies for u in uids[b.name]}
    assert keys == {"chene"}                       # oak, 710 kg/m³, as in the example


def test_chains_are_not_offered():
    assert set(excopy.NOT_COPYABLE) == {"chaine", "chaine_souple", "draisine"}
