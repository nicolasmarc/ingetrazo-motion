"""Every French text the code passes to _T / T has its English entry
(the panel follows IngeTrazo's language)."""
import ast, os, sys
HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from motion.lang_en import EN
from motion.lang_es import ES


def _texts():
    out = {}
    pkg = os.path.join(ROOT, "motion")
    for fn in sorted(os.listdir(pkg)):
        if not fn.endswith(".py") or fn in ("lang_en.py", "lang_es.py"):
            continue
        tree = ast.parse(open(os.path.join(pkg, fn), encoding="utf-8").read())
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                    and node.func.id in ("_T", "T") and node.args
                    and isinstance(node.args[0], ast.Constant)
                    and isinstance(node.args[0].value, str)):
                out.setdefault(node.args[0].value, f"{fn}:{node.lineno}")
            # the tables translated when shown (tool tips, kinds of joint)
            if (isinstance(node, ast.Assign) and len(node.targets) == 1
                    and isinstance(node.targets[0], ast.Name)
                    and node.targets[0].id in ("TIPS", "KIND_NAMES")
                    and isinstance(node.value, ast.Dict)):
                for v in node.value.values:
                    if isinstance(v, ast.Constant) and isinstance(v.value, str):
                        out.setdefault(v.value, f"{fn}:{v.lineno}")
    return out


def test_every_new_text_has_its_english():
    missing = {t: where for t, where in _texts().items() if t not in EN}
    assert not missing, "\n".join(f"{w}: {t!r}" for t, w in missing.items())


def test_no_text_is_translated_twice():
    tree = ast.parse(open(os.path.join(ROOT, "motion", "lang_en.py"), encoding="utf-8").read())
    d = next(n.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
             and isinstance(n.value, ast.Dict))
    keys = [k.value for k in d.keys if isinstance(k, ast.Constant)]
    twice = sorted({k for k in keys if keys.count(k) > 1})
    assert not twice, twice


def test_every_text_has_its_spanish():
    missing = {t: where for t, where in _texts().items() if t not in ES}
    assert not missing, "\n".join(f"{w}: {t!r}" for t, w in missing.items())


def test_spanish_keeps_the_placeholders():
    import re
    ph = lambda s: sorted(re.findall(r"\{\w*\}", s))
    bad = [k for k, v in ES.items() if ph(k) != ph(v)]
    assert not bad, bad
    assert set(ES) <= set(EN) | set(_texts())


def test_the_panel_speaks_spanish():
    from motion import lang, icons
    try:
        lang.set_language("es")
        assert lang.T("Pièce fixe") == "Pieza fija"
        assert lang.T("Exemple copié : {n} pièce(s), {j} liaison(s) — modifiables.", n=2, j=1) \
            == "Ejemplo copiado: 2 pieza(s), 1 unión(es), modificables."
        assert lang.T("pas dans la table") == "pas dans la table"
        assert icons.label("pivot") == "Pivote"
        lang.set_language("en")
        assert icons.label("pivot") == "Pivot" and lang.T("Pièce fixe") == "Fixed part"
        lang.set_language("fr")
        assert icons.label("slider") == "Glissière"
    finally:
        lang.set_language(None)
