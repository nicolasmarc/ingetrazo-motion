"""Every French text the code passes to _T / T has its English entry
(the panel follows IngeTrazo's language)."""
import ast, os, sys
HERE = os.path.dirname(__file__)
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from motion.lang_en import EN


def _texts():
    out = {}
    pkg = os.path.join(ROOT, "motion")
    for fn in sorted(os.listdir(pkg)):
        if not fn.endswith(".py") or fn == "lang_en.py":
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
