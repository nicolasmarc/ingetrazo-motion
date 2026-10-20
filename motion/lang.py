"""Motion's words in French or English.

The texts are written in French in the code; ``T`` gives them back in
English when IngeTrazo is not set to French (Window ▸ Language), or when
the ``MOTION_LANG`` environment variable says ``en`` (``fr`` forces
French). Outside IngeTrazo (scripts, tests) Motion speaks French unless
told otherwise.

    T("Pièce fixe")                              -> "Fixed part"
    T("{name} : {n} groupe(s).", name=b, n=3)    -> "b: 3 group(s)."

A text missing from the table stays in French: nothing breaks.
"""
import os

_forced = None


def set_language(code):
    """Force ``"fr"`` or ``"en"`` (None: follow IngeTrazo again)."""
    global _forced
    _forced = code


def language():
    if _forced:
        return _forced
    env = os.environ.get("MOTION_LANG", "").strip().lower()
    if env in ("fr", "en"):
        return env
    try:
        from core.i18n import current_language
        code = str(current_language() or "en")
    except Exception:
        return "fr"
    return "fr" if code.lower().startswith("fr") else "en"


def english():
    return language() == "en"


def T(text, **kw):
    out = EN.get(text, text) if english() else text
    return out.format(**kw) if kw else out


EN = {}          # filled by motion/lang_en.py
try:
    from .lang_en import EN as _EN
    EN.update(_EN)
except Exception:           # pragma: no cover
    pass
