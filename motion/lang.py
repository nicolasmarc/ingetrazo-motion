"""Motion's words in French, English or Spanish.

The texts are written in French in the code; ``T`` gives them back in
Spanish when IngeTrazo is set to Spanish, in English when it is set to
any other language but French (Window ▸ Language), or as the
``MOTION_LANG`` environment variable says (``fr``, ``en`` or ``es``). Outside IngeTrazo (scripts, tests) Motion speaks French unless
told otherwise.

    T("Pièce fixe")                              -> "Fixed part"
    T("{name} : {n} groupe(s).", name=b, n=3)    -> "b: 3 group(s)."

A text missing from the table stays in French: nothing breaks.
"""
import os

_forced = None


def set_language(code):
    """Force ``"fr"``, ``"en"`` or ``"es"`` (None: follow IngeTrazo again)."""
    global _forced
    _forced = code


def language():
    if _forced:
        return _forced
    env = os.environ.get("MOTION_LANG", "").strip().lower()
    if env in ("fr", "en", "es"):
        return env
    try:
        from core.i18n import current_language
        code = str(current_language() or "en")
    except Exception:
        return "fr"
    code = code.lower()
    return "fr" if code.startswith("fr") else "es" if code.startswith("es") else "en"


def english():
    """Not French (English, or Spanish falling back on English)."""
    return language() != "fr"


def T(text, **kw):
    lang = language()
    out = (text if lang == "fr" else ES.get(text) or EN.get(text, text) if lang == "es"
           else EN.get(text, text))
    return out.format(**kw) if kw else out


EN = {}          # filled by motion/lang_en.py
ES = {}          # and motion/lang_es.py
try:
    from .lang_en import EN as _EN
    EN.update(_EN)
except Exception:           # pragma: no cover
    pass
try:
    from .lang_es import ES as _ES
    ES.update(_ES)
except Exception:           # pragma: no cover
    pass
