"""Materials: what the dynamics needs to know about a part's matter.

``density`` (kg/m^3) gives the mass and inertia from the part's volume;
``friction`` (dry sliding, indicative), ``restitution`` (shocks) and
``young`` (GPa, stiffness: flexible parts) come in later steps.

The values are common reference averages (woods at about 12 % moisture,
along the grain for the modulus); a supplier's or a species' own figure is
better — the library is a starting point, and a document may add or
override materials (``data["matieres"]``).
"""
import re
import unicodedata

#: key -> properties. Keys are stable (stored in documents); names are shown.
LIBRARY = {
    # woods
    "chene": dict(name="Chêne", family="bois", density=710, young=12.0, friction=0.40, restitution=0.40),
    "hetre": dict(name="Hêtre", family="bois", density=720, young=14.0, friction=0.40, restitution=0.40),
    "frene": dict(name="Frêne", family="bois", density=690, young=13.0, friction=0.40, restitution=0.40),
    "noyer": dict(name="Noyer", family="bois", density=650, young=11.0, friction=0.40, restitution=0.40),
    "pin": dict(name="Pin sylvestre", family="bois", density=520, young=11.0, friction=0.40, restitution=0.40),
    "epicea": dict(name="Épicéa", family="bois", density=450, young=11.0, friction=0.40, restitution=0.40),
    "douglas": dict(name="Douglas", family="bois", density=530, young=12.0, friction=0.40, restitution=0.40),
    "contreplaque": dict(name="Contreplaqué bouleau", family="bois", density=680, young=8.0,
                         friction=0.40, restitution=0.35),
    "mdf": dict(name="MDF", family="bois", density=750, young=3.0, friction=0.40, restitution=0.30),
    "osb": dict(name="OSB", family="bois", density=620, young=4.0, friction=0.40, restitution=0.30),
    # metals
    "acier": dict(name="Acier", family="métal", density=7850, young=210.0, friction=0.50, restitution=0.60),
    "inox": dict(name="Inox", family="métal", density=7900, young=193.0, friction=0.50, restitution=0.60),
    "aluminium": dict(name="Aluminium", family="métal", density=2700, young=70.0, friction=0.45,
                      restitution=0.50),
    "laiton": dict(name="Laiton", family="métal", density=8500, young=100.0, friction=0.35, restitution=0.50),
    "bronze": dict(name="Bronze", family="métal", density=8800, young=110.0, friction=0.30, restitution=0.50),
    "fonte": dict(name="Fonte grise", family="métal", density=7200, young=110.0, friction=0.40,
                  restitution=0.45),
    # plastics
    "pom": dict(name="POM (acétal)", family="plastique", density=1410, young=3.0, friction=0.20,
                restitution=0.50),
    "pa": dict(name="PA 6 (nylon)", family="plastique", density=1140, young=2.8, friction=0.30,
               restitution=0.50),
    "pehd": dict(name="PEHD", family="plastique", density=950, young=1.0, friction=0.20, restitution=0.45),
    "abs": dict(name="ABS", family="plastique", density=1050, young=2.2, friction=0.35, restitution=0.45),
    "pmma": dict(name="PMMA (acrylique)", family="plastique", density=1190, young=3.0, friction=0.40,
                 restitution=0.45),
    # other
    "verre": dict(name="Verre", family="autre", density=2500, young=70.0, friction=0.40, restitution=0.50),
}

FIELDS = ("name", "family", "density", "young", "friction", "restitution")

# words that name a material in a part or paint name (lower case, no accents)
_WORDS = [
    ("contreplaque", ("contreplaque", "contre-plaque", "plywood", "multiplis")),
    ("mdf", ("mdf", "medium")),
    ("osb", ("osb",)),
    ("inox", ("inox", "stainless")),
    ("aluminium", ("alu", "aluminium", "aluminum")),
    ("laiton", ("laiton", "brass")),
    ("bronze", ("bronze",)),
    ("fonte", ("fonte", "cast iron")),
    ("acier", ("acier", "steel", "metal", "s235", "s355", "zingue")),
    ("chene", ("chene", "oak")),
    ("hetre", ("hetre", "beech")),
    ("frene", ("frene", "ash")),
    ("noyer", ("noyer", "walnut")),
    ("douglas", ("douglas",)),
    ("epicea", ("epicea", "spruce", "sapin", "fir", "c24")),
    ("pin", ("pin", "pine")),
    ("pom", ("pom", "acetal", "delrin")),
    ("pa", ("nylon", "polyamide", "pa6", "pa 6")),
    ("pehd", ("pehd", "hdpe", "polyethylene")),
    ("abs", ("abs",)),
    ("pmma", ("pmma", "plexi", "acrylique", "acrylic")),
    ("verre", ("verre", "glass")),
]


def _plain(s):
    s = unicodedata.normalize("NFKD", str(s)).encode("ascii", "ignore").decode()
    return s.lower()


def library(data=None):
    """The library, with a document's own materials added or overriding."""
    out = {k: dict(v) for k, v in LIBRARY.items()}
    for k, v in ((data or {}).get("matieres") or {}).items():
        rec = dict(out.get(k, {}))
        rec.update({f: v[f] for f in FIELDS if f in v})
        rec.setdefault("name", k)
        rec.setdefault("family", "autre")
        rec.setdefault("young", None)
        rec.setdefault("friction", None)
        rec.setdefault("restitution", None)
        if "density" in rec:
            out[k] = rec
    return out


def guess(*names):
    """Material key named in a part's or a paint's name ('Pied chêne',
    'Steel_brushed', 'Axe inox'), else None."""
    text = " ".join(_plain(n) for n in names if n)
    for key, words in _WORDS:
        for w in words:
            if re.search(r"(?<![a-z])" + re.escape(w) + r"(?![a-z])", text):
                return key
    return None
