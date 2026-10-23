"""English for Motion's texts (see motion/lang.py). Keys: the French text
as written in the code, placeholders included."""

EN = {
    # -- panel ------------------------------------------------------------------
    "Importer STEP…": "Import STEP…",
    "Assemblage STEP (AP203/214/242) : une pièce = un groupe, en une seule étape d'annulation.":
        "STEP assembly (AP203/214/242): one part = one group, in a single undo step.",
    "Détecter…": "Detect…",
    "Analyse les pièces (la sélection, sinon tout le modèle) et propose les liaisons : arbres "
    "dans des alésages (pivots), appuis plans (glissières), engrenages. Rien n'est appliqué "
    "avant validation.":
        "Scans the parts (the selection, else the whole model) and proposes joints: shafts in "
        "bores (pivots), flat contacts (sliders), gears. Nothing is applied until you confirm.",
    "Calcul des masses…": "Computing masses…",
    "Nom": "Name",
    "Type": "Type",
    "Détail": "Detail",
    "Supprimer": "Delete",
    "Matériau": "Material",
    "Matériau de la pièce choisie dans la liste (ou des groupes sélectionnés) : masse, centre "
    "de gravité et inertie en découlent.":
        "Material of the part chosen in the list (or of the selected groups): its mass, centre "
        "of gravity and inertia follow.",
    "Nouveau…": "New…",
    "Ajouter un matériau (ou corriger une valeur) pour ce document.":
        "Add a material (or correct a value) for this document.",
    "Liaison choisie": "Selected joint",
    "Pièce A": "Part A",
    "Pièce B": "Part B",
    "Butées": "Limits",
    "Course moteur": "Driver travel",
    "Moteur (plusieurs possibles : même 0–100 %, chacun sa course)":
        "Driver (several allowed: same 0–100 %, each its own travel)",
    "Coché : cette liaison est un moteur. Vitesse = course ; déphasage = valeur de départ.":
        "Ticked: this joint is a driver. Speed = travel; phase = start value.",
    "Ressort": "Spring",
    "Raideur": "Stiffness",
    "Amort.": "Damper",
    "Raideur (N·m/rad pour un pivot, N/m pour une glissière ou une liaison élastique), au "
    "repos dans la position dessinée. 0 : aucun.":
        "Stiffness (N·m/rad for a pivot, N/m for a slider or an elastic joint), at rest in the "
        "drawn position. 0: none.",
    "Raideur en translation (N/m) ; en rotation : 1 % de cette valeur en N·m/rad.":
        "Translational stiffness (N/m); rotational: 1 % of it, in N·m/rad.",
    "Amortissement visqueux (N·m·s/rad ou N·s/m). Dynamique seulement.":
        "Viscous damping (N·m·s/rad or N·s/m). Dynamics only.",
    "Moteur en couple": "Torque driver",
    "Dynamique : un couple (N·m) ou une force (N) constant ; le mécanisme répond (au lieu "
    "d'imposer la course).":
        "Dynamics: a constant torque (N·m) or force (N); the mechanism responds (instead of "
        "following a set travel).",
    "Appliquer": "Apply",
    "Lecture / pause": "Play / pause",
    "Aller-retour": "Back and forth",
    "Exporter la vidéo…": "Export video…",
    "Vidéo en écran partagé : la vue IngeTrazo à gauche, l'animation ombrée et les mesures à "
    "droite.":
        "Split-screen video: the IngeTrazo view on the left, the shaded animation and the "
        "readings on the right.",
    "Jeu mini": "Min. gap",
    "Pas": "Steps",
    "Dynamique": "Dynamics",
    "durée": "duration",
    "Chocs": "Shocks",
    "Ralenti": "Slow motion",
    "Simulation dynamique : la vidéo la montre ce nombre de fois plus lentement (1 : en temps réel).":
        "Dynamic simulation: the video shows it this many times slower (1: real time).",
    "ralenti ×{k}": "slow motion ×{k}",
    "Chocs et appuis entre pièces libres (dominos, blocs, butées posées) : les boîtes des pièces se repoussent, avec le frottement et le rebond de leurs matériaux. Les pièces tenues par une liaison ne se heurtent pas.":
        "Shocks and resting contacts between free parts (dominoes, blocks, loose stops): the parts' boxes push each other back, with the friction and the rebound of their materials. Parts held by a joint do not knock each other.",
    "Chocs entre pièces : enfoncement maxi {d} mm (pièces indéformables).":
        "Shocks between parts: deepest overlap {d} mm (rigid parts).",
    "Masses (matériaux), pesanteur, ressorts, amortisseurs, butées, moteurs en couple : "
    "Valider, la lecture et la vidéo suivent la simulation dans le temps ; les moteurs en "
    "vitesse font leur course en « durée » et le rapport donne leur couple.":
        "Masses (materials), gravity, springs, dampers, limits, torque drivers: Validate, "
        "playback and the video follow the simulation in time; speed drivers run their "
        "travel over the « duration » and the report gives their torque.",
    "— non défini —": "— not set —",
    "{name} : {m} kg, volume {v} cm³ — centre de gravité ({x} ; {y} ; {z}) mm":
        "{name}: {m} kg, volume {v} cm³ — centre of gravity ({x}; {y}; {z}) mm",
    "{name} : volume {v} cm³ — choisir un matériau": "{name}: volume {v} cm³ — choose a material",
    " ; {n} groupe(s) sans matériau": "; {n} group(s) without material",
    "{label} : {n} groupe(s).": "{label}: {n} group(s).",
    "Matériau du document": "Document material",
    "(nouveau)": "(new)",
    "Partir de": "Start from",
    "Masse volumique": "Density",
    "Module d'Young": "Young's modulus",
    "Frottement (sec)": "Friction (dry)",
    "Restitution (chocs)": "Restitution (impacts)",
    "Annuler": "Cancel",
    "Matériau « {name} » : {rho} kg/m³.": "Material « {name} »: {rho} kg/m³.",
    "matériau ?": "material?",
    " (+{n} sans matériau)": " (+{n} without material)",
    "{name} : {n} groupe(s).": "{name}: {n} group(s).",
    "{n} groupe(s)": "{n} group(s)",
    " · moteur": " · driver",
    "Hélicoïdale": "Screw",
    "Pas (mm par tour) :": "Pitch (mm per turn):",
    "{name} : {a} ↔ {b}, pas {p} mm.": "{name}: {a} ↔ {b}, pitch {p} mm.",
    "{n} galet(s) sur le rail de {a}.": "{n} roller(s) on the rail of {a}.",
    "Moteur : {j} ({n} moteur(s)).": "Driver: {j} ({n} driver(s)).",
    "{name} mis à jour.": "{name} updated.",
    "{name} supprimé.": "{name} deleted.",
    "Choisir une pièce dans la liste, ou sélectionner des groupes.":
        "Choose a part in the list, or select groups.",
    "Sélectionner un ou plusieurs groupes.": "Select one or more groups.",
    "définir d'abord au moins deux pièces": "define at least two parts first",
    "sélectionner UNE arête (ou ligne de construction)": "select ONE edge (or construction line)",
    "arête de longueur nulle": "zero-length edge",
    "sélectionner des arêtes au point de liaison (un cercle autour du centre, ou une arête)":
        "select edges at the joint point (a circle around the centre, or an edge)",
    "sélectionner UNE arête sur l'axe de l'alésage (et un cercle de la sphère)":
        "select ONE edge on the bore's axis (and a circle of the ball)",
    "Dynamique : simulation dans le temps.": "Dynamics: simulation in time.",
    "Dynamique : arrêtée.": "Dynamics: off.",
    "sélectionner le rail et un trait par galet": "select the rail and one stroke per roller",
    "aucun galet : tracer un petit trait en travers du rail":
        "no roller: draw a short stroke across the rail",
    "définir d'abord la pièce fixe qui porte le rail": "define the fixed part carrying the rail first",
    "aucune pièce mobile près du galet": "no moving part near the roller",
    "choisir une liaison dans la liste": "choose a joint in the list",
    "pas de moteur": "no driver",
    "✔ Mécanisme valide de 0 à 100 %.": "✔ Mechanism valid from 0 to 100 %.",
    "✖ {n} problème(s) :": "✖ {n} problem(s):",
    "Jeu le plus faible : {d} mm entre {a} ({ga}) et {b} ({gb}) à {p} %.":
        "Smallest gap: {d} mm between {a} ({ga}) and {b} ({gb}) at {p} %.",
    "Degrés de liberté (sans moteur) : {n}": "Degrees of freedom (without drivers): {n}",
    "Simulation dynamique de {t} s : énergie {e0} → {e1} mJ.":
        "Dynamic simulation of {t} s: energy {e0} → {e1} mJ.",
    "Couple moteur {nm} : maxi {mx} N·m (à {t} s), moyen {mean} N·m.":
        "Driver torque {nm}: max {mx} N·m (at {t} s), mean {mean} N·m.",
    "{name} : en butée à partir de {t} s.": "{name}: on its limit from {t} s.",
    "ⓘ sans matériau, masse estimée (boîtes, 700 kg/m³) : {names}.":
        "ⓘ no material, mass estimated (boxes, 700 kg/m³): {names}.",
    "Masse totale : {m} kg — ": "Total mass: {m} kg — ",
    "ⓘ {n} groupe(s) sans matériau : masse non comptée.":
        "ⓘ {n} group(s) without material: mass not counted.",
    "Trajectoire de {n} point(s).": "Trajectory of {n} point(s).",
    "STEP : {n} pièce(s) importée(s)": "STEP: {n} part(s) imported",
    " — non lu : {w}": " — not read: {w}",
    "Détection : {p} pivot(s), {s} glissière(s), {g} engrenage(s) ; pièce fixe : {f}.":
        "Detection: {p} pivot(s), {s} slider(s), {g} gear pair(s); fixed part: {f}.",
    " Choisir un moteur dans la liste.": " Choose a driver in the list.",
    "Détecter les liaisons": "Detect joints",
    "{n} pièce(s) analysée(s) : aucune liaison reconnue (arbre dans un alésage, appuis plans, "
    "engrenage).":
        "{n} part(s) scanned: no joint recognised (shaft in a bore, flat contacts, gear).",
    "{n} pièce(s) analysée(s). Cocher les liaisons à créer :":
        "{n} part(s) scanned. Tick the joints to create:",
    "Liaisons proposées": "Proposed joints",
    "Liaison": "Joint",
    "Pièces": "Parts",
    "Confiance": "Confidence",
    "Engrenage": "Gear pair",
    "Créer les liaisons": "Create the joints",
    "Importer un assemblage STEP": "Import a STEP assembly",
    "Importer un assemblage STEP…": "Import a STEP assembly…",
    "Importer STEP": "Import STEP",
    "Lecture : {e}": "Playback: {e}",
    "{p} % : position non atteinte.": "{p} %: position not reached.",
    "Exporter la vidéo : pas de moteur.": "Export video: no driver.",
    "Exporter la vidéo": "Export video",
    "Durée": "Duration",
    "Résolution": "Resolution",
    "Aller-retour (0 → 100 → 0 %)": "Back and forth (0 → 100 → 0 %)",
    "Indiquer ffmpeg.exe…": "Locate ffmpeg.exe…",
    "AVI Motion-JPEG — pour du MP4, indiquer ffmpeg.exe": "AVI Motion-JPEG — for MP4, locate ffmpeg.exe",
    "Vidéo MP4 (*.mp4)": "MP4 video (*.mp4)",
    "Vidéo AVI (*.avi)": "AVI video (*.avi)",
    "Rendu de la vidéo…": "Rendering the video…",
    "Vidéo enregistrée : {out} — {label}.": "Video saved: {out} — {label}.",
    "Export annulé.": "Export cancelled.",
    "Panneau Motion": "Motion panel",
    "Mécanismes : pivots, glissières, rails, validation.": "Mechanisms: pivots, sliders, rails, validation.",
    # video readings
    "{v} tr": "{v} turns",
    "temps {t} s": "time {t} s",
    "couple {nm} {v} N·m": "torque {nm} {v} N·m",
    "jeu mini {d} mm": "min. gap {d} mm",
    "aucune collision": "no collision",
    "position non atteinte": "position not reached",
    "collision : ": "collision: ",
    "Animation Motion": "Motion animation",
    "Mécanisme": "Mechanism",
    # -- tools --------------------------------------------------------------------
    "Pièce fixe (Fixed part) — les groupes sélectionnés forment le bâti.":
        "Fixed part — the selected groups form the frame.",
    "Pièce mobile (Moving part) — les groupes sélectionnés bougent ensemble.":
        "Moving part — the selected groups move together.",
    "Pivot — sélectionner UNE arête ou ligne de construction sur l'axe.":
        "Pivot — select ONE edge or construction line on the axis.",
    "Glissière (Slider) — sélectionner UNE arête dans le sens du coulissement.":
        "Slider — select ONE edge along the travel.",
    "Guide / rail — sélectionner les arêtes du rail + un petit trait en travers à chaque galet.":
        "Guide / rail — select the rail's edges + a short stroke across it at each roller.",
    "Moteur 0–100 % (Driver) — la liaison choisie dans la liste, avec la course indiquée.":
        "Driver 0–100 % — the joint chosen in the list, over the travel typed.",
    "Valider (Validate) — de 0 à 100 % : collisions, butées, jeu minimal.":
        "Validate — from 0 to 100 %: collisions, limits, minimum gap.",
    "Trajectoire (Trajectory) — trace les extrémités des arêtes sélectionnées.":
        "Trajectory — traces the end points of the selected edges.",
    "Pivot glissant (Cylindrical) — UNE arête sur l'axe : la pièce tourne et coulisse.":
        "Cylindrical — ONE edge on the axis: the part turns and slides.",
    "Rotule (Ball) — des arêtes autour du centre (un petit cercle) : toutes les rotations.":
        "Ball joint — edges around the centre (a small circle): every rotation.",
    "Encastrement (Lock) — des arêtes au point de liaison : les deux pièces n'en font qu'une.":
        "Lock — edges at the joint point: the two parts move as one.",
    "Liaison élastique (Elastic) — des arêtes au point de liaison : un silentbloc, rigide en "
    "cinématique, ressorts et amortisseurs en dynamique.":
        "Elastic joint — edges at the joint point: a rubber mount, rigid in kinematics, "
        "springs and dampers in dynamics.",
    "Linéaire annulaire (Annular) — UNE arête sur l'axe de l'alésage, plus un cercle de la "
    "sphère : la sphère glisse dans l'alésage et tourne en tous sens.":
        "Annular (sphere in a cylinder) — ONE edge on the bore's axis, plus a circle of the "
        "ball: the ball slides in the bore and turns every way.",
    "Ponctuelle (Point–plane) — UNE arête le long de la normale, partant du point de contact : "
    "le point reste sur le plan.":
        "Point–plane — ONE edge along the normal, from the contact point: the point stays on "
        "the plane.",
    "Appui plan (Planar) — UNE arête le long de la normale au plan : la pièce glisse sur le "
    "plan et tourne autour de la normale.":
        "Planar — ONE edge along the plane's normal: the part slides on the plane and turns "
        "about the normal.",
    "Hélicoïdale (Screw) — UNE arête sur l'axe, puis le pas : la pièce avance d'un pas par "
    "tour (pas négatif : filet à gauche).":
        "Screw — ONE edge on the axis, then the pitch: the part advances one pitch per turn "
        "(negative pitch: left-hand thread).",
    # -- kinds ---------------------------------------------------------------------
    "Pièce fixe": "Fixed part",
    "Pièce mobile": "Moving part",
    "Pivot": "Pivot",
    "Glissière": "Slider",
    "Guide / rail": "Guide / rail",
    "Pivot glissant": "Cylindrical",
    "Rotule": "Ball joint",
    "Encastrement": "Lock",
    "Liaison élastique": "Elastic joint",
    "Linéaire annulaire": "Annular",
    "Ponctuelle": "Point–plane",
    "Appui plan": "Planar",
    "Couplage": "Coupling",
    # default names
    "fixe": "fixed", "pièce": "part", "pivot": "pivot", "glissière": "slider",
    "galet": "roller", "pivot glissant": "cylindrical", "rotule": "ball", "encastrement": "lock",
    "élastique": "elastic", "linéaire annulaire": "annular", "ponctuelle": "point-plane",
    "appui plan": "planar", "hélicoïdale": "screw", "engrenage": "gear",
    # -- validation ---------------------------------------------------------------
    "{p} % : position non atteinte (résidu {r})": "{p} %: position not reached (residual {r})",
    "{p} % : collision {a} / {b}": "{p} %: collision {a} / {b}",
    "{p} % : butée dépassée {name} = {v}": "{p} %: limit exceeded {name} = {v}",
    "{p} % : jeu {d} mm < {j} mm entre {a} ({ga}) et {b} ({gb})":
        "{p} %: gap {d} mm < {j} mm between {a} ({ga}) and {b} ({gb})",
    "sélectionner les arêtes du rail": "select the rail's edges",
    "aucune pièce fixe : ajouter une « pièce fixe »": "no fixed part: add a « fixed part »",
    "aucun moteur : choisir une liaison et « Moteur 0–100 % »":
        "no driver: choose a joint and « Driver 0–100 % »",
    "{name} : aucun groupe trouvé dans le modèle": "{name}: no group found in the model",
    "{name} : pièce inconnue": "{name}: unknown part",
    "{name} : relie une pièce à elle-même": "{name}: joins a part to itself",
    "{name} : liaison couplée inconnue": "{name}: unknown coupled joint",
    "moteur {name} : liaison inconnue": "driver {name}: unknown joint",
    # -- examples -------------------------------------------------------------------
    "Modèle (document)": "Model (document)",
    "Exemple : chaise pliante": "Example: folding chair",
    "Exemple : porte sectionnelle": "Example: sectional door",
    "Exemple : oiseau à battement d'ailes": "Example: flapping bird",
    "Exemple : draisine à balancier": "Example: lever handcar",
    "Exemple : chaîne détendue et tendeur": "Example: slack chain and tensioner",
    "Exemple : chaîne souple (dynamique)": "Example: supple chain (dynamics)",
    "Exemple : pendule double (dynamique)": "Example: double pendulum (dynamics)",
    "Chaise pliante en X": "X folding chair",
    "Porte sectionnelle": "Sectional door",
    "Oiseau à battement d'ailes": "Flapping bird",
    "Draisine à balancier": "Lever handcar",
    "Chaîne détendue et tendeur": "Slack chain and tensioner",
    "Chaîne souple (dynamique)": "Supple chain (dynamics)",
    "Pendule double (dynamique)": "Double pendulum (dynamics)",
    "flèche {s} mm ({p} % de l'entraxe)": "sag {s} mm ({p} % of the centre distance)",
    "entraxe {c} mm": "centre distance {c} mm",
    "chaîne tendue (pas de mou)": "chain taut (no slack)",
    "course dans la lumière {d} mm": "travel in the slot {d} mm",
    "bras {v}°": "arm {v}°",
    "main {v}°": "hand {v}°",
    "pliage {v}°": "fold {v}°",
    "balancier {v}°": "lever {v}°",
    "avance {v} m": "travel {v} m",
    "roues {v} tr": "wheels {v} turns",
    "pignon mené {v} tr": "driven sprocket {v} turns",
    "dynamique, amortissement {v}/s": "dynamics, damping {v}/s",
    "bras supérieur {v}°": "upper arm {v}°",
    "bras inférieur {v}° (relatif)": "lower arm {v}° (relative)",
    "énergie {e} mJ (cinétique {k})": "energy {e} mJ (kinetic {k})",
    # -- materials -------------------------------------------------------------------
    "Chêne": "Oak", "Hêtre": "Beech", "Frêne": "Ash", "Noyer": "Walnut",
    "Pin sylvestre": "Scots pine", "Épicéa": "Spruce", "Douglas": "Douglas fir",
    "Contreplaqué bouleau": "Birch plywood", "MDF": "MDF", "OSB": "OSB",
    "Acier": "Steel", "Inox": "Stainless steel", "Aluminium": "Aluminium", "Laiton": "Brass",
    "Bronze": "Bronze", "Fonte grise": "Grey cast iron", "POM (acétal)": "POM (acetal)",
    "PA 6 (nylon)": "PA 6 (nylon)", "PEHD": "HDPE", "PLA (impression 3D)": "PLA (3D printing)",
    "ABS": "ABS", "PMMA (acrylique)": "PMMA (acrylic)", "Verre": "Glass",
}
