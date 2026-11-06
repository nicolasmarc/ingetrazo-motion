"""Spanish for Motion's texts (see motion/lang.py). Keys: the French text
as written in the code, placeholders included; a text missing here
falls back to English."""

ES = {
    'Importer STEP…':
        'Importar STEP…',
    "Assemblage STEP (AP203/214/242) : une pièce = un groupe, en une seule étape d'annulation.":
        'Ensamblaje STEP (AP203/214/242): una pieza = un grupo, en un solo paso de deshacer.',
    'Détecter…':
        'Detectar…',
    "Analyse les pièces (la sélection, sinon tout le modèle) et propose les liaisons : arbres dans des alésages (pivots), appuis plans (glissières), engrenages. Rien n'est appliqué avant validation.":
        'Analiza las piezas (la selección, si no todo el modelo) y propone las uniones: ejes en alojamientos (pivotes), apoyos planos (correderas), engranajes. No se aplica nada antes de validar.',
    'Calcul des masses…':
        'Cálculo de masas…',
    'Glisser pour agrandir la liste':
        'Arrastrar para agrandar la lista',
    'Copier dans le modèle':
        'Copiar en el modelo',
    "Crée les pièces de l'exemple dans le document (vrais groupes, ombrés, enregistrés dans le fichier) avec leurs liaisons et moteurs, à modifier comme un mécanisme fait à la main. L'exemple lui-même ne change pas.":
        'Crea las piezas del ejemplo en el documento (grupos reales, sombreados, guardados en el archivo) con sus uniones y motores, para modificarlas como un mecanismo hecho a mano. El ejemplo en sí no cambia.',
    "Le document a déjà un mécanisme. Le remplacer par celui de l'exemple ? (Ses groupes restent dans le modèle.)":
        'El documento ya tiene un mecanismo. ¿Sustituirlo por el del ejemplo? (Sus grupos se quedan en el modelo.)',
    'Exemple copié : {n} pièce(s), {j} liaison(s) — modifiables.':
        'Ejemplo copiado: {n} pieza(s), {j} unión(es), modificables.',
    "Matière de l'exemple":
        'Material del ejemplo',
    'Simulation dynamique…':
        'Simulación dinámica…',
    'Simulation interrompue : ▶ ou « Valider » la relance.':
        'Simulación interrumpida: ▶ o « Validar » la reanuda.',
    'Validation…':
        'Validación…',
    'Validation interrompue.':
        'Validación interrumpida.',
    '+{n} autres':
        '+{n} más',
    "Couplage (Coupling) — engrenage, pignon-crémaillère, roulement : deux liaisons de la liste dont l'une entraîne l'autre selon un rapport.":
        'Acoplamiento (Coupling) — engranaje, piñón-cremallera, rodadura: dos uniones de la lista, una arrastra a la otra según una relación.',
    'il faut deux liaisons (pivots ou glissières) dans la liste':
        'hacen falta dos uniones (pivotes o correderas) en la lista',
    'Liaison menante':
        'Unión motriz',
    'Liaison menée':
        'Unión arrastrada',
    'Dents (menante → menée)':
        'Dientes (motriz → arrastrada)',
    'Denture intérieure (même sens de rotation)':
        'Dentado interior (mismo sentido de giro)',
    'Rayon primitif (pignon, roue)':
        'Radio primitivo (piñón, rueda)',
    'Inverser le sens':
        'Invertir el sentido',
    'Rapport':
        'Relación',
    'choisir deux liaisons différentes':
        'elegir dos uniones distintas',
    'Couplage {name} : rapport {r}.':
        'Acoplamiento {name}: relación {r}.',
    "sélectionner les groupes de DEUX pièces (ou une arête sur l'axe)":
        'seleccionar los grupos de DOS piezas (o una arista sobre el eje)',
    'aucune rotule trouvée (une sphère dans une sphère creuse)':
        'no se ha encontrado ninguna rótula (una esfera en una esfera hueca)',
    'aucun coulissement trouvé (appuis plans, ou arbre dans un alésage)':
        'no se ha encontrado ningún deslizamiento (apoyos planos, o eje en un alojamiento)',
    '{a} ↔ {b} : ':
        '{a} ↔ {b}: ',
    'aucun axe commun trouvé (un arbre dans un alésage)':
        'no se ha encontrado ningún eje común (un eje en un alojamiento)',
    " — sélectionner une arête sur l'axe.":
        ' — seleccionar una arista sobre el eje.',
    "trouvé d'après les formes : {d}.":
        'encontrado según las formas: {d}.',
    'pièce sans géométrie':
        'pieza sin geometría',
    'Pièces (Parts) — les groupes sélectionnés : le premier ensemble devient le bâti, les suivants des pièces mobiles. Sans sélection : détection automatique de tout le modèle (pièces et liaisons, à cocher).':
        'Piezas (Parts) — los grupos seleccionados: el primer conjunto pasa a ser el bastidor, los siguientes piezas móviles. Sin selección: detección automática de todo el modelo (piezas y uniones, para marcar).',
    "Liaison (Joint) — DEUX pièces sélectionnées : la liaison est trouvée d'après leurs formes (pivot, rotule, glissière), à confirmer ou changer. UNE arête : la liaison à la main, sur cet axe.":
        'Unión (Joint) — DOS piezas seleccionadas: la unión se encuentra según sus formas (pivote, rótula, corredera), para confirmar o cambiar. UNA arista: la unión a mano, sobre ese eje.',
    'Plus…':
        'Más…',
    'Toutes les liaisons et les outils, pour les cas particuliers.':
        'Todas las uniones y herramientas, para los casos particulares.',
    'Pièces':
        'Piezas',
    "ces groupes forment déjà une pièce : en choisir d'autres":
        'estos grupos ya forman una pieza: elegir otros',
    "sélectionner DEUX pièces (ou une arête sur l'axe)":
        'seleccionar DOS piezas (o una arista sobre el eje)',
    "{a} ↔ {b} : rien trouvé d'après les formes — sélectionner une arête sur l'axe.":
        '{a} ↔ {b}: nada encontrado según las formas — seleccionar una arista sobre el eje.',
    "Trouvé d'après les formes : {k} — {d}":
        'Encontrado según las formas: {k} — {d}',
    "Rien trouvé d'après les formes entre {a} et {b} : choisir le type, puis sélectionner une arête sur l'axe.":
        'Nada encontrado según las formas entre {a} y {b}: elegir el tipo y luego seleccionar una arista sobre el eje.',
    "Liaison sur l'arête sélectionnée :":
        'Unión sobre la arista seleccionada:',
    'Moteur':
        'Motor',
    ' {n} rotule(s).':
        ' {n} rótula(s).',
    'Calcul':
        'Cálculo',
    'Cinématique':
        'Cinemática',
    'Les moteurs imposent le mouvement, pas à pas de 0 à 100 % : positions, collisions, butées, jeu minimal.':
        'Los motores imponen el movimiento, paso a paso de 0 a 100 %: posiciones, colisiones, topes, holgura mínima.',
    'Simulation dans le temps : masses (matériaux), pesanteur, ressorts, amortisseurs, butées, moteurs en couple ; les chocs entre pièces libres en option.':
        'Simulación en el tiempo: masas (materiales), gravedad, muelles, amortiguadores, topes, motores de par; choques entre piezas libres como opción.',
    'Compte rendu':
        'Informe',
    'Courbes':
        'Curvas',
    'Ce qui est tracé, pour la pièce ou la liaison choisie dans la liste (rien de choisi : tout le mécanisme).':
        'Lo que se traza, para la pieza o la unión elegida en la lista (nada elegido: todo el mecanismo).',
    'CSV…':
        'CSV…',
    'Enregistrer les valeurs de la courbe (tableur).':
        'Guardar los valores de la curva (hoja de cálculo).',
    'Choisir une pièce ou une liaison dans la liste.':
        'Elegir una pieza o una unión en la lista.',
    'Lancer ▶ ou « Valider » : la courbe suit le calcul.':
        'Lanzar ▶ o « Validar »: la curva sigue el cálculo.',
    'Mécanisme':
        'Mecanismo',
    'Enregistrer la courbe':
        'Guardar la curva',
    'Courbe enregistrée : {p}':
        'Curva guardada: {p}',
    'Position de la liaison (°)':
        'Posición de la unión (°)',
    'Position de la liaison (mm)':
        'Posición de la unión (mm)',
    'Vitesse de la liaison (°/s)':
        'Velocidad de la unión (°/s)',
    'Vitesse de la liaison (mm/s)':
        'Velocidad de la unión (mm/s)',
    'Effort dans la liaison (N)':
        'Esfuerzo en la unión (N)',
    'Couple du moteur (N·m)':
        'Par del motor (N·m)',
    'Force du moteur (N)':
        'Fuerza del motor (N)',
    'Hauteur du centre de gravité (mm)':
        'Altura del centro de gravedad (mm)',
    'Vitesse du centre de gravité (m/s)':
        'Velocidad del centro de gravedad (m/s)',
    'Énergie du mécanisme (J)':
        'Energía del mecanismo (J)',
    'temps (s)':
        'tiempo (s)',
    'course du moteur (%)':
        'recorrido del motor (%)',
    'cinétique':
        'cinética',
    'potentielle':
        'potencial',
    'totale':
        'total',
    'rien de calculé':
        'nada calculado',
    'Nom':
        'Nombre',
    'Type':
        'Tipo',
    'Détail':
        'Detalle',
    'Supprimer':
        'Eliminar',
    'Matériau':
        'Material',
    'Matériau de la pièce choisie dans la liste (ou des groupes sélectionnés) : masse, centre de gravité et inertie en découlent.':
        'Material de la pieza elegida en la lista (o de los grupos seleccionados): de él se deducen la masa, el centro de gravedad y la inercia.',
    'Nouveau…':
        'Nuevo…',
    'Ajouter un matériau (ou corriger une valeur) pour ce document.':
        'Añadir un material (o corregir un valor) para este documento.',
    'Liaison choisie':
        'Unión elegida',
    'Pièce A':
        'Pieza A',
    'Pièce B':
        'Pieza B',
    'Butées':
        'Topes',
    'Course moteur':
        'Recorrido del motor',
    'Moteur (plusieurs possibles : même 0–100 %, chacun sa course)':
        'Motor (varios posibles: mismo 0–100 %, cada uno su recorrido)',
    'Coché : cette liaison est un moteur. Vitesse = course ; déphasage = valeur de départ.':
        'Marcado: esta unión es un motor. Velocidad = recorrido; desfase = valor inicial.',
    'Ressort':
        'Muelle',
    'Raideur':
        'Rigidez',
    'Amort.':
        'Amort.',
    'Raideur (N·m/rad pour un pivot, N/m pour une glissière ou une liaison élastique), au repos dans la position dessinée. 0 : aucun.':
        'Rigidez (N·m/rad para un pivote, N/m para una corredera o una unión elástica), en reposo en la posición dibujada. 0: ninguna.',
    'Raideur en translation (N/m) ; en rotation : 1 % de cette valeur en N·m/rad.':
        'Rigidez en traslación (N/m); en rotación: 1 % de este valor en N·m/rad.',
    'Amortissement visqueux (N·m·s/rad ou N·s/m). Dynamique seulement.':
        'Amortiguamiento viscoso (N·m·s/rad o N·s/m). Solo dinámica.',
    'Moteur en couple':
        'Motor de par',
    "Dynamique : un couple (N·m) ou une force (N) constant ; le mécanisme répond (au lieu d'imposer la course).":
        'Dinámica: un par (N·m) o una fuerza (N) constante; el mecanismo responde (en lugar de imponer el recorrido).',
    'Appliquer':
        'Aplicar',
    'Lecture / pause':
        'Reproducir / pausa',
    'Aller-retour':
        'Ida y vuelta',
    'Exporter la vidéo…':
        'Exportar el vídeo…',
    "Vidéo en écran partagé : la vue IngeTrazo à gauche, l'animation ombrée et les mesures à droite.":
        'Vídeo en pantalla dividida: la vista de IngeTrazo a la izquierda, la animación sombreada y las medidas a la derecha.',
    'Jeu mini':
        'Holgura mín.',
    'Pas':
        'Paso',
    'Dynamique':
        'Dinámica',
    'durée':
        'duración',
    'Chocs':
        'Choques',
    'Ralenti':
        'Cámara lenta',
    'Simulation dynamique : la vidéo la montre ce nombre de fois plus lentement (1 : en temps réel).':
        'Simulación dinámica: el vídeo la muestra este número de veces más despacio (1: en tiempo real).',
    'ralenti ×{k}':
        'cámara lenta ×{k}',
    'Chocs et appuis entre pièces libres (dominos, blocs, butées posées) : les boîtes des pièces se repoussent, avec le frottement et le rebond de leurs matériaux. Les pièces tenues par une liaison ne se heurtent pas.':
        'Choques y apoyos entre piezas libres (dominós, bloques, topes sueltos): las cajas de las piezas se repelen, con el rozamiento y el rebote de sus materiales. Las piezas sujetas por una unión no chocan entre sí.',
    'Chocs entre pièces : enfoncement maxi {d} mm (pièces indéformables).':
        'Choques entre piezas: penetración máx. {d} mm (piezas indeformables).',
    'Masses (matériaux), pesanteur, ressorts, amortisseurs, butées, moteurs en couple : Valider, la lecture et la vidéo suivent la simulation dans le temps ; les moteurs en vitesse font leur course en « durée » et le rapport donne leur couple.':
        'Masas (materiales), gravedad, muelles, amortiguadores, topes, motores de par: Validar, la reproducción y el vídeo siguen la simulación en el tiempo; los motores de velocidad hacen su recorrido en la « duración » y el informe da su par.',
    '— non défini —':
        '— no definido —',
    '{name} : {m} kg, volume {v} cm³ — centre de gravité ({x} ; {y} ; {z}) mm':
        '{name}: {m} kg, volumen {v} cm³ — centro de gravedad ({x}; {y}; {z}) mm',
    '{name} : volume {v} cm³ — choisir un matériau':
        '{name}: volumen {v} cm³ — elegir un material',
    ' ; {n} groupe(s) sans matériau':
        '; {n} grupo(s) sin material',
    '{label} : {n} groupe(s).':
        '{label}: {n} grupo(s).',
    'Matériau du document':
        'Material del documento',
    '(nouveau)':
        '(nuevo)',
    'Partir de':
        'Partir de',
    'Masse volumique':
        'Densidad',
    "Module d'Young":
        'Módulo de Young',
    'Frottement (sec)':
        'Rozamiento (seco)',
    'Restitution (chocs)':
        'Restitución (choques)',
    'Annuler':
        'Cancelar',
    'Matériau « {name} » : {rho} kg/m³.':
        'Material « {name} »: {rho} kg/m³.',
    'matériau ?':
        '¿material?',
    ' (+{n} sans matériau)':
        ' (+{n} sin material)',
    '{name} : {n} groupe(s).':
        '{name}: {n} grupo(s).',
    '{n} groupe(s)':
        '{n} grupo(s)',
    ' · moteur':
        ' · motor',
    'Hélicoïdale':
        'Helicoidal',
    'Pas (mm par tour) :':
        'Paso (mm por vuelta):',
    '{name} : {a} ↔ {b}, pas {p} mm.':
        '{name}: {a} ↔ {b}, paso {p} mm.',
    '{n} galet(s) sur le rail de {a}.':
        '{n} rodillo(s) sobre el raíl de {a}.',
    'Moteur : {j} ({n} moteur(s)).':
        'Motor: {j} ({n} motor(es)).',
    '{name} mis à jour.':
        '{name} actualizado.',
    '{name} supprimé.':
        '{name} eliminado.',
    'Choisir une pièce dans la liste, ou sélectionner des groupes.':
        'Elegir una pieza en la lista, o seleccionar grupos.',
    'Sélectionner un ou plusieurs groupes.':
        'Seleccionar uno o varios grupos.',
    "définir d'abord au moins deux pièces":
        'definir primero al menos dos piezas',
    'sélectionner UNE arête (ou ligne de construction)':
        'seleccionar UNA arista (o línea de construcción)',
    'arête de longueur nulle':
        'arista de longitud nula',
    'sélectionner des arêtes au point de liaison (un cercle autour du centre, ou une arête)':
        'seleccionar aristas en el punto de unión (un círculo alrededor del centro, o una arista)',
    "sélectionner UNE arête sur l'axe de l'alésage (et un cercle de la sphère)":
        'seleccionar UNA arista sobre el eje del alojamiento (y un círculo de la esfera)',
    'Dynamique : simulation dans le temps.':
        'Dinámica: simulación en el tiempo.',
    'Dynamique : arrêtée.':
        'Dinámica: detenida.',
    'sélectionner le rail et un trait par galet':
        'seleccionar el raíl y un trazo por rodillo',
    'aucun galet : tracer un petit trait en travers du rail':
        'ningún rodillo: trazar un pequeño trazo a través del raíl',
    "définir d'abord la pièce fixe qui porte le rail":
        'definir primero la pieza fija que lleva el raíl',
    'aucune pièce mobile près du galet':
        'ninguna pieza móvil cerca del rodillo',
    'choisir une liaison dans la liste':
        'elegir una unión en la lista',
    'pas de moteur':
        'sin motor',
    '✔ Mécanisme valide de 0 à 100 %.':
        '✔ Mecanismo válido de 0 a 100 %.',
    '✖ {n} problème(s) :':
        '✖ {n} problema(s):',
    'Jeu le plus faible : {d} mm entre {a} ({ga}) et {b} ({gb}) à {p} %.':
        'Holgura más pequeña: {d} mm entre {a} ({ga}) y {b} ({gb}) al {p} %.',
    'Degrés de liberté (sans moteur) : {n}':
        'Grados de libertad (sin motor): {n}',
    'Simulation dynamique de {t} s : énergie {e0} → {e1} mJ.':
        'Simulación dinámica de {t} s: energía {e0} → {e1} mJ.',
    'Couple moteur {nm} : maxi {mx} N·m (à {t} s), moyen {mean} N·m.':
        'Par del motor {nm}: máx. {mx} N·m (a {t} s), medio {mean} N·m.',
    '{name} : en butée à partir de {t} s.':
        '{name}: en tope a partir de {t} s.',
    'ⓘ sans matériau, masse estimée (volume des pièces, 700 kg/m³) : {names}.':
        'ⓘ sin material, masa estimada (volumen de las piezas, 700 kg/m³): {names}.',
    'Masse totale : {m} kg — ':
        'Masa total: {m} kg — ',
    'ⓘ {n} groupe(s) sans matériau : masse non comptée.':
        'ⓘ {n} grupo(s) sin material: masa no contada.',
    'Trajectoire de {n} point(s).':
        'Trayectoria de {n} punto(s).',
    'STEP : {n} pièce(s) importée(s)':
        'STEP: {n} pieza(s) importada(s)',
    ' — non lu : {w}':
        ' — no leído: {w}',
    'Détection : {p} pivot(s), {s} glissière(s), {g} engrenage(s) ; pièce fixe : {f}.':
        'Detección: {p} pivote(s), {s} corredera(s), {g} engranaje(s); pieza fija: {f}.',
    ' Choisir un moteur dans la liste.':
        ' Elegir un motor en la lista.',
    'Détecter les liaisons':
        'Detectar las uniones',
    '{n} pièce(s) analysée(s) : aucune liaison reconnue (arbre dans un alésage, appuis plans, engrenage).':
        '{n} pieza(s) analizada(s): ninguna unión reconocida (eje en un alojamiento, apoyos planos, engranaje).',
    '{n} pièce(s) analysée(s). Cocher les liaisons à créer :':
        '{n} pieza(s) analizada(s). Marcar las uniones que se crearán:',
    'Liaisons proposées':
        'Uniones propuestas',
    'Liaison':
        'Unión',
    'Confiance':
        'Confianza',
    'Engrenage':
        'Engranaje',
    'Créer les liaisons':
        'Crear las uniones',
    'Importer un assemblage STEP':
        'Importar un ensamblaje STEP',
    'Importer un assemblage STEP…':
        'Importar un ensamblaje STEP…',
    'Importer STEP':
        'Importar STEP',
    'Lecture : {e}':
        'Lectura: {e}',
    '{p} % : position non atteinte.':
        '{p} %: posición no alcanzada.',
    'Exporter la vidéo : pas de moteur.':
        'Exportar el vídeo: sin motor.',
    'Exporter la vidéo':
        'Exportar el vídeo',
    'Durée':
        'Duración',
    'Résolution':
        'Resolución',
    'Aller-retour (0 → 100 → 0 %)':
        'Ida y vuelta (0 → 100 → 0 %)',
    'Indiquer ffmpeg.exe…':
        'Indicar ffmpeg.exe…',
    'AVI Motion-JPEG — pour du MP4, indiquer ffmpeg.exe':
        'AVI Motion-JPEG — para MP4, indicar ffmpeg.exe',
    'Vidéo MP4 (*.mp4)':
        'Vídeo MP4 (*.mp4)',
    'Vidéo AVI (*.avi)':
        'Vídeo AVI (*.avi)',
    'Rendu de la vidéo…':
        'Renderizado del vídeo…',
    "L'encodeur MP4 de Qt ne répond plus. Indiquer ffmpeg.exe dans la fenêtre d'export, ou réessayer.":
        'El codificador MP4 de Qt ya no responde. Indicar ffmpeg.exe en la ventana de exportación, o volver a intentarlo.',
    'Vidéo enregistrée : {out} — {label}.':
        'Vídeo guardado: {out} — {label}.',
    'Export annulé.':
        'Exportación cancelada.',
    'Panneau Motion':
        'Panel Motion',
    'Mécanismes : pivots, glissières, rails, validation.':
        'Mecanismos: pivotes, correderas, raíles, validación.',
    '{v} tr':
        '{v} vuelta(s)',
    'temps {t} s':
        'tiempo {t} s',
    'couple {nm} {v} N·m':
        'par {nm} {v} N·m',
    'jeu mini {d} mm':
        'holgura mín. {d} mm',
    'aucune collision':
        'ninguna colisión',
    'position non atteinte':
        'posición no alcanzada',
    'collision : ':
        'colisión: ',
    'Animation Motion':
        'Animación Motion',
    'Pièce fixe (Fixed part) — les groupes sélectionnés forment le bâti.':
        'Pieza fija (Fixed part) — los grupos seleccionados forman el bastidor.',
    'Pièce mobile (Moving part) — les groupes sélectionnés bougent ensemble.':
        'Pieza móvil (Moving part) — los grupos seleccionados se mueven juntos.',
    "Pivot — sélectionner les DEUX pièces (l'axe est trouvé : arbre dans l'alésage), ou UNE arête ou ligne de construction sur l'axe.":
        'Pivote — seleccionar las DOS piezas (el eje se encuentra: eje en el alojamiento), o UNA arista o línea de construcción sobre el eje.',
    'Glissière (Slider) — sélectionner les DEUX pièces (appuis plans ou tige dans une douille), ou UNE arête dans le sens du coulissement.':
        'Corredera (Slider) — seleccionar las DOS piezas (apoyos planos o vástago en un casquillo), o UNA arista en el sentido del deslizamiento.',
    'Guide / rail — sélectionner les arêtes du rail + un petit trait en travers à chaque galet.':
        'Guía / raíl — seleccionar las aristas del raíl + un pequeño trazo transversal en cada rodillo.',
    "Moteur (Driver) — sélectionner la pièce à entraîner : sa liaison avec le bâti devient le moteur (trouvée d'après les formes s'il le faut). Ou la liaison choisie dans la liste, avec la course indiquée.":
        'Motor (Driver) — seleccionar la pieza que se arrastra: su unión con el bastidor pasa a ser el motor (encontrada según las formas si hace falta). O la unión elegida en la lista, con el recorrido indicado.',
    'sélectionner la pièce à entraîner (ou choisir une liaison dans la liste)':
        'seleccionar la pieza que se arrastra (o elegir una unión en la lista)',
    "pivot trouvé d'après les formes ({d}), ":
        'pivote encontrado según las formas ({d}), ',
    '{p} : aucun pivot trouvé avec le bâti — sélectionner une arête sur son axe, puis « Liaison ».':
        '{p}: no se ha encontrado ningún pivote con el bastidor — seleccionar una arista sobre su eje y luego « Unión ».',
    'Moteur actuel : {o}.\nLe remplacer par {n}, ou garder les deux (même 0–100 %, chacun sa course) ?':
        'Motor actual: {o}.\n¿Sustituirlo por {n}, o conservar los dos (mismo 0–100 %, cada uno su recorrido)?',
    'Remplacer':
        'Sustituir',
    'axe propre de la pièce : {d}':
        'eje propio de la pieza: {d}',
    'Glisser une pièce sur une autre pour les fusionner ; clic droit : fusionner, supprimer.':
        'Arrastrar una pieza sobre otra para fusionarlas; clic derecho: fusionar, eliminar.',
    '{a} fusionnée dans {b} : une seule pièce.':
        '{a} fusionada en {b}: una sola pieza.',
    'Fusionner dans':
        'Fusionar en',
    'bâti : {n} (le plus gros groupe — « Pièces » pour en choisir un autre)':
        'bastidor: {n} (el grupo más grande — « Piezas » para elegir otro)',
    '{n} : pièce mobile':
        '{n}: pieza móvil',
    '⚠ {n} pièce(s) sans matériau : choisir la pièce, puis « Matériau » (masses, dynamique).':
        '⚠ {n} pieza(s) sin material: elegir la pieza y luego « Material » (masas, dinámica).',
    "Axe (Axis) — sélectionner une pièce : son axe (centre de la denture, alésage, forme ronde) tracé en ligne de construction, pour le vérifier ou s'en servir avec « Liaison ».":
        'Eje (Axis) — seleccionar una pieza: su eje (centro del dentado, alojamiento, forma redonda) trazado como línea de construcción, para comprobarlo o usarlo con « Unión ».',
    'sélectionner une pièce':
        'seleccionar una pieza',
    "{p} : pas d'axe trouvé (ni denture, ni alésage, ni forme ronde)":
        '{p}: no se ha encontrado ningún eje (ni dentado, ni alojamiento, ni forma redonda)',
    '{p} : {d}':
        '{p}: {d}',
    'Axe — ':
        'Eje — ',
    'Contact':
        'Contacto',
    'sélectionner la pièce poussée et celle(s) qui la poussent':
        'seleccionar la pieza empujada y la(s) que la empuja(n)',
    'aucune des pièces ne peut être poussée : il lui faut un seul pivot (ou une glissière) et aucun moteur':
        'ninguna de las piezas puede ser empujada: necesita un solo pivote (o una corredera) y ningún motor',
    'Contact : déjà en place.':
        'Contacto: ya existe.',
    'Contact : {p} poussée par {o}.':
        'Contacto: {p} empujada por {o}.',
    'Pièce poussée :':
        'Pieza empujada:',
    'Contact — sélectionner la pièce poussée (tenue au bâti par son seul pivot) et celles qui la poussent : elle reste immobile, puis tourne juste assez pour ne jamais être traversée (croix de Genève, cliquet, came).':
        'Contacto — seleccionar la pieza empujada (sujeta al bastidor solo por su pivote) y las que la empujan: se queda quieta y luego gira justo lo necesario para no ser atravesada nunca (cruz de Malta, trinquete, leva).',
    'contact : {p} doit tenir à une autre pièce par un seul pivot (ou une glissière), sans autre liaison':
        'contacto: {p} debe estar unida a otra pieza por un solo pivote (o una corredera), sin otra unión',
    'Coché : cette liaison est un moteur, de la première valeur à la seconde. Plusieurs moteurs possibles : même 0–100 %, chacun sa course.':
        'Marcado: esta unión es un motor, del primer valor al segundo. Varios motores posibles: mismo 0–100 %, cada uno su recorrido.',
    'Course':
        'Recorrido',
    'Liaison choisie : {n} — {k}':
        'Unión elegida: {n} — {k}',
    '{a} ↔ {b} : pas deux roues dentées qui engrènent — « Plus… » → Couplage pour le régler à la main.':
        '{a} ↔ {b}: no son dos ruedas dentadas que engranan — « Más… » → Acoplamiento para ajustarlo a mano.',
    "Trouvé d'après les formes : engrenage, {za} et {zb} dents (rapport {za}/{zb}). Les pivots manquants avec le bâti sont ajoutés.":
        'Encontrado según las formas: engranaje, {za} y {zb} dientes (relación {za}/{zb}). Se añaden los pivotes que faltan con el bastidor.',
    'Engrenage {name} : {za} → {zb} dents, rapport {r}.':
        'Engranaje {name}: {za} → {zb} dientes, relación {r}.',
    'Garder les deux':
        'Conservar los dos',
    'Valider (Validate) — de 0 à 100 % : collisions, butées, jeu minimal.':
        'Validar (Validate) — de 0 a 100 %: colisiones, topes, holgura mínima.',
    'Trajectoire (Trajectory) — trace les extrémités des arêtes sélectionnées.':
        'Trayectoria (Trajectory) — traza los extremos de las aristas seleccionadas.',
    "Pivot glissant (Cylindrical) — les DEUX pièces (arbre dans l'alésage) ou UNE arête sur l'axe : la pièce tourne et coulisse.":
        'Pivote deslizante (Cylindrical) — las DOS piezas (eje en el alojamiento) o UNA arista sobre el eje: la pieza gira y se desliza.',
    'Rotule (Ball) — sélectionner les DEUX pièces (une sphère dans une sphère creuse), ou des arêtes autour du centre (un petit cercle) : toutes les rotations.':
        'Rótula (Ball) — seleccionar las DOS piezas (una esfera en una esfera hueca), o aristas alrededor del centro (un pequeño círculo): todas las rotaciones.',
    "Encastrement (Lock) — les DEUX pièces, ou des arêtes au point de liaison : les deux pièces n'en font qu'une.":
        'Empotramiento (Lock) — las DOS piezas, o aristas en el punto de unión: las dos piezas se convierten en una.',
    'sélectionner les groupes de DEUX pièces (ou des arêtes au point de liaison)':
        'seleccionar los grupos de DOS piezas (o aristas en el punto de unión)',
    "{name} : {a} ↔ {b} — les deux pièces n'en font plus qu'une.":
        '{name}: {a} ↔ {b} — las dos piezas ya son una sola.',
    'Liaison élastique (Elastic) — des arêtes au point de liaison : un silentbloc, rigide en cinématique, ressorts et amortisseurs en dynamique.':
        'Unión elástica (Elastic) — aristas en el punto de unión: un silentblock, rígido en cinemática, muelles y amortiguadores en dinámica.',
    "Linéaire annulaire (Annular) — UNE arête sur l'axe de l'alésage, plus un cercle de la sphère : la sphère glisse dans l'alésage et tourne en tous sens.":
        'Lineal anular (Annular) — UNA arista sobre el eje del alojamiento, más un círculo de la esfera: la esfera se desliza en el alojamiento y gira en todos los sentidos.',
    'Ponctuelle (Point–plane) — UNE arête le long de la normale, partant du point de contact : le point reste sur le plan.':
        'Puntual (Point–plane) — UNA arista a lo largo de la normal, partiendo del punto de contacto: el punto se queda sobre el plano.',
    'Appui plan (Planar) — UNE arête le long de la normale au plan : la pièce glisse sur le plan et tourne autour de la normale.':
        'Apoyo plano (Planar) — UNA arista a lo largo de la normal al plano: la pieza se desliza sobre el plano y gira alrededor de la normal.',
    "Hélicoïdale (Screw) — UNE arête sur l'axe, puis le pas : la pièce avance d'un pas par tour (pas négatif : filet à gauche).":
        'Helicoidal (Screw) — UNA arista sobre el eje y luego el paso: la pieza avanza un paso por vuelta (paso negativo: rosca a izquierdas).',
    'Pièce fixe':
        'Pieza fija',
    'Pièce mobile':
        'Pieza móvil',
    'Pivot':
        'Pivote',
    'Glissière':
        'Corredera',
    'Guide / rail':
        'Guía / raíl',
    'Pivot glissant':
        'Pivote deslizante',
    'Rotule':
        'Rótula',
    'Encastrement':
        'Empotramiento',
    'Liaison élastique':
        'Unión elástica',
    'Linéaire annulaire':
        'Lineal anular',
    'Ponctuelle':
        'Puntual',
    'Appui plan':
        'Apoyo plano',
    'Couplage':
        'Acoplamiento',
    'fixe':
        'fija',
    'pièce':
        'pieza',
    'pivot':
        'pivote',
    'glissière':
        'corredera',
    'galet':
        'rodillo',
    'pivot glissant':
        'pivote deslizante',
    'rotule':
        'rótula',
    'encastrement':
        'empotramiento',
    'élastique':
        'elástica',
    'linéaire annulaire':
        'lineal anular',
    'ponctuelle':
        'puntual',
    'appui plan':
        'apoyo plano',
    'hélicoïdale':
        'helicoidal',
    'engrenage':
        'engranaje',
    '{p} % : position non atteinte (résidu {r})':
        '{p} %: posición no alcanzada (residuo {r})',
    '{p} % : collision {a} / {b}':
        '{p} %: colisión {a} / {b}',
    '{p} % : butée dépassée {name} = {v}':
        '{p} %: tope superado {name} = {v}',
    '{p} % : jeu {d} mm < {j} mm entre {a} ({ga}) et {b} ({gb})':
        '{p} %: holgura {d} mm < {j} mm entre {a} ({ga}) y {b} ({gb})',
    'sélectionner les arêtes du rail':
        'seleccionar las aristas del raíl',
    'aucune pièce fixe : ajouter une « pièce fixe »':
        'ninguna pieza fija: añadir una « pieza fija »',
    "{name} : reliée à aucune pièce fixe, sa position n'est pas définie (ajouter une liaison ou en faire une pièce fixe)":
        '{name}: no está unida a ninguna pieza fija, su posición no está definida (añadir una unión o convertirla en pieza fija)',
    'aucun moteur : choisir une liaison et « Moteur 0–100 % »':
        'ningún motor: elegir una unión y « Motor 0–100 % »',
    '{name} : aucun groupe trouvé dans le modèle':
        '{name}: no se ha encontrado ningún grupo en el modelo',
    '{name} : pièce inconnue':
        '{name}: pieza desconocida',
    '{name} : relie une pièce à elle-même':
        '{name}: une una pieza consigo misma',
    '{name} : liaison couplée inconnue':
        '{name}: unión acoplada desconocida',
    'moteur {name} : liaison inconnue':
        'motor {name}: unión desconocida',
    'Modèle (document)':
        'Modelo (documento)',
    'Exemple : chaise pliante':
        'Ejemplo: silla plegable',
    'Exemple : porte sectionnelle':
        'Ejemplo: puerta seccional',
    "Exemple : oiseau à battement d'ailes":
        'Ejemplo: pájaro que bate las alas',
    'Exemple : draisine à balancier':
        'Ejemplo: dresina de balancín',
    'Exemple : chaîne détendue et tendeur':
        'Ejemplo: cadena floja y tensor',
    'Exemple : chaîne souple (dynamique)':
        'Ejemplo: cadena flexible (dinámica)',
    'Exemple : pendule double (dynamique)':
        'Ejemplo: péndulo doble (dinámica)',
    'Chaise pliante en X':
        'Silla plegable en X',
    'Porte sectionnelle':
        'Puerta seccional',
    "Oiseau à battement d'ailes":
        'Pájaro que bate las alas',
    'Draisine à balancier':
        'Dresina de balancín',
    'Chaîne détendue et tendeur':
        'Cadena floja y tensor',
    'Chaîne souple (dynamique)':
        'Cadena flexible (dinámica)',
    'Pendule double (dynamique)':
        'Péndulo doble (dinámica)',
    "flèche {s} mm ({p} % de l'entraxe)":
        'flecha {s} mm ({p} % de la distancia entre ejes)',
    'entraxe {c} mm':
        'distancia entre ejes {c} mm',
    'chaîne tendue (pas de mou)':
        'cadena tensa (sin holgura)',
    'course dans la lumière {d} mm':
        'recorrido en la ranura {d} mm',
    'bras {v}°':
        'brazo {v}°',
    'main {v}°':
        'mano {v}°',
    'pliage {v}°':
        'plegado {v}°',
    'balancier {v}°':
        'balancín {v}°',
    'avance {v} m':
        'avance {v} m',
    'roues {v} tr':
        'ruedas {v} vuelta(s)',
    'pignon mené {v} tr':
        'piñón arrastrado {v} vuelta(s)',
    'dynamique, amortissement {v}/s':
        'dinámica, amortiguamiento {v}/s',
    'bras supérieur {v}°':
        'brazo superior {v}°',
    'bras inférieur {v}° (relatif)':
        'brazo inferior {v}° (relativo)',
    'énergie {e} mJ (cinétique {k})':
        'energía {e} mJ (cinética {k})',
    'Chêne':
        'Roble',
    'Hêtre':
        'Haya',
    'Frêne':
        'Fresno',
    'Noyer':
        'Nogal',
    'Pin sylvestre':
        'Pino silvestre',
    'Épicéa':
        'Pícea',
    'Douglas':
        'Abeto Douglas',
    'Contreplaqué bouleau':
        'Contrachapado de abedul',
    'MDF':
        'MDF',
    'OSB':
        'OSB',
    'Acier':
        'Acero',
    'Inox':
        'Acero inoxidable',
    'Aluminium':
        'Aluminio',
    'Laiton':
        'Latón',
    'Bronze':
        'Bronce',
    'Fonte grise':
        'Fundición gris',
    'POM (acétal)':
        'POM (acetal)',
    'PA 6 (nylon)':
        'PA 6 (nailon)',
    'PEHD':
        'PEAD',
    'PLA (impression 3D)':
        'PLA (impresión 3D)',
    'ABS':
        'ABS',
    'PMMA (acrylique)':
        'PMMA (acrílico)',
    'Verre':
        'Vidrio',
    " {n} encastrement(s) (bras serrés).":
        " {n} empotramiento(s) (brazos apretados).",
    'Pesanteur':
        'Gravedad',
    "Pesanteur (9,81 m/s², vers le bas de l'axe Z) : cochée par défaut, toute pièce libre tombe ou bascule sur son axe. Décochée : mécanisme en apesanteur, seuls les moteurs, ressorts et contacts agissent.":
        'Gravedad (9,81 m/s², hacia abajo según el eje Z): marcada por defecto, toda pieza libre cae o bascula sobre su eje. Desmarcada: mecanismo en ingravidez, solo actúan los motores, los muelles y los contactos.',
    "Pièces libres, non menées par les moteurs : calcul en Dynamique (contacts et pesanteur) pour qu'elles bougent.":
        'Piezas libres, no movidas por los motores: cálculo en Dinámica (contactos y gravedad) para que se muevan.',
    'Mode':
        'Modo',
    'Physique':
        'Física',
    'Jeu minimal':
        'Holgura mínima',
    'Toutes les liaisons et outils':
        'Todas las uniones y herramientas',
    'Lecture et vidéo':
        'Reproducción y vídeo',
    'Compte rendu et courbes':
        'Informe y curvas',
    '{p} : bascule de {lo}° à {hi}° (libre, menée par les contacts).':
        '{p}: bascula de {lo}° a {hi}° (libre, movida por los contactos).',
    '{p} : course de {lo} mm à {hi} mm (libre, menée par les contacts).':
        '{p}: recorrido de {lo} mm a {hi} mm (libre, movida por los contactos).',
    # -- beads (0.10) --
    "Billes (Beads) — une source de petites billes : sélectionner la pièce au-dessus de laquelle elles tombent (ou rien : au-dessus du modèle), puis leur diamètre, leur nombre, leur débit. Elles roulent, s'empilent, s'écoulent, en Dynamique.":
        'Bolas — una fuente de bolitas: seleccionar la pieza sobre la que caen (o nada: encima del modelo), luego su diámetro, número y caudal. Ruedan, se apilan y fluyen, en Dinámica.',
    'Convoyeur (Conveyor) — sélectionner une pièce : sa surface entraîne les billes dans une direction, à une vitesse (un tapis, une bande).':
        'Transportador — seleccionar una pieza: su superficie arrastra las bolas en una dirección, a una velocidad (una cinta).',
    'Plus grand que le diamètre : des billes de toutes tailles entre les deux (des cailloux à trier).':
        'Mayor que el diámetro: bolas de todos los tamaños entre ambos (piedras para clasificar).',
    ' /s':
        ' /s',
    'Largeur de la bouche : les billes naissent sur un disque de ce diamètre.':
        'Ancho de la boca: las bolas nacen en un disco de este diámetro.',
    'Diamètre':
        'Diámetro',
    'Diamètre maxi':
        'Diámetro máx.',
    'Nombre':
        'Número',
    'Débit':
        'Caudal',
    'Matière':
        'Material',
    'Largeur de la bouche':
        'Ancho de la boca',
    'Début':
        'Inicio',
    'Couleur':
        'Color',
    '{name} : {n} billes de {d} mm, en Dynamique.':
        '{name}: {n} bolas de {d} mm, en Dinámica.',
    'Pas un convoyeur':
        'No es transportador',
    'Direction':
        'Dirección',
    'Vitesse':
        'Velocidad',
    '{p} : convoyeur {w} à {v} m/s':
        '{p}: transportador {w} a {v} m/s',
    'Billes…':
        'Bolas…',
    '{dia} · {n} billes · {r}/s':
        '{dia} · {n} bolas · {r}/s',
    '{name} modifiée.':
        '{name} modificada.',
    '{p} : plus un convoyeur':
        '{p}: ya no es transportador',
    'Billes interrompues.':
        'Bolas interrumpidas.',
    "Des billes : calcul en Dynamique (elles tombent, roulent, s'écoulent).":
        'Bolas: cálculo en Dinámica (caen, ruedan y fluyen).',
    'Billes : {b} versées, {left} encore en jeu, {lost} tombées hors du modèle.':
        'Bolas: {b} vertidas, {left} aún en juego, {lost} caídas fuera del modelo.',
    ' · convoyeur {v} m/s':
        ' · transportador {v} m/s',
    'Double-clic : ses réglages.':
        'Doble clic: sus ajustes.',
    'Plastique':
        'Plástico',
    'Roche':
        'Roca',
    'Bois':
        'Madera',
    'Sable':
        'Arena',
    'Or':
        'Oro',
    'Bleu':
        'Azul',
    'Rouge':
        'Rojo',
    'Vert':
        'Verde',
    'Gris':
        'Gris',
    'Blanc':
        'Blanco',
    "Bac (Bin) — sélectionner une pièce : elle compte les billes qui y arrivent (un bac, une trémie de tri) ; elles y restent. De nouveau : ce n'est plus un bac.":
        'Cubeta — seleccionar una pieza: cuenta las bolas que llegan a ella (una cubeta, una tolva de clasificación); se quedan en ella. Otra vez: deja de ser cubeta.',
    ' · bac':
        ' · cubeta',
    '{p} : bac':
        '{p}: cubeta',
    '{p} : plus un bac':
        '{p}: ya no es cubeta',
    '{n} arrivées dans {p}':
        '{n} llegadas a {p}',
}
