# Exploration et pilotage

## Utilisation

- **Settings → Maze size** : 6×6, 9×9 ou 16×16. Le choix concerne la prochaine
  exploration ; charger une carte reprend sa taille et son point de départ.
- Le robot commence au centre d'une cellule du bord, aligné sur les murs. Aucun
  coin ni cap à saisir. La direction initiale correspond au haut de l'écran.
- Le départ avec la main devant F10 puis son retrait est conservé.
- Exploration : carte plein écran, zoom 16 pixels/cellule, cadrage suivant le robot.
  Pointillés dans la carte : murs encore inconnus. Sur le pourtour, les bandes
  pleines indiquent une détection ; sans détection, un pointillé fin de 1 pixel.
  F10 est en haut à gauche, F5 en haut à droite. Sur les côtés, 10 cm en haut
  (bande longue), 5 cm plus bas (bande courte).
- À la fin, même en cas d'erreur : vue générale initiale, Escape pour changer le
  zoom (vue générale / 16 / 24 pixels), flèches pour déplacer la carte,
  centre du joystick pour alterner carte / temps / résultat, **bouton Zoom/Escape
  maintenu 0,8 seconde pour quitter**. Le centre ne quitte jamais cette vue. Pendant un mouvement, Escape reste un arrêt immédiat.
- Après un apprentissage certifié, sauvegarde automatique puis accès aux runs.
  La bibliothèque conserve jusqu'à huit cartes avec leur chemin optimal clignotant.

## Runs et vitesse

**Settings → Fast run** permet désormais 20 à 1 000 mm/s. Le réglage enregistré
reste inchangé lors de la mise à jour. Au-dessus de 300 mm/s, les pas du menu
sont de 50 mm/s. L'accélération et le freinage restent limités à 800 mm/s² :
la vitesse choisie est un plafond, atteint seulement sur une ligne assez longue.
À 1 000 mm/s, même la roue accélérée par la correction reste sous le plafond
matériel de 100 000 impulsions/s.

**Curves** exécute maintenant des arcs de 90° sur la carte apprise : approche
d'une demi-cellule, arc de rayon égal à une demi-cellule, sortie d'une demi-cellule.
Les trois phases s'enchaînent dans la commande moteur sans arrêt au virage.
La vitesse de l'ensemble est limitée à 220 mm/s ; les lignes droites utilisent
le réglage Fast run. Les deux roues avancent, avec des vitesses proportionnelles
aux rayons intérieur/extérieur et une synchronisation de leur progression.
La calibration de rotation fournit la différence de trajet des roues, à la
vitesse de rotation équivalente. La géométrie mesurée vérifie le dégagement.

L'arc nécessite des cellules visitées, un angle entièrement connu et des
calibrations mur/rotation valides. Les demi-tours, angles sans assez de place
pour l'approche/sortie ou sans profils valides utilisent un pivot. Des virages
rapprochés peuvent donc encore inclure des pivots et des arrêts intermédiaires.
Les capteurs périmés, Escape et les obstacles frontaux arrêtent aussi un arc.
Ce premier mode courbe ne relance pas automatiquement un run interrompu.

Les tests intègrent les déplacements des deux roues pour vérifier les arcs
gauche/droite, le centre d'arrivée, le cap et les murs. La tenue mécanique à
haute vitesse et en courbe reste à valider sur le robot.

## Formats et départ automatique

Le [règlement Micromouse Classic UKMARS](https://ukmars.org/contests/contest-rules/micromouse-classic/)
définit le format classique 16×16 et une arrivée centrale 2×2. Le mode 16×16
utilise cette arrivée et une limite d'exploration de dix minutes. Les modes 6×6
et 9×9 conservent la recherche d'une unique salle 2×2 ouverte, avec cinq minutes.
La géométrie réellement calibrée (179 mm de pas / 167 mm libres ici) reste
prioritaire ; sélectionner 16×16 ne remplace pas ces dimensions par 180/168 mm.

Une mesure de trois murs ne distingue pas toujours un coin extérieur d'une
impasse intérieure. La carte commence donc dans un repère relatif au départ et
se décale lorsque de nouvelles ouvertures élargissent l'étendue connue. Les
bornes sont fixées lorsqu'une seule position est compatible avec cette étendue,
la taille choisie et un départ sur le bord. Aucune ouverture observée n'est
transformée arbitrairement en mur extérieur. Dans un petit montage fermé 6×6 ou
9×9 dont la position absolue reste ambiguë, l'exploration complète de la composante
accessible permet néanmoins de certifier son chemin. Pour l'arrivée centrale
16×16, le placement doit être déterminé ; sinon le résultat précise
`START AMBIGUOUS`.

Le planificateur utilise un Dijkstra sur (cellule, cap), avec tas borné. La
longueur du parcours est prioritaire ; à longueur égale, moins de quarts de tour.
Une comparaison avec la carte optimiste (passages inconnus ouverts) détermine
quelles observations peuvent encore améliorer le chemin. La carte est mise à
jour transactionnellement : une contradiction n'écrase pas les murs acquis.

## Boucles de pilotage et mesure anticipée

- Impulsions moteurs par timers, supervision à 1 kHz, télémètres filtrés et
  estimation de pose à chaque nouveau scan (~100 Hz), planification et OLED
  dans la boucle principale. L'affichage et Dijkstra ne suspendent pas la commande.
- La distance d'un déplacement droit est la moyenne des deux roues. Les budgets
  de pas supplémentaires permettent à la correction de direction de produire
  une vraie différence de trajet entre roues.
- L'observateur intègre la différence de pas avec la voie calibrée et contraint
  la position latérale par les intervalles des capteurs 5 cm. Un seuil à moins
  de 5 mm du centre peut servir de référence de suivi : petite consigne de cap
  ±20 mrad, retour amorti en cap, variation limitée de la correction. Le choix
  se fait parmi les murs 10 cm présents et les seuils les plus proches du centre.
  Sans seuil proche, les mesures restent des contraintes de position.
- Sur un long couloir, deux déclenchements dans le même sens du même capteur
  latéral fournissent un cycle complet de suivi. Le firmware estime progressivement
  le biais de rotation permanent des roues à partir de ces cycles et le corrige
  dans l'observateur. L'apprentissage est borné, exclut les transitions de porte,
  les changements de capteur et les demi-cycles ; il est réinitialisé avec la
  commande suivante. Deux positions optiques successives recalculent aussi le
  cap à partir du trajet et de la différence de pas. Les bornes latérales sont
  projetées pendant le retard du filtrage et le gain de cap est réduit à haute
  vitesse. Ce mécanisme vise les petites dissymétries persistantes,
  pas une perte d'adhérence importante. Tests : couloirs simulés de 20 m,
  200/1 000 mm/s, retard de trois scans, dissymétrie ±1 %, décalage initial et
  perturbation de cap.
- Un état binaire constant ne fournit pas une distance. Le suivi recherche les
  changements d'état autour du seuil réel, au lieu de considérer toute une
  demi-cellule comme parfaitement centrée. La position suivie peut différer du
  centre de quelques millimètres selon le montage des capteurs.
- Les transitions des poteaux 10 cm recalent la distance longitudinale avec
  les profils d'ouverture/fermeture interpolés à la vitesse courante. Une paire
  gauche/droite fournit aussi une référence de cap. Corrections bornées : résidu
  accepté ±15 mm, correction au plus 6 mm par événement. Le front 5 cm recale
  également une arrivée devant un mur autorisé. F10 recale déjà l'approche
  avant F5. Si le canal 5 cm d'une calibration d'angle est valide, il affine le
  recalage après un poteau 10 cm concordant ; une simple transition de suivi
  latéral n'est pas prise pour une porte. Aucune valeur 5 cm absente du
  rapport d'angle n'est employée comme un offset nul.
- Le quart de tour est ajusté au cap résiduel estimé en fin de ligne droite ;
  les procédures de calibration gardent leurs rotations de référence intactes.
- L'observation de la cellule suivante commence uniquement dans une fenêtre
  calculée à partir du seuil frontal, des offsets latéraux, du filtrage, de la
  vitesse et d'une marge frontale de 20 mm, augmentée de la dispersion mesurée.
  Trois scans distincts stables **dans cette
  fenêtre** sont nécessaires. Si la décision est tout droit, le budget moteur
  est prolongé avant le centre, sans arrêter les timers ni réinitialiser le
  contrôleur. Les portions déjà connues sont regroupées en une commande.
- Sans calibrations suffisantes pour calculer cette fenêtre, le firmware observe
  à l'arrêt au centre. Il n'extrapole pas les offsets au-delà des vitesses mesurées.

Les pas commandés ne sont pas des encodeurs. Les capteurs permettent de corriger
les erreurs observables aux murs et poteaux ; un dérapage arbitraire dans une
zone sans référence n'est pas mesurable. Les essais logiciels vérifient les
signes, limites, transitions, arrêts et corrections dans un modèle. La stabilité
mécanique et l'adhérence doivent être vérifiées sur le robot avant une course rapide.

## Recentrage et confirmation des portes

Après le départ à la main, le firmware recherche une référence de position sur
les murs connus et visibles. Il fait de même aux arrêts d'exploration, après une
récupération et avant de confirmer un mur suspect. Le recentrage initial d'un
run est exclu de son chronomètre. Un enchaînement droit anticipé reste continu.

Le robot se tourne d'abord vers un mur latéral pour recaler l'axe transversal,
puis utilise le mur avant ou arrière pour l'axe longitudinal. Les quarts de tour
utilisent les profils de rotation. À 40 mm/s, une recherche bornée à 25 mm maximum
repère le déclenchement F5 ou son relâchement, confirmé sur trois scans. Le seuil
**on/off mesuré**, la largeur libre de la cellule et la position de l'axe des
roues donnent alors le déplacement vers le centre. La correction de pose est
limitée à ±15 mm. Aucun talonnage n'est demandé. Sans transition détectée, le
petit déplacement de recherche est annulé avant d'essayer une autre référence.

Il faut des références sur deux axes pour recaler les deux coordonnées. En
leur absence, les pas moteurs et les contraintes des capteurs restent utilisés ;
un centrage parfait n'est pas observable avec une lecture tout ou rien fixe.
Les tests du recentrage couvrent des décalages initiaux de ±12 mm et l'hystérésis,
avec moins de 0,7 mm d'erreur dans le modèle, pas une garantie mécanique.

En exploration, une contradiction persistante ou l'absence de chemin lance une
vérification. Le robot rejoint, par les passages connus, une cellule accessible
avec un mur restant à vérifier. Il se recentre, se place face à ce mur et exige
cinq scans concordants au repos. Une ouverture confirmée corrige **les deux faces**
de la carte puis relance immédiatement le calcul et l'exploration. Une lecture
latérale isolée ne suffit pas à effacer un mur.

Au maximum **trois passes** des murs internes accessibles sont effectuées ; un
même mur n'est vérifié qu'une fois par passe. Les limites extérieures restent
fermées. Si les trois passes ne permettent pas de retrouver un passage, l'écran
indique `CHECK LIMIT`. Annulation, capteurs périmés et limite de temps restent
actifs. La garantie qu'un labyrinthe a une solution déclenche cette recherche,
mais ne remplace jamais une mesure par une ouverture inventée. Les runs gardent
leur arrêt strict sur obstacle ou contradiction.

## Mur frontal détecté tardivement

L'absence de mur exige que F10 **et** F5 soient libres, en brut et en filtré.
La fenêtre anticipée est plus tardive pour éviter de décider juste au seuil F10.
La prolongation tout droit reste possible sans arrêt au centre.

En **exploration uniquement**, un arrêt F5 peut être récupéré si :

- la calibration des murs est valide et la position d'arrêt correspond à un
  seuil F5 de mur de cellule, avec une tolérance de ±15 mm ;
- trois scans distincts confirment F5 en brut et en filtré, sous 200 ms ;
- la cellule de l'autre côté du mur n'a pas déjà été visitée.

L'ancienne limite globale de quatre récupérations est supprimée : des murs
différents peuvent être corrigés pendant toute l'exploration. Les contradictions
répétées restent bornées par les trois passes de confirmation.

Le robot recule à 80 mm/s vers le dernier centre dépassé, sur le trajet qu'il
vient de parcourir (au maximum une cellule). À la fin du recul, les deux faces
du mur sont corrigées dans la carte, puis le robot se recentre et le trajet est recalculé. Si une contradiction
empêche la reprise, les passes de confirmation prennent le relais. La position
est mémorisée dès l'arrêt des moteurs ; l'attente de confirmation n'ajoute pas
une distance fictive. Escape et le contrôle de fraîcheur des capteurs restent
actifs. Si le mur apparaît alors que le robot est déjà arrêté au centre, seule
la carte est corrigée après confirmation et vérification des côtés.

Une position incohérente, une détection non confirmée, un passage déjà visité
ou une calibration absente maintiennent l'arrêt. Pendant un **run**, il n'y a
ni recul automatique ni correction de carte : l'obstacle arrête le parcours.
Ces comportements sont testés en simulation ; leur précision dépend encore
du glissement et des mesures physiques du robot.

## Sauvegardes

Format 7 : carte à pas mémoire de 16 cellules, taille active 6/9/16, origine et
axes connus ; 12 492 octets par snapshot, dans les mêmes deux secteurs de 16 Kio.
Les formats 1 à 6 sont migrés en mémoire, y compris les huit cartes 9×9 et les
calibrations. Aucun effacement n'est déclenché par la seule lecture. La prochaine
sauvegarde transactionnelle écrit le format 7 dans l'autre banque. Les anciens
firmwares ne savent pas charger ce format ; conserver l'archive avant mise à jour.
