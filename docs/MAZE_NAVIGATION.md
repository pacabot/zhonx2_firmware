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
  zoom (vue générale / 16 / 24 / 32 pixels), flèches pour déplacer la carte,
  centre du joystick pour alterner carte / temps / résultat, centre maintenu
  0,8 seconde pour quitter. Pendant un mouvement, Escape reste un arrêt immédiat.
- Après un apprentissage certifié, sauvegarde automatique puis accès aux runs.
  La bibliothèque conserve jusqu'à huit cartes avec leur chemin optimal clignotant.

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
- Un état binaire constant ne fournit pas une distance. Le suivi recherche les
  changements d'état autour du seuil réel, au lieu de considérer toute une
  demi-cellule comme parfaitement centrée. La position suivie peut différer du
  centre de quelques millimètres selon le montage des capteurs.
- Les transitions des poteaux 10 cm recalent la distance longitudinale avec
  les profils d'ouverture/fermeture interpolés à la vitesse courante. Une paire
  gauche/droite fournit aussi une référence de cap. Corrections bornées : résidu
  accepté ±15 mm, correction au plus 6 mm par événement. Le front 5 cm recale
  également une arrivée devant un mur autorisé. Aucune valeur 5 cm absente du
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

## Mur frontal détecté tardivement

L'absence de mur exige que F10 **et** F5 soient libres, en brut et en filtré.
La fenêtre anticipée est plus tardive pour éviter de décider juste au seuil F10.
La prolongation tout droit reste possible sans arrêt au centre.

En **exploration uniquement**, un arrêt F5 peut être récupéré si :

- la calibration des murs est valide et la position d'arrêt correspond à un
  seuil F5 de mur de cellule, avec une tolérance de ±15 mm ;
- trois scans distincts confirment F5 en brut et en filtré, sous 200 ms ;
- la cellule de l'autre côté du mur n'a pas déjà été visitée ;
- moins de quatre corrections ont été effectuées pendant cette exploration.

Le robot recule à 80 mm/s vers le dernier centre dépassé, sur le trajet qu'il
vient de parcourir (au maximum une cellule). À la fin du recul, les deux faces
du mur sont corrigées dans la carte, puis le trajet est recalculé. La position
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
