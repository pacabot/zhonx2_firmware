# Firmware Nîmes, persistance et mise à jour distante

## État de cette branche

Code compilé pour STM32F405RG, tests logiciels exécutables sur ordinateur. La
version `446199d` a été programmée par SWD le 26 septembre 2026, relue intégralement
et démarrée jusqu’au menu. Les corrections décrites ci-dessous restent à installer.
Le transport SPI, les seuils IR,
la géométrie et les profils moteurs restent à valider sur la carte. Le module radio
sera choisi ensuite ; son logiciel de pont n'est pas inclus. L'outil d'envoi TCP
attend le protocole de pont décrit ci-dessous : il ne fonctionne pas directement
avec un module Bluetooth série du commerce.

La base avec bibliothèques mises à jour, dump et compilation reproductible a été
commitée et poussée avant ce travail : `56b950c`, branche
`fw/baseline-libraries-backup-2026-09-26`. La branche d'évolution incorpore aussi les
deux commits du `master` distant qui manquaient au checkout initial.

## Règles utilisées

Source : `Labyrinthe-V1.0.pdf`, pages 2 à 5, règlement Nîmes V1.0.

- 9 × 9 cases ; pas géométrique 180 mm (passage 168 mm et mur 12 mm).
- Départ dans un coin, case avec trois murs, orientation libre.
- Arrivée : salle de 2 × 2 cases ; sa position n'est pas supposée centrale.
- Exploration limitée à cinq minutes ; trois parcours chronométrés ensuite.
- Score : moyenne des trois parcours, avec +2 s par contact, +5 s par intervention
  et durée d'exploration / 30. Ces pénalités rendent les oscillations et les
  explorations inutiles coûteuses.
- Le chapitre 4 exige l'arrêt automatique dans l'arrivée ; les communications
  externes sont interdites pendant l'épreuve.

Le logiciel détecte la salle par ses quatre passages internes confirmés, vérifie
les trois murs du départ, impose 5 minutes d'exploration et s'arrête dans l'arrivée.
Le choix du coin et de l'orientation est explicite au menu. Plusieurs salles 2 × 2
confirmées produisent une erreur, plutôt qu'un choix arbitraire. La chronométrie
et les pénalités officielles des trois parcours restent celles de l'arbitrage ;
le firmware ne prétend pas détecter automatiquement les contacts ou interventions.

## Construction et fichiers

```sh
make clean
make -j4
make test
```

GNU Arm Embedded GCC 13.2.1 utilisé ici ; `cc` et Python 3 sont nécessaires aux tests.
Le code STM32 récent est également contrôlé avec `-Wall -Wextra -Werror`.

| Fichier dans `build/` | Usage |
|---|---|
| `ZHONX_II_M4.elf`, `.map`, `.hex`, `.bin` | Application liée à **0x08010000** |
| `application.ota.bin` | Application alignée sur 4 octets, à envoyer au BL |
| `bootloader.elf`, `.map`, `.bin` | BL lié à 0x08000000 |
| `initial-install.hex` | Installation SWD initiale : BL + manifeste + application |
| `firmware-manifest.json` | Adresses, tailles, SHA-256 et CRC de livraison |

`make` et les outils Python ne programment jamais la carte. `initial-install.hex`
est une image Intel HEX discontinue : les secteurs de sauvegarde sont omis.
L'application seule ne remplace pas l'ancien firmware lié à 0x08000000. La première
migration nécessite une programmation SWD du BL, du manifeste et de l'application,
avec effacement limité aux secteurs concernés. Un effacement global supprimerait
les données persistantes. Ne pas utiliser l'ancien projet Xcode pour produire une
image destinée au nouveau BL : le Makefile et ses scripts de liaison font référence.

Le dump historique complet de 1 Mio est conservé dans `backups/`, avec son SHA-256.
Un retour à cet ancien firmware est une restauration complète par SWD, et remplace
également le nouveau découpage Flash. Il ne passe pas par le protocole OTA.

## Découpage Flash (1 Mio)

| Secteurs | Adresse | Taille | Rôle |
|---|---|---:|---|
| 0 | 0x08000000 | 16 Kio | Bootloader, jamais effacé par le code de mise à jour |
| 1 | 0x08004000 | 16 Kio | Sauvegarde A |
| 2 | 0x08008000 | 16 Kio | Sauvegarde B |
| 3 | 0x0800C000 | 16 Kio | Manifeste d'installation |
| 4–7 | 0x08010000 | 448 Kio | Application, taille autorisée **384 Kio** |
| 8–10 | 0x08080000 | 384 Kio | Image temporaire reçue |
| 11 | 0x080E0000 | 128 Kio | Réservé |

Le dernier secteur de l'application dépasse sa taille logique maximale à cause de
la granularité d'effacement. Le linker et le BL imposent tous deux 384 Kio.

### Sauvegarde des réglages et du labyrinthe

`fw_store` alterne A/B ; un en-tête contient schéma, numéro de génération, taille,
CRC de l'en-tête et du contenu. Le marqueur de validation est programmé en dernier.
Le démarrage sélectionne la copie valide la plus récente ; une écriture interrompue
laisse l'autre copie disponible. Un contenu inchangé n'entraîne aucun effacement.

Le snapshot contient les réglages historiques, la carte 9 × 9, le coin et
l'orientation de départ, les vitesses de recherche/course et le temps d'exploration.
Il est chargé automatiquement au démarrage ; sauvegarde manuelle et restauration
sont disponibles au menu. La sortie d'exploration/course sauvegarde aussi la carte,
y compris partielle. Les écritures sont faites **moteurs arrêtés** : cette Flash
à une banque bloque les accès aux instructions pendant les effacements.

Les anciennes écritures brutes `hal_nvm_write/init_sector` renvoient une erreur :
elles pouvaient modifier une zone de code. Les menus de sauvegarde utilisent la
nouvelle API transactionnelle. Le schéma doit changer si l'ABI des réglages change.
La pose physique n'est jamais restaurée : replacer le robot au départ sélectionné
avant une reprise ou un parcours. Le temps continue entre tentatives dans une même
session alimentée ; après une coupure, seul le temps déjà sauvegardé est connu.
L'arbitrage reste maître du temps total, y compris coupures/interventions.

## Connecteur JTAG : SPI matériel à trois fils

Les fonctions alternatives du STM32F405 autorisent **SPI3 esclave, mode 0,
half duplex sur MISO**, avec DMA. Le maître utilise une seule ligne de données
bidirectionnelle. Les broches UART TX/RX ne sont pas disponibles sur ce connecteur.

| P2 | Signal | Fonction |
|---:|---|---|
| 13 | PB3 / JTDO | SPI3 SCK, AF6 |
| 3 | PB4 / JTRST | SPI3 MISO bidirectionnel, AF6 ; BUSY/READY hors transfert |
| 5 | PA15 / JTDI | SPI3 NSS, AF6 |
| 15 | NSRST | Reset, sortie du pont **à drain ouvert** |
| 7 | PA13 | SWDIO conservé |
| 9 | PA14 | SWCLK conservé |
| 1/2 | +3,3 V | Référence/alimentation suivant budget de courant à vérifier |
| pairs 4–20 | GND | Masse commune |

**Le pont GS2/GS3 doit laisser PB4 indépendant de NSRST.** Cette vérification
matérielle est encore en attente. Le module radio doit pouvoir libérer DATA et
piloter un SPI à trois fils. Un pont Wi-Fi → SPI est une possibilité pour le débit ;
BLE → SPI reste possible avec le même protocole, mais réclame un transport hôte
adapté. Le module définit le débit radio réel et l'authentification.

Commencer les essais à 1 MHz ; viser 8 MHz après contrôle à l'analyseur logique,
avec fils courts et vérification électrique. **Aucun débit n'a été mesuré.** Le DMA
évite les limites d'un UART émulé, les données sont envoyées par blocs de 1 Kio et
seuls les secteurs couvrant l'image sont effacés. Le débit total dépend aussi des
temps d'effacement/programmation de la Flash.

Sources ST : [DS8626, table des fonctions alternatives, pages 64–65](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf),
[RM0090, DMA1 et SPI half duplex](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
SPI3 utilise DMA1 stream 0/channel 0 en réception et stream 5/channel 0 en émission.
Le front montant de NSS marque la fin du transfert ; le BL n'attend pas indéfiniment
le bit BSY d'un SPI esclave.

### Entrée dans le BL

- Menu `Firmware update` : moteurs arrêtés, marqueur RTC BKP0R, reset logiciel.
- À distance : le pont maintient NSS bas, pulse NSRST bas, attend au moins 100 ms
  après relâchement du reset puis relâche NSS. Le BL contrôle NSS après 50 ms.
- Une image absente/invalide laisse automatiquement le robot dans le BL.
- En fonctionnement normal, le BL valide l'application puis lui passe la main.

La liaison distante n'est active que dans le BL. Les UART sont désactivés pendant
les essais Nîmes ; le pont radio externe doit également être désactivé/déconnecté
pour l'épreuve. La récupération SWD reste disponible sur PA13/PA14.

### Contrat du pont radio

Une requête SPI fait exactement **1056 octets**, une réponse exactement **32**.
Tous les mots sont little endian, SPI MSB-first au niveau des octets.

1. Le pont abaisse NSS, attend ≥100 µs, transmet la requête puis remet sa broche
   DATA en entrée **avant** de relever NSS (plus aucune horloge).
2. Le BL force DATA bas pendant le traitement, parfois plusieurs secondes pour
   un effacement. Le pont attend au moins 100 µs avant d'échantillonner DATA,
   puis attend DATA haut (READY), avec timeout de 60 s.
3. Le pont abaisse NSS, attend ≥100 µs, reçoit 32 octets sur DATA puis relève NSS.
   Le BL connecte la sortie SPI seulement après ce second front descendant.
4. Attendre ≥1 ms avant la requête suivante. Aucun autre maître ne doit piloter
   ces lignes pendant la session. Réception/émission abandonnée : le BL réarme RX
   après timeout ; réinitialiser la session et recommencer en cas de désynchronisation.

Le pont TCP envisagé expose un flux : chaque paquet de 1056 octets reçu sur le port
9020 produit cet échange SPI et renvoie les 32 octets. Il doit assembler les lectures
TCP partielles, sérialiser les requêtes et appliquer les timeouts/retournements.

Requête : 8 mots (`magic=0x3257465A`, commande, séquence, arg0, arg1, longueur,
version=1, CRC32), suivis de 1024 octets de zone utile/remplissage. Le CRC IEEE
couvre les 28 premiers octets puis les `longueur` octets utiles, sans remplissage.
Réponse : magic, séquence renvoyée, statut (0=OK), taille maximale, offset reçu,
adresse application, version, CRC32 des 28 premiers octets.

| Commande | Arguments |
|---|---|
| 1 INFO | longueur 0 |
| 2 BEGIN | arg0=taille, arg1=CRC32 de toute l'image ; efface la zone temporaire nécessaire |
| 3 DATA | arg0=offset, longueur 4..1024, alignée sur 4 ; offsets séquentiels |
| 4 COMMIT | longueur 0 ; vérifie, installe, revérifie, puis répond |
| 5 BOOT | longueur 0 ; démarre l'application valide après avoir transmis la réponse |

Un bloc déjà reçu identique peut être répété. Les CRC, tailles, vecteurs et offsets
sont contrôlés avant installation. Le CRC protège l'intégrité accidentelle ; il ne
constitue pas une signature ni une authentification. L'accès radio devra être
restreint par le pont choisi, avant usage réel.

```sh
# Seulement avec un pont qui implémente le contrat ci-dessus et le robot dans le BL :
python3 tools/fw_upload.py build/release/application.ota.bin ADRESSE_DU_PONT
```

### Coupures pendant une mise à jour

L'application en place reste intacte pendant la réception. Une fois l'image
entièrement validée, le BL écrit un manifeste PENDING et son marqueur de validation,
puis copie la zone temporaire vers l'application. Une coupure pendant cette copie
entraîne sa reprise complète au prochain démarrage. Les snapshots A/B sont exclus
de toutes ces opérations. Après vérification finale, le manifeste devient INSTALLED.

Une coupure pendant l'effacement/écriture du manifeste peut laisser un manifeste
invalide : le BL reste alors disponible pour **renvoyer l'image**. Il ne démarre pas
une application dont il ne peut plus vérifier la longueur et le CRC. Ce n'est pas
un mécanisme de retour automatique à l'ancienne version après installation réussie.

## Recherche et chemin

`nimes.c` est indépendant du matériel : carte compacte, murs connus/inconnus
séparés, réciprocité des murs, limites fermées et observations atomiques. Une mesure
contradictoire n'efface pas silencieusement la carte : arrêt et diagnostic au menu.

Dijkstra explore les états **case + orientation**. Le coût `1024 × nombre de cases
+ quarts de tour` minimise d'abord la distance, puis les rotations à distance égale.
L'exploration rejoint la frontière accessible la plus proche par des passages
confirmés. Après détection de la salle, comparaison du meilleur chemin connu à une
borne optimiste où les passages inconnus seraient ouverts. La recherche ne raffine
que les inconnues susceptibles d'améliorer ce chemin. Quand les coûts sont égaux,
le plus court chemin est certifié pour la carte observée ; retour au départ.

Ce critère ne prétend pas minimiser le temps réel de course, ni le score global
avec pénalités. Il évite l'hypothèse historique d'une arrivée imposée au centre et
les allocations dynamiques dans le solveur.

## Déplacements, acquisitions et asservissement

| Travail | Exécution |
|---|---|
| Impulsions STEP, budget fini par roue | TIM2/TIM3 et interruptions priorité 1 |
| Alternance IR 5/10 cm | TIM6, 200 phases/s, priorité 2 |
| Trame complète horodatée + filtre | 100 trames/s, compteurs saturants avec hystérésis |
| Profil vitesse et correction | SysTick 1 kHz, priorité 4 |
| Choix du trajet, carte, menu/OLED | Premier plan, machine à états |

La génération des impulsions et l'acquisition continuent pendant les calculs de
chemin. Le trajet de retour/course est anticipé pendant le déplacement précédent.
Les observations cartographiques sont acceptées aux centres des cases, après
stabilisation : les capteurs tout-ou-rien ne localisent pas précisément un mur en
pleine transition. Les corridors rectilignes déjà cartographiés sont regroupés en
un mouvement, jusqu'à la première destination non visitée. Les cellules inconnues
et les rotations gardent des arrêts ; les virages courbes et la cartographie de
nouvelles cellules sans arrêt ne sont pas activés sans validation physique.

Le nouveau contrôleur utilise un profil avec accélération bornée, correction différentielle filtrée, terme dérivé et vitesse
de correction limitée. Pas d'intégrale accumulée dans une ouverture. Les budgets
d'impulsions indépendants arrêtent chaque roue à sa cible ; une correction bornée
réduit leur décalage. Trame âgée de plus de 50 ms, obstacle frontal inattendu ou mouvement
supérieur à 60 s : arrêt et défaut mémorisé. Le bouton retour arrête l'essai.

Les timers moteurs étaient calculés à 42 MHz alors que leur horloge APB1 doublée
est 84 MHz. La conversion distingue les micro-pas mécaniques et les deux événements
de basculement STEP par impulsion ; le pilote historique compensait déjà ce facteur
2 dans ses déplacements, sa macro de conversion est conservée. Le nouveau pilote
fait cette conversion explicitement. Les GPIO moteurs sont initialisés complètement.

Vitesses initiales : exploration 220 mm/s, course 260 mm/s, plage 20–300 mm/s ;
rotation sur place 120 mm/s par roue. Géométrie reprise du projet (roues 24,45 mm,
entraxe 83,5 mm). Gains, hystérésis, vitesse et freinage nécessitent des mesures sur
robot : un test logiciel ne démontre ni absence de perte de pas, ni diminution des
oscillations sur le sol réel. Le robot n'a pas d'estimation métrique des distances
aux murs avec ses seuls capteurs binaires.

## Vérifications effectuées

`make test` compile le cœur et le contrôleur sous AddressSanitizer et UBSan :

- CRC de référence et CRC incrémental ; sauvegardes inchangées sans effacement.
- 139 points de coupure de sauvegarde (programmation partielle/effacement partiel),
  récupération de la copie précédente, corruption et version de schéma.
- 1027 points de coupure d'installation d'une image test de 4 Kio ; redémarrage
  et restauration depuis l'image temporaire, plus 11 coupures de manifeste.
- Requêtes corrompues, tailles/offsets invalides, doublons de blocs.
- 500 cartes aléatoires comparées à BFS pour la longueur ; départs et limites,
  salle non centrale, départage par rotations.
- 40 explorations simulées de labyrinthes générés, avec les quatre coins de
  départ ; arrivée découverte et coût optimal comparé à la carte complète.
- Correction gauche/droite symétrique, limitation du changement et retour à zéro.
- Simulation du contrôleur matériel à 20..300 mm/s : budgets d'impulsions,
  rotations, capteurs périmés, obstacle et débordement du compteur de temps.
- Tests Python : image/vecteurs, manifeste, HEX discontinu, CRC des paquets,
  séquence complète d'envoi avec réponses TCP fragmentées.

Les simulations d'effacement/programme ne modélisent pas les baisses de tension
analogiques du silicium. Restent les essais au banc : SWD avec la sonde ancienne,
démarrage réel du BL, SPI et changement de sens DATA, coupure d'alimentation réelle,
mesure d'une case/rotation et réglage des oscillations, puis exploration sur sol.


## Corrections après essai sur le robot

Régressions signalées : exploration lente, arrêt en ligne droite, départ à la main
absent, carte absente et textes débordant de l'écran.

- Départ rétabli par F10 : main détectée pendant 200 ms, puis retrait confirmé
  pendant 100 ms. Les moteurs restent désactivés durant cette attente. Retour annule.
- Carte 9×9, murs inconnus en pointillés, salles d'arrivée et pose/orientation
  affichés pendant l'attente, la navigation et sur le résultat. Menu `Voir carte`.
- Titres courts, textes des boîtes découpés en lignes ; le rendu coupe à la bordure
  avant la conversion des coordonnées en octets. Correction d'un décalage binaire
  indéfini dans le rendu des polices de hauteur 8.
- Le précédent profil descendait à 5 mm/s à chaque case. Remplacement par une
  enveloppe de freinage avec accélération bornée à 800 mm/s² et vitesse finale de
  40 mm/s (ou vitesse demandée si inférieure). Une case simulée passe de 2115 ms
  à 120 mm/s avec l'ancien profil à 992 ms à 220 mm/s avec le nouveau. Ce sont des
  mesures de simulation, pas des chronométrages du robot.
- Réutilisation de `CELL_LENGTH=178` comme distance **calibrée** de commande,
  déjà employée par le pilote historique ; la géométrie logique Nîmes reste 180 mm.
  La base des timers moteurs passe à 2 MHz pour réduire la quantification des
  corrections de vitesse.
- F5 ne doit plus abandonner toute l'exploration lors d'une arrivée normale devant
  un mur. Cette fin de mouvement est autorisée seulement dans les 30 derniers mm
  de la dernière case et si le passage frontal de destination n'est pas déjà connu
  ouvert. Les autres détections arrêtent toujours le robot avec `OBSTACLE HORS CASE`.
  La pose est alors estimée à la case d'arrivée ; ce seuil nécessite une vérification
  physique de l'alignement. Il ne remplace pas une mesure métrique de distance.
- Une contradiction de mesure est relue pendant 150 ms avant arrêt explicite ;
  aucune contradiction persistante n'est effacée silencieusement de la carte.
- Snapshot schéma 2 : import du schéma 1, migration unique des anciennes vitesses
  par défaut 120/200 vers 220/260. Les autres valeurs et la carte sont conservées.

Tests supplémentaires : départ à la main et capteurs périmés, arrêt devant un mur
puis rotation, obstacle trop précoce, trajectoire d'application complète (10
mouvements, 6 rotations, découverte de salle et retour), rendu réel du framebuffer
OLED sur les 81 positions, découpage et absence de rebouclage du texte à gauche.
Les rendus de test sont `build/ui-maze.pgm` et `build/ui-prompt.pgm`.

### Installation des corrections — 26 septembre 2026

Firmware `812789e` installé avec la version antérieure de
`scripts/build/build.sh --test` via la ST-Link
V2J16S4 à 1000 kHz. Sauvegarde préalable des 1 Mio, programmation puis comparaison
complète : images conformes, secteurs de persistance et autres zones non écrites
inchangés. Application de 54 996 octets, SHA-256
`67e1c4a9b0e242c1b1de4a31fdf1dd6e696aca65ced649cc2fb6b531bc7f3b07`.

Session locale : `backups/flash-sessions/20260926T174234Z-6HY8FN/`.
Après redémarrage : VTOR `0x08010000`, CFSR/HFSR nuls, moteurs désactivés,
ticks et séquence capteurs progressant. Ces contrôles au repos ne valident pas
encore les déplacements, le départ à la main ni l'exploration sur le terrain.

## Calibration des capteurs binaires dans une case à trois murs

Menu `Calibration` → `Wall calibration`, rapport `Wall report`.
Réglages : `Nose x0.1mm`, `Width x0.1mm`, `Cell clear mm`, `Cell pitch mm`.
Un schéma vu de dessus montre les murs, le robot au centre et une flèche vers
le mur frontal. La procédure démarre uniquement après l'action `RIGHT:GO` ; Retour arrête
les moteurs pendant toute la mesure. Elle n'est jamais lancée au démarrage.

Les cotes fournies sont préremplies : **47 mm entre axe et avant, 94 mm de largeur**
(robot carré). Le calcul d'encombrement suppose le robot centré sur l'axe des roues,
avec le même encombrement devant et derrière. Il vérifie le rayon balayé par ce
rectangle avec 2 mm de marge avant d'autoriser les positions de mesure latérales.
Les valeurs 167 mm entre faces et 179 mm entre axes de murs sont préremplies,
modifiables au menu. Le centre est donc à 83,5 mm d'une face de mur, pas à 89,5 mm.

Placer le robot approximativement au centre, face au mur du fond, avec un mur à
gauche et à droite et l'ouverture derrière ; laisser 10 cm libres derrière la case
pour les reculs de mesure. Les trois capteurs 10 cm doivent détecter leurs murs.
La première rotation suppose également cet espace latéral de départ : les
capteurs binaires seuls ne peuvent pas certifier une position précise au départ.

La séquence automatique est la suivante :

1. Appui frontal à 30 mm/s. Le premier appui de chaque axe couvre au maximum
   `largeur intérieure − 2 × axe-avant + 5 mm` de course moteur ; cela peut
   commander plusieurs secondes de pas après le contact. Une fois la référence
   établie, les appuis suivants ajoutent seulement 5 mm à la course attendue.
   Si la position est connue, l'approche se fait à 80 mm/s jusqu'à 8 mm du
   contact prévu, puis à 30 mm/s pour l'appui et les 5 mm supplémentaires.
   Une première prise de référence, de position incertaine, reste à 30 mm/s.
   F10 vérifie le mur avant l'approche ; l'état F5/F10 au contact n'est pas
   utilisé comme contacteur mécanique.
2. Trois reculs/avances à 20 mm/s, depuis l'appui jusqu'à une distance axe-mur
   égale à la largeur intérieure. Les transitions F5 et F10 sont confirmées sur
   trois acquisitions brutes consécutives ; la position du premier échantillon
   est retenue pour limiter le biais du filtrage. Les moyennes de déclenchement,
   relâchement, hystérésis et dispersion sont calculées séparément. Si le capteur
   ne détecte pas à très courte distance, le recul attend l'entrée dans la bande
   détectée avant de mesurer sa sortie éloignée. Au retour, seule cette zone proche
   observée au recul (avec 2 mm de marge) peut contenir d'autres transitions.
   Un capteur sans bande détectée ou sans sortie reste une erreur.
3. Recul au centre, rotation de 90°, appui sur le mur perpendiculaire et recul
   de `largeur intérieure / 2 − axe-avant`. Pour mesurer le capteur gauche, le
   robot se tourne vers le mur droit ; le mur initial se trouve alors à gauche.
   La procédure est symétrique pour le capteur droit.
4. À partir du centre, si le capteur latéral 5 cm détecte, le point suivant est
   éloigné du mur de 1 mm ; sinon il est rapproché de 1 mm. Chaque point répète
   les appuis et le recentrage. Trois recherches par côté donnent un intervalle
   détecté/libre, élargi de la dispersion observée. Les répétitions 2 et 3
   commencent près du seuil précédent, avec de nouveaux appuis à chaque point,
   pour éviter de reparcourir tous les millimètres depuis le centre.
   La recherche s'arrête à
   ±15 mm au plus, réduits si le rayon de rotation l'impose.
5. Appui final et retour au centre dans l'orientation initiale. Sauvegarde
   automatique de la calibration avec les réglages et la carte, puis rapport
   OLED en cinq pages : géométrie, F5, F10, gauche 5 cm et droite 5 cm.

Le rapport distingue l'hystérésis frontale, mesurée avec une orientation constante,
des intervalles latéraux **statiques** : les rotations et appuis entre les points
ne permettent pas d'isoler l'hystérésis optique latérale. Les distances sont
estimées à partir des pas et de la géométrie des roues, par rapport au centre de
rotation. Sans codeur ni contacteur, le firmware ne mesure ni l'effort contre le
mur ni les pas perdus ; les résultats dépendent de la réussite des appuis, de
l'absence de glissement et des cotes saisies.

Un mur absent, une acquisition périmée, un seuil hors plage, des transitions
instables ou une dispersion supérieure à 2 mm interrompent la procédure. Aucun
résultat partiel ne remplace la calibration sauvegardée. Les mouvements de contact
sont réservés à cette procédure : distance bornée à 180 mm, vitesse au plus 30 mm/s,
repositionnements à 80 mm/s et rotations à 80 mm/s par roue, bouton Retour et surveillance des acquisitions actifs.

Le snapshot utilise le schéma 4 et importe les anciens schémas 1/2/3 sans perdre
réglages ou carte. Le choix du secteur à effacer prend désormais en compte le
dernier snapshot valide, même d'un autre schéma, pour conserver une copie lors
d'une coupure pendant la migration.

Les déplacements par case utilisent le pas saisi (179 mm par défaut, également
sans calibration). Après calibration des murs, à l'approche d'un mur final, le seuil frontal F5 mesuré définit
la zone de détection attendue. Le robot termine son budget de pas jusqu'au centre,
au lieu de déclarer immédiatement une arrivée jusqu'à 30 mm avant celui-ci.
Un obstacle plus précoce reste un défaut. Les seuils latéraux sont mémorisés et
consultables ; ils ne constituent pas une mesure continue de la distance et ne
suffisent pas, à eux seuls, à valider un nouvel asservissement de trajectoire.

Validation logicielle : simulation de la séquence complète avec appuis (pas
commandés mais position bloquée), défaut de centrage initial, seuils et hystérésis
connus, bruit ponctuel, seuil absent, annulation et encombrement en rotation ;
tests des déplacements signés réels du contrôleur, arrêt sur acquisition périmée,
arrivée calibrée, restauration après redémarrage, échec de sauvegarde et coupures
pendant migration des deux secteurs. La calibration et la précision des
trajectoires restent à vérifier sur le robot avec les dimensions réelles.


## Calibration de rotation avec les capteurs tout ou rien

`Calibration` → `Rotation cal.`, dans la même case à trois murs, robot au centre,
face au mur du fond, ouverture derrière. Les appuis successifs sur le mur gauche
puis frontal établissent les deux références. Aucun module ou capteur supplémentaire
n'est nécessaire. Durée : plusieurs minutes ; `BACK` interrompt les moteurs.

La référence optique est le milieu du **plus grand intervalle libre du capteur
frontal 10 cm**, correspondant à l'ouverture arrière. Ce repère réapparaît tous
les 360° pour le même capteur et le même sens. La différence entre passages
annule l'angle de montage fixe et le seuil propre au capteur. Si plusieurs
intervalles rendent la période ambiguë ou si aucun repère n'est exploitable,
la calibration est refusée.

Pour chacune des vitesses **40, 80 et 120 mm/s par roue**, dans les deux sens :

1. Appuis et recentrage ; cinq tours nominaux pour estimer la période en course
   de roue. L'entraxe effectif vaut `course par tour / pi`.
2. Retour à une orientation de tour complet, puis **16 quarts de tour avec
   arrêts**, suivant le profil d'accélération/freinage réellement utilisé.
   Jusqu'à trois passes corrigent le budget de pas à partir de la période optique.
3. Lorsque le frontal 5 cm sépare les trois murs en trois zones détectées,
   leurs centres fournissent aussi un contrôle optique à 90°. Un écart supérieur
   à 2° rejette la mesure. Sinon le rapport indique explicitement
   `90: PERIOD / 4 ONLY` : seule la division de la période est disponible.
4. Sauvegarde après réussite des six profils ; rapport `Rotation report` :
   entraxe effectif, course pour 90°, dispersion des périodes et contrôle à 90°.

La correction est appliquée aux virages normaux (120 mm/s par roue) et aux
rotations de référence (80 mm/s), séparément CW/CCW. L'API interpole entre profils
mesurés, sans extrapolation. Un demi-tour utilise deux fois le budget du quart de
tour ; son effet de glissement propre n'est pas calibré indépendamment.

Il s'agit d'un **entraxe effectif**, incluant les effets reproductibles de roulement,
non d'une mesure mécanique des roues. Les appuis et le sol doivent être répétables.
La littérature [UMBmark, Borenstein et Feng](https://websites.umich.edu/~ykoren/uploads/Umbmark.pdf)
distingue les erreurs systématiques d'entraxe des erreurs dues au sol et au
glissement, et motive les essais dans les deux sens. La référence optique de la
case à trois murs et les séquences décrites ici sont l'implémentation de ce projet,
pas le parcours carré UMBmark. Un tour correct ne garantit pas à lui seul chaque
quart de tour ; le contrôle F5 et l'essai réel restent nécessaires.

## Calibration de détection d'angle et de porte

`Calibration` → `Corner detection` → `Left wall fixture` ou `Right wall fixture`.
Faire les deux montages. La calibration de rotation doit être enregistrée pour
la même géométrie. Chaque lancement commence dans la case d'angle : mur frontal,
un seul mur latéral, côté opposé ouvert. Une portion droite libre d'au moins deux
cases prolonge le trajet derrière le robot.

Exemple du montage gauche, vu de dessus :

```text
           mur frontal
       +-------------------
       |       ^ robot     côté droit ouvert
       |       |           axe à 83,5 mm de la face frontale
       |
       # poteau de fin du mur gauche
               |           parcours aller/retour
               |
```

`Post ctr mm` est la distance **face intérieure du mur frontal → centre du poteau**.
Par défaut **173 mm** : 167 mm libres + 6 mm (demi-épaisseur du mur de 12 mm).
Ce n'est pas le pas de 179 mm entre centres de murs. Modifier cette cote si le
montage réel diffère ; elle est enregistrée avec chaque résultat.

- Appui sur le mur latéral choisi, recul au centre, rotation vers le mur frontal,
  appui frontal, puis recul au centre : les deux zéros sont établis.
- Trois aller/retours à chacune des vitesses **40, 120 et 220 mm/s**. Le robot
  dépasse le poteau jusqu'à 100 mm derrière son centre. Les fronts doivent se
  produire sur le plateau de vitesse, pas pendant accélération ou freinage.
- Première orientation : recul jusqu'à l'ouverture puis avance jusqu'à la
  fermeture. Après retournement de 180°, la même géométrie mesure une **ouverture
  en marche avant** et une fermeture en marche arrière avec le capteur opposé.
  Chaque répétition reprend les appuis de référence.
- Acquisition des seuils 5 et 10 cm du côté concerné. Le 10 cm est obligatoire ;
  le 5 cm est omis s'il ne voit pas le mur à la position centrale.
- Rapport `Corner report` : côté physique L/R, capteur, sens d'ouverture, vitesse,
  positions brutes/filtrées d'ouverture et de fermeture et dispersion, en mm
  **par rapport au centre du poteau**. Positif signifie au-delà du poteau, en
  s'éloignant du mur frontal, quelle que soit l'orientation du robot.

Les fronts bruts sont confirmés sur trois scans en conservant la position du
premier ; les fronts filtrés enregistrent le retard réellement vu par le contrôleur.
La dispersion maximale acceptée est 5 mm, l'offset maximal 60 mm. Les données des
deux montages sont sauvegardées séparément. Une nouvelle calibration de rotation
invalide les anciennes données d'angle ; un changement de géométrie invalide les
mesures incompatibles. Une erreur de sauvegarde conserve les anciens profils.

`fw_corner_offset()` expose les offsets filtrés interpolés dans la plage mesurée.
Ces mesures sont consultables et persistées ; **le déclenchement anticipé de virages
pendant l'exploration n'utilise pas encore ces offsets**. Cela nécessite une
validation sur la piste avec les murs et les capteurs réels.

## Validation de cette extension

- `make test` : ASan/UBSan, simulations géométriques des capteurs binaires,
  entraxes distincts selon sens/vitesse, effet des arrêts, deux montages d'angle
  et deux orientations, référence absente et annulation.
- Contrôleur réel testé avec moteurs simulés : budgets CW/CCW 90°/180°, signe de
  course en rotation, limites de déplacement, acquisitions périmées.
- Migration des snapshots 2/3 vers 4, restauration des profils après redémarrage,
  échec de sauvegarde et invalidation des mesures dépendantes.
- 35 pages de rapports rendues avec le pilote OLED réel ; compilation application
  et bootloader en Debug (`-Og`) et Release (`-O3`).

Aucun mouvement matériel ni flash exécuté pour cette extension. Les résultats
numériques des tests sont simulés ; la précision physique n'est pas encore validée.


## Correction des arrêts de calibration après appui

Les anciennes procédures exigeaient à la fois F5/F10 détectés au contact et cinq
scans identiques sur les six capteurs. Ces hypothèses pouvaient interrompre
l'appui commun aux calibrations mur/rotation : état optique différent à très courte
distance, ou clignotement d'un capteur latéral sans rapport avec l'étape courante.
Ces scénarios sont reproduits par les tests ; ils ne constituent pas encore un
diagnostic confirmé de l'arrêt observé sur le robot.

Les lectures attendent désormais la stabilité des seuls bits utiles : trois
10 cm pour le montage initial, frontal 10 cm avant appui, paire latérale pour sa
mesure. Au début du balayage depuis le contact, les scans doivent être frais mais
leur état n'est pas imposé. Les contrôles de fraîcheur, d'arrêt utilisateur,
de course maximale et de validité des seuils restent actifs.

Les écrans de lancement montrent les murs et le robot vu de dessus (avec roues et
flèche de cap), y compris les montages d'angle en miroir et leur poteau. Les rares
légendes indiquent l'espace libre derrière. Aucun déplacement n'est lancé avant
`RIGHT:GO` ; `BACK:EXIT` permet d'annuler depuis le schéma.

L'écran d'arrêt affiche le code `E` de calibration, le défaut moteur `M`, la cause
(`IR NOT STABLE`, `IR DATA STALE`, `MOVE REJECTED`…), la dernière étape et les
états F5/F10/bruts. Une photo donne donc les informations nécessaires au diagnostic.
Les codes E2/E3/E4/E5 correspondent respectivement à un montage incorrect,
un problème de mouvement ou d'acquisition, un seuil absent, des mesures instables.
Aucune mesure partielle n'est sauvegardée.

Validation : balayages complets avec zones proches non détectées, recalage de
rotation avec cette même condition, lecture de montage avec R5 alternant à chaque
scan, refus d'un F10 réellement instable, annulation depuis les trois schémas et
rendu OLED des erreurs. Application et bootloader compilés Debug et Release.


## Interface, bibliothèque et flash rapide

### Navigation

Le menu principal est maintenant `Maze`, `Calibration`, `Settings`, `Update`.
Une seule action occupe l'écran, avec icône et texte 7×16 pixels. Haut/bas parcourt,
droite **ou appui central** valide, gauche du joystick revient. PC13 est Escape,
également actif pour arrêter les mouvements. Le relâchement est consommé pour
éviter qu'un seul appui fasse revenir de deux niveaux. Le choix reste mémorisé
lorsqu'on revient dans un menu. Les anciens réglages PID/moteurs sans effet sur
le contrôleur moderne ne sont plus présentés comme réglages de navigation.

`Settings` regroupe neuf paramètres effectifs : axe–avant, largeur, intérieur et
pas de cellule, centre du poteau, vitesses d'exploration/run rapide, coin et cap
de départ. Les valeurs sont bornées, illustrées, sauvegardées sur validation ;
gauche annule l'édition. Les dimensions saisies préparent la prochaine calibration ;
elles ne remplacent pas silencieusement la géométrie des mesures déjà validées.

Les schémas d'angle montrent le poteau, la cellule arrière, le trajet libre
(flèche), les murs obligatoires en trait plein et les segments indifférents en
pointillés. La prolongation du mur **après le poteau du côté mesuré doit rester
ouverte**. La frontière frontale de la cellule de départ et le mur latéral opposé
suivent le montage à deux murs indiqué. Les murs supplémentaires ne doivent pas
couper le trajet libre. Le montage droit est le miroir du gauche.

Les séquences gauche/droite ont le même nombre d'avances/reculs et de rotations,
vérifié en simulation. Après une mesure, le rapport n'affiche plus que le montage
qui vient d'être mesuré. Auparavant, le second lancement affichait les deux jeux
de résultats, donc davantage de pages. Le temps d'attente d'un signal stable peut
aussi varier entre côtés ; il n'y a pas de séquence droite volontairement doublée.

### Bibliothèque de labyrinthes

Le schéma de sauvegarde **5** ajoute huit emplacements au snapshot transactionnel
existant. Le préfixe contenant réglages, carte active et calibrations conserve son
ABI ARM ; migration des versions 1 à 4. L'espace reste inférieur aux 16 Kio de
chaque secteur A/B. La mémoire de travail du snapshot est statique pour ne pas
consommer la pile de 8 Kio utilisée par la recherche de chemin.

La carte partielle active reste sauvegardée séparément. Une nouvelle entrée est
archivée à l'issue de la recherche certifiée et du retour au départ réussis. Une
carte certifiée exige une arrivée unique et un chemin connu ayant le même coût
que la borne optimiste autorisant les passages inconnus. Cela n'exige pas de visiter
toutes les cellules sans intérêt pour le chemin optimal.

La bibliothèque déduplique les cartes identiques, conserve le départ, la durée et
le chemin calculé. Le défilement affiche murs et chemin clignotant toutes les
500 ms. Valider charge la carte et ouvre directement les runs. La bibliothèque
pleine ne supprime rien automatiquement : la carte active est conservée et une
suppression explicite avec confirmation libère un emplacement.

Les runs sont bloqués côté firmware et masqués au menu tant que l'apprentissage
n'est pas validé. La certification est recontrôlée pour le départ sélectionné.
Le run lent commande 120 mm/s sans modifier la vitesse rapide sauvegardée. Les
virages restent des rotations sur place suivies de lignes droites ; la carte
`Curves / Unavailable` annonce honnêtement l'absence de contrôleur de courbes.

### Utilisation des calibrations

| Données | Exploration | Runs | Fonction |
|---|---|---|---|
| Pas de cellule | Oui | Oui | Distance commandée par cellule |
| Frontal F5 | Oui | Oui | Fenêtre d'arrivée devant un mur final |
| Rotation CW/CCW | Oui | Oui | Budget de pas des virages |
| Intervalles latéraux 5 cm | Oui, nouveau | Oui, nouveau | Bornes de position pour le centrage |
| Hystérésis F10 / offsets de poteau | Rapport | Rapport | Pas encore utilisés pour anticiper les virages |

Le centrage construit un intervalle possible de distance axe–mur gauche à partir
des deux observations binaires et des intervalles calibrés. Si cet intervalle
contient le centre, aucune correction arbitraire n'est demandée. Sinon, l'écart
minimal certain donne une correction bornée ; filtrage et variation limitée de
commande restent actifs. En cas d'absence de murs ou de bornes contradictoires,
la correction décroît. Sans calibration valide, l'ancien contrôle logique reste
le repli. Une position continue exacte n'est pas observable avec ces capteurs.

### Flash et extraction

`scripts/flash/flash.sh --release --fast` conserve une sauvegarde préalable de la
zone basse jusque la fin du dernier secteur susceptible d'être effacé (incluant
A/B). `verify_image` contrôle chaque image ; une relecture des 32 Kio A/B vérifie
leur identité exacte. Le mode complet, sans `--fast`, garde les deux dumps de
1 Mio et la vérification de tous les octets hors zones programmées. Aucun mode
n'efface les secteurs A/B. Le débit SWD reste 1 MHz, compatible avec l'ancienne sonde.

`scripts/flash/dump.sh [--calibration-only]` ne programme rien. Il capture flash ou
A/B, écrit une empreinte SHA-256 et produit `calibrations.json` avec le décodeur
`tools/calibration_dump.py`. Le rapport contrôle CRC et générations, expose les
mesures signées par vitesse/sens, et signale les valeurs hors bornes ou les
incohérences à examiner. Un CRC correct ne valide pas la précision mécanique.

### Validation et limite matérielle de cette révision

Tests ASan/UBSan : bibliothèque pleine/déduplication/suppression/chemin invalide,
sauvegarde automatique et restauration, migration 2/3/4 vers 5, commandes joystick,
blocage des runs, chargement direct, centrage avec seuils asymétriques et rendu OLED.
Tests Python : sauvegarde rapide, identité des secteurs persistants, marqueurs de
vérification des images, décodage CRC/générations et offsets négatifs.

La lecture demandée du robot a été tentée : sonde V2J16S4 détectée, mais tension
cible ~0,03 V, connexion impossible. L'archive antérieure au dernier essai ne contient
que le schéma 2, sans calibrations. **Aucune conclusion sur les calibrations actuelles
ne peut donc être tirée de cette archive.** Aucun flash ni mouvement automatique
n'a été exécuté pour cette révision ; le dump actuel nécessite une cible alimentée.

### Accélération des calibrations et icônes

Les vitesses de mise en position sont définies dans `fw_calibration.h` :
repositionnement 80 mm/s, contact 30 mm/s, mesure frontale 20 mm/s, rotation
de référence 80 mm/s par roue. Les profils mesurés restent 40/80/120 mm/s
pour la rotation et 40/120/220 mm/s pour les angles : ils décrivent les vitesses
réelles utilisées par les déplacements. Les trois répétitions, les contrôles
de dispersion, la surveillance capteurs et l'arrêt utilisateur sont conservés.
Les sauvegardes de calibration existantes restent compatibles.

Le cas de test des seuils latéraux décalés passe de 78 à 42 rotations de mise
en position, avec les mêmes intervalles détectés. Ce résultat est simulé ;
le gain de durée et la répétabilité à vitesse augmentée restent à mesurer
sur le robot. Le menu présente quatre pictogrammes distincts : trois murs
avec flèche d'appui, rotation, poteau gauche et poteau droit.
