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
python3 tools/fw_upload.py build/application.ota.bin ADRESSE_DU_PONT
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

Firmware `812789e` installé avec `.scripts/build/build.sh --test` via la ST-Link
V2J16S4 à 1000 kHz. Sauvegarde préalable des 1 Mio, programmation puis comparaison
complète : images conformes, secteurs de persistance et autres zones non écrites
inchangés. Application de 54 996 octets, SHA-256
`67e1c4a9b0e242c1b1de4a31fdf1dd6e696aca65ced649cc2fb6b531bc7f3b07`.

Session locale : `backups/flash-sessions/20260926T174234Z-6HY8FN/`.
Après redémarrage : VTOR `0x08010000`, CFSR/HFSR nuls, moteurs désactivés,
ticks et séquence capteurs progressant. Ces contrôles au repos ne valident pas
encore les déplacements, le départ à la main ni l'exploration sur le terrain.
