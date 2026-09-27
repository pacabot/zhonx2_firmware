# ZHONX II M4

## Compilation

Pré-requis : GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`), GNU Make et Python 3.

```sh
scripts/build/build.sh
```

Le script compile uniquement l'application en mode debug (`-Og -g3`), sans accès
au robot. `--bootloader` construit uniquement le bootloader ; `--all` construit
les deux images et le paquet complet. `--release` utilise `-O3` ; `--debug` est
le défaut. `--help` décrit également `--test` et `--clean`. Les sorties sont
séparées entre `build/debug/` et `build/release/`.

La cible est un STM32F405RG avec quartz externe à 8 MHz. Le script `pacabot_link.ld` place la table des vecteurs de l’application à `0x08010000` ; le bootloader occupe `0x08000000`. Le fichier `cmsis_boot/system_stm32f4xx.c` reste la configuration d'horloge spécifique à cette carte.

## Bibliothèques

Les fichiers utilisés de la STM32F4 Standard Peripheral Library et de CMSIS ont été mis à jour depuis le paquet STSW-STM32065 **V1.9.0** (les en-têtes du paquet indiquent SPL 1.8.x et CMSIS 4.00). Source de référence : [fiche ST](https://www.st.com/en/embedded-software/stsw-stm32065.html). Copie de travail : [miroir GitHub](https://github.com/synapticonashaikh/STM32F446RE-Sample-codes/tree/684f3c374d5a92ed025aca768537a3a8df66463b/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries), révision `684f3c374d5a92ed025aca768537a3a8df66463b`.

## Sauvegarde de la carte

La copie complète du firmware d'origine et ses informations de vérification sont dans `backups/`. Les corrections sont documentées dans `docs/FIRMWARE.md`.

## Flasher par ST-Link

Robot alimenté et sonde branchée (les anciennes ST-Link/V2 sont prises en charge) :

```sh
scripts/flash/flash.sh
```

Le script fonctionne depuis n'importe quel dossier. Sans option, il compile puis
flashe uniquement l'application et son manifeste en Debug. `--release` sélectionne
la version optimisée ; `--bootloader` n'écrit que le bootloader ; `--all` écrit les
trois images. La compilation est incrémentale, **y compris avec `--fast`** :
`scripts/flash/flash.sh --release --fast` remet à jour `build/release/` avant
programmation. Si les images sont déjà à jour, Make ne les recompile pas.
Un échec de compilation annule le flash avant tout accès à la cible.

Pour mettre à jour en conservant les calibrations :

```sh
scripts/flash/flash.sh --release
```

**Le mode rapide est maintenant le défaut.** Il sauvegarde seulement les **48 Kio**
du bootloader et des données persistantes, puis programme les images demandées.
Sans `--verify`, il ne relit pas la flash après écriture et ne lance pas
`verify_image`. `--fast` reste accepté comme alias explicite de ce mode.
Cette sauvegarde minimale ne contient pas l'ancienne application.

```sh
scripts/flash/flash.sh --release --verify                 # contrôle images + données
scripts/flash/flash.sh --release --full-backup            # sauvegarde 1 Mio avant
scripts/flash/flash.sh --release --full-backup --verify   # contrôle complet avant/après
```

| Options | Sauvegarde avant | Relecture après |
|---|---:|---:|
| défaut / `--fast` | 48 Kio | aucune |
| `--verify` | 48 Kio | images contrôlées sur cible + 32 Kio de données |
| `--full-backup` | 1 Mio | aucune |
| `--full-backup --verify` | 1 Mio | 1 Mio, comparaison complète |

Les **secteurs 1 et 2 (`0x08004000` à `0x0800BFFF`)**, qui contiennent les
calibrations, réglages et labyrinthes, ne sont ni effacés ni programmés, y compris
avec `--all`. Le script n'écrit pas toute la flash : il écrit uniquement les
images sélectionnées et efface seulement les secteurs qui les contiennent.
L'effacement se fait toutefois par secteurs matériels, plus grands que les images.
L'ancien comportement par défaut transférait **2 Mio de dumps** ; le nouveau
transfère **48 Kio**. Ce gain concerne les lectures : l'effacement, la programmation
et le contrôle de démarrage prennent encore du temps.

Avant une mise à jour de l'application seule, le script vérifie que les vecteurs
du bootloader restent dans le secteur 0. Un firmware monolithique à `0x08000000`
est refusé avant écriture : utiliser alors `--all --release`, de préférence avec
`--full-backup` pour archiver aussi l'ancien firmware. Les secteurs de données
restent préservés ; cela ne restaure pas des calibrations déjà écrasées par un autre outil.
Avec un autre outil, utiliser `initial-install.hex` pour l'installation complète ;
ne pas programmer `application.ota.bin` à `0x08000000`.

Le contrôle de démarrage au repos reste actif dans tous les modes : absence de
défaut processeur, moteurs désactivés et, pour l'application, interruptions et
acquisitions capteurs actives. Il ne remplace pas la vérification de flash demandée
par `--verify`. Le rapport distingue explicitement programmation et vérification.

Pour lire la flash et extraire les calibrations, sans programmer :

```sh
scripts/flash/dump.sh                     # flash complète + rapport JSON
scripts/flash/dump.sh --calibration-only  # seulement les 32 Kio persistants
```

Les sauvegardes horodatées, images, journaux et rapports restent dans
`backups/flash-sessions/` (non versionné, conservé par `make clean`). Le dump
historique n'est jamais remplacé. En cas d'échec, consulter le journal avant de
débrancher le robot ; le firmware peut nécessiter une nouvelle programmation.

Prérequis supplémentaires pour flasher : OpenOCD. Variables facultatives :
`JOBS=4`, `SWD_KHZ=1000`,
`STLINK_SERIAL=<numéro>` est facultatif et ne peut désigner que la sonde associée.

## Association sonde / robot

**ZHONX II : ST-Link `51FF66064982565324552187`**, confirmée par l'utilisateur.
UID STM32 lu sur ce robot : **`003300373432471234373230`**.
Le script exige aussi cet UID : échanger les câbles entre robots doit faire
échouer la vérification, même si la bonne sonde USB est sélectionnée.

Après confirmation du raccordement physique, robot alimenté :

```sh
python3 scripts/flash/robot_guard.py associate zhonx2 --serial 51FF66064982565324552187
scripts/flash/flash.sh --check
scripts/flash/flash.sh --release --fast
```

La première commande lit l'UID sans commande de reset, halt ou écriture flash.
L'association est enregistrée localement dans
`${XDG_CONFIG_HOME:-$HOME/.config}/pacabot/robots.json`. `--check` vérifie la sonde
et le MCU sans compilation, reset ni flash. Pour consulter le registre :

```sh
python3 scripts/flash/robot_guard.py show
```

Tous les modes de flash (`--fast`, `--verify`, `--bootloader`, `--all`) et le dump
imposent le numéro de sonde et vérifient l'UID **dans chaque session OpenOCD avant
reset/effacement/écriture**. L'identité attendue est également archivée dans le
rapport de flash. Une association absente, un UID différent ou une surcharge
`STLINK_SERIAL` incompatible bloque l'opération. Il n'y a pas de sélection
implicite de la première sonde, même lorsqu'une seule est branchée.

L'association n'est jamais remplacée automatiquement : déplacer/remplacer une
sonde ou un MCU demande de vérifier le câblage puis de mettre à jour le registre
consciemment. Sur une autre machine, refaire l'association. Les commandes OpenOCD
ou st-flash lancées directement contournent ces scripts : les agents doivent
utiliser les chemins protégés, conformément à [AGENTS.md](AGENTS.md).

Le travail en cours dans `../ZHONX_III` a son propre contrôle sonde + UID,
avec `scripts/flash/probes.tsv` et `scripts/flash/flash.sh --check`.
Son association actuelle désigne la sonde `55FF68064967515341421587`.
Ne pas remplacer son script par celui de ZHONX II.

## Évolutions firmware

La branche `fw/remote-update-motion-nimes` ajoute le bootloader SPI, les sauvegardes
transactionnelles, le solveur Nîmes 9×9 et le pilotage asynchrone. Voir
[architecture, protocole, usage et limites des validations](docs/FIRMWARE.md).

L'interface utilise une carte avec icône par écran et des caractères de 16 pixels
pour l'action choisie. Haut/bas fait défiler ; droite ou appui central valide ;
gauche du joystick revient ; Escape peut arrêter les mouvements. Les menus de
sélection affichent directement leur action, sans titre de catégorie en haut.
Un ascenseur à droite indique la position dans la liste.
Les consignes propres à une action apparaissent dans un cartouche inversé ;
les menus ne répètent plus les commandes de navigation.

Après **30 secondes sans interaction au repos** dans les menus de sélection,
la bibliothèque ou l'édition des réglages, une veille affiche **ZHONX II** en
lettrage géométrique carré : chaque caractère se déplace et pivote indépendamment
pour former le nom, reste aligné un instant, puis se disperse. Le cycle de dix
secondes vise 30 images/seconde, avec interpolation progressive des mouvements,
sans titre, jauge ni pourcentage superposé. Le premier appui réveille l'écran sans agir ; la sélection et la valeur
en cours d'édition sont conservées. La veille n'est pas activée dans les procédures
de déplacement, les calibrations, les tests matériels ou l'attente du top départ.

- **Maze** : nouvelle exploration, bibliothèque, reprise ; runs proposés uniquement
  pour un apprentissage validé. Jusqu'à **8 labyrinthes**, avec aperçu du chemin
  optimal clignotant. La validation enregistre automatiquement le labyrinthe.
- **Calibration** : murs, rotation, angles gauche/droit, rapports. Les schémas
  montrent aussi la cellule derrière le poteau ; pointillés = mur facultatif.
- **Settings** : dimensions, vitesses et départ, une valeur illustrée par écran.
- **Hardware / Tests** : moteurs gauche/droit/ensemble, en avant ou en arrière
  (robot soulevé, 20 mm de roue à 20 mm/s par appui), six télémètres en direct
  (brut/filtré, carré plein = mur), beeper, LED, motifs écran, boutons et ADC batterie.
  Gauche ou Escape quitte et coupe les moteurs. Un appui maintenu ne répète pas
  le mouvement ; les sons sont limités à 120 ms et le réglage beeper est restauré.
- **Update** : entrée dans le bootloader.

Le pourcentage batterie reste visible dans les menus et les écrans de calibration,
sans pictogramme.
La jauge **LiPo 2S** utilise une tension filtrée et une courbe approximative de
charge au repos. Sans étalonnage ou si la mesure ADC est invalide, elle affiche
**--%**. L'écran **Hardware → Battery** précise la cause : absence
d'étalonnage, ADC trop bas/saturé, tension incohérente ou attente au repos. Pour l'étalonner : **Hardware → Battery → OK**, robot immobile, saisir la
tension du pack relevée au multimètre (pas la tension 3,3 V de la sonde), puis
valider. Attendre au moins cinq secondes au repos avant d'enregistrer.
La référence est sauvegardée avec les calibrations et conservée au flash normal.
Voir [jauge batterie : fonctionnement et limites](docs/BATTERY.md).

Les runs lent (120 mm/s) et rapide (vitesse réglable) sont disponibles. Le mode
courbes est explicitement indiqué indisponible : son contrôleur n'est pas implémenté.
La calibration latérale est désormais utilisée par le centrage pendant exploration
et runs ; les capteurs binaires donnent un intervalle possible, pas une distance
continue. Les offsets d'angle restent enregistrés et consultables, sans encore
déclencher les virages. Voir les [procédures et limites](docs/FIRMWARE.md).

Le mode `--all` produit aussi `bootloader.bin`, `application.ota.bin`,
`initial-install.hex` et les empreintes dans `firmware-manifest.json` sous le profil choisi.
`make test` exécute les simulations et les tests du protocole sans accéder à la carte.
