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

Le script fonctionne depuis n'importe quel dossier. Il accepte les mêmes options
que le script de compilation, avec en plus `--fast`. Sans option, il compile puis flashe uniquement
l'application et son manifeste. `--bootloader` n'écrit que le bootloader ; `--all`
écrit les trois images. Il sauvegarde les 1 Mio de Flash avant toute écriture,
puis compare la relecture complète à l'image attendue, y compris les secteurs conservés.
Avant une mise à jour de l'application seule, il vérifie que les vecteurs du
bootloader restent dans le secteur 0. Un ancien firmware monolithique installé
à `0x08000000` est refusé avant écriture : utiliser alors
`scripts/flash/flash.sh --all --release` pour installer le bootloader et l'application.
Avec un autre outil, utiliser le fichier `initial-install.hex` pour l'installation
complète ; ne pas programmer `application.ota.bin` à `0x08000000`.
Les réglages, la carte et la zone de téléchargement sont préservés.
Pour l'application, il redémarre ensuite le robot au menu et contrôle les
interruptions, les capteurs, l'absence de défaut processeur et la désactivation
des moteurs au repos. Pour le bootloader seul, il contrôle le redémarrage et les
registres de défaut processeur.

Pour les mises à jour courantes, le mode rapide garde les calibrations :

```sh
scripts/flash/flash.sh --release --fast
```

Il sauvegarde la zone basse contenant les secteurs à modifier et les deux secteurs
de données. Il vérifie les images sur la cible puis compare les **32 Kio de données**
avant/après. Avec l'application Release actuelle : environ **288 Kio de dumps**
contre **2 Mio** en mode complet. Ce ratio concerne les lectures ; le temps total
inclut encore compilation, effacement, programmation et démarrage. Le mode complet
reste le défaut et vérifie aussi les zones non concernées.

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
`STLINK_SERIAL=<numéro>` pour sélectionner une sonde précise.

## Évolutions firmware

La branche `fw/remote-update-motion-nimes` ajoute le bootloader SPI, les sauvegardes
transactionnelles, le solveur Nîmes 9×9 et le pilotage asynchrone. Voir
[architecture, protocole, usage et limites des validations](docs/FIRMWARE.md).

L'interface utilise une carte avec icône par écran et des caractères de 16 pixels
pour l'action choisie. Haut/bas fait défiler ; droite ou appui central valide ;
gauche du joystick revient ; Escape peut arrêter les mouvements.

- **Maze** : nouvelle exploration, bibliothèque, reprise ; runs proposés uniquement
  pour un apprentissage validé. Jusqu'à **8 labyrinthes**, avec aperçu du chemin
  optimal clignotant. La validation enregistre automatiquement le labyrinthe.
- **Calibration** : murs, rotation, angles gauche/droit, rapports. Les schémas
  montrent aussi la cellule derrière le poteau ; pointillés = mur facultatif.
- **Settings** : dimensions, vitesses et départ, une valeur illustrée par écran.
- **Update** : entrée dans le bootloader.

Les runs lent (120 mm/s) et rapide (vitesse réglable) sont disponibles. Le mode
courbes est explicitement indiqué indisponible : son contrôleur n'est pas implémenté.
La calibration latérale est désormais utilisée par le centrage pendant exploration
et runs ; les capteurs binaires donnent un intervalle possible, pas une distance
continue. Les offsets d'angle restent enregistrés et consultables, sans encore
déclencher les virages. Voir les [procédures et limites](docs/FIRMWARE.md).

Le mode `--all` produit aussi `bootloader.bin`, `application.ota.bin`,
`initial-install.hex` et les empreintes dans `firmware-manifest.json` sous le profil choisi.
`make test` exécute les simulations et les tests du protocole sans accéder à la carte.
