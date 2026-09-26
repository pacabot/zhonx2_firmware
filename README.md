# ZHONX II M4

## Compilation

Pré-requis : GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`), GNU Make et Python 3.

```sh
make clean
make -j4
```

Les fichiers `build/ZHONX_II_M4.elf`, `.bin`, `.hex` et `.map` sont produits sans programmer la carte. Le `Makefile` reprend la liste des sources du projet Xcode historique et ajoute les modules nécessaires au `main.c` actuel.

La cible est un STM32F405RG avec quartz externe à 8 MHz. Le script `pacabot_link.ld` place la table des vecteurs de l’application à `0x08010000` ; le bootloader occupe `0x08000000`. Le fichier `cmsis_boot/system_stm32f4xx.c` reste la configuration d'horloge spécifique à cette carte.

## Bibliothèques

Les fichiers utilisés de la STM32F4 Standard Peripheral Library et de CMSIS ont été mis à jour depuis le paquet STSW-STM32065 **V1.9.0** (les en-têtes du paquet indiquent SPL 1.8.x et CMSIS 4.00). Source de référence : [fiche ST](https://www.st.com/en/embedded-software/stsw-stm32065.html). Copie de travail : [miroir GitHub](https://github.com/synapticonashaikh/STM32F446RE-Sample-codes/tree/684f3c374d5a92ed025aca768537a3a8df66463b/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries), révision `684f3c374d5a92ed025aca768537a3a8df66463b`.

## Sauvegarde de la carte

La copie complète du firmware d'origine et ses informations de vérification sont dans `backups/`. Les corrections sont documentées dans `docs/FIRMWARE.md`.

## Compiler et flasher par ST-Link

Robot alimenté et sonde branchée (les anciennes ST-Link/V2 sont prises en charge) :

```sh
.scripts/build/build.sh
```

Le script fonctionne depuis n'importe quel dossier. Il compile, sauvegarde les
1 Mio de Flash, écrit le bootloader, le manifeste et l'application, puis compare
la relecture complète à l'image attendue, y compris les secteurs conservés.
Les réglages, la carte et la zone de téléchargement sont préservés.
Il redémarre ensuite le robot au menu et contrôle les interruptions, les capteurs,
l'absence de défaut processeur et la désactivation des moteurs au repos.

Les sauvegardes horodatées, images, journaux et rapports restent dans
`backups/flash-sessions/` (non versionné, conservé par `make clean`). Le dump
historique n'est jamais remplacé. En cas d'échec, consulter le journal avant de
débrancher le robot ; le firmware peut nécessiter une nouvelle programmation.

Options : `--build-only` pour compiler sans accès matériel, `--test` pour exécuter
aussi les tests, `--clean` pour recompiler entièrement. Prérequis supplémentaires
pour flasher : OpenOCD. Variables facultatives : `JOBS=4`, `SWD_KHZ=1000`,
`STLINK_SERIAL=<numéro>` pour sélectionner une sonde précise.

## Évolutions firmware

La branche `fw/remote-update-motion-nimes` ajoute le bootloader SPI, les sauvegardes
transactionnelles, le solveur Nîmes 9×9 et le pilotage asynchrone. Voir
[architecture, protocole, usage et limites des validations](docs/FIRMWARE.md).

`make` produit aussi `build/bootloader.bin`, `build/application.ota.bin`,
`build/initial-install.hex` et les empreintes dans `build/firmware-manifest.json`.
`make test` exécute les simulations et les tests du protocole sans accéder à la carte.
