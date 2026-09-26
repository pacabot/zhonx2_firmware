# ZHONX II M4

## Compilation

Pré-requis : GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`), GNU Make et Python 3.

```sh
make clean
make -j4
```

Les fichiers `build/ZHONX_II_M4.elf`, `.bin`, `.hex` et `.map` sont produits sans programmer la carte. Le `Makefile` reprend la liste des sources du projet Xcode historique et ajoute les modules nécessaires au `main.c` actuel.

La cible est un STM32F405RG avec quartz externe à 8 MHz. Le script `pacabot_link.ld` place la table des vecteurs à `0x08000000`. Le fichier `cmsis_boot/system_stm32f4xx.c` reste la configuration d'horloge spécifique à cette carte.

## Bibliothèques

Les fichiers utilisés de la STM32F4 Standard Peripheral Library et de CMSIS ont été mis à jour depuis le paquet STSW-STM32065 **V1.9.0** (les en-têtes du paquet indiquent SPL 1.8.x et CMSIS 4.00). Source de référence : [fiche ST](https://www.st.com/en/embedded-software/stsw-stm32065.html). Copie de travail : [miroir GitHub](https://github.com/synapticonashaikh/STM32F446RE-Sample-codes/tree/684f3c374d5a92ed025aca768537a3a8df66463b/STM32F4xx_DSP_StdPeriph_Lib_V1.9.0/Libraries), révision `684f3c374d5a92ed025aca768537a3a8df66463b`.

## Sauvegarde de la carte

La copie complète de la Flash et ses informations de vérification sont dans `backups/`. Le nouveau binaire n'a pas été écrit sur la carte. La compilation seule ne prouve pas le fonctionnement sur le robot ; les paramètres persistants et le comportement des moteurs devront être contrôlés avant toute mise à jour matérielle.
