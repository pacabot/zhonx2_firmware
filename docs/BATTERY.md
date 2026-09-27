# Jauge batterie LiPo 2S

## Étalonnage facultatif

La mesure passe par le pont diviseur de la carte, PA4 / ADC1 canal 4.
Sans étalonnage manuel, la conversion utilise R3 = 10 kΩ / R2 = 6,8 kΩ du schéma
et la tension VDDA mesurée par VREFINT. Une référence au multimètre reste préférable
pour corriger les tolérances du pont ou un changement de résistances.

1. Robot immobile, relever la tension du pack au multimètre.
2. Ouvrir **Hardware → Battery**. Vérifier que l'ADC n'est ni nul ni saturé.
3. Appuyer sur le joystick pour ouvrir **METER VOLTAGE / 2S**.
4. Haut/bas règle la tension par pas de 0,01 V. Saisir la valeur mesurée,
   même si elle diffère des 8,40 V proposés initialement.
5. Après au moins cinq secondes au repos, valider ; gauche/Escape annule.

La conversion est `tension = ADC filtré × tension de référence / ADC de référence`.
Il s'agit d'un étalonnage de gain à un point, supposant un pont linéaire sans offset.
Une seconde mesure à une autre tension permet de vérifier cette hypothèse.
La tension cible de la ST-Link (~3,3 V) n'est pas celle du pack.

## Pourquoi le pourcentage affiche `--%`

Dans **Hardware → Battery**, lire le diagnostic :

- Référence manuelle absente : estimation nominale disponible, sans blocage de la jauge.
- ADC saturé (4090 à 4095) : affichage d’une borne, par exemple `85%+` ;
  la tension réelle peut être supérieure à la plage mesurable.
- `ADC INPUT LOW` : entrée proche de zéro (moins de 16), vérifier PA4 et son alimentation.
- `NEEDS METER REF` : conversion nominale hors plage plausible ; vérifier la
  tension réelle, le câblage et les résistances, puis enregistrer la référence avec OK.
- `Wait 5s at rest` : attendre une mesure au repos après le démarrage ou un mouvement.

Le schéma communiqué utilise R3 = 10 kΩ et R2 = 6,8 kΩ : à 8,4 V, PA4 reçoit
théoriquement 3,40 V. Avec une référence ADC à 3,3 V, la mesure sature en haut de
charge. R3 = 12 kΩ donnerait 3,04 V à 8,4 V. Le firmware ne suppose pas que cette
modification a été réalisée ; après changement de résistance, refaire l'étalonnage.

## Estimation de charge

- Destinée aux LiPo 2S classiques chargées à 4,20 V par cellule.
- Filtre de tension d'environ quatre secondes, puis estimation après cinq secondes
  sans mouvement détecté. Ce délai réduit les transitoires ; il ne garantit pas
  une relaxation électrochimique complète.
- Pendant les mouvements, conservation du dernier pourcentage au repos ; après
  le démarrage, `--%` jusqu'à la première estimation valide.
- Courbe indicative : 8,40 V → 100 %, 8,00 V → 80 %, 7,80 V → 70 %,
  7,60 V → 40 %, 7,40 V → 15 %, 7,00 V → 5 %, 6,60 V → 0 %.
  Interpolation entre points et hystérésis d'affichage de deux points de pourcentage.
- L'écran Battery indique la tension convertie et `SOC: ~…%`. Les menus,
  et diagnostics affichent uniquement le pourcentage en haut à droite, sans
  pictogramme batterie. La veille réserve tout l’écran à l’animation ; la mesure
  continue en arrière-plan sans superposition.
- ADC absent, référence interne invalide ou conversion incohérente : `--%`,
  jamais interprété comme une batterie vide. Une saturation cohérente affiche
  une borne de pourcentage avec `+`, et non un 100 % supposé.

Cette courbe n'est pas une caractérisation du pack du robot. Température,
vieillissement, charge électrique et déséquilibre des cellules influencent le
résultat. Aucun courant ni aucune tension individuelle de cellule n'est mesuré.
La jauge n'introduit pas de coupure moteur et ne remplace pas une protection du pack.
Une tension haute n'est pas interprétée comme une détection du chargeur.

La référence est conservée dans le format de sauvegarde 7. Le firmware lit aussi les
formats 1 à 6 : calibrations, réglages et labyrinthes restent disponibles ; une
ancienne sauvegarde n'a simplement pas encore de référence batterie.
`scripts/flash/flash.sh --release` préserve les secteurs de données ; l'étalonnage
y reste enregistré. Un ancien firmware ne sait pas relire le format 7.
`tools/calibration_dump.py` expose la référence sous `battery_reference`.

## Corrections de l'acquisition

L'horloge GPIO est activée avant la configuration de PA4 en entrée analogique.
L'horloge ADC est divisée par quatre (21 MHz avec APB2 à 84 MHz), au lieu de deux.
La structure d'initialisation ADC est entièrement initialisée avant configuration.
Le DMA alterne PA4 (rang 1) et VREFINT (rang 2), avec 480 cycles d'échantillonnage.
VDDA = 3300 × VREFINT_CAL / ADC_VREFINT, avec la constante d'usine à 0x1FFF7A2A
([STM32F405, DS8626, table 73](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)).

## Références et validation

- [ST, STM32F405/407 : caractéristiques ADC](https://www.st.com/resource/en/datasheet/stm32f405vg.pdf),
  fréquence ADC maximale de 36 MHz.
- [TI, limites d'une estimation par tension](https://www.ti.com/document-viewer/lit/html/SSZT786/GUID-5DD8C3D9-BCBE-467B-90E5-FAA672AAC15D).
  La courbe du firmware est indicative, pas une courbe de pack fournie par TI.

Les tests hôte couvrent la conversion, la courbe, le filtrage, l'attente au repos,
les mesures invalides, le débordement de l'horloge et la persistance/migration.
Les rendus OLED sont générés avec le pilote graphique réel. La justesse de la
mesure sur le robot reste à vérifier au multimètre après flash.

Le diagnostic `scripts/flash/dump.sh --diagnostics` lit les registres GPIO/ADC/DMA
et quatre paires ADC, sans reset ni écriture en flash, après contrôle sonde/UID.
Une conversion nominale incohérente ne bloque pas la saisie d’une référence au
multimètre : le signal ADC valide et stable reste utilisable pour cet étalonnage.


## Mesures du 27 septembre : isoler le pont et l'entrée PA4

Le multimètre indique **7,92 V au pack et 3,20 V au pont**, cohérent avec
10 kΩ / 6,8 kΩ (valeur calculée 3,206 V). Cependant le canal ADC4 fournit
1475–1486 points, soit 1,17–1,19 V avec VDDA calculé par VREFINT.
Une mesure à 3,20 V sur PA4 devrait donner environ 4010–4030 points.

`scripts/flash/dump.sh --adc-test` vérifie cela par des conversions injectées
PA4 / VREFINT, lues directement dans JDR1, indépendamment du DMA. Les mesures
concordent avec le DMA. Le script exige les moteurs arrêtés, suspend le CPU,
restaure le séquenceur injecté, reprend le CPU et ne touche pas à la flash.
Référence : [ST RM0090, séquences injectées ADC](https://www.st.com.cn/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).

Journal : `backups/flash-sessions/20260927T131448Z-dump-8dhXR8/read.log`.
Le prochain contrôle est la tension directement sur PA4, puis, robot et sonde
hors tension, la continuité entre le pont et PA4. Pour le STM32F405RG en LQFP64,
PA4 est la patte 20 ([ST DS8626, tableau 7](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)).
La mesure au pont ne prouve pas à elle seule que cette tension arrive à la broche.
Aucun facteur de gain artificiel n'a été enregistré pour masquer cet écart.
