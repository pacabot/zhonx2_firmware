# Jauge batterie LiPo 2S

## Étalonnage

La mesure passe par le pont diviseur de la carte, PA4 / ADC1 canal 4.
Les anciens seuils ADC ne suffisent pas à connaître son rapport réel.
La jauge affiche donc `--%` dans les menus tant qu'une référence n'a pas été enregistrée.

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

- `NOT CALIBRATED` : enregistrer une tension mesurée au multimètre avec OK.
- `ADC SATURATED` : entrée proche du maximum (4090 à 4095), vérifier le diviseur.
- `ADC INPUT LOW` : entrée proche de zéro (moins de 16), vérifier PA4 et son alimentation.
- `VOLTAGE OUT OF RANGE` : conversion hors plage plausible, vérifier l'étalonnage.
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
  diagnostics et la veille affichent uniquement le pourcentage en haut à droite,
  sans pictogramme batterie.
- ADC absent ou proche de la saturation : `--%`,
  jamais interprété comme une batterie vide.

Cette courbe n'est pas une caractérisation du pack du robot. Température,
vieillissement, charge électrique et déséquilibre des cellules influencent le
résultat. Aucun courant ni aucune tension individuelle de cellule n'est mesuré.
La jauge n'introduit pas de coupure moteur et ne remplace pas une protection du pack.
Une tension haute n'est pas interprétée comme une détection du chargeur.

La référence est ajoutée au format de sauvegarde 6. Le firmware lit aussi les
formats 1 à 5 : calibrations, réglages et labyrinthes restent disponibles ; une
ancienne sauvegarde n'a simplement pas encore de référence batterie.
`scripts/flash/flash.sh --release` préserve les secteurs de données ; l'étalonnage
y reste enregistré. Un ancien firmware ne sait pas relire le format 6.
`tools/calibration_dump.py` expose la référence sous `battery_reference`.

## Corrections de l'acquisition

L'horloge GPIO est activée avant la configuration de PA4 en entrée analogique.
L'horloge ADC est divisée par quatre (21 MHz avec APB2 à 84 MHz), au lieu de deux.
La structure d'initialisation ADC est entièrement initialisée avant configuration.

## Références et validation

- [ST, STM32F405/407 : caractéristiques ADC](https://www.st.com/resource/en/datasheet/stm32f405vg.pdf),
  fréquence ADC maximale de 36 MHz.
- [TI, limites d'une estimation par tension](https://www.ti.com/document-viewer/lit/html/SSZT786/GUID-5DD8C3D9-BCBE-467B-90E5-FAA672AAC15D).
  La courbe du firmware est indicative, pas une courbe de pack fournie par TI.

Les tests hôte couvrent la conversion, la courbe, le filtrage, l'attente au repos,
les mesures invalides, le débordement de l'horloge et la persistance/migration.
Les rendus OLED sont générés avec le pilote graphique réel. La justesse de la
mesure sur le robot reste à vérifier au multimètre après flash.
