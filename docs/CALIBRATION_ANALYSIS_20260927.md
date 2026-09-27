# Lecture des calibrations du 27 septembre 2026

Source : `backups/calibrations/20260927T120319Z-cal-colyx4na.json`.
Sonde et UID ZHONX II vérifiés avant lecture. Aucun mouvement commandé.

## Batterie

Référence enregistrée : ADC = 0, tension = 0. Le firmware ne dispose donc
pas d'une référence de conversion et affiche `--%`. Ce n'est pas une indication
que la batterie est vide. Une tension réellement mesurée permet de calibrer le gain.

## Murs

Géométrie : nez 47 mm, largeur 94 mm, intérieur 167 mm, pas 179 mm.
Distances référencées au centre des roues :

| Capteur | Déclenchement | Relâchement | Dispersion |
|---|---:|---:|---:|
| Avant 5 cm | 91,848 mm | 92,415 mm | 0,408 mm |
| Avant 10 cm | 131,991 mm | 132,353 mm | 0,192 mm |
| Gauche 5 cm | intervalle 84,5–85,5 mm | | 0 mm |
| Droit 5 cm | intervalle 81,5–83,5 mm | | 1 mm |

Le centre théorique est à 83,5 mm des murs. Le droit 5 cm est donc à sa limite de
portée au centre. Les tolérances de placement, de rotation et de surface peuvent
changer son état. Le gauche devrait détecter dans les conditions de la calibration
mur ; son absence en calibration d'angle indique que ces conditions n'ont pas été
retrouvées. Le dump ne contient pas les états instantanés permettant d'isoler
surface, pose ou bruit.

## Rotation

À 40/80/120 mm/s : voie effective environ 83,42–83,52 mm. Six contrôles à 90°
par sens et vitesse. Erreurs optiques enregistrées : 0,322 à 1,235 degré.
Dispersion des références : au plus 0,600 mm. Ces valeurs passent les bornes du
firmware ; le comptage des pas n'est pas une mesure directe d'un dérapage.

## Angles

Les deux montages sont présents. Tous les profils ont `mask = 2` : canal 10 cm
présent, canal 5 cm absent, aux trois vitesses et dans les deux orientations.
L'ancien algorithme décidait ce masque à partir de l'état initial du capteur 5 cm
au centre. Un capteur initialement inactif était exclu du parcours, sans tentative
supplémentaire. `NOT MEASURED` ne signifie donc pas que trois mesures ont échoué.

Offsets filtrés par rapport au poteau :

| Montage / capteur / sens d'ouverture | 40 mm/s | 120 mm/s | 220 mm/s |
|---|---:|---:|---:|
| Gauche / L10 / arrière | +45,629 | +47,897 | +50,634 |
| Gauche / R10 / avant | −30,119 | −27,714 | −25,706 |
| Droit / R10 / arrière | +50,394 | +52,602 | +54,870 |
| Droit / L10 / avant | −26,682 | −24,210 | −22,206 |

Dispersion maximale : 2,208 mm. Le retard brut→filtré est d'environ 20 ms,
soit 0,8 / 2,4 / 4,4 mm aux vitesses mesurées. Il faut utiliser le profil du
capteur, du sens et de la vitesse réellement observés, et distinguer ouverture
et fermeture. Les offsets 5 cm absents ne doivent jamais être remplacés par zéro.
