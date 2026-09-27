# Arrêt du 27 septembre 2026 — ZHONX II

## Capture avant modification

Commande : `scripts/flash/dump.sh --state`, sans reset ni écriture flash.
Sonde `51FF66064982565324552187`, UID `003300373432471234373230`.
Tension cible 3,227 V ; moteurs désactivés (`GPIOA_ODR=0x160`).

Dossier local : `backups/flash-sessions/20260927T164540Z-dump-BNdqLt/`.
Flash 1 Mio, SRAM 128 Kio, CCM 64 Kio, journal OpenOCD, ELF candidat,
rapport de calibration et `state.json` décodé depuis la SRAM.
L'application capturée correspond **octet pour octet** au binaire de 101 980
octets du commit `9901ee1` ; les symboles utilisés sont donc ceux de cette image.
SHA256 flash : `eba0ef07b4acb9f7d599cc017edf72cae56becf3b15fac9cc8c9a1001ea7c787`.

## Cause confirmée

- `fault=2`, `fw_last_stop_code=2` ; pointeurs de pile vers `EARLY OBSTACLE`.
- `CFSR=0`, `HFSR=0` : aucun HardFault signalé.
- Exploration arrêtée à 65 011 ms ; aucune passe de confirmation engagée.
- Déplacement droit en avant, quatre cellules : cible 715 995 µm,
  position estimée 701 528 µm, correction longitudinale −6 000 µm.
- Il restait donc 14 467 µm avant le centre attendu.
- F5 calibré à 91 848 µm de l'axe ; demi-largeur libre 83 500 µm.
  Son déclenchement nominal est 8 348 µm avant le centre.
- L'ancienne condition acceptait seulement 8 348 + 5 000 = 13 348 µm.
  L'écart de 1 119 µm a déclenché l'arrêt obstacle.

La dernière pose logique est (5,4), cap sud, au départ du segment de quatre
cellules vers (5,0). Le mur sud y est déjà connu et extérieur ; la récupération
réservée aux murs manqués refuse de réécrire ce mur. L'exploration s'arrête.
La pose logique de fin d'erreur n'est pas une mesure de la position physique finale.

Le dump ne contient pas l'historique complet des mouvements. Il confirme
néanmoins qu'un recalage précédent avait sondé F5 en recul (`centre_direction=-1`,
front à −10 634 µm), puis réussi (`centre_ok=1`). Les manœuvres systématiques
sont présentes dans le code ; on ne peut pas attribuer tout le recul observé
à une séquence précise sur la seule capture finale.

## Carte et sauvegarde

Les deux banques flash ont un CRC valide (séquences 38 et 37). La banque 38
contient exactement la carte active de la SRAM : 6×6, 27 cellules visitées,
arrivée 2×2 reconnue en (4,4), (5,4), (4,5), (5,5). `has_map=1`, `learned=0`.
La bibliothèque contient une carte précédemment apprise.

Le chemin connu vers l'arrivée fait 23 déplacements (coût 23 560), mais un
chemin optimiste utilisant des passages inconnus en ferait 15 (coût 15 366).
L'apprentissage n'est donc pas certifié. La carte est sauvegardée comme
apprentissage en cours, accessible par **Maze → Resume / Learning**, après
repositionnement au départ dans son orientation initiale. Elle n'est pas ajoutée
comme nouveau labyrinthe appris à la bibliothèque.

## Corrections

- Suppression des rotations/recherches F5 systématiques au départ et aux arrêts.
  Les corrections latérales, de cap et longitudinales restent exécutées pendant
  le déplacement à partir des calibrations des murs et des poteaux.
- Arrivée frontale : fenêtre de cohérence ±15 mm, comme les autres références ;
  une transition F5 crédible fixe la distance restante mesurée jusqu'au centre,
  sans recul. Un obstacle hors fenêtre ou dans un passage connu reste bloquant.
- Avec les deux calibrations de portes valides, exploration limitée à leur
  domaine mesuré de 220 mm/s. Le réglage était 300 mm/s, ce qui désactivait
  l'anticipation. Les runs gardent leur vitesse réglée.
- `dump.sh --state` conserve les données volatiles sans reset, refuse un robot
  dont les moteurs sont actifs et reprend le CPU après capture ou erreur de lecture.
  Les registres sont journalisés avec le retour Tcl de
  [get_reg (OpenOCD)](https://openocd.org/doc/html/General-Commands.html).

## Vérifications

Suite `make test` avec ASan/UBSan et 32 tests Python réussie ; compilation
Release `-O3` réussie. Régression du seuil frontal reproduite en simulation,
arrivée vérifiée sans inversion moteur ; obstacles hors fenêtre conservés.
Navigation testée sans appel de recalage à l'arrêt, avec enchaînement droit,
récupération de mur et trois passes de confirmation. L'exploration demandée à
300 mm/s utilise bien 220 mm/s et l'anticipation.
La tenue physique dans le labyrinthe reste à vérifier par un nouvel essai.
