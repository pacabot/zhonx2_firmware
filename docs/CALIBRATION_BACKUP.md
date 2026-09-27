# Sauvegarder et restaurer les calibrations

Robot alimenté, moteurs arrêtés, sonde associée à ZHONX II. Python 3 et OpenOCD
sont requis. La sonde et l’UID STM32 sont vérifiés dans la session d’accès,
avant toute écriture. Une archive d’un autre robot est refusée.

```sh
scripts/flash/calibration.sh backup
scripts/flash/calibration.sh backup backups/calibrations/avant-modification.json
scripts/flash/calibration.sh inspect backups/calibrations/avant-modification.json
scripts/flash/calibration.sh restore backups/calibrations/avant-modification.json
scripts/flash/calibration.sh --help
```

`backup` lit uniquement les 32 Kio de données persistantes. Il suspend brièvement
le processeur, puis reprend son exécution sans reset. Sans chemin, l’archive JSON
horodatée est créée dans `backups/calibrations/`. Un fichier existant n’est jamais
remplacé. Conserver une copie de l’archive sur un autre support pour la protéger
contre la perte du poste de travail. `inspect` fonctionne sans robot branché.

L’archive contient les deux banques brutes, leur empreinte SHA-256, l’identité du
robot et un rapport décodé. Une sauvegarde reste possible en l’absence de
calibrations : le rapport le signale. Les CRC vérifient l’intégrité des données,
pas leur précision physique.

## Ce qui est restauré

- Calibration murs et capteurs, rotation aux trois vitesses, angles gauche/droit.
- Dimensions et position du poteau associées à ces mesures.
- Référence ADC/tension batterie, si le format la contient.
- Les absences de calibration restent des absences ; aucune mesure n’est inventée.

Les réglages de vitesse, le départ, le labyrinthe actif et la bibliothèque de
labyrinthes restent ceux du robot au moment de la restauration. Le firmware et
le bootloader sont conservés.

La restauration accepte les snapshots **v5 et v6**, après contrôle des CRC et des
bornes de validité du firmware. Elle exige un snapshot actuel valide pour pouvoir
conserver les réglages et labyrinthes. Elle conserve le format du snapshot cible :
une référence batterie v6 non nulle ne peut pas être injectée dans une cible v5.
Dans ce cas, démarrer le firmware actuel et sauvegarder ses réglages pour créer
un snapshot v6, puis relancer la restauration. Une source v5 laisse intacte la
référence batterie d’une cible v6.

## Écriture transactionnelle

L’état actuel est d’abord sauvegardé dans
`backups/flash-sessions/<session>/before.calibration.json`. La lecture, la fusion
et l’écriture ont lieu pendant le même arrêt du processeur, pour empêcher une
sauvegarde concurrente du firmware.

Seul le secteur de données inactif (16 Kio, secteur 1 ou 2) est effacé. Environ
4,7 Kio sont programmés. Le contenu est vérifié avant l’écriture du marqueur de
validation, puis les deux banques sont relues : la banque précédente doit être
inchangée. Le robot redémarre ensuite pour charger les calibrations restaurées.
Cette vérification fait partie de la restauration et ne se désactive pas.

Les journaux et la sauvegarde antérieure restent dans le dossier de session en
cas d’erreur. Une interruption avant le marqueur de validation laisse la banque
précédente disponible. Pour une simple mise à jour du firmware, aucune restauration
n’est nécessaire : `scripts/flash/flash.sh --release --fast` préserve ces secteurs.

## Lecture des rapports sur le robot

Les écrans actifs utilisent des caractères d’au moins **7 × 12 pixels** ; les
cartes du menu conservent leurs caractères de 16 pixels de haut. Les anciennes
polices 3 × 6 et 5 × 8 ne sont plus référencées dans le binaire de l’application.

Dans un rapport : **haut/bas** change de page, **OK** avance et quitte à la fin,
**gauche/Escape** quitte à tout moment. L’ascenseur indique la position.

`Edge report` commence par l’état des **deux montages** : `SAVED` ou `MISSING`.
`INCOMPLETE` indique qu’au moins un montage manque. Les pages détaillées indiquent
`Left setup` / `Right setup`, puis le capteur physique (`L5`, `R10`, etc.), le sens
et la vitesse. Après rotation, un montage peut mesurer le capteur du côté opposé :
cela ne signifie pas que l’autre montage a été calibré. Un canal non mesuré est
indiqué `NOT MEASURED`. Dans `Turn report`, l’absence de contrôle indépendant à
90° est également signalée par `MISSING`.
