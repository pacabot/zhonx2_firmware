# Sauvegarde du firmware présent sur la carte

- Date : 2026-09-26
- Fichier : `firmware_STM32F4x5_2026-09-26.bin`
- Lecture : `st-flash 1.8.0`, ST-Link `V2J16S4`, numéro de série `51FF66064982565324552187`
- Cible détectée : STM32F4x5/F4x7, ID `0x413`, Flash 1 048 576 octets
- Plage lue : `0x08000000` à `0x080FFFFF` (Flash intégrale)
- SHA-256 : `4bf8065aec9c86108f080970f5a9c351f00d9cdd89e81cf2c0a1fc3d8f640212`
- Vérification : deuxième lecture de la même plage, comparaison octet par octet identique.

Cette sauvegarde inclut les secteurs de paramètres stockés en Flash interne. Elle ne contient ni la RAM, ni les option bytes, ni une éventuelle mémoire externe. Aucune écriture sur la cible n'a été effectuée.
