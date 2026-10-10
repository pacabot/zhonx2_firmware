# Ariane Link sur ZHONX II (module BLE HM-18)

Ce firmware parle le protocole Ariane Link (`ariane-tools/link/PROTOCOL.md`, schéma v1,
CRC `0x2CB70B58`) avec le module **DSD TECH HM-18** (CC2640R2F, commandes HM-10).
Le robot se présente comme `robot = 2` (`ZHONX2`), firmware `Zhonx2 ariane`, et suit les
conventions de traces et de commandes de ZHONX III.

État au 2 octobre 2026 : compilé (`-Wall -Wextra -Werror`), tests hôte verts.
**Rien n'a encore tourné sur le robot** : ni le dialogue AT avec le module, ni le DMA de
l'USART1, ni une session Ariane. La liste des essais est à la fin.

## Câblage

Le seul UART sorti sur la carte mère est le connecteur **K1** (3 broches, sans alimentation) :

| K1 | Signal STM32 | Module HM-18 |
|---:|---|---|
| 1 | GND | GND |
| 2 | PA10 = USART1 RX (via R33 510 Ω) | TXD |
| 3 | PA9 = USART1 TX (via R32 510 Ω) | RXD |

L'alimentation du module se prend ailleurs : P2 (JTAG) broches 1/2 ou P5 broche 12 (+3,3 V).
Les cartes DSD TECH ont leur propre régulateur : vérifier la plage annoncée pour la broche
VCC. Les broches STATE et BRK ne sont pas utilisées : la connexion est connue par les
notifications `OK+CONN` / `OK+LOST` du module.

## Configuration automatique du module

Au démarrage (`fw_hm18.c`, sans blocage du menu) :

1. le firmware cherche la vitesse du module : 115 200 bauds, puis 9 600 (réglage d'usine),
   57 600, 38 400 et 19 200. Chaque commande part d'abord sans terminateur, puis avec CR LF ;
2. si le module répond à une autre vitesse : `AT+BAUD4`, puis `AT+RESET`, puis passage à
   115 200 bauds ;
3. `AT+NAME?`, puis `AT+NAMEZhonx-2` si besoin ; `AT+NOTI?`, puis `AT+NOTI1` si besoin ;
   `AT+ADDR?` ; un dernier `AT+RESET` si un réglage a changé.

La première fois, cela prend environ 4 s ; ensuite, moins d'une seconde, et rien n'est réécrit.
Sans réponse, le module est déclaré absent et recherché de nouveau toutes les 10 s.
Si le module refuse la nouvelle vitesse, le lien fonctionne quand même à sa vitesse d'origine,
avec moins de débit.

**Hardware tests > Bluetooth** affiche l'état (`probe`, `ready`, `absent`…), la connexion
(`NOT CONNECTED`, `CONNECTED`, `ARIANE SESSION`) et l'adresse BLE. OK passe aux compteurs :
octets reçus et envoyés, trames, pertes, erreurs CRC.
Dans la bannière, une rune Bluetooth apparaît à gauche du pourcentage de batterie quand un
client est connecté ; elle est inversée pendant une session Ariane. Un arpège montant salue
chaque connexion, un arpège descendant chaque déconnexion.

Point important du HM-10/HM-18 : **non connecté, le module interprète comme commande AT
tout ce qu'il reçoit**. Le firmware n'émet donc rien tant qu'aucun client n'est connecté.
Autre particularité du module : un `AT` isolé coupe la connexion en cours.
Après un `REBOOT` à distance, la recherche de vitesse du démarrage déconnecte donc le client ;
l'application se reconnecte.

## Architecture

| Fichier | Rôle |
|---|---|
| `firmware/link/` + `firmware/include/middleware/link/` | Copies conformes : tramage et schéma (`ariane-tools/link`), anneaux, signal lumineux, paramètres, ligne de console (ZHONX III). `scripts/link/sync.sh` les met à jour, **ne jamais les modifier à la main**. |
| `firmware/include/middleware/link/link_port.h` | Propre à ZHONX II : verrou PRIMASK de la SPL, horloges. |
| `firmware/core/fw_hm18.c` | Automate AT du module et détection `OK+CONN` / `OK+LOST` (testé sur hôte). |
| `firmware/platform/fw_bt.c` | USART1, réception DMA2 flux 2 circulaire (1 Kio), émission DMA2 flux 7 par blocs de 64 octets, budget d'émission. |
| `firmware/platform/fw_link.c` | Le lien : commandes, télémétrie, événements, console, signal lumineux, exécution des ordres. |
| `firmware/platform/fw_link_params.c` | Paramètres exposés à l'application. |

ZHONX II n'a pas de boucle principale : chaque écran est une boucle bloquante. Le lien tourne
donc en **interruption PendSV** (priorité la plus basse), déclenchée par SysTick à chaque
milliseconde ; il répond pendant les menus, l'exploration, les courses et les écritures
flash. La télémétrie est échantillonnée dans SysTick, après le contrôleur de mouvement.
Le temps robot `t_us` vient de SysTick (millisecondes + compteur interne), sans timer dédié.
L'anneau de télémétrie occupe 38 Kio de CCM (section `.ccm_noinit`, hors image flash) :
1024 enregistrements, soit 51 s à 20 Hz.

Le module n'a pas de contrôle de flux et perd en silence ce que le BLE ne peut pas porter.
L'émission est donc plafonnée par un budget (`bt_tx_bps`, 4000 octets/s par défaut) ;
la télémétrie part par lots de 10 enregistrements, au plus tard toutes les 100 ms.
Les pertes éventuelles restent rattrapables par `RESEND` tant que l'anneau les contient.

## Mêmes traces et mêmes commandes que ZHONX III

Le firmware suit les conventions fixées par ZHONX III (`ariane-tools/link/PROTOCOL.md`,
« Conventions communes ») : l'application et les outils Mac lisent une session Zhonx2 comme
une session Zhonx3, et `config/robots/zhonx2.json` porte les mêmes `gyro_sign` (−1),
`cell_y_sign` (−1) et `cell_event_at` (`entry`) que `zhonx3.json`.

### Repère

`nimes.c` a le nord en +y et un cap de départ absolu (coin et orientation choisis au menu).
`fw_ariane.c` convertit vers le repère de ZHONX III : origine à la cellule de départ, **nord =
cap du robot au départ**, x vers l'est, **y vers le sud**. Les murs N1 E2 S4 W8 tournent avec.

### Commandes

| Commande | ZHONX II |
|---|---|
| `STOP` | Toujours `DONE`, `EVENT STOP` raison 1. Arme `fw_link_stop_flag` : chaque point d'arrêt `fw_cancel_pressed()` (exploration, courses, tests, attente de la main) arrête le robot à la milliseconde suivante, comme Échap. Le drapeau s'efface à la prochaine touche ou au prochain ordre. |
| `MOVE CELLS n` | `fw_motion_straight_to(n, vitesse, 1)` ; vitesse 0 = `scan_max_mm_s`. `START_CELL` : une cellule (ZHONX II part du centre de la case), annoncée `START_CELL`. |
| `TURN` | Sur place ±90° et 180°, roues à la vitesse demandée (0 = `scan_max_mm_s`) bornée à 120–300 mm/s, vitesse annoncée dans `MOVE_BEGIN` ; courbes ±90° (`fw_motion_curve`, calibrations requises) ; `UTURN` : demi-tour sur place, annoncé `UTURN`. |
| `EXPLORE` | Nouvelle exploration ; `CMD_DONE DONE` seulement si le labyrinthe est prouvé. |
| `RUN 1/2` | `fw_app_trial(1 ou 2)` : course 1 en lignes droites, course 2 en courbes, puis retour au départ. Vitesse 0 = `run1_max_mm_s` (vitesse d'exploration tant qu'il n'est pas réglé) ou `run2_max_mm_s` (réglage Fast run). Retour au départ à `scan_max_mm_s`. |
| `LED_SYNC` | Séquence de départ ou bip : LED rouge et orange, plus l'écran tout allumé (0xA5) quand le menu est libre. Pendant une activité : LED seules. |
| `PARAM_LIST/GET/SET` | Refusées (`BUSY`) hors du menu, comme ZHONX III. |
| `PARAM_SAVE` | En plus de ZHONX III : sauvegarde flash des paramètres `PERSISTENT`, puis `EVENT PARAMS_SAVED`. |
| `REBOOT` | `STARTED`, puis redémarrage après 300 ms, sans `CMD_DONE`. |
| `CONSOLE` | `help`, `status` (mêmes lignes que ZHONX III), `bt`, `version`, `beep` ; `BUSY` pendant une activité. Les réponses partent en `LOG` niveau `DEBUG`. |
| `TELEM_CFG`, `FLUSH`, `DUMP` | Comme ZHONX III. |
| `CALIB`, `HALT`, `BOOTLOADER` | `UNSUPPORTED` : calibrations guidées au menu, pas d'interrupteur commandé, chargeur en SPI3. |

Les ordres longs ne démarrent que si le robot attend une touche dans un menu ; sinon `BUSY`.
Les indicateurs `CmdFlags` sont respectés : séquence lumineuse avant le départ, attente de la
main (capteur F10), bips pendant le mouvement.

Paramètres (même identifiant et même nom que ZHONX III quand le sens est le même) :

| Id | Nom | Bornes | Persistant |
|---:|---|---|---|
| 1 | `telemetry_hz` | 0..100 | non |
| 2 | `blip_period_ms` | 100..60000 | non |
| 4 | `bt_tx_bps` (ZHONX II) | 500..11000 | non |
| 10 | `scan_max_mm_s` (Explore) | 20..1500 | oui |
| 13 | `run1_max_mm_s` | 20..1500 | non (suit `scan_max_mm_s` au démarrage) |
| 16 | `run2_max_mm_s` (Fast run) | 20..1500 | oui |
| 20 | `maze_max_side` | 6, 9 ou 16 | oui |
| 50 | `sound_level` (ZHONX II) | 0 muet .. 3 fort | oui |

### Télémétrie

| Champ | Source ZHONX II |
|---|---|
| `cell_x/y`, `heading` | Ceux du dernier `EVENT CELL`, comme ZHONX III. |
| `enc_left_um`, `enc_right_um` | Pas **commandés** des moteurs (odomètre signé dans les interruptions TIM2/TIM3), jamais remis à zéro. Un pas perdu ou un glissement n'y apparaît pas. |
| `gyro_cdeg`, `omega_ddps` | Pas de gyroscope : cap déduit des pas, (gauche − droite) / voie, **positif dans le sens horaire** comme le gyroscope de ZHONX III. La voie vient de la calibration de rotation, sinon 83,5 mm. |
| `speed_mm_s`, `target_mm` | Pas des roues ; distance restante du mouvement en cours. |
| `tel_*_mm` | 0 : capteurs IR tout-ou-rien. |
| `walls` | `FL\|FR` = F10, `DL` = L10, `DR` = R10. |
| `flags` | `WALL_FOLLOW` dès le premier mouvement (ZHONX II suit toujours les murs). |
| `bat_mv` | Tension du pack au repos (jauge du menu). |

### Événements

- `CELL` : une par arrivée dans une nouvelle cellule, **datée au passage de son bord
  d'entrée**. ZHONX II ne voit les murs qu'au centre de la case : l'événement part à ce
  moment-là, avec la date du bord d'entrée. Il peut donc suivre des événements plus récents ;
  l'application et les outils trient par date. La première CELL est émise juste après `START`,
  à la cellule de départ. Pendant les courses et le retour : une CELL à l'entrée de la dernière
  case de chaque segment, avec les murs de la carte. Aucune pour `MOVE` / `TURN` à distance.
- `DECISION` : avant chaque segment, direction relative au cap avant le virage, case visée =
  dernière case du segment.
- `MOVE_BEGIN` / `MOVE_END` : virages en degrés (90, −90, 180) ; pendant les courses, une ligne
  droite ou une courbe par morceau du chemin. `0x20` (RESET_START) n'est plus émis : plus
  d'appui contre les murs du départ avant les courses.
- `STOP` : 1 = commande STOP ; 2 = défaut de mouvement, suivi de `FAULT` (code 1, détail =
  `fw_motion_fault()` : 1 capteurs ou délai, 2 obstacle, 4 centrage, 5 alignement).
- `START`, `HAND_WAIT`, `HAND_GO`, `RUN_DONE`, `BATTERY` (1 Hz), `LED_SYNC`, `CMD_DONE`,
  `DROPPED`, `DUMP_DONE`, `PARAMS_SAVED`, `TELEM_CFG` : comme ZHONX III.

## Côté application et Mac

- **Android (Ariane)** : écran Connexion > « Robots BLE (Zhonx-2) » > Rechercher, puis toucher
  le robot. L'adresse est retenue avec le préfixe `ble:` (aussi accepté dans le champ
  d'adresse manuelle et par `ariane phone link connect`). `BleLink.kt` : GATT FFE0/FFE1,
  écritures sans réponse de 20 octets, notifications en réception.
- **Mac** : `ariane bt --address ble <sous-commande>` (`bleak`).

## Épreuve

Le règlement interdit les communications externes pendant l'épreuve. Pendant une
exploration ou une course **sans session Ariane**, le firmware n'émet rien sur le module (les
autres UART restent coupés comme avant). Le module, lui, reste alimenté et continue d'annoncer
sa présence BLE : **le débrancher pour l'épreuve**. Avec une session Ariane ouverte, la
télémétrie continue pendant les courses, pour les essais filmés.

## Sons

Le buzzer SMT-1027-S-R (magnétique, résonance 2,7 kHz ± 0,5) était piloté avec des notes
entre 294 et 1 800 Hz, soit 10 à 15 dB sous son maximum. Les mélodies sont maintenant dans
la bande 1,8–4,2 kHz :

- victoire : 2093, 2637, 3136, 4186 Hz ; échec : 2794, 2349, 1865 Hz ;
- menus : clic 2,7 kHz sur haut/bas, deux notes montantes pour OK, descendantes pour retour ;
- connexion et déconnexion BLE : arpèges ;
- **Settings > Sound** : Off, Low, Medium, High, réglé à l'oreille et sauvegardé avec les
  réglages. Le volume suit le rapport cyclique du PWM de TIM4 : 50 % (le maximum de cet
  étage), 12 % et 4 % (environ −9 dB et −18 dB). Les sauvegardes existantes donnent High ;
- **Hardware tests > Beeper** : trois tons (2 000, 2 700, 3 500 Hz) puis les mélodies.

Pour aller plus fort, il faut toucher au matériel : R30 (12 Ω) fixe le courant, sous
3,3 V, sans diode de roue libre sur SP1.

## Essais à faire sur le robot

1. Flasher (`scripts/flash/flash.sh --release`), puis **Hardware tests > Bluetooth** : état
   `ready`, et une adresse affichée. Sinon, vérifier le câblage de K1 (TX/RX croisés).
2. Depuis le Mac : `ariane bt --address ble hello`, puis `ping` et `record --seconds 30`.
   Le Mac doit avoir l'autorisation Bluetooth pour le terminal, et `bleak` doit être installé
   (`pip install bleak`).
3. `ariane bt --address ble console` puis `bt`, et `status` : compteurs et budget.
3 bis. Sur le téléphone : Connexion > Robots BLE > Rechercher, toucher « Zhonx-2 » ; le
   HELLO_ACK doit afficher `Zhonx2`, `Zhonx2 ariane` et le schéma identique.
4. `ariane bt --address ble --i-know cmd move 1`, robot posé au centre d'une case ; puis
   `cmd stop` pendant un mouvement.
5. Ajuster `bt_tx_bps` si `record` montre des trous (pertes du module) ou si la télémétrie
   prend du retard.
