# Robot identity before any hardware access

This repository targets **ZHONX II**, not ZHONX III.
The user confirmed ST-Link serial **51FF66064982565324552187** for ZHONX II.
Its STM32 UID was read and registered as **003300373432471234373230**.
ZHONX III uses a different probe; never choose the first available USB probe.

- Use `scripts/flash/flash.sh` for programming and `scripts/flash/dump.sh` for dumps.
  `scripts/flash/calibration.sh` is the authorized calibration backup/restore path.
  These tools require an associated probe and check the STM32 UID in the same OpenOCD
  session, before any erase/write. This also applies to agent-initiated flashing.
- Associations are in `${XDG_CONFIG_HOME:-$HOME/.config}/pacabot/robots.json`.
  `scripts/flash/flash.sh --check` checks the target without flashing or resetting.
- If the probe or UID differs, stop hardware operations. Do not bypass the guard,
  fall back to an unselected OpenOCD/st-flash command, or automatically rebind.
- Initial association is allowed only after the user identifies the physical robot
  and probe. `python3 scripts/flash/robot_guard.py associate zhonx2 --serial ...`
  reads identity only, without flash writes or reset/halt commands.
- A firmware already on the chip, its file name, or its MCU family is not proof of
  robot identity. ZHONX II and III both use STM32F405 devices.
- Other sessions can work in ../ZHONX_III. Preserve their changes and their own
  `scripts/flash/probes.tsv` / UID checks; do not overwrite their flash workflow.
