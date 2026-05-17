# Changelog — Rob's ZombieVerter Fork (RW)

All changes are relative to the upstream Stm32-vcu master branch by Damien Maguire / Tom de Bree.

---

## 2.40A-RW — May 2026

### Added
- **VESC inverter class** (`include/vescinv.h`, `src/vescinv.cpp`)
  - New `InvModes::VESC = 9` inverter type selectable from the web UI
  - Controls VESC motor controller over CAN bus at 500 kbps
  - Torque control via `CAN_PACKET_SET_CURRENT` mapped from ZombieVerter throttle percent
  - Decodes VESC STATUS1 (ERPM, current, duty), STATUS4 (temperatures), STATUS5 (voltage)
  - 50 Hz CAN command rate in MOD_RUN; 0A keepalive in all other modes to prevent VESC watchdog timeout
  - Tunable constants in vescinv.h: `VESC_CONTROLLER_ID`, `VESC_MAX_CURRENT_A`, `VESC_MAX_REGEN_A`, `VESC_POLE_PAIRS`
  - Designed for bench training setup with RC motor and LiPo pack

### Modified
- **`src/stm32_vcu.cpp`**
  - Added `#include "vescinv.h"`
  - Added `static VescInverter vescInv` instance
  - Added `case InvModes::VESC` to `UpdateInv()` switch

- **`include/param_prj.h`**
  - Added `VESC = 9` to `InvModes` enum
  - Bumped `Inverter` param max from 8 to 9
  - Added `9=VESC` to `INVMODES` display string
  - Updated version string to `2.40A-RW`

- **`Makefile`**
  - Added `vescinv.o` to `OBJSL` build list

- **`src/utils.cpp`**
  - Fixed `CpSpoofOutput()` — three bugs corrected:
  - **Fix 1 — Wrong param gating:** Original checked `interface` (charge interface) to decide whether to output CP. For Outlander and Elcon builds `interface = Unused` is correct, so `CpVal` was always 0. Fix checks `chargemodes` (charger type) instead for the OBC-only case.
  - **Fix 2 — Missing charger types:** Added all OBCs that need a spoofed CP signal to the `chargerNeedsSpoof` check: `Out_lander`, `Elcon`, `Volt_Ampera`, `TeslaOI`, `MGgen2`, `EXT_DIGI`. Leaf PDM correctly excluded (handles CP internally). CHAdeMO correctly excluded (not J1772).
  - **Fix 3 — Missing opmode gate:** Original ran unconditionally every 200ms. The guard line above it (`// if(opmode==MOD_CHARGE) utils::CpSpoofOutput;`) was commented out and was a no-op anyway — missing the `()`. Fix gates the entire computation on `opmode == MOD_CHARGE` so the pin goes low when not charging.

---

## Upstream Base

Built from: https://github.com/damienmaguire/Stm32-vcu  
Branch: master  
Cloned: May 2026
