# Changelog — Rob's ZombieVerter Fork (RW)

All changes are relative to the upstream Stm32-vcu master branch by Damien Maguire / Tom de Bree.

---

## 2.40A-RW — May 2026

### Added

- **VESC inverter class** (`include/vescinv.h`, `src/vescinv.cpp`)
  - New `InvModes::VESC = 9` inverter type selectable from the web UI
  - Controls VESC motor controller over CAN bus at 500 kbps
  - Torque control via `CAN_PACKET_SET_CURRENT_REL` (packet 10, ID `0xA01`) — relative
    current command scaled to [-100000, +100000] matching ZombieVerter throttle percent
  - All current limits configured in VESC Tool; no current constants in firmware
  - Decodes VESC STATUS1 (ERPM, current, duty), STATUS4 (temperatures), STATUS5 (voltage)
  - STATUS frame receive IDs registered in `SetCanInterface()` so frames are correctly routed
  - 100 Hz CAN command rate in MOD_RUN (every Task10Ms call)
  - Task100Ms empty — VESC watchdog times out gracefully when not in MOD_RUN
  - Posts final torque value to `Param::torque` for web UI visibility
  - Tunable constants in `vescinv.h`: `VESC_CONTROLLER_ID`, `VESC_POLE_PAIRS`
  - Designed for bench training setup with RC motor and LiPo pack

- **DriveInhibit immobilizer** (`include/param_prj.h`, `src/stm32_vcu.cpp`)
  - Param `DriveInhibit` (ID 156) — already declared in upstream but never implemented
  - Mode `0=Off` — normal operation
  - Mode `1=PlugDetect` — inhibit drive when charge plug detected via PP ADC input
  - Mode `2=Always` — unconditional immobilizer; intended for M5Dial PIN-code control
  - New spot value `DriveInhibited` (ID 2124) — read-only, reflects current inhibit state
  - New status bit `STAT_DRIVEINHIBIT = 512` in status word
  - Blocks `MOD_OFF → MOD_PRECHARGE` transition when inhibited (drive start prevented)
  - Forces `MOD_RUN → MOD_OFF` with normal shutdown sequence if inhibited mid-run
  - Zeroes torque demand every 10ms tick regardless of throttle position when inhibited
  - Charging (`MOD_CHARGE`) is unaffected — precharge for charging always permitted

### Modified

- **`src/stm32_vcu.cpp`**
  - Added `#include "vescinv.h"`
  - Added `static VescInverter vescInv` instance
  - Added `case InvModes::VESC` to `UpdateInv()` switch
  - Added `static bool driveInhibited` state variable
  - Added DriveInhibit evaluation block in `Ms10Task()` (runs every 10ms)
  - Added torque zero gate in throttle processing block
  - Added `!driveInhibited` guard to `MOD_OFF` drive start condition
  - Added `driveInhibited` shutdown trigger in `MOD_RUN` case

- **`include/param_prj.h`**
  - Added `VESC = 9` to `InvModes` enum
  - Bumped `Inverter` param max from 8 to 9
  - Added `9=VESC` to `INVMODES` display string
  - Updated `DRIVEINHIBITMODES` string from `"0=Off,1=Plug detect"` to `"0=Off,1=PlugDetect,2=Always"`
  - Bumped `DriveInhibit` param max from 1 to 2
  - Added `STAT_DRIVEINHIBIT = 512` to `status` enum
  - Updated `STATUS` string to include `512=DriveInhibit`
  - Added `VALUE_ENTRY(DriveInhibited, ONOFF, 2124)`
  - Updated version string to `2.40A-RW`

- **`Makefile`**
  - Added `vescinv.o` to `OBJSL` build list

- **`src/utils.cpp`**
  - Fixed `CpSpoofOutput()` — three bugs corrected:
  - **Fix 1 — Wrong param gating:** Original checked `interface` (charge interface) to decide
    whether to output CP. For Outlander and Elcon builds `interface = Unused` is correct, so
    `CpVal` was always 0. Fix checks `chargemodes` (charger type) instead.
  - **Fix 2 — Missing charger types:** Added all OBCs that need a spoofed CP signal:
    `Out_lander`, `Elcon`, `Volt_Ampera`, `TeslaOI`, `MGgen2`, `EXT_DIGI`. Leaf PDM correctly
    excluded (handles CP internally). CHAdeMO correctly excluded (not J1772).
  - **Fix 3 — Missing opmode gate:** Original ran unconditionally every 200ms. The guard line
    above it was commented out and also a no-op (missing `()`). Fix gates the entire
    computation on `opmode == MOD_CHARGE` so the CP pin goes low when not charging.

### Known issues / not yet tested

- DriveInhibit lock via M5Dial: unlock confirmed working; lock (writing `DriveInhibit=2` via
  SDO) not yet confirmed — M5Dial may be updating display state locally without sending the
  SDO write to param ID 156. Investigate M5Dial lock action handler.
- CPSpoof fix: not yet tested end-to-end with OBC connected
- VESC inverter type: not yet tested after extended frame and RegisterUserMessage fixes

---

## Upstream Base

Built from: https://github.com/damienmaguire/Stm32-vcu
Branch: master
Cloned: May 2026