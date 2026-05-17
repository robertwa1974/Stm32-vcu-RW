/*
 * vescinv.h - VESC Inverter Class for ZombieVerter VCU
 *
 * Implements the Inverter base class interface to allow ZombieVerter
 * to control a VESC motor controller over CAN bus.
 *
 * VESC CAN protocol reference:
 *   https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md
 *
 * Place in: include/vescinv.h
 * Place src: src/vescinv.cpp
 *
 * WIRING:
 *   ZombieVerter CAN1 (or CAN2) -> VESC CAN H/L
 *   500 kbps, 120R termination at each end of the bus
 *
 * VESC TOOL SETUP (required before connecting to ZombieVerter):
 *   1. Run FOC motor detection wizard to completion
 *   2. App Settings -> General:
 *        VESC ID            : 1  (match VESC_CONTROLLER_ID below)
 *        App to use         : No App
 *        CAN Mode           : VESC
 *        CAN Status Rate 1  : 20 Hz
 *        CAN Status Mode 1  : Status 1, 4, and 5 checked
 *        Timeout (ms)       : 500  (default)
 *   3. Write config, disconnect USB, connect CAN to ZombieVerter
 *
 * ZOMBIEVERTER WEB UI:
 *   Inverter type: VESC  (enum value 9)
 *   InverterCan  : whichever CAN port the VESC is wired to (0=CAN1, 1=CAN2)
 *
 * CURRENT LIMITING:
 *   All current limits are set in VESC Tool — not in this code.
 *   ZombieVerter sends a relative current command (-100% to +100%).
 *   VESC scales that against its own configured motor/battery current limits.
 *   Set Motor Config -> Current -> Max current and Max regen current in VESC Tool.
 */

#ifndef VESCINV_H
#define VESCINV_H

#include "canhardware.h"
#include "inverter.h"
#include <stdint.h>

// -------------------------------------------------------------------
// Configuration
// -------------------------------------------------------------------

// VESC CAN node ID — must match VESC Tool: App Settings -> VESC ID
#define VESC_CONTROLLER_ID    1

// Motor electrical pole pairs — for ERPM->RPM conversion
// Read from VESC Tool after FOC detection (Motor Config -> General -> Pole pairs)
#define VESC_POLE_PAIRS       7

// -------------------------------------------------------------------
// VESC CAN packet IDs (comm_can.h in VESC firmware)
// -------------------------------------------------------------------
// TX — sent by ZombieVerter to VESC (29-bit extended frames)
#define VESC_PKT_SET_CURRENT_REL  10  // Relative current: int32 * 100000 = [-100000..+100000]
                                      // CAN ID = (10 << 8) | node = 0xA01 for node 1

// RX — sent by VESC to ZombieVerter
#define VESC_PKT_STATUS1      9   // ERPM(i32), Current*10(i16), Duty*1000(i16)  → 0x901
#define VESC_PKT_STATUS4      16  // TempFET*10(i16), TempMot*10(i16)            → 0x1001
#define VESC_PKT_STATUS5      27  // Tach(i32), VoltIn*10(i16)                   → 0x1B01

// Extended CAN ID formula: (packet << 8) | node_id
#define VESC_EID(pkt)  (((uint32_t)(pkt) << 8) | (uint8_t)(VESC_CONTROLLER_ID))


class VescInverter : public Inverter
{
public:
    VescInverter();

    // SetCanInterface: capture pointer locally AND register VESC status RX IDs
    void SetCanInterface(CanHardware* c) override;

    // Inverter base class interface
    void  Task10Ms() override;    // sends torque command at 100 Hz in MOD_RUN
    void  Task100Ms() override;   // empty — VESC watchdog handled by timeout/restart
    void  DecodeCAN(int id, uint32_t* data) override;
    void  SetTorque(float torquePercent) override;

    float GetMotorTemperature() override;
    float GetInverterTemperature() override;
    float GetInverterVoltage() override;
    float GetMotorSpeed() override;
    int   GetInverterState() override;

private:
    void    SendRelativeCurrentCommand(float torquePercent);
    int32_t DecodeInt32BE(uint32_t* data, int byteOffset);
    int16_t DecodeInt16BE(uint32_t* data, int byteOffset);

    CanHardware* can;

    float   erpm;           // electrical RPM (STATUS1)
    float   motorCurrent;   // phase current in A (STATUS1)
    float   dutyNow;        // duty cycle 0..1 (STATUS1)
    float   tempFet;        // MOSFET temp °C (STATUS4)
    float   tempMotor;      // motor temp °C (STATUS4)
    float   voltageIn;      // input voltage V (STATUS5)

    float   cmdTorquePercent;  // last torque% set by SetTorque()
    bool    statusReceived;    // true once first STATUS1 frame arrives
};

#endif // VESCINV_H
