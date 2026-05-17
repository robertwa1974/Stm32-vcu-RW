/*
 * vescinv.cpp - VESC Inverter Class for ZombieVerter VCU
 *
 * Control flow:
 *   ZombieVerter Ms10Task calls SetTorque(torquePercent) every 10ms.
 *   torquePercent is [-100, +100]; positive = forward, negative = regen.
 *
 *   We use CAN_PACKET_SET_CURRENT_REL (packet 10, CAN ID 0xA01 for node 1).
 *   This sends a relative current command scaled to [-100000, +100000]
 *   where 100000 = 100.000% of the VESC's configured motor current limit.
 *   All current limits are set in VESC Tool — not in this file.
 *
 *   Frames MUST be sent as 29-bit extended CAN frames (4th arg true to Send()).
 *   VESC ignores standard 11-bit frames entirely.
 *
 *   Task10Ms() is called by ZombieVerter only when in MOD_RUN.
 *   We send the torque command every tick (100 Hz).
 *
 *   Task100Ms() is empty — VESC watchdog will time out when not in MOD_RUN
 *   and restart cleanly on the next MOD_RUN entry.
 *
 *   VESC STATUS frames arrive asynchronously via DecodeCAN().
 *   RegisterUserMessage() in SetCanInterface() ensures they are routed here.
 */

#include "vescinv.h"
#include "params.h"
#include <stdint.h>

// -------------------------------------------------------------------
// Constructor
// -------------------------------------------------------------------
VescInverter::VescInverter()
    : can(nullptr)
    , erpm(0.0f)
    , motorCurrent(0.0f)
    , dutyNow(0.0f)
    , tempFet(25.0f)
    , tempMotor(25.0f)
    , voltageIn(0.0f)
    , cmdTorquePercent(0.0f)
    , statusReceived(false)
{
}

// -------------------------------------------------------------------
// SetCanInterface
// Called by ZombieVerter when the CAN interface is assigned or changed.
// We register the three VESC STATUS frame IDs so DecodeCAN() receives them.
// -------------------------------------------------------------------
void VescInverter::SetCanInterface(CanHardware* c)
{
    can = c;
    Inverter::SetCanInterface(c);

    can->RegisterUserMessage(VESC_EID(VESC_PKT_STATUS1));  // 0x901
    can->RegisterUserMessage(VESC_EID(VESC_PKT_STATUS4));  // 0x1001
    can->RegisterUserMessage(VESC_EID(VESC_PKT_STATUS5));  // 0x1B01
}

// -------------------------------------------------------------------
// SetTorque
// Called every 10ms by ZombieVerter Ms10Task with [-100, +100].
// Stores the value; Task10Ms() transmits it.
// -------------------------------------------------------------------
void VescInverter::SetTorque(float torquePercent)
{
    cmdTorquePercent = torquePercent;
}

// -------------------------------------------------------------------
// Task10Ms
// Called by ZombieVerter Ms10Task only when opmode == MOD_RUN.
// Sends relative current command at 100 Hz.
// -------------------------------------------------------------------
void VescInverter::Task10Ms()
{
    SendRelativeCurrentCommand(cmdTorquePercent);
}

// -------------------------------------------------------------------
// Task100Ms
// Empty — VESC watchdog times out gracefully when not in MOD_RUN.
// -------------------------------------------------------------------
void VescInverter::Task100Ms()
{
}

// -------------------------------------------------------------------
// SendRelativeCurrentCommand
// Transmits CAN_PACKET_SET_CURRENT_REL (packet 10) to the VESC.
//
// Extended CAN ID (29-bit): (10 << 8) | VESC_CONTROLLER_ID = 0xA01
//
// Data: 4 bytes big-endian int32
//   value = torquePercent * 1000
//   range: -100000 = -100% (full regen), +100000 = +100% (full drive)
//   VESC scales this against its own configured current limits.
//
// CRITICAL: 4th argument to Send() must be true for 29-bit extended frame.
//   VESC ignores standard 11-bit frames.
// -------------------------------------------------------------------
void VescInverter::SendRelativeCurrentCommand(float torquePercent)
{
    if (can == nullptr) return;

    int32_t value = (int32_t)(torquePercent * 1000.0f);

    uint32_t buf[2] = {0, 0};
    uint8_t* b = (uint8_t*)buf;
    b[0] = (value >> 24) & 0xFF;
    b[1] = (value >> 16) & 0xFF;
    b[2] = (value >>  8) & 0xFF;
    b[3] = (value      ) & 0xFF;

    // Post torque value to web UI
    Param::SetInt(Param::torque, value);

    // true = 29-bit extended frame — required for VESC CAN protocol
    can->Send(VESC_EID(VESC_PKT_SET_CURRENT_REL), buf, 4, true);
}

// -------------------------------------------------------------------
// DecodeCAN
// Called by ZombieVerter CanCallback for every received CAN frame.
//
// VESC STATUS1 (pkt 9, ID 0x901):
//   Bytes 0-3: ERPM as int32 BE
//   Bytes 4-5: Current * 10 as int16 BE
//   Bytes 6-7: Duty * 1000 as int16 BE
//
// VESC STATUS4 (pkt 16, ID 0x1001):
//   Bytes 0-1: Temp FET * 10 as int16 BE
//   Bytes 2-3: Temp Motor * 10 as int16 BE
//
// VESC STATUS5 (pkt 27, ID 0x1B01):
//   Bytes 4-5: Voltage In * 10 as int16 BE
// -------------------------------------------------------------------
void VescInverter::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* b = (uint8_t*)data;

    if (id == VESC_EID(VESC_PKT_STATUS1))
    {
        erpm         = (float)DecodeInt32BE(data, 0);
        motorCurrent = (float)DecodeInt16BE(data, 4) / 10.0f;
        dutyNow      = (float)DecodeInt16BE(data, 6) / 1000.0f;
        statusReceived = true;
    }
    else if (id == VESC_EID(VESC_PKT_STATUS4))
    {
        tempFet   = (float)DecodeInt16BE(data, 0) / 10.0f;
        tempMotor = (float)DecodeInt16BE(data, 2) / 10.0f;
    }
    else if (id == VESC_EID(VESC_PKT_STATUS5))
    {
        voltageIn = (float)DecodeInt16BE(data, 4) / 10.0f;
    }
}

// -------------------------------------------------------------------
// Getters
// -------------------------------------------------------------------

float VescInverter::GetMotorSpeed()
{
    return erpm / (float)VESC_POLE_PAIRS;
}

float VescInverter::GetInverterVoltage()
{
    return voltageIn;
}

float VescInverter::GetMotorTemperature()
{
    return tempMotor;
}

float VescInverter::GetInverterTemperature()
{
    return tempFet;
}

int VescInverter::GetInverterState()
{
    return statusReceived ? 1 : 0;
}

// -------------------------------------------------------------------
// Decode helpers
// ZombieVerter packs 8 CAN payload bytes into uint32_t[2]:
//   buf[0] = bytes 0-3, buf[1] = bytes 4-7
// Reinterpret as uint8_t* and decode big-endian multi-byte fields.
// -------------------------------------------------------------------

int32_t VescInverter::DecodeInt32BE(uint32_t* data, int byteOffset)
{
    uint8_t* b = (uint8_t*)data;
    return ((int32_t)b[byteOffset    ] << 24) |
           ((int32_t)b[byteOffset + 1] << 16) |
           ((int32_t)b[byteOffset + 2] <<  8) |
           ((int32_t)b[byteOffset + 3]      );
}

int16_t VescInverter::DecodeInt16BE(uint32_t* data, int byteOffset)
{
    uint8_t* b = (uint8_t*)data;
    return ((int16_t)b[byteOffset    ] << 8) |
           ((int16_t)b[byteOffset + 1]     );
}
