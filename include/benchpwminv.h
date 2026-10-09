/*
 * benchpwminv.h - Bench-trainer PWM H-bridge "inverter" for ZombieVerter VCU
 *
 * Drives a small brushed DC gearmotor through a DRV8871-style H-bridge using
 * two TIM3 PWM channels (no CAN, no telemetry):
 *
 *   PWM1 (PA6, TIM3_CH1) -> DRV8871 IN1   forward
 *   PWM2 (PA7, TIM3_CH2) -> DRV8871 IN2   reverse
 *
 *   torque > 0 : IN1 = PWM, IN2 = 0
 *   torque < 0 : IN1 = 0,   IN2 = PWM
 *   torque = 0 : IN1 = IN2 = 0 (coast)
 *
 * Required settings (web UI):
 *   PWM1Func = PWM_TIM3, PWM2Func = PWM_TIM3   (generic, NOT CP_SPOOF etc.)
 *   Tim3_Presc / Tim3_Period set the carrier frequency; duty is scaled to
 *   Tim3_Period at run time. Reboot after changing PWMxFunc or Tim3_*.
 */
#ifndef BENCHPWMINV_H
#define BENCHPWMINV_H

#include <inverter.h>

// Motor is 12 V rated but the bench pack is 5S (up to ~21 V). Cap the average
// voltage at the motor: 12 / 18.5 ~= 64 %.
#define BENCH_MAX_DUTY_PCT 64.0f

class BenchPwmInverter : public Inverter {
public:
  BenchPwmInverter();
  void SetTorque(float torquePercent) override;
  void DeInit() override;
  float GetMotorTemperature() override { return 0; }
  float GetInverterTemperature() override { return 0; }
  float GetInverterVoltage() override { return 0; }
  float GetMotorSpeed() override { return 0; }
  int GetInverterState() override { return 0; }

private:
  void SetDuty(float ch1Pct, float ch2Pct);
};

#endif
