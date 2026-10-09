/*
 * benchpwminv.cpp - Bench-trainer PWM H-bridge "inverter" (see header)
 *
 * stm32_vcu.cpp calls SetTorque() every 10 ms in ALL operating modes (it
 * passes 0 outside MOD_RUN), so the output always returns to 0 when the VCU
 * leaves run. DeInit() also zeroes the outputs when the inverter type is
 * changed away from this class.
 */
#include "benchpwminv.h"
#include "iomatrix.h"
#include "params.h"
#include <libopencm3/stm32/timer.h>
#include <stdint.h>

BenchPwmInverter::BenchPwmInverter() {}

void BenchPwmInverter::SetDuty(float ch1Pct, float ch2Pct) {
  // Generic PWM_TIM3 mode leaves the timer on the user's Tim3_Period.
  uint32_t period = (uint32_t)Param::GetInt(Param::Tim3_Period);

  if (Param::GetInt(Param::PWM1Func) == IOMatrix::PWM_TIM3)
    timer_set_oc_value(TIM3, TIM_OC1, (uint32_t)(period * ch1Pct / 100.0f));
  if (Param::GetInt(Param::PWM2Func) == IOMatrix::PWM_TIM3)
    timer_set_oc_value(TIM3, TIM_OC2, (uint32_t)(period * ch2Pct / 100.0f));
}

void BenchPwmInverter::SetTorque(float torquePercent) {
  // Clamp request to +/-100 %, then scale to the motor-safe duty ceiling.
  if (torquePercent > 100.0f)
    torquePercent = 100.0f;
  if (torquePercent < -100.0f)
    torquePercent = -100.0f;

  float duty = (torquePercent < 0 ? -torquePercent : torquePercent) *
               (BENCH_MAX_DUTY_PCT / 100.0f);

  if (torquePercent > 0.0f)
    SetDuty(duty, 0.0f);
  else if (torquePercent < 0.0f)
    SetDuty(0.0f, duty);
  else
    SetDuty(0.0f, 0.0f);
}

void BenchPwmInverter::DeInit() { SetDuty(0.0f, 0.0f); }
