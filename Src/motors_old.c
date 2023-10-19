/*
 * motors.c
 *
 *  Created on: 2 Nov 2018
 *      Author: erkim
 */

#include "motors.h"

#include "usart.h"

#define MOTORS_SHORTED 99
#define MOTORS_MAX 85
#define MOTORS_NEUTRAL 0
#define MOTORS_MIN -85

#define PROCESS_DELAY 75
#define MAX_DELTA 5

#define CLAMP_MOTORS(x) CLAMP(MOTORS_MAX, MOTORS_NEUTRAL, x)

static void _MotorsSet(const int16_t left, const int16_t right, PER_Motors_t* motors)
{
  if (left > 0)
  {
    __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_fwd_left, CLAMP_MOTORS(left));
    __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_bck_left, MOTORS_NEUTRAL);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_fwd_left, MOTORS_NEUTRAL);
    __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_bck_left, CLAMP_MOTORS((-1 * left)));
  }

  if (right > 0)
  {
    __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_fwd_right, CLAMP_MOTORS(right));
    __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_bck_right, MOTORS_NEUTRAL);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_fwd_right, MOTORS_NEUTRAL);
    __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_bck_right, CLAMP_MOTORS((-1 * right)));
  }

  motors->power_left = CLAMP(MOTORS_MAX, MOTORS_MIN, left);
  motors->power_right = CLAMP(MOTORS_MAX, MOTORS_MIN, right);
}


PER_Motors_t PER_MotorsInit(TIM_HandleTypeDef* tim_left, TIM_HandleTypeDef* tim_right,
                            const uint32_t ch_fwd_l, const uint32_t ch_bck_l,
                            const uint32_t ch_fwd_r, const uint32_t ch_bck_r,
                            PER_IOPair_t sleep)
{
  PER_Motors_t motors = {
      .tim_left = tim_left,
      .tim_right = tim_right,

      .ch_fwd_left = ch_fwd_l,
      .ch_bck_left = ch_bck_l,
      .ch_fwd_right = ch_fwd_r,
      .ch_bck_right = ch_bck_r,

      .power_left = 0,
      .power_right = 0,

      .sleep = sleep,
      .is_sleeping = false
  };

  PER_UtilResetIO(&motors.sleep);

  HAL_TIM_PWM_Start(tim_left, ch_fwd_l);
  HAL_TIM_PWM_Start(tim_left, ch_bck_l);

  HAL_TIM_PWM_Start(tim_right, ch_fwd_r);
  HAL_TIM_PWM_Start(tim_right, ch_bck_r);

  __HAL_TIM_SET_COMPARE(tim_left, ch_fwd_l, 0);
  __HAL_TIM_SET_COMPARE(tim_left, ch_bck_l, 0);

  __HAL_TIM_SET_COMPARE(tim_left, ch_fwd_r, 0);
  __HAL_TIM_SET_COMPARE(tim_left, ch_bck_r, 0);

  PER_UtilSetIO(&motors.sleep);

  return motors;
}

void PER_MotorsProcess(PER_Motors_t* motors)
{
  const uint32_t curr_tick = HAL_GetTick();

  switch (motors->state)
  {
  case MotorsForward:
  case MotorsBackward:
  {
    if ((curr_tick - motors->last_process) > PROCESS_DELAY)
    {
      const int16_t diff_l = CLAMP(MAX_DELTA, -MAX_DELTA, motors->target_left - motors->power_left);
      const int16_t diff_r = CLAMP(MAX_DELTA, -MAX_DELTA, motors->target_right - motors->power_right);

      _MotorsSet(motors->power_left + diff_l, motors->power_right + diff_r, motors);

      motors->last_process = HAL_GetTick();

      PER_UTIL_PrintToUart(&huart2, "Set speeds: l %d, r %d\n\r", motors->power_left, motors->power_right);
    }

    break;
  }
  case MotorsBraking:
  {
    if (motors->target_state == MotorsBraking)
      break;

    if (_CanHopDirection(motors))
      motors->state = motors->target_state;

    break;
  }
  }
}

void PER_MotorsSetTarget(const int16_t left, const int16_t right, PER_Motors_t* motors)
{
  switch (motors->state)
  {
  case MotorsForward:
  {

  }
  }

  motors->target_left = CLAMP(MOTORS_MAX, MOTORS_MIN, left);
  motors->target_right = CLAMP(MOTORS_MAX, MOTORS_MIN, right);
}

void PER_MotorsBreak(PER_Motors_t* motors)
{
  __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_fwd_left, MOTORS_NEUTRAL);
  __HAL_TIM_SET_COMPARE(motors->tim_left, motors->ch_bck_left, MOTORS_NEUTRAL);
  __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_fwd_right, MOTORS_NEUTRAL);
  __HAL_TIM_SET_COMPARE(motors->tim_right, motors->ch_bck_right, MOTORS_NEUTRAL);

  motors->power_left = 0;
  motors->power_right = 0;
}


void PER_MotorsSleep(PER_Motors_t* motors)
{
  motors->is_sleeping = true;
  PER_UtilResetIO(&motors->sleep);
}

void PER_MotorsEnable(PER_Motors_t* motors)
{
  motors->is_sleeping = false;
  PER_UtilSetIO(&motors->sleep);
}
