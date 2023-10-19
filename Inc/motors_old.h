/*
 * motors.h
 *
 *  Created on: 2 Nov 2018
 *      Author: erkim
 */

#ifndef PERIPHERALS_MOTORS_H_
#define PERIPHERALS_MOTORS_H_

#include <stdbool.h>

#include "tim.h"
#include "utility.h"

typedef enum
{
  MotorsCoasting = 0,
  MotorBraking,
  MotorsForward,
  MotorsBackward
} PER_Motors_State_e;

typedef struct
{
  TIM_HandleTypeDef* tim_left;
  TIM_HandleTypeDef* tim_right;

  const uint32_t ch_fwd_left;
  const uint32_t ch_bck_left;

  const uint32_t ch_fwd_right;
  const uint32_t ch_bck_right;

  int16_t power_left;
  int16_t power_right;
  int16_t target_left;
  int16_t target_right;

  uint32_t last_process;

  PER_IOPair_t sleep;

  PER_Motors_State_e state;
  PER_Motors_State_e target_state;
} PER_Motors_t;

PER_Motors_t PER_MotorsInit(TIM_HandleTypeDef* tim_left, TIM_HandleTypeDef* tim_right,
                            const uint32_t ch_fwd_l, const uint32_t ch_bck_l,
                            const uint32_t ch_fwd_r, const uint32_t ch_bck_r,
                            PER_IOPair_t sleep);

void PER_MotorsProcess(PER_Motors_t* motors);
void PER_MotorsSetTarget(const int16_t left, const int16_t right, PER_Motors_t* motors);

void PER_MotorsBreak(PER_Motors_t* motors);

void PER_MotorsSleep(PER_Motors_t* motors);
void PER_MotorsEnable(PER_Motors_t* motors);

#endif /* PERIPHERALS_MOTORS_H_ */
