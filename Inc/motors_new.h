#ifndef PERIPHERALS_MOTORS_H_
#define PERIPHERALS_MOTORS_H_

#include "tim.h"
#include "utility.h"

typedef enum
{
  MotorState_Sleeping,
  MotorState_Forward,
  MotorState_Backward,
  MotorState_Braking,
  MotorState_RampingDown
} PER_Motors_State_e;

typedef struct
{
  TIM_HandleTypeDef* timer;

  const uint32_t ch_fwd;
  const uint32_t ch_bck;

  int16_t power_current;
  int16_t power_target;

  uint32_t last_process;

  PER_IOPair_t sleep;

  PER_Motors_State_e state;
  PER_Motors_State_e target_state;
} PER_Motors_t;

PER_Motors_t PER_Motors_Init(TIM_HandleTypeDef* timer, const uint32_t ch_fwd, const uint32_t ch_bck,
    PER_IOPair_t sleep);

void PER_Motors_Process(PER_Motors_t* motor);
void PER_Motors_SetTarget(PER_Motors_t* motor, const int16_t target);

#endif // PERIPHERALS_MOTORS_H_
