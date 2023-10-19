
#include "motors.h"

#include <stdbool.h>
#include "adc.h"
#include "usart.h"

#define MAX_RAMP_STEP 5
#define PROCESS_DELAY 100
#define PWM_MAX 99
#define CLAMP_PWM(x) CLAMP(85, 0, x)
#define CLAMP_SPEED(x) CLAMP(85, -85, x)

static inline bool _CanTick(PER_Motors_t* motor);
static inline void _ApplyRampedSpeed(PER_Motors_t* motor);
static inline void _MotorSet(PER_Motors_t* motor, int16_t speed);
static inline void _ApplyActiveBraking(PER_Motors_t* motor);
static inline bool _CanExitBraking(PER_Motors_t* motor);
static uint32_t _ReadAdc(ADC_HandleTypeDef* hadc, uint8_t channel);

PER_Motors_t PER_Motors_Init(TIM_HandleTypeDef* timer, const uint32_t ch_fwd, const uint32_t ch_bck,
    PER_IOPair_t sleep)
{
  PER_Motors_t motors = {
      .timer = timer,

      .ch_fwd = ch_fwd,
      .ch_bck = ch_bck,

      .power_current = 0,
      .power_target = 0,

      .last_process = 0,

      .sleep = sleep,

      .state = MotorState_Braking,
      .target_state = MotorState_Braking
  };

  PER_UtilResetIO(&sleep);

  HAL_TIM_PWM_Start(timer, ch_fwd);
  HAL_TIM_PWM_Start(timer, ch_bck);

  __HAL_TIM_SET_COMPARE(timer, ch_fwd, 0);
  __HAL_TIM_SET_COMPARE(timer, ch_bck, 0);

  PER_UtilSetIO(&sleep);

  return motors;
}

void PER_Motors_Process(PER_Motors_t* motor)
{
  if (!_CanTick(motor))
    return;

  switch (motor->state)
  {
  case MotorState_Forward:
  case MotorState_Backward:
    _ApplyRampedSpeed(motor);

    break;
  case MotorState_Braking:
    _ApplyActiveBraking(motor);

    if (motor->target_state != MotorState_Braking)
    {
      if (!_CanExitBraking(motor))
        break;

      motor->state = motor->target_state;
      _ApplyRampedSpeed(motor);
    }
    break;
  default:
    break;
  }

//  PER_UTIL_PrintToUart(&huart2, "Power: %d, state: %d\n\r", motor->power_current, (int16_t)motor->state);
}

void PER_Motors_SetTarget(PER_Motors_t* motor, const int16_t target)
{
  PER_Motors_State_e new_state = MotorState_Sleeping;
  if (target > 0)
    new_state = MotorState_Forward;
  else if (target < 0)
    new_state = MotorState_Backward;
  else
    new_state = MotorState_Braking;

  motor->power_target = target;

  if (new_state != motor->state)
  {
    switch (motor->state)
    {
    case MotorState_Forward:
    case MotorState_Backward:
      motor->state = MotorState_Braking;
      motor->target_state = new_state;
      break;
    case MotorState_Braking:
      motor->target_state = new_state;
      break;
    default:
      break;
    }
  }
}

bool _CanTick(PER_Motors_t* motor)
{
  volatile uint32_t curr_tick = HAL_GetTick();

  if ((curr_tick - motor->last_process) > PROCESS_DELAY)
  {
    motor->last_process = curr_tick;
    return true;
  }
  else
  {
    return false;
  }
}

void _ApplyRampedSpeed(PER_Motors_t* motor)
{
  volatile int16_t diff = CLAMP(MAX_RAMP_STEP, -MAX_RAMP_STEP, motor->power_target - motor->power_current);

  _MotorSet(motor, motor->power_current + diff);
}

void _MotorSet(PER_Motors_t* motor, int16_t speed)
{
  if (speed > 0)
  {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_fwd, CLAMP_PWM(speed));
    __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_bck, 0);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_fwd, 0);
    __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_bck, CLAMP_PWM(-1 * speed));
  }

  motor->power_current = CLAMP_SPEED(speed);
}

void _ApplyActiveBraking(PER_Motors_t* motor)
{
  __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_fwd, PWM_MAX);
  __HAL_TIM_SET_COMPARE(motor->timer, motor->ch_bck, PWM_MAX);

  motor->power_current = 0;
}

bool _CanExitBraking(PER_Motors_t* motor)
{
  volatile uint32_t sens_one = _ReadAdc(&hadc1, 1);
  volatile uint32_t sens_two = _ReadAdc(&hadc1, 2);

  PER_UTIL_PrintToUart(&huart2, "Ileft: %d, Iright: %d\n\r",
      sens_one, sens_two);

  if (sens_one > 100 || sens_two > 100)
  {
    return false;
  }

  return true;
}

uint32_t _ReadAdc(ADC_HandleTypeDef* hadc, uint8_t channel)
{
  ADC_ChannelConfTypeDef s_conf;

  s_conf.Channel = channel;
  s_conf.Rank = 1;
  s_conf.SamplingTime = ADC_SAMPLETIME_4CYCLES_5;

  HAL_ADC_ConfigChannel(hadc, &s_conf);
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 20);

  return HAL_ADC_GetValue(hadc);
}
