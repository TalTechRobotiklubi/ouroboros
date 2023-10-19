#include "motors.h"

#include <stdbool.h>
#include <math.h>

#define PROCESS_DELAY 20

#define DRV_ADDR ((uint8_t)128)
#define DRV_START_BYTE ((uint8_t)0b10101010)
#define DRV_CMD_FWD ((uint8_t)8)
#define DRV_CMD_BCK ((uint8_t)9)
#define DRV_CMD_RIGHT ((uint8_t)10)
#define DRV_CMD_LEFT ((uint8_t)11)

#define PWM_MAX     2000
#define PWM_NEUTRAL 1500
#define PWM_MIN     1000

#define MAX_RAMP_FRWD 40
#define MAX_RAMP_TURN 40
#define CURVE_CONSTANT_TURN 3.f
#define CURVE_CONSTANT_FRWD 2.f

#define CLAMP_PWM(x) CLAMP(PWM_MAX, PWM_MIN, x)
#define CLAMP_SPEED(x) CLAMP(250, -250, x)

inline void _MotorSet(PER_Motors_t* motor, int16_t speed, int16_t rotation);
inline bool _CanTick(PER_Motors_t* motor);
int16_t _Map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max);
inline void _SendCmdToMotors(PER_Motors_t* motors, uint8_t cmd, uint8_t data);
int16_t _LogCurve(int16_t power, float curve_const);

PER_Motors_t PER_Motors_Init(UART_HandleTypeDef* huart, uint8_t addr)
{
  PER_Motors_t motors = {
      .uart = huart,

      .address = addr,

      .speed_current = 0,
      .speed_target = 0,

      .turn_current = 0,
      .turn_target = 0,

      .last_process = 0
  };

  return motors;
}

void PER_Motors_Activate(PER_Motors_t* motor)
{
  static uint8_t start_byte = DRV_START_BYTE;

  HAL_UART_Transmit(motor->uart, &start_byte, 1, 10);

  HAL_Delay(500);
}

void PER_Motors_Process(PER_Motors_t* motor)
{
  if (!_CanTick(motor))
    return;

  int16_t diff_speed = CLAMP(MAX_RAMP_FRWD, -MAX_RAMP_FRWD, motor->speed_target - motor->speed_current);
  int16_t diff_rotation = CLAMP(MAX_RAMP_TURN, -MAX_RAMP_TURN, motor->turn_target - motor->turn_current);

  _MotorSet(motor, motor->speed_current + diff_speed, motor->turn_current + diff_rotation);
}

void _MotorSet(PER_Motors_t* motor, int16_t speed, int16_t rotation)
{
  motor->speed_current = CLAMP_SPEED(speed);
  motor->turn_current = CLAMP_SPEED(rotation);

  volatile uint8_t speed_set = 0;

  if (speed >= 0)
  {
    speed = _LogCurve(speed, CURVE_CONSTANT_FRWD);
    speed_set = _Map(speed, 0, 500, 0, 127);
    _SendCmdToMotors(motor, DRV_CMD_FWD, speed_set);
  }
  else
  {
    speed = -1 * speed;
    speed = _LogCurve(speed, CURVE_CONSTANT_FRWD);
    speed_set = _Map(speed, 0, 500, 0, 127);
    _SendCmdToMotors(motor, DRV_CMD_BCK, speed_set);
  }

  volatile uint8_t rotation_set = 0;

  if (rotation >= 0)
  {
    rotation = _LogCurve(rotation, CURVE_CONSTANT_TURN);
    rotation_set = _Map(rotation, 0, 500, 0, 127);
    _SendCmdToMotors(motor, DRV_CMD_LEFT, rotation_set);
  }
  else
  {
    rotation = -1 * rotation;
    rotation = _LogCurve(rotation, CURVE_CONSTANT_TURN);
    rotation_set = _Map(rotation, 0, 500, 0, 127);
    _SendCmdToMotors(motor, DRV_CMD_RIGHT, rotation_set);
  }
}

void PER_Motors_SetTarget(PER_Motors_t* motor, int16_t speed, int16_t rotation)
{
  motor->speed_target = CLAMP_SPEED(speed);
  motor->turn_target = CLAMP_SPEED(rotation);
}

void PER_Motors_Stop(PER_Motors_t* motor)
{
  motor->speed_target = 0;
  motor->speed_current = 0;
  motor->turn_target = 0;
  motor->turn_current = 0;
  _SendCmdToMotors(motor, DRV_CMD_FWD, 0);
  _SendCmdToMotors(motor, DRV_CMD_LEFT, 0);
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

int16_t _Map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int16_t _LogCurve(int16_t power, float curve_const)
{
  if (!power)
    return 0;

  float volatile power_f = power;

  power_f = power_f * (curve_const/9.f) + powf(power_f, power_f / 500.f) * (9.f - curve_const) / 9;

  return (int16_t)power_f;
}

void _SendCmdToMotors(PER_Motors_t* motors, uint8_t cmd, uint8_t data)
{
  static uint8_t buffer[4] = { 0 };

  buffer[0] = motors->address;
  buffer[1] = cmd;
  buffer[2] = data;

  buffer[3] = (motors->address + cmd + data) & ((uint8_t)0b01111111);

  HAL_UART_Transmit(motors->uart, buffer, 4, 10);
}
