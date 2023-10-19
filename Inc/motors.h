#ifndef PERIPHERALS_MOTORS_H_
#define PERIPHERALS_MOTORS_H_

#include "usart.h"
#include "utility.h"

typedef struct
{
  UART_HandleTypeDef* uart;

  const uint8_t address;

  int16_t speed_current;
  int16_t speed_target;

  int16_t turn_current;
  int16_t turn_target;

  uint32_t last_process;
} PER_Motors_t;

PER_Motors_t PER_Motors_Init(UART_HandleTypeDef* huart, uint8_t addr);
void PER_Motors_Activate(PER_Motors_t* motor);

void PER_Motors_Process(PER_Motors_t* motor);
void PER_Motors_SetTarget(PER_Motors_t* motor, int16_t speed, int16_t rotation);
void PER_Motors_Stop(PER_Motors_t* motor);

#endif // PERIPHERALS_MOTORS_H_
