/*
 * utility.h
 *
 *  Created on: 2 Nov 2018
 *      Author: erkim
 */

#ifndef PERIPHERALS_UTILITY_H_
#define PERIPHERALS_UTILITY_H_

#include "gpio.h"

#define _BV(x) (1 << x)
#define CLAMP(h, l, x) (x > h ? h : (x < l ? l : x))

typedef struct
{
  GPIO_TypeDef* port;
  uint16_t pin;
} PER_IOPair_t;

void PER_UTIL_PrintToUart(UART_HandleTypeDef* uart, const char* fmt, ...);

inline uint32_t PER_UtilGetMicros()
{
  volatile uint32_t micros;
  micros = (HAL_GetTick() * 1000) + ((8000 - SysTick->VAL) / 8);

  return micros;
}

inline void PER_UtilDelayMicros(const uint32_t delay)
{
  const uint32_t time_now = PER_UtilGetMicros();

  if ((delay + time_now) < 0xFFFFFFFF)
  {
    while ((PER_UtilGetMicros() - time_now) < delay);
  }
}

inline void PER_UtilSetIO(const PER_IOPair_t* p)
{
  HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_SET);
}

inline void PER_UtilResetIO(const PER_IOPair_t* p)
{
  HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_RESET);
}

inline void PER_UtilSetCS(const PER_IOPair_t* p, const uint32_t delay_us)
{
  HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_RESET);

  if (delay_us)
    PER_UtilDelayMicros(delay_us);
}

inline void PER_UtilUnsetCS(const PER_IOPair_t* p)
{
  HAL_GPIO_WritePin(p->port, p->pin, GPIO_PIN_SET);
}

#endif /* PERIPHERALS_UTILITY_H_ */
