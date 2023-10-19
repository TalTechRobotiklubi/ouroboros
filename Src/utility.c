
#include "utility.h"

#include <stdlib.h>
#include <stdarg.h>

void PER_UTIL_PrintToUart(UART_HandleTypeDef* uart, const char* fmt, ...)
{
  static char buff[400] = { 0 };

  va_list arg_ptr;
  va_start(arg_ptr, fmt);

  const int16_t len = vsprintf(buff, fmt, arg_ptr);

  va_end(arg_ptr);

  if (len > 0)
    HAL_UART_Transmit(uart, (uint8_t*)buff, len, 100);
}
