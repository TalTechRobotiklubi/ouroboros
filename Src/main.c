/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "control_state.h"

#include "weapon.h"
#include "motors.h"

#define AXIS_MID 1500

#define FLAG_HAS_2 (1 << 0)
#define FLAG_HAS_4 (1 << 1)
#define FLAG_HAS_FINAL (1 << 2)

volatile uint8_t itbuff[40] = { 0 };
volatile uint8_t flags = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* uart)
{
  if (!flags)
  {
    if (itbuff[0] == 0x20)
    {
      flags = FLAG_HAS_2;
      HAL_UART_Receive_IT(uart, itbuff + 1, 1);
    }
    else
    {
      HAL_UART_Receive_IT(uart, itbuff, 1);
    }
  }
  else if (flags == FLAG_HAS_2)
  {
    if (itbuff[1] != 0x40)
    {
      flags = 0;
      HAL_UART_Receive_IT(uart, itbuff, 1);
    }
    else
    {
      flags = FLAG_HAS_4;
      HAL_UART_Receive_IT(uart, itbuff + 2, 30);
    }
  }
  else
  {
    flags = FLAG_HAS_FINAL;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* uart)
{
  __NOP();
}

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

ReceiverState_t HandleReceiverMessage();
void CheckUartInterruptStatus();
void HandleMotorsUpdate(ControlState_t* ctrl_state, ReceiverState_t* commands, PER_Motors_t* motors);
void HandleWeaponsUpdate(ControlState_t* ctrl_state, PER_Weapon_t* wpn);
void ActivateRobot(PER_Motors_t* motors);
void DeactivateRobot(PER_Motors_t* motors, PER_Weapon_t* wpn);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Init(&huart2);
  HAL_UART_Init(&huart1);
  HAL_UART_Receive_IT(&huart1, itbuff, 1);

  HAL_ADC_Start(&hadc2);

  PER_IOPair_t wpn_io = {
      .port = OUT_WPN_EN_GPIO_Port,
      .pin = OUT_WPN_EN_Pin
  };

  PER_Weapon_t weapon = PER_Weapon_Init(wpn_io);
  PER_Motors_t motors = PER_Motors_Init(&huart2, 128);
  ControlState_t current_state = InitializeControlState();

  while (flags != FLAG_HAS_FINAL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    bool has_new_commands = false;
    ReceiverState_t new_commands = { 0 };

    if (flags == FLAG_HAS_FINAL)
    {
       new_commands = HandleReceiverMessage();
       has_new_commands = true;
    }
    else
    {
      CheckUartInterruptStatus();
    }

    if (RequiresConnectionCheck(&current_state))
    {
      if (!DoConnectionCheck(&current_state))
      {
        if (current_state.robot_active)
        {
          DeactivateRobot(&motors, &weapon);
          current_state.robot_active = false;
          current_state.engines_active = false;
          current_state.weapon_active = false;
        }

        continue;
      }
    }

    if (!has_new_commands)
      continue;

    if (current_state.robot_active)
    {
      PER_Motors_Process(&motors);
      PER_Weapon_Process(&weapon);

      ApplyControlState(&current_state, &new_commands);

      if (new_commands.switch_c == ReceiverSwitchUp)
      {
        DeactivateRobot(&motors, &weapon);
        continue;
      }

      HandleMotorsUpdate(&current_state, &new_commands, &motors);

      HandleWeaponsUpdate(&current_state, &weapon);
    }
    else
    {
      ApplyControlState(&current_state, &new_commands);

      if (new_commands.switch_c == ReceiverSwitchDown)
        ActivateRobot(&motors);
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

ReceiverState_t HandleReceiverMessage()
{
  ReceiverState_t rec = DecodeReceiverState(itbuff);

  flags = 0;

  for (uint8_t i = 0; i < 40; i++)
  {
    itbuff[i] = 0;
  }

  HAL_UART_Receive_IT(&huart1, itbuff, 1);

  return rec;
}

void CheckUartInterruptStatus()
{
  uint8_t uart_status = HAL_UART_GetState(&huart1);

  if (uart_status == 32)
  {
    flags = 0;
    HAL_UART_Receive_IT(&huart1, itbuff, 1);
  }
}

int16_t CalculateRobotSpeed(ReceiverState_t* commands)
{
  return commands->right_vertical - AXIS_MID;
}

int16_t CalculateRobotTurn(ReceiverState_t* commands)
{
  return commands->right_horizontal - AXIS_MID;
}

void HandleMotorsUpdate(ControlState_t* ctrl_state, ReceiverState_t* commands, PER_Motors_t* motors)
{
  if (ctrl_state->engines_active)
  {
    int16_t new_speed = CalculateRobotSpeed(commands);
    int16_t new_turn = CalculateRobotTurn(commands);

    PER_Motors_SetTarget(motors, new_speed, new_turn);
  }
  else
  {
    PER_Motors_SetTarget(motors, 0, 0);
  }
}

void HandleWeaponsUpdate(ControlState_t* ctrl_state, PER_Weapon_t* wpn)
{
  if (ctrl_state->weapon_active && !wpn->is_active)
    PER_Weapon_Enable(wpn);
  else if (!ctrl_state->weapon_active && wpn->is_active)
    PER_Weapon_Disable(wpn);
}

void ActivateRobot(PER_Motors_t* motors)
{
  HAL_GPIO_WritePin(OUT_MOTORS_RESET_GPIO_Port, OUT_MOTORS_RESET_Pin, GPIO_PIN_SET);

  HAL_Delay(5000);

  PER_Motors_Activate(motors);
}

void DeactivateRobot(PER_Motors_t* motors, PER_Weapon_t* wpn)
{
  PER_Weapon_Disable(wpn);
  PER_Motors_Stop(motors);

  HAL_Delay(500);

  HAL_GPIO_WritePin(OUT_MOTORS_RESET_GPIO_Port, OUT_MOTORS_RESET_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
