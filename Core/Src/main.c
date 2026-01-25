/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "adc.h"
#include "can.h"
#include "dma.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#include "Sensors.h"
#include "daq_payload.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float* flow_rate;
// static int* isRxed;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t FAN_PWM;
extern uint8_t PUMP_PWM;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  /* USER CODE BEGIN 1 */
  uint16_t voltage;
  uint16_t result;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SEGGER_RTT_Init();
  SEGGER_RTT_WriteString(0, "Reached main\r\n");

  SEGGER_RTT_WriteString(0, "HAL_Init OK\r\n");

  SEGGER_RTT_WriteString(0, "Before SystemClock_Config\r\n");
  SystemClock_Config();
  SEGGER_RTT_WriteString(0, "After SystemClock_Config\r\n");
  /* USER CODE END Init */

  /* Configure the system clock */
  // SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // HAL_Delay(500);
  SEGGER_RTT_printf(0, "start\n");
  flow_rate = fetch_flowrate();
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim1, TIM_CHANNEL_1);
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&voltage, 1) != HAL_OK) {
    SEGGER_RTT_printf(0, "ADC initialization error!\n");
  }

  bsp_can1_filter_config();
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // FAN
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // PUMP

  SetFanDuty(60);   // FAN
  SetPumpDuty(70);  // PUMP

  DS18B20_SampleTemp(&huart1);  // starting first time is to avoid reading null value
  // DS18B20_SampleTemp(&huart2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    SEGGER_RTT_printf(0, "tick=%lu\r\n", HAL_GetTick());
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    int t_pre = DS18B20_ReadTemp(&huart1);
    SEGGER_RTT_printf(0, "time: %d, temperature_1: %d\n", HAL_GetTick(), t_pre);
    int t_post = DS18B20_ReadTemp(&huart2);
    SEGGER_RTT_printf(0, "time: %d, temperature_2: %d\n", HAL_GetTick(), t_post);

    DAQData_to_DataLogger[DAQ_PRE_TEMP] = (uint8_t)t_pre;
    DAQData_to_DataLogger[DAQ_POST_TEMP] = (uint8_t)t_post;

    DS18B20_SampleTemp(&huart1);
    DS18B20_SampleTemp(&huart2);

    result = voltage * 3300 / 4095;
    SEGGER_RTT_printf(0, "voltage = %d\n", result);

    SEGGER_RTT_printf(0, "flow rate = %d mL/min\n", (int)*flow_rate);

    HAL_Delay(500);

    DAQData_to_DataLogger[DAQ_PRESSURE_H] = (result >> 8) & 0xFF;  // PRESSURE(H)
    DAQData_to_DataLogger[DAQ_PRESSURE_L] = result & 0xFF;         // PRESSURE(L)

    DAQData_to_DataLogger[DAQ_FLOW_H] = ((int)*flow_rate >> 8) & 0xFF;  // FLOW_RATE(H)
    DAQData_to_DataLogger[DAQ_FLOW_L] = (int)*flow_rate & 0xFF;         // FLOW_RATE(L)
    DAQData_to_DataLogger[DAQ_FAN_PWM] = FAN_PWM;                       // FAN_PWM
    DAQData_to_DataLogger[DAQ_PUMP_PWM] = PUMP_PWM;                     // PUMP_PWM
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (g_daq_enabled) {
      if (!CAN_SendMsg(CA_DAQ_DATA, DAQData_to_DataLogger)) {
        SEGGER_RTT_printf(0, "CAN TX timeout or fail\n");
      }
    }
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
