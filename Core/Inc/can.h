/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>

#include "SEGGER_RTT.h"

#pragma once
#include <daq_payload.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
extern uint32_t CA_DAQ_EN;    // 0x70
extern uint32_t CA_DAQ_DATA;  // 0x71
extern uint8_t DAQData_to_DataLogger[DAQ_PAYLOAD_LEN];
extern uint8_t DAQEN[8];
extern uint8_t CAN_RxData[8];

extern volatile bool g_daq_enabled;
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
bool CAN_SendMsg(uint16_t msgID, uint8_t* Data);
uint8_t bsp_can1_filter_config(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

