/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
uint32_t CAN_ID_DAQEN=0x70;
uint32_t CAN_ID_DAQData=0x71;
uint32_t Rx_CANID = 0 ;
uint8_t DAQData_to_DataLogger[8];
uint8_t DAQEN[8];
uint8_t CAN_RxData[8];
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 24;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
  /* USER CODE BEGIN CAN1:USB_HP_CAN1_TX_IRQn disable */
    /**
    * Uncomment the line below to disable the "USB_HP_CAN1_TX_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn); */
  /* USER CODE END CAN1:USB_HP_CAN1_TX_IRQn disable */

  /* USER CODE BEGIN CAN1:USB_LP_CAN1_RX0_IRQn disable */
    /**
    * Uncomment the line below to disable the "USB_LP_CAN1_RX0_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn); */
  /* USER CODE END CAN1:USB_LP_CAN1_RX0_IRQn disable */

    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_SendMsg(uint16_t msgID, uint8_t *Data)
{
  CAN_TxHeaderTypeDef   TxHeader;
	TxHeader.StdId = msgID;			//stdID
	TxHeader.RTR = CAN_RTR_DATA;		
	TxHeader.IDE = CAN_ID_STD;		//standard mode
	TxHeader.DLC =8;   				    //data length
	TxHeader.TransmitGlobalTime = DISABLE;
	

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) < 1) {
	}  //waiting valid mailbox 
	uint32_t TxMailbox;		
	
	if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, Data, &TxMailbox) != HAL_OK)
    {
        
    }
}

uint8_t bsp_can1_filter_config(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterActivation = ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK; //mask mode
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterBank = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0; // set FIFO mode
    filter.FilterIdLow = 0;
    filter.FilterIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterMaskIdHigh = 0;  //All 0 means no messages are filtered and all messages are received.
    HAL_CAN_ConfigFilter(&hcan, &filter); 
    
}

static CAN_RxHeaderTypeDef RxMessage;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

		uint8_t data[8];
		HAL_StatusTypeDef status;
		
		status = HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, data);
		if (HAL_OK == status){

			uint16_t rx_id=RxMessage.StdId;
      if (rx_id == 0x70)
      {
          uint8_t enable_flag = data[0];
          if (enable_flag == 1) {
              DAQ_Enable();  //還沒寫
          } else {
              DAQ_Disable(); //還沒寫
          }
      }
      else if (rx_id == 0x71)
        {
            memcpy(CAN_RxData, data, 8);
            Rx_CANID = rx_id;
        }
		}
	
}

/* USER CODE END 1 */
