#ifndef _SENSOR_H
#define _SENSOR_H

#include <stdlib.h>
#include "main.h"
#include "SEGGER_RTT.h"


// water flow rate
float* fetch_flowrate();

// Temperature sensors onewire communication implementation
void uart_init(UART_HandleTypeDef* huart, int baudrate);
void DS18B20_WriteByte(UART_HandleTypeDef* huart, uint8_t data);
uint8_t DS18B20_ReadByte(UART_HandleTypeDef* huart);
uint8_t DS18B20_Init(UART_HandleTypeDef* huart);
void DS18B20_SampleTemp(UART_HandleTypeDef* huart);
int DS18B20_ReadTemp(UART_HandleTypeDef* huart);


// 
#endif