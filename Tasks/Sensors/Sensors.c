#include "Sensors.h"

static volatile uint8_t ow_rx_done = 0;  // ow = OneWire
static volatile uint8_t ow_tx_done = 0;
static UART_HandleTypeDef* ow_active_uart = NULL;

static uint8_t RxData[8];
static float flow_rate;

float* fetch_flowrate() { return &flow_rate; }

static inline int int_to_int(uint8_t k) {
  if (k == 0) return 0;
  if (k == 1) return 1; /* optional */
  return (k % 2) + 10 * int_to_int(k / 2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == ow_active_uart) {
    ow_tx_done = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart == ow_active_uart) {
    ow_rx_done = 1;
  }
}

void uart_init(UART_HandleTypeDef* huart, int baudrate) {
  huart->Init.BaudRate = baudrate;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(huart) != HAL_OK) {
    Error_Handler();
  }
}

void DS18B20_WriteByte(UART_HandleTypeDef* huart, uint8_t data) {
  uint8_t TxBuffer[8];
  for (int i = 0; i < 8; i++) {
    if ((data & (1 << i)) != 0) {
      TxBuffer[i] = 0xFF;
    } else {
      TxBuffer[i] = 0;
    }
  }
  HAL_UART_Transmit(huart, TxBuffer, 8, 1000);
}

uint8_t DS18B20_ReadByte(UART_HandleTypeDef* huart) {
  uint8_t buffer[8];
  uint8_t value = 0;

  for (int i = 0; i < 8; i++) {
    buffer[i] = 0xFF;
    RxData[i] = 0x00;
  }

  // clean flag
  ow_rx_done = 0;
  ow_tx_done = 0;
  ow_active_uart = huart;

  // start DMA
  HAL_StatusTypeDef st_tx = HAL_UART_Transmit_DMA(huart, buffer, 8);
  HAL_StatusTypeDef st_rx = HAL_UART_Receive_DMA(huart, RxData, 8);

  if (st_tx != HAL_OK || st_rx != HAL_OK) {
    (void)HAL_UART_Abort(huart);
    ow_active_uart = NULL;
    SEGGER_RTT_printf(0, "start failed!\n");
    return 0xFF;
  }

  uint32_t start = HAL_GetTick();
  const uint32_t timeout_ms = 10;

  while (!ow_rx_done) {
    if ((HAL_GetTick() - start) > timeout_ms) {
      (void)HAL_UART_Abort(huart);
      ow_active_uart = NULL;
      SEGGER_RTT_printf(0, "DS18B20_ReadByte timeout on UART%1d!\n", (huart->Instance == USART1) ? 1 : 2);
      return 0xFF;
    }
  }

  for (int i = 0; i < 8; i++) {
    if (RxData[i] == 0xFF)  // if the pin is HIGH
    {
      value |= 1 << i;  // read = 1
    }
  }
  ow_active_uart = NULL;
  return value;
}

uint8_t DS18B20_Init(UART_HandleTypeDef* huart) {
  uint8_t ResetByte = 0xF0, PresenceByte;
  // LL_USART_SetBaudRate(huart->Instance, HAL_RCC_GetPCLK2Freq(), 9600);
  uart_init(huart, 9600);

  // Send reset pulse (0xF0)
  HAL_UART_Transmit(huart, &ResetByte, 1, 100);

  // Wait for the presence pulse
  if (HAL_UART_Receive(huart, &PresenceByte, 1, 20) != HAL_OK) {
    SEGGER_RTT_WriteString(0, "Presence RX timeout\r\n");
    return -1;
  }
  SEGGER_RTT_printf(0, "PresenceByte=0x%02X (ResetByte=0x%02X)\r\n", PresenceByte, ResetByte);

  // LL_USART_SetBaudRate(huart->Instance, HAL_RCC_GetPCLK2Freq(), 115200);
  uart_init(huart, 115200);

  if (PresenceByte != ResetByte) {
    // SEGGER_RTT_printf(0, "detected!\n");
    return 1;  // Presence pulse detected
  } else {
    // SEGGER_RTT_printf(0, "not detected!\n");
    return 0;  // No presence pulse detected
  }
}

void DS18B20_SampleTemp(UART_HandleTypeDef* huart) {
  DS18B20_Init(huart);
  DS18B20_WriteByte(huart, 0xCC);  // Skip ROM   (ROM-CMD)
  DS18B20_WriteByte(huart, 0x44);  // Convert T  (F-CMD)
}

int DS18B20_ReadTemp(UART_HandleTypeDef* huart) {
  uint8_t Temp_LSB, Temp_MSB;
  uint16_t Temp;
  int Temperature;

  DS18B20_Init(huart);
  DS18B20_WriteByte(huart, 0xCC);  // Skip ROM         (ROM-CMD)
  DS18B20_WriteByte(huart, 0xBE);  // Read Scratchpad  (F-CMD)
  Temp_LSB = DS18B20_ReadByte(huart);
  Temp_MSB = DS18B20_ReadByte(huart);
  // SEGGER_RTT_printf(0, "LSB: %d\n", int_to_int(Temp_LSB));
  // SEGGER_RTT_printf(0, "MSB: %d\n", int_to_int(Temp_MSB));
  Temp = ((Temp_MSB << 8)) | Temp_LSB;
  // SEGGER_RTT_printf(0, "x16: %d\n", Temp);
  Temperature = Temp >>= 4;
  return Temperature;
}
