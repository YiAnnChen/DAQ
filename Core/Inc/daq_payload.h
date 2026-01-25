#ifndef DAQ_PAYLOAD_H
#define DAQ_PAYLOAD_H

#include <stdint.h>

typedef enum {
  DAQ_PRE_TEMP = 0,
  DAQ_POST_TEMP,
  DAQ_PRESSURE_H,
  DAQ_PRESSURE_L,
  DAQ_FLOW_H,
  DAQ_FLOW_L,
  DAQ_FAN_PWM,
  DAQ_PUMP_PWM,

  DAQ_PAYLOAD_LEN
} DaqPayloadIndex_t;

#define U16_HI(x) ((uint8_t)(((uint16_t)(x) >> 8) & 0xFFU))
#define U16_LO(x) ((uint8_t)((uint16_t)(x) & 0xFFU))

#endif