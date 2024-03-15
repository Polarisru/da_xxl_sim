#ifndef ANALOG_H
#define ANALOG_H

#include "defines.h"

enum {
  ADC_CHANNEL_CURRENT,
  ADC_CHANNEL_TEMP_MOTOR,
  ADC_CHANNEL_VOLTAGE1,
  ADC_CHANNEL_VOLTAGE2,
  ADC_CHANNEL_NUM
};

typedef struct {
  uint8_t port;
  uint8_t pin;
  uint8_t pin_mode;
  uint8_t channel;
} TAnalogChannel;

extern void ANALOG_SetChannel(uint8_t channel);
extern uint16_t ANALOG_GetValue(void);
extern void ANALOG_Configuration(void);

#endif
