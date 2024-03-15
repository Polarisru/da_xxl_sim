#ifndef MONITOR_H
#define MONITOR_H

#define MONITOR_TEMP_MIN    10U
#define MONITOR_TEMP_MAX    200U

#define MONITOR_TEMP_LOW    0U
#define MONITOR_TEMP_HIGH   255U

enum {
  MONITOR_PARAM_CURRENT,
  MONITOR_PARAM_VOLTAGE1,
  MONITOR_PARAM_VOLTAGE2,
  MONITOR_PARAM_TEMP_MOTOR,
  MONITOR_PARAM_TEMP_PCB,
  MONITOR_PARAM_BLDC,
  MONITOR_PARAM_LAST
};

enum {
  MONITOR_U_CH1,
  MONITOR_U_CH2
};

enum {
  MONITOR_TEMP_MOTOR,
  MONITOR_TEMP_PCB
};

uint8_t MONITOR_GetCurrent(void);
void MONITOR_SetCurrent(uint8_t value);
uint8_t MONITOR_GetMaxCurrent(void);
uint8_t MONITOR_GetVoltage(uint8_t channel);
void MONITOR_SetVoltage(uint8_t channel, uint8_t value);
bool MONITOR_GetVoltageStatus(void);
bool MONITOR_GetSupply(void);
uint8_t MONITOR_GetTemperature(uint8_t channel);
void MONITOR_SetTemperature(uint8_t channel, uint8_t value);
bool MONITOR_GetTemperatureStatus(uint8_t channel);
void MONITOR_Task(void *pParameters);

#endif
