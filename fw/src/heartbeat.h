#ifndef HEARTBEAT_H
#define HEARTBEAT_H

#include "heartbeat.h"

#define HEARTBEAT_DEFAULT_MAX_VALUE   5
#define HEARTBEAT_NEW_ROLE_VALUE      -5
#define HEARTBEAT_DOWN_CNT_MAX_VALUE  3    // number of heartbeats to reset heartbeat counter


#define HEARTBEAT_PORT      GPIO_PORTB
#define HEARTBEAT_PIN       23

extern void HEARTBEAT_Configuration(bool out);
extern void HEARTBEAT_SetLimit(int8_t value);
extern void HEARTBEAT_Enable(void);
extern void HEARTBEAT_Disable(void);
extern bool HEARTBEAT_Process(void);

#endif
