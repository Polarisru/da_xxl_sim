#ifndef HALL_H
#define HALL_H

#include "defines.h"

#define HALL_CHECK_BIT_A    (1UL << 0U)
#define HALL_CHECK_BIT_B    (1UL << 1U)
#define HALL_CHECK_BIT_C    (1UL << 2U)
#define HALL_CHECK_BIT_ALL  (HALL_CHECK_BIT_A | HALL_CHECK_BIT_B | HALL_CHECK_BIT_C)
#define HALL_CHECK_HALL_A   (1UL << 2U)
#define HALL_CHECK_HALL_B   (1UL << 1U)
#define HALL_CHECK_HALL_C   (1UL << 0U)
#define HALL_CHECK_WIND_A   (1UL << 5U)
#define HALL_CHECK_WIND_B   (1UL << 4U)
#define HALL_CHECK_WIND_C   (1UL << 3U)

#define HALL_COMM_LEN       6U

void EIC_Handler(void);
uint8_t HALL_ReadValues(void);
void HALL_Configuration(void);
void HALL_Start(void);
void HALL_Stop(void);
bool HALL_IsReady(void);
extern uint8_t HALL_CheckSensors(const uint8_t *data);
extern uint8_t HALL_CheckWindings(const uint8_t *data, uint8_t dir);
uint8_t HALL_DoCheck(void);
uint8_t HALL_DoAnalyze(void);

#endif
