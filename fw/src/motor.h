#ifndef MOTOR_H
#define MOTOR_H

#include "defines.h"

extern void MOTOR_Init(void);
extern bool MOTOR_IsSaverActive(void);
extern void MOTOR_Off(void);
extern uint8_t MOTOR_GetError(void);
extern void MOTOR_SetMode(uint8_t mode);
extern void MOTOR_SetSlaveMode(uint8_t mode);
extern void MOTOR_DoSlaveFree(bool yes);
extern uint8_t MOTOR_GetDir(void);
extern void MOTOR_SetExtPower(uint8_t power);
extern uint8_t MOTOR_GetExtPower(void);
extern uint8_t MOTOR_GetMode(void);
extern uint8_t MOTOR_GetSlaveMode(void);
extern uint8_t MOTOR_GetMaxPwmValue(void);
extern void MOTOR_DoTest(uint8_t *resp_1, uint8_t *resp_2);
extern int16_t MOTOR_CalcPower(void);
extern void MOTOR_ResetPID(void);
extern int16_t MOTOR_DoDamping(int16_t power_in);
extern void MOTOR_ProcessMagnet(void);
extern int16_t MOTOR_Saver(int16_t power_in);
extern uint8_t MOTOR_Heater(uint8_t power_in);
extern int16_t MOTOR_CurrentLimiter(uint8_t reversal, int16_t power_in, bool degraded);
extern uint8_t MOTOR_GetLimitPWM(void);
extern int8_t MOTOR_GetShaftSpeed(void);
extern int16_t MOTOR_GetErrPos(void);
extern void MOTOR_Task(void *pParameters);

#endif
