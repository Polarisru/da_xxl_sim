#ifndef GLOBAL_H
#define GLOBAL_H

#include "defines.h"
#include "rtos.h"

/**< EEPROM variables */
extern uint8_t  EE_DevId;
extern uint8_t  EE_CanBitrate;
extern uint8_t  EE_Baudrate;
extern uint16_t EE_CanId;
extern uint8_t  EE_Kd;
extern uint8_t  EE_Kp;
extern uint8_t  EE_Ki;
extern uint8_t  EE_PWM_Min;
extern uint8_t  EE_FreshCntLevel;
extern uint8_t  EE_PWM_Max;
extern uint8_t  EE_PWM_Safe;
extern uint8_t  EE_SaverTime;
extern uint8_t  EE_SaverFrame;
extern uint8_t  EE_Sensor_DB;
extern uint8_t  EE_Options;
extern uint8_t  EE_PWM_Test;
extern uint8_t  EE_StationId;
extern int16_t  EE_ZeroPos;
extern uint16_t EE_LossPos;
extern uint8_t  EE_LossTime;
extern uint8_t  EE_LossBehavior;
extern uint8_t  EE_MaxMotorTemp;
extern uint8_t  EE_MotorTempHyst;
extern int8_t   EE_MagSensorOffs;
extern uint8_t  EE_PowerDamper;
extern uint8_t  EE_NotchWidth;
extern uint8_t  EE_NotchDepth;
extern uint8_t  EE_Options2;
extern uint8_t  EE_ServoId;
extern uint8_t  EE_HeaterTemp;
extern uint8_t  EE_MaxCurrent;
extern uint8_t  EE_MaxCurrentCW;
extern uint16_t EE_NotchFreq1;
extern uint16_t EE_NotchFreq2;
extern uint8_t  EE_PosErrTime;
extern uint8_t  EE_PosErrAngle;
extern uint8_t  EE_ResponseDelay;

/**< List of global variables */
extern uint8_t  GLOBAL_InternalID;
extern int16_t  GLOBAL_TargetPos;
extern int16_t  GLOBAL_RealPos;
extern int16_t  GLOBAL_RawPos;
extern uint16_t GLOBAL_MagnetValue;
extern uint8_t  GLOBAL_Power;
extern uint32_t GLOBAL_WorkingTime;
extern uint16_t GLOBAL_MyStatus;
extern uint16_t GLOBAL_ExtStatus;
extern uint16_t GLOBAL_ExternalStatus;
extern uint8_t  GLOBAL_DoStartBootloader;
extern uint8_t  GLOBAL_DoSensorCalibration;
extern bool     GLOBAL_IsMaster;
#ifdef DEF_DUPLEX
  extern uint8_t  GLOBAL_PartnerId;
  extern uint16_t GLOBAL_MagnetExtHall;
  extern uint8_t  GLOBAL_ExtHallStatus;
  extern uint8_t  GLOBAL_WaitTN;
  extern uint8_t  GLOBAL_MyTN;
  extern uint8_t  GLOBAL_PartnerI;
  extern uint8_t  GLOBAL_PartnerU1;
  extern uint8_t  GLOBAL_PartnerU2;
  extern uint8_t  GLOBAL_PartnerTempM;
  extern uint8_t  GLOBAL_PartnerTempP;
  extern uint8_t  GLOBAL_PartnerRH;
  extern bool     GLOBAL_PartnerIsMaster;
  extern uint16_t GLOBAL_PartnerMagnet;
  extern int16_t  GLOBAL_PartnerTargetPos;
  extern uint16_t GLOBAL_PartnerStatus;
  extern uint16_t GLOBAL_PartnerExtStatus;
  extern uint8_t  GLOBAL_PartnerMotorDir;
  extern uint8_t  GLOBAL_PartnerPower;
  extern uint16_t GLOBAL_PartnerCRC;
  extern uint8_t  GLOBAL_SlaveMotorDir;
  extern uint8_t  GLOBAL_SlavePower;
  extern uint8_t  GLOBAL_ForceSlaveMode;
  extern bool     GLOBAL_ProcessHostRequest;
  extern bool     GLOBAL_MasterRequest;
  extern bool     GLOBAL_PartnerMasterRequest;
  extern uint8_t  GLOBAL_PartnerMaxPwmValue;
  extern uint8_t  GLOBAL_RoleForce;
#endif
extern uint8_t  GLOBAL_DoDefault;
extern bool     GLOBAL_MotorExtMode;
extern bool     GLOBAL_IsMemoryOk;
extern uint8_t  GLOBAL_Windings;
extern bool     GLOBAL_ErrMotor;
extern bool     GLOBAL_ErrMagnet;
extern bool     GLOBAL_ErrFreshness;

#ifndef DEF_UNITTEST
  //SemaphoreHandle_t xSemaphoreEEPROM;
  extern QueueHandle_t xQueueRS485;
  extern QueueHandle_t xQueueCAN;
  extern TaskHandle_t xTaskInet;
  extern TaskHandle_t xTaskRS485;
  extern TaskHandle_t xTaskMain;
#endif

extern uint32_t crc32_build;

#endif
