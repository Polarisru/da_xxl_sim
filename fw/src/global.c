#include "global.h"

/**< EEPROM variables */
uint8_t  EE_DevId;          // CAN Id of the servo
uint8_t  EE_CanBitrate;     // bitrate for CAN bus
uint8_t  EE_Baudrate;       // baudrate for RS485 bus
uint16_t EE_CanId;          // CAN StdId for bootloader
uint8_t  EE_Kd;             // Differential coefficient for PID-regulator
uint8_t  EE_Kp;             // Proportional coefficient for PID-regulator
uint8_t  EE_Ki;             // Integral coefficient for PID-regulator
uint8_t  EE_PWM_Min;        // minimal value of motor PWM
uint8_t  EE_FreshCntLevel;  // freshness counter threshold (>15 - control is Off)
uint8_t  EE_PWM_Max;        // maximal value of motor PWM
uint8_t  EE_PWM_Safe;       // motor saver PWM
uint8_t  EE_SaverTime;      // time to enter to saver mode
uint8_t  EE_SaverFrame;     // position deviation to return from saver mode
uint8_t  EE_Sensor_DB;      // Magnet sensor deadband
uint8_t  EE_Options;        // options register cell
uint8_t  EE_PWM_Test;       // use as test power for BLDC self-test routine
uint8_t  EE_StationId;      // RS485 Id of the servo
int16_t  EE_ZeroPos;        // production zero position value
uint16_t EE_LossPos;        // Loss position
uint8_t  EE_LossTime;       // Loss timeout
uint8_t  EE_LossBehavior;   // Loss behavior
uint8_t  EE_MaxMotorTemp;   // maximal temperature of the motor, degrees
uint8_t  EE_MotorTempHyst;  // hysteresis for maximal motor temperature, degrees
int8_t   EE_MagSensorOffs;  // magnet sensor offset (for calibration)
uint8_t  EE_PowerDamper;    // power damping value (0...255, 0 - no damping, 255 - maximal damping, no movement)
uint8_t  EE_NotchWidth;     // notch filter relative width
uint8_t  EE_NotchDepth;     // notch filter attenuation
uint8_t  EE_Options2;       // options register cell #2
uint8_t  EE_ServoId;        // Servo ID (0x58)
uint8_t  EE_HeaterTemp;     // temperature for heater activation
uint8_t  EE_MaxCurrent;     // maximal current value (for current limiter)
uint8_t  EE_MaxCurrentCW;     // maximal current value (for current limiter)
uint16_t EE_NotchFreq1;     // notch filter 1 frequency (in 1/100 rad/sec)
uint16_t EE_NotchFreq2;     // notch filter 2 frequency (in 1/100 rad/sec)
uint8_t  EE_PosErrTime;     // position error time
uint8_t  EE_PosErrAngle;    // position error angle
uint8_t  EE_ResponseDelay;  // response delay (RS485)

/**< List of global variables */
uint8_t  GLOBAL_InternalID;
int16_t  GLOBAL_TargetPos;
int16_t  GLOBAL_RealPos;
int16_t  GLOBAL_RawPos;
uint16_t GLOBAL_MagnetValue;
uint8_t  GLOBAL_Power;
uint32_t GLOBAL_WorkingTime;
uint16_t GLOBAL_MyStatus;
uint16_t GLOBAL_ExtStatus;
uint16_t GLOBAL_ExternalStatus;
uint8_t  GLOBAL_DoStartBootloader;
uint8_t  GLOBAL_DoSensorCalibration;
bool     GLOBAL_IsMaster;
#ifdef DEF_DUPLEX
  uint8_t  GLOBAL_PartnerId;
  uint16_t GLOBAL_MagnetExtHall;
  uint8_t  GLOBAL_ExtHallStatus;
  uint8_t  GLOBAL_WaitTN;
  uint8_t  GLOBAL_MyTN;
  uint8_t  GLOBAL_PartnerI;
  uint8_t  GLOBAL_PartnerU1;
  uint8_t  GLOBAL_PartnerU2;
  uint8_t  GLOBAL_PartnerTempM;
  uint8_t  GLOBAL_PartnerTempP;
  uint8_t  GLOBAL_PartnerRH;
  bool     GLOBAL_PartnerIsMaster;
  uint16_t GLOBAL_PartnerMagnet;
  int16_t  GLOBAL_PartnerTargetPos;
  uint16_t GLOBAL_PartnerStatus;
  uint16_t GLOBAL_PartnerExtStatus;
  uint8_t  GLOBAL_PartnerMotorDir;
  uint8_t  GLOBAL_PartnerPower;
  uint16_t GLOBAL_PartnerCRC;
  uint8_t  GLOBAL_SlaveMotorDir;
  uint8_t  GLOBAL_SlavePower;
  uint8_t  GLOBAL_ForceSlaveMode;
  bool     GLOBAL_ProcessHostRequest;
  bool     GLOBAL_MasterRequest;
  bool     GLOBAL_PartnerMasterRequest;
  uint8_t  GLOBAL_PartnerMaxPwmValue;
  uint8_t  GLOBAL_RoleForce;
#endif
uint8_t  GLOBAL_DoDefault;
bool     GLOBAL_MotorExtMode;
bool     GLOBAL_IsMemoryOk;
uint8_t  GLOBAL_Windings;
bool     GLOBAL_ErrMotor;
bool     GLOBAL_ErrMagnet;
bool     GLOBAL_ErrFreshness;

#ifndef DEF_UNITTEST
  /**< Semaphores for RTOS */
  /**< Queues to collect messages */
  QueueHandle_t xQueueRS485;
  QueueHandle_t xQueueCAN;
  /**< IDs for RTOS tasks */
  TaskHandle_t xTaskInet;
  TaskHandle_t xTaskRS485;
  TaskHandle_t xTaskMain;
#endif
