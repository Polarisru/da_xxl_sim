#include <math.h>
#include "drivers.h"
#include "drv8320.h"
#include "global.h"
#include "hall.h"
#include "magnet.h"
#include "monitor.h"
#include "motor.h"
#include "notch.h"
#include "utils.h"

#if defined DEF_DA58
  #define MOTOR_ACT_PIN       2
  #define MOTOR_ACT_PORT      GPIO_PORTB
#endif

static uint16_t MOTOR_JammedCnt;
static int16_t  MOTOR_PosJammed;
static int16_t  MOTOR_TargetPosJammed;
static uint16_t MOTOR_Error;
static uint8_t  MOTOR_Mode;
static uint8_t  MOTOR_MaxPwmValue;
static uint8_t  MOTOR_LimitPWM;
static int16_t  errPosLast;
static int16_t  i_sum;
static int8_t   MOTOR_ShaftSpeed;
static int16_t  MOTOR_ErrPos;
static uint8_t  MOTOR_Dir;
static uint8_t  MOTOR_ExtPower;
#ifdef DEF_DUPLEX
static uint8_t  MOTOR_SlaveMode;
static bool     MOTOR_SlaveFree;
#endif

/** \brief Initialize motor module
 *
 * \return void
 *
 */
#ifndef DEF_UNITTEST
void MOTOR_Init(void)
{
//  uint8_t i;

  MOTOR_Dir = MOTOR_DIR_CW;

  #if defined DEF_DA58
    /**< Initialize MCU pins: overvoltage pin */
    GPIO_ClearPin(MOTOR_ACT_PORT, MOTOR_ACT_PIN);
    GPIO_SetDir(MOTOR_ACT_PORT, MOTOR_ACT_PIN, false);
    GPIO_SetFunction(MOTOR_ACT_PORT, MOTOR_ACT_PIN, GPIO_PIN_FUNC_OFF);
  #endif

  HALL_Configuration();

  SPI_Init();

  /**< Setup motor driver */
  DRV8320_Configuration();

  /**< Setup PWM */
  PWM_Init();
  PWM_Set(0);
  PWM_Enable(0);

  DRV8320_Enable(true);
  vTaskDelay(pdMS_TO_TICKS(2));
  /**< Toggling Enable pin for about 2ms to get high voltage level? */
//  for (i = 0; i < 24; i++)
//  {
//    TIMER_DelayUs(100);
//    DRV8320_EnableToggle();
//  }
  DRV8320_Activate();

  NOTCH_RecalcCoeff(NOTCH_NUM_1, EE_NotchFreq1, EE_NotchWidth, EE_NotchDepth);
  NOTCH_RecalcCoeff(NOTCH_NUM_2, EE_NotchFreq2, EE_NotchWidth, EE_NotchDepth);
}
#endif

/** \brief Reset motor jam
 *
 * \return
 *
 */
static void MOTOR_ResetJam(void)
{
  MOTOR_JammedCnt = 0;
  MOTOR_PosJammed = 0;
  MOTOR_TargetPosJammed = 0;
}

/** \brief Check Motor Saver status
 *
 * \return True if motor saver is active
 *
 */
#ifndef DEF_UNITTEST
bool MOTOR_IsSaverActive(void)
{
  return ((EE_SaverTime > 0U) && (MOTOR_JammedCnt > (EE_SaverTime * 1000U * pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS))));
}
#endif

/** \brief Switch motor off
 *
 * \return void
 *
 */
#ifndef DEF_UNITTEST
//void MOTOR_Off(void)
//{
//  MOTOR_On = false;
//  PWM_Set(0);
//}
#endif

/** \brief Get motor error
 *
 * \return Error code as uint8_t
 *
 */
uint8_t MOTOR_GetError(void)
{
  uint8_t uval8 = 0;

  if ((MOTOR_Error & DRV8320_ERR_GATE_HIGHA) > 0U)
  {
    uval8 |= (1UL << 5U);
  }
  if ((MOTOR_Error & DRV8320_ERR_GATE_LOWA) > 0U)
  {
    uval8 |= (1UL << 2U);
  }
  if ((MOTOR_Error & DRV8320_ERR_GATE_HIGHB) > 0U)
  {
    uval8 |= (1UL << 4U);
  }
  if ((MOTOR_Error & DRV8320_ERR_GATE_LOWB) > 0U)
  {
    uval8 |= (1UL << 1U);
  }
  if ((MOTOR_Error & DRV8320_ERR_GATE_HIGHC) > 0U)
  {
    uval8 |= (1UL << 3U);
  }
  if ((MOTOR_Error & DRV8320_ERR_GATE_LOWC) > 0U)
  {
    uval8 |= (1UL << 0U);
  }

  return uval8;
}

/** \brief Set motor mode
 *
 * \param [in] mode Motor mode to set
 * \return void
 *
 */
void MOTOR_SetMode(uint8_t mode)
{
  if (mode > MOTOR_MODE_GOCCW)
  {
    return;
  }
  MOTOR_Mode = mode;
  switch (mode)
  {
    case MOTOR_MODE_RUN1:
      if (GLOBAL_MotorExtMode == true)
      {
        MOTOR_MaxPwmValue = MOTOR_ExtPower;
      } else
      {
        MOTOR_MaxPwmValue = EE_PWM_Max;
      }
      #ifdef DEF_DUPLEX
      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
      #endif
      break;
    case MOTOR_MODE_RUN2:
      if (GLOBAL_MotorExtMode == true)
      {
        MOTOR_MaxPwmValue = (uint8_t)((uint16_t)MOTOR_ExtPower * INV_SQRT2);
      } else
      {
        MOTOR_MaxPwmValue = (uint8_t)((uint16_t)EE_PWM_Max * INV_SQRT2);
      }
      break;
    case MOTOR_MODE_FREE:
      MOTOR_MaxPwmValue = 0;
      break;
    case MOTOR_MODE_BRAKE:
      MOTOR_MaxPwmValue = 0;
      #ifdef DEF_DUPLEX
      GLOBAL_ForceSlaveMode = SLAVE_MODE_BRAKE;
      #endif
      break;
    case MOTOR_MODE_GOCW:
    case MOTOR_MODE_GOCCW:
      if (GLOBAL_MotorExtMode == true)
      {
        MOTOR_MaxPwmValue = MOTOR_ExtPower;
      } else
      {
        MOTOR_MaxPwmValue = EE_PWM_Max;
      }
      #ifdef DEF_DUPLEX
      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
      #endif
      break;
    default:
      /**< Must not occur */
      break;
  }
}

/** \brief Set motor mode for slave
 *
 * \param [in] mode Motor mode to set
 * \return void
 *
 */
#ifdef DEF_DUPLEX
void MOTOR_SetSlaveMode(uint8_t mode)
{
  if (MOTOR_SlaveFree == true)
  {
    /**< Ignore changing mode */
    return;
  }
  switch (mode)
  {
    case SLAVE_MODE_BRAKE:
      MOTOR_SlaveMode = SLAVE_MODE_BRAKE;
      MOTOR_Mode = MOTOR_MODE_BRAKE;
      MOTOR_MaxPwmValue = 0;
      break;
    case SLAVE_MODE_FREE:
      MOTOR_SlaveMode = SLAVE_MODE_FREE;
      MOTOR_Mode = MOTOR_MODE_FREE;
      MOTOR_MaxPwmValue = 0;
      break;
    case SLAVE_MODE_RUN1:
      MOTOR_SlaveMode = SLAVE_MODE_RUN1;
      MOTOR_Mode = MOTOR_MODE_RUN1;
      MOTOR_MaxPwmValue = GLOBAL_PartnerMaxPwmValue;
      break;
    case SLAVE_MODE_RUN2:
      MOTOR_SlaveMode = SLAVE_MODE_RUN2;
      MOTOR_Mode = MOTOR_MODE_RUN2;
      MOTOR_MaxPwmValue = GLOBAL_PartnerMaxPwmValue;
      break;
    default:
      /**< Must not occur */
      break;
  }
}
#endif

#ifdef DEF_DUPLEX
void MOTOR_DoSlaveFree(bool yes)
{
  MOTOR_SlaveFree = yes;
  if (yes == true)
  {
    /**< Slave is free, not necessary to change it otherwise */
    MOTOR_SlaveMode = SLAVE_MODE_FREE;
  }
}
#endif

/** \brief Get motor rotation direction
 *
 * \return Current motor rotation direction as uint8_t
 *
 */
uint8_t MOTOR_GetDir(void)
{
  return MOTOR_Dir;
}

/** \brief Set external power value
 *
 * \param [in] power Value to set
 * \return void
 *
 */
void MOTOR_SetExtPower(uint8_t power)
{
  MOTOR_ExtPower = power;
}

/** \brief Get external power value
 *
 * \return Value as uint8_t
 *
 */
uint8_t MOTOR_GetExtPower(void)
{
  return MOTOR_ExtPower;
}

/** \brief Get motor mode
 *
 * \return Current motor mode as uint8_t
 *
 */
uint8_t MOTOR_GetMode(void)
{
  #ifdef DEF_DUPLEX
  if (GLOBAL_IsMaster == false)
  {
    return MOTOR_SlaveMode;
  }
  #endif
  return MOTOR_Mode;
}

#ifdef DEF_DUPLEX
/** \brief Get motor mode for slave
 *
 * \return Current slave mode as uint8_t
 *
 */
uint8_t MOTOR_GetSlaveMode(void)
{
  return MOTOR_SlaveMode;
}
#endif

/** \brief Get motor mode
 *
 * \return Current motor mode as uint8_t
 *
 */
uint8_t MOTOR_GetMaxPwmValue(void)
{
  return MOTOR_MaxPwmValue;
}

/** \brief Do motor brake
 *
 * \param [in] on Do brake if true
 * \return void
 *
 */
#ifndef DEF_UNITTEST
static void MOTOR_DoBrake(bool on)
{
  DRV8320_DoBrake(on);
}
#endif

/** \brief Switch motor to free running mode
 *
 * \param [in] on Switch to free running mode if true
 * \return void
 *
 */
#ifndef DEF_UNITTEST
static void MOTOR_DoFree(bool on)
{
  DRV8320_DoFree(on);
}
#endif

/** \brief Set rotation direction
 *
 * \param [in] dir Rotation direction
 * \return void
 *
 */
#ifndef DEF_UNITTEST
static void MOTOR_SetDir(uint8_t dir)
{
  DRV8320_SetDir(dir);
}
#endif

/** \brief Do BLDC motor test
 *
 * \param [out] resp_1 Response byte 1
 * \param [out] resp_2 Response byte 2
 * \return  Nothing
 *
 */
#ifndef DEF_UNITTEST
void MOTOR_DoTest(uint8_t *resp_1, uint8_t *resp_2)
{
  /**< Old testing function, works with actual DA58-D, doesn't test windings */

  #ifdef DEF_DUPLEX
  /**< Check Halls, duplex only! */
  MOTOR_ExtPower = 0x80U;
  MOTOR_Mode = MOTOR_MODE_GOCW;
  (void)HALL_DoCheck();
  MOTOR_Mode = MOTOR_MODE_FREE;
  vTaskDelay(pdMS_TO_TICKS(10));
  *resp_1 = HALL_DoAnalyze();
  #else
  *resp_1 = 0;
  #endif
  /**< Check MOSFETs */
  *resp_2 = MOTOR_GetError();

  /**< New testing function, hardware must be changed! */
  //*resp_1 = GLOBAL_Windings;
  //*resp_2 = MOTOR_GetError();
}
#endif

/** \brief Perform power damping
 *
 * \param [in] power_in Input power value
 * \return Output power value as int16_t
 *
 */
int16_t MOTOR_DoDamping(int16_t power_in)
{
  #define KALMAN_MULT         100
  #define KALMAN_MAX_COEFF    255
  static int32_t power = 0;
  int16_t power_out;

  if (power_in == 0)
  {
    power = 0;
    return 0;
  }
  /**< Kalman filtering, checked with Excel implementation */
  power = (power * (int32_t)EE_PowerDamper + (int32_t)power_in * (KALMAN_MAX_COEFF - (int32_t)EE_PowerDamper) * KALMAN_MULT) / KALMAN_MAX_COEFF;
  if (power_in < 0)
  {
    power_out = (int16_t)((power - (KALMAN_MULT / (int32_t)2)) / (int32_t)KALMAN_MULT);
  } else
  {
    power_out = (int16_t)((power + (KALMAN_MULT / (int32_t)2)) / (int32_t)KALMAN_MULT);
  }

  return power_out;
}

/** \brief Reset PID regulator, not used in a real firmware
 *
 * \return void
 *
 */
void MOTOR_ResetPID(void)
{
  errPosLast = 0;
  i_sum = 0;
}

/** \brief Calculate new power value for BLDC motor
 *
 * \return New power value as int16_t
 *
 */
int16_t MOTOR_CalcPower(void)
{
  int32_t powerOut;
  int16_t errDiff;

  /**< Calculate error (shaft angle error) */
  MOTOR_ErrPos = GLOBAL_TargetPos - GLOBAL_RealPos;
  MOTOR_ErrPos = NOTCH_Calculate(NOTCH_NUM_1, MOTOR_ErrPos);
  MOTOR_ErrPos = NOTCH_Calculate(NOTCH_NUM_2, MOTOR_ErrPos);
  /**< Apply deadband for position */
  if (abs(MOTOR_ErrPos) <= (int16_t)EE_Sensor_DB)
  {
    /**< If position error is less than deadband => GLOBAL_ErrPos = 0 */
    MOTOR_ErrPos = 0;
  } else
  {
    /**< If position error is more than deadband => GLOBAL_ErrPos = GLOBAL_ErrPos + deadband */
    if (MOTOR_ErrPos > 0)
    {
      MOTOR_ErrPos -= (int16_t)EE_Sensor_DB;
    } else
    {
      MOTOR_ErrPos += (int16_t)EE_Sensor_DB;
    }
  }
  /**< D-part of the regulator */
  errDiff = MOTOR_ErrPos - errPosLast;
  /**< Save position error for differential part */
  errPosLast = MOTOR_ErrPos;

  /**< Add position error to I-part */
  i_sum = i_sum + MOTOR_ErrPos;
  if (i_sum > MOTOR_I_SUM_MAX)
  {
    i_sum = MOTOR_I_SUM_MAX;
  }
  if (i_sum < MOTOR_I_SUM_MIN)
  {
    i_sum = MOTOR_I_SUM_MIN;
  }

  if (abs(MOTOR_ErrPos) > MOTOR_I_MAX_SENS_ANGLE)
  {
    /**< Reset I-part if position error is too big */
    i_sum = 0;
  }
  /**< Calculate regulation factor */
  powerOut = (((int32_t)MOTOR_ErrPos * (int32_t)EE_Kp) / 16) + (((int32_t)i_sum * (int32_t)EE_Ki) / 32) +
              (((int32_t)errDiff * (int32_t)EE_Kd) / 4);
  /**< Check power limits, 2 modes */
  if (GLOBAL_MotorExtMode == true)
  {
    return UTILS_LimitValue(powerOut, MOTOR_ExtPower);
  } else
  {
    return UTILS_LimitValue(powerOut, EE_PWM_Max);
  }
}

/** \brief Process motor saver option
 *
 * \param [in] power_in Input power value
 * \return Output power value
 *
 */
int16_t MOTOR_Saver(int16_t power_in)
{
  int16_t power = power_in;

  if ((abs(power) > (int16_t)EE_PWM_Safe) && (EE_SaverTime > 0U))
  {
    if (MOTOR_JammedCnt > (EE_SaverTime * 1000U / pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS)))
    {
      /**< Servo is in a jam for not less than EE_SaverTime seconds */
      if (MOTOR_PosJammed == 0)
      {
        /**< Remember jam position for the first occurrence */
        MOTOR_PosJammed = GLOBAL_RealPos;
        MOTOR_TargetPosJammed = GLOBAL_TargetPos;
      }
      /**< If position where we jammed is out of frame now (due to target position change) then get out of saver mode */
      if ((MOTOR_TargetPosJammed != 0) && (abs(MOTOR_TargetPosJammed - GLOBAL_TargetPos) > (int)EE_SaverFrame))
      {
        MOTOR_ResetJam();
      }
      /**< Check if within the frame */
      if (abs(GLOBAL_RealPos - MOTOR_PosJammed) < (int)EE_SaverFrame)
      {
        if (power > 0)
        {
          power = EE_PWM_Safe;
        } else
        {
          power = -EE_PWM_Safe;
        }
      } else
      {
        MOTOR_ResetJam();
      }
    } else
    {
      MOTOR_JammedCnt++;
    }
  } else
  {
    MOTOR_ResetJam();
  }

  return power;
}

/** \brief Heating function based on anti-backlash
 *
 * \param [in] power_in Input PWM value for anti-backlash
 * \return New PWM value as uint8_t
 *
 */
uint8_t MOTOR_Heater(uint8_t power_in)
{
  int32_t out;
  uint8_t temp = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
  uint8_t result = power_in;

  if ((temp < EE_HeaterTemp) && (EE_HeaterTemp > 0U))
  {
    /**< Heater activated */
    out = ((int32_t)EE_HeaterTemp - (int32_t)temp) * HEATER_GAIN_COEFF;
    if (out > HEATER_MAX_PWM)
    {
      out = HEATER_MAX_PWM;
    }
    if (out > (int32_t)EE_PWM_Min)
    {
      result = (uint8_t)out;
    }
  }

  return result;
}

/** \brief Current limiter
 *
 * \param [in] reversal
 * \param [in] power_in Input power value
 * \param [in] degraded True if degraded mode is active (only one side is running)
 * \return Output power value after limiting
 *
 */
int16_t MOTOR_CurrentLimiter(uint8_t reversal, int16_t power_in, bool degraded)
{
  static uint8_t counter = 0;
  static int16_t posErr50Hz = 0;
  static uint16_t pwm_limit = MAX_PWM;
  static int16_t shaftSpeedOld = 0;

  /**< Make sure that the shaft speed calculation will be re-started upon direction reversal */
  if (reversal == REVERSAL)
  {
    /**< Force current limiter algorithm */
    counter = 0;
  }

  if (counter == 0U)
  {
    int16_t shaftSpeed;

    if (reversal == REVERSAL)
    {
      /**< Make sure to use the last calculated shaft speed upon a direction reversal */
      shaftSpeed = shaftSpeedOld;
    } else
    {
      /**< Calculate signed shaft speed in Sensor counts per 20 ms */
      shaftSpeed = GLOBAL_RealPos - posErr50Hz;
      shaftSpeedOld = shaftSpeed;
    }
    counter = 20;
    posErr50Hz = GLOBAL_RealPos;
    /**< Just to report for debugging */
    MOTOR_ShaftSpeed = (int8_t)shaftSpeed;

    if (EE_MaxCurrent > 0U)
    {
      /** current set-point is defined for the entire actuator. Hence, 50% per motor.
       *  example: eeprom = 10 ==> 1.0 A ==> 0.5 A per Motor ==> 10 / 20 = 0.5 A
       */
      int32_t max_current = (int32_t)EE_MaxCurrent * 5;

      /** asymmetrical power limit
       */
      if ((EE_MaxCurrentCW > 0U) && (EE_MaxCurrentCW < 255U))
      {
          if (power_in < 0)
          {
            max_current = (int32_t)EE_MaxCurrentCW * 5;
          }
      }

      int16_t max_PWM = 0;

      if (degraded == true)
      {
        /**< Multiply by 2.2 to achieve approximately the same degraded output torque with one motor as with two motors */
        max_current = max_current * 22 / 10;
      }
      if (power_in < 0)
      {
        shaftSpeed = -shaftSpeed;
      }
      max_PWM = (int16_t)((((max_current * MOTOR_IMPEDANCE) + (VOLTAGE_CONSTANT * (int32_t)shaftSpeed)) /
                           (int32_t)SUPPLY_VOLTAGE) * MAX_PWM / 10000);

      if (max_PWM < 0)
      {
        pwm_limit = 0;
      } else
      {
        pwm_limit = max_PWM;
      }
    } else
    {
      pwm_limit = MAX_PWM;
    }

    /**< Just to report for debugging */
    MOTOR_LimitPWM = (uint8_t)pwm_limit;
  }
  counter--;

  return UTILS_LimitValue(power_in, pwm_limit);
}

/** \brief Get limit value for PWM (current limiter)
 *
 * \return Limit value as uint8_t
 *
 */
uint8_t MOTOR_GetLimitPWM(void)
{
  return MOTOR_LimitPWM;
}

/** \brief Get shaft speed (current limiter)
 *
 * \return Shaft speed as int8_t
 *
 */
int8_t MOTOR_GetShaftSpeed(void)
{
  return MOTOR_ShaftSpeed;
}

/** \brief Get position error value
 *
 * \return Position error as int16_t
 *
 */
int16_t MOTOR_GetErrPos(void)
{
  return MOTOR_ErrPos;
}

/** \brief Process magnet sensor (readout and error handling)
 *
 * \return void
 *
 */
void MOTOR_ProcessMagnet(void)
{
  static uint8_t errCounter = 0;

  (void)MAGNET_Read(&GLOBAL_MagnetValue);
  #ifdef DEF_DUPLEX
  if (GLOBAL_InternalID == ID_ACE2)
  {
    /**< Apply magnet sensor offset */
    int16_t sval16 = (int16_t)GLOBAL_MagnetValue + EE_MagSensorOffs;
    GLOBAL_MagnetValue = (uint16_t)sval16;
    GLOBAL_MagnetValue &= 0xFFFU;
  }
  #endif
  GLOBAL_RawPos = (int16_t)GLOBAL_MagnetValue - 0x800;
  int16_t sval16 = (int16_t)GLOBAL_MagnetValue + EE_ZeroPos;
  uint16_t uval16 = (uint16_t)sval16 & 0xFFFU;
  GLOBAL_RealPos = (int16_t)uval16 - 0x800;

  if (MAGNET_HasError() == true)
  {
    errCounter++;
  } else
  {
    errCounter = 0;
  }

  if (errCounter > MAGNET_MAX_ERRORS)
  {
    /**< Magnet sensor error */
    errCounter = MAGNET_MAX_ERRORS;
    /**< 0xFFFF will be return as actual - faulty signal response */
    GLOBAL_RealPos = (int16_t)0xFFFFU;
    /**< Magnet error is permanent, set error bit */
    GLOBAL_ErrMagnet = true;
  } else
  {
    /**< No magnet sensor error anymore */
    GLOBAL_ErrMagnet = false;
  }
}

/** \brief RTOS task for motor regulation, has highest priority, runs once a millisecond
 */
#ifndef DEF_UNITTEST
void MOTOR_Task(void *pParameters)
{
  (void) pParameters;
  static uint8_t currMode = MOTOR_MODE_RUN1;
  static int16_t powerValOld = 0;
  int16_t powerVal;
  bool degraded;
  int16_t master_PWM;
  int16_t slave_PWM;
  #ifdef DEF_DUPLEX
  static uint8_t currDir = MOTOR_DIR_CW;
  #endif
  #if defined DEF_DA58 && defined DEF_DUPLEX
  //static uint8_t overVoltageCounter = 0;
  #endif

  MOTOR_Init();

  MOTOR_ResetPID();

  GLOBAL_TargetPos = 0;
  GLOBAL_Power = 0;
  #ifdef DEF_DUPLEX
  GLOBAL_PartnerPower = 0;
  #endif

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_DELAY_MS));

    #ifndef DEF_DA58_SD
    /**< Toogle Enable pin to watchdog the BLDC driver */
    DRV8320_EnableToggle();
    #endif

    #ifdef DEF_DUPLEX
    /**< Get current duplex status to check degraded mode */
    degraded = ((GLOBAL_ExtStatus & EXTSTAT_DEGRADED_BIT) > 0U);
    #else
    degraded = false;
    #endif

    /**< Read magnet sensor */
    MOTOR_ProcessMagnet();
    /**< Read BLDC driver status */
    MOTOR_Error = DRV8320_ReadError();

    if (GLOBAL_ErrMagnet == true)
    {
      powerVal = 0;
    } else
    { /**< No magnet sensor error */
      if (GLOBAL_IsMaster == true)
      { /**< Motor power must be calculated here */
        powerVal = MOTOR_CalcPower();

        if (((int32_t)powerValOld * (int32_t)powerVal) < 0)
        {
          /**< Motor direction changed, disable power to avoid current peaks */
          (void)MOTOR_DoDamping(0);
          (void)MOTOR_CurrentLimiter(REVERSAL, powerVal, degraded);
          powerVal = 0;
        } else
        {
          /**< Current limiter */
          powerVal = MOTOR_CurrentLimiter(NO_REVERSAL, powerVal, degraded);
          /**< Kalman filtering */
          powerVal = MOTOR_DoDamping(powerVal);
        }
        powerValOld = powerVal;
        #if defined DEF_DA58
//          if (GPIO_GetPin(MOTOR_ACT_PORT, MOTOR_ACT_PIN) == true)
//          {
//            // Limit motor power for detected overvoltage (generator mode)
//            if (powerVal > EE_PWM_Safe)
//              powerVal = EE_PWM_Safe;
//          }
        #endif
      }
    }

    if (GLOBAL_IsMaster == true)
    { /**< I'm master */
      switch (MOTOR_Mode)
      {
        case MOTOR_MODE_FREE:
          /**< Free running mode, windings open */
          /**< Will be transmitted to partner */
          if (currMode == MOTOR_MODE_BRAKE)
          {
            MOTOR_DoBrake(false);
          }
          if (currMode != MOTOR_Mode)
          {
            MOTOR_DoFree(true);
            currMode = MOTOR_Mode;
          }
          master_PWM = 0;
          slave_PWM = 0;
          break;
        case MOTOR_MODE_BRAKE:
          /**< Brake mode */
          /**< Will be transmitted to partner */
          (void)MOTOR_DoDamping(0);
          master_PWM = 0;
          slave_PWM = 0;
          if (currMode == MOTOR_MODE_FREE)
          {
            MOTOR_DoFree(false);
          }
          if (currMode != MOTOR_Mode)
          {
            MOTOR_DoBrake(true);
            currMode = MOTOR_Mode;
          }
          break;
        case MOTOR_MODE_GOCW:
          /**< Forward run (CW) */
          if (currMode == MOTOR_MODE_BRAKE)
          {
            MOTOR_DoBrake(false);
          }
          if (currMode != MOTOR_Mode)
          {
            master_PWM = 0;
            slave_PWM = 0;
            (void)MOTOR_DoDamping(0);
            MOTOR_DoFree(false);
            currMode = MOTOR_Mode;
            break;
          }
          master_PWM = MOTOR_ExtPower;
          slave_PWM = MOTOR_ExtPower;
          MOTOR_Dir = MOTOR_DIR_CW;
          #ifdef DEF_DUPLEX
          GLOBAL_SlaveMotorDir = MOTOR_DIR_CW;
          #endif
          break;
        case MOTOR_MODE_GOCCW:
          /**< Backward run (CCW) */
          if (currMode == MOTOR_MODE_BRAKE)
          {
            MOTOR_DoBrake(false);
          }
          if (currMode != MOTOR_Mode)
          {
            master_PWM = 0;
            slave_PWM = 0;
            (void)MOTOR_DoDamping(0);
            MOTOR_DoFree(false);
            currMode = MOTOR_Mode;
            break;
          }
          master_PWM = MOTOR_ExtPower;
          slave_PWM = MOTOR_ExtPower;
          MOTOR_Dir = MOTOR_DIR_CCW;
          #ifdef DEF_DUPLEX
          GLOBAL_SlaveMotorDir = MOTOR_DIR_CCW;
          #endif
          break;
        default:
          /**< Normal mode */
          if (currMode == MOTOR_MODE_BRAKE)
          {
            MOTOR_DoBrake(false);
          }
          if (currMode != MOTOR_Mode)
          {
            (void)MOTOR_DoDamping(0);
            MOTOR_DoFree(false);
            currMode = MOTOR_Mode;
          }
          /**< Check hall sensors and activate motor if they are Ok */
          if (HALL_ReadValues() != HALL_CHECK_BIT_ALL)
          {
            MOTOR_DoFree(false);
          }
          /**< Motor saver section */
          powerVal = MOTOR_Saver(powerVal);

          #ifdef DEF_DUPLEX
          /**< Anti-backlash option enabled? */
          if ((EE_Options & OPTIONS_ANTI_BACKLASH) > 0U)
          {
            if (degraded == true)
            {
              master_PWM = powerVal;
              slave_PWM = 0;
            } else
            {
              uint16_t anti_backlash_pwm;// = EE_PWM_Min;

              anti_backlash_pwm = MOTOR_Heater(EE_PWM_Min);
              /**< Strategy one: gradually reduce anti-backlash PWM to 0 at 100% PD output, taking EE_PWM_Max into account */
              if ((abs(powerVal) < (int)EE_PWM_Max) && (EE_PWM_Max > 0U))
              {
                /**< Reduce anti backlash PWM gradually */
                int tmp = (int)EE_PWM_Max - abs(powerVal);
                uint16_t uval16 = (uint16_t)tmp;
                anti_backlash_pwm =  anti_backlash_pwm * uval16 / EE_PWM_Max;
              } else
              {
                anti_backlash_pwm = 0U;
              }
              master_PWM = powerVal + (int16_t)anti_backlash_pwm;
              slave_PWM = powerVal - (int16_t)anti_backlash_pwm;
            }
          } else
          {
            /**< No anti-backlash */
            if (degraded == true)
            {
              master_PWM = powerVal;
              slave_PWM = 0;
            } else
            {
              master_PWM = powerVal;
              slave_PWM = powerVal;
            }
          }
          #else
          /**< No anti-backlash for simplex */
          if (abs(powerVal) < (int16_t)EE_PWM_Min)
          {
            (void)MOTOR_DoDamping(0);
            powerVal = 0;
          }
          master_PWM = powerVal;
          #endif

          if (master_PWM < 0)
          {
            MOTOR_Dir = MOTOR_DIR_CCW;
          } else
          {
            MOTOR_Dir = MOTOR_DIR_CW;
          }
          master_PWM = UTILS_LimitValue(master_PWM, 255);

          #ifdef DEF_DUPLEX
          if (slave_PWM < 0)
          {
            GLOBAL_SlaveMotorDir = MOTOR_DIR_CCW;
          } else
          {
            GLOBAL_SlaveMotorDir = MOTOR_DIR_CW;
          }
          slave_PWM = UTILS_LimitValue(slave_PWM, 255);
          #endif

          if (HALL_ReadValues() == HALL_CHECK_BIT_ALL)
          {
            /**< Deactivate motor if Hall sensors are not Ok */
            MOTOR_DoFree(true);
          }
          break;
      }
      GLOBAL_Power = abs(master_PWM);
      #ifdef DEF_DUPLEX
      GLOBAL_SlavePower = abs(slave_PWM);
      #else
      (void)slave_PWM;
      #endif
      MOTOR_SetDir(MOTOR_Dir);
      PWM_Set(GLOBAL_Power);
    } else
    { /**< I'm slave */
      #ifdef DEF_DUPLEX
      powerVal = 0;
      switch (MOTOR_SlaveMode)
      {
        case SLAVE_MODE_BRAKE:
          /**< BLDC brakes */
          if (currMode == MOTOR_MODE_FREE)
          {
            MOTOR_DoFree(false);
          }
          if (currMode != MOTOR_SlaveMode)
          {
            MOTOR_DoBrake(true);
            currMode = MOTOR_SlaveMode;
          }
          break;
        case SLAVE_MODE_FREE:
          /**< Windings open, BLDC rotates free */
          MOTOR_DoBrake(false);
          if (currMode != MOTOR_SlaveMode)
          {
            MOTOR_DoFree(true);
            currMode = MOTOR_SlaveMode;
          }
          break;
        default:
          MOTOR_DoBrake(false);
          #if defined DEF_DA58
//            if ((GPIO_GetPin(MOTOR_ACT_PORT, MOTOR_ACT_PIN) == true))
//            {
//              // High speed, do free running for some ms
//              MOTOR_DoFree(true);
//              overVoltageCounter = MOTOR_OVERVOLTAGE_LIMIT;
//              break;
//            } else
//            {
//              if (overVoltageCounter > 0)
//                overVoltageCounter--;
//              if (overVoltageCounter > 0)
//                break;
//              MOTOR_DoFree(false);
//            }
          #endif
          if (currMode != MOTOR_SlaveMode)
          {
            MOTOR_DoFree(false);
            currMode = MOTOR_SlaveMode;
            break;
          }
          /**< Normal operation, just move with power from master */
          if (currDir != GLOBAL_PartnerMotorDir)
          {
            currDir = GLOBAL_PartnerMotorDir;
            powerVal = 0;
            break;
          }
          /**< Set power value received from master */
          powerVal = GLOBAL_PartnerPower;
          /**< Get direction from my partner */
          MOTOR_Dir = GLOBAL_PartnerMotorDir;
          MOTOR_SetDir(MOTOR_Dir);
          break;
      }
      PWM_Set(powerVal);
      #endif
    }
  }
}
#endif
