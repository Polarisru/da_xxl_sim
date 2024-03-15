#include "drivers.h"
#include "counters.h"
#include "crc32.h"
#include "drv8320.h"
#include "eeprom.h"
#include "global.h"
#include "logic.h"
#include "monitor.h"
#include "motor.h"
#include "power.h"
#include "redundancy.h"
#include "timeouts.h"
#ifdef DEF_DUPLEX
  #include "heartbeat.h"
  #include "inetwork.h"
  #include "voting.h"
#endif

#include "chain.h"
#include "rs485.h"

static uint8_t force_cnt;
static uint8_t power_cnt;
static uint8_t pos_cnt;
static uint8_t voting_cnt;
static bool motor_is_free;

/** \brief Reset status bits
 *
 * \return void
 *
 */
void LOGIC_ResetStatusBits(void)
{
  /**< Reset error bits */
  GLOBAL_ErrMotor = false;
  GLOBAL_ErrMagnet = false;
  GLOBAL_ErrFreshness = false;
  /**< Error events and signal to partner be master deactivated */
  GLOBAL_MyStatus = 0;
  GLOBAL_ExternalStatus = 0;
  GLOBAL_ExtStatus = 0;
  #ifdef DEF_DUPLEX
    GLOBAL_PartnerStatus = 0;
    GLOBAL_PartnerExtStatus = 0;
  #endif
}

/** \brief Reset internal PWM values
 *
 * \param void
 * \return void
 *
 */
#ifdef DEF_DUPLEX
void LOGIC_ResetPWM(void)
{
  GLOBAL_PartnerPower = 0;
  GLOBAL_SlavePower = 0;
  GLOBAL_Power = 0;
}
#endif

/** \brief Set default values
 *
 * \return void
 *
 */
void LOGIC_SetDefaultValues(uint8_t motor_mode, uint8_t reset_mode)
{
  /**< Reset error bits */
  GLOBAL_ErrMotor = false;
  GLOBAL_ErrMagnet = false;
  GLOBAL_ErrFreshness = false;
  /**< Reset all important timeout counters */
  TIMEOUTS_Reset(TIMEOUT_TYPE_ALL);
  /**< Reset status bits */
  LOGIC_ResetStatusBits();
  /**< Motor mode is RUN */
  MOTOR_SetMode(motor_mode);
  motor_is_free = false;
  GLOBAL_MotorExtMode = false;
  MOTOR_SetExtPower(0);
  #ifdef DEF_DUPLEX
    LOGIC_ResetPWM();
    GLOBAL_ForceSlaveMode = motor_mode;
    if (reset_mode == RESET_MODE_ALL)
    {
      GLOBAL_DoDefault = INET_BE_DFLT;
      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
    }
    MOTOR_SetSlaveMode(motor_mode);
    if (GLOBAL_InternalID == ID_ACE1)
    {
      GLOBAL_IsMaster = true;
      HEARTBEAT_Enable();
    } else
    {
      GLOBAL_IsMaster = false;
      HEARTBEAT_Disable();
    }
    MOTOR_DoSlaveFree(false);
  #else
    (void) reset_mode;
  #endif
  /**< Reset internal counters */
  force_cnt = 0;
  power_cnt = 0;
  pos_cnt = 0;
  voting_cnt = 0;
}

/** \brief Reset timeout
 *
 * \param [in] type Timeout counter to reset
 * \return void
 *
 */
//void LOGIC_ResetTimeout(uint8_t type)
//{
//  switch (type)
//  {
//    case TIMEOUT_TYPE_PARTNER:
//      GLOBAL_TimeoutPartnerACE = 0U;
//      break;
//    case TIMEOUT_TYPE_HALL:
//      GLOBAL_TimeoutHall = 0U;
//      break;
//    case TIMEOUT_TYPE_HOST:
//      GLOBAL_TimeoutConnection = 0U;
//      break;
//    case TIMEOUT_TYPE_ALL:
//      GLOBAL_TimeoutPartnerACE = 0U;
//      GLOBAL_TimeoutHall = 0U;
//      GLOBAL_TimeoutConnection = 0U;
//      break;
//    default:
//      break;
//  }
//}

/** \brief RTOS task for common motor logic
 */
#ifndef DEF_UNITTEST
void LOGIC_Task(void *pParameters)
{
  (void) pParameters;
  uint32_t ticks = 0;
  uint32_t ticks_1s = 0;
  uint8_t bVal;
  uint32_t crc;
  uint32_t len;
  uint32_t crcCounter = 0;

  #ifdef DEF_DUPLEX
  HEARTBEAT_Configuration(GLOBAL_InternalID == ID_ACE1);
  #endif

  LOGIC_SetDefaultValues(MOTOR_MODE_FREE, RESET_MODE_SINGLE);

  len = (uint32_t)((void*)&crc32_build) - APP_ADDRESS;
  //crc = CRC32_Calc((uint8_t*)APP_ADDRESS, len);
  //if (crc)
  DSU_StartCRC(APP_ADDRESS, len);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(LOGIC_TASK_DELAY_MS));

    /**< Reset watchdog */
    WDT_Reset();
    /**< Process load counters */
    //LOGIC_ProcessLoadCounters();
    COUNTERS_Process();

    /**< Bit 0: position error, ToDo */
    if (EE_PosErrTime > 0U)
    {
      /**< Active only for non-zero values */
      if ((abs(GLOBAL_TargetPos - GLOBAL_RealPos) > (int16_t)EE_PosErrAngle) && (GLOBAL_IsMaster == true) &&
          (MOTOR_GetMode() != MOTOR_MODE_FREE))
      {
        if (pos_cnt > (EE_PosErrTime * (POS_ERROR_TIME_RES_MS / LOGIC_TASK_DELAY_MS)))
        {
          GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_POSITION;
          GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_POSITION;
        } else
        {
          pos_cnt++;
        }
      } else
      {
        pos_cnt = 0;
        GLOBAL_MyStatus &= ~STAT_ERROR_POSITION;
      }
    }

    #ifdef DEF_DUPLEX
    /**< Bit 1: 2oo3 voting error */
    switch (VOTING_Process())
    {
      case VOTING_ERROR:
        if (++voting_cnt > VOTING_CNT_MAX)
        {
          voting_cnt = VOTING_CNT_MAX;
          GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_2OO3;
          GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_2OO3;
          GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ACE2ACE_POS_ERROR | EXTSTAT_ACE2VOTING_POS_ERROR;
        }
        break;
      case VOTING_OK:
        GLOBAL_MyStatus &= ~STAT_ERROR_2OO3;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus & ~EXTSTAT_ACE2VOTING_POS_ERROR & ~EXTSTAT_ACE2ACE_POS_ERROR;
        voting_cnt = 0;
        break;
      case VOTING_OK_ACE2ACE:
        GLOBAL_MyStatus &= ~STAT_ERROR_2OO3;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ACE2ACE_POS_ERROR;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus & ~EXTSTAT_ACE2VOTING_POS_ERROR;
        voting_cnt = 0;
        break;
      case VOTING_OK_ACE2VOT:
        GLOBAL_MyStatus &= ~STAT_ERROR_2OO3;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ACE2VOTING_POS_ERROR;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus & ~EXTSTAT_ACE2ACE_POS_ERROR;
        voting_cnt = 0;
        break;
      case VOTING_OK_ACEVBOTH:
        GLOBAL_MyStatus &= ~STAT_ERROR_2OO3;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ACE2ACE_POS_ERROR | EXTSTAT_ACE2VOTING_POS_ERROR;
        voting_cnt = 0;
        break;
      default:
        /**< Do nothing */
        break;
    }
    #endif

    /**< Bit 2: Power supply voltage is too low */
    if (MONITOR_GetVoltageStatus() == false)
    {
      if (power_cnt > POWER_FAIL_MAX)
      {
        GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_POWER;
        GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_POWER;
      } else
      {
        power_cnt++;
      }
    } else
    {
      power_cnt = 0;
      GLOBAL_MyStatus &= ~STAT_ERROR_POWER;
      //GLOBAL_ExternalStatus &= ~STAT_ERROR_POWER;
    }

    /**< Bit 3: Communication freshness-counter failure */
    if (GLOBAL_ErrFreshness == true)
    {
      GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_FRESHNESS;
      GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_FRESHNESS;
    } else
    {
      GLOBAL_MyStatus &= ~STAT_ERROR_FRESHNESS;
      //GLOBAL_ExternalStatus &= ~STAT_ERROR_FRESHNESS;
    }

    /**< Bit 4: Motor temperature error */
    if (MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR) == true)
    {
      bVal = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
      if ((bVal >= (EE_MaxMotorTemp + 50U)) || (GLOBAL_MyStatus & STAT_ERROR_MOTOR_TEMP))
      {
        /**< Motor temperature IS not Ok or WAS not Ok */
        if (bVal < (EE_MaxMotorTemp + 50U - EE_MotorTempHyst))
        {
          /**< Temperature is Ok, reset motor temperature error bit */
          GLOBAL_MyStatus &= ~STAT_ERROR_MOTOR_TEMP;
        } else
        {
          /**< Set motor temperature error bit */
          GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_MOTOR_TEMP;
          GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_MOTOR_TEMP;
        }
      }
    }

    /**< Bit 5: EEPROM CRC error */
    if (crcCounter > 250U)
    {
      if (EEPROM_CheckCRC() == false)
      {
        GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_EEPROM;
        GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_EEPROM;
      } else
      {
        GLOBAL_MyStatus &= ~STAT_ERROR_EEPROM;
        //GLOBAL_ExternalStatus &= ~STAT_ERROR_EEPROM;
      }
    }

    /**< Bit 6: RAM/Flash integrity failure */
    crcCounter++;
    if ((DSU_IsDone() == true) && (crcCounter > 5000U))
    {
      /**< Flash CRC calculation, DSU unit is used instead of DMA now */
      crcCounter = 0;
      crc = DSU_GetCRC();
      DSU_StartCRC(APP_ADDRESS, len);
      if (crc != crc32_build)
      {
        GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_INTEGRITY;
        GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_INTEGRITY;
        GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_FLASH;
      }
    }
    if (GLOBAL_IsMemoryOk == false)
    {
      GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_INTEGRITY;
      GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_INTEGRITY;
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_RAM;
    }

    /**< Bit 7: BLDC Driver/Motor failure */
    if (GLOBAL_ErrMotor == true)
    {
      GLOBAL_ExternalStatus = GLOBAL_ExternalStatus | STAT_ERROR_BLDC;
      GLOBAL_MyStatus = GLOBAL_MyStatus | STAT_ERROR_BLDC;
    } else
    {
      GLOBAL_MyStatus &= ~STAT_ERROR_BLDC;
    }

    /**< Bit 8: Position sensor error */
    if (GLOBAL_ErrMagnet == true)
    {
      GLOBAL_MyStatus |= STAT_ERROR_MAGNET;
      GLOBAL_ExternalStatus |= STAT_ERROR_MAGNET;
    } else
    {
      GLOBAL_MyStatus &= ~STAT_ERROR_MAGNET;
      //GLOBAL_ExternalStatus &= ~STAT_ERROR_MAGNET;
    }

    #if defined (DEF_DUPLEX)
    /**< Bit 9: Other ACE is Master Bit */
    if (GLOBAL_PartnerExtStatus != STAT_NO_PARTNER)
    {
      if ((GLOBAL_PartnerExtStatus & STAT_MASTER_BIT) > 0U)
      {
        GLOBAL_ExternalStatus |= STAT_OTHER_ACE_MSTR_BIT;
        GLOBAL_MyStatus |= STAT_OTHER_ACE_MSTR_BIT;
      } else
      {
        GLOBAL_ExternalStatus &= ~STAT_OTHER_ACE_MSTR_BIT;
        GLOBAL_MyStatus &= ~STAT_OTHER_ACE_MSTR_BIT;
      }
    }	else
	  {
	    /**< Other ACE is Master bit must be cleared if no partner ACE is detected */
		  GLOBAL_ExternalStatus &= ~STAT_OTHER_ACE_MSTR_BIT;
		  GLOBAL_MyStatus &= ~STAT_OTHER_ACE_MSTR_BIT;
	  }
    #endif

    /**< Bit 10: motor is free */
    if ((MOTOR_GetMode() == MOTOR_MODE_FREE) || (MOTOR_GetMode() == MOTOR_MODE_BRAKE))
    {
      GLOBAL_MyStatus |= STAT_MOTOR_FREE_BIT;
      GLOBAL_ExternalStatus |= STAT_MOTOR_FREE_BIT;
    } else
    {
      GLOBAL_MyStatus &= ~STAT_MOTOR_FREE_BIT;
      GLOBAL_ExternalStatus &= ~STAT_MOTOR_FREE_BIT;
    }

    /**< Bit 11: Loss of communication with host */
    if (EE_LossTime > 0U)
    {
      //if (timeout_Connection > (uint16_t)EE_LossTime * 100U / LOGIC_TASK_DELAY_MS)
      if (TIMEOUTS_IsActive(TIMEOUT_TYPE_HOST) == true)
      {
        /**< Host connection is lost */
        GLOBAL_MyStatus |= STAT_ERROR_LOSSCOMM;
        GLOBAL_ExternalStatus |= STAT_ERROR_LOSSCOMM;
      } else
      {
        TIMEOUTS_Inc(TIMEOUT_TYPE_HOST);
        /**< Communication with host is Ok */
        GLOBAL_MyStatus &= ~STAT_ERROR_LOSSCOMM;
        /**< Is sticky now, to discuss */
        //GLOBAL_ExternalStatus &= ~EXTSTS_ERROR_LOSSCOMM;
      }
    }

    /**< Bit 12: Internal RS485 bus timeout */
    #ifdef DEF_DUPLEX
    //if (timeout_PartnerACE > (TIMEOUT_PARTNER_MS / LOGIC_TASK_DELAY_MS))
    if (TIMEOUTS_IsActive(TIMEOUT_TYPE_PARTNER) == true)
    {
      /**< Partner ACE is not online */
      GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
      GLOBAL_ExternalStatus |= STAT_ERROR_NOINTCOMM;
      /**< Reset status and position values for non-connected partner */
      GLOBAL_PartnerExtStatus = STAT_NO_PARTNER;
      /**< Reset all partner values */
      GLOBAL_PartnerMagnet = 0;
      GLOBAL_PartnerU1 = 0;
      GLOBAL_PartnerU2 = 0;
      GLOBAL_PartnerI = 0;
      GLOBAL_PartnerTempM = 0;
      GLOBAL_PartnerTempP = 0;
      GLOBAL_PartnerPower = 0;
      GLOBAL_PartnerIsMaster = false;
    } else
    {
      /**< Partner ACE is online */
      TIMEOUTS_Inc(TIMEOUT_TYPE_PARTNER);
      GLOBAL_MyStatus &= ~STAT_ERROR_NOINTCOMM;
      //GLOBAL_ExternalStatus &= ~STAT_ERROR_NOINTCOMM;
    }
    /**< Short timeout for slave to reset PWM value */
    //if ((GLOBAL_IsMaster == false) && (timeout_PartnerACE > (TIMEOUT_PARTNER_SHORT_MS / LOGIC_TASK_DELAY_MS)))
    if ((GLOBAL_IsMaster == false) && (TIMEOUTS_IsActive(TIMEOUT_TYPE_PARTNER_SHORT) == true))
    {
      LOGIC_ResetPWM();
    }
    #endif

    #ifdef DEF_DUPLEX
    /**< Bit 13: Heartbeat error */
    if (HEARTBEAT_Process() == false)
    {
      GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
      GLOBAL_ExternalStatus |= STAT_ERROR_HEARTBEAT;
    } else
    {
      GLOBAL_MyStatus &= ~STAT_ERROR_HEARTBEAT;
    }
    #endif

    /**< Bit 14: Current health status */
    if ((GLOBAL_MyStatus & (STAT_CRITICAL_MASK | STAT_NONCRITICAL_MASK)) > 0U)
    {
      GLOBAL_MyStatus |= STAT_ERROR_ACE;
      GLOBAL_ExternalStatus |= STAT_ERROR_ACE;
    }

    #ifdef DEF_DUPLEX
    /**< Bit 15: Master bit */
    if (GLOBAL_IsMaster == true)
    {
      GLOBAL_MyStatus |= STAT_MASTER_BIT;
      GLOBAL_ExternalStatus |= STAT_MASTER_BIT;
    } else
    {
      /**< This bit is not sticky */
      GLOBAL_MyStatus &= ~STAT_MASTER_BIT;
      GLOBAL_ExternalStatus &= ~STAT_MASTER_BIT;
    }
    #endif

    /**< Extended status */
    /**< Bit 0: Partner: Loss of communication with host */
    #ifdef DEF_DUPLEX
    if ((GLOBAL_PartnerStatus & STAT_ERROR_LOSSCOMM) > 0U)
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_LOSSCOMM2;
    }
    #endif

    /**< Bit 1: Partner: health status */
    #ifdef DEF_DUPLEX
    if ((GLOBAL_PartnerStatus & STAT_ERROR_ACE) > 0U)
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_ACE2;
    }
    #endif

    /**< Bit 2: Temperature sensor error */
    if (MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR) == false)
    {
      /**< Temperature sensor is faulty if out of range -40�...150� */
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_TEMP_SENSOR;
    } else
    {
      //GLOBAL_ExtStatus &= ~EXTSTAT_ERROR_TEMP_SENSOR;
    }

    /**< Bit 3: PCB sensor temperature error */
    if (MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB) == false)
    {
      /**< Temperature sensor is faulty if out of range -40�...150� */
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_PCB_TEMP;
    } else
    {
      //GLOBAL_ExtStatus &= ~EXTSTAT_ERROR_PCB_TEMP;
    }

    /**< Bit 4: Motor saver is active */
    if (MOTOR_IsSaverActive() == true)
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_MOTOR_SAVER_BIT;
    } else
    {
      //GLOBAL_ExtStatus &= ~EXTSTAT_MOTOR_SAVER_BIT;
    }

    /**< Bit 5: ID mismatch */
    #ifdef DEF_DUPLEX
    if (EE_StationId != GLOBAL_PartnerId)
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_ID_MISMATCH;
    }
    #endif

    /**< Bit 6: Extra Hall sensor error */
    #ifdef DEF_DUPLEX
    //if (timeout_Hall > (TIMEOUT_HALL_MS / LOGIC_TASK_DELAY_MS))
    if (TIMEOUTS_IsActive(TIMEOUT_TYPE_HALL) == true)
    {
      /**< Hall PCB is not online */
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_EXTHALL;
      GLOBAL_MagnetExtHall = 0xFFFFU;
    } else
    {
      /**< Hall PCB is online */
      TIMEOUTS_Inc(TIMEOUT_TYPE_HALL);
      //GLOBAL_ExtStatus &= ~EXTSTAT_ERROR_EXTHALL;
    }
    #endif

    /**< Bit 7: Error: free-running mode */
    if (motor_is_free == true)
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_ERROR_FREERUN;
    }
    /**< Bit 8: RAM integrity error */
    /**< Done above! */
    /**< Bit 9: Flash integrity error */
    /**< Done above! */

    /**< Bit 10: Error: degraded mode, only one ACE is running */
    #ifdef DEF_DUPLEX
    if ((((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) > 0U) || ((GLOBAL_PartnerStatus & STAT_CRITICAL_MASK) > 0U) ||
         ((GLOBAL_PartnerStatus & STAT_MOTOR_FREE_BIT) > 0U)) && (MOTOR_GetMode() != MOTOR_MODE_FREE))
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus | EXTSTAT_DEGRADED_BIT;
    } else
    {
      GLOBAL_ExtStatus = GLOBAL_ExtStatus & ~EXTSTAT_DEGRADED_BIT;
    }
    #endif

    /**< Magnet sensor errors and motor errors are processed in MOTOR_Task! */

    /**< Process current status of the servo */
    #ifdef DEF_DUPLEX
    if ((GLOBAL_RoleForce == (uint8_t)ROLE_FORCE_NONE) && (motor_is_free == false))
    {
      /**< Role is not forced */
      switch (REDUNDANCY_Process())
      {
        case REDUNDANCY_FREE:
          /**< Critical error, motor is free */
          MOTOR_SetMode(MOTOR_MODE_FREE);
          motor_is_free = true;
          /**< Must also be slave */
          HEARTBEAT_Disable();
          GLOBAL_IsMaster = false;
          MOTOR_DoSlaveFree(true);
          LOGIC_ResetPWM();
          break;
        case REDUNDANCY_SLAVE:
          /**< Become slave */
          HEARTBEAT_Disable();
          GLOBAL_IsMaster = false;
          LOGIC_ResetPWM();
          break;
        case REDUNDANCY_MASTER:
          /**< Become master */
          HEARTBEAT_Enable();
          GLOBAL_IsMaster = true;
          /**< Send current motor mode to slave */
          GLOBAL_ForceSlaveMode = MOTOR_GetMode();
          /**< Reset Heartbeat error to avoid false warnings */
          GLOBAL_ExternalStatus &= ~STAT_ERROR_HEARTBEAT;
          break;
        case REDUNDANCY_FAILSAFE:
          /**< Failsafe */
          if (EE_LossTime > 0U)
          {
            /**< Failsafe is activated */
            if (EE_LossBehavior == 0U)
            {
              /**< Go to failsafe position */
              GLOBAL_TargetPos = EE_LossPos;
            } else
            if (EE_LossBehavior > 0x0FU)
            {
              /**< Disable motor */
              MOTOR_SetMode(MOTOR_MODE_FREE);
              GLOBAL_ForceSlaveMode = SLAVE_MODE_FREE;
            } else
            {
              /**< Keep last position, do nothing */
            }
          }
          break;
        case REDUNDANCY_NO:
          /**< No changes */
          break;
        default:
          /**< Must not occur */
          break;
      }
    } else
    {
      /**< Role switching is forced */
      switch (GLOBAL_RoleForce)
      {
        case ROLE_FORCE_MASTER:
          /**< Force to be master */
          GLOBAL_IsMaster = true;
          LOGIC_ResetPWM();
          HEARTBEAT_Enable();
          /**< Send current motor mode to slave */
          GLOBAL_ForceSlaveMode = MOTOR_GetMode();
          INETWORK_ForceRole(true);
          if (GLOBAL_PartnerIsMaster == false)
          {
            /**< Partner is slave already, no need to force anymore */
            GLOBAL_RoleForce = ROLE_FORCE_NONE;
            INETWORK_ForceRole(false);
          }
          break;
        case ROLE_FORCE_SLAVE:
          /**< Force to be slave */
          GLOBAL_IsMaster = false;
          LOGIC_ResetPWM();
          HEARTBEAT_Disable();
          INETWORK_ForceRole(true);
          if (GLOBAL_PartnerIsMaster == true)
          {
            /**< Partner is master already, no need to force anymore */
            GLOBAL_RoleForce = ROLE_FORCE_NONE;
            INETWORK_ForceRole(false);
          }
          break;
        case ROLE_FORCE_OPPOSITE:
          /**< Must be repeated several times to be sure */
          force_cnt++;
          LOGIC_ResetPWM();
          if (GLOBAL_PartnerIsMaster == true)
          {
            GLOBAL_IsMaster = false;
            HEARTBEAT_Disable();
          } else
          {
            GLOBAL_IsMaster = true;
            HEARTBEAT_Enable();
            /**< Send current motor mode to slave */
            GLOBAL_ForceSlaveMode = MOTOR_GetMode();
          }
          if (force_cnt > FORCE_COUNT_MAX)
          {
            GLOBAL_RoleForce = ROLE_FORCE_NONE;
          }
          break;
        default:
          GLOBAL_RoleForce = ROLE_FORCE_NONE;
          break;
      }
    }
    #else
    /**< Single servo */
    if ((GLOBAL_MyStatus & STAT_CRITICAL_MASK) > 0U)
    {
      /**< Disable BLDC motor if error occurred */
      MOTOR_SetMode(MOTOR_MODE_FREE);
      motor_is_free = true;
    }
    if ((GLOBAL_MyStatus & STAT_ERROR_LOSSCOMM) > 0U)
    {
      /**< Failsafe behavior for simplex */
      if (EE_LossBehavior == 0U)
      {
        /**< Go to failsafe position */
        GLOBAL_TargetPos = EE_LossPos;
      } else
      if (EE_LossBehavior > 0x0FU)
      {
        /**< Disable motor */
        MOTOR_SetMode(MOTOR_MODE_FREE);
      } else
      {
        /**< Keep last position, do nothing */
      }
    }
    #endif

    /**< Process bootloader request */
    if ((GLOBAL_DoStartBootloader == BL_STOP_SIGN) && (GLOBAL_InternalID == ID_ACE1))
    {
      POWER_RestartApp();
    }

    #ifdef DEF_DUPLEX
      /**< Calibrate magnet sensor offset */
      if (GLOBAL_InternalID == ID_ACE2)
      {
        if (GLOBAL_DoSensorCalibration > 0U)
        {
          int16_t diff = (int16_t)GLOBAL_PartnerMagnet - ((int16_t)GLOBAL_MagnetValue - (int16_t)EE_MagSensorOffs);
          if (diff != EE_MagSensorOffs)
          {
            EE_MagSensorOffs = (int8_t)diff;
            EEPROM_SaveVariable(&EE_MagSensorOffs);
          }
        }
      }
    #endif

    /**< LED blinking */
    if (xTaskGetTickCount() >= ticks)
    {
      ticks += pdMS_TO_TICKS(LED_BLINKING_DELAY_MS);
      GPIO_TogglePin(LED_PORT, LED_PIN);
    }
    if (xTaskGetTickCount() >= ticks_1s)
    {
      /**< Every 1 second */
      ticks_1s += pdMS_TO_TICKS(1000);
      /**< Increment working time counter */
      GLOBAL_WorkingTime++;
    }
  }
}
#endif
