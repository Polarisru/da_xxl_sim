#include "drivers.h"
#include "canbus.h"
#include "can_volz.h"
#include "conversion.h"
#include "eeprom.h"
#include "global.h"
#include "logic.h"
#include "monitor.h"
#include "motor.h"
#include "parser_can.h"
#include "timeouts.h"
#include "version.h"

/** \brief Build reply for GetPosition command
 *
 * \param [out] data Pointer to data buffer
 * \return Length of reply as uint8_t
 *
 */
static uint8_t PARSER_PosReply(uint8_t *data)
{
  uint8_t len = 0U;
  uint16_t uval16;

  /**< Position */
  uval16 = CONVERSION_CalcDegreesFromPos(GLOBAL_RealPos) & 0xFFFU;
  data[CANMSG_OFFSET_DATA] = (uint8_t)uval16;
  data[CANMSG_OFFSET_DATA + 1U] = (uint8_t)(uval16 >> 8U);
  len += 2U;
  /**< Status */
  data[CANMSG_OFFSET_DATA + len] = CONVERSION_BuildVolzStatus(GLOBAL_ExternalStatus);
  len++;
  /**< Current */
  data[CANMSG_OFFSET_DATA + len] = MONITOR_GetCurrent();
  len++;
  /**< Voltage */
  data[CANMSG_OFFSET_DATA + len] = MONITOR_GetVoltage(MONITOR_U_CH1);
  len++;
  /**< PWM value */
  if ((MOTOR_GetMode() == MOTOR_MODE_FREE) || (MOTOR_GetMode() == MOTOR_MODE_BRAKE))
  {
    data[CANMSG_OFFSET_DATA + len] = 0;
  } else
  {
    data[CANMSG_OFFSET_DATA + len] = GLOBAL_Power;
  }
  len++;
  /**< Motor temperature */
  data[CANMSG_OFFSET_DATA + len] = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
  len++;

  return len;
}

/** \brief
 *
 * \param rx_msg TCanMsg*
 * \param tx_msg struct can_message*
 * \return bool
 *
 */
bool PARSER_CanVolz(TCanMsg *rx_msg, struct can_message *tx_msg)
{
  uint8_t rec_id;
  uint8_t cmd;
  uint16_t uval16;
  uint8_t uval8;
  uint32_t uval32;
  int16_t tmp = 0;
  eeVal_t param;
  uint8_t opcode;
  bool boolVal;

  rec_id = (uint8_t)(rx_msg->id & CANMSG_ID_MASK);
  cmd = rx_msg->data[CANMSG_OFFSET_CMD] & CANMSG_CMD_MASK;
  /**< Allow own ID, broadcast ID and group message for SET_GROUP_POS command */
  if (((rx_msg->id & CANBUS_ID_OWN_MASK) == EE_CanId) && \
      ((rec_id == EE_DevId) || (rec_id == CANMSG_ID_BROADCAST) || ((cmd == CANMSG_CMD_SET_GROUP_POS) && (EE_DevId >= rec_id))))
  {
    /**< Reset communication timeout */
    TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
    tx_msg->fmt = CAN_FMT_STDID;
    tx_msg->id = EE_CanId | (uint16_t)EE_DevId | CANBUS_ID_REPLY_BIT;
    tx_msg->data[CANMSG_OFFSET_CMD] = cmd;
    tx_msg->len = CANMSG_OFFSET_DATA;
    tx_msg->type = CAN_TYPE_DATA;
    /**< Standard CAN message */
    switch (cmd)
    {
      case CANMSG_CMD_ECHO:
        /**< Just echo current message */
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA], &rx_msg->data[CANMSG_OFFSET_DATA], rx_msg->len);
        tx_msg->len = rx_msg->len;
        break;

      case CANMSG_CMD_SET_POS:
        /**< Set servo position */
        uval16 = (rx_msg->data[CANMSG_OFFSET_DATA + 1U] << 8U) + rx_msg->data[CANMSG_OFFSET_DATA];
        tmp = CONVERSION_CalcPosFromDegrees(uval16);
        if (tmp < SERVO_MIN_TP)
        {
          tmp = SERVO_MIN_TP;
        }
        if (tmp > SERVO_MAX_TP)
        {
          tmp = SERVO_MAX_TP;
        }
        GLOBAL_TargetPos = tmp;
        /**< Activate motor */
        MOTOR_SetMode(MOTOR_MODE_RUN1);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
        tx_msg->len += PARSER_PosReply(tx_msg->data);
        break;

      case CANMSG_CMD_GET_POS:
        /**< Read position, status, current, voltage, PWM value, temperature */
        tx_msg->len += PARSER_PosReply(tx_msg->data);
        break;

      case CANMSG_CMD_GET_STAT:
        /**< Get extended status */
        /**< Extended status */
        tx_msg->data[CANMSG_OFFSET_DATA] = (uint8_t)GLOBAL_ExternalStatus;
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = (uint8_t)(GLOBAL_ExternalStatus >> 8U);
        /**< Motor status (errors only) */
        tx_msg->data[CANMSG_OFFSET_DATA + 2U] = MOTOR_GetError();
        /**< PCB temperature */
        tx_msg->data[CANMSG_OFFSET_DATA + 3U] = MONITOR_GetTemperature(MONITOR_TEMP_PCB);
        /**< Stalls counter */
        //uval16 = EEPROM_GetStallEvents();
        //tx_msg->data[CANMSG_OFFSET_DATA + 4U] = (uint8_t)uval16;
        //tx_msg->data[CANMSG_OFFSET_DATA + 5U] = (uint8_t)(uval16 >> 8);
        /**< Second voltage */
        tx_msg->data[CANMSG_OFFSET_DATA + 4U] = MONITOR_GetVoltage(MONITOR_U_CH2);
        /**< Maximal current */
        tx_msg->data[CANMSG_OFFSET_DATA + 6U] = MONITOR_GetMaxCurrent();
        tx_msg->len += 7U;
        break;

      case CANMSG_CMD_GET_TIME:
        /**< Get working time */
        uval32 = GLOBAL_WorkingTime;
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA], &uval32, 3);
        uval32 = EEPROM_GetCounter(COUNTER_LOAD_ALL);
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + 3U], &uval32, sizeof(uint32_t));
        tx_msg->len += 7U;
        break;

      case CANMSG_CMD_GET_PARAM:
        /**< Read servo parameter */
        uval8 = rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS] & CANMSG_PARAM_ID_MASK;
        tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS] = rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS];
        tx_msg->len++;
        boolVal = EEPROM_GetParam(uval8, &param);
        if ((rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS] & CANMSG_PARAM_NAME_BIT) > 0U)
        {
          if (boolVal == false)
          {
            (void)memset(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS], 0, PARAM_NAME_LEN);
            tx_msg->len += PARAM_NAME_LEN;
            break;
          }
          /**< Name is placed to OPCODE offset because of length */
          (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS], param.name, PARAM_NAME_LEN);
          tx_msg->len += PARAM_NAME_LEN;
        } else
        {
          /**< Type of parameter */
          if (rx_msg->len == 2U)
          {
            /**< Short command, just read value */
            opcode = CANMSG_PARAM_VAL;
          } else
          {
            /**< Normal command, get opcode */
            opcode = (rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS] >> CANMSG_PARAM_OPCODE_OFFS) & CANMSG_PARAM_OPCODE_MASK;
          }
          if (boolVal == false)
          {
            param.type = PARAM_TYPE_EMPTY;
          }
          tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS] = (opcode << CANMSG_PARAM_OPCODE_OFFS) | (param.type << CANMSG_PARAM_TYPE_OFFS);
          tx_msg->len++;
          if (boolVal == false)
          {
            break;
          }
          switch (opcode)
          {
            case CANMSG_PARAM_VAL:
              (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS], param.pVal, param.size);
              tx_msg->len += param.size;
              break;
            case CANMSG_PARAM_MINVAL:
              (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS], &param.minVal, param.size);
              tx_msg->len += param.size;
              break;
            case CANMSG_PARAM_MAXVAL:
              (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS], &param.maxVal, param.size);
              tx_msg->len += param.size;
              break;
            case CANMSG_PARAM_DFLTVAL:
              (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS], &param.defVal, param.size);
              tx_msg->len += param.size;
              break;
            default:
              return false;
          }
        }
        break;

      case CANMSG_CMD_SET_PARAM:
        /**< Write servo parameter */
        uval8 = rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS] & CANMSG_PARAM_ID_MASK;
        if (EEPROM_GetParam(uval8, &param) == false)
        {
          return false;
        }
        opcode = (rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS] >> CANMSG_PARAM_OPCODE_OFFS) & CANMSG_PARAM_OPCODE_MASK;
        if (opcode != CANMSG_PARAM_VAL)
        {
          return false;
        }
        EEPROM_SetParam(uval8, (void*)&rx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS]);
        /**< Build reply with param data */
        tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_ID_OFFS] = uval8;
        tx_msg->len++;
        tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_CMD_OFFS] = (opcode << CANMSG_PARAM_OPCODE_OFFS) | (param.type << CANMSG_PARAM_TYPE_OFFS);
        tx_msg->len++;
        (void)EEPROM_GetParam(uval8, &param);
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + CANMSG_PARAM_DATA_OFFS], param.pVal, param.size);
        tx_msg->len += param.size;
        break;

      case CANMSG_CMD_GET_HWFW:
        /**< Get HW/FW revisions and power-ups */
        /**< FW revision */
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA], VERSION_GetFW(), sizeof(uint16_t));
        tx_msg->len += sizeof(uint16_t);
        /**< HW revision */
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + sizeof(uint16_t)], VERSION_GetHW(), sizeof(uint16_t));
        tx_msg->len += sizeof(uint16_t);
        /**< Power-ups */
        uval16 = EEPROM_GetPowerUps();
        tx_msg->data[CANMSG_OFFSET_DATA + sizeof(uint16_t) * 2U] = (uint8_t)uval16;
        tx_msg->data[CANMSG_OFFSET_DATA + sizeof(uint16_t) * 2U + 1U] = (uint8_t)(uval16 >> 8U);
        tx_msg->len += sizeof(uint16_t);
        break;

      case CANMSG_CMD_SET_GROUP_POS:
        /**< Set group position, up to 4 sequential IDs can be commanded */
        switch (EE_DevId - rec_id)
        {
          case 0:
            (void)memcpy((void*)&uval16, &rx_msg->data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
            break;
          case 1:
            if (rx_msg->len < 4U)
            {
              return false;
            }
            (void)memcpy((void*)&uval16, &rx_msg->data[CANMSG_OFFSET_DATA + 1U], sizeof(uint16_t));
            uval16 >>= 4U;
            break;
          case 2:
            if (rx_msg->len < 6U)
            {
              return false;
            }
            (void)memcpy((void*)&uval16, &rx_msg->data[CANMSG_OFFSET_DATA + 3U], sizeof(uint16_t));
            break;
          case 3:
            if (rx_msg->len < 7U)
            {
              return false;
            }
            (void)memcpy((void*)&uval16, &rx_msg->data[CANMSG_OFFSET_DATA + 4U], sizeof(uint16_t));
            uval16 >>= 4U;
            break;
          default:
            return false;
        }
        tmp = CONVERSION_CalcPosFromDegrees(uval16);
        GLOBAL_TargetPos = tmp;
        /**< Activate motor */
        MOTOR_SetMode(MOTOR_MODE_RUN1);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
        /**< Build reply */
        tx_msg->len += PARSER_PosReply(tx_msg->data);
        break;

      case CANMSG_CMD_GET_STRING:
        /**< Get value from strings area */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = EEPROM_Strings[rx_msg->data[CANMSG_OFFSET_DATA]];
        tx_msg->len += 2U;
        break;

      case CANMSG_CMD_SET_STRING:
        /**< Set value of strings area */
        EEPROM_Strings[rx_msg->data[CANMSG_OFFSET_DATA]] = rx_msg->data[CANMSG_OFFSET_DATA + 1U];
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = rx_msg->data[CANMSG_OFFSET_DATA + 1U];
        tx_msg->len += 2U;
        break;

      case CANMSG_CMD_EXECUTE:
        /**< Perform any action for motor testing or configuration */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = CANMSG_REPLY_OK;
        tx_msg->len += 2U;
        switch (rx_msg->data[CANMSG_OFFSET_DATA])
        {
          case CANMSG_EXECUTE_NORMMODE:
            /**< Go to normal operation mode */
            MOTOR_SetMode(MOTOR_MODE_RUN1);
//            #ifdef DEF_DUPLEX
//            GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//            #endif
            break;
          case CANMSG_EXECUTE_FREE:
            /**< Go to free-running mode */
            MOTOR_SetMode(MOTOR_MODE_FREE);
            #ifdef DEF_DUPLEX
            GLOBAL_ForceSlaveMode = SLAVE_MODE_FREE;
            #endif
            break;
          case CANMSG_EXECUTE_ROTCW:
            /**< Do permanent CW rotation */
            MOTOR_SetMode(MOTOR_MODE_GOCW);
            MOTOR_SetExtPower(rx_msg->data[CANMSG_OFFSET_DATA + 1U]);
//            #ifdef DEF_DUPLEX
//            GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//            #endif
            break;
          case CANMSG_EXECUTE_ROTCCW:
            /**< Do permanent CCW rotation */
            MOTOR_SetMode(MOTOR_MODE_GOCCW);
            MOTOR_SetExtPower(rx_msg->data[CANMSG_OFFSET_DATA + 1U]);
//            #ifdef DEF_DUPLEX
//            GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//            #endif
            break;
          case CANMSG_EXECUTE_NEUTRAL:
            /**< Set current position as neutral position */
            //EE_ZeroPos = CONVERSIONS_CalcDegreesFromPos(GLOBAL_RawPos); //???
            EE_ZeroPos = (0x800U - GLOBAL_MagnetValue) & 0xFFFU;
            EEPROM_SaveVariable(&EE_ZeroPos);
            /**< Set target position = 0 to avoid movement after this command */
            GLOBAL_TargetPos = 0;
            break;
          case CANMSG_EXECUTE_SAVESTR:
            /**< Save EEPROM strings to non-volatile memory */
            /**< Check passphrase first */
            (void)memcpy((void*)&uval32, &rx_msg->data[CANMSG_OFFSET_DATA + 3U], sizeof(uint32_t));
            if (uval32 == CANMSG_EXECUTE_PASS)
            {
              EEPROM_SaveStrings();
            } else
            {
              tx_msg->data[CANMSG_OFFSET_DATA + 1U] = CANMSG_REPLY_ERR;
            }
            break;
          case CANMSG_EXECUTE_RESETSTS:
            /**< Reset status */
            GLOBAL_MyStatus = 0U;
            GLOBAL_ExternalStatus = 0U;
            break;
          case CANMSG_EXECUTE_MOTORTEST:
            /**< Start BLDC motor test */
            // Why inactive??
            {
              uint8_t val1 = 0;
              uint8_t val2 = 0;
              MOTOR_DoTest(&val1, &val2);
            }
            break;
          default:
            tx_msg->data[CANMSG_OFFSET_DATA + 1U] = CANMSG_REPLY_ERR;
            break;
        }
        break;

      #ifdef DEF_DUPLEX
      case CANMSG_CMD_DUPLEX:
        /**< Perform any action for duplex actuator */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = CANMSG_REPLY_OK;
        tx_msg->len += 2U;
        switch (rx_msg->data[CANMSG_OFFSET_DATA])
        {
          case CANMSG_DUPLEX_FORCEMASTER:
            TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
            GLOBAL_RoleForce = ROLE_FORCE_MASTER;
            break;
          case CANMSG_DUPLEX_FORCESLAVE:
            TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
            GLOBAL_RoleForce = ROLE_FORCE_SLAVE;
            break;
          default:
            tx_msg->data[CANMSG_OFFSET_DATA + 1U] = CANMSG_REPLY_ERR;
            break;
        }
        break;
      #endif

      #ifdef DEF_DUPLEX
      case CANMSG_CMD_GET_COUNTER:
        /**< Get load counters */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->len += (1U + sizeof(uint32_t));
        uval8 = rx_msg->data[CANMSG_OFFSET_DATA];
        if (uval8 >= COUNTER_LOAD_LAST)
        {
          return false;
        }
        uval32 = EEPROM_GetCounter(uval8);
        (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA + 1U], (void*)&uval32, sizeof(uint32_t));
        break;
      #endif

      case CANMSG_CMD_GET_REVISION:
        /**< Get revision string */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = VERSION_GetRevisionStr(rx_msg->data[CANMSG_OFFSET_DATA]);
        tx_msg->len += 2U;
        break;

      case CANMSG_CMD_READ_EE:
        /**< Read EEPROM data */
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = EEPROM_ReadByte((uint16_t)rx_msg->data[CANMSG_OFFSET_DATA]);
        tx_msg->len += 2U;
        break;

      case CANMSG_CMD_WRITE_EE:
        /**< Write EEPROM data */
        EEPROM_WriteByte((uint16_t)rx_msg->data[CANMSG_OFFSET_DATA], rx_msg->data[CANMSG_OFFSET_DATA + 1U]);
        tx_msg->data[CANMSG_OFFSET_DATA] = rx_msg->data[CANMSG_OFFSET_DATA];
        tx_msg->data[CANMSG_OFFSET_DATA + 1U] = rx_msg->data[CANMSG_OFFSET_DATA + 1U];
        tx_msg->len += 2U;
        break;

      case CANMSG_CMD_BOOTLOADER:
        /**< Start bootloader */
        (void)memcpy((void*)&uval32, &rx_msg->data[CANMSG_OFFSET_DATA + 3U], sizeof(uint32_t));
        if ((rx_msg->data[CANMSG_OFFSET_DATA] == FLASH_CMD_STARTBL) && (uval32 == CANMSG_BL_SIGN_START))
        {
          /**< Initialize internal bus reset (duplex) */
          GLOBAL_DoStartBootloader = BL_START_SIGN;
          vTaskDelay(50);
          #ifndef DEF_UNITTEST
            WDT_Restart();
          #endif
        }
        return false;

      default:
        return false;
    }
  } else
  if (rx_msg->id == CANBUS_BACKDOOR_ID)
  {
    /**< Backdoor ID to get real IDs */
    cmd = rx_msg->data[CANMSG_OFFSET_CMD] & CANMSG_CMD_MASK;
    if (cmd == CANMSG_CMD_ECHO)
    {
      tx_msg->fmt = CAN_FMT_STDID;
      tx_msg->id = EE_CanId | (uint16_t)EE_DevId | CANBUS_ID_REPLY_BIT;
      tx_msg->data[CANMSG_OFFSET_CMD] = CANMSG_CMD_ECHO;// | CANMSG_CMD_REPLY_BIT;
      (void)memcpy(&tx_msg->data[CANMSG_OFFSET_DATA], &rx_msg->data[CANMSG_OFFSET_DATA], rx_msg->len);
      tx_msg->len = rx_msg->len;
    }
  } else
  {
    /**< Unknown command or ID, don't reply */
    return false;
  }

  if ((rx_msg->data[CANMSG_OFFSET_CMD] & CANMSG_CMD_SILENT_BIT) > 0U)
  {
    /**< Don't reply silent commands */
    return false;
  }

  return true;
}
