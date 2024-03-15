#include "canbus.h"
#include "chain.h"
#include "eeprom.h"
#include "global.h"
#include "monitor.h"
#include "motor.h"
#include "parser_can.h"
#include "version.h"

bool PARSER_ProcessCAN(TCanMsg *rx_msg, TCanMsg *tx_msg)
{
  uint32_t id;
  uint8_t  cmd;
  uint8_t  rec_id;
  int16_t  tmp;
  uint8_t  uval8;
  uint16_t uval16;
  uint32_t uval32;
  uint8_t  resp_cmd;
  uint8_t  resp_len;
  uint8_t  reply[CANBUS_BUF_LEN];
  static bool access_ee_bit = false;

  id = rx_msg->Id;
  /**< ID check moved to CAN Rx interrupt because of wrong LossComm behavior */
  cmd = (id & CANBUS_MASK_CMD) >> CANBUS_OFFS_CMD;
  rec_id = (id & CANBUS_MASK_ID) >> CANBUS_OFFS_ID;

  resp_cmd = 0;

  switch (cmd)
  {
    case CAN_CMD_SETPOS:
      /**< Set motor position */
      tmp = (int16_t)(((uint16_t)rx_msg->Data[0] << 8) + rx_msg->Data[1]);
      if (tmp > MAX_TARGET_POS)
        tmp = MAX_TARGET_POS;
      if (tmp < -MAX_TARGET_POS)
        tmp = -MAX_TARGET_POS;
      GLOBAL_TargetPos = tmp;
      reply[0] = (uint8_t)(GLOBAL_RealPos >> 8);
      reply[1] = (uint8_t)GLOBAL_RealPos;
      reply[2] = MONITOR_GetCurrent();
      reply[3] = MONITOR_GetVoltage(MONITOR_U_CH1);
      reply[4] = MONITOR_GetVoltage(MONITOR_U_CH2);
      uval32 = EEPROM_GetStallEvents();
      if (uval32 > 0xff)
        reply[5] = 0xff;
      else
        reply[5] = (uint8_t)uval32;
      reply[6] = (uint8_t)(GLOBAL_ExtStatus >> 8);
      reply[7] = (uint8_t)GLOBAL_ExtStatus;
      resp_len = 8;
      resp_cmd = CAN_CMD_GETPOS;
      break;

    case CAN_CMD_GETPOS:
      /**< Get motor position */
      reply[0] = (uint8_t)(GLOBAL_RealPos >> 8);
      reply[1] = (uint8_t)GLOBAL_RealPos;
      reply[2] = MONITOR_GetCurrent();
      reply[3] = MONITOR_GetVoltage(MONITOR_U_CH1);
      reply[4] = MONITOR_GetVoltage(MONITOR_U_CH2);
      uval32 = EEPROM_GetStallEvents();
      if (uval32 > 0xff)
        reply[5] = 0xff;
      else
        reply[5] = (uint8_t)uval32;
      reply[6] = (uint8_t)(GLOBAL_ExtStatus >> 8);
      reply[7] = (uint8_t)GLOBAL_ExtStatus;
      resp_len = 8;
      resp_cmd = CAN_CMD_GETPOS;
      break;

    case CAN_CMD_GETCOUNTERS:
      /**< Get load counters */
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_0);
      /**< Read minutes, seconds */
      reply[0] = (uint8_t)(uval32 >> 16);
      reply[1] = (uint8_t)(uval32 >> 8);
      reply[2] = (uint8_t)uval32;
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_25);
      reply[3] = (uint8_t)(uval32 >> 8);
      reply[4] = (uint8_t)uval32;
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_50);
      reply[5] = (uint8_t)(uval32 >> 8);
      reply[6] = (uint8_t)uval32;
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_75) + EEPROM_GetCounter(COUNTER_LOAD_100);
      if (uval32 > 0xff)
        uval32 = 0xff;
      reply[7] = (uint8_t)uval32;
      resp_len = 8;
      resp_cmd = CAN_CMD_GETCOUNTERS + 1;
      break;

    case CAN_CMD_GETSTATUS:
      /**< Get extended status */
      uval32 = EEPROM_GetPowerUps();
      if (uval32 > UINT16_MAX)
        uval32 = UINT16_MAX;
      reply[0] = (uint8_t)(uval32 >> 8);
      reply[1] = (uint8_t)uval32;
      reply[2] = MONITOR_GetTemperature(MONITOR_TEMP_PCB);
      reply[3] = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
      resp_len = 8;
      resp_cmd = CAN_CMD_GETSTATUS + 1;
      break;

    case CAN_CMD_SETID:
      /**< Change ID of the servo, saved internally */
      if (rec_id == CANBUS_BROADCAST)
      {
        reply[0] = rx_msg->Data[0];
        if (rx_msg->Data[0] == 0)
        {
          /**< Set ID */
          //if (CHAIN_GetInput() == false)
          //  break;
          uval8 = rx_msg->Data[1];
          if (uval8 > CAN_MAX_ID_VALUE)
            break;
          if (uval8 != 0)
          {
            EE_CanId = uval8;
            /**< Save value to EEPROM */
            EEPROM_SaveVariable(&EE_CanId);
            id = CANBUS_BuildReplyID(id, CAN_CMD_SETID + 1);
          } else
          {
            /**< Build ID for answering scan requests */
            id = CANBUS_BuildReplySetID(id, CAN_CMD_SETID + 1);
          }
        } else
        {
          /**< Get ID, just answer */
          id = CANBUS_BuildReplyID(id, CAN_CMD_SETID + 1);
        }
        /**< Send CAN reply */
        reply[1] = EE_CanId;
        resp_len = 2;
        resp_cmd = CAN_CMD_SETID + 1;
      } else
      {
        /**< No reply for non-broadcast command */
        resp_cmd = 0;
      }
      break;

    case CAN_CMD_GETVER:
      /**< Get FW version */
      reply[0] = DEVICE_VERSION[0];
      reply[1] = DEVICE_VERSION[1];
      /**< Get HW version */
      reply[2] = 0;
      reply[3] = 0;
      /**< Get checksums */
      uval16 = EEPROM_GetCRC();
      reply[4] = (uint8_t)(uval16 >> 8);
      reply[5] = (uint8_t)uval16;
      uval16 = (uint16_t)crc32_build;
      reply[6] = (uint8_t)(uval16 >> 8);
      reply[7] = (uint8_t)uval16;
      resp_len = 8;
      resp_cmd = CAN_CMD_GETVER + 1;
      break;

    case CAN_CMD_GETWORKTIME:
      /**< Get working time */
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_ALL);
      /**< Read minutes, seconds */
      reply[1] = (uval32 % 3600) / 60;
      reply[0] = uval32 % 60;
      /**< Read hours */
      uval16 = (uint16_t)(uval32 / 3600);
      reply[3] = (uint8_t)uval16;
      reply[2] = (uint8_t)(uval16 >> 8);
      resp_len = 4;
      resp_cmd = CAN_CMD_GETWORKTIME + 1;
      break;

    case CAN_CMD_RESETCOUNTERS:
      /**< Reset load counters */
      for (uval8 = COUNTER_LOAD_0; uval8 < COUNTER_LOAD_LAST; uval8++)
        EEPROM_ResetCounter(uval8);
      resp_len = 0;
      resp_cmd = CAN_CMD_RESETCOUNTERS + 1;
      break;

    case CAN_CMD_GETIBIT:
      /**< Get IBIT value */
      reply[0] = GLOBAL_Windings;
      reply[1] = MOTOR_GetError();
      resp_len = 2;
      resp_cmd = CAN_CMD_GETIBIT + 1;
      break;

    case CAN_CMD_GETDIGINPUT:
      /**< Get digital input */
      if (CHAIN_GetInput() == true)
        reply[0] = 1;
      else
        reply[0] = 0;
      resp_len = 1;
      resp_cmd = CAN_CMD_GETDIGINPUT + 1;
      break;

    case CAN_CMD_SETDIGOUT:
      /**< Set digital output, processed internally */
      /**< Broadcast messages will be processed here now */
      if (rx_msg->Data[0] == 0)
      {
        /**< Set/reset DO */
        if (rx_msg->Data[1] == 0)
        {
          CHAIN_SetOutput(false);
        } else
        {
          /**< Set DO = 1 only if DI = 1 */
          if (CHAIN_GetInput() == true)
            CHAIN_SetOutput(true);
        }
        reply[1] = rx_msg->Data[1];
      } else
      {
        /**< Read DO state */
        if (CHAIN_GetOutput() == true)
          reply[1] = 1;
        else
          reply[1] = 0;
      }
      reply[0] = rx_msg->Data[0];
      resp_len = 2;
      resp_cmd = CAN_CMD_SETDIGOUT + 1;
      break;

    case CAN_CMD_WRITEACCESS:
      /**< Write access for the next EEPROM operation */
      access_ee_bit = true;
      resp_len = 0;
      resp_cmd = CAN_CMD_WRITEACCESS + 1;
      break;

    case CAN_CMD_READEEPROM:
      /**< Read something from EEPROM (1 byte) */
      reply[0] = rx_msg->Data[0];
      reply[1] = EEPROM_Page[rx_msg->Data[0]];
      /**< Get EEPROM CRC, big endian */
      reply[2] = EEPROM_Page[1];
      reply[3] = EEPROM_Page[0];
      resp_len = 4;
      resp_cmd = CAN_CMD_READEEPROM + 1;
      break;

    case CAN_CMD_WRITEEEPROM:
      /**< Write something to EEPROM (1 byte) */
      if (access_ee_bit == true)
      {
        reply[0] = rx_msg->Data[0];
        reply[1] = rx_msg->Data[1];
        //EEPROM_Page[reply[0]] = reply[1];
        EEPROM_WriteCell(reply[0], reply[1]);
        /**< Recalculate EEPROM CRC */
        EEPROM_RecalculateCRC();
        reply[2] = EEPROM_Page[1];
        reply[3] = EEPROM_Page[0];
        resp_len = 4;
        resp_cmd = CAN_CMD_WRITEEEPROM + 1;
        access_ee_bit = false;
      }
      break;

    case CAN_CMD_SETPOSOFFSET:
      /**< Set/read position offset */
      resp_len = 3;
      resp_cmd = CAN_CMD_SETPOSOFFSET + 1;
      switch (rx_msg->Data[0])
      {
        case 0:
          /**< Set zero position offset */
          uval16 = ((uint16_t)rx_msg->Data[1] << 8) + rx_msg->Data[2];
          EE_ZeroPos = uval16;
          EEPROM_SaveVariable((void*)&EE_ZeroPos);
          memcpy(reply, rx_msg->Data, 3);
          break;
        case 1:
          /**< Read zero position offset */
          reply[0] = 1;
          reply[1] = (uint8_t)(EE_ZeroPos >> 8);
          reply[2] = (uint8_t)EE_ZeroPos;
          break;
        default:
          resp_cmd = 0;
          break;
      }
      break;

    case CAN_CMD_RESETBITS:
      /**< Reset error bits (set default settings) */
      resp_len = 0;
      resp_cmd = CAN_CMD_WRITEACCESS + 1;
      break;

    case CAN_CMD_STARTBL:
      /**< Jump to bootloader */
      if ((rx_msg->Data[0] == 0))// && (*(uint32_t*)&rx_msg.Data[1] == SIGNATURE_START))
      {
        GLOBAL_DoStartBootloader = BL_START_SIGN;
      }
      resp_cmd = 0;
      break;

    default:
      resp_cmd = 0;
      break;
  }

  if (resp_cmd == 0)
    return false;

  tx_msg->Len = resp_len;
  tx_msg->Id = CANBUS_BuildReplyID(id, resp_cmd);
  memcpy(tx_msg->Data, reply, resp_len);

  return true;
}
