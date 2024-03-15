#include "cmd_set.h"
#include "conversion.h"
#include "crc32.h"
#include "eeprom.h"
#include "global.h"
#include "hall.h"
#include "logic.h"
#include "magnet.h"
#include "monitor.h"
#include "motor.h"
#include "notch.h"
#include "parser_rs485.h"
#include "rs485.h"
#include "timeouts.h"
#include "utils.h"
#include "version.h"
#include "inetwork.h"

/** \brief RS485 command parser
 *
 * \param [in] input Pointer to structure with input data
 * \param [out] output Pointer to structure with output data
 * \return void
 *
 */
void PARSER_ProcessRS485(TCommData *input, TCommData *output)
{
  bool     broadcast;
  uint8_t  cmd;
  uint8_t  arg_1;
  uint8_t  arg_2;
  uint8_t  resp_c = 0;
  uint8_t  resp_1;
  uint8_t  resp_2;
  int16_t  tmp;
  uint32_t uval32;
  uint16_t uval16;
  uint8_t  uval8;
  uint8_t  gap;
  bool     clear_accessbit = true;
  static bool freshStart = false;
  static bool access_ee_bit = false;
  static uint8_t freshness_counter = 0;
  #ifdef DEF_DUPLEX
  static uint8_t freshness_tx = 0xFFU;
  static uint8_t freshness_error = 0;
  #endif

  cmd = input->cmd;
  arg_1 = input->arg_1;
  arg_2 = input->arg_2;

  /**< Reset reply values */
  resp_c = 0;
  resp_1 = 0;
  resp_2 = 0;

  if (input->id == RS485_BROADCAST_ID)
  {
    broadcast = true;
  } else
  {
    broadcast = false;
  }

  switch (cmd)
  {
    case C_Set_Pos170:
    case C_Set_Pos170_Silent:
      /**< Set new servo position with response or without response */
      /**< Reset connection timeout counter */
      TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
      if (broadcast == true)
      {
        break;
      }
      if (cmd == C_Set_Pos170)
      {
        resp_c = R_Set_Pos170;
      }
      MOTOR_SetMode(MOTOR_MODE_RUN1);
//      #ifdef DEF_DUPLEX
//      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//      #endif
      /**< Calculate freshness difference */
      uval8 = arg_1 >> 4;
      gap = (uval8 - (freshness_counter + 1U)) & 0x0FU;
      if (freshStart == true)
      {
        /**< Freshness counter analysis starts after first position receiving */
        if ((EE_FreshCntLevel < FRESHNESS_OFF_LEVEL) && (gap > EE_FreshCntLevel))
        {
          /**< Set flag for freshness-counter problem */
          GLOBAL_ErrFreshness = true;
        } else
        {
          /**< Reset freshness flag */
          GLOBAL_ErrFreshness = false;
        }
        #ifdef DEF_DUPLEX
        freshness_error = gap;
        #endif
      } else
      {
        freshStart = true;
        #ifdef DEF_DUPLEX
        freshness_error = 0;
        #endif
      }
      /**< Store expected freshness counter value for control by next position */
      freshness_counter = uval8;
      /**< Build new target position */
      GLOBAL_TargetPos = CONVERSION_CalcPosFromSetPos170(arg_1, arg_2);
      tmp = GLOBAL_RealPos;
      /**< Current position high byte */
      resp_1 = (uint8_t)(((uint16_t)tmp >> 8) & 0x0FU);
      #ifdef DEF_DUPLEX
        /**< + decremented freshness counter */
        resp_1 += (uint8_t)(freshness_tx << 4);
        freshness_tx--;
      #endif
      /**< Current position low byte */
      resp_2 = (uint8_t)tmp;
      break;

    case C_Act_Pos170_report:
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      resp_c = R_Act_Pos170_report;
      #ifdef DEF_DUPLEX
      /**< 4high bits are 0, duplex only */
      tmp =  GLOBAL_RealPos & 0xFFF;
      #else
      tmp =  GLOBAL_RealPos;
      #endif
      resp_1 = (uint8_t)((uint16_t)tmp >> 8);
      resp_2 = (uint8_t)tmp;
      break;

    case C_SET_POS100:
      /**< Set new Servo position (legacy Kearfott kompatible) */
      /**< Reset connection timeout counter */
      TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
      GLOBAL_TargetPos = CONVERSION_CalcPosFromSetPos100(arg_1, arg_2);
      MOTOR_SetMode(MOTOR_MODE_RUN1);
//      #ifdef DEF_DUPLEX
//      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//      #endif
      resp_c = R_SET_POS100;
      if (broadcast == true)
      {
        /**< Don't answer broadcast commands */
        resp_c = 0;
        break;
      }
      /**< formula: 2048 + ((2048 - RealPos(magnet))*angular_ratio) */
      //tmp = (GLOBAL_RealPos * 1.687F) + 0x800;
      //resp_1 = (uint8_t)((uint16_t)tmp >> 8);
      //resp_2 = (uint8_t)tmp;
      /**< high byte: 0 0 0 bit11 bit10 bit9 bit8 0 */
      //resp_1 = ((uint8_t)(resp_1 << 1U) & 0x1EU);
      /**< add bit7 as bit 0 to high byte: 0 0 0 bit11 bit10 bit9 bit8 bit7 */
      //if ((resp_2 & 0x80U) > 0U)
      //{
      //  resp_1++;
      //}
      /**< low byte: 0 bit6 bit5 bit4 bit3 bit2 bit1 bit0 */
      //resp_2 = resp_2 & 0x7FU;
      uval16 = CONVERSION_CalcResponseForSetPos100(GLOBAL_RealPos);
      resp_1 = (uint8_t)(uval16 >> 8);
      resp_2 = (uint8_t)uval16;
      break;

    case C_Act_Pos100_report:
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      resp_c = R_Act_Pos100_report;
      /**< formula: 2048 + ((2048 - RealPos(magnet))*angular_ratio) */
      //tmp = (GLOBAL_RealPos * 1.687F) + 0x800;
      //resp_1 = (uint8_t)((uint16_t)tmp >> 8);
      //resp_2 = (uint8_t)tmp;
      /**< high byte: 0 0 0 bit11 bit10 bit9 bit8 0 */
      //resp_1 = ((uint8_t)(resp_1 << 1U) & 0x1EU);
      /**< add bit7 as bit 0 to high byte: 0 0 0 bit11 bit10 bit9 bit8 bit7 */
      //if ((resp_2 & 0x80U) > 0U)
      //{
      //  resp_1++;
      //}
      /**< low byte: 0 bit6 bit5 bit4 bit3 bit2 bit1 bit0 */
      //resp_2 = resp_2 & 0x7FU;
      uval16 = CONVERSION_CalcResponseForSetPos100(GLOBAL_RealPos);
      resp_1 = (uint8_t)(uval16 >> 8);
      resp_2 = (uint8_t)uval16;
      break;

    case C_Set_PosExt:
      /**< Reset connection timeout counter */
      TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
      if (broadcast == true)
      {
        break;
      }
      MOTOR_SetMode(MOTOR_MODE_RUN1);
//      #ifdef DEF_DUPLEX
//      GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//      #endif
//      uval16 = ((uint16_t)arg_1 << 8) + arg_2;
//      if (uval16 < MIN_TP_EXT)
//      {
//        uval16 = MIN_TP_EXT;
//      }
//      if (uval16 > MAX_TP_EXT)
//      {
//        uval16 = MAX_TP_EXT;
//      }
//      GLOBAL_TargetPos = ((int16_t)uval16 - 0x800) * 0.5933F;
      GLOBAL_TargetPos = CONVERSION_CalcPosFromSetPosExt(arg_1, arg_2);
      resp_c = R_Set_PosExt;
      /**< Formula: 2048 + ((2048 - RealPos(magnet))*angular_ratio) */
      //uval16 = (uint16_t)((GLOBAL_RealPos * 1.687F) + 0x800);
      uval16 = CONVERSION_CalcResponseForSetPosExt(GLOBAL_RealPos);
      resp_1 = (uint8_t)(uval16 >> 8);
      resp_2 = (uint8_t)uval16;
      break;

    case C_Act_PosExt_report:
      if (broadcast == true)
      {
        break;
      }
      resp_c = R_Act_PosExt_report;
      //uval16 = (uint16_t)((GLOBAL_RealPos * 1.687F) + 0x800);
      uval16 = CONVERSION_CalcResponseForSetPosExt(GLOBAL_RealPos);
      resp_1 = (uint8_t)(uval16 >> 8);
      resp_2 = (uint8_t)uval16;
      break;

    case C_StatusRW:
      if (broadcast == true)
      {
        break;
      }
      //resp_c = g_MagSnsComparator + 0x80; ??
      break;

    #ifdef DEF_DUPLEX
    case C_ACE2_as_Master:
      /**< Make ACE2 (Standby) master */
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 != SIGN_COMM_1) || (arg_2 != SIGN_COMM_2))
      {
        break;
      }
      resp_c = R_ACE2_as_Master;
      if (GLOBAL_InternalID == ID_ACE2)
      {
        TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
        GLOBAL_RoleForce = ROLE_FORCE_MASTER;
      } else
      {
        if ((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) == 0U)
        {
          /**< Perform only with connected ACE2 */
          TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
          GLOBAL_RoleForce = ROLE_FORCE_SLAVE;
        }
      }
      resp_1 = SIGN_COMM_2;
      resp_2 = SIGN_COMM_1;
      break;
    #endif

    #ifdef DEF_DUPLEX
    case C_ACE1_as_Master:
      /**< Make ACE1 (Active) master */
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 != SIGN_COMM_1) || (arg_2 != SIGN_COMM_2))
      {
        break;
      }
      resp_c = R_ACE1_as_Master;
      if (GLOBAL_InternalID == ID_ACE1)
      {
        TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
        GLOBAL_RoleForce = ROLE_FORCE_MASTER;
      } else
      {
        if ((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) == 0U)
        {
          /**< Perform only with connected ACE1 */
          TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
          GLOBAL_RoleForce = ROLE_FORCE_SLAVE;
        }
      }
      resp_1 = SIGN_COMM_2;
      resp_2 = SIGN_COMM_1;
      break;
    #endif

    #ifdef DEF_DUPLEX
    case C_Read_Role:
      /**< Read current role */
      if ((arg_1) || (broadcast == true))
      {
        break;
      }
      resp_c = R_Read_Role;
      switch (arg_2)
      {
        case 0:
          resp_1 = SIGN_COMM_1;
          if (GLOBAL_IsMaster == true)
          {
            resp_2 = 1;
          } else
          {
            resp_2 = 0;
          }
          break;
        case 1:
          /**< Get dropped frames number */
          resp_1 = freshness_counter;
          resp_2 = freshness_error;
          break;
        case 2:
          /**< Reset dropped frames number */
          freshness_error = 0;
          resp_1 = freshness_counter;
          resp_2 = freshness_error;
          break;
        default:
          /**< Unknown argument, ignore */
          resp_c = 0;
          break;
      }
      break;
      #endif

    case C_Read_Current:
      /**< Read current value [x20mA] */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      resp_c = R_Read_Current;
      switch (arg_2)
      {
        case 0:
          /**< 0 = readout my current consumption */
          resp_1 = MONITOR_GetCurrent();
          resp_2 = MONITOR_GetCurrent();
          break;
        #ifdef DEF_DUPLEX
        case 1:
          /**< 1 = readout partner's current consumption */
          resp_1 = GLOBAL_PartnerI;
          resp_2 = GLOBAL_PartnerI;
          break;
        #endif
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_Read_Voltage:
      /**< Read voltage value [x200mV] */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      resp_c = R_Read_Voltage;
      switch (arg_2)
      {
        case 0:
          /**< 0 = readout my voltage: primary, secondary */
          resp_1 = MONITOR_GetVoltage(MONITOR_U_CH1);
          resp_2 = MONITOR_GetVoltage(MONITOR_U_CH2);
          break;
        #ifdef DEF_DUPLEX
        case 1:
          /**< 1 = readout partner's voltage: primary, secondary */
          resp_1 = GLOBAL_PartnerU1;
          resp_2 = GLOBAL_PartnerU2;
          break;
        #endif
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_Read_Temperature:
    case C_Read_Temperature_V:
      /**< Read temperature value */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      if (cmd == C_Read_Temperature)
      {
        resp_c = R_Read_Temperature;
      } else
      {
        resp_c = R_Read_Temperature_V;
      }
      switch (arg_2)
      {
        case 0:
          /**< Get own temperature: motor / PCB */
          resp_1 = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
          resp_2 = MONITOR_GetTemperature(MONITOR_TEMP_PCB);
          break;
        #ifdef DEF_DUPLEX
        case 1:
          /**< Get partner temperature */
          resp_1 = GLOBAL_PartnerTempM;
          resp_2 = GLOBAL_PartnerTempP;
          break;
        #endif
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_Read_Humidity:
    case C_Read_Humidity_V:
      /**< Read humidity value */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      if (cmd == C_Read_Humidity)
      {
        resp_c = R_Read_Humidity;
      } else
      {
        resp_c = R_Read_Humidity_V;
      }
      switch (arg_2)
      {
        case 0:
          /**< Own humidity(1-100): 0 - no sensor, 255 - defect */
          resp_1 = 0;
          resp_2 = 0;
          break;
        #ifdef DEF_DUPLEX
        case 1:
          /**< Partner humidity(1-100): 0 - no sensor, 255 - defect */
          resp_1 = GLOBAL_PartnerRH;
          resp_2 = GLOBAL_PartnerRH;
          break;
        #endif
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_SET_ID:
      /**< Set new ID for RS485 connection */
      if ((arg_1 != arg_2) || (arg_1 >= RS485_BROADCAST_ID) || (broadcast == true))
      {
        break;
      }
      /**< To do? DA26-D doesn't need it, DA58-D does have it */
      //if (access_ee_bit == false)
      //  break;
      resp_c = R_SET_ID;
      resp_1 = arg_1;
      resp_2 = arg_1;
      EE_StationId = arg_1;
      EEPROM_SaveVariable((void*)&EE_StationId);
      /**< Recalculate EEPROM CRC */
      (void)EEPROM_RecalculateCRC();
      break;

    case C_ID_REPORT:
      /**< Get current ID */
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      resp_c = R_ID_REPORT;
      resp_1 = EE_StationId;
      resp_2 = resp_1;
      break;

    case C_Read_Serial_Num:
    case C_Read_Product_Descr:
    case C_Read_Firmware_Num:
    case C_Read_Hardware_Num:
    case C_Read_Serial_Num_V:
    case C_Read_Product_Descr_V:
    case C_Read_Firmware_Num_V:
    case C_Read_Hardware_Num_V:
      /**< Read serial number, byte by byte */
      //if (broadcast == true)
      //  break;
      if (arg_2 >= STRINGS_MAX_LEN)
      {
        break;
      }
      switch (cmd)
      {
        case C_Read_Serial_Num:
        case C_Read_Serial_Num_V:
          if (cmd == C_Read_Serial_Num)
          {
            resp_c = R_Read_Serial_Num;
          } else
          {
            resp_c = R_Read_Serial_Num_V;
          }
          uval8 = EEPROM_ESN_ADDRESS;
          break;
        case C_Read_Product_Descr:
        case C_Read_Product_Descr_V:
          if (cmd == C_Read_Product_Descr)
          {
            resp_c = R_Read_Product_Descr;
          } else
          {
            resp_c = R_Read_Product_Descr_V;
          }
          uval8 = EEPROM_PROD_ADDRESS;
          break;
        case C_Read_Firmware_Num:
        case C_Read_Firmware_Num_V:
          if (cmd == C_Read_Firmware_Num)
          {
            resp_c = R_Read_Firmware_Num;
          } else
          {
            resp_c = R_Read_Firmware_Num_V;
          }
          uval8 = EEPROM_FWREV_ADDRESS;
          break;
        case C_Read_Hardware_Num:
        case C_Read_Hardware_Num_V:
          if (cmd == C_Read_Hardware_Num)
          {
            resp_c = R_Read_Hardware_Num;
          } else
          {
            resp_c = R_Read_Hardware_Num_V;
          }
          uval8 = EEPROM_HWREV_ADDRESS;
          break;
        default:
          resp_c = 0;
          break;
      }
      if (resp_c != 0U)
      {
        /**< Read char from strings */
        resp_1 = EEPROM_Strings[uval8 + arg_2];
        /**< Length of serial number string */
        resp_2 = UTILS_GetLength((char*)&EEPROM_Strings[uval8]);
      }
      break;

    case C_Read_Revision_Str:
      /**< Get character from revision string */
      resp_c = R_Read_Revision_Str;
      resp_1 = arg_1;
      resp_2 = VERSION_GetRevisionStr(arg_1);
      break;

    case C_PowerUp_Counter:
    case C_PowerUp_Counter_V:
      /**< Read number of power cycles */
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      if (cmd == C_PowerUp_Counter)
      {
        resp_c = R_PowerUp_Counter;
      } else
      {
        resp_c = R_PowerUp_Counter_V;
      }
      uval32 = EEPROM_GetPowerUps();
      if (uval32 > UINT16_MAX)
      {
        uval32 = UINT16_MAX;
      }
      resp_1 = (uint8_t)(uval32 >> 8);
      resp_2 = (uint8_t)uval32;
      break;

    case C_Runtimer:
    case C_Runtimer_V:
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      if (cmd == C_Runtimer)
      {
        resp_c = R_Runtimer;
      } else
      {
        resp_c = R_Runtimer_V;
      }
      uval32 = EEPROM_GetCounter(COUNTER_LOAD_ALL);
      if (arg_2 == 1U)
      {
        /**< Read minutes, seconds */
        resp_1 = (uint8_t)((uval32 % 3600U) / 60U);
        resp_2 = (uint8_t)(uval32 % 60U);
      } else
      {
        /**< Read hours */
        uval16 = (uint16_t)(uval32 / 3600U);
        resp_1 = (uint8_t)(uval16 >> 8);
        resp_2 = (uint8_t)uval16;
      }
      break;

    case C_No_Load_Runtimer:
    case C_No_Load_Runtimer_V:
    case C_Load_Runtimer_25:
    case C_Load_Runtimer_25_V:
    case C_Load_Runtimer_50:
    case C_Load_Runtimer_50_V:
    case C_Load_Runtimer_75:
    case C_Load_Runtimer_75_V:
    case C_Load_Runtimer_100:
    case C_Load_Runtimer_100_V:
      /**< Read no-load counter */
      if (broadcast == true)
      {
        break;
      }
      switch (cmd)
      {
        case C_No_Load_Runtimer:
        case C_No_Load_Runtimer_V:
          if (cmd == C_No_Load_Runtimer)
          {
            resp_c = R_No_Load_Runtimer;
          } else
          {
            resp_c = R_No_Load_Runtimer_V;
          }
          uval8 = COUNTER_LOAD_0;
          break;
        case C_Load_Runtimer_25:
        case C_Load_Runtimer_25_V:
          if (cmd == C_Load_Runtimer_25)
          {
            resp_c = R_Load_Runtimer_25;
          } else
          {
            resp_c = R_Load_Runtimer_25_V;
          }
          uval8 = COUNTER_LOAD_25;
          break;
        case C_Load_Runtimer_50:
        case C_Load_Runtimer_50_V:
          if (cmd == C_Load_Runtimer_50)
          {
            resp_c = R_Load_Runtimer_50;
          } else
          {
            resp_c = R_Load_Runtimer_50_V;
          }
          uval8 = COUNTER_LOAD_50;
          break;
        case C_Load_Runtimer_75:
        case C_Load_Runtimer_75_V:
          if (cmd == C_Load_Runtimer_75)
          {
            resp_c = R_Load_Runtimer_75;
          } else
          {
            resp_c = R_Load_Runtimer_75_V;
          }
          uval8 = COUNTER_LOAD_75;
          break;
        case C_Load_Runtimer_100:
        case C_Load_Runtimer_100_V:
          if (cmd == C_Load_Runtimer_100)
          {
            resp_c = R_Load_Runtimer_100;
          } else
          {
            resp_c = R_Load_Runtimer_100_V;
          }
          uval8 = COUNTER_LOAD_100;
          break;
        default:
          resp_c = R_Runtimer;
          uval8 = COUNTER_LOAD_ALL;
          break;
      }
      uval32 = EEPROM_GetCounter(uval8);
      switch (arg_2)
      {
        case 0:
          /**< Read hours */
          uval16 = (uint16_t)(uval32 / 3600U);
          resp_1 = (uint8_t)(uval16 >> 8);
          resp_2 = (uint8_t)uval16;
          break;
        case 1:
          /**< Read minutes, seconds */
          resp_1 = (uint8_t)((uval32 % 3600U) / 60U);
          resp_2 = (uint8_t)(uval32 % 60U);
          break;
        case SIGN_COMM_2:
          if (arg_1 != SIGN_COMM_1)
          {
            resp_c = 0;
            break;
          }
          /**< Reset counter */
          EEPROM_ResetCounter(uval8);
          break;
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_Stall_Counter:
    case C_Stall_Counter_V:
      /**< Read stall events counter */
      if (broadcast == true)
      {
        break;
      }
      if (cmd == C_Stall_Counter)
      {
        resp_c = R_Stall_Counter;
      } else
      {
        resp_c = R_Stall_Counter_V;
      }
      uval32 = EEPROM_GetStallEvents();
      if (uval32 > UINT16_MAX)
      {
        uval32 = UINT16_MAX;
      }
      resp_1 = (uint8_t)(uval32 >> 8);
      resp_2 = (uint8_t)uval32;
      if ((arg_1 == SIGN_COMM_1) && (arg_2 == SIGN_COMM_2))
      {
        /**< Reset of stall event counter in EEPROM */
        EEPROM_ResetStallEvents();
      }
      break;

    #ifndef DEF_DUPLEX
    case C_RS_MODEreport:
    case C_RS_MODEreport_V:
      /**< Does nothing, just to be compatible */
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      if (cmd == C_RS_MODEreport)
      {
        resp_c = R_RS_MODEreport;
      } else
      {
        resp_c = R_RS_MODEreport_V;
      }
      resp_1 = 5;
      resp_2 = 5;
      break;
    #endif

    #ifndef DEF_DUPLEX
    case C_Stroke_Len_report:
    case C_Stroke_Len_report_V:
      /**< Does nothing, just to be compatible */
      if ((arg_1 != 0U) || (arg_2 != 0U))
      {
        break;
      }
      if (cmd == C_Stroke_Len_report)
      {
        resp_c = R_Stroke_Len_report;
      } else
      {
        resp_c = R_Stroke_Len_report_V;
      }
      resp_1 = 0x5A;
      resp_2 = 0x5A;
      break;
    #endif

    case C_Read_Magnet:
      resp_c = R_Read_Magnet;
      switch (arg_2)
      {
        case 0:
          /**< Read absolute magnet position */
          resp_1 = (uint8_t)(GLOBAL_MagnetValue >> 8);
          resp_2 = (uint8_t)GLOBAL_MagnetValue;
          break;
        case 1:
          /**< Read production compensated magnet position */
          resp_1 = (uint8_t)((uint16_t)GLOBAL_RealPos >> 8);
          resp_2 = (uint8_t)GLOBAL_RealPos;
          break;
        case 2:
          /**< Read BLDC hall sensor position */
          resp_1 = 0;
          resp_2 = HALL_ReadValues();
          break;
        #ifdef DEF_DUPLEX
        case 3:
          /**< Read magnet sensor status */
          resp_1 = MAGNET_GetStatus();
          resp_2 = MAGNET_GetStatus();
          break;
        case 4:
          /**< Read absolute magnet position of partner ACE */
          resp_1 = (uint8_t)(GLOBAL_PartnerMagnet >> 8);
          resp_2 = (uint8_t)GLOBAL_PartnerMagnet;
          break;
        case 5:
          /**< Read absolute magnet position of external hall sensor */
          resp_1 = (uint8_t)(GLOBAL_MagnetExtHall >> 8);
          resp_2 = (uint8_t)GLOBAL_MagnetExtHall;
          break;
        case 0x11:
          /**< Do magnet sensor calibration, experimental feature */
          if (GLOBAL_InternalID == ID_ACE1)
          {
            GLOBAL_DoSensorCalibration = SENS_CALIBR_VALUE;
            resp_1 = SIGN_COMM_1;
            resp_2 = SIGN_COMM_2;
          }
          break;
        case 0x12:
          /**< Read magnet sensor offset after calibration */
          resp_1 = EE_MagSensorOffs;
          resp_2 = EE_MagSensorOffs;
          break;
         case 0x1C:
          /**< Read magnet sensor offset after calibration */
          resp_1 = MOTOR_GetLimitPWM();
          resp_2 = MOTOR_GetShaftSpeed();
          break;
        #endif
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_Set_as_Zero:
      /**< Set current position as zero */
      if ((arg_1 != 0U) || (arg_2 != 0U) || (access_ee_bit == false))
      {
        break;
      }
      resp_c = R_Set_as_Zero;
      tmp = 0x800 - (int16_t)GLOBAL_MagnetValue;
      uval16 = (uint16_t)tmp & 0xFFFU;
      EE_ZeroPos = (int16_t)uval16;
      EEPROM_SaveVariable((void*)&EE_ZeroPos);
      break;

    case C_Read_Offset:
      /**< Read position offset */
      if (arg_1 != 0U)
      {
        break;
      }
      resp_c = R_Read_Offset;
      resp_1 = (uint8_t)((uint16_t)EE_ZeroPos >> 8);
      resp_2 = (uint8_t)EE_ZeroPos;
      break;

    case C_Write_Offset:
      /**< Write position offset */
	  if (access_ee_bit == false)
      {
        break;
      }
      resp_c = R_Write_Offset;
      uval16 = ((uint16_t)arg_1 << 8) + arg_2;
      EE_ZeroPos = uval16;
      EEPROM_SaveVariable((void*)&EE_ZeroPos);
      resp_1 = SIGN_COMM_1;
      resp_2 = SIGN_COMM_1;
      break;

    case C_Read_MP:
      /**< Read Motor Parameters (EEPROM address range 0x20...0x2F) */
      if ((arg_1 < 0x20U) || (arg_1 > 0x2FU))
      {
        break;
      }
      resp_c = R_Read_MP;
      resp_1 = EEPROM_Page[arg_1];
      resp_2 = EEPROM_Page[arg_1];
      break;

    case C_Write_MP:
      /**< Write Motor Parameters (EEPROM address range 0x20...0x2F) */
      //if (broadcast == true)
      //  break;
      if (access_ee_bit == false)
      {
        break;
      }
      if ((arg_1 < 0x20U) || (arg_1 > 0x2FU))
      {
        break;
      }
      resp_c = R_Write_MP;
      clear_accessbit = false;
      /**< Write to EEPROM page */
      resp_1 = arg_1;
      resp_2 = arg_2;
      EEPROM_WriteByte(resp_1, resp_2);
      break;

    case C_Write_Serial_Num:
    case C_Write_Prod_Descr:
    case C_Write_FWrev_num:
    case C_Write_HWrev_num:
      //if (broadcast == true)
      //  break;
      if (arg_2 >= STRINGS_MAX_LEN)
      {
        /**< command wrong (arg_1 must be 0, arg_2 = 0...0x20) */
        break;
      }
      switch (cmd)
      {
        case C_Write_Serial_Num:
          resp_c = R_Write_Serial_Num;
          uval8 = EEPROM_ESN_ADDRESS;
          break;
        case C_Write_Prod_Descr:
          resp_c = R_Write_Prod_Descr;
          uval8 = EEPROM_PROD_ADDRESS;
          break;
        case C_Write_FWrev_num:
          resp_c = R_Write_FWrev_num;
          uval8 = EEPROM_FWREV_ADDRESS;
          break;
        case C_Write_HWrev_num:
          resp_c = R_Write_HWrev_num;
          uval8 = EEPROM_HWREV_ADDRESS;
          break;
        default:
          /**< Must not occur */
          uval8 = 0;
          break;
      }
      EEPROM_Strings[uval8 + arg_2] = arg_1;
      clear_accessbit = false;
      resp_1 = arg_1;
      resp_2 = arg_2;
      break;

    case C_RD_EEPROM_Lo:
      /**< Read EEPROM value */
      resp_c = R_RD_EEPROM_Lo;
      resp_1 = EEPROM_Page[arg_2];
      resp_2 = EEPROM_Page[arg_2];
      break;

    case C_WR_EEPROM_Lo:
      /**< Write EEPROM value */
      if (access_ee_bit == false)
      {
        break;
      }
      clear_accessbit = false;
      if (arg_1 >= sizeof(uint16_t))
      {
        /**< Don't let to write the CRC manually */
        (void)EEPROM_WriteByte(arg_1, arg_2);
      }
      resp_c = R_WR_EEPROM_Lo;
      /**< Recalculate EEPROM CRC */
      (void)EEPROM_RecalculateCRC();
      /**< Just repeat command parameters */
      resp_1 = arg_1;
      resp_2 = arg_2;
      break;

    case C_RD_EEPROM_Hi:
      /**< Read EEPROM value from upper page (strings) */
      resp_c = R_RD_EEPROM_Hi;
      switch (arg_1)
      {
        case 0:
          resp_2 = EEPROM_Page[arg_2];
          resp_1 = EEPROM_Page[arg_2 + 1U];
          break;
        case 1:
          resp_2 = EEPROM_Strings[arg_2];
          resp_1 = EEPROM_Strings[arg_2 + 1U];
          break;
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_WR_EEPROM_Hi:
      /**< Write EEPROM value to upper page (strings) */
      if (access_ee_bit == false)
      {
        break;
      }
      clear_accessbit = false;
      resp_c = R_WR_EEPROM_Hi;
      EEPROM_Strings[arg_1] = arg_2;
      resp_1 = arg_1;
      resp_2 = arg_2;
      break;

    case C_SetDefault:
      /**< Reset flags to default values */
      if ((arg_1 != SIGN_DFLT_1) || (arg_2 != SIGN_DFLT_2))
      {
        break;
      }
      resp_c = R_SetDefault;
      resp_1 = SIGN_DFLT_1;
      resp_2 = SIGN_DFLT_2;
      if (GLOBAL_InternalID == ID_ACE1)
      {
        #ifdef DEF_DUPLEX
        GLOBAL_RoleForce = ROLE_FORCE_MASTER;
        #endif
      }
      LOGIC_SetDefaultValues(MOTOR_MODE_RUN1, RESET_MODE_ALL);
      break;

    case C_Set_Notch_Freq:
      /**< Set notch filter frequency, non-volatile */
      resp_c = R_Set_Notch_Freq;
      uval16 = (((uint16_t)arg_1 << 8U) | arg_2) & 0x7FFFU;
      if ((arg_1 & 0x80U) == 0U)
      {
        /**< Filter #1 */
        NOTCH_RecalcCoeff(NOTCH_NUM_1, uval16, EE_NotchWidth, EE_NotchDepth);
      } else
      {
        /**< Filter #2 */
        NOTCH_RecalcCoeff(NOTCH_NUM_2, uval16, EE_NotchWidth, EE_NotchDepth);
      }
      resp_1 = SIGN_COMM_1;
      resp_2 = SIGN_COMM_1;
      break;

    case C_Test:
      /**< Some test functions */
      if (broadcast == true)
      {
        break;
      }
      if (arg_1 != SIGN_COMM_1)
      {
        break;
      }
      resp_c = R_Test;
      switch (arg_2)
      {
        case 'R':
          /**< Reset actuator status words (both ACEs) at 0x40 ID 0xAA 0x52 */
          resp_1 = 0;
          resp_2 = 0;
          /**< Force ACE1 to be master */
          if (GLOBAL_InternalID == ID_ACE1)
          {
            #ifdef DEF_DUPLEX
            GLOBAL_RoleForce = ROLE_FORCE_MASTER;
            #endif
          }
          /**< Reset both */
          LOGIC_SetDefaultValues(MOTOR_MODE_RUN1, RESET_MODE_ALL);
          break;
        case 'S':
          /**< Reset actuator status word (only THIS ACE) at 0x40 ID 0xAA 0x53 */
          resp_1 = 0;
          resp_2 = 0;
          /**< Reset own ACE */
          LOGIC_SetDefaultValues(MOTOR_MODE_RUN1, RESET_MODE_SINGLE);
          break;
        case 0:
          /**< Check memory integrity */
          uval8 = 0;
          /**< Check for BL Flash error */
          uval32 = CRC32_Calc((uint8_t*)BL_ADDRESS, BL_SIZE - sizeof(uint32_t));
          if (uval32 != *(uint32_t*)(BL_SIZE - sizeof(uint32_t)))
          {
            uval8 |= 1U;
          }
          /**< Check for App Flash error */
          if ((GLOBAL_ExtStatus & EXTSTAT_ERROR_FLASH) > 0U)
          {
            uval8 |= 2U;
          }
          /**< Check for EEPROM error */
          if (EEPROM_CheckCRC() == false)
          {
            uval8 |= 4U;
          }
          if (GLOBAL_IsMemoryOk == false)
          {
            uval8 |= 8U;
          }
          resp_1 = uval8;
          resp_2 = uval8;
          break;
        case 1:
          /**< BLDC test */
          if (access_ee_bit == false)
          {
            resp_c = 0;
            break;
          }
          /**< Perform BLDC motor test and get results */
          MOTOR_DoTest(&resp_1, &resp_2);
          if ((resp_1 > 0U) || (resp_2 > 0U))
          {
            GLOBAL_ErrMotor = true;
          }
          break;
        #ifdef DEF_DUPLEX
        case 2:
          /**< We have to build status in old format (8bit) */
          /**< ACE1 status is always resp_2! */
          if (GLOBAL_PartnerExtStatus == STAT_NO_PARTNER)
          {
            uval8 = OLDSTS_NO_PARTNER;
          } else
          {
            uval8 = CONVERSION_BuildOldStatus(GLOBAL_PartnerExtStatus);
          }
          if (GLOBAL_InternalID == ID_ACE1)
          {
            resp_1 = uval8;
            resp_2 = CONVERSION_BuildOldStatus(GLOBAL_ExternalStatus);
          } else
          {
            resp_1 = CONVERSION_BuildOldStatus(GLOBAL_ExternalStatus);
            resp_2 = uval8;
          }
          break;
        case 3:
          /**< Readout of actuator's slave parameters */
          resp_1 = MOTOR_GetSlaveMode();
          resp_2 = GLOBAL_Power;
          break;
        #endif
        case 4:
          resp_1 = (uint8_t)(GLOBAL_ExternalStatus >> 8);
          resp_2 = (uint8_t)GLOBAL_ExternalStatus;
          break;
        case 5:
          resp_1 = (uint8_t)((uint16_t)GLOBAL_TargetPos >> 8);
          resp_2 = (uint8_t)GLOBAL_TargetPos;
          break;
        case 6:
          resp_1 = (uint8_t)(GLOBAL_ExtStatus >> 8);
          resp_2 = (uint8_t)GLOBAL_ExtStatus;
          break;
        case 9:
          /**< read internal actuator Status (unlateched) */
          resp_1 = (uint8_t)(GLOBAL_MyStatus >> 8);
          resp_2 = (uint8_t)GLOBAL_MyStatus;
          break;
        default:
          resp_c = 0;
          break;
      }
      break;

    case C_SimulateACEfailure:
      /**< Simulating ACE Failures */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      if (arg_2 == 1U)
      {
        /**< BLDC free */
        GLOBAL_MotorExtMode = false;
        MOTOR_SetMode(MOTOR_MODE_FREE);
        #ifdef DEF_DUPLEX
        GLOBAL_ForceSlaveMode = SLAVE_MODE_FREE;
        #endif
      }
      if (arg_2 == 2U)
      {
        /**< BLDC brake */
        MOTOR_SetMode(MOTOR_MODE_BRAKE);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_BRAKE;
//        #endif
      }
      clear_accessbit = false;
      resp_c = R_SimulateACEfailure;
      resp_1 = arg_1;
      resp_2 = arg_2;
      break;

    case C_Motor_Power:
      /**< Processing motor actions */
      uval16 = (uint16_t)arg_1 << 8;
      uval16 += arg_2;
      if (uval16 == 0U)
      {
        /**< Motor terminals open (free rotation) */
        GLOBAL_MotorExtMode = false;
        MOTOR_SetMode(MOTOR_MODE_FREE);
        #ifdef DEF_DUPLEX
        GLOBAL_ForceSlaveMode = SLAVE_MODE_FREE;
        #endif
		    TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);   /**< reset comm. timeout MJ 04/2023 */
      } else
      if (uval16 == 0x8000U)
      {
        /**< Rotation with power from EEPROM */
        GLOBAL_MotorExtMode = false;
        MOTOR_SetMode(MOTOR_MODE_RUN1);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
      } else
      if (uval16 == 0x8001U)
      {
        /**< Motor terminals close to GND (brake) */
        MOTOR_SetMode(MOTOR_MODE_BRAKE);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_BRAKE;
//        #endif
		    TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);   /**< reset comm. timeout MJ 04/2023 */
      } else
      if ((uval16 & 0xFF00U) == 0xFD00U)
      {
        /**< Permanent rotation (FD=Forward Direction) with arg_2 motor power value */
        GLOBAL_MotorExtMode = false;
        MOTOR_SetExtPower((uint8_t)uval16);
        MOTOR_SetMode(MOTOR_MODE_GOCW);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
      } else
      if ((uval16 & 0xFF00U) == 0xBD00U)
      {
        /**< Permanent rotation (BD=Backward Direction) with arg_2 motor power value */
        GLOBAL_MotorExtMode = false;
        MOTOR_SetExtPower((uint8_t)uval16);
        MOTOR_SetMode(MOTOR_MODE_GOCCW);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
      } else
      if (uval16 == 0x8002U)
      {
        /**< Get motor mode */
        switch (MOTOR_GetMode())
        {
          case MOTOR_MODE_RUN1:
            uval16 = 0x8000U;
            break;
          case MOTOR_MODE_FREE:
            uval16 = 0;
            break;
          case MOTOR_MODE_BRAKE:
            uval16 = 0x8001U;
            break;
          case MOTOR_MODE_GOCW:
            uval16 = 0xFD00U + MOTOR_GetExtPower();
            break;
          case MOTOR_MODE_GOCCW:
            uval16 = 0xBD00U + MOTOR_GetExtPower();
            break;
          default:
            /**< Unknown motor mode, ignore */
            break;
        }
      } else
      if (uval16 <= 0xFFU)
      {
        /**< Max PWM value defined externally */
        GLOBAL_MotorExtMode = true;
        MOTOR_SetExtPower((uint8_t)uval16);
        MOTOR_SetMode(MOTOR_MODE_RUN1);
//        #ifdef DEF_DUPLEX
//        GLOBAL_ForceSlaveMode = SLAVE_MODE_RUN1;
//        #endif
      } else
      {
        resp_c = 0;
        break;
      }
      clear_accessbit = false;
      resp_c = R_Motor_Power;
      resp_1 = (uint8_t)(uval16 >> 8);
      resp_2 = (uint8_t)uval16;
      break;

    #ifdef DEF_DUPLEX
    case C_Slave_Free:
      /**< Set slave as free-running */
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 == SIGN_ACCESS_1) && (arg_2 < 2U))
      {
        /**< Access protection */
        MOTOR_DoSlaveFree(arg_2 == 1U);
        resp_c = R_Slave_Free;
        resp_1 = SIGN_COMM_1;
        resp_2 = arg_2;
      }
      break;
    #endif

    case C_Set_AccessBit:
      /**< Set access bit for writing to EEPROM */
      //if (broadcast == true)
      //  break;
      if ((arg_1 == SIGN_ACCESS_1) && (arg_2 == SIGN_ACCESS_2))
      {
        /**< Access to EEPROM is enable */
        access_ee_bit = true;
        /**< Don't clear access bit */
        clear_accessbit = false;
        resp_c = C_Set_AccessBit;  // response code
        resp_1 = 0x41;             // "Access"
        resp_2 = 0x45;             // "Enable"
        break;
      }
      break;

    case C_Read_PWM:
      /**< Get motor parameters (power, direction) */
      if ((arg_1 != 0U) || (broadcast == true))
      {
        break;
      }
      resp_c = R_Read_PWM;
      /**< Current motor direction */
      resp_1 = MOTOR_GetDir();
      /**< BLDC motor PWM value */
      resp_2 = GLOBAL_Power;
      break;

    case C_StartBootLoader:
      /**< Start bootloader */
      if (broadcast == true)
      {
        break;
      }
      if ((arg_1 != BL_SIGN1) || (arg_2 != BL_SIGN2))
      {
        break;
      }
      /**< Initialize internal bus reset */
      #ifdef DEF_DUPLEX
      GLOBAL_DoStartBootloader = BL_START_SIGN;
      #else
      GLOBAL_DoStartBootloader = BL_STOP_SIGN;
      #endif
      resp_c = R_StartBootLoader;
      resp_1 = SIGN_COMM_1;
      resp_2 = SIGN_COMM_1;
      break;

    case C_Misc:
      /**< A lot of different functions */
      if (broadcast == true)
      {
        break;
      }
      resp_c = R_Misc;
      switch (arg_1)
      {
        case 0:
          /**< Get CRC of the bootloader */
          uval32 = CRC32_Calc((uint8_t*)BL_ADDRESS, BL_SIZE - sizeof(uint32_t));
          /**< We have CRC32 but transmit only lower 16 bits */
          resp_1 = (uint8_t)(uval32 >> 8);
          resp_2 = (uint8_t)uval32;
          break;
        case 1:
          /**< Get CRC of the firmware */
          uval32 = (uint32_t)((uintptr_t)&crc32_build) - APP_ADDRESS;
          uval32 = CRC32_Calc((uint8_t*)APP_ADDRESS, uval32);
          /**< We have CRC32 but transmit only lower 16 bits */
          resp_1 = (uint8_t)(uval32 >> 8);
          resp_2 = (uint8_t)uval32;
          break;
        case 2:
          /**< Get CRC of EEPROM, don't need to recalculate! */
          uval16 = EEPROM_CalculateCRC();
          resp_1 = (uint8_t)(uval16 >> 8);
          resp_2 = (uint8_t)uval16;
          break;
        case 3:
          /**< Recalculate CRC of EEPROM */
          uval16 = EEPROM_RecalculateCRC();
          resp_1 = (uint8_t)(uval16 >> 8);
          resp_2 = (uint8_t)uval16;
          break;
        case 0x10:
          /**< Read stored CRC of the bootloader */
          uval32 = *(uint32_t*)(BL_SIZE - sizeof(uint32_t));
          /**< We have CRC32 but transmit only 16 bits */
          resp_1 = (uint8_t)(uval32 >> 8);
          resp_2 = (uint8_t)uval32;
          break;
        case 0x11:
          /**< Read stored CRC of the firmware */
          resp_1 = (uint8_t)(crc32_build >> 8);
          resp_2 = (uint8_t)crc32_build;
          break;
        case 0x12:
          /**< Read CRC of EEPROM */
          uval16 = EEPROM_GetCRC();
          resp_1 = (uint8_t)(uval16 >> 8);
          resp_2 = (uint8_t)uval16;
          break;
        case 0x20:
          /**< Get default role (ACE1 or ACE2) */
          resp_1 = GLOBAL_InternalID;
          resp_2 = GLOBAL_InternalID;
          break;
        case 0x80:
          /**< Get PWM values from master and slave */
          resp_1 = GLOBAL_Power;
          #ifdef DEF_DUPLEX
            resp_2 = GLOBAL_PartnerPower;
          #else
            resp_2 = 0;
          #endif
          break;
        case 0x81:
          /**< Perform BLDC test */
          MOTOR_DoTest(&resp_1, &resp_2);
          if ((resp_1 > 0U) || (resp_2 > 0U))
          {
            GLOBAL_ErrMotor = true;
          }
          break;
        case 0x83:
          /**< Check RAM, works only once at startup! */
          /**< 0 is Ok */
          if (GLOBAL_IsMemoryOk == true)
          {
            resp_1 = 0;
          } else
          {
            resp_1 = 1;
          }
          resp_2 = resp_1;
          break;
        case 0x88:
          /**< Internal network and external timeout flags reset */
          TIMEOUTS_Reset(TIMEOUT_TYPE_ALL);
          resp_1 = 0;
          resp_2 = 0;
          break;
        case 0xAB:
          /**< !Reset all counters! */
          if (arg_2 != SIGN_COMM_2)
          {
            resp_c = 0;
            break;
          }
          for (uval8 = COUNTER_LOAD_ALL; uval8 < COUNTER_LOAD_LAST; uval8++)
          {
            EEPROM_ResetCounter(uval8);
          }
          EEPROM_ResetStallEvents();
          EEPROM_ResetPowerUps();
          resp_1 = 0U;
          resp_2 = 0U;
          break;

        default:
          resp_c = 0U;
          break;
      }
      break;

    default:
      resp_c = 0U;
      break;
  }

  /**< Reset access bit */
  if (clear_accessbit == true)
  {
    access_ee_bit = false;
  }

  /**< Build reply structure */
  output->cmd = resp_c;
  output->id = EE_StationId;
  output->arg_1 = resp_1;
  output->arg_2 = resp_2;
}
