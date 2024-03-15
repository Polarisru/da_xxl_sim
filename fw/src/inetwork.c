#include "drivers.h"
#include "crc16.h"
#include "eeprom.h"
#include "global.h"
#include "inetwork.h"
#include "logic.h"
#include "magnet.h"
#include "motor.h"
#include "monitor.h"
#include "power.h"
#include "timeouts.h"

#ifndef DEF_UNITTEST
  static uint8_t INETWORK_RxBuffer[INET_PACKET_LEN];
  static uint8_t INETWORK_TxBuffer[INET_PACKET_LEN];
#endif
static bool INETWORK_Active;
static bool INETWORK_ForceRoleBit;

/** \brief SERCOM interrupt handler to process internal network messages
 *
 * \return void
 *
 */
#ifndef DEF_UNITTEST
#ifdef DEF_DUPLEX
void INETWORK_IRQ_Handler(void)
{
  uint8_t ch;
  uint8_t i;
  uint16_t crc;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t rx_buf[INET_PACKET_LEN];

  if (((INETWORK_CHANNEL->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) >> SERCOM_USART_INTFLAG_RXC_Pos) > 0U)
  {
    if ((INETWORK_CHANNEL->USART.STATUS.reg & (SERCOM_USART_STATUS_PERR | SERCOM_USART_STATUS_FERR |
                                     SERCOM_USART_STATUS_BUFOVF | SERCOM_USART_STATUS_ISF | SERCOM_USART_STATUS_COLL)) > 0U)
    {
      INETWORK_CHANNEL->USART.STATUS.reg = SERCOM_USART_STATUS_MASK;
      return;
    }
    ch = INETWORK_CHANNEL->USART.DATA.reg;
    /**< Shift receiving buffer */
    for (i = 0U; i < (INET_PACKET_LEN - 1U); i++)
    {
      rx_buf[i] = rx_buf[i + 1U];
    }
    rx_buf[INET_PACKET_LEN - 1U] = ch;
    if (rx_buf[0] == INET_PACKET_SIGN)
    {
      /**< Packet starting signature detected */
      crc = CRC16_CalcCCITT(rx_buf, INET_PACKET_CRC_POS);
      if (((uint8_t)(crc >> 8U) == rx_buf[INET_PACKET_CRC_POS]) && ((uint8_t)crc == rx_buf[INET_PACKET_CRC_POS + 1U]))
      {
        (void) memcpy(INETWORK_RxBuffer, rx_buf, INET_PACKET_LEN);
        /**< CRC is Ok, give notification to processing task */
        vTaskNotifyGiveFromISR(xTaskInet, &xHigherPriorityTaskWoken);
      }
    }
  }

  /**< Now the buffer is empty we can switch context if necessary */
  if (xHigherPriorityTaskWoken != pdFALSE)
  {
    /**< Actual macro used here is port specific */
    portYIELD_FROM_ISR(1U);
  }
}
#endif
#endif

/** \brief Initialize internal network module
 *
 * \return void
 *
 */
#ifndef DEF_UNITTEST
static void INETWORK_Configuration(void)
{
  TDmaSettings DmaSett;

  /**< configure DIR pin and TE pin */
  GPIO_ClearPin(INETWORK_PORT, INETWORK_PIN_DIR);
  GPIO_SetDir(INETWORK_PORT, INETWORK_PIN_DIR, true);
  GPIO_SetFunction(INETWORK_PORT, INETWORK_PIN_DIR, INETWORK_MUX_DIR);
  GPIO_SetFunction(INETWORK_PORT, INETWORK_PIN_TX, INETWORK_MUX_TX);
  GPIO_SetFunction(INETWORK_PORT, INETWORK_PIN_RX, INETWORK_MUX_RX);
  /**< Configure UART */
  UART_Init(INETWORK_CHANNEL, INETWORK_RXPO, INETWORK_TXPO, INETWORK_BAUDRATE, UART_STOP_2BITS, UART_PARITY_NONE);

  /**< enable receiving interrupt */
  INETWORK_CHANNEL->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
  NVIC_EnableIRQ(INETWORK_IRQn);

  /**< Setup DMA channel for transmission */
  DmaSett.beat_size = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
  DmaSett.trig_src = INETWORK_DMA_TRIG;
  DmaSett.dst_addr = (void*)(uintptr_t)&INETWORK_CHANNEL->USART.DATA.reg;
  DmaSett.src_addr = (void*)INETWORK_TxBuffer;
  DmaSett.src_inc = true;
  DmaSett.dst_inc = false;
  DmaSett.len = INET_PACKET_LEN;
  DmaSett.priority = 2;
  DMA_SetupChannel(DMA_CHANNEL_INETWORK, &DmaSett);

  INETWORK_Active = true;
  INETWORK_ForceRoleBit = false;
}
#endif

/** \brief Send data packet to internal bus
 *
 * \param [in] data Buffer with data to send
 * \param [in] len Length of data packet
 * \return void
 *
 */
#ifndef DEF_UNITTEST
static void INETWORK_SendPacket(void)
{
  taskENTER_CRITICAL();
  DMA_StartChannel(DMA_CHANNEL_INETWORK);
  taskEXIT_CRITICAL();
}
#endif

/** \brief Parse received inetwork packet
 *
 * \param [in] data Pointer to buffer with data to parse
 * \return void
 *
 */
void INETWORK_ParsePacket(uint8_t *data)
{
  uint8_t i;
  uint16_t uval16;
  uint8_t rec_id = data[INET_PACKET_ID_POS] & MASK_TN;

  if (rec_id == GLOBAL_MyTN)
  {
    /**< Doesn't process own packets */
    return;
  }

  switch (rec_id)
  {
    case ACE1_TN:
    case ACE2_TN:
      /**< Packet format: 7..6: ID, 5: FORCE_ROLE, 4: MASTER_BIT, 3..0: freshness counter */
      /**< Reset to zero as a signal "internal network works" */
      TIMEOUTS_Reset(TIMEOUT_TYPE_PARTNER);
      if ((data[INET_PACKET_ID_POS] & 0x10U) > 0U)
      {
        /**< Role of partner's ACE: master */
        GLOBAL_PartnerIsMaster = true;
      } else
      {
        /**< Role of partner's ACE: slave */
        GLOBAL_PartnerIsMaster = false;
      }
      if ((data[INET_PACKET_ID_POS] & 0x20U) > 0U)
      {
        /**< Set role forcing request */
        if (GLOBAL_RoleForce == ROLE_FORCE_NONE)
        {
          GLOBAL_RoleForce = ROLE_FORCE_OPPOSITE;
        }
      }
      /**< Byte format: 7: MOT_DIR, 5..4 - motor mode, 3..0: high nibble of magnet sensor */
      /**< Get motor direction */
      if ((data[INET_PACKET_ID_POS + 1U] & 0x80U) > 0U)
      {
        GLOBAL_PartnerMotorDir = MOTOR_DIR_CW;
      } else
      {
        GLOBAL_PartnerMotorDir = MOTOR_DIR_CCW;
      }
      /**< Get motor mode request from master, bits 5..4 */
      if (GLOBAL_IsMaster == false)
      {
        MOTOR_SetSlaveMode((data[INET_PACKET_ID_POS + 1U] >> 4U) & 0x03U);
      }
      /**< Magnet sensor value from partner */
      GLOBAL_PartnerMagnet = ((data[INET_PACKET_ID_POS + 1U] & 0x0FU) << 8U) + data[INET_PACKET_ID_POS + 2U];
      /**< Power for slave ACE */
      GLOBAL_PartnerPower = data[INET_PACKET_ID_POS + 3U];
      /**< Internal network command parser by freshness number. Parses received commands */
      i = data[INET_PACKET_ID_POS] & 0x0FU;
      /**< Data receiving by freshness number */
      switch (i)
      {
        case ICMD_N:
          if ((data[INET_PACKET_ID_POS + 4U] == INET_BE_DFLT) && (data[INET_PACKET_ID_POS + 5U] == (INET_BE_DFLT ^ 0xFFU)))
          {
            LOGIC_SetDefaultValues(MOTOR_MODE_RUN1, RESET_MODE_SINGLE);
          }
          break;
        case ICMD_S:
          /**< Internal partner ACE status */
          GLOBAL_PartnerStatus = ((uint16_t)data[INET_PACKET_ID_POS + 4U] << 8) + data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_A:
          /**< External partner ACE status */
          GLOBAL_PartnerExtStatus = ((uint16_t)data[INET_PACKET_ID_POS + 4U] << 8) + data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_T:
          /**< Temperature of Motor Winding [°C] */
          GLOBAL_PartnerTempM = data[INET_PACKET_ID_POS + 4U];
          /**< Temperature of PCB [°C] */
          GLOBAL_PartnerTempP = data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_U:
          /**< Voltage of primary power supply [x200mV] */
          GLOBAL_PartnerU1 = data[INET_PACKET_ID_POS + 4U];
          /**< Voltage of secondary power supply [x200mV] */
          GLOBAL_PartnerU2 = data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_P:
          /**< Target position */
          uval16 = (uint16_t)data[INET_PACKET_ID_POS + 4U] << 8;
          uval16 += data[INET_PACKET_ID_POS + 5U];
          GLOBAL_PartnerTargetPos = (int16_t)uval16;
          break;
        case ICMD_PWR:
          /**< Maximal PWM value */
          GLOBAL_PartnerMaxPwmValue = data[INET_PACKET_ID_POS + 4U];
          break;
        case ICMD_I:
          /**< Supply current */
          GLOBAL_PartnerI = data[INET_PACKET_ID_POS + 4U];
          /**< Relative humidity */
          GLOBAL_PartnerRH = data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_D:
          /**< SID from partner ACE */
          if (data[INET_PACKET_ID_POS + 4U] == data[INET_PACKET_ID_POS + 5U])
          {
            GLOBAL_PartnerId = data[INET_PACKET_ID_POS + 4U];
          }
          break;
        case ICMD_CRC:
          /**< CRC16 of the EEPROM */
          GLOBAL_PartnerCRC = data[INET_PACKET_ID_POS + 4U] << 8;
          GLOBAL_PartnerCRC += data[INET_PACKET_ID_POS + 5U];
          break;
        case ICMD_C:
          /**< Calibrate sensor offset, beta! */
          if (GLOBAL_InternalID == ID_ACE2)
          {
            if ((data[INET_PACKET_ID_POS + 4U] == 0xA5U) && (data[INET_PACKET_ID_POS + 5U] == 0x5AU))
            {
              /**< Perform offset calibration */
              GLOBAL_DoSensorCalibration = 1;
            } else
            {
              GLOBAL_DoSensorCalibration = 0;
            }
          }
          break;
        case ICMD_V1:
        case ICMD_V2:
          /**< Target position from partner ACE */
          break;
        case ICMD_RST:
          /**< Raw reset request 1/2 from partner ACE */
          break;
        case ICMD_BL:
          /**< Go to bootloader */
          if ((data[INET_PACKET_ID_POS + 4U] == BL_SIGN1) && (data[INET_PACKET_ID_POS + 5U] == BL_SIGN2))
          {
            #ifndef DEF_UNITTEST
            POWER_RestartApp();
            #endif
          }
          break;
        default:
          /**< Unknown command code, ignore */
          break;
      }
      break;
    case HALL_TN:
      if ((data[INET_PACKET_ID_POS + 1U] == data[INET_PACKET_ID_POS + 3U]) &&
          (data[INET_PACKET_ID_POS + 2U] == data[INET_PACKET_ID_POS + 4U]) &&
          (data[INET_PACKET_ID_POS + 5U] == 0xE4U))
      {
        /**< Right packet received, process */
        if (!(data[INET_PACKET_ID_POS + 1U] & 0x80U))
        {
          /**< Data is trustworthy */
          TIMEOUTS_Reset(TIMEOUT_TYPE_HALL);
          GLOBAL_ExtHallStatus = 1;
          data[INET_PACKET_ID_POS + 1U] &= 0x0FU;
          /**< 12bit value from external Hall sensor */
          GLOBAL_MagnetExtHall = (data[INET_PACKET_ID_POS + 1U] << 8U) + data[INET_PACKET_ID_POS + 2U];
        }
      }
      break;
    default:
      /**< Unknown ID, ignore */
      break;
  }
}

/** \brief Build reply packet
 *
 * \param [in] id Token ID number
 * \param [out] data Pointer to data buffer for output
 * \return void
 *
 */
void INETWORK_BuildPacket(uint8_t id, uint8_t *data)
{
  uint16_t crc;
  uint16_t uval16;

  data[INET_PACKET_SIGN_POS] = INET_PACKET_SIGN;
  /**< Packet format: 7..6: ID, 5: ROLE_FORCE, 4: MASTER_BIT, 3..0: token counter */
  id &= 0x0FU;
  /**< Bits 7..6: Put token ID */
  id |= GLOBAL_MyTN;
  /**< Bit 5: force partner to get opposite role (0-no, 1-yes) */
  if (INETWORK_ForceRoleBit == true)
  {
    id |= 0x20U;
  }
  /**< Bit 4: Role bit of ACE (slave/master) */
  if (GLOBAL_IsMaster == true)
  {
    id |= 0x10U;
  }
  data[INET_PACKET_ID_POS] = id;
  /**< Byte format: 7: motor direction, 5..4: motor mode, 3..0: high nibble of magnet sensor */
  data[INET_PACKET_ID_POS + 1U] = 0;
  /**< Bit 7: Motor direction bit */
  if (GLOBAL_SlaveMotorDir == MOTOR_DIR_CW)
  {
    data[INET_PACKET_ID_POS + 1U] |= 0x80U;
  }
  /**< Bit 6: reserved */
  /**< Bits 5..4: Send motor mode request from host */
  data[INET_PACKET_ID_POS + 1U] |= ((GLOBAL_ForceSlaveMode & 0x03U) << 4U);
  /**< Bits 3..0: Hall sensor bits 11..8 */
  data[INET_PACKET_ID_POS + 1U] = data[INET_PACKET_ID_POS + 1U] | (uint8_t)(GLOBAL_MagnetValue >> 8U);
  /**< Hall sensor bits 7..0 */
  data[INET_PACKET_ID_POS + 2U] = (uint8_t)GLOBAL_MagnetValue;
  /**< Power for slave motor */
  data[INET_PACKET_ID_POS + 3U] = GLOBAL_SlavePower;

  /**< Send data to partner ACE by freshness number */
  switch (id & 0x0FU)
  {
    case ICMD_N:
      /**< Sending my own status to partner */
      data[INET_PACKET_ID_POS + 4U] = GLOBAL_DoDefault;
      data[INET_PACKET_ID_POS + 5U] = ~GLOBAL_DoDefault;
      /**< Will be transmitted once */
      GLOBAL_DoDefault = 0;
      break;
    case ICMD_S:
      /**< Sending my own status to partner */
      data[INET_PACKET_ID_POS + 4U] = (uint8_t)(GLOBAL_MyStatus >> 8U);
      data[INET_PACKET_ID_POS + 5U] = (uint8_t)GLOBAL_MyStatus;
      break;
    case ICMD_A:
      /**< Sending my external status to partner */
      data[INET_PACKET_ID_POS + 4U] = (uint8_t)(GLOBAL_ExternalStatus >> 8U);
      data[INET_PACKET_ID_POS + 5U] = (uint8_t)GLOBAL_ExternalStatus;
      break;
    case ICMD_T:
      /**< Temperature of Motor Winding [°C] */
      data[INET_PACKET_ID_POS + 4U] = MONITOR_GetTemperature(MONITOR_TEMP_MOTOR);
      /**< Temperature of PCB [°C] */
      data[INET_PACKET_ID_POS + 5U] = MONITOR_GetTemperature(MONITOR_TEMP_PCB);
      break;
    case ICMD_U:
      /**< Voltage of primary power supply [x200mV] */
      data[INET_PACKET_ID_POS + 4U] = MONITOR_GetVoltage(MONITOR_U_CH1);
      /**< Voltage of secondary power supply [x200mV] */
      data[INET_PACKET_ID_POS + 5U] = MONITOR_GetVoltage(MONITOR_U_CH2);
      break;
    case ICMD_P:
      /**< Target position */
      uval16 = (uint16_t)GLOBAL_TargetPos;
      data[INET_PACKET_ID_POS + 4U] = (uint8_t)(uval16 >> 8U);
      data[INET_PACKET_ID_POS + 5U] = (uint8_t)uval16;
      break;
    case ICMD_PWR:
      /**< Maximal PWM value */
      data[INET_PACKET_ID_POS + 4U] = MOTOR_GetMaxPwmValue();
      data[INET_PACKET_ID_POS + 5U] = MOTOR_GetMaxPwmValue();
      break;
    case ICMD_I:
      /**< Supply current sensor value */
      data[INET_PACKET_ID_POS + 4U] = MONITOR_GetCurrent();
      /**< Relative humidity sensor value */
      data[INET_PACKET_ID_POS + 5U] = 0;
      break;
    case ICMD_D:
      /**< SID value */
      data[INET_PACKET_ID_POS + 4U] = EE_StationId;
      data[INET_PACKET_ID_POS + 5U] = EE_StationId;
      break;
    case ICMD_CRC:
      /**< CRC16 of the EEPROM */
      crc = EEPROM_GetCRC();
      data[INET_PACKET_ID_POS + 4U] = (uint8_t)(crc >> 8U);
      data[INET_PACKET_ID_POS + 5U] = (uint8_t)crc;
      break;
    case ICMD_C:
      /**< Sensor offset function, beta! */
      if ((GLOBAL_DoSensorCalibration > 0U) && (GLOBAL_InternalID == ID_ACE1))
      {
        /**< Magnet sensor offset calibration */
        data[INET_PACKET_ID_POS + 4U] = 0xA5U;
        data[INET_PACKET_ID_POS + 5U] = 0x5AU;
        GLOBAL_DoSensorCalibration--;
      } else
      {
        data[INET_PACKET_ID_POS + 4U] = 0;
        data[INET_PACKET_ID_POS + 5U] = 0;
      }
      break;
    case ICMD_V1:
    case ICMD_V2:
      /**< Target position 1/2 from partner ACE */
      data[INET_PACKET_ID_POS + 4U] = 0;
      data[INET_PACKET_ID_POS + 5U] = 0;
      break;
    case ICMD_RST:
      /**< Raw reset request 1/2 from partner ACE */
      data[INET_PACKET_ID_POS + 4U] = 0;
      data[INET_PACKET_ID_POS + 5U] = 0;
      break;
    case ICMD_BL:
      /**< Go to bootloader */
      if ((GLOBAL_DoStartBootloader == BL_START_SIGN) && (GLOBAL_InternalID == ID_ACE1))
      {
        data[INET_PACKET_ID_POS + 4U] = BL_SIGN1;
        data[INET_PACKET_ID_POS + 5U] = BL_SIGN2;
        GLOBAL_DoStartBootloader = BL_STOP_SIGN;
      }
      break;
    default:
      /**< Unknown command code, ignore */
      break;
  }
}

/**< Task to process internal network messages */
#ifndef DEF_UNITTEST
void INETWORK_Task(void *pParameters)
{
  (void) pParameters;
  uint16_t crc;
  /**< output of USART1 for toking ring. [0]Sign [1]TTMSFFFF, [2][3]Hall, [4]Motor power, [5][6]versatile, [7][8]CRC */
  uint8_t tokenN = 0;
  bool doSend;
  uint8_t recTN;
  uint32_t timeout;

  INETWORK_Configuration();

  if (GLOBAL_MyTN == ACE1_TN)
  {
    timeout = INET_TIMEOUT_1;
  } else
  {
    timeout = INET_TIMEOUT_2;
  }

  while (1)
  {
    doSend = false;
    /**< Wait for notification from the RX interrupt */
    if (ulTaskNotifyTake(pdTRUE, timeout) == 0U)
    {
      doSend = true;
    }

    recTN = INETWORK_RxBuffer[INET_PACKET_ID_POS] & MASK_TN;

    if (doSend == false)
    {
      /**< Process internal network messages */
      INETWORK_ParsePacket(INETWORK_RxBuffer);
    }

    if (recTN == GLOBAL_WaitTN)
    {
      doSend = true;
    }

    if (doSend == true)
    {
      /**< Send reply */
      INETWORK_BuildPacket(tokenN, INETWORK_TxBuffer);

      crc = CRC16_CalcCCITT(INETWORK_TxBuffer, INET_PACKET_CRC_POS);
      INETWORK_TxBuffer[INET_PACKET_CRC_POS] = (uint8_t)(crc >> 8U);
      INETWORK_TxBuffer[INET_PACKET_CRC_POS + 1U] = (uint8_t)crc;

      /**< Send data to internal network bus */
      if (INETWORK_Active == true)
      {
        INETWORK_SendPacket();
      }

      tokenN++;
    }
  }
}
#endif

/** \brief Activate sending packets on internal network
 *
 * \param [in] on Send packets if True
 * \return void
 *
 */
void INETWORK_Activate(bool on)
{
  INETWORK_Active = on;
}

/** \brief Switch role forcing on/off
 *
 * \param [in] on Switch forcing on if true
 * \return void
 *
 */
void INETWORK_ForceRole(bool on)
{
  INETWORK_ForceRoleBit = on;
}
