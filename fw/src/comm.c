#include "defines.h"
#include "drivers.h"
#include "canbus.h"
#include "can_volz.h"
#include "comm.h"
#include "crc16.h"
#include "eeprom.h"
#include "global.h"
#include "logic.h"
#include "parser_can.h"
#include "parser_rs485.h"
#include "rs485.h"
#include "timeouts.h"

/**< The queue is to be created to hold a maximum of 10 CAN packets */
#define CAN_QUEUE_LENGTH    10
#define CAN_ITEM_SIZE       sizeof(TCanMsg)

/**< Task to send periodic CAN messages */
void COMM_CAN_Periodic_Task(void *pParameters)
{
  (void) pParameters;
  uint32_t ticks;

  while (1)
  {
    //vTaskDelay(5);
    ticks = xTaskGetTickCount();
    (void) ticks;
  }
}

/**< Task to process CAN messages */
void COMM_CAN_Task(void *pParameters)
{
  (void) pParameters;
  TCanMsg  rx_msg;
  uint8_t tx_data[CANBUS_BUF_LEN];
  struct can_message tx_msg;
  struct can_filter filter;

  /**< Initialite CAN bus */
  CANBUS_Configuration(EE_CanBitrate);
  /**< Setup CAN filters for Volz 11-bit protocol */
  EE_CanId &= CANBUS_ID_OWN_MASK;
  if (EE_DevId == 0xFFU)
  {
    EE_DevId = 0;
  }
  EE_DevId &= CANMSG_ID_MASK;
  /**< Add backdoor CAN filter (own protocol) */
  filter.id   = CANBUS_BACKDOOR_ID;
  filter.mask = CANBUS_ID_MASK;
  (void)CAN_SetFilter(CAN_FMT_STDID, &filter);
  /**< Add main CAN filter (own protocol) */
  filter.id   = EE_CanId | (uint16_t)EE_DevId;
  filter.mask = CANBUS_ID_MASK;
  (void)CAN_SetFilter(CAN_FMT_STDID, &filter);
  if (EE_DevId != 0U)
  {
    /**< Support broadcating ID */
    filter.id   = EE_CanId;
    filter.mask = CANBUS_ID_MASK;
    (void)CAN_SetFilter(CAN_FMT_STDID, &filter);
  }

  /**< Create a queue capable of containing CAN packets */
  xQueueCAN = xQueueCreate(CAN_QUEUE_LENGTH, CAN_ITEM_SIZE);

  /**< Prepare message structure for replies */
  tx_msg.data = tx_data;

  while (1)
  {
    /**< Wait for CAN message */
    if (xQueueReceive(xQueueCAN, &rx_msg, 50) == pdTRUE)
    {
      /**< Reset connection timeout counter */
      TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
      /**< Parsing command and send reply if needed */
      if (PARSER_CanVolz(&rx_msg, &tx_msg) == true)
      {
        (void)CANBUS_Send(rx_msg.channel, &tx_msg);
      }
    } else
    {
      rx_msg.id = 0;
    }
  }
}

/**< Task to process RS485 messages */
void COMM_RS485_Task(void *pParameters)
{
  (void) pParameters;
  uint8_t  *rx_buffer = RS485_RxBuffer;
  uint8_t  *tx_buffer = RS485_TxBuffer;
  uint16_t crc;
  uint32_t value;
  TCommData input;
  TCommData output;

  RS485_Configuration(EE_Baudrate);

  if ((EE_Options2 & OPTIONS2_BIT_TE) > 0U)
  {
    RS485_SwitchTermination(true);
  } else
  {
    RS485_SwitchTermination(false);
  }

  while (1)
  {
    /**< Wait for notification from RS485 interrupt */
    //if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0)
    if (xTaskNotifyWait(pdFALSE, ULONG_MAX, &value, portMAX_DELAY) > 0)
    {
      /**< Reset connection timeout counter */
      //LOGIC_ResetTimeout(TIMEOUT_TYPE_HOST);

      if ((value & 0x01U) > 0U)
      {
        rx_buffer = RS485_RxBuffer;
        tx_buffer = RS485_TxBuffer;
      } else
      #ifdef DEF_DUPLEX_COMM
      if ((value & 0x02U) > 0U)
      {
        rx_buffer = RS485_2_RxBuffer;
        tx_buffer = RS485_2_TxBuffer;
      } else
      #endif
      {
        continue;
      }

      /**< Build structure for parser */
      input.cmd = rx_buffer[RS485_PACKET_CMD_POS];
      input.id = rx_buffer[RS485_PACKET_ID_POS];
      input.arg_1 = rx_buffer[RS485_PACKET_ARG1_POS];
      input.arg_2 = rx_buffer[RS485_PACKET_ARG2_POS];

      /**< Parse received command */
      PARSER_ProcessRS485(&input, &output);

      /**< Send reply */
      if (output.cmd != 0U)
      {
        tx_buffer[RS485_PACKET_CMD_POS] = output.cmd;
        tx_buffer[RS485_PACKET_ID_POS] = output.id;
        tx_buffer[RS485_PACKET_ARG1_POS] = output.arg_1;
        tx_buffer[RS485_PACKET_ARG2_POS] = output.arg_2;

        /**< Calculate checksum */
        crc = CRC16_INIT_VAL;
        crc = CRC16_Calc(crc, tx_buffer, RS485_TX_PACKET_CRC_POS);
        tx_buffer[RS485_TX_PACKET_CRC_POS] = (uint8_t)(crc >> 8);
        tx_buffer[RS485_TX_PACKET_CRC_POS + 1U] = (uint8_t)crc;
        RS485_SendPacket(tx_buffer);
      }
    }
  }
}
