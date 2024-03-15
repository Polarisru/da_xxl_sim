#include "defines.h"
#include "drivers.h"
#include "canbus.h"
#include "cmd_set.h"
#include "crc16.h"
#include "eeprom.h"
#include "global.h"
#include "rs485.h"

/**
 * \brief CAN interrupt handler
 */
void CAN1_IRQ_Handler(void)
{
  uint32_t ir;
  //uint8_t rec_id;
  BaseType_t xHigherPriorityTaskWoken;

  ir = CAN1_NUM->IR.reg;

  /**< We have not woken a task at the start of the ISR */
  xHigherPriorityTaskWoken = pdFALSE;

  if ((ir & CAN_IR_RF0N) > 0U)
  {
    struct can_message msg;
    uint8_t data[64];
    msg.data = data;
    TCanMsg CanMsg;
    (void)CAN_ReadMsg(CAN1_NUM, &msg);
    /**< Check for our ID or broadcast */
    //rec_id = (msg.id & CANBUS_MASK_ID) >> CANBUS_OFFS_ID;
    //if ((rec_id == CANBUS_BROADCAST) || (rec_id == EE_CanId))
    {
      CanMsg.id = msg.id;
      CanMsg.len = msg.len;
      CanMsg.fmt = msg.fmt;
      CanMsg.channel = CAN_CHANNEL_1;
      (void) memcpy(CanMsg.data, msg.data, msg.len);
      xQueueSendFromISR(xQueueCAN, &CanMsg, &xHigherPriorityTaskWoken);
    }
  }

  CAN1_NUM->IR.reg = ir;
  /**< Now the buffer is empty we can switch context if necessary */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * \brief CAN interrupt handler
 */
#ifdef DEF_DUPLEX_COMM
void CAN2_IRQ_Handler(void)
{
  uint32_t ir;
  //uint8_t rec_id;
  BaseType_t xHigherPriorityTaskWoken;

  ir = CAN2_NUM->IR.reg;

  /**< We have not woken a task at the start of the ISR */
  xHigherPriorityTaskWoken = pdFALSE;

  if ((ir & CAN_IR_RF0N) > 0U)
  {
    struct can_message msg;
    uint8_t data[64];
    msg.data = data;
    TCanMsg CanMsg;
    (void)CAN_ReadMsg(CAN2_NUM, &msg);
    /**< Check for our ID or broadcast */
    //rec_id = (msg.id & CANBUS_MASK_ID) >> CANBUS_OFFS_ID;
    //if ((rec_id == CANBUS_BROADCAST) || (rec_id == EE_CanId))
    {
      CanMsg.id = msg.id;
      CanMsg.len = msg.len;
      CanMsg.fmt = msg.fmt;
      CanMsg.channel = CAN_CHANNEL_2;
      (void) memcpy(CanMsg.data, msg.data, msg.len);
      xQueueSendFromISR(xQueueCAN, &CanMsg, &xHigherPriorityTaskWoken);
    }
  }

  CAN2_NUM->IR.reg = ir;
  /**< Now the buffer is empty we can switch context if necessary */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

/** \brief Send data packet via CAN bus
 *
 * \param [in] msg Pointer to message structure
 * \return True if succeed
 *
 */
bool CANBUS_Send(enum can_channel channel, struct can_message *msg)
{
  switch (channel)
  {
    case CAN_CHANNEL_1:
      return (CAN_WriteMsg(CAN1_NUM, msg) == ERR_NONE);
    #ifdef DEF_DUPLEX_COMM
    case CAN_CHANNEL_2:
      return (CAN_WriteMsg(CAN2_NUM, msg) == ERR_NONE);
    #endif
    default:
      return true;
  }
}

/** \brief Initialize CAN communication module
 *
 * \param [in] bitrate Bitrate value to set
 * \return void
 *
 */
void CANBUS_Configuration(uint8_t bitrate)
{
  CAN_Init(CAN1_NUM);

  /**< Enable CAN interrupt */
  NVIC_DisableIRQ(CAN1_IRQn);
  NVIC_ClearPendingIRQ(CAN1_IRQn);
  NVIC_EnableIRQ(CAN1_IRQn);
  CAN1_NUM->ILE.reg = CAN_ILE_EINT0;

  if (bitrate > CAN_BITRATE_1M)
  {
    bitrate = CAN_BITRATE_500K;
  }

  /**< Configure CAN pins */
  /**< There is only one available pin configuration for this MCU */
  GPIO_SetFunction(CAN1_PORT, CAN1_PIN_TX, CAN1_MUX_TX);
  GPIO_SetFunction(CAN1_PORT, CAN1_PIN_RX, CAN1_MUX_RX);

  CAN_Enable(CAN1_NUM);
  (void)CAN_SetBitrateRaw(CAN1_NUM, bitrate, bitrate);

  #ifdef DEF_DUPLEX_COMM
  CAN_Init(CAN2_NUM);

  /**< Configure CAN pins */
  /**< There is only one available pin configuration for this MCU */
  GPIO_SetFunction(CAN2_PORT, CAN2_PIN_TX, CAN2_MUX_TX);
  GPIO_SetFunction(CAN2_PORT, CAN2_PIN_RX, CAN2_MUX_RX);

  /**< Enable CAN and set selected bitrate */
  CAN_Enable(CAN2_NUM);
  (void)CAN_SetBitrateRaw(CAN2_NUM, bitrate, bitrate);

  /**< Enable CAN interrupt */
  NVIC_DisableIRQ(CAN2_IRQn);
  NVIC_ClearPendingIRQ(CAN2_IRQn);
  NVIC_EnableIRQ(CAN2_IRQn);
  CAN2_NUM->ILE.reg = CAN_ILE_EINT0;
  #endif
}

/** \brief Disable CAN module
 *
 * \return void
 *
 */
void CANBUS_Disable(void)
{
  CAN_Disable(CAN1_NUM);
  #ifdef DEF_DUPLEX_COMM
  CAN_Disable(CAN2_NUM);
  #endif
//  switch (channel)
//  {
//    case CAN_CHANNEL_1:
//      CAN_Disable(CAN1_NUM);
//      break;
//    #ifdef DEF_DUPLEX_COMM
//    case CAN_CHANNEL_2:
//      CAN_Disable(CAN2_NUM);
//      break;
//    #endif
//    default:
//      break;
//  }
}

/** \brief Enable CAN module
 *
 * \return void
 *
 */
void CANBUS_Enable(void)
{
  CAN_Enable(CAN1_NUM);
  #ifdef DEF_DUPLEX_COMM
  CAN_Enable(CAN2_NUM);
  #endif
}
