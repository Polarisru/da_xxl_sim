#include "defines.h"
#include "drivers.h"
#include "crc16.h"
#include "rs485.h"
#include "global.h"
#include "cmd_set.h"

static const uint32_t RS485_BAUDRATES[RS485_BAUDRATE_LAST] = {
  9600U,
  19200U,
  38400U,
  57600U,
  115200U,
  230400U,
  250000U
};

uint8_t RS485_RxBuffer[RS485_RX_PACKET_LEN] = {0};
uint8_t RS485_TxBuffer[RS485_TX_PACKET_LEN] = {0};

#ifndef DEF_DUPLEX
uint8_t RS485_2_RxBuffer[RS485_RX_PACKET_LEN] = {0};
uint8_t RS485_2_TxBuffer[RS485_TX_PACKET_LEN] = {0};
#endif

/** \brief SERCOM interrupt handler to process RS485 messages
 *
 * \return void
 *
 */
void RS485_IRQ_Handler(void)
{
  uint8_t ch;
  uint8_t i;
  uint16_t crc;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t rx_buf[RS485_RX_PACKET_LEN];

  if (((RS485_CHANNEL->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) >> SERCOM_USART_INTFLAG_RXC_Pos) > 0U)
  {
    if ((RS485_CHANNEL->USART.STATUS.reg & (SERCOM_USART_STATUS_PERR | SERCOM_USART_STATUS_FERR |
                                     SERCOM_USART_STATUS_BUFOVF | SERCOM_USART_STATUS_ISF | SERCOM_USART_STATUS_COLL)) > 0U)
    {
      RS485_CHANNEL->USART.STATUS.reg = SERCOM_USART_STATUS_MASK;
      return;
    }
    ch = RS485_CHANNEL->USART.DATA.reg;
    /**< shift receiving buffer */
    for (i = 0; i < (RS485_RX_PACKET_LEN - 1U); i++)
    {
      rx_buf[i] = rx_buf[i + 1U];
    }
    rx_buf[RS485_RX_PACKET_LEN - 1U] = ch;
    if ((rx_buf[RS485_PACKET_ID_POS] == EE_StationId) || (rx_buf[RS485_PACKET_ID_POS] == RS485_BROADCAST_ID))
    {
      crc = CRC16_INIT_VAL;
      crc = CRC16_Calc(crc, rx_buf, RS485_RX_PACKET_CRC_POS);
      i = RS485_RX_PACKET_CRC_POS;
      if (((uint8_t)(crc >> 8) == rx_buf[i]) && ((uint8_t)crc == rx_buf[i + 1U]))
      {
        /**< CRC is Ok, place packet to RS485 processing queue */
        (void)memcpy(RS485_RxBuffer, rx_buf, RS485_RX_PACKET_LEN);
        xTaskNotifyFromISR(xTaskRS485, 0x01, eSetBits, &xHigherPriorityTaskWoken);
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

/** \brief SERCOM interrupt handler for 2nd RS485 channel
 *
 * \return void
 *
 */
#ifndef DEF_DUPLEX
void RS485_2_IRQ_Handler(void)
{
  uint8_t ch;
  uint8_t i;
  uint16_t crc;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  static uint8_t rx_buf[RS485_RX_PACKET_LEN];

  if (((RS485_2_CHANNEL->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC) >> SERCOM_USART_INTFLAG_RXC_Pos) > 0U)
  {
    if ((RS485_2_CHANNEL->USART.STATUS.reg & (SERCOM_USART_STATUS_PERR | SERCOM_USART_STATUS_FERR |
        SERCOM_USART_STATUS_BUFOVF | SERCOM_USART_STATUS_ISF | SERCOM_USART_STATUS_COLL)) > 0U)
    {
      RS485_2_CHANNEL->USART.STATUS.reg = SERCOM_USART_STATUS_MASK;
      return;
    }
    ch = RS485_2_CHANNEL->USART.DATA.reg;
    /**< shift receiving buffer */
    for (i = 0; i < (RS485_RX_PACKET_LEN - 1U); i++)
    {
      rx_buf[i] = rx_buf[i + 1U];
    }
    rx_buf[RS485_RX_PACKET_LEN - 1U] = ch;
    if ((rx_buf[RS485_PACKET_ID_POS] == EE_StationId) || (rx_buf[RS485_PACKET_ID_POS] == RS485_BROADCAST_ID))
    {
      crc = CRC16_Calc(CRC16_INIT_VAL, rx_buf, RS485_RX_PACKET_CRC_POS);
      i = RS485_RX_PACKET_CRC_POS;
      if (((uint8_t)(crc >> 8) == rx_buf[i]) && ((uint8_t)crc == rx_buf[i + 1U]))
      {
        /**< CRC is Ok, place packet to RS485 processing queue */
        (void)memcpy(RS485_2_RxBuffer, rx_buf, RS485_RX_PACKET_LEN);
        xTaskNotifyFromISR(xTaskRS485, 0x02, eSetBits, &xHigherPriorityTaskWoken);
      }
    }
  }

  /**< Now the buffer is empty we can switch context if necessary */
  if (xHigherPriorityTaskWoken != pdFALSE)
  {
    /**< Actual macro used here is port specific */
    portYIELD_FROM_ISR(1);
  }
}
#endif

/** \brief Initialize RS485 module
 *
 * \return void
 *
 */
void RS485_Configuration(uint8_t baudrate)
{
  TDmaSettings DmaSett;

  if (baudrate >= RS485_BAUDRATE_LAST)
  {
    baudrate = RS485_BAUDRATE_115200;
  }

  /**< configure DIR pin and TE pin */
  GPIO_ClearPin(RS485_PORT, RS485_PIN_DIR);
  GPIO_ClearPin(RS485_PORT, RS485_PIN_TE);
  GPIO_SetDir(RS485_PORT, RS485_PIN_DIR, true);
  GPIO_SetDir(RS485_PORT, RS485_PIN_TE, true);
  GPIO_SetFunction(RS485_PORT, RS485_PIN_DIR, RS485_MUX_DIR);
  GPIO_SetFunction(RS485_TX_PORT, RS485_PIN_TX, RS485_MUX_TX);
  GPIO_SetFunction(RS485_PORT, RS485_PIN_RX, RS485_MUX_RX);
  GPIO_SetFunction(RS485_PORT, RS485_PIN_TE, GPIO_PIN_FUNC_OFF);
  /**< Configure UART */
  UART_Init(RS485_CHANNEL, RS485_RXPO, RS485_TXPO, RS485_BAUDRATES[baudrate], UART_STOP_1BIT, UART_PARITY_NONE);

  /**< Enable receiving interrupt */
  RS485_CHANNEL->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
  NVIC_EnableIRQ(RS485_IRQn);
  /**< Setup DMA channel for transmission */
  DmaSett.beat_size = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
  DmaSett.trig_src = RS485_DMA_TRIG;
  DmaSett.dst_addr = (void*)(uintptr_t)&RS485_CHANNEL->USART.DATA.reg;
  DmaSett.src_addr = (void*)RS485_TxBuffer;
  DmaSett.src_inc = true;
  DmaSett.dst_inc = false;
  DmaSett.len = RS485_TX_PACKET_LEN;
  DmaSett.priority = 3;
  DMA_SetupChannel(DMA_CHANNEL_RS485, &DmaSett);

#ifndef DEF_DUPLEX
    /**< configure DIR pin and TE pin */
    GPIO_ClearPin(RS485_2_PORT, RS485_2_PIN_DIR);
    GPIO_SetDir(RS485_2_PORT, RS485_2_PIN_DIR, true);
    GPIO_SetFunction(RS485_2_PORT, RS485_2_PIN_DIR, RS485_2_MUX_DIR);
    GPIO_SetFunction(RS485_2_TX_PORT, RS485_2_PIN_TX, RS485_2_MUX_TX);
    GPIO_SetFunction(RS485_2_PORT, RS485_2_PIN_RX, RS485_2_MUX_RX);
    /**< Configure UART */
    UART_Init(RS485_2_CHANNEL, RS485_2_RXPO, RS485_2_TXPO, RS485_BAUDRATES[baudrate], UART_STOP_1BIT, UART_PARITY_NONE);

    RS485_2_CHANNEL->USART.INTENSET.reg = SERCOM_USART_INTFLAG_RXC;
    NVIC_EnableIRQ(RS485_2_IRQn);
    /**< Setup DMA channel for transmission */
    DmaSett.beat_size = DMAC_BTCTRL_BEATSIZE_BYTE_Val;
    DmaSett.trig_src = RS485_2_DMA_TRIG;
    DmaSett.dst_addr = (void*)(uintptr_t)&RS485_2_CHANNEL->USART.DATA.reg;
    DmaSett.src_addr = (void*)RS485_2_TxBuffer;
    DmaSett.src_inc = true;
    DmaSett.dst_inc = false;
    DmaSett.len = RS485_TX_PACKET_LEN;
    DmaSett.priority = 3;
    DMA_SetupChannel(DMA_CHANNEL_RS485_2, &DmaSett);
#endif
}

/** \brief Switch RS485 internal termination resistor
 *
 * \param [in] on True if enable, otherwise disable
 * \return void
 *
 */
void RS485_SwitchTermination(bool on)
{
  if (on)
  {
    GPIO_SetPin(RS485_PORT, RS485_PIN_TE);
  } else
  {
    GPIO_ClearPin(RS485_PORT, RS485_PIN_TE);
  }
}

/** \brief Send data packet via RS485 bus
 *
 * \param [in] data Buffer with data to send
 * \param [in] len Length of data packet
 * \return void
 *
 */
void RS485_SendPacket(const uint8_t *data)
{
  //while (DMA_IsReady(DMA_CHANNEL_RS485) == false)
  //  vTaskDelay(pdMS_TO_TICKS(1));
  taskENTER_CRITICAL();
  #ifndef DEF_DUPLEX
    if (data == RS485_TxBuffer)
    {
      DMA_StartChannel(DMA_CHANNEL_RS485);
    } else
    {
      DMA_StartChannel(DMA_CHANNEL_RS485_2);
    }
  #else
    (void) data;
    DMA_StartChannel(DMA_CHANNEL_RS485);
  #endif
  taskEXIT_CRITICAL();
}

/** \brief Receive RS485 packet, wait for reply
 *
 * \param [out] data Buffer to receive data
 * \param [in] timeout Timeout for waiting
 * \return true if packet was received
 *
 */
bool RS485_ReceivePacket(uint8_t *data, uint16_t timeout)
{
  /**< Clear task notifications first to remove received packets */
  xTaskNotifyStateClear(xTaskRS485);
  /**< Wait for notification from the RS485 Rx interrupt */
  if (ulTaskNotifyTake(pdTRUE, timeout) == 0U)
  {
    return false;
  }
  (void)memcpy(data, RS485_RxBuffer, RS485_RX_PACKET_CRC_POS);
  return true;
}

/** \brief Disable RS485
 *
 * \return void
 *
 */
void RS485_Disable(void)
{
  GPIO_SetDir(RS485_PORT, RS485_PIN_DIR, true);
  GPIO_SetPin(RS485_PORT, RS485_PIN_DIR);
  GPIO_SetFunction(RS485_PORT, RS485_PIN_DIR, GPIO_PIN_FUNC_OFF);
}
