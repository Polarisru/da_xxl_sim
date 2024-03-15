#include "driver_uart.h"
#include "driver_clk.h"

/**< Normal clock for UART is external resonator */
#define UART_GCLK_FREQ    CONF_XOSC_FREQUENCY
#define UART_GCLK_SRC     GCLK_PCHCTRL_GEN_GCLK2

/*
   RXPO setting (RxD pin)
 RXPO[1:0]   Name Description
-------------------------------
|   0x0   |       PAD[0]      |
|   0x1   |       PAD[1]      |
|   0x2   |       PAD[2]      |
|   0x3   |       PAD[3]      |
-------------------------------

   TXPO setting (TxD pin)
 TXPO[1:0]   Name Description
-------------------------------
|   0x0   |       PAD[0]      |
|   0x1   |       PAD[2]      |
|   0x2   |       PAD[0]      |
|   0x3   |       PAD[0]      |
-------------------------------
*/

/** \brief Initialize UART module
 *
 * \param [in] channel Number of SERCOM channel
 * \param [in] pinRXPO Number of RXPO configuration (RX pin)
 * \param [in] pinTXPO Number of TXPO configuration (TX pin)
 * \param [in] baud Baudrate settings
 * \param [in] stop Stop bit settings (1/2)
 * \param [in] parity Parity bit settings (none/even/odd)
 * \return void
 *
 */
void UART_Init(Sercom *channel, uint8_t pinRXPO, uint8_t pinTXPO, uint32_t baud,
               enum uart_stop_mode stop, enum uart_parity_mode parity)
{
  uint64_t br = (uint64_t)65536UL * (UART_GCLK_FREQ - (uint64_t)baud * 16UL) / UART_GCLK_FREQ;

  if (channel == SERCOM0)
  {
    GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM0_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0;
  } else
  if (channel == SERCOM1)
  {
    GCLK->PCHCTRL[SERCOM1_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM1_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM1;
  } else
  if (channel == SERCOM2)
  {
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM2;
  } else
  if (channel == SERCOM3)
  {
    GCLK->PCHCTRL[SERCOM3_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM3_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM3;
  } else
  #ifdef SERCOM4
  if (channel == SERCOM4)
  {
    GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM4_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM4;
  } else
  #endif
  #ifdef SERCOM5
  if (channel == SERCOM5)
  {
    GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].reg = UART_GCLK_SRC | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM5;
  } else
  #endif
  {
    /**< channel doesn't exist */
    return;
  }

  channel->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1/*USART_INT_CLK*/) |
      SERCOM_USART_CTRLA_RXPO(pinRXPO) | SERCOM_USART_CTRLA_TXPO(pinTXPO);

  if (parity != UART_PARITY_NONE)
  {
    channel->USART.CTRLA.bit.FORM = 1;
  }

  channel->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN | SERCOM_USART_CTRLB_TXEN |
      SERCOM_USART_CTRLB_CHSIZE(0/*8 bits*/);

  if (parity == UART_PARITY_ODD)
  {
    channel->USART.CTRLB.bit.PMODE = 1;
  }

  if (stop == UART_STOP_2BITS)
  {
    channel->USART.CTRLB.bit.SBMODE = 1;
  }

  channel->USART.BAUD.reg = (uint16_t)br;

  /**< Add guarding time settings for RS485 mode */
  //channel->USART.CTRLC.reg = SERCOM_USART_CTRLC_GTIME(3);

  channel->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

/** \brief Deinitialize UART module
 *
 * \param [in] channel Number of SERCOM channel
 * \return void
 *
 */
void UART_DeInit(Sercom *channel)
{
  channel->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
}

/** \brief Set new baudrate for selected channel
 *
 * \param [in] channel Number of SERCOM channel
 * \param [in] baud Baudrate settings
 * \return void
 *
 */
void UART_SetBaudrate(Sercom *channel, uint32_t baud)
{
  uint64_t br = (uint64_t)65536UL * (UART_GCLK_FREQ - (uint64_t)baud * 16UL) / UART_GCLK_FREQ;

  channel->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
  channel->USART.BAUD.reg = (uint16_t)br;
  channel->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

/** \brief Send one byte to UART interface
 *
 * \param [in] channel Number of SERCOM channel
 * \param [in] data Byte to transmit
 * \return void
 *
 */
void UART_SendByte(Sercom *channel, uint8_t data)
{
  channel->USART.DATA.reg = data;
  while (channel->USART.INTFLAG.bit.TXC == 0) {}
}

/** \brief Check if any data is present in serial input buffer
 *
 * \param [in] channel Number of SERCOM channel
 * \return True if receiving buffer has data
 *
 */
bool UART_HaveData(Sercom *channel)
{
  return (channel->USART.INTFLAG.bit.RXC == 1);
}

/** \brief Receive one byte from serial interface
 *
 * \param [in] channel Number of SERCOM channel
 * \return Received byte as uint8_t
 *
 */
uint8_t UART_GetByte(Sercom *channel)
{
  /**< Wait until the data is ready to be received */
  while (channel->USART.INTFLAG.bit.RXC == 0) {}
  /**< Read RX data, combine with mask */
  return (uint8_t)(channel->USART.DATA.reg & 0xff);
}
