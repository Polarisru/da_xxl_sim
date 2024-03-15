#include "driver_i2c.h"

#define I2C_GCLK_FREQ    CONF_CPU_FREQUENCY
#define I2C_GCLK_SRC     GCLK_PCHCTRL_GEN_GCLK0

/** \brief Initialize I2C connection
 *
 * \param [in] channel SERCOM number for I2C bus
 * \param [in] baudrate Bus baudrate
 * \return
 *
 */
void I2C_Init(Sercom *channel, uint32_t baudrate)
{
  uint8_t val = (uint8_t)(CONF_CPU_FREQUENCY / baudrate / 2);

  /**< Configure clock and power */
  if (channel == SERCOM0)
  {
    GCLK->PCHCTRL[SERCOM0_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM0_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM0;
  } else
  if (channel == SERCOM1)
  {
    GCLK->PCHCTRL[SERCOM1_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM1_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM1;
  } else
  if (channel == SERCOM2)
  {
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM2_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM2;
  } else
  if (channel == SERCOM3)
  {
    GCLK->PCHCTRL[SERCOM3_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM3_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM3;
  } else
  #ifdef SERCOM4
  if (channel == SERCOM4)
  {
    GCLK->PCHCTRL[SERCOM4_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM4_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM4;
  } else
  #endif
  #ifdef SERCOM5
  if (channel == SERCOM5)
  {
    GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].reg = I2C_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
    GCLK->PCHCTRL[SERCOM5_GCLK_ID_SLOW].reg = GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
    MCLK->APBCMASK.reg |= MCLK_APBCMASK_SERCOM5;
  } else
  #endif
  {
    return;
  }

  channel->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_SPEED(0) | SERCOM_I2CM_CTRLA_SDAHOLD(0x2) |
                            SERCOM_I2CM_CTRLA_RUNSTDBY | SERCOM_I2CM_CTRLA_MODE(0x5);
  /**< Smart mode enabled by setting the bit SMEN as 1 */
  channel->I2CM.CTRLB.reg = SERCOM_I2CM_CTRLB_SMEN;
  /**< Wait for synchronization busy */
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
  /**< BAUDLOW is non-zero, and baud register is loaded */
  channel->I2CM.BAUD.reg = SERCOM_I2CM_BAUD_BAUD(val) | SERCOM_I2CM_BAUD_BAUDLOW(val);
  /**< Wait for synchronization busy */
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
  /**< SERCOM peripheral enabled by setting the ENABLE bit as 1 */
  channel->I2CM.CTRLA.reg |= SERCOM_I2CM_CTRLA_ENABLE;
  /**< SERCOM Enable synchronization busy */
  while ((channel->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE) > 0U) {}
  /**< Bus state is forced into idle state */
  channel->I2CM.STATUS.bit.BUSSTATE = 0x1;
  /**< Wait for synchronization busy */
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
}

/** \brief Reset bus after bus errors
 *
 * \param [in] channel SERCOM number
 * \return void
 *
 */
static void I2C_ResetBus(Sercom *channel)
{
  /**< Reset I2C bus to get it work with problematic devices */
  channel->I2CM.CTRLA.bit.ENABLE = 0;
  /**< SERCOM Enable synchronization busy */
  while ((channel->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE) > 0U) {}
  channel->I2CM.CTRLA.bit.ENABLE = 1;
  /**< SERCOM Enable synchronization busy */
  while ((channel->I2CM.SYNCBUSY.reg & SERCOM_I2CM_SYNCBUSY_ENABLE) > 0U) {}
  /**< Bus state is forced into idle state */
  channel->I2CM.STATUS.bit.BUSSTATE = 0x1;
  /**< Wait for synchronization busy */
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
}

/** \brief Wait for acknowledge from slave
 *
 * \param [in] channel SERCOM number
 * \return True if no timeout occurred
 *
 */
static bool I2C_WaitAck(Sercom *channel)
{
  uint16_t timeout = I2C_ACK_TIMEOUT;

  /**< Wait till MB flag set */
  while ((channel->I2CM.INTFLAG.bit.MB == 0U) && (timeout > 0U)) {
    timeout--;
  }
  /**< Clearing the MB interrupt */
  channel->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_MB;

  return (timeout > 0U);
}

/** \brief Wait for transmissions from slave
 *
 * \param [in] channel SERCOM number
 * \return True if no timeout occurred
 *
 */
static bool I2C_WaitSlave(Sercom *channel)
{
  uint16_t timeout = I2C_ACK_TIMEOUT;

  /**< Wait till MB flag set */
  while ((channel->I2CM.INTFLAG.bit.SB == 0U) && (timeout > 0U)) {
    timeout--;
  }
  /**< Clearing the MB interrupt */
  channel->I2CM.INTFLAG.reg |= SERCOM_I2CM_INTFLAG_SB;

  return (timeout > 0U);
}

/** \brief Write data to I2C register
 *
 * \param [in] channel SERCOM number
 * \param [in] addr Address of the I2C device
 * \param [in] reg Register number
 * \param [in] tx_data Pointer to data to transmit
 * \param [in] tx_len Data length
 * \return True if transmission was successfully
 *
 */
bool I2C_WriteReg(Sercom *channel, uint8_t addr, uint8_t reg, uint8_t *tx_data, uint8_t tx_len)
{
  I2C_ResetBus(channel);
  /**< Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
  channel->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
  /**< Slave address with Write(0) */
  channel->I2CM.ADDR.reg = (addr << 1U) | 0U;
  (void)I2C_WaitAck(channel);
  channel->I2CM.DATA.reg = reg;
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
  (void)I2C_WaitAck(channel);
  while (tx_len-- > 0U)
  {
    channel->I2CM.DATA.reg = *tx_data;
    tx_data++;
    while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
    (void)I2C_WaitAck(channel);
  }
  /**< After transferring the last byte stop condition will be sent */
  channel->I2CM.CTRLB.bit.CMD = 0x3;
  while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}

  return true;
}

/** \brief Read data from I2C register
 *
 * \param [in] channel SERCOM number
 * \param [in] addr Address of the I2C device
 * \param [in] reg Register number
 * \param [in] rx_data Pointer to data to receive
 * \param [in] rx_len Data length
 * \return True if transmission was successfully
 *
 */
bool I2C_ReadReg(Sercom *channel, uint8_t addr, uint8_t reg, uint8_t *rx_data, uint8_t rx_len)
{
  uint8_t i = I2C_REPEATS;
  uint8_t pos;

  while (i > 0U)
  {
    I2C_ResetBus(channel);
    i--;
    /**< Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
    channel->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
    while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
    /**< Slave address with Write(0) */
    channel->I2CM.ADDR.reg = (addr << 1U) | 0U;
    if (!I2C_WaitAck(channel))
    {
      continue;
    }
    channel->I2CM.DATA.reg = reg;
    while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
    if (!I2C_WaitAck(channel))
    {
      continue;
    }
    /**< After transferring the last byte stop condition will be sent */
    channel->I2CM.CTRLB.bit.CMD = 0x3;
    while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}

    /**< Acknowledge section is set as ACK signal by writing 0 in ACKACT bit */
    channel->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
    while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
    /**< Slave address with read (1) */
    channel->I2CM.ADDR.reg = (addr << 1U) | 1U;
    //I2C_WaitAck(channel);

    pos = 0;

    while (pos < rx_len)
    {
      if (!I2C_WaitSlave(channel))
      {
        break;
      }
      if (pos == (rx_len - 1U))
      {
        /**< NACK should be sent before reading the last byte */
        channel->I2CM.CTRLB.reg |= SERCOM_I2CM_CTRLB_ACKACT;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
        channel->I2CM.CTRLB.bit.CMD = 0x3;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
        rx_data[pos] = channel->I2CM.DATA.reg;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
      } else
      {
        channel->I2CM.CTRLB.reg &= ~SERCOM_I2CM_CTRLB_ACKACT;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
        rx_data[pos] = channel->I2CM.DATA.reg;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
        /**< Sending ACK after reading each byte */
        channel->I2CM.CTRLB.bit.CMD = 0x2;
        while (channel->I2CM.SYNCBUSY.bit.SYSOP > 0U) {}
      }
      pos++;
    }

    if (pos == rx_len)
    {
      break;
    }
  }

  return (i > 0U);
}
