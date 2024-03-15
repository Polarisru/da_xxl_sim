#include "driver_ac.h"

/** \brief Initialize analog comparator
 *
 * \param [in] channel Number of the channel
 * \param [in] flen
 * \param [in] muxpos Positive input selection
 * \param [in] muxneg Negative input selection
 * \param [in] scaler Scaling factor for VDD
 * \param [in] enInt Enable interrupt if true
 * \return void
 *
 */
void AC_Init(uint8_t channel, uint8_t flen, uint8_t muxpos, uint8_t muxneg, uint8_t scaler, bool enInt)
{
  uint32_t temp = 0;

  if (channel > AC_MAX_CHANNEL)
  {
    return;
  }
  if (scaler > AC_MAX_SCALER)
  {
    scaler = AC_MAX_SCALER;
  }
  /**< Configure clock and power */
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_AC;
  /**<
   * The Analog Comparators and ADC1 use the same generic clock configuration.
   * GCLK_ADC1 must be used to configure the clock for AC as GCLK_AC is not
   * functional. Errata reference: 13404
   */
  GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
  GCLK->PCHCTRL[AC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  /**< Reset AC registers */
  AC->CTRLA.reg = AC_CTRLA_SWRST;

  /**< Setup AC */
  temp = AC_COMPCTRL_FLEN(flen) | AC_COMPCTRL_MUXPOS(muxpos) | AC_COMPCTRL_MUXNEG(muxneg) |
         AC_COMPCTRL_INTSEL(AC_COMPCTRL_INTSEL_FALLING_Val) | AC_COMPCTRL_ENABLE;
  AC->COMPCTRL[channel].reg = temp;
  while (AC->SYNCBUSY.reg > 0U) {}
  AC->SCALER[channel].reg = scaler;
  while (AC->SYNCBUSY.reg > 0U) {}
  AC->CTRLA.reg |= AC_CTRLA_ENABLE;
  while ((AC->SYNCBUSY.reg & AC_SYNCBUSY_ENABLE) > 0U) {}
  /**< Enable AC compare interrupt for selected channel */
  if (enInt == true)
  {
    switch (channel)
    {
      case 0:
        AC->INTENSET.bit.COMP0 = 1U;
        break;
      case 1:
        AC->INTENSET.bit.COMP1 = 1U;
        break;
      case 2:
        AC->INTENSET.bit.COMP2 = 1U;
        break;
      case 3:
        AC->INTENSET.bit.COMP3 = 1U;
        break;
      default:
        /**< Channel doesn't exist */
        break;
    }
  }
}
