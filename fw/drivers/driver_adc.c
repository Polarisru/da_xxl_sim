#include "driver_adc.h"

/** \brief Setup ADC module
 *
 * \param [in] ref Reference source
 * \param [in] resolution ADC resolution
 * \return void
 *
 */
void ADC_Init(uint8_t ref, uint8_t resolution)
{
  /**< Configure clock and power */
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_ADC0;
  GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);
  /**< Reset ADC registers */
  ADC0->CTRLA.bit.SWRST = 1;
  /**< Set reference */
  ADC0->REFCTRL.bit.REFSEL = ref;
  /**< Set freerun mode */
  ADC0->CTRLC.bit.FREERUN = 1;
  /**< Set resolution */
  ADC0->CTRLC.bit.RESSEL = resolution;
  /**< Set GND as negative ADC pin */
  ADC0->INPUTCTRL.bit.MUXNEG = 0x18;
  /**< Adjusting result */
  ADC0->AVGCTRL.reg = ADC_AVGCTRL_ADJRES(4) | ADC_AVGCTRL_SAMPLENUM_32;
  /**< Set prescaler value */
  ADC0->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
}

/** \brief Set active ADC channel
 *
 * \param [in] channel Number of the channel
 * \return void
 *
 */
void ADC_SetChannel(uint8_t channel)
{
  /**< Disable ADC */
  ADC0->CTRLA.bit.ENABLE = 0;
  while ((ADC0->SYNCBUSY.reg & (ADC_SYNCBUSY_SWRST | ADC_SYNCBUSY_ENABLE)) > 0U) {}
  /**< Set channel */
  ADC0->INPUTCTRL.bit.MUXPOS = channel;
  /**< Enable ADC */
  ADC0->CTRLA.bit.ENABLE = 1;
  while ((ADC0->SYNCBUSY.reg & (ADC_SYNCBUSY_SWRST | ADC_SYNCBUSY_ENABLE)) > 0U) {}
  /**< Reset ready flag */
  ADC0->INTFLAG.reg |= ADC_INTFLAG_RESRDY;
  /**< Start conversion */
  ADC0->SWTRIG.bit.START = 1;
}

/** \brief Get result status
 *
 * \return True if ADC result is ready
 *
 */
bool ADC_IsReady(void)
{
  return (ADC0->INTFLAG.bit.RESRDY == 1);
}

/** \brief Get conversion result
 *
 * \return Result as uint16_t
 *
 */
uint16_t ADC_GetResult(void)
{
  return ADC0->RESULT.reg;
}
