#include "driver_dac.h"

#define CONF_DAC_REFSEL   1     // <0x01=> AVCC
#define CONF_DAC_EOEN     1     // External output is enabled

/** \brief Initialize DAC module
 *
 * \return void
 *
 */
void DAC_Init(void)
{
  /**< Setup DAC clock */
  MCLK->APBCMASK.reg |= MCLK_APBCMASK_DAC;
  GCLK->PCHCTRL[DAC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);

  /**< Setup DAC registers */
  DAC->EVCTRL.reg = (0 << DAC_EVCTRL_EMPTYEO_Pos) | (0 << DAC_EVCTRL_STARTEI_Pos);
  DAC->CTRLB.reg = (CONF_DAC_REFSEL << DAC_CTRLB_REFSEL_Pos) | (0 << DAC_CTRLB_DITHER_Pos) |
                   (0 << DAC_CTRLB_VPD_Pos) | (0 << DAC_CTRLB_LEFTADJ_Pos) |
                   (0 << DAC_CTRLB_IOEN_Pos) | (CONF_DAC_EOEN << DAC_CTRLB_EOEN_Pos);
  DAC->CTRLA.reg = (0 << DAC_CTRLA_RUNSTDBY_Pos);
  DAC->DBGCTRL.reg = (1 << DAC_DBGCTRL_DBGRUN_Pos);
}

/** \brief Enable DAC module
 *
 * \return void
 *
 */
void DAC_Enable(void)
{
  DAC->CTRLA.reg |= DAC_CTRLA_ENABLE;
  while ((DAC->SYNCBUSY.reg & (DAC_SYNCBUSY_SWRST | DAC_SYNCBUSY_ENABLE)) > 0U) {}
}

/** \brief Set new DAC value
 *
 * \param [in] data New value to set for output
 * \return void
 *
 */
void DAC_Set(uint32_t data)
{
  DAC->DATA.reg = data;
  while ((DAC->SYNCBUSY.reg & DAC_SYNCBUSY_DATA) > 0U) {}
}
