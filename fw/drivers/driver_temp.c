#include "driver_temp.h"
#include "tsens.h"

/** \brief Initialize internal temperature sensor
 *
 * \return void
 *
 */
void TEMP_Init(void)
{
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_TSENS;
  GCLK->PCHCTRL[TSENS_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);

  TSENS->CTRLA.reg = TSENS_CTRLA_SWRST;
  while ((TSENS->SYNCBUSY.reg & TSENS_SYNCBUSY_SWRST) > 0U) {}

  TSENS->CAL.reg = TSENS_CAL_TCAL(CONF_TSENS_CAL_TCAL) | TSENS_CAL_FCAL(CONF_TSENS_CAL_FCAL);
  TSENS->GAIN.reg = CONF_TSENS_CAL_GAIN;
  TSENS->OFFSET.reg = CONF_TSENS_CAL_OFFSET;

  TSENS->CTRLC.reg = TSENS_CTRLC_FREERUN | TSENS_CTRLC_WINMODE(TSENS_CTRLC_WINMODE_DISABLE_Val);
  TSENS->CTRLA.reg = TSENS_CTRLA_ENABLE;
  TSENS->CTRLB.reg = TSENS_CTRLB_START;
}

/** \brief Get temperature of the internal sensor
 *
 * \return Temperature in °C as float
 *
 */
float TEMP_GetValue(void)
{
  uint32_t value = TSENS->VALUE.bit.VALUE & 0xFFFFFFU;

  if ((value & 0x800000U) > 0U)
  {
    /**< Negative value */
    value = 0x1000000U - value;
    return (-(float)value / 100);
  }

  return ((float)value / 100);
}

/** \brief Get temperature of the internal sensor as byte
 *
 * \return Temperature in °C + 50°C as uint8_t
 *
 */
uint8_t TEMP_GetByteValue(void)
{
  #define TEMP_MIN_VALUE      50U
  #define TEMP_MAX_VALUE      200U

  uint32_t value = TSENS->VALUE.bit.VALUE & 0xFFFFFFU;
  uint8_t  uval8;

  if ((value & 0x800000U) > 0U)
  {
    /**< Negative value */
    value = 0x1000000U - value;
    uval8 = (uint8_t)((value + TEMP_MIN_VALUE) / 100U);
    if (uval8 > TEMP_MIN_VALUE)
    {
      return 0;
    }
    return (TEMP_MIN_VALUE - uval8);
  } else
  {
    uval8 = (uint8_t)((value + TEMP_MIN_VALUE) / 100U);
    if (uval8 > TEMP_MAX_VALUE)
    {
      return 0xff;
    }
    return (uval8 + TEMP_MIN_VALUE);
  }
}
