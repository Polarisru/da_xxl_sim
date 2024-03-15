#include "driver_wdt.h"
#include "wdt.h"

/** \brief Initialize watchdog
 *
 * \return void
 *
 */
void WDT_Init(uint8_t period)
{
  uint8_t tmp;

  if (period >= WDT_PER_LAST)
  {
    return;
  }

  MCLK->APBAMASK.reg |= MCLK_APBAMASK_WDT;
  WDT->CTRLA.reg &= ~WDT_CTRLA_WEN;
  while ((WDT->SYNCBUSY.reg & (WDT_SYNCBUSY_ENABLE | WDT_SYNCBUSY_WEN | WDT_SYNCBUSY_ALWAYSON)) > 0U) {}

  tmp = WDT->CONFIG.reg;
  tmp &= ~WDT_CONFIG_PER_Msk;
  tmp |= WDT_CONFIG_PER(period);
  WDT->CONFIG.reg = tmp;
}

/** \brief Enable watchdog
 *
 * \return void
 *
 */
void WDT_Enable(void)
{
  WDT->CTRLA.reg |= WDT_CTRLA_ENABLE;
  while ((WDT->SYNCBUSY.reg & (WDT_SYNCBUSY_ENABLE | WDT_SYNCBUSY_WEN | WDT_SYNCBUSY_ALWAYSON)) > 0U) {}
}

/** \brief Disable watchdog
 *
 * \return void
 *
 */
void WDT_Disable(void)
{
  WDT->CTRLA.reg &= ~WDT_CTRLA_ENABLE;
  while ((WDT->SYNCBUSY.reg & (WDT_SYNCBUSY_ENABLE | WDT_SYNCBUSY_WEN | WDT_SYNCBUSY_ALWAYSON)) > 0U) {}
}

/** \brief Reset watchdog
 *
 * \return void
 *
 */
void WDT_Reset(void)
{
  WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  while ((WDT->SYNCBUSY.reg & WDT_SYNCBUSY_CLEAR) > 0U) {}
}

/** \brief Restart MCU by activating watchdog
 *
 * \return void
 *
 */
void WDT_Restart(void)
{
  WDT_Init(WDT_PER_8CYCLES);
  WDT_Enable();
  while (1) {}
}
