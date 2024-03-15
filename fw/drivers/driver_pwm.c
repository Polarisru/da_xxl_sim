#include "driver_pwm.h"
#include "driver_gpio.h"

#define PWM_PORT        GPIO_PORTA
#define PWM_PIN         21
#define PWM_PIN_MUX     MUX_PA21E_TC3_WO1

#define PWM_TIMER       TC3
#define PWM_GCLK_ID     TC3_GCLK_ID
#define PWM_TIMER_MCLK  MCLK_APBCMASK_TC3

#define PWM_WO_CHANNEL  1

#define PWM_MAX_VALUE   255

//void TC0_Handler(void)
//{
//  if (PWM_TIMER->COUNT8.INTFLAG.reg & TC_INTFLAG_MC1)
//  {
//    PWM_TIMER->COUNT8.INTFLAG.reg = TC_INTFLAG_MC1;
//    PORT_IOBUS->Group[PWM_PORT].OUTTGL.reg = (1UL << PWM_PIN);
//  }
//}

void PWM_Init(void)
{
  /**< Enable PWM pin */
  GPIO_ClearPin(PWM_PORT, PWM_PIN);
  GPIO_SetDir(PWM_PORT, PWM_PIN, true);
  GPIO_SetFunction(PWM_PORT, PWM_PIN, PWM_PIN_MUX);

  /**< Power on the device */
  MCLK->APBCMASK.reg |= PWM_TIMER_MCLK;
  GCLK->PCHCTRL[PWM_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1UL << GCLK_PCHCTRL_CHEN_Pos);

  /**< Reset timer module */
  PWM_TIMER->COUNT8.CTRLA.reg = TC_CTRLA_SWRST;
  while ((PWM_TIMER->COUNT8.SYNCBUSY.reg & TC_SYNCBUSY_SWRST) > 0U) {}
  /**< Set counting up */
  PWM_TIMER->COUNT8.CTRLBCLR.bit.DIR = 0;
  while ((PWM_TIMER->COUNT8.SYNCBUSY.reg & TC_SYNCBUSY_CTRLB) > 0U) {}

  /**< Configure timer */
  PWM_TIMER->COUNT8.CTRLA.reg = TC_CTRLA_PRESCSYNC_GCLK | TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_MODE_COUNT8;

  /**< Select the waveform generation mode -> normal PWM */
  PWM_TIMER->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
  /**< Set the selected period */
  PWM_TIMER->COUNT8.PER.reg = PWM_MAX_VALUE - 1;
  while ((PWM_TIMER->COUNT8.SYNCBUSY.reg & TC_SYNCBUSY_PER) > 0U) {}
  PWM_TIMER->COUNT8.COUNT.reg = 0;
  //PWM_TIMER->COUNT8.CC[0].reg = PWM_MAX_VALUE - 1;

//  PWM_TIMER->COUNT8.EVCTRL.bit.MCEO1 = 1;
//  NVIC_ClearPendingIRQ(TC0_IRQn);
//  PWM_TIMER->COUNT8.INTENSET.reg = TC_INTENSET_MC1;//TC_INTENSET_OVF;
//  NVIC_EnableIRQ(TC0_IRQn);
}

/** \brief Enable PWM channel
 *
 * \param [in] channel Number of the channel
 * \return void
 *
 */
void PWM_Enable(uint8_t channel)
{
  (void) channel;
  PWM_TIMER->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
  while ((PWM_TIMER->COUNT8.SYNCBUSY.reg & TC_SYNCBUSY_ENABLE) > 0U) {}
}

/** \brief Disable PWM channel
 *
 * \param [in] channel Number of the channel
 * \return void
 *
 */
void PWM_Disable(uint8_t channel)
{
  (void) channel;
  PWM_TIMER->COUNT8.CTRLA.reg &= (~TC_CTRLA_ENABLE);
  while ((PWM_TIMER->COUNT8.SYNCBUSY.reg & TC_SYNCBUSY_ENABLE) > 0U) {}
}

/** \brief Set PWM value
 *
 * \param [in] value New PWM value to set
 * \return void
 *
 */
void PWM_Set(uint8_t value)
{
  PWM_TIMER->COUNT8.CC[PWM_WO_CHANNEL].reg = value;
}
