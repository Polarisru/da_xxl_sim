#include "drivers.h"
#include "analog.h"

static const TAnalogChannel ANALOG_Channels[ADC_CHANNEL_NUM] = {
  {GPIO_PORTA, 2, MUX_PA02B_ADC0_AIN0, ADC_INPUTCTRL_MUXPOS_AIN0_Val},
  {GPIO_PORTA, 3, MUX_PA03B_ADC0_AIN1, ADC_INPUTCTRL_MUXPOS_AIN1_Val},
  {GPIO_PORTB, 8, MUX_PB08B_ADC0_AIN2, ADC_INPUTCTRL_MUXPOS_AIN2_Val},
  {GPIO_PORTB, 9, MUX_PB09B_ADC0_AIN3, ADC_INPUTCTRL_MUXPOS_AIN3_Val}
};

/** \brief Set ADC channel for next conversion
 *
 * \param [in] channel Number of channel
 * \return void
 *
 */
void ANALOG_SetChannel(uint8_t channel)
{
  ADC_SetChannel(ANALOG_Channels[channel].channel);
}

/** \brief Get ADC result
 *
 * \return ADC values as uint16_t
 *
 */
uint16_t ANALOG_GetValue(void)
{
  while (ADC_IsReady() != true) {}
  return ADC_GetResult();
}

/** \brief ADC configuration
 *
 * \return void
 *
 */
void ANALOG_Configuration(void)
{
  uint8_t i;

  /**< Setup analog pins */
  for (i = 0; i < ADC_CHANNEL_NUM; i++)
  {
    GPIO_SetDir(ANALOG_Channels[i].port, ANALOG_Channels[i].pin, false);
    GPIO_SetFunction(ANALOG_Channels[i].port, ANALOG_Channels[i].pin, ANALOG_Channels[i].pin_mode);
  }

  /**< Setup ADC */
  VREF_Init(SUPC_VREF_SEL_4V096_Val);
	ADC_Init(ADC_REFCTRL_REFSEL_INTREF_Val, ADC_CTRLC_RESSEL_12BIT_Val);
}

