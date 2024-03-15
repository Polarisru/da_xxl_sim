#include "drivers.h"
#include "analog.h"
#include "conversion.h"
#include "drv8320.h"
#include "eeprom.h"
#include "power.h"
#include "heartbeat.h"

#define POWER_AC_CHANNEL    0U
#define POWER_PORT          GPIO_PORTA
#define POWER_PIN           5U
#define POWER_MUX           MUX_PA05B_AC_AIN1
#define POWER_AC_INP_POS    AC_COMPCTRL_MUXPOS_PIN1_Val

#define POWER_AC_FILTER     AC_COMPCTRL_FLEN_MAJ5_Val
#define POWER_AC_INP_NEG    AC_COMPCTRL_MUXNEG_VSCALE_Val
#define POWER_AC_VCC        5.0F
#define POWER_AC_REF        2.0F
#define POWER_AC_SCALER     (POWER_AC_REF * 64 / POWER_AC_VCC)

/** \brief AC interrupt handler to process power off
 *
 * \return void
 *
 */
void AC_Handler(void)
{
  if ((AC->INTFLAG.reg & AC_INTFLAG_COMP0) > 0U)
  {
    AC->INTFLAG.reg = AC_INTFLAG_COMP0;
    POWER_RestartApp();
  }
}

/** \brief Perform application restart
 *
 * \return void
 *
 */
void POWER_RestartApp(void)
{
  /**< Stop interrupts and block MCU after writing to EEPROM */
  __disable_irq();
  /**< Disable motor, very important!!! */
  DRV8320_DoFree(true);
  /**< Disable all pins to stop leakages */
  GPIO_DisablePort(GPIO_PORTA);
  GPIO_DisablePortPins(GPIO_PORTB, ~(1UL << HEARTBEAT_PIN));
  #ifdef DEF_DUPLEX
  HEARTBEAT_Disable();
  #endif

  /**< Save all settings and counters */
  //CHAIN_SetOutput(true);
  EEPROM_SaveBoth();
  EEPROM_SaveStrings();
  EEPROM_SaveCounters();
  //CHAIN_SetOutput(false);
  /**< Wait for watchdog activation */
  while (1) {}
}

/** \brief Check power supply
 *
 * \return True if power supply is not zero
 *
 */
bool POWER_CheckSupply(void)
{
  uint8_t volt1;
  uint8_t volt2;

  ANALOG_SetChannel(ADC_CHANNEL_VOLTAGE1);
  volt1 = CONVERSION_CalcVoltage(ANALOG_GetValue());
  ANALOG_SetChannel(ADC_CHANNEL_VOLTAGE2);
  volt2 = CONVERSION_CalcVoltage(ANALOG_GetValue());

  return ((volt1 > VOLTAGE_ZERO) || (volt2 > VOLTAGE_ZERO));
}

/** \brief Initialize power monitoring module
 *
 * \return void
 *
 */
void POWER_Configuration(void)
{
  /**< Configure pins */
  GPIO_SetFunction(POWER_PORT, POWER_PIN, POWER_MUX);
  /**< Configure analog comparator */
  AC_Init(POWER_AC_CHANNEL, POWER_AC_FILTER, POWER_AC_INP_POS, POWER_AC_INP_NEG, POWER_AC_SCALER, true);
  /**< Enable AC interrupt */
  //NVIC_SetPriority(AC_IRQn, 0);
  NVIC_EnableIRQ(AC_IRQn);
}
