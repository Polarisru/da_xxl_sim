#include "drivers.h"
#include "chain.h"

#define CHAIN_PORT          GPIO_PORTA
#define CHAIN_DI_PIN        13
#define CHAIN_DO_PIN        9

/** \brief Configure pins for chain network (DI/DO)
 *
 * \return void
 *
 */
void CHAIN_Configuration(void)
{
  /**< configure DI pin and DO pin */
  GPIO_ClearPin(CHAIN_PORT, CHAIN_DI_PIN);
  GPIO_ClearPin(CHAIN_PORT, CHAIN_DO_PIN);
  GPIO_SetDir(CHAIN_PORT, CHAIN_DI_PIN, false);
  GPIO_SetDir(CHAIN_PORT, CHAIN_DO_PIN, true);
  /**< DI is configured as input with pull-down */
  GPIO_SetPullMode(CHAIN_PORT, CHAIN_DI_PIN, GPIO_PULL_DOWN);
  GPIO_SetFunction(CHAIN_PORT, CHAIN_DI_PIN, GPIO_PIN_FUNC_OFF);
  GPIO_SetFunction(CHAIN_PORT, CHAIN_DO_PIN, GPIO_PIN_FUNC_OFF);
}

/** \brief Read chain input pin (DI)
 *
 * \return True if DI pin is set
 *
 */
bool CHAIN_IsInputSet(void)
{
  return GPIO_GetPin(CHAIN_PORT, CHAIN_DI_PIN);
}

/** \brief Set chain output pin (DO)
 *
 * \param [in] on Switch output pin on/off
 * \return void
 *
 */
void CHAIN_SetOutput(bool on)
{
  if (on)
  {
    GPIO_SetPin(CHAIN_PORT, CHAIN_DO_PIN);
  } else
  {
    GPIO_ClearPin(CHAIN_PORT, CHAIN_DO_PIN);
  }
}

/** \brief Get chain output state (DO)
 *
 * \return True if DO is set, false otherwise
 *
 */
bool CHAIN_GetOutput(void)
{
  return GPIO_GetOutputPin(CHAIN_PORT, CHAIN_DI_PIN);
}
