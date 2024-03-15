#include "drivers.h"
#include "led.h"

/** \brief Configure blinking LED
 *
 * \return void
 *
 */
void LED_Configuration(void)
{
  /**< Configure LED pin */
}

/** \brief Toggle LED
 *
 * \return void
 *
 */
void LED_Toggle(void)
{
  static uint8_t counter = 0;

  if ((counter % 2) == 0U)
  {
    printf("ON\n");
  } else
  {
    printf("OFF\n");
  }
  counter++;
}
