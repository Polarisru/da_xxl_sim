#include "drivers.h"
#include "eeprom.h"
#include "global.h"
#include "inetwork.h"
#include "master.h"

/**< Pin to detect master/slave */
#define MASTER_PORT               GPIO_PORTB
#define MASTER_PIN                3

/** \brief Initialize master/slave roles
 *
 * \return void
 *
 */
void MASTER_Configuration(void)
{
  GPIO_ClearPin(MASTER_PORT, MASTER_PIN);
  GPIO_SetDir(MASTER_PORT, MASTER_PIN, false);
  GPIO_SetFunction(MASTER_PORT, MASTER_PIN, GPIO_PIN_FUNC_OFF);
  GPIO_SetPullMode(MASTER_PORT, MASTER_PIN, GPIO_PULL_UP);
	/**< Check config pin to detect master */
	/**< Master ACE is connected to GND */
  if (GPIO_GetPin(MASTER_PORT, MASTER_PIN) == true)
  {
    GLOBAL_InternalID = ID_ACE2;
    GLOBAL_WaitTN = ACE1_TN;        // previous tokenID  (>Hall-->ACE1-->ACE2-->Hall-->ACE1...)
    GLOBAL_MyTN = ACE2_TN;          // my token
    GLOBAL_IsMaster = false;
  } else
  {
    GLOBAL_InternalID = ID_ACE1;
    GLOBAL_WaitTN = HALL_TN;        // previous tokenID  (>Hall-->ACE1-->ACE2-->Hall-->ACE1...)
    GLOBAL_MyTN = ACE1_TN;          // my token
    GLOBAL_IsMaster = true;
  }
}
