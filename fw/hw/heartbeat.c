#include "drivers.h"
#include "heartbeat.h"

static int8_t HEARTBEAT_Counter;
static bool HEARTBEAT_Role;
static int8_t HEARTBEAT_MaxCounter;
static int8_t HEARTBEAT_DownCounter;

/** \brief Enable heartbeat signal (for master)
 *
 * \param void
 * \return void
 *
 */
void HEARTBEAT_Enable(void)
{
  HEARTBEAT_Role = true;
  HEARTBEAT_Counter = HEARTBEAT_NEW_ROLE_VALUE;
  /**< Configure pin as output */
  GPIO_ClearPin(HEARTBEAT_PORT, HEARTBEAT_PIN);
  GPIO_SetDir(HEARTBEAT_PORT, HEARTBEAT_PIN, true);
  GPIO_SetFunction(HEARTBEAT_PORT, HEARTBEAT_PIN, GPIO_PIN_FUNC_OFF);
}

/** \brief Disable heartbeat signal (for slave)
 *
 * \param void
 * \return void
 *
 */
void HEARTBEAT_Disable(void)
{
  HEARTBEAT_Role = false;
  HEARTBEAT_Counter = HEARTBEAT_NEW_ROLE_VALUE;
  /**< Configure pin as input */
  GPIO_ClearPin(HEARTBEAT_PORT, HEARTBEAT_PIN);
  GPIO_SetDir(HEARTBEAT_PORT, HEARTBEAT_PIN, false);
  GPIO_SetFunction(HEARTBEAT_PORT, HEARTBEAT_PIN, GPIO_PIN_FUNC_OFF);
}

/** \brief Initialize Heartbeat module, configure pins
 *
 * \param [in] out True for ACE1 and false for ACE2
 * \return void
 *
 */
void HEARTBEAT_Configuration(bool out)
{
  /**< Configure both pins as output and input for ACE1/ACE2 */
  if (out == true)
  {
    HEARTBEAT_Enable();
  } else
  {
    HEARTBEAT_Disable();
  }
  GPIO_SetPullMode(HEARTBEAT_PORT, HEARTBEAT_PIN, GPIO_PULL_UP);
  HEARTBEAT_MaxCounter = HEARTBEAT_DEFAULT_MAX_VALUE;
  /**< X heartbeats are needed to reset heartbeat timeout counter (HEARTBEAT_Counter) */
  HEARTBEAT_DownCounter = HEARTBEAT_DOWN_CNT_MAX_VALUE;
}

/** \brief Set maximal value for a counter
 *
 * \param [in] value Maximal value for a counter
 * \return void
 *
 */
void HEARTBEAT_SetLimit(int8_t value)
{
  HEARTBEAT_MaxCounter = value;
}

/** \brief Process heartbeat functionality, must be called periodically
 *
 * \return True if everything is Ok, false otherwise (heartbeat signal is missing)
 *
 */
bool HEARTBEAT_Process(void)
{
  static bool old_pin = false;
  static bool slow_mode = false;
  bool new_pin;

  if (HEARTBEAT_Role == true)
  {
    /**< Master generates heartbeat at half the frequency */
    slow_mode = !slow_mode;
    if (slow_mode == true)
    {
      /**< Generate heartbeat */
      GPIO_TogglePin(HEARTBEAT_PORT, HEARTBEAT_PIN);
    }
    return true;
  } else
  {
    /**< Read heartbeat line */
    new_pin = GPIO_GetPin(HEARTBEAT_PORT, HEARTBEAT_PIN);

    if (new_pin == old_pin)
    {
      if (HEARTBEAT_Counter < HEARTBEAT_MaxCounter)
      {
        HEARTBEAT_Counter++;
      }
    } else
    {
      if (HEARTBEAT_DownCounter <= 1)
      {
        HEARTBEAT_DownCounter = HEARTBEAT_DOWN_CNT_MAX_VALUE;
        if (HEARTBEAT_Counter > 0)
        {
          HEARTBEAT_Counter = 0;
        }
      }
      else
      {
        HEARTBEAT_DownCounter--;
      }
    }
    old_pin = new_pin;

    return (HEARTBEAT_Counter < HEARTBEAT_MaxCounter);
  }
}
