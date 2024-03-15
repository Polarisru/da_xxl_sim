#include "counters.h"
#include "eeprom.h"
#include "monitor.h"
#include "motor.h"

static uint8_t  COUNTERS_LoadMs[COUNTER_LOAD_LAST];
static uint8_t stall_cnt;

/** \brief Reset selected counter
 *
 * \param [in] id ID of the counter
 * \return void
 *
 */
void COUNTERS_Reset(uint8_t id)
{
  if (id >= COUNTER_LOAD_LAST)
  {
    return;
  }
  COUNTERS_LoadMs[id] = 0;
}

/** \brief Get value of selected counter
 *
 * \param [in] id ID of the counter
 * \return Counter value as uint8_t
 *
 */
uint8_t COUNTERS_GetValue(uint8_t id)
{
  if (id >= COUNTER_LOAD_LAST)
  {
    return 0;
  }
  return COUNTERS_LoadMs[id];
}

/** \brief Process load counters
 *
 * \return void
 *
 */
void COUNTERS_Process(void)
{
  uint8_t i;

  /**< Increment load counters depending on measure current */
  i = MONITOR_GetCurrent();
  if (i > CURR_LOAD_100)
  {
    COUNTERS_LoadMs[COUNTER_LOAD_100]++;
  } else
  if (i > CURR_LOAD_75)
  {
    COUNTERS_LoadMs[COUNTER_LOAD_75]++;
  } else
  if (i > CURR_LOAD_50)
  {
    COUNTERS_LoadMs[COUNTER_LOAD_50]++;
  } else
  if (i > CURR_LOAD_25)
  {
    COUNTERS_LoadMs[COUNTER_LOAD_25]++;
  } else
  {
    COUNTERS_LoadMs[COUNTER_LOAD_0]++;
  }
  /**< Process stall events */
  if ((i > STALL_CURR) && (abs(MOTOR_GetErrPos()) > STALL_DIFF_POS))
  {
    if (stall_cnt == STALL_TIME)
    {
      EEPROM_IncStallEvents();
    }
    if (stall_cnt <= STALL_TIME)
    {
      stall_cnt++;
    }
  }

  /**< Increment runtime counter */
  COUNTERS_LoadMs[COUNTER_LOAD_ALL]++;
  for (i = 0; i < COUNTER_LOAD_LAST; i++)
  {
    if (COUNTERS_LoadMs[i] > 99U)
    {
      EEPROM_IncCounter(i);
      COUNTERS_LoadMs[i] = 0;
    }
  }
}
