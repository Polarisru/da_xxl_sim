#include "global.h"
#include "timeouts.h"

/**< Timeouts */
static uint16_t timeout_PartnerACE;
static uint16_t timeout_Hall;
static uint16_t timeout_Connection;

/** \brief Reset timer
 *
 * \param [in] num Number of timer to reset
 * \return void
 *
 */
void TIMEOUTS_Reset(uint8_t num)
{
  switch (num)
  {
    case TIMEOUT_TYPE_PARTNER:
      timeout_PartnerACE = 0U;
      break;
    case TIMEOUT_TYPE_HALL:
      timeout_Hall = 0U;
      break;
    case TIMEOUT_TYPE_HOST:
      timeout_Connection = 0U;
      break;
    case TIMEOUT_TYPE_ALL:
      timeout_PartnerACE = 0U;
      timeout_Hall = 0U;
      timeout_Connection = 0U;
      break;
    default:
      /**< Unknown timeout ID, ignore */
      break;

  }
}

/** \brief Increment timer
 *
 * \param [in] num Number of timer to increment
 * \return void
 *
 */
void TIMEOUTS_Inc(uint8_t num)
{
  switch (num)
  {
    case TIMEOUT_TYPE_PARTNER:
      timeout_PartnerACE++;
      break;
    case TIMEOUT_TYPE_HALL:
      timeout_Hall++;
      break;
    case TIMEOUT_TYPE_HOST:
      timeout_Connection++;
      break;
    default:
      /**< Should not occur, ignore */
      break;
  }
}

/** \brief Check if timeout occurred
 *
 * \param [in] num Number of timer to increment
 * \return True if timeout event is detected
 *
 */
bool TIMEOUTS_IsActive(uint8_t num)
{
  switch (num)
  {
    case TIMEOUT_TYPE_PARTNER:
      return (timeout_PartnerACE > (TIMEOUT_PARTNER_MS / LOGIC_TASK_DELAY_MS));
    case TIMEOUT_TYPE_PARTNER_SHORT:
      return (timeout_PartnerACE > (TIMEOUT_PARTNER_SHORT_MS / LOGIC_TASK_DELAY_MS));
    case TIMEOUT_TYPE_HALL:
      return (timeout_Hall > (TIMEOUT_HALL_MS / LOGIC_TASK_DELAY_MS));
    case TIMEOUT_TYPE_HOST:
      return (timeout_Connection > (uint16_t)EE_LossTime * 100U / LOGIC_TASK_DELAY_MS);
    default:
      /**< Should not occur, ignore */
      return false;
  }
}

/** \brief Get timeout value
 *
 * \param [in] num Number of timer to read
 * \return Current timeout value as uint16_t
 *
 */
uint16_t TIMEOUTS_GetValue(uint8_t num)
{
  switch (num)
  {
    case TIMEOUT_TYPE_PARTNER:
      return timeout_PartnerACE;
    case TIMEOUT_TYPE_PARTNER_SHORT:
      return timeout_PartnerACE;
    case TIMEOUT_TYPE_HALL:
      return timeout_Hall;
    case TIMEOUT_TYPE_HOST:
      return timeout_Connection;
    default:
      /**< Should not occur, ignore */
      return 0;
  }
}

/** \brief Initialize timeouts module
 *
 * \return void
 *
 */
void TIMEOUTS_Configure(void)
{
  TIMEOUTS_Reset(TIMEOUT_TYPE_ALL);
}

