#include "global.h"
#include "voting.h"

/** \brief Get distance between two sensor values
 *
 * \param [in] p1 Value #1
 * \param [in] p2 Value #2
 * \return Distance between two values
 *
 */
uint16_t VOTING_GetDist(uint16_t p1, uint16_t p2)
{
  uint16_t temp;

  if (p1 > p2)
  {
    temp = p1 - p2;
  } else
  {
    temp = p2 - p1;
  }
  if (temp > MAGNET_MAX_VALUE_DIV2)
  {
    temp = MAGNET_MAX_VALUE - temp;
  }

  return temp;
}

/** \brief Perform 2oo3 voting if sensors are available
 *
 * \return Comparison status as uint8_t
 *
 */
uint8_t VOTING_Process(void)
{
  uint16_t sensor1;
  uint16_t sensor2;
  uint16_t sensor3;
  uint16_t diff1;
  uint16_t diff2;
  uint8_t res;

  /**< Save all current values */
  sensor1 = GLOBAL_MagnetValue;
  sensor2 = GLOBAL_PartnerMagnet;
  sensor3 = GLOBAL_MagnetExtHall;

  /**< Calculate distance between sensors ACE1 and ACE2 */
  diff1 = VOTING_GetDist(sensor1, sensor2);
  /**< Calculate distance between sensors ACE1 and extHall */
  diff2 = VOTING_GetDist(sensor1, sensor3);
  /**< Calculate distance between sensors ACE2 and extHall */
  //diff3 = VOTING_GetDist(sensor2, sensor3);

  if (((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) == 0U) && ((GLOBAL_ExtStatus & EXTSTAT_ERROR_EXTHALL) == 0U))
  {
    /**< Can vote, all 3 sensors are present */
    if ((diff1 <= MAX_MAGNET_DEVIATION) || (diff2 <= MAX_MAGNET_DEVIATION))
    {
      /**< One of distances is within limits, so we can trust our sensor */
      res = VOTING_OK;
        if (diff1 > MAX_MAGNET_DEVIATION)
        {
          /**< own ACE Sensor vs. other ACE Sensor mismatch */
          res = VOTING_OK_ACE2ACE;
        }

        if (diff2 > MAX_MAGNET_DEVIATION)
        {
          /**< own ACE Sensor vs. Voting Sensor mismatch */
          res = VOTING_OK_ACE2VOT;
        }
    } else
    {
      res = VOTING_ERROR;
    }
  } else
  if ((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) == 0U)
  {
    /**< No extHall sensor detected, compare with ACE2 */
    if (diff1 > MAX_MAGNET_DEVIATION)
    {
      /**< Values of ACEs differ, it's an error */
      res = VOTING_ERROR;
    } else
    {
      res = VOTING_OK_ACE2VOT;
    }
  } else
  if ((GLOBAL_ExtStatus & EXTSTAT_ERROR_EXTHALL) == 0U)
  {
    /**< No partner ACE sensor detected, compare with extHall sensor */
    if (diff2 > MAX_MAGNET_DEVIATION)
    {
      /**< Values of ACE1 and extHall differ, stop the motor */
      res = VOTING_ERROR;
    } else
    {
      res = VOTING_OK_ACE2ACE;
    }
  } else
  {
    /**< Standalone, always Ok */
    res = VOTING_OK_ACEVBOTH;
  }

  return res;
}
