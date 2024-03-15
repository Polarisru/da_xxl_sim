#include "global.h"
#include "redundancy.h"

/** \brief Process redundancy logic
 *
 * \return Decision code as uint8_t
 *
 */
uint8_t REDUNDANCY_Process(void)
{
  uint8_t result = REDUNDANCY_NO;

  if ((GLOBAL_MyStatus & STAT_CRITICAL_MASK) > 0U)
  {
    /**< ACE has critical failure, can't continue */
    result = REDUNDANCY_FREE;
  } else
  if (GLOBAL_IsMaster)
  {
    /**< I'm master */
    if ((GLOBAL_PartnerIsMaster == true) && (GLOBAL_InternalID == ID_ACE2))
    {
      /**< Partner is master and I'm ACE2 */
      result = REDUNDANCY_SLAVE;
    } else
    if ((GLOBAL_MyStatus & STAT_NONCRITICAL_MASK) > 0U)
    {
      /**< Non-critical error */
      result = REDUNDANCY_SLAVE;
      if ((GLOBAL_InternalID == ID_ACE2) && ((GLOBAL_MyStatus & STAT_NONCRITICAL_MASK_ACE2) == 0U))
      {
        /**< Don't switch to ACE1 in case of special non-critical errors */
        result = REDUNDANCY_NO;
      }
    } else
    if ((GLOBAL_MyStatus & STAT_ERROR_LOSSCOMM) > 0U)
    {
      /**< LoC */
      if (((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) > 0U) || ((GLOBAL_PartnerStatus & STAT_ERROR_LOSSCOMM) > 0U) ||
          ((GLOBAL_PartnerStatus & STAT_ERROR_ACE) > 0U))
      {
        /**< Int.comm.error OR Partner also has LoC: move to LoC position */
        result = REDUNDANCY_FAILSAFE;
      } else
      {
        /**< I have LoC, partner doesn't have LoC */
        result = REDUNDANCY_SLAVE;
      }
    } else
    {
      /**< Nothing to do, ignore */
    }
  } else
  {
    /**< I'm slave */
    if ((GLOBAL_MyStatus & STAT_ERROR_HEARTBEAT) == 0U)
    {
      /**< Heartbeat is on */
      if ((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) > 0U)
      {
        /**< Int.comm.error + HB */
        result = REDUNDANCY_FREE;
      }
    } else
    {
      /**< Heartbeat is off */
      if ((GLOBAL_MyStatus & STAT_ERROR_ACE) == 0U)
      {
        /**< I can be master */
        if (((GLOBAL_MyStatus & STAT_ERROR_NOINTCOMM) > 0U) || (GLOBAL_PartnerIsMaster == false))
        {
          /**< (Int.comm.error + no HB) OR (Partner is slave + no HB) */
          result = REDUNDANCY_MASTER;
        }
      }
    }
  }
  return result;
}
