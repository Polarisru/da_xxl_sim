#include "drivers.h"
#include "global.h"
#include "magnet.h"

#ifdef DEF_UNITTEST
  #define SPI_SELECT_NONE     0
  #define SPI_SELECT_MAGNET   1
#endif

static uint8_t MAGNET_Error = 0;
static uint8_t MAGNET_Status = 0;

/** \brief Read data from magnet sensor via SPI (AS5045)
 *
 * \param [out] val Pointer to raw value from magnet sensor
 * \return True if no error
 *
 */
bool MAGNET_Read(uint16_t *val)
{
  uint16_t value = 0;
  uint16_t status_bits = 0;
  uint8_t parity = 0;
  uint8_t i;

  while (1)
  {
    SPI_Select(SPI_SELECT_MAGNET);
    value = SPI_Receive(MAGNET_DATA_LENGTH);
    status_bits = SPI_Receive(MAGNET_STATUS_LENGTH);
    SPI_Select(SPI_SELECT_NONE);
    break;
  }

  for (i = 0; i < MAGNET_DATA_LENGTH; i++)
  {
    if ((value & (1UL << i)) > 0U)
    {
      parity++;
    }
  }
  /**< Invert value because of the rotation direction, bugfix */
  value = value ^ 0xFFFU;

  for (i = 0; i < MAGNET_STATUS_LENGTH; i++)
  {
    if ((status_bits & (1UL << i)) > 0U)
    {
      parity++;
    }
  }

  if ((parity % 2U) != 0U)
  {
    MAGNET_Error = 1;
    return false;
  }

  MAGNET_Status = (uint8_t)status_bits;

  if ((status_bits & MAGNET_ERR_MASK) != MAGNET_VALUE_OK)
  {
    MAGNET_Error = 1;
    return false;  /**< (OCF != 1) || (COF != 0) || (LIN != 0) */
  }

  MAGNET_Error = 0;

  *val = value;

  return true;
}

/** \brief Check if magnet sensor has error
 *
 * \return True if magnet sensor has error bit, false otherwise
 *
 */
bool MAGNET_HasError(void)
{
  return (MAGNET_Error > 0U);
}

/** \brief Get magnet status value
 *
 * \return Magnet status value as uint8_t
 *
 */
uint8_t MAGNET_GetStatus(void)
{
  return MAGNET_Status;
}
