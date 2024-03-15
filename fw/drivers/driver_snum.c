#include "driver_snum.h"

/** \brief Get unique MCU serial number
 *
 * \param [out] data Pointer to data buffer to store serial number
 * \return void
 *
 */
void SNUM_Read(uint8_t *data)
{
  (void)memcpy(&data[0],  (void*)SNUM_ADDR_1, sizeof(uint32_t));
  (void)memcpy(&data[4],  (void*)SNUM_ADDR_2, sizeof(uint32_t));
  (void)memcpy(&data[8],  (void*)SNUM_ADDR_3, sizeof(uint32_t));
  (void)memcpy(&data[12], (void*)SNUM_ADDR_4, sizeof(uint32_t));
}
