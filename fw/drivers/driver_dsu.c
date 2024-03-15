#include "driver_dsu.h"

/** \brief Initialize DSU module
 *
 * \return void
 *
 */
void DSU_Init(void)
{
  MCLK->APBBMASK.reg |= MCLK_APBBMASK_DSU;
  PAC->WRCTRL.reg = PAC_WRCTRL_KEY_CLR | PAC_WRCTRL_PERID(ID_DSU);
}

/** \brief Start MBIST (memory built-in self test)
 *
 * \param [in] addr Starting address in memory
 * \param [in ]len Length of the checked memory area
 * \return void
 *
 */
void DSU_StartMemoryTest(uint32_t addr, uint32_t len)
{
  DSU->ADDR.reg = addr;
  DSU->LENGTH.reg = len;
  DSU->STATUSA.bit.DONE = 1;
  DSU->CTRL.reg = DSU_CTRL_MBIST;
}

/** \brief Get MBIST result
 *
 * \return True if memory test was successful
 *
 */
bool DSU_IsMemoryOk(void)
{
  return ((DSU->STATUSA.reg & DSU_STATUSA_FAIL) == false);
}

/** \brief Check if DSU action is ready
 *
 * \return True if action is ready
 *
 */
bool DSU_IsDone(void)
{
  return (DSU->STATUSA.reg & DSU_STATUSA_DONE);
}

/** \brief Start CRC calculation
 *
 * \param [in] addr Starting address
 * \param [in] len Length of buffer
 * \return void
 *
 */
void DSU_StartCRC(uint32_t addr, uint32_t len)
{
  DSU->ADDR.reg = addr;
  DSU->LENGTH.reg = len;
  DSU->DATA.reg = 0xFFFFFFFFU;
  DSU->STATUSA.bit.DONE = 1;
  DSU->CTRL.reg = DSU_CTRL_CRC;
}

/** \brief Get calculated CRC32 value
 *
 * \return CRC32 value as uint32_t
 *
 */
uint32_t DSU_GetCRC(void)
{
  return (DSU->DATA.reg ^ 0xFFFFFFFFU);
}
