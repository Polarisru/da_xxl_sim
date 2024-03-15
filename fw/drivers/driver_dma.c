#include "driver_dma.h"

static volatile bool DMA_TransferComplete[DMA_CHANNELS_NUM];

static DmacDescriptor dma_descr[DMA_CHANNELS_NUM]  __attribute__((aligned(16))) SECTION_DMAC_DESCRIPTOR;
static DmacDescriptor writeback[DMA_CHANNELS_NUM] __attribute__((aligned(16))) SECTION_DMAC_DESCRIPTOR;
static DmacDescriptor descriptor  __attribute__((aligned(16))) SECTION_DMAC_DESCRIPTOR;

/**< Interrupt for DMA transfer completion */
void DMAC_Handler(void)
{
  uint8_t channel;
  uint8_t old_channel;
  uint32_t res;

  old_channel = DMAC->CHID.reg;
  res = DMAC->INTSTATUS.reg;
  /**< Get channel number */
  //channel =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk;
  //if (channel < DMA_CHANNELS_NUM)
  for (channel = 0; channel < DMA_CHANNELS_NUM; channel++)
  {
    if ((res & (1UL << channel)) > 0U)
    {
      /**< Set transfer complete flag */
      DMA_TransferComplete[channel] = true;
      /**< Select channel to work with */
      DMAC->CHID.reg = DMAC_CHID_ID(channel);
      /**< Clear all flags for DMA channel */
      DMAC->CHINTFLAG.reg = (DMAC_CHINTENCLR_TCMPL | DMAC_CHINTENCLR_TERR | DMAC_CHINTENCLR_SUSP);
      /**< Disable DMA transfer */
      DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
    }
  }
  DMAC->CHID.reg = old_channel;
}

/** \brief Initialize DMA module
 *
 * \return void
 *
 */
void DMA_Init(void)
{
  uint8_t i;

  MCLK->AHBMASK.reg |= MCLK_AHBMASK_DMAC;

  DMAC->CTRL.reg &= ~DMAC_CTRL_DMAENABLE;
  DMAC->CTRL.reg = DMAC_CTRL_SWRST;

  DMAC->BASEADDR.reg = (uint32_t)dma_descr;
  DMAC->WRBADDR.reg = (uint32_t)writeback;

  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0x0FU);

  for (i = 0; i < DMA_CHANNELS_NUM; i++)
  {
    DMA_TransferComplete[i] = true;
  }

  NVIC_EnableIRQ(DMAC_IRQn);
  NVIC_SetPriority(DMAC_IRQn, 8);
}

/** \brief Setup DMA transfer for selected channel
 *
 * \param [in] channel Number of the channel to act
 * \param [in] settings Pointer to TDmaSettings structure with DMA settings
 * \return void
 *
 */
void DMA_SetupChannel(uint8_t channel, TDmaSettings *settings)
{
  uint16_t btctrlVal;

  if (channel >= DMA_CHANNELS_NUM)
  {
    return;
  }

  /**< Setup channel */
  DMAC->CHID.reg = DMAC_CHID_ID(channel);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  DMAC->SWTRIGCTRL.reg &= (uint32_t)(~(1UL << channel));

  if (settings->trig_src == 0U)
  {
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(settings->priority) | DMAC_CHCTRLB_TRIGACT_BLOCK;
  } else
  {
    DMAC->CHCTRLB.reg = DMAC_CHCTRLB_LVL(settings->priority) | DMAC_CHCTRLB_TRIGSRC(settings->trig_src) | DMAC_CHCTRLB_TRIGACT_BEAT;
  }
  /**< Enable interrupts */
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK;
  /**< Prepare descriptor block */
  descriptor.DESCADDR.reg = 0;          // only one block to transfer, so 0
  descriptor.BTCNT.reg = settings->len;
  btctrlVal = DMAC_BTCTRL_VALID | DMAC_BTCTRL_BEATSIZE(settings->beat_size);
  descriptor.DSTADDR.reg = (uint32_t)settings->dst_addr;
  if (settings->dst_inc == true)
  {
    descriptor.DSTADDR.reg += settings->len * (1UL << settings->beat_size);
    btctrlVal |= DMAC_BTCTRL_DSTINC;
  }
  descriptor.SRCADDR.reg = (uint32_t)settings->src_addr;
  if (settings->src_inc == true)
  {
    descriptor.SRCADDR.reg += settings->len * (1UL << settings->beat_size);
    btctrlVal |= DMAC_BTCTRL_SRCINC;
  }
  descriptor.BTCTRL.reg = btctrlVal;

  (void) memcpy(&dma_descr[channel], &descriptor, sizeof(DmacDescriptor));
}

/** \brief Start DMA transfer, channel should be already initialized
 *
 * \param [in] channel Number of the channel to act
 * \return void
 *
 */
void DMA_StartChannel(uint8_t channel)
{
  if (channel >= DMA_CHANNELS_NUM)
  {
    return;
  }

  /**< Setup channel */
  DMAC->CHID.reg = DMAC_CHID_ID(channel);
  if (DMA_TransferComplete[channel] == false)
  {
    DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
  }

  /**< Start DMA transfer */
  DMA_TransferComplete[channel] = false;
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void DMA_StopChannel(uint8_t channel)
{
  if (channel >= DMA_CHANNELS_NUM)
  {
    return;
  }

  DMAC->CHID.reg = DMAC_CHID_ID(channel);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;
}

/** \brief Check if DMA transmission is ready
 *
 * \param [in] channel Number of the channel to act
 * \return True if transmission is ready
 *
 */
bool DMA_IsReady(uint8_t channel)
{
  if (channel >= DMA_CHANNELS_NUM)
  {
    return true;
  }

  return DMA_TransferComplete[channel];
}
