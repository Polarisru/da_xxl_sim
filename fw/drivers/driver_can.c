#include <string.h>
#include "driver_can.h"
#include "hpl_can_config.h"
#include "hpl_can_base.h"
#include "err_codes.h"

#define CAN_GCLK_SRC     GCLK_PCHCTRL_GEN_GCLK2

/**< Supported baudrates */
static const uint32_t CAN_Bitrates[CAN_BITRATE_LAST] = {125, 250, 500, 1000, 2000, 4000};

static uint8_t can_rx_fifo_0[CONF_CAN0_F0DS * CONF_CAN0_RXF0C_F0S];
static uint8_t can_rx_fifo_1[CONF_CAN0_F0DS * CONF_CAN0_RXF0C_F0S];
static uint8_t can_tx_fifo_0[CONF_CAN0_TBDS * CONF_CAN0_TXBC_TFQS];
static uint8_t can_tx_fifo_1[CONF_CAN0_TBDS * CONF_CAN0_TXBC_TFQS];
static struct _can_tx_event_entry can_tx_event_fifo[CONF_CAN0_TXEFC_EFS];
static struct _can_standard_message_filter_element can_rx_std_filter[CONF_CAN0_SIDFC_LSS];
static struct _can_extended_message_filter_element can_rx_ext_filter[CONF_CAN0_XIDFC_LSS];
static uint8_t can_std_filter_counter;
static uint8_t can_ext_filter_counter;

/** \brief Check if CAN bus has unprocessed messages
 *
 * \return True if has unprocessed message
 *
 */
bool CAN_HasMessage(Can *channel)
{
  uint32_t ir;

  ir = channel->IR.reg;
  if ((ir & CAN_IR_RF0N) > 0U)
  {
    channel->IR.reg = ir;
    return true;
  }
  return false;
}

/** \brief Read a CAN message from CAN bus
 *
 * \param [in] channel Number of the CAN channel
 * \param [in] msg Pointer for CAN message to receive
 * \return Error code
 *
 */
int32_t CAN_ReadMsg(Can *channel, struct can_message *msg)
{
  const uint8_t dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  struct _can_rx_fifo_entry *f = NULL;
  uint32_t get_index;

  if (((channel->RXF0S.reg & CAN_RXF0S_F0FL_Msk) >> CAN_RXF0S_F0FL_Pos) == 0U)
  {
    return ERR_NOT_FOUND;
  }

  get_index = (channel->RXF0S.reg & CAN_RXF0S_F0GI_Msk) >> CAN_RXF0S_F0GI_Pos;

  if (channel == CAN0)
  {
    f = (struct _can_rx_fifo_entry *)(can_rx_fifo_0 + (get_index * CONF_CAN0_F0DS));
  } else
  {
    f = (struct _can_rx_fifo_entry *)(can_rx_fifo_1 + (get_index * CONF_CAN0_F0DS));
  }
  if (f == NULL)
  {
    return ERR_NO_RESOURCE;
  }

  if (f->R0.bit.XTD == 1U)
  {
    msg->fmt = CAN_FMT_EXTID;
    msg->id  = f->R0.bit.ID;
  } else
  {
    msg->fmt = CAN_FMT_STDID;
    /**< A standard identifier is stored into ID[28:18] */
    msg->id = f->R0.bit.ID >> 18U;
  }

  if (f->R0.bit.RTR == 1U)
  {
    msg->type = CAN_TYPE_REMOTE;
  }

  if (f->R1.bit.FDF == 1U)
  {
    msg->type = CAN_TYPE_FD;
  }

  msg->len = dlc2len[f->R1.bit.DLC];

  (void)memcpy(msg->data, f->data, msg->len);

  channel->RXF0A.bit.F0AI = get_index;

  return ERR_NONE;
}

/** \brief Write a CAN message to bus
 *
 * \param [in] channel Number of the CAN channel
 * \param [in] msg CAN message to write
 * \return Error code
 *
 */
int32_t CAN_WriteMsg(Can *channel, struct can_message *msg)
{
  struct _can_tx_fifo_entry *f = NULL;
  if (((channel->TXFQS.reg & CAN_TXFQS_TFQF) >> CAN_TXFQS_TFQF_Pos) > 0U)
  {
    return ERR_NO_RESOURCE;
  }
  if (channel == CAN0)
  {
    f = (struct _can_tx_fifo_entry *)(can_tx_fifo_0 + ((channel->TXFQS.reg & CAN_TXFQS_TFQPI_Msk) >> CAN_TXFQS_TFQPI_Pos) * CONF_CAN0_TBDS);
  } else
  {
    f = (struct _can_tx_fifo_entry *)(can_tx_fifo_1 + ((channel->TXFQS.reg & CAN_TXFQS_TFQPI_Msk) >> CAN_TXFQS_TFQPI_Pos) * CONF_CAN0_TBDS);
  }
  if (f == NULL)
  {
    return ERR_NO_RESOURCE;
  }

  if (msg->fmt == CAN_FMT_EXTID)
  {
    f->R0.val = msg->id;
    f->R0.bit.XTD = 1;
  } else
  {
    /**< A standard identifier is stored into ID[28:18] */
    f->R0.val = msg->id << 18;
  }

  if (msg->len <= 8U)
  {
    f->R1.bit.DLC = msg->len;
  } else
  if (msg->len <= 12U)
  {
    f->R1.bit.DLC = 0x9;
  } else
  if (msg->len <= 16U)
  {
    f->R1.bit.DLC = 0xA;
  } else
  if (msg->len <= 20U)
  {
    f->R1.bit.DLC = 0xB;
  } else
  if (msg->len <= 24U)
  {
    f->R1.bit.DLC = 0xC;
  } else
  if (msg->len <= 32U)
  {
    f->R1.bit.DLC = 0xD;
  } else
  if (msg->len <= 48U)
  {
    f->R1.bit.DLC = 0xE;
  } else
  if (msg->len <= 64U)
  {
    f->R1.bit.DLC = 0xF;
  } else
  {
    return ERR_NONE;
  }
  if ((msg->type == CAN_TYPE_FD) || (msg->type == CAN_TYPE_FD_BRS))
  {
    f->R1.bit.FDF = 1;
    if (msg->type == CAN_TYPE_FD_BRS)
    {
      f->R1.bit.BRS = 1;
    } else
    {
      f->R1.bit.BRS = 0;
    }
  } else
  {
    f->R1.bit.FDF = 0;
    f->R1.bit.BRS = 0;
  }

  (void)memcpy(f->data, msg->data, msg->len);

  channel->TXBAR.reg = 1 << ((channel->TXFQS.reg & CAN_TXFQS_TFQPI_Msk) >> CAN_TXFQS_TFQPI_Pos);
  return ERR_NONE;
}

/** \brief Set CAN filter (standard mask)
 *
 * \param [in] fmt Format of the filter (standard or extended)
 * \param [in] filter Pointer to structure with classic filter settings (mask/value)
 * \return Error code
 *
 */
int32_t CAN_SetFilter(enum can_format fmt, struct can_filter *filter)
{
  struct _can_standard_message_filter_element *sf;
  struct _can_extended_message_filter_element *ef;

  if (filter == NULL)
  {
    return ERR_NONE;
  }

  if (fmt == CAN_FMT_STDID)
  {
    if (can_std_filter_counter == CONF_CAN0_SIDFC_LSS)
    {
      return ERR_NO_MEMORY;
    }
    sf = &can_rx_std_filter[can_std_filter_counter];
    can_std_filter_counter++;
//    if (filter == NULL)
//    {
//      sf->S0.val = 0;
//      return ERR_NONE;
//    }
    sf->S0.val       = filter->mask;
    sf->S0.bit.SFID1 = filter->id;
    sf->S0.bit.SFT   = _CAN_SFT_CLASSIC;
    sf->S0.bit.SFEC  = _CAN_SFEC_STF0M;
  } else //if (fmt == CAN_FMT_EXTID)
  {
    if (can_ext_filter_counter == CONF_CAN0_XIDFC_LSS)
    {
      return ERR_NO_MEMORY;
    }
    ef = &can_rx_ext_filter[can_ext_filter_counter];
    can_ext_filter_counter++;
//    if (filter == NULL)
//    {
//      ef->F0.val = 0;
//      return ERR_NONE;
//    }
    ef->F0.val      = filter->id;
    ef->F0.bit.EFEC = _CAN_EFEC_STF0M;
    ef->F1.val      = filter->mask;
    ef->F1.bit.EFT  = _CAN_EFT_CLASSIC;
  }

  return ERR_NONE;
}

/** \brief Set CAN filter (range)
 *
 * \param [in] fmt Format of the filter (standard or extended)
 * \param [in] start_id First ID of the range
 * \param [in] stop_id Last ID of the range
 * \return Error code
 *
 */
int32_t CAN_SetRangeFilter(enum can_format fmt, uint32_t start_id, uint32_t stop_id)
{
  struct _can_standard_message_filter_element *sf;
  struct _can_extended_message_filter_element *ef;

  if (start_id > stop_id)
  {
    return ERR_NONE;
  }

  if (fmt == CAN_FMT_STDID)
  {
//    if (start_id > stop_id)
//    {
//      sf->S0.val = 0;
//      return ERR_NONE;
//    }
    if (can_std_filter_counter == CONF_CAN0_SIDFC_LSS)
    {
      return ERR_NO_MEMORY;
    }
    sf = &can_rx_std_filter[can_std_filter_counter];
    can_std_filter_counter++;
    sf->S0.bit.SFID1 = start_id;
    sf->S0.bit.SFID2 = stop_id;
    sf->S0.bit.SFT   = _CAN_EFT_RANGE;
    sf->S0.bit.SFEC  = _CAN_SFEC_STF0M;
  } else //if (fmt == CAN_FMT_EXTID)
  {
//    if (start_id > stop_id)
//    {
//      ef->F0.val = 0;
//      return ERR_NONE;
//    }
    if (can_ext_filter_counter == CONF_CAN0_XIDFC_LSS)
    {
      return ERR_NO_MEMORY;
    }
    ef = &can_rx_ext_filter[can_ext_filter_counter];
    can_ext_filter_counter++;
    ef->F0.val      = start_id;
    ef->F0.bit.EFEC = _CAN_EFEC_STF0M;
    ef->F1.val      = stop_id;
    ef->F1.bit.EFT  = _CAN_EFT_RANGE;
  }

  return ERR_NONE;
}

/** \brief Reset all CAN filters
 *
 * \param fmt Format of the filter (standard or extended)
 * \return void
 *
 */
void CAN_ResetFilters(enum can_format fmt)
{
  struct _can_standard_message_filter_element *sf;
  struct _can_extended_message_filter_element *ef;

  if (fmt == CAN_FMT_STDID)
  {
    for (uint8_t i = 0; i < can_std_filter_counter; i++)
    {
      sf = &can_rx_std_filter[can_std_filter_counter];
      sf->S0.val = 0;
    }
    can_std_filter_counter = 0;
  } else //if (fmt == CAN_FMT_EXTID)
  {
    for (uint8_t i = 0; i < can_ext_filter_counter; i++)
    {
      ef = &can_rx_ext_filter[can_ext_filter_counter];
      ef->F0.val = 0;
    }
    can_ext_filter_counter = 0;
  }
}

/** \brief Enable CAN interface
 *
 * \param [in] channel Number of the CAN channel
 * \return void
 *
 */
void CAN_Enable(Can *channel)
{
  channel->CCCR.reg &= ~CAN_CCCR_CCE;
  channel->CCCR.reg &= ~CAN_CCCR_INIT;
  while ((channel->CCCR.reg & CAN_CCCR_INIT) != 0) {}

  /**< New message received interrupt enable */
  channel->IE.bit.RF0NE = 1;
}

/** \brief Disable CAN interface
 *
 * \param [in] channel Number of the CAN channel
 * \return void
 *
 */
void CAN_Disable(Can *channel)
{
  /**< Put CAN to initializing mode */
  channel->CCCR.reg |= CAN_CCCR_INIT;
  while ((channel->CCCR.reg & CAN_CCCR_INIT) == 0) {}
  channel->CCCR.reg |= CAN_CCCR_CCE;

  /**< Disable Rx interrupt */
  channel->IE.bit.RF0NE = 0;
  /**< Clear Rx flag */
  channel->IR.bit.RF0N = 1;
}

/** \brief Return baudrate settings value for selected baudrate
 *
 * \param [in] baudrate Baudrate in kbaud
 * \return Baudrate settings value as uint8_t
 *
 */
static uint8_t CAN_GetBitrate(uint32_t baudrate)
{
  uint8_t i;

  for (i = 0; i < sizeof(CAN_Bitrates) / sizeof(uint32_t); i++)
  {
    if (baudrate == CAN_Bitrates[i])
    {
      return i;
    }
  }

  return CAN_BITRATE_LAST;
}

/** \brief Set bitrate from raw parameter
 *
 * \param [in] n_br Nominal bitrate (from table)
 * \param [in] d_br Data bitrate (from table) (CAN FD only)
 * \return bool True if succeed
 *
 */
bool CAN_SetBitrateRaw(Can *channel, uint8_t n_br, uint8_t d_br)
{
  uint8_t brp;
  uint8_t tseg1;
  uint8_t tseg2;
  uint8_t sjw;

  if ((n_br >= CAN_BITRATE_LAST) || (d_br >= CAN_BITRATE_LAST))
  {
    return false;
  }

  /**< Disable CAN to allow changes */
  CAN_Disable(channel);
  switch (n_br)
  {
    case CAN_BITRATE_125K:
      brp = 8;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_250K:
      brp = 4;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_500K:
      brp = 2;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_1M:
      brp = 1;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_2M:
      brp = 1;
      tseg1 = 5;
      tseg2 = 2;
      sjw = 1;
      break;
    case CAN_BITRATE_4M:
      brp = 1;
      tseg1 = 2;
      tseg2 = 1;
      sjw = 1;
      break;
    default:
      return false;
  }
  /**< Nominal bit timing and prescaling */
  channel->NBTP.reg = CAN_NBTP_NBRP(brp - 1U) | CAN_NBTP_NTSEG1(tseg1 - 1U) | CAN_NBTP_NTSEG2(tseg2 - 1U) | CAN_NBTP_NSJW(sjw - 1U);
  switch (d_br)
  {
    case CAN_BITRATE_125K:
      brp = 8;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_250K:
      brp = 4;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_500K:
      brp = 2;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_1M:
      brp = 1;
      tseg1 = 11;
      tseg2 = 4;
      sjw = 1;
      break;
    case CAN_BITRATE_2M:
      brp = 1;
      tseg1 = 5;
      tseg2 = 2;
      sjw = 1;
      break;
    case CAN_BITRATE_4M:
      brp = 1;
      tseg1 = 2;
      tseg2 = 1;
      sjw = 1;
      break;
    default:
      return false;
  }
  /**< Data bit timing and prescaling */
  channel->DBTP.reg = CAN_DBTP_DBRP(brp - 1U) | CAN_DBTP_DTSEG1(tseg1 - 1U) | CAN_DBTP_DTSEG2(tseg2 - 1U) | CAN_DBTP_DSJW(sjw - 1U);
  /**< Enable CAN to apply changes */
  CAN_Enable(channel);

  return true;
}

/** \brief Set bitrate
 *
 * \param [in] nominal_br Nominal bitrate in kbaud
 * \param [in] data_br Data bitrate in kbaud (CAN FD only)
 * \return bool True if succeed
 *
 */
bool CAN_SetBitrate(Can *channel, uint32_t nominal_br, uint32_t data_br)
{
  uint8_t n_br;
  uint8_t d_br;

  n_br = CAN_GetBitrate(nominal_br);
  d_br = CAN_GetBitrate(data_br);

  return CAN_SetBitrateRaw(channel, n_br, d_br);
}

/** \brief Check if CAN bus is stopped and restore it
 *
 * \param [in] channel Number of the CAN channel
 * \return True if bus error occurred
 *
 */
bool CAN_CheckBus(Can *channel)
{
  uint32_t ir;

  ir = channel->IR.reg;
  if ((ir & CAN_IR_BO) > 0U)
  {
    /**< Bus_Off occurred, CAN should be reinitialized */
    channel->IR.reg = ir;
    CAN_Enable(channel);
    return true;
  }
  return false;
}

/** \brief Get error counters from CAN registers
 *
 * \param [in] channel Number of the CAN channel
 * \param [out] data Pointer to data structure with counters
 * \return void
 *
 */
void CAN_GetErrors(Can *channel, struct can_errors *data)
{
  data->rx_err_counter = channel->ECR.bit.REC;
  data->tx_err_counter = channel->ECR.bit.TEC;
  data->rx_err_passive = (channel->ECR.bit.RP > 0U);
}

/** \brief Enable CAN module, no parameters because of complexity
 *
 * \param [in] channel Number of the CAN channel
 * \return void
 *
 */
void CAN_Init(Can *channel)
{
  /**< Configure clock */
  if (channel == CAN0)
  {
    MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN0;
    GCLK->PCHCTRL[CAN0_GCLK_ID].reg = CAN_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
  } else
  #ifdef CAN1
  if (channel == CAN1)
  {
    MCLK->AHBMASK.reg |= MCLK_AHBMASK_CAN1;
    GCLK->PCHCTRL[CAN1_GCLK_ID].reg = CAN_GCLK_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos);
  } else
  #endif
  {
    /**< Non-available CAN connection */
    return;
  }

  /**< Put CAN to initializing mode */
  channel->CCCR.reg |= CAN_CCCR_INIT;
  while ((channel->CCCR.reg & CAN_CCCR_INIT) == 0) {}
  channel->CCCR.reg |= CAN_CCCR_CCE;

  /**< Common settings */
  channel->CCCR.reg |= CONF_CAN0_CCCR_REG;
  /**< Message RAM configuration (memory priority access during read/write operations) */
  channel->MRCFG.reg = CONF_CAN0_MRCFG_REG;
  /**< Nominal bit timing and prescaling */
  channel->NBTP.reg = CONF_CAN0_BTP_REG;
  /**< Data bit timing and prescaling */
  channel->DBTP.reg = CONF_CAN0_DBTP_REG;
  /**< Rx FIFO 0 Configuration */
  if (channel == CAN0)
  {
    channel->RXF0C.reg = CONF_CAN0_RXF0C_REG | CAN_RXF0C_F0SA((uint32_t)can_rx_fifo_0);
  } else
  {
    channel->RXF0C.reg = CONF_CAN0_RXF0C_REG | CAN_RXF0C_F0SA((uint32_t)can_rx_fifo_1);
  }
  /**< Rx Buffer / FIFO Element Size Configuration */
  channel->RXESC.reg = CONF_CAN0_RXESC_REG;
  /**< Tx Buffer Element Size Configuration */
  channel->TXESC.reg = CONF_CAN0_TXESC_REG;
  /**< Tx Buffer Configuration */
  if (channel == CAN0)
  {
    channel->TXBC.reg = CONF_CAN0_TXBC_REG | CAN_TXBC_TBSA((uint32_t)can_tx_fifo_0);
  } else
  {
    channel->TXBC.reg = CONF_CAN0_TXBC_REG | CAN_TXBC_TBSA((uint32_t)can_tx_fifo_1);
  }
  /**< Tx Event FIFO Configuration */
  channel->TXEFC.reg = CONF_CAN0_TXEFC_REG | CAN_TXEFC_EFSA((uint32_t)can_tx_event_fifo);
  /**< Global Filter Configuration */
  channel->GFC.reg = CONF_CAN0_GFC_REG;
  /**< Standard filter configuration */
  channel->SIDFC.reg = CONF_CAN0_SIDFC_REG | CAN_SIDFC_FLSSA((uint32_t)can_rx_std_filter);
  /**< Extended filter configuration */
  channel->XIDFC.reg = CONF_CAN0_XIDFC_REG | CAN_XIDFC_FLESA((uint32_t)can_rx_ext_filter);
  /**< Extended ID AND mask, should have all ones! */
  channel->XIDAM.reg = CONF_CAN0_XIDAM_REG;

  /**< Initialize counters for filters */
  can_std_filter_counter = 0;
  can_ext_filter_counter = 0;
}
