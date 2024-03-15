#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "defines.h"
#include "err_codes.h"
#include "can_structs.h"

bool CAN_HasMessage(Can *channel);
int32_t CAN_ReadMsg(Can *channel, struct can_message *msg);
int32_t CAN_WriteMsg(Can *channel, struct can_message *msg);
int32_t CAN_SetFilter(enum can_format fmt, struct can_filter *filter);
int32_t CAN_SetRangeFilter(enum can_format fmt, uint32_t start_id, uint32_t stop_id);
void CAN_ResetFilters(enum can_format fmt);
void CAN_Enable(Can *channel);
void CAN_Disable(Can *channel);
bool CAN_SetBitrateRaw(Can *channel, uint8_t nominal_br, uint8_t data_br);
bool CAN_SetBitrate(Can *channel, uint32_t nominal_br, uint32_t data_br);
void CAN_SetFD(Can *channel, bool on);
void CAN_SetBRS(Can *channel, bool on);
bool CAN_CheckBus(Can *channel);
void CAN_GetErrors(Can *channel, struct can_errors *data);
void CAN_Init(Can *channel);

#endif

