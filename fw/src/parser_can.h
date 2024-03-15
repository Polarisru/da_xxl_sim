#ifndef PARSER_CAN_H
#define PARSER_CAN_H

#include "defines.h"
#include "canbus.h"

extern bool PARSER_CanVolz(TCanMsg *rx_msg, struct can_message *tx_msg);
extern void PARSER_CanVerticalEmpty(void);
extern bool PARSER_CanVertical(TCanMsg *rx_msg, struct can_message *tx_msg);
extern void PARSER_CanPipistrelEmpty(void);
extern bool PARSER_CanPipistrel(TCanMsg *rx_msg, struct can_message *tx_msg);

#endif
