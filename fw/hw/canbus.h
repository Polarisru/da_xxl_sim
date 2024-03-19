#ifndef CANBUS_H
#define CANBUS_H

#include "defines.h"
#include "can_structs.h"

#define CANBUS_BUF_LEN    8U

#define CAN1_PORT          GPIO_PORTB
#define CAN1_PIN_RX        11
#define CAN1_PIN_TX        10

#define CAN1_MUX_RX        MUX_PB11G_CAN1_RX
#define CAN1_MUX_TX        MUX_PB10G_CAN1_TX

#define CAN1_NUM           CAN1
#define CAN1_IRQ_Handler   CAN1_Handler
#define CAN1_IRQn          CAN1_IRQn

#ifdef DEF_DUPLEX_COMM
  #define CAN2_PORT          GPIO_PORTB
  #define CAN2_PIN_RX        23
  #define CAN2_PIN_TX        22

  #define CAN2_MUX_RX        MUX_PB23G_CAN0_RX
  #define CAN2_MUX_TX        MUX_PB22G_CAN0_TX

  #define CAN2_NUM           CAN0
  #define CAN2_IRQ_Handler   CAN0_Handler
  #define CAN2_IRQn          CAN0_IRQn
#endif

typedef struct
{
  uint32_t  id;
  uint8_t   len;
  uint8_t   data[CANBUS_BUF_LEN];
  enum can_format fmt;
  enum can_channel channel;
} TCanMsg;

void CAN1_IRQ_Handler(void);
void CAN2_IRQ_Handler(void);
bool CANBUS_Send(enum can_channel channel, struct can_message *msg);
void CANBUS_Configuration(uint8_t bitrate);
void CANBUS_Disable(void);
void CANBUS_Enable(void);

#endif
