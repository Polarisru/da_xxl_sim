#ifndef RS485_H
#define RS485_H

#include "defines.h"

#define RS485_PORT          GPIO_PORTA
#define RS485_TX_PORT       GPIO_PORTA

#define RS485_PIN_DIR       10
#define RS485_PIN_TX        12
#define RS485_PIN_RX        11
#define RS485_PIN_TE        8
//#ifdef DEF_DUPLEX
//  #define RS485_PIN_TE        8
//#else
//  #define RS485_PIN_TE        0
//#endif
#define RS485_MUX_DIR       MUX_PA10D_SERCOM2_PAD2
#define RS485_MUX_RX        MUX_PA11D_SERCOM2_PAD3
#define RS485_MUX_TX        MUX_PA12C_SERCOM2_PAD0

#define RS485_CHANNEL       SERCOM2
#define RS485_IRQ_Handler   SERCOM2_Handler
#define RS485_IRQn          SERCOM2_IRQn

#define RS485_RXPO          3
#define RS485_TXPO          3

#define RS485_DMA_TRIG      SERCOM2_DMAC_ID_TX

#define RS485_BAUDRATE      115200UL

#ifndef DEF_DUPLEX
  #define RS485_2_PORT          GPIO_PORTA
  #define RS485_2_TX_PORT       GPIO_PORTA

  #define RS485_2_PIN_DIR       6
  #define RS485_2_PIN_TX        4
  #define RS485_2_PIN_RX        7

  #define RS485_2_MUX_DIR       MUX_PA06D_SERCOM0_PAD2
  #define RS485_2_MUX_RX        MUX_PA07D_SERCOM0_PAD3
  #define RS485_2_MUX_TX        MUX_PA04D_SERCOM0_PAD0

  #define RS485_2_CHANNEL       SERCOM0
  #define RS485_2_IRQ_Handler   SERCOM0_Handler
  #define RS485_2_IRQn          SERCOM0_IRQn

  #define RS485_2_RXPO          3
  #define RS485_2_TXPO          3

  #define RS485_2_DMA_TRIG      SERCOM0_DMAC_ID_TX
#endif

#define RS485_TIMEOUT           15U
#define RS485_BLDC_TIMEOUT      100U

#define RS485_STD_PACKET_LEN    6U

#define RS485_RX_PACKET_LEN     6U
#define RS485_TX_PACKET_LEN     6U

#define RS485_PACKET_CMD_POS    0U
#define RS485_PACKET_ID_POS     1U
#define RS485_PACKET_ARG1_POS   2U
#define RS485_PACKET_ARG2_POS   3U

#define RS485_STD_PACKET_CRC_POS  (RS485_STD_PACKET_LEN - sizeof(uint16_t))
#define RS485_RX_PACKET_CRC_POS   (RS485_RX_PACKET_LEN - sizeof(uint16_t))
#define RS485_TX_PACKET_CRC_POS   (RS485_TX_PACKET_LEN - sizeof(uint16_t))

#define RS485_BROADCAST_ID      0x1FU
#define RS485_DEFAULT_ID        1U

/**
 * \brief RS485 baudrates
 */
enum {
  RS485_BAUDRATE_9600 = 0U,
  RS485_BAUDRATE_19200,
  RS485_BAUDRATE_38400,
  RS485_BAUDRATE_57600,
  RS485_BAUDRATE_115200,
  RS485_BAUDRATE_230400,
  RS485_BAUDRATE_250000,
  RS485_BAUDRATE_LAST
};

/**< The queue is to be created to hold a maximum of 10 RS485 packets */
//#define RS485_QUEUE_LENGTH      10
//#define RS485_ITEM_SIZE         sizeof(uint8_t) * RS485_PACKET_LEN

extern uint8_t RS485_RxBuffer[RS485_RX_PACKET_LEN];
extern uint8_t RS485_TxBuffer[RS485_TX_PACKET_LEN];
#ifdef DEF_DUPLEX_COMM
extern uint8_t RS485_2_RxBuffer[RS485_RX_PACKET_LEN];
extern uint8_t RS485_2_TxBuffer[RS485_TX_PACKET_LEN];
#endif

extern void RS485_IRQ_Handler(void);
#ifndef DEF_DUPLEX
  extern void RS485_2_IRQ_Handler(void);
#endif
extern void RS485_Configuration(uint8_t baudrate);
extern void RS485_SwitchTermination(bool on);
extern void RS485_SendPacket(const uint8_t *data);
extern bool RS485_ReceivePacket(uint8_t *data, uint16_t timeout);
extern void RS485_Disable(void);

#endif

