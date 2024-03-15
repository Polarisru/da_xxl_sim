#ifndef INETWORK_H
#define INETWORK_H

#include "defines.h"

#define INET_TIMEOUT_1            3U
#define INET_TIMEOUT_2            4U

/**< Internal network packet settings */
#define INET_PACKET_LEN           9U
#define INET_PACKET_SIGN_POS      0U
#define INET_PACKET_ID_POS        1U
#define INET_PACKET_CRC_POS       (INET_PACKET_LEN - sizeof(uint16_t))
#define INET_PACKET_SIGN          0xA5U

#define INETWORK_BAUDRATE         250000UL

#define INETWORK_CHANNEL          SERCOM0
#define INETWORK_IRQ_Handler      SERCOM0_Handler
#define INETWORK_IRQn             SERCOM0_IRQn

/**< Internal RS485 pins */
#define INETWORK_PORT             GPIO_PORTA

#define INETWORK_PIN_DIR          6
#define INETWORK_PIN_TX           4
#define INETWORK_PIN_RX           7
//#define INETWORK_PIN_TE           0

#define INETWORK_MUX_DIR          MUX_PA06D_SERCOM0_PAD2
#define INETWORK_MUX_RX           MUX_PA07D_SERCOM0_PAD3
#define INETWORK_MUX_TX           MUX_PA04D_SERCOM0_PAD0

#define INETWORK_RXPO             3
#define INETWORK_TXPO             3

#define INETWORK_DMA_TRIG         SERCOM0_DMAC_ID_TX

/**< Internal network IDs */
#define MASK_TN                   0xC0U
#define HALL_TN                   0xC0U
#define EHALL_TN                  0x00U
#define ACE1_TN                   0x40U
#define ACE2_TN                   0x80U

/**< Signal to partner make reset all status bytes */
#define INET_BE_DFLT              0xA5U

/**< Internal network command set by number of freshness counter */
#define ICMD_U    0x00U    // returns to partner ACE 2 voltages
#define ICMD_V1   0x01U    // raw target position #1 from partner ACE
#define ICMD_T    0x02U    // returns to partner ACE 2 temperatures
#define ICMD_I    0x04U    // returns to partner ACE supply current and RH
#define ICMD_PWR  0x05U    // returns PWM value GLOBAL_PartnerMaxPwmValue
#define ICMD_S    0x06U    // returns to partner ACE own status information
#define ICMD_V2   0x07U    // raw target position #2 from partner ACE
#define ICMD_P    0x08U    // returns to partner ACE own TargetPos
#define ICMD_RST  0x09U    // raw reset request from partner ACE
#define ICMD_N    0x0AU    // signal  to partner reset statuses to default
#define ICMD_CRC  0x0BU    // CRC16 of the EEPROM
#define ICMD_A    0x0CU    // status byte of partner ACE for external control
#define ICMD_D    0x0DU    // send SID (CAN identifier)
#define ICMD_C    0x0EU    // calibrate all other sensors with our value
#define ICMD_BL   0x0FU    // go to bootloader

/**< Bits for raw target positions from partner ACE */
#define INETWORK_VTARGET_BIT_MSGVALID   (1UL << 15U)
#define INETWORK_VTARGET_BIT_VALID      (1UL << 14U)
#define INETWORK_VTARGET_BIT_SHDN       (1UL << 13U)
#define INETWORK_VTARGET_BIT_ARMED      (1UL << 12U)
#define INETWORK_VTARGET_POS_MASK       (0x0FFFU)

/**< Bits for reset request from partner ACE */
#define INETWORK_VRESET_BIT_MSGVALID    (1UL << 2U)
#define INETWORK_VRESET_BIT_VALID       (1UL << 1U)
#define INETWORK_VRESET_BIT_RESET       (1UL << 0U)

#ifdef DEF_DUPLEX
  extern void INETWORK_IRQ_Handler(void);
#endif
extern void INETWORK_ParsePacket(uint8_t *data);
extern void INETWORK_BuildPacket(uint8_t id, uint8_t *data);
extern void INETWORK_Task(void *pParameters);
extern void INETWORK_Activate(bool on);
extern void INETWORK_ForceRole(bool on);

#endif
