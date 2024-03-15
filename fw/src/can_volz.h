#ifndef CAN_VOLZ_H
#define CAN_VOLZ_H

/**< Masks and default CAN IDs */
#define CANBUS_ID_MASK            0x7FFU
#define CANBUS_BACKDOOR_ID        0x7F0U
#define CANBUS_ID_OWN_MASK        0x7E0U
#define CANBUS_DFLT_ID            0x3E0U

#define CANBUS_ID_REPLY_BIT       (1UL << 4U)

#define CANMSG_OFFSET_CMD         0U
#define CANMSG_OFFSET_DATA        1U

/**< Command set */
#define CANMSG_CMD_ECHO           0x00U
#define CANMSG_CMD_SET_POS        0x01U
#define CANMSG_CMD_GET_POS        0x02U
#define CANMSG_CMD_GET_STAT       0x03U
#define CANMSG_CMD_GET_TIME       0x04U
#define CANMSG_CMD_GET_PARAM      0x05U
#define CANMSG_CMD_SET_PARAM      0x06U
#define CANMSG_CMD_SET_GROUP_POS  0x07U
#define CANMSG_CMD_GET_HWFW       0x08U
#define CANMSG_CMD_GET_STRING     0x09U
#define CANMSG_CMD_SET_STRING     0x0AU
#define CANMSG_CMD_EXECUTE        0x0BU
#define CANMSG_CMD_DUPLEX         0x0CU
#define CANMSG_CMD_GET_COUNTER    0x0DU
#define CANMSG_CMD_GET_REVISION   0x0EU
#define CANMSG_CMD_READ_EE        0x3DU
#define CANMSG_CMD_WRITE_EE       0x3EU
#define CANMSG_CMD_BOOTLOADER     0x3FU

/**< Structure of the CAN ID */
#define CANMSG_ID_WIDTH           4U
#define CANMSG_ID_MASK            ((1UL << CANMSG_ID_WIDTH) - 1U)
#define CANMSG_ID_BROADCAST       0x00U

/**< Structure of the command byte */
//#define CANMSG_CMD_REPLY_BIT      (1UL << 7U)
#define CANMSG_CMD_SILENT_BIT     (1UL << 6U)
#define CANMSG_CMD_WIDTH          6U
#define CANMSG_CMD_MASK           ((1UL << CANMSG_CMD_WIDTH) - 1U)

#define FLASH_CMD_STARTBL         0x00U

/**< Structure of the CAN data packet */
#define CANMSG_PARAM_ID_OFFS      0U
#define CANMSG_PARAM_CMD_OFFS     1U
#define CANMSG_PARAM_DATA_OFFS    2U

#define CANMSG_PARAM_ID_WIDTH     7U
#define CANMSG_PARAM_ID_MASK      ((1UL << CANMSG_PARAM_ID_WIDTH) - 1U)

#define CANMSG_PARAM_TYPE_OFFS    0U
#define CANMSG_PARAM_TYPE_WIDTH   4U
#define CANMSG_PARAM_TYPE_MASK    ((1UL << CANMSG_PARAM_TYPE_WIDTH) - 1U)
#define CANMSG_PARAM_OPCODE_OFFS  4U
#define CANMSG_PARAM_OPCODE_WIDTH 2U
#define CANMSG_PARAM_OPCODE_MASK  ((1UL << CANMSG_PARAM_OPCODE_WIDTH) - 1U)

#define CANMSG_PARAM_VAL          0U
#define CANMSG_PARAM_MINVAL       1U
#define CANMSG_PARAM_MAXVAL       2U
#define CANMSG_PARAM_DFLTVAL      3U

#define CANMSG_PARAM_NAME_BIT     (1UL << 7U)

/**< Opcodes for Execute command */
#define CANMSG_EXECUTE_NORMMODE   0U
#define CANMSG_EXECUTE_FREE       1U
#define CANMSG_EXECUTE_ROTCW      2U
#define CANMSG_EXECUTE_ROTCCW     3U
#define CANMSG_EXECUTE_NEUTRAL    4U
#define CANMSG_EXECUTE_SAVESTR    5U
#define CANMSG_EXECUTE_RESETSTS   6U
#define CANMSG_EXECUTE_MOTORTEST  7U
/**< Passphrase for critical commands */
#define CANMSG_EXECUTE_PASS       0x11223344U

/**< Replies for Execute/Duplex command */
#define CANMSG_REPLY_OK           0xAAU
#define CANMSG_REPLY_ERR          0xEEU

/**< Opcodes for Duplex command */
#define CANMSG_DUPLEX_FORCEMASTER 0U
#define CANMSG_DUPLEX_FORCESLAVE  1U

/**< Signature to start the bootloader */
#define CANMSG_BL_SIGN_START      0xAA55A55AU

#endif
