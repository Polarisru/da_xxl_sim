#ifndef DRV8320_H
#define DRV8320_H

#define DRV8320_WRITE_BIT     0x0000U
#define DRV8320_READ_BIT      0x8000U

#define DRV8320_ADDR_OFFSET   11U
#define DRV8320_OUT_LENGTH    ((sizeof(uint16_t) * 8U) - DRV8320_ADDR_OFFSET)
#define DRV8320_ADDR_MASK     0x07UL

#define DRV8320_REG_FAULT1    0x00U
#define DRV8320_REG_FAULT2    0x01U
#define DRV8320_REG_CTRL      0x02U
#define DRV8320_REG_DRV_HS    0x03U
#define DRV8320_REG_DRV_LS    0x04U
#define DRV8320_REG_OCP       0x05U
#define DRV8320_REG_CSA       0x06U

#define DRV8320_REG_VALUE(n)  (((n) & DRV8320_ADDR_MASK) << DRV8320_ADDR_OFFSET)

#define DRV8320_DATA_LENGTH   DRV8320_ADDR_OFFSET
#define DRV8320_VALUE_MASK    ((1UL << DRV8320_DATA_LENGTH) - 1U)

// Logic OR of FAULT status registers. Mirrors nFAULT pin
#define DRV8320_FAULT1_BIT_FAULT    (1UL << 10U)
// Indicates VDS monitor overcurrent fault condition
#define DRV8320_FAULT1_BIT_VDS_OCP  (1UL << 9U)
// Indicates gate drive fault condition
#define DRV8320_FAULT1_BIT_GDF      (1UL << 8U)
// Indicates undervoltage lockout fault condition
#define DRV8320_FAULT1_BIT_UVLO     (1UL << 7U)
// Indicates overtemperature shutdown
#define DRV8320_FAULT1_BIT_OTSD     (1UL << 6U)
// Indicates VDS overcurrent fault on the A high-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_HA   (1UL << 5U)
// Indicates VDS overcurrent fault on the A low-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_LA   (1UL << 4U)
// Indicates VDS overcurrent fault on the B high-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_HB   (1UL << 3U)
// Indicates VDS overcurrent fault on the B low-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_LB   (1UL << 2U)
// Indicates VDS overcurrent fault on the C high-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_HC   (1UL << 1U)
// Indicates VDS overcurrent fault on the C low-side MOSFET
#define DRV8320_FAULT1_BIT_VDS_LC   (1UL << 0U)

// Indicates overcurrent on phase A sense amplifier
#define DRV8320_FAULT2_BIT_SA_OC    (1UL << 10U)
// Indicates overcurrent on phase B sense amplifier
#define DRV8320_FAULT2_BIT_SB_OC    (1UL << 9U)
// Indicates overcurrent on phase C sense amplifier
#define DRV8320_FAULT2_BIT_SC_OC    (1UL << 8U)
// Indicates overtemperature warning
#define DRV8320_FAULT2_BIT_OTW      (1UL << 7U)
// Indicates charge pump undervoltage fault condition
#define DRV8320_FAULT2_BIT_CPUV     (1UL << 6U)
// Indicates gate drive fault on the A high-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_HA   (1UL << 5U)
// Indicates gate drive fault on the A low-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_LA   (1UL << 4U)
// Indicates gate drive fault on the B high-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_HB   (1UL << 3U)
// Indicates gate drive fault on the B low-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_LB   (1UL << 2U)
// Indicates gate drive fault on the C high-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_HC   (1UL << 1U)
// Indicates gate drive fault on the C low-side MOSFET
#define DRV8320_FAULT2_BIT_VGS_LC   (1UL << 0U)

/**< PWM modes */
#define DRV8320_MODE_6X_PWM         0U
#define DRV8320_MODE_3X_PWM         1U
#define DRV8320_MODE_1X_PWM         2U
#define DRV8320_MODE_IND_PWM        3U

// 0b = Charge pump UVLO fault is enabled
// 1b = Charge pump UVLO fault is disabled
#define DRV8320_CTRL_BIT_DIS_CPUV   (1UL << 9U)
// 0b = Gate drive fault is enabled
// 1b = Gate drive fault is disabled
#define DRV8320_CTRL_BIT_DIS_GDF    (1UL << 8U)
// 0b = OTW is not reported on nFAULT or the FAULT bit
// 1b = OTW is reported on nFAULT and the FAULT bit
#define DRV8320_CTRL_BIT_OTW_REP    (1UL << 7U)
// 00b = 6x PWM Mode
// 01b = 3x PWM mode
// 10b = 1x PWM mode
// 11b = Independent PWM mode
#define DRV8320_CTRL_BIT_PWM_MODE(n)  (((n) & 0x03U) << 5U)
// 0b = 1x PWM mode uses synchronous rectification
// 1b = 1x PWM mode uses asynchronous rectification (diode freewheeling)
#define DRV8320_CTRL_BIT_1PWM_COM   (1UL << 4U)
//In 1x PWM mode this bit is ORed with the INHC (DIR) input
#define DRV8320_CTRL_BIT_1PWM_DIR   (1UL << 3U)
// Write a 1 to this bit to put all MOSFETs in the Hi-Z state
#define DRV8320_CTRL_BIT_COAST      (1UL << 2U)
// Write a 1 to this bit to turn on all three low-side MOSFETs in 1x PWM mode
// This bit is ORed with the INLC (BRAKE) input
#define DRV8320_CTRL_BIT_BRAKE      (1UL << 1U)
// Write a 1 to this bit to clear latched fault bits
// This bit automatically resets after being written
#define DRV8320_CTRL_BIT_CLR_FLT    (1UL << 0U)

// Write 110b to lock the settings by ignoring further register writes
// except to these bits and address 0x02 bits 0-2.
// Writing any sequence other than 110b has no effect when unlocked.
// Write 011b to this register to unlock all registers.
// Writing any sequence other than 011b has no effect when locked.
#define DRV8320_DRV_HS_BIT_LOCK     8
// 0000b = 10 mA
// 0001b = 30 mA
// 0010b = 60 mA
// 0011b = 80 mA
// 0100b = 120 mA
// 0101b = 140 mA
// 0110b = 170 mA
// 0111b = 190 mA
// 1000b = 260 mA
// 1001b = 330 mA
// 1010b = 370 mA
// 1011b = 440 mA
// 1100b = 570 mA
// 1101b = 680 mA
// 1110b = 820 mA
// 1111b = 1000 mA
#define DRV8320_DRV_HS_BIT_IDRIVEP_HS   4
// 0000b = 20 mA
// 0001b = 60 mA
// 0010b = 120 mA
// 0011b = 160 mA
// 0100b = 240 mA
// 0101b = 280 mA
// 0110b = 340 mA
// 0111b = 380 mA
// 1000b = 520 mA
// 1001b = 660 mA
// 1010b = 740 mA
// 1011b = 880 mA
// 1100b = 1140 mA
// 1101b = 1360 mA
// 1110b = 1640 mA
// 1111b = 2000 mA
#define DRV8320_DRV_HS_BIT_IDRIVEN_HS   0

// Cycle-by cycle operation. In retry OCP_MODE, for both
// VDS_OCP and SEN_OCP, the fault is automatically cleared
// when a PWM input is given
#define DRV8320_DRV_LS_BIT_CBC       10
// 00b = 500-ns peak gate-current drive time
// 01b = 1000-ns peak gate-current drive time
// 10b = 2000-ns peak gate-current drive time
// 11b = 4000-ns peak gate-current drive time
#define DRV8320_DRV_LS_BIT_TDRIVE    8
// 0000b = 10 mA
// 0001b = 30 mA
// 0010b = 60 mA
// 0011b = 80 mA
// 0100b = 120 mA
// 0101b = 140 mA
// 0110b = 170 mA
// 0111b = 190 mA
// 1000b = 260 mA
// 1001b = 330 mA
// 1010b = 370 mA
// 1011b = 440 mA
// 1100b = 570 mA
// 1101b = 680 mA
// 1110b = 820 mA
// 1111b = 1000 mA
#define DRV8320_DRV_LS_BIT_IDRIVEP_LS    4
// 0000b = 20 mA
// 0001b = 60 mA
// 0010b = 120 mA
// 0011b = 160 mA
// 0100b = 240 mA
// 0101b = 280 mA
// 0110b = 340 mA
// 0111b = 380 mA
// 1000b = 520 mA
// 1001b = 660 mA
// 1010b = 740 mA
// 1011b = 880 mA
// 1100b = 1140 mA
// 1101b = 1360 mA
// 1110b = 1640 mA
// 1111b = 2000 mA
#define DRV8320_DRV_LS_BIT_IDRIVEN_LS    0

/**< Error bits */
#define DRV8320_ERR_OVC_HIGHA     (1UL << 0U)
#define DRV8320_ERR_OVC_LOWA      (1UL << 1U)
#define DRV8320_ERR_OVC_HIGHB     (1UL << 2U)
#define DRV8320_ERR_OVC_LOWB      (1UL << 3U)
#define DRV8320_ERR_OVC_HIGHC     (1UL << 4U)
#define DRV8320_ERR_OVC_LOWC      (1UL << 5U)
#define DRV8320_ERR_GATE_HIGHA    (1UL << 6U)
#define DRV8320_ERR_GATE_LOWA     (1UL << 7U)
#define DRV8320_ERR_GATE_HIGHB    (1UL << 8U)
#define DRV8320_ERR_GATE_LOWB     (1UL << 9U)
#define DRV8320_ERR_GATE_HIGHC    (1UL << 10U)
#define DRV8320_ERR_GATE_LOWC     (1UL << 11U)
#define DRV8320_ERR_UNDERVOLT     (1UL << 12U)
#define DRV8320_ERR_OVERTEMP      (1UL << 13U)

#define DRV8320_CTRL_INIT_VALUE   (DRV8320_CTRL_BIT_DIS_CPUV | DRV8320_CTRL_BIT_DIS_GDF | \
                                  DRV8320_CTRL_BIT_OTW_REP | DRV8320_CTRL_BIT_PWM_MODE(DRV8320_MODE_1X_PWM))

#define DRV8320_HS_INIT_VALUE     (3UL << DRV8320_DRV_HS_BIT_IDRIVEP_HS) | (3UL << DRV8320_DRV_HS_BIT_IDRIVEN_HS)
#define DRV8320_LS_INIT_VALUE     (3UL << DRV8320_DRV_LS_BIT_IDRIVEP_LS) | (3UL << DRV8320_DRV_LS_BIT_IDRIVEN_LS)

void DRV8320_Enable(bool on);
void DRV8320_EnableToggle(void);
void DRV8320_DoBrake(bool on);
void DRV8320_SetDir(uint8_t dir);
void DRV8320_DoFree(bool on);
void DRV8320_SwitchToNormal(void);
uint16_t DRV8320_ReadError(void);
void DRV8320_Activate(void);
void DRV8320_Configuration(void);

#endif
