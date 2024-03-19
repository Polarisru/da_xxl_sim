#ifndef DEFINES_H
#define DEFINES_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/**< Platform-depended includes */
#ifndef DEF_SIM
  #include "samc21g18a.h"
  #include "err_codes.h"
#else
  #include <stdio.h>
#endif

#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

#define CONF_CPU_FREQUENCY      48000000UL

#define ADC_REF_VOLTAGE         4.096F
#define ADC_MAX_VALUE           4096

#define VOLTAGE_STEP            0.2F
#define CURRENT_STEP            0.05F

#define BL_ADDRESS              0x0000U
#define BL_SIZE                 0x2000U

#ifdef DEF_RELEASE
  #define APP_ADDRESS           BL_SIZE
#else
  #define APP_ADDRESS           0x0000U
#endif

enum COMM_TYPE
{
  COMM_TYPE_PWM,
  COMM_TYPE_RS485,
  COMM_TYPE_CAN,
  COMM_TYPE_LAST
};

#define MAX_TARGET_POS          1934                // 1934=+170°, -1934=-170° for 0x76 com (Cmd_Set_Pos)
#define MIN_TP_KF_TR            0x100U              // -100° for 0xDD com transformed according to Kearfott/Schiebel
#define MIN_TP_KF               0x80                // -100° for 0xDD com decoded
#define MAX_TP_KF_TR            0x1F00U             // +100° for 0xDD com transformed according to Kearfott/Schiebel
#define MAX_TP_KF               0x0F80U             // +100° for 0xDD com decoded
#define MIN_TP_EXT              0x0080U             // -100°
#define MAX_TP_EXT              0x0F80U             // +100°
#define MAX_MAGNET_DEVIATION    114U                // 10°, was not able to work with 5°
#define MAGNET_MAX_VALUE        0x1000U
#define MAGNET_MAX_ERRORS       10U                 // maximal number of consequent errors for magnet sensor
#define FRESHNESS_OFF_LEVEL     0x10U               // disabling level for a freshness counter
/**< Servo span value in ° */
#define SERVO_SPAN_VALUE        170
#define SERVO_MIN_TP            (int16_t)(-SERVO_SPAN_VALUE * 0x800 / 180)
#define SERVO_MAX_TP            (int16_t)(SERVO_SPAN_VALUE * 0x800 / 180)

/**< External IDs (1 - ACE1, 2 - ACE2) */
#define ID_ACE1                 1U
#define ID_ACE2                 2U

/**< Set of statuses for g_my_status */
#define STAT_ERROR_POSITION     (1UL << 0U)
#define STAT_ERROR_2OO3         (1UL << 1U)
#define STAT_ERROR_POWER        (1UL << 2U)
#define STAT_ERROR_FRESHNESS    (1UL << 3U)
#define STAT_ERROR_MOTOR_TEMP   (1UL << 4U)
#define STAT_ERROR_EEPROM       (1UL << 5U)
#define STAT_ERROR_INTEGRITY    (1UL << 6U)
#define STAT_ERROR_BLDC         (1UL << 7U)
#define STAT_ERROR_MAGNET       (1UL << 8U)
#define STAT_OTHER_ACE_MSTR_BIT (1UL << 9U)
#define STAT_DEGRADED_BIT       (1UL << 9U)
#define STAT_MOTOR_FREE_BIT     (1UL << 10U)
#define STAT_ERROR_LOSSCOMM     (1UL << 11U)
#define STAT_ERROR_NOINTCOMM    (1UL << 12U)
#define STAT_ERROR_HEARTBEAT    (1UL << 13U)
#define STAT_ERROR_ACE          (1UL << 14U)
#define STAT_MASTER_BIT         (1UL << 15U)

#ifdef DEF_RELEASE
  #define STAT_ERROR_MASK         (STAT_ERROR_MAGNET | STAT_ERROR_EEPROM | STAT_ERROR_2OO3)
  #define STAT_CRITICAL_MASK      (STAT_ERROR_INTEGRITY | STAT_ERROR_MOTOR_TEMP | STAT_ERROR_BLDC)
#else
  #define STAT_ERROR_MASK         (STAT_ERROR_MAGNET | STAT_ERROR_EEPROM | STAT_ERROR_2OO3)
  #define STAT_CRITICAL_MASK      (STAT_ERROR_MOTOR_TEMP | STAT_ERROR_BLDC)
#endif

#define STAT_NONCRITICAL_MASK       (STAT_ERROR_2OO3 | STAT_ERROR_EEPROM | STAT_ERROR_MAGNET | STAT_ERROR_FRESHNESS)
#define STAT_NONCRITICAL_MASK_ACE2  (STAT_ERROR_2OO3 | STAT_ERROR_EEPROM | STAT_ERROR_MAGNET | STAT_ERROR_FRESHNESS)

#define STAT_NO_PARTNER             (uint16_t)(0xFFFFU & (~STAT_ERROR_ACE))

/**< Extended status bits */
#define EXTSTAT_ERROR_LOSSCOMM2         (1UL << 0U)
#define EXTSTAT_ERROR_ACE2              (1UL << 1U)
#define EXTSTAT_ERROR_TEMP_SENSOR       (1UL << 2U)
#define EXTSTAT_ERROR_PCB_TEMP          (1UL << 3U)
#define EXTSTAT_MOTOR_SAVER_BIT         (1UL << 4U)
#define EXTSTAT_ERROR_ID_MISMATCH       (1UL << 5U)
#define EXTSTAT_ERROR_EXTHALL           (1UL << 6U)
#define EXTSTAT_ERROR_FREERUN           (1UL << 7U)
#define EXTSTAT_ERROR_RAM               (1UL << 8U)
#define EXTSTAT_ERROR_FLASH             (1UL << 9U)
#define EXTSTAT_DEGRADED_BIT            (1UL << 10U)
#define EXTSTAT_ACE2ACE_POS_ERROR       (1UL << 11U)
#define EXTSTAT_ACE2VOTING_POS_ERROR    (1UL << 12U)

/**< Set of old status bits */
#define OLDSTS_ERROR_TOTAL          (1UL << 7U)
#define OLDSTS_MASTER_BIT           (1UL << 6U)
#define OLDSTS_ERROR_FRESHNESS      (1UL << 5U)
#define OLDSTS_ERROR_LOSSCOMM       (1UL << 4U)
#define OLDSTS_ERROR_POWER          (1UL << 3U)
#define OLDSTS_ERROR_MOTORTEMP      (1UL << 2U)
#define OLDSTS_ERROR_NOINTCOMM      (1UL << 1U)
#define OLDSTS_ERROR_MAGNET         (1UL << 0U)

//#define OLDSTS_NO_PARTNER       (0xFF & ~OLDSTS_MASTER_BIT)
#define OLDSTS_NO_PARTNER           (0x00)

/**< VOLZ 11-bit Status byte: bits */
#define VOLZSTS_BIT_MOTOR_ERR       (1UL << 7U)
#define VOLZSTS_BIT_MAGNET_ERR      (1UL << 6U)
#define VOLZSTS_BIT_OVERCURR        (1UL << 5U)
#define VOLZSTS_BIT_UNDERVOLT       (1UL << 4U)
#define VOLZSTS_BIT_COMM_TIMEOUT    (1UL << 3U)
#define VOLZSTS_BIT_MOTORSAVER      (1UL << 2U)
#define VOLZSTS_BIT_SYSTEM_ERR      (1UL << 1U)
#define VOLZSTS_BIT_MOTOR_DIR       (1UL << 0U)

#define INV_SQRT2               70U / 99U
/**< Half-power for the motor: 255 / sqrt(2) */
//#define MOTOR_HALF_POWER        (255 * INV_SQRT2)

/**< Limits for I-component of PID-controller */
#define MOTOR_I_SUM_MAX         25500
#define MOTOR_I_SUM_MIN         -25500
#define MOTOR_I_MAX_ANGLE       10
#define MOTOR_I_MAX_SENS_ANGLE  (MOTOR_I_MAX_ANGLE * 0x800 / 180)

/**< Motor modes */
#define MOTOR_MODE_RUN1         0U       // motor is running with EEPROM power
#define MOTOR_MODE_RUN2         1U       // motor is running with half of EEPROM power
#define MOTOR_MODE_FREE         2U       // free running mode
#define MOTOR_MODE_BRAKE        3U       // motor brake (all windings closed to GND)
#define MOTOR_MODE_GOCW         4U       // permanent rotation forward (CW)
#define MOTOR_MODE_GOCCW        5U       // permanent rotation backward (CCW)

/**< Reset modes */
#define RESET_MODE_SINGLE       0U
#define RESET_MODE_ALL          1U

/**< Slave modes */
#define SLAVE_MODE_RUN1         0U
#define SLAVE_MODE_RUN2         1U
#define SLAVE_MODE_FREE         2U
#define SLAVE_MODE_BRAKE        3U

/**< Motor direction */
#define MOTOR_DIR_CW            0U
#define MOTOR_DIR_CCW           1U

/**< current_limiter */
#define REVERSAL                1U
#define NO_REVERSAL             0U
#define MAX_PWM                 255

/**< Bootloader section */
#define BL_SIGN1                0x12U
#define BL_SIGN2                0x34U
#define BL_START_SIGN           0xA5U
#define BL_STOP_SIGN            0x5AU

/**< DMA channels */
#define DMA_CHANNEL_INETWORK    0
#define DMA_CHANNEL_RS485       1
#define DMA_CHANNEL_RS485_2     2

#define STRINGS_MAX_LEN         32U

/**< Some signatures for communication protocol */
#define SIGN_ACCESS_1           0x41U
#define SIGN_ACCESS_2           0x3FU
#define SIGN_DFLT_1             0x41U
#define SIGN_DFLT_2             0x53U
#define SIGN_COMM_1             0xAAU
#define SIGN_COMM_2             0x55U

/**< Load counters */
enum {
  COUNTER_LOAD_ALL,
  COUNTER_LOAD_0,
  COUNTER_LOAD_25,
  COUNTER_LOAD_50,
  COUNTER_LOAD_75,
  COUNTER_LOAD_100,
  COUNTER_LOAD_LAST
};

/**< general actuator defines */
#define SUPPLY_VOLTAGE        28.0F   // Volt

#ifdef DEF_DA30
  /**< Current levels for load counters (based on 50mA per count) */
  #define CURR_LOAD_25          5U    // 0.26 A - counter 25%  ++
  #define CURR_LOAD_50          10U   // 0.52 A - counter 50%  ++
  #define CURR_LOAD_75          21U   // 1.04 A - counter 75%  ++
  #define CURR_LOAD_100         42U   // 2.08 A  - counter 100% ++
  /**< Parameters for stall event and current limiter (DA 58-D)*/
  #define MOTOR_IMPEDANCE       (int32_t)(7.78F * 100)   // Ohm (28V / 3,6 A)
  #define VOLTAGE_CONSTANT      (int32_t)(0.538F * 10000)  // Voltage / no-load-speed ( actuator units)
#endif

#ifdef DEF_DA30HT
  /**< Current levels for load counters (based on 50mA per count) */
  #define CURR_LOAD_25          7U    // 0.36 A - counter 25%  ++
  #define CURR_LOAD_50          14U   // 0.72 A - counter 50%  ++
  #define CURR_LOAD_75          29U   // 1.44 A - counter 75%  ++
  #define CURR_LOAD_100         58U   // 2.88 A  - counter 100% ++
  /**< Parameters for stall event and current limiter (DA 58-D)*/
  #define MOTOR_IMPEDANCE       (int32_t)(5.83F * 100)   // Ohm (28V / 4.8 A)
  #define VOLTAGE_CONSTANT      (int32_t)(0.737F * 10000)  // Voltage / no-load-speed ( actuator units)
#endif

#ifdef DEF_DA58
  /**< Current levels for load counters (based on 50mA per count) */
  #define CURR_LOAD_25          19U    // 1.0 A - counter 25%  ++
  #define CURR_LOAD_50          38U    // 1.9 A  - counter 50%  ++
  #define CURR_LOAD_75          76U    // 3.8 A  - counter 75%  ++
  #define CURR_LOAD_100         152U   // 7.6 A  - counter 100% ++
  /**< Parameters for stall event and current limiter (DA 58-D)*/
  #define MOTOR_IMPEDANCE       (int32_t)(1.233F * 100)   // Ohm (28V / 22.7 A)
  #define VOLTAGE_CONSTANT      (int32_t)(0.933F * 10000)  // Voltage / no-load-speed ( actuator units)
#endif

/**< general stall event counter defines*/
#define STALL_TIME            100U  // 1.0 s (timeslot=10mS)
#define STALL_DIFF_POS        30    // ~3°   value must be compared with position error
#define STALL_CURR            CURR_LOAD_100   // = 100% current current

/**< Sensor calibration counter value */
#define SENS_CALIBR_VALUE     90U

#define TIMEOUT_PARTNER_MS         100U
#define TIMEOUT_PARTNER_SHORT_MS   10U
#define TIMEOUT_HALL_MS            100U

#define MAGNET_MAX_VALUE_DIV2   (MAGNET_MAX_VALUE / 2U)

/**< Options bits */
/**< Bit 0: anti backlash option enabled (both BLDCs getting PWM_min value in opposite directions) */
#define OPTIONS_ANTI_BACKLASH    0x01U
/**< Bit 1: disable role switching (master to slave) if active */
#define OPTIONS_DISABLE_ROLES    0x02U
/**< Bit 2: if PWM is lower than dead-band motor rotates free if active, otherwise motor does brake */
#define OPTIONS_PD0_BEHAVIOR     0x04U
//#define OPTIONS_ACTUATOR_CURR_LIMIT     0x08U    // option mask 4 in this bit: true ==> switches the current limiter from MOTOR current limit to ACTUATOR current limit

/**< Options#2 bits */
#define OPTIONS2_BIT_TE         0x01U

/**< Voltage limit value */
#define VOLTAGE_OK              110U   // *0.2V = 22V (MJ, 14.03.2023 aligned with DO-160)
#define VOLTAGE_ZERO            40U    // *0.2V = 8V

/**< Maximal number of power errors to set error bit */
#define POWER_FAIL_MAX          10U

#ifdef DEF_OLD_DA58
  #define LED_PORT            GPIO_PORTB
  #define LED_PIN             22
#else
  #define LED_PORT            GPIO_PORTA
  #define LED_PIN             0
#endif

/**< Heater constants */
#define HEATER_GAIN_COEFF       30
#define HEATER_MAX_PWM          120

/**< Position error */
#define POS_ERROR_TIME_RES_MS   50U

/**< Forcing to change role */
enum {
  ROLE_FORCE_NONE,
  ROLE_FORCE_MASTER,
  ROLE_FORCE_SLAVE,
  ROLE_FORCE_OPPOSITE
};

/**< Repeat ROLE_FORCE_OPPOSITE several times */
#define FORCE_COUNT_MAX     5U

#ifndef DEF_DUPLEX
  #define DEF_DUPLEX_COMM
#endif

/**< List of types for parameter value */
enum {
  PARAM_TYPE_EMPTY,
  PARAM_TYPE_BOOL,
  PARAM_TYPE_UINT8,
  PARAM_TYPE_INT8,
  PARAM_TYPE_UINT16,
  PARAM_TYPE_INT16,
  PARAM_TYPE_UINT32,
  PARAM_TYPE_INT32,
  PARAM_TYPE_FLOAT16,
  PARAM_TYPE_FLOAT32
};

/**< List of channels for internal bus */
enum {
  INET_VOTING_CHANNEL1,
  INET_VOTING_CHANNEL2,
  INET_VOTING_LAST
};

/**< Length of parameter's name */
#define PARAM_NAME_LEN            6U

#endif
