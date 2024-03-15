#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include "cmocka.h"
#include <stdio.h>

#include "can_structs.h"
#include "can_volz.h"
#include "cmd_set.h"
#include "conversion.h"
#include "counters.h"
#include "crc16.h"
#include "crc32.h"
#include "eeprom.h"
#include "global.h"
#include "hall.h"
#include "hdc1080.h"
#include "inetwork.h"
#include "logic.h"
#include "magnet.h"
#include "monitor.h"
#include "motor.h"
#include "mvs.h"
#include "notch.h"
#include "parser_can.h"
#include "parser_rs485.h"
#include "redundancy.h"
#include "rs485.h"
#include "timeouts.h"
#include "utils.h"
#include "voting.h"
#include "version.h"

uint32_t crc32_build;

extern void EEPROM_LoadCopy(uint8_t ee_copy);
extern void EEPROM_LoadStrings(void);
extern void EEPROM_LoadCounters(void);
extern void EEPROM_SaveCopy(uint8_t ee_copy);
extern void EEPROM_SaveStrings(void);
extern void EEPROM_SaveCounters(void);
extern void FLASH_EraseRowEE(uint32_t *dst);
extern void FLASH_WriteWordsEE(uint32_t *dst, uint32_t *src, uint32_t n_words);
extern void MOTOR_DoTest(uint8_t *resp_1, uint8_t *resp_2);
extern void DRV8320_DoBrake(bool on);
extern void DRV8320_DoFree(bool on);
extern void DRV8320_SetDir(bool dir);

#define array_length(x) (sizeof(x) / sizeof((x)[0]))

/**< Convert SetPos170 command to position value */
static void test_conversion_setpos170(void **state)
{
  (void) state;

  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x00, 0x00), 0);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x10, 0x00), 0);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x02, 0x00), 0x200);  // 45�
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x0E, 0x00), -0x200); // -45�
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x07, 0xFF), 1934);   // apr. 170�
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x08, 0x00), -1934);  // apr. -170�
}

/**< Convert SetPos100 command to position value and reverse */
static void test_conversion_setpos100(void **state)
{
  (void) state;

  /**< Convert from */
  assert_int_equal(CONVERSION_CalcPosFromSetPos100(0x10, 0x00), 0);      // 0�
  assert_int_equal(CONVERSION_CalcPosFromSetPos100(0x16, 0x60), 0x200);  // 45�
  assert_int_equal(CONVERSION_CalcPosFromSetPos100(0x09, 0x20), -0x200); // -45�
  assert_int_equal(CONVERSION_CalcPosFromSetPos100(0x1D, 0x3F), 0x400);  // 90�
  assert_int_equal(CONVERSION_CalcPosFromSetPos100(0x02, 0x41), -0x400); // -90�
  /**< Convert to */
  assert_int_equal(CONVERSION_CalcResponseForSetPos100(0x0000), 0x1000); // 0�
  assert_int_equal(CONVERSION_CalcResponseForSetPos100(0x0200), 0x165F); // 45�
  assert_int_equal(CONVERSION_CalcResponseForSetPos100(0x0400), 0x1D3F); // 90�
  assert_int_equal(CONVERSION_CalcResponseForSetPos100(0xFE00), 0x0920); // -45�
  assert_int_equal(CONVERSION_CalcResponseForSetPos100(0xFC00), 0x0240); // -90�
}

/**< Convert SetPosExt command to position value and reverse */
static void test_conversion_setposext(void **state)
{
  (void) state;

  /**< Convert from */
  //assert_int_equal(CONVERSION_CalcPosFromSetPosExt(0x00, 0x00), 0);
  /**< Convert to */
  assert_int_equal(CONVERSION_CalcResponseForSetPosExt(0x0000), 0x0800); // 0�
  assert_int_equal(CONVERSION_CalcResponseForSetPosExt(0x0200), 0x0B5F); // 45�
  assert_int_equal(CONVERSION_CalcResponseForSetPosExt(0x0400), 0x0EBF); // 90�
  assert_int_equal(CONVERSION_CalcResponseForSetPosExt(0xFE00), 0x04A0); // -45�
  assert_int_equal(CONVERSION_CalcResponseForSetPosExt(0xFC00), 0x0140); // -90�
}

/**< Calculate temperature from ADC value */
static void test_conversion_temperature(void **state)
{
  (void) state;

  assert_int_equal(CONVERSION_CalcTemperature(100), 10);
  assert_int_equal(CONVERSION_CalcTemperature(1024), 102);
  assert_int_equal(CONVERSION_CalcTemperature(2048), 204);
  assert_int_equal(CONVERSION_CalcTemperature(3000), 255);
}

/**< Calculate current from ADC value */
static void test_conversion_current(void **state)
{
  (void) state;

  /**< Targets have different current sensors! */
  assert_int_equal(CONVERSION_CalcCurrent(0), 0);
  #if defined DEF_DA58_SD
    assert_int_equal(CONVERSION_CalcCurrent(1000), 40);
    assert_int_equal(CONVERSION_CalcCurrent(2048), 81);
  #elif defined DEF_DA58
    assert_int_equal(CONVERSION_CalcCurrent(1000), 160);
    /**< Overcurrent event here! */
    assert_int_equal(CONVERSION_CalcCurrent(2048), 255);
  #else
    assert_int_equal(CONVERSION_CalcCurrent(1000), 80);
    assert_int_equal(CONVERSION_CalcCurrent(2048), 163);
  #endif // defined
}

/**< Calculate voltage from ADC value */
static void test_conversion_voltage(void **state)
{
  (void) state;

  assert_int_equal(CONVERSION_CalcVoltage(0), 0);
  assert_int_equal(CONVERSION_CalcVoltage(2048), 160);
}

/**< Calculate Volz conversions */
static void test_conversion_volz(void **state)
{
  (void) state;

  /**< Check position-from-degrees conversion */
  assert_int_equal(CONVERSION_CalcPosFromDegrees(0), 0);
  assert_int_equal(CONVERSION_CalcPosFromDegrees(450), 0x200);
  assert_int_equal(CONVERSION_CalcPosFromDegrees(900), 0x400);
  assert_int_equal(CONVERSION_CalcPosFromDegrees(-900) & 0xFFFF, 0xFC00);
  assert_int_equal(CONVERSION_CalcPosFromDegrees(1700), 0x78E);
  assert_int_equal(CONVERSION_CalcPosFromDegrees(-1700) & 0xFFFF, 0xF872);

  /**< Check degrees-from-position conversion */
  assert_int_equal(CONVERSION_CalcDegreesFromPos(0x000), 0);
  assert_int_equal(CONVERSION_CalcDegreesFromPos(0x200), 450);
  assert_int_equal(CONVERSION_CalcDegreesFromPos(0x400), 900);
  assert_int_equal(CONVERSION_CalcDegreesFromPos(0x700), 0x627);
  assert_int_equal(CONVERSION_CalcDegreesFromPos(-0x400), 0xFC7C);
  assert_int_equal(CONVERSION_CalcDegreesFromPos(-0x700), 0xF9D9);

  /**< Check status conversion */
  uint16_t status = 0;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), 0);
  status = STAT_ERROR_BLDC;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), VOLZSTS_BIT_MOTOR_ERR);
  status = STAT_ERROR_MAGNET;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), VOLZSTS_BIT_MAGNET_ERR);
  status = STAT_ERROR_POWER;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), VOLZSTS_BIT_UNDERVOLT);
  status = STAT_ERROR_LOSSCOMM;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), VOLZSTS_BIT_COMM_TIMEOUT);
  status = STAT_ERROR_ACE;
  assert_int_equal(CONVERSION_BuildVolzStatus(status), VOLZSTS_BIT_SYSTEM_ERR);
}

/**< Get length of string */
static void test_utils_getlength(void **state)
{
  (void) state;

  assert_int_equal(UTILS_GetLength(""), 0);
  assert_int_equal(UTILS_GetLength(" "), 1);
  assert_int_equal(UTILS_GetLength("Test"), 4);
  assert_int_equal(UTILS_GetLength("  Test"), 6);
  assert_int_equal(UTILS_GetLength("Test Test"), 9);
  /**< 0xff must cut the string */
  assert_int_equal(UTILS_GetLength("Te\xffst"), 2);
  assert_int_equal(UTILS_GetLength("\xff"), 0);
  assert_int_equal(UTILS_GetLength("\xffTest"), 0);
  /**< Maximal length check */
  assert_int_equal(UTILS_GetLength("LongLongLongLongLongLongLongLongString"), STRINGS_MAX_LEN);
}

/**< Sign of integer */
static void test_utils_sign(void **state)
{
  (void) state;

  assert_int_equal(UTILS_Sign(0), 0);
  assert_int_equal(UTILS_Sign(1), 1);
  assert_int_equal(UTILS_Sign(10), 1);
  assert_int_equal(UTILS_Sign(100), 1);
  assert_int_equal(UTILS_Sign(-1), -1);
  assert_int_equal(UTILS_Sign(-10), -1);
  assert_int_equal(UTILS_Sign(-100), -1);
}

/**< Value limiter */
static void test_utils_limit(void **state)
{
  (void) state;

  assert_int_equal(UTILS_LimitValue(0, 255), 0);
  assert_int_equal(UTILS_LimitValue(255, 0), 0);
  assert_int_equal(UTILS_LimitValue(254, 255), 254);
  assert_int_equal(UTILS_LimitValue(255, 255), 255);
  assert_int_equal(UTILS_LimitValue(256, 255), 255);
  assert_int_equal(UTILS_LimitValue(-254, 255), -254);
  assert_int_equal(UTILS_LimitValue(-255, 255), -255);
  assert_int_equal(UTILS_LimitValue(-255, 128), -128);
  assert_int_equal(UTILS_LimitValue(-256, 128), -128);
  assert_int_equal(UTILS_LimitValue(300, 128), 128);
}

/**< Convert int to string */
static void test_utils_inttostr(void **state)
{
  (void) state;

  assert_string_equal(UTILS_IntToStr(0, 1), "0");
  assert_string_equal(UTILS_IntToStr(1, 1), "1");
  assert_string_equal(UTILS_IntToStr(10, 1), "10");
  assert_string_equal(UTILS_IntToStr(91, 2), "91");
  assert_string_equal(UTILS_IntToStr(100, 2), "100");
  assert_string_equal(UTILS_IntToStr(0, 2), "00");
  assert_string_equal(UTILS_IntToStr(1, 2), "01");
}

/**< Convert float to string */
static void test_utils_floattostr(void **state)
{
  (void) state;

  assert_string_equal(UTILS_FloatToStr(0.0F, 1), "0.0");
  assert_string_equal(UTILS_FloatToStr(0.1F, 1), "0.1");
  assert_string_equal(UTILS_FloatToStr(0.1F, 2), "0.10");
  assert_string_equal(UTILS_FloatToStr(0.15F, 2), "0.15");
  assert_string_equal(UTILS_FloatToStr(0.15F, 1), "0.1");
  assert_string_equal(UTILS_FloatToStr(1.15F, 1), "1.1");
  assert_string_equal(UTILS_FloatToStr(3.15F, 2), "3.15");
  assert_string_equal(UTILS_FloatToStr(10.15F, 3), "10.150");
  assert_string_equal(UTILS_FloatToStr(9.015F, 2), "9.01");
  assert_string_equal(UTILS_FloatToStr(10.0F, 3), "10.000");
}

/**< Process new redundancy */
static void test_redundancy(void **state)
{
  (void) state;

  #ifdef DEF_DUPLEX
  /**< ACE1 */
  GLOBAL_InternalID = ID_ACE1;
  /**< No errors, do nothing */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_PartnerStatus = 0;
  GLOBAL_IsMaster = true;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Non-critical error, become slave */
  GLOBAL_MyStatus |= STAT_NONCRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_SLAVE);
  /**< Critical error, free */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_CRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Partner is master, do nothing */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_PartnerStatus = STAT_MASTER_BIT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Int.comm.error, do nothing */
  GLOBAL_PartnerStatus = 0;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Int.comm.error + no HB, do nothing */
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< no HB, do nothing */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner doesn't have LoC */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_SLAVE);
  /**< LoC, partner also has LoC */
  GLOBAL_PartnerStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FAILSAFE);
  /**< LoC, partner has critical/non-critical error error */
  GLOBAL_PartnerStatus = STAT_ERROR_ACE;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FAILSAFE);
  /**< LoC, Int.comm.error */
  GLOBAL_PartnerStatus = 0 ;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FAILSAFE);
  /**< ACE1 is slave, ACE2 is master */
  /**< No errors, do nothing */
  GLOBAL_MyStatus = 0;
  GLOBAL_PartnerIsMaster = true;
  GLOBAL_PartnerStatus = STAT_MASTER_BIT;
  GLOBAL_IsMaster = false;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Non-critical error, do nothing */
  GLOBAL_MyStatus = STAT_NONCRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Critical error, free */
  GLOBAL_MyStatus = STAT_CRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Int.comm.error, free */
  GLOBAL_MyStatus = STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Int.comm.error + no HB, become master */
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_MASTER);
  /**< no HB, do nothing */
  GLOBAL_MyStatus = STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Partner is slave, no HB, become master */
  GLOBAL_MyStatus = STAT_ERROR_HEARTBEAT;
  GLOBAL_PartnerStatus = 0;
  GLOBAL_PartnerIsMaster = false;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_MASTER);
  /**< Partner is slave, do nothing */
  GLOBAL_MyStatus = 0;
  GLOBAL_PartnerStatus = 0;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner doesn't have LoC */
  GLOBAL_MyStatus = STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner also has LoC */
  GLOBAL_PartnerStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, Int.comm.error */
  GLOBAL_PartnerStatus = 0 ;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);

  /**< ACE2 is master */
  GLOBAL_InternalID = ID_ACE2;
  /**< No errors, do nothing */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_PartnerStatus = 0;
  GLOBAL_IsMaster = true;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Non-critical error, become slave */
  GLOBAL_MyStatus |= STAT_NONCRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_SLAVE);
  /**< Critical error, free */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_CRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Partner is master, be slave */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_PartnerStatus = STAT_MASTER_BIT;
  GLOBAL_PartnerIsMaster = true;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_SLAVE);
  /**< Int.comm.error, do nothing */
  GLOBAL_PartnerStatus = 0;
  GLOBAL_PartnerIsMaster = false;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Int.comm.error + no HB, do nothing */
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< no HB, do nothing */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner doesn't have LoC */
  GLOBAL_MyStatus = STAT_MASTER_BIT;
  GLOBAL_MyStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_SLAVE);
  /**< LoC, partner also has LoC */
  GLOBAL_PartnerStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FAILSAFE);
  /**< LoC, Int.comm.error */
  GLOBAL_PartnerStatus = 0 ;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FAILSAFE);
  /**< ACE2 is slave, ACE1 is master */
  /**< No errors, do nothing */
  GLOBAL_MyStatus = 0;
  GLOBAL_PartnerStatus = STAT_MASTER_BIT;
  GLOBAL_IsMaster = false;
  GLOBAL_PartnerIsMaster = true;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Non-critical error, do nothing */
  GLOBAL_MyStatus = STAT_NONCRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Critical error, free */
  GLOBAL_MyStatus = STAT_CRITICAL_MASK;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Int.comm.error, free */
  GLOBAL_MyStatus = STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  /**< Int.comm.error + no HB, become master */
  GLOBAL_MyStatus |= STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_MASTER);
  /**< no HB, do nothing */
  GLOBAL_MyStatus = STAT_ERROR_HEARTBEAT;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< Partner is slave, no HB, become master */
  GLOBAL_MyStatus = STAT_ERROR_HEARTBEAT;
  GLOBAL_PartnerStatus = 0;
  GLOBAL_PartnerIsMaster = false;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_MASTER);
  /**< Partner is slave, do nothing */
  GLOBAL_MyStatus = 0;
  GLOBAL_PartnerStatus = 0;
  GLOBAL_PartnerIsMaster = false;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner doesn't have LoC */
  GLOBAL_MyStatus = STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, partner also has LoC */
  GLOBAL_PartnerStatus |= STAT_ERROR_LOSSCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_NO);
  /**< LoC, Int.comm.error */
  GLOBAL_PartnerStatus = 0 ;
  GLOBAL_MyStatus |= STAT_ERROR_NOINTCOMM;
  assert_int_equal(REDUNDANCY_Process(), REDUNDANCY_FREE);
  #endif
}

/**< Calculate notch filter coefficients */
static void test_notch_init(void **state)
{
  (void) state;
  // This test doesn't work now, problems with calculation for different versions of gcc
  // assert_in_range shows stupid error and can't be used

//  notch_t fc1, fc2, fc3, fc4, fc5;

//  NOTCH_RecalcCoeff(NOTCH_NUM_1, 1000, 50, 10);
//  NOTCH_GetCoefficients(NOTCH_NUM_1, &fc1, &fc2, &fc3, &fc4, &fc5);
//  assert_int_equal(fc1, 9955225);
//  assert_int_equal(fc2, -19899504);
//  assert_int_equal(fc3, 9945275);
//  assert_int_equal(fc4, 19899504);
//  assert_int_equal(fc5, -9900500);
//  NOTCH_RecalcCoeff(NOTCH_NUM_2, 2000, 50, 10);
//  NOTCH_GetCoefficients(NOTCH_NUM_2, &fc1, &fc2, &fc3, &fc4, &fc5);
//  assert_int_equal(fc1, 9910900);
//  assert_int_equal(fc2, -19798040);
//  assert_int_equal(fc3, 9891100);
//  assert_int_equal(fc4, 19798040);
//  assert_int_equal(fc5, -9802000);
//  NOTCH_GetCoefficients(NOTCH_NUM_1, &fc1, &fc2, &fc3, &fc4, &fc5);
//  assert_int_equal(fc1, 9955225);
//  assert_int_equal(fc2, -19899504);
//  assert_int_equal(fc3, 9945275);
//  assert_int_equal(fc4, 19899504);
//  assert_int_equal(fc5, -9900500);
}

/**< Calculate notch filter value */
static void test_notch_calculation(void **state)
{
  (void) state;

// TODO (AKiselev#1#): Add Notch tests

}

/**< CRC32 calculation */
static void test_crc32(void **state)
{
  (void) state;
  uint8_t buff[4] = {0x11, 0x22, 0x33, 0x44};
  uint8_t buff2[4] = {0x44, 0x33, 0x22, 0x11};

  assert_int_equal(CRC32_Calc(buff, 4), 0x77f29dd1);
  assert_int_equal(CRC32_Calc(buff2, 4), 0xde1d2d6d);
}

/**< CRC16 calculation */
static void test_crc16(void **state)
{
  (void) state;
  uint8_t buff[4] = {0x11, 0x22, 0x33, 0x44};
  uint8_t buff2[4] = {0x44, 0x33, 0x22, 0x11};

  /**< Test CCITT */
  assert_int_equal(CRC16_CalcCCITT(buff, 4), 0x59f3);
  assert_int_equal(CRC16_CalcCCITT(buff2, 4), 0xdecc);
  /**< Test Kearfott */
  assert_int_equal(CRC16_Calc(CRC16_INIT_VAL, buff, 4), 0x7d11);
  assert_int_equal(CRC16_Calc(CRC16_INIT_VAL, buff2, 4), 0x9fa0);
}

/**< Initialize EEPROM */
static void test_eeprom_init(void **state)
{
  (void) state;

  assert_false(EEPROM_CheckCRC());
  EEPROM_Configuration();
  assert_int_equal(EE_Kd, 50);
  assert_int_equal(EE_Kp, 40);
  assert_true(EEPROM_CheckCRC());
  assert_string_equal(VERSION_GetStringFW(), (char*)&EEPROM_Strings[EEPROM_FWREV_ADDRESS]);
}

/**< Write variable to EEPROM and check CRC */
static void test_eeprom_write(void **state)
{
  (void) state;

  EE_Kd = 11;
  EEPROM_SaveVariable(&EE_Kd);
  assert_true(EEPROM_CheckCRC());
  EE_Kd = 0;
  EEPROM_LoadVariables();
  assert_int_equal(EE_Kd, 11);
}

/**< Write 1 byte to EEPROM (low-level access) */
static void test_eeprom_writebyte(void **state)
{
  (void) state;

  /**< Write 0x10 to Kd */
  EEPROM_WriteByte(0x20, 0x10);
  assert_int_equal(EE_Kd, 0x10);
  /**< Write 0x30 to Kd */
  EEPROM_WriteByte(0x20, 0x30);
  assert_int_equal(EE_Kd, 0x30);
  /**< Write 0x10 to Kp */
  EEPROM_WriteByte(0x21, 0x10);
  assert_int_equal(EE_Kp, 0x10);
  /**< Write 0x30 to Kp */
  EEPROM_WriteByte(0x21, 0x30);
  assert_int_equal(EE_Kp, 0x30);
  /**< Write 0x07FF to CanId (uint16_t) */
  EEPROM_WriteByte(0x12, 0xFF);
  EEPROM_WriteByte(0x13, 0x07);
  assert_int_equal(EE_CanId, 0x07FF);
  /**< Write 0x03E0 to CanId (uint16_t) */
  EEPROM_WriteByte(0x12, 0xE0);
  EEPROM_WriteByte(0x13, 0x03);
  assert_int_equal(EE_CanId, 0x03E0);
}

/**< Read 1 byte from EEPROM (low-level access) */
static void test_eeprom_readbyte(void **state)
{
  (void) state;
  uint8_t bval;
  uint16_t wval;

  /**< Read Kd value from EEPROM address 0x20 */
  bval = EEPROM_ReadByte(0x20);
  assert_int_equal(bval, EE_Kd);
  /**< Read Kp value from EEPROM address 0x21 */
  bval = EEPROM_ReadByte(0x21);
  assert_int_equal(bval, EE_Kp);
  /**< Read CanId value from EEPROM addresses 0x12..0x13 */
  wval = EEPROM_ReadByte(0x13);
  wval <<= 8;
  wval += EEPROM_ReadByte(0x12);
  assert_int_equal(wval, EE_CanId);
}

/**< Get parameter from EEPROM, high-level function */
static void test_eeprom_getparam(void **state)
{
  (void) state;
  eeVal_t param;

  /**< Parameter #0, EE_CanBitrate */
  assert_true(EEPROM_GetParam(0, &param));
  assert_int_equal(param.address, 0x10);
  assert_int_equal(param.defVal.byte, CAN_BITRATE_500K);
  /**< Parameter #3, EE_Kd */
  assert_true(EEPROM_GetParam(3, &param));
  assert_int_equal(param.address, 0x20);
  assert_int_equal(param.defVal.byte, 50);
  assert_int_equal(param.minVal.byte, 0);
  assert_int_equal(param.maxVal.byte, 255);
  /**< Parameter #12, EE_ZeroPos */
  assert_true(EEPROM_GetParam(12, &param));
  assert_int_equal(param.address, 0x33);
  assert_int_equal(param.defVal.sword, 0);
  assert_int_equal(param.minVal.sword, -2048);
  assert_int_equal(param.maxVal.sword, 2048);
  /**< Parameter doesn't exist */
  assert_false(EEPROM_GetParam(255, &param));
}

/**< Magnet sensor readout */
static void test_magnet_read(void **state)
{
  (void) state;
  uint16_t position;

  /**< Read position without error */
  will_return(SPI_Receive, 0x100);
  will_return(SPI_Receive, MAGNET_STS_OCF);
  assert_true(MAGNET_Read(&position));
  /**< Position is inverted! */
  assert_int_equal(position, 0xEFF);
  assert_false(MAGNET_HasError());
  /**< Read position with error */
  will_return(SPI_Receive, 0x100);
  will_return(SPI_Receive, MAGNET_STS_PAR);
  assert_false(MAGNET_Read(&position));
  assert_true(MAGNET_HasError());
}

/**< Measure distance between two position values */
static void test_voting_get_dist(void **state)
{
  (void) state;

  #ifdef DEF_DUPLEX
  /**< Naive tests */
  assert_int_equal(VOTING_GetDist(0, 555), 555);
  assert_int_equal(VOTING_GetDist(100, 100), 0);
  assert_int_equal(VOTING_GetDist(100, 200), 100);
  assert_int_equal(VOTING_GetDist(200, 100), 100);
  assert_int_equal(VOTING_GetDist(100, 300), 200);
  /**< More complex tests with zero-crossing */
  assert_int_equal(VOTING_GetDist(0, MAGNET_MAX_VALUE - 1), 1);
  assert_int_equal(VOTING_GetDist(1, MAGNET_MAX_VALUE - 1), 2);
  assert_int_equal(VOTING_GetDist(100, MAGNET_MAX_VALUE - 1), 101);
  assert_int_equal(VOTING_GetDist(100, MAGNET_MAX_VALUE - 100), 200);
  #endif
}

/**< Perform 2oo3 voting */
static void test_voting_process(void **state)
{
  (void) state;

  #ifdef DEF_DUPLEX
  /**< All sensors are connected */
  GLOBAL_MyStatus = 0;
  /**< Same values everywhere */
  GLOBAL_MagnetValue = 0;
  GLOBAL_PartnerMagnet = 0;
  GLOBAL_MagnetExtHall = 0;
  assert_int_equal(VOTING_Process(), VOTING_OK);
  /**< One value differs */
  GLOBAL_MagnetExtHall = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_OK_ACE2VOT);
  /**< Two values differ */
  GLOBAL_PartnerMagnet = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_ERROR);
  /**< External sensor is not connected */
  GLOBAL_ExtStatus = EXTSTAT_ERROR_EXTHALL;
  /**< Same values for ACE1 and ACE2 */
  GLOBAL_MagnetValue = 0;
  GLOBAL_PartnerMagnet = 0;
  GLOBAL_MagnetExtHall = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_OK_ACE2VOT);
  /**< Different values for ACE1 and ACE2 */
  GLOBAL_PartnerMagnet = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_ERROR);
  /**< ACE2 sensor is not connected, External sensor is connected */
  GLOBAL_MyStatus = STAT_ERROR_NOINTCOMM;
  GLOBAL_ExtStatus = 0;
  /**< Same values for ACE1 and extHall */
  GLOBAL_MagnetValue = 0;
  GLOBAL_PartnerMagnet = MAX_MAGNET_DEVIATION + 1;
  GLOBAL_MagnetExtHall = 0;
  assert_int_equal(VOTING_Process(), VOTING_OK_ACE2ACE);
  /**< Different values for ACE1 and extHall */
  GLOBAL_MagnetExtHall = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_ERROR);
  /**< Standalone mode */
  GLOBAL_MyStatus = STAT_ERROR_NOINTCOMM;
  GLOBAL_ExtStatus = EXTSTAT_ERROR_EXTHALL;
  assert_int_equal(VOTING_Process(), VOTING_OK_ACEVBOTH);
  GLOBAL_MagnetValue = MAX_MAGNET_DEVIATION + 1;
  assert_int_equal(VOTING_Process(), VOTING_OK_ACEVBOTH);
  #endif
}

static void test_hall_sensors(void **state)
{
  (void) state;
  uint8_t halls_all_ok_cw[HALL_COMM_LEN] = {1, 3, 2, 6, 4, 5};
  uint8_t halls_all_ok_ccw[HALL_COMM_LEN] = {3, 1, 5, 4, 6, 2};
  uint8_t halls_no_data[HALL_COMM_LEN] = {0, 0, 0, 0, 0, 0};
  uint8_t halls_only_one[HALL_COMM_LEN] = {2, 2, 2, 2, 2, 2};
  uint8_t halls_a_ok[HALL_COMM_LEN] = {3, 2, 2, 2, 2, 2};
  uint8_t halls_a_gnd[HALL_COMM_LEN] = {0, 2, 6, 4, 0, 2};
  uint8_t halls_a_vcc[HALL_COMM_LEN] = {1, 3, 7, 5, 1, 3};

  assert_int_equal(HALL_CheckSensors(halls_all_ok_cw), 0);
  assert_int_equal(HALL_CheckSensors(halls_all_ok_ccw), 0);
  assert_int_equal(HALL_CheckSensors(halls_no_data), HALL_CHECK_HALL_A | HALL_CHECK_HALL_B | HALL_CHECK_HALL_C);
  assert_int_equal(HALL_CheckSensors(halls_only_one), HALL_CHECK_HALL_A | HALL_CHECK_HALL_B | HALL_CHECK_HALL_C);
  assert_int_equal(HALL_CheckSensors(halls_only_one), HALL_CHECK_HALL_A | HALL_CHECK_HALL_B | HALL_CHECK_HALL_C);
  assert_int_equal(HALL_CheckSensors(halls_a_ok), HALL_CHECK_HALL_B | HALL_CHECK_HALL_C);
  assert_int_equal(HALL_CheckSensors(halls_a_gnd), HALL_CHECK_HALL_A);
  assert_int_equal(HALL_CheckSensors(halls_a_vcc), HALL_CHECK_HALL_A);
}

static void test_hall_windings(void **state)
{
  (void) state;
  uint8_t windings_all_wrong[HALL_COMM_LEN] = {0, 0, 0, 0, 0, 0};
  uint8_t windings_all_ok[HALL_COMM_LEN] = {1, 1, 1, 1, 1, 1};
  uint8_t windings_a_wrong_cw[HALL_COMM_LEN] = {1, 0, 0, 0, 0, 1};
  uint8_t windings_a_wrong_ccw[HALL_COMM_LEN] = {0, 1, 0, 0, 1, 0};
  uint8_t windings_b_wrong_cw[HALL_COMM_LEN] = {0, 1, 0, 0, 1, 0};
  uint8_t windings_b_wrong_ccw[HALL_COMM_LEN] = {0, 0, 1, 1, 0, 0};
  uint8_t windings_c_wrong_cw[HALL_COMM_LEN] = {0, 0, 1, 1, 0, 0};
  uint8_t windings_c_wrong_ccw[HALL_COMM_LEN] = {1, 0, 0, 0, 0, 1};

  assert_int_equal(HALL_CheckWindings(windings_all_wrong, MOTOR_DIR_CW), HALL_CHECK_WIND_A | HALL_CHECK_WIND_B | HALL_CHECK_WIND_C);
  assert_int_equal(HALL_CheckWindings(windings_all_wrong, MOTOR_DIR_CCW), HALL_CHECK_WIND_A | HALL_CHECK_WIND_B | HALL_CHECK_WIND_C);
  assert_int_equal(HALL_CheckWindings(windings_all_ok, MOTOR_DIR_CW), 0);
  assert_int_equal(HALL_CheckWindings(windings_all_ok, MOTOR_DIR_CCW), 0);
  assert_int_equal(HALL_CheckWindings(windings_a_wrong_cw, MOTOR_DIR_CW), HALL_CHECK_WIND_A);
  assert_int_equal(HALL_CheckWindings(windings_a_wrong_ccw, MOTOR_DIR_CCW), HALL_CHECK_WIND_A);
  assert_int_equal(HALL_CheckWindings(windings_b_wrong_cw, MOTOR_DIR_CW), HALL_CHECK_WIND_B);
  assert_int_equal(HALL_CheckWindings(windings_b_wrong_ccw, MOTOR_DIR_CCW), HALL_CHECK_WIND_B);
  assert_int_equal(HALL_CheckWindings(windings_c_wrong_cw, MOTOR_DIR_CW), HALL_CHECK_WIND_C);
  assert_int_equal(HALL_CheckWindings(windings_c_wrong_ccw, MOTOR_DIR_CCW), HALL_CHECK_WIND_C);
}

static void test_inetwork_send(void **state)
{
  (void) state;
  #ifdef DEF_DUPLEX
  uint8_t data[INET_PACKET_LEN];
  uint16_t value;

  GLOBAL_MyTN = ACE1_TN;
  GLOBAL_IsMaster = false;
  GLOBAL_MasterRequest = false;
  GLOBAL_Power = 0x80;
  GLOBAL_SlavePower = 0xA1;
  /**< Check ICMD_N */
  GLOBAL_DoDefault = INET_BE_DFLT;
  INETWORK_BuildPacket(ICMD_N, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_N | GLOBAL_MyTN);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  assert_int_equal(data[INET_PACKET_ID_POS + 4], INET_BE_DFLT);
  assert_int_equal(data[INET_PACKET_ID_POS + 5], (uint8_t)(~INET_BE_DFLT));
  /**< Check ICMD_S */
  GLOBAL_MyStatus = 0xAA55;
  INETWORK_BuildPacket(ICMD_S, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_S | GLOBAL_MyTN);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  value = ((uint16_t)data[INET_PACKET_ID_POS + 4] << 8) + data[INET_PACKET_ID_POS + 5];
  assert_int_equal(value, GLOBAL_MyStatus);
  /**< Check ICMD_A */
  GLOBAL_IsMaster = true;
  GLOBAL_ExternalStatus = 0x55AA;
  INETWORK_BuildPacket(ICMD_A, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_A | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  value = ((uint16_t)data[INET_PACKET_ID_POS + 4] << 8) + data[INET_PACKET_ID_POS + 5];
  assert_int_equal(value, GLOBAL_ExternalStatus);
  /**< Check ICMD_T */
  //will_return(MONITOR_GetTemperature, 0x80);
  //will_return(MONITOR_GetTemperature, 0x90);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 0x90);
  INETWORK_BuildPacket(ICMD_T, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_T | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  assert_int_equal(data[INET_PACKET_ID_POS + 4], 0x80);
  assert_int_equal(data[INET_PACKET_ID_POS + 5], 0x90);
  /**< Check ICMD_U */
  MONITOR_SetVoltage(MONITOR_U_CH1, 0x81);
  MONITOR_SetVoltage(MONITOR_U_CH2, 0x91);
  INETWORK_BuildPacket(ICMD_U, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_U | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  assert_int_equal(data[INET_PACKET_ID_POS + 4], 0x81);
  assert_int_equal(data[INET_PACKET_ID_POS + 5], 0x91);
  /**< Check ICMD_P */
  GLOBAL_TargetPos = 0x400;
  INETWORK_BuildPacket(ICMD_P, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_P | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  value = ((uint16_t)data[INET_PACKET_ID_POS + 4] << 8) + data[INET_PACKET_ID_POS + 5];
  assert_int_equal(value, GLOBAL_TargetPos);
  /**< Check ICMD_I */
  GLOBAL_Power = 0x90;
  //will_return(MONITOR_GetCurrent, CURR_LOAD_25);
  for (int i = 0; i < 50; i++)
    MONITOR_SetCurrent(CURR_LOAD_25);
  INETWORK_BuildPacket(ICMD_I, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_I | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  assert_int_equal(data[INET_PACKET_ID_POS + 4], CURR_LOAD_25);
  assert_int_equal(data[INET_PACKET_ID_POS + 5], 0);
  /**< Check ICMD_CRC */
  INETWORK_BuildPacket(ICMD_CRC, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_CRC | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  value = ((uint16_t)data[INET_PACKET_ID_POS + 4] << 8) + data[INET_PACKET_ID_POS + 5];
  assert_int_equal(value, EEPROM_GetCRC());
  /**< Check ICMD_BL */
  GLOBAL_DoStartBootloader = BL_START_SIGN;
  GLOBAL_InternalID = ID_ACE1;
  INETWORK_BuildPacket(ICMD_BL, data);
  assert_int_equal(data[INET_PACKET_SIGN_POS], INET_PACKET_SIGN);
  assert_int_equal(data[INET_PACKET_ID_POS], ICMD_BL | GLOBAL_MyTN | 0x10);
  assert_int_equal(data[INET_PACKET_ID_POS + 3], GLOBAL_SlavePower);
  assert_int_equal(data[INET_PACKET_ID_POS + 4], BL_SIGN1);
  assert_int_equal(data[INET_PACKET_ID_POS + 5], BL_SIGN2);
  assert_int_equal(GLOBAL_DoStartBootloader, BL_STOP_SIGN);
  #endif
}

static void test_inetwork_parse(void **state)
{
  (void) state;

  #ifdef DEF_DUPLEX
  /**< Parse ICMD_N */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_N, 0x00, 0x00, 0x80, 0xAA, 0x55});
  assert_int_equal(GLOBAL_PartnerPower, 0x80);
  assert_int_equal(GLOBAL_PartnerMagnet, 0);
  /**< Parse ICMD_S */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_S, 0x00, 0x00, 0x80, 0xAA, 0x55});
  assert_int_equal(GLOBAL_PartnerPower, 0x80);
  assert_int_equal(GLOBAL_PartnerStatus, 0xAA55);
  assert_int_equal(GLOBAL_PartnerMagnet, 0);
  /**< Parse ICMD_A */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_A, 0x00, 0x01, 0x81, 0x55, 0xAA});
  assert_int_equal(GLOBAL_PartnerPower, 0x81);
  assert_int_equal(GLOBAL_PartnerExtStatus, 0x55AA);
  assert_int_equal(GLOBAL_PartnerMagnet, 1);
  /**< Parse ICMD_T */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_T, 0x01, 0x00, 0x82, 0x11, 0x12});
  assert_int_equal(GLOBAL_PartnerPower, 0x82);
  assert_int_equal(GLOBAL_PartnerTempM, 0x11);
  assert_int_equal(GLOBAL_PartnerTempP, 0x12);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x100);
  /**< Parse ICMD_U */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_U, 0x01, 0x01, 0x83, 0x21, 0x22});
  assert_int_equal(GLOBAL_PartnerPower, 0x83);
  assert_int_equal(GLOBAL_PartnerU1, 0x21);
  assert_int_equal(GLOBAL_PartnerU2, 0x22);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x101);
  /**< Parse ICMD_P */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_P, 0x01, 0x02, 0x84, 0x01, 0x11});
  assert_int_equal(GLOBAL_PartnerPower, 0x84);
  assert_int_equal(GLOBAL_PartnerTargetPos, 0x111);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x102);
  /**< Parse ICMD_I */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_I, 0x02, 0x02, 0x85, 0x20, 0x21});
  assert_int_equal(GLOBAL_PartnerPower, 0x85);
  assert_int_equal(GLOBAL_PartnerI, 0x20);
  assert_int_equal(GLOBAL_PartnerRH, 0x21);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x202);
  /**< Parse ICMD_CRC */
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_CRC, 0x02, 0x03, 0x86, 0xA5, 0x5A});
  assert_int_equal(GLOBAL_PartnerPower, 0x86);
  assert_int_equal(GLOBAL_PartnerCRC, 0xA55A);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x203);
  /**< Parse ICMD_C */
  GLOBAL_InternalID = ID_ACE2;
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_C, 0x03, 0x03, 0x87, 0xA5, 0x5A});
  assert_int_equal(GLOBAL_PartnerPower, 0x87);
  assert_int_equal(GLOBAL_DoSensorCalibration, 1);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x303);
  INETWORK_ParsePacket((uint8_t[INET_PACKET_LEN]){INET_PACKET_SIGN, ACE2_TN | ICMD_C, 0x03, 0x04, 0x88, 0xA4, 0x5A});
  assert_int_equal(GLOBAL_PartnerPower, 0x88);
  assert_int_equal(GLOBAL_DoSensorCalibration, 0);
  assert_int_equal(GLOBAL_PartnerMagnet, 0x304);
  /**< Parse ICMD_BL */
  #endif
}

/**< Check resetting default values */
static void test_logic_default_values(void **state)
{
  (void) state;

  GLOBAL_IsMaster = true;
  GLOBAL_InternalID = ID_ACE1;
  /**< MOTOR_MODE_FREE */
  LOGIC_SetDefaultValues(MOTOR_MODE_FREE, RESET_MODE_SINGLE);
  assert_int_equal(GLOBAL_MyStatus, 0);
  assert_int_equal(GLOBAL_ExtStatus, 0);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_FREE);
  assert_int_equal(MOTOR_GetExtPower(), 0);
  #ifdef DEF_DUPLEX
  assert_int_not_equal(GLOBAL_DoDefault, INET_BE_DFLT);
  assert_int_equal(GLOBAL_ForceSlaveMode, MOTOR_MODE_FREE);
  assert_int_equal(MOTOR_GetSlaveMode(), MOTOR_MODE_FREE);
  assert_true(GLOBAL_IsMaster);
  #endif
  /**< MOTOR_MODE_RUN1 */
  LOGIC_SetDefaultValues(MOTOR_MODE_RUN1, RESET_MODE_SINGLE);
  assert_int_equal(GLOBAL_MyStatus, 0);
  assert_int_equal(GLOBAL_ExtStatus, 0);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  assert_int_equal(MOTOR_GetExtPower(), 0);
  #ifdef DEF_DUPLEX
  assert_int_not_equal(GLOBAL_DoDefault, INET_BE_DFLT);
  assert_int_equal(GLOBAL_ForceSlaveMode, MOTOR_MODE_RUN1);
  assert_int_equal(MOTOR_GetSlaveMode(), MOTOR_MODE_RUN1);
  assert_true(GLOBAL_IsMaster);
  #endif
  /**< MOTOR_MODE_FREE, reset ALL */
  LOGIC_SetDefaultValues(MOTOR_MODE_FREE, RESET_MODE_ALL);
  assert_int_equal(GLOBAL_MyStatus, 0);
  assert_int_equal(GLOBAL_ExtStatus, 0);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_FREE);
  assert_int_equal(MOTOR_GetExtPower(), 0);
  #ifdef DEF_DUPLEX
  assert_int_equal(GLOBAL_DoDefault, INET_BE_DFLT);
  assert_int_equal(GLOBAL_ForceSlaveMode, MOTOR_MODE_RUN1);
  assert_int_equal(MOTOR_GetSlaveMode(), MOTOR_MODE_FREE);
  assert_true(GLOBAL_IsMaster);
  #endif
}

/**< Check Timeouts module */
static void test_logic_timeouts(void **state)
{
  (void) state;
  uint8_t i;

  for (i = 0; i < 10; i++)
    TIMEOUTS_Inc(TIMEOUT_TYPE_HOST);
  for (i = 0; i < 11; i++)
    TIMEOUTS_Inc(TIMEOUT_TYPE_HALL);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_HOST), 10);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_HALL), 11);
  TIMEOUTS_Reset(TIMEOUT_TYPE_HOST);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_HOST), 0);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_HALL), 11);
  for (i = 0; i < 12; i++)
    TIMEOUTS_Inc(TIMEOUT_TYPE_PARTNER);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_PARTNER), 12);
  TIMEOUTS_Reset(TIMEOUT_TYPE_ALL);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_PARTNER), 0);
  assert_int_equal(TIMEOUTS_GetValue(TIMEOUT_TYPE_HALL), 0);
}

/**< Check load counters */
static void test_logic_load_counters(void **state)
{
  (void) state;
  uint8_t i;

  /**< Reset volatile counters */
  for (i = 0; i < COUNTER_LOAD_LAST; i++)
  {
    COUNTERS_Reset(i);
    assert_int_equal(COUNTERS_GetValue(i), 0);
  }
  /**< Increment every volatile counter once */
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(0);
  COUNTERS_Process();
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_0), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_ALL), 1);
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(CURR_LOAD_25 + 1);
  COUNTERS_Process();
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_0), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_25), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_ALL), 2);
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(CURR_LOAD_50 + 1);
  COUNTERS_Process();
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_0), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_25), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_50), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_ALL), 3);
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(CURR_LOAD_75 + 1);
  COUNTERS_Process();
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_0), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_25), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_50), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_75), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_ALL), 4);
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(CURR_LOAD_100 + 1);
  COUNTERS_Process();
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_0), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_25), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_50), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_75), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_100), 1);
  assert_int_equal(COUNTERS_GetValue(COUNTER_LOAD_ALL), 5);

  /**< Check EEPROM counters after 100 increments of COUNTER_LOAD_0 */
  for (i = 0; i < COUNTER_LOAD_LAST; i++)
    assert_int_equal(EEPROM_GetCounter(i), 0);
  for (i = 0; i < 50; i++)
    MONITOR_SetCurrent(0);
  for (i = 0; i < 100; i++)
  {
    COUNTERS_Process();
  }
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_ALL), 1);
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_0), 1);
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_25), 0);
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_50), 0);
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_75), 0);
  assert_int_equal(EEPROM_GetCounter(COUNTER_LOAD_100), 0);
}

/**< Check reset PWM function */
static void test_logic_reset_pwm(void **state)
{
  (void) state;

  #ifdef DEF_DUPLEX
  GLOBAL_PartnerPower = 0x10;
  GLOBAL_SlavePower = 0x20;
  GLOBAL_Power = 0x30;
  LOGIC_ResetPWM();
  assert_int_equal(GLOBAL_PartnerPower, 0);
  assert_int_equal(GLOBAL_SlavePower, 0);
  assert_int_equal(GLOBAL_Power, 0);
  #endif
}

static void test_motor_process_magnet(void **state)
{
  (void) state;

  GLOBAL_InternalID = ID_ACE1;
  /**< Calculate value with no offset */
  EE_ZeroPos = 0;
  will_return(SPI_Receive, 0xEFF);
  will_return(SPI_Receive, MAGNET_STS_OCF);
  MOTOR_ProcessMagnet();
  assert_false(GLOBAL_ErrMagnet);
  assert_int_equal(GLOBAL_MagnetValue, 0x100);
  assert_int_equal(GLOBAL_RealPos & 0xfff, 0x900);
  /**< Calculate value with zero offset */
  EE_ZeroPos = -0x100;
  will_return(SPI_Receive, 0xEFF);
  will_return(SPI_Receive, MAGNET_STS_OCF);
  MOTOR_ProcessMagnet();
  assert_false(GLOBAL_ErrMagnet);
  assert_int_equal(GLOBAL_MagnetValue, 0x100);
  assert_int_equal(GLOBAL_RealPos & 0xfff, 0x800);
}

static void test_motor_calc_power(void **state)
{
  (void) state;

  GLOBAL_MotorExtMode = false;
  /**< No difference, power = 0 */
  GLOBAL_TargetPos = 0;
  GLOBAL_RealPos = 0;
  assert_int_equal(MOTOR_CalcPower(), 0);
  /**< Big difference, power = 0xff */
  GLOBAL_TargetPos = 0x100;
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), 0xff);
  /**< Big difference, PWM limitation, power = 0x80 */
  EE_PWM_Max = 0x80;
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), 0x80);
  /**< Small difference, PWM limitation, power = 0 */
  //GLOBAL_TargetPos = 0x02;
  //MOTOR_ResetPID();
  //assert_int_equal(MOTOR_CalcPower(), 0);
  /**< Big difference, limitation through GLOBAL_MotorExtP */
  GLOBAL_TargetPos = 0x100;
  GLOBAL_MotorExtMode = true;
  MOTOR_SetExtPower(0x90);
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), 0x90);
  GLOBAL_MotorExtMode = false;
  /**< Negative value, no limitation (EE_PWM_Max = 0xFF) */
  EE_PWM_Max = 0xFF;
  GLOBAL_TargetPos = -0x100;
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), -0xFF);
  /**< Negative value, limitation through 0x80 */
  EE_PWM_Max = 0x80;
  GLOBAL_TargetPos = -0x100;
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), -0x80);
  /**< Test EE_Sensor_DB limitation */
  EE_Sensor_DB = 0x20;
  GLOBAL_TargetPos = 0x1F;
  MOTOR_ResetPID();
  assert_int_equal(MOTOR_CalcPower(), 0);
  /**< Test real values  */
  {
    EE_Sensor_DB = 0x00;
    EE_PWM_Max = 0xFF;
    GLOBAL_TargetPos = 0x10;
    MOTOR_ResetPID();
    int32_t val = ((GLOBAL_TargetPos * EE_Kp) >> 4) + ((GLOBAL_TargetPos * EE_Kd) >> 2);
    assert_int_equal(MOTOR_CalcPower(), val);
  }
  /**< Test real values with positive TargetPos and EE_Sensor_DB */
  {
    EE_Sensor_DB = 0x08;
    EE_PWM_Max = 0xFF;
    GLOBAL_TargetPos = 0x10;
    MOTOR_ResetPID();
    int32_t val = (((GLOBAL_TargetPos - EE_Sensor_DB) * EE_Kp) >> 4) + (((GLOBAL_TargetPos - EE_Sensor_DB) * EE_Kd) >> 2);
    assert_int_equal(MOTOR_CalcPower(), val);
  }
  /**< Test real values with negative TargetPos and EE_Sensor_DB */
  {
    EE_Sensor_DB = 0x08;
    EE_PWM_Max = 0xFF;
    GLOBAL_TargetPos = 0x10;
    MOTOR_ResetPID();
    int32_t val = (((GLOBAL_TargetPos - EE_Sensor_DB) * EE_Kp) >> 4) + (((GLOBAL_TargetPos - EE_Sensor_DB) * EE_Kd) >> 2);
    assert_int_equal(MOTOR_CalcPower(), val);
  }
}

static void test_motor_damping(void **state)
{
  (void) state;
  uint8_t power;
  uint8_t i;

  /**< No damping - no delays */
  EE_PowerDamper = 0;
  power = 0xff;
  assert_int_equal(MOTOR_DoDamping(power), 0xff);
  /**< Enable damping, first value must be lower than input power */
  EE_PowerDamper = 10;
  MOTOR_DoDamping(0);
  assert_int_not_equal(MOTOR_DoDamping(power), 0xff);
  for (i = 0; i < 100; i++)
    MOTOR_DoDamping(power);
  /**< After several iterations damping value must be equal to input power */
  assert_int_equal(MOTOR_DoDamping(power), 0xff);
  /**< Total damping, always zero power */
  EE_PowerDamper = 255;
  MOTOR_DoDamping(0);
  assert_int_equal(MOTOR_DoDamping(power), 0);
  for (i = 0; i < 100; i++)
    MOTOR_DoDamping(power);
  assert_int_equal(MOTOR_DoDamping(power), 0);
}

/**< Motor saver function */
static void test_motor_saver(void **state)
{
  (void) state;
  uint16_t i;

  /**< Motor saver is active, EE_SaverTime - time in seconds */
  GLOBAL_RealPos = 1;
  GLOBAL_TargetPos = 1;
  assert_int_equal(MOTOR_Saver(0xff), 0xff);
  for (i = 0; i < (EE_SaverTime + 1) * 1000; i++)
    MOTOR_Saver(0xff);
  assert_int_equal(MOTOR_Saver(0xff), EE_PWM_Safe);
  /**< Check saver frame functionality */
  GLOBAL_RealPos += (EE_SaverFrame + 1);
  assert_int_equal(MOTOR_Saver(0xff), 0xFF);
  /**< Motor saver is not active */
  EE_SaverTime = 0;
  for (i = 0; i < (EE_SaverTime + 1) * 1000; i++)
    MOTOR_Saver(0xff);
  assert_int_equal(MOTOR_Saver(0xff), 0xFF);
}

/**< Motor heater function */
static void test_motor_heater(void **state)
{
  (void) state;
  uint8_t uval8;
  uint8_t pwm_value = 0x20;

  EE_PWM_Min = 0x10;

  /**< Heater is disabled */
  EE_HeaterTemp = 0;
  //will_return(MONITOR_GetTemperature, 60);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 60);
  assert_int_equal(MOTOR_Heater(pwm_value), pwm_value);
  /**< Actual temperature is higher than heater start point */
  EE_HeaterTemp = 50;
  //will_return(MONITOR_GetTemperature, 60);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x60);
  assert_int_equal(MOTOR_Heater(pwm_value), pwm_value);
  /**< Actual temperature makes maximal PWM value */
  EE_HeaterTemp = 50;
  uval8 = EE_HeaterTemp - (uint8_t)(0x100 / HEATER_GAIN_COEFF);
  //will_return(MONITOR_GetTemperature, uval8);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, uval8);
  assert_int_equal(MOTOR_Heater(pwm_value), HEATER_MAX_PWM);
  /**< Small temperature difference */
  EE_HeaterTemp = 50;
  uval8 = EE_HeaterTemp - 1;
  EE_PWM_Min = HEATER_GAIN_COEFF;
  //will_return(MONITOR_GetTemperature, uval8);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, uval8);
  assert_int_equal(MOTOR_Heater(pwm_value), pwm_value);
}

/**< Motor current limiter */
static void test_motor_current(void **state)
{
  (void) state;

  assert_int_equal(MOTOR_CurrentLimiter(0x00, 0xff, false), 0xff);
}

/**< Motor mode functions */
static void test_motor_mode(void **state)
{
  (void) state;

  GLOBAL_IsMaster = true;
  MOTOR_SetMode(MOTOR_MODE_FREE);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_FREE);
  MOTOR_SetMode(MOTOR_MODE_RUN1);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  MOTOR_SetMode(MOTOR_MODE_RUN2);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN2);
  /**< Wrong value, ignore */
  MOTOR_SetMode(0xff);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN2);
}

/**< Monitor: current check */
static void test_monitor_current(void **state)
{
  (void) state;

  MONITOR_GetMaxCurrent();
  for (int i = 0; i < 256; i++)
  {
    MONITOR_SetCurrent((uint8_t)i);
    assert_int_equal(MONITOR_GetMaxCurrent(), i);
  }
  /**< Check maximal current algorithm */
  for (int i = 0; i < 50; i++)
    MONITOR_SetCurrent(255);
  assert_int_equal(MONITOR_GetMaxCurrent(), 255);
  for (int i = 0; i < 50; i++)
    MONITOR_SetCurrent(0);
  MONITOR_GetMaxCurrent();
  assert_int_equal(MONITOR_GetMaxCurrent(), 0);
}

/**< Monitor: voltage check */
static void test_monitor_voltage(void **state)
{
  (void) state;

  MONITOR_SetVoltage(MONITOR_U_CH2, 0);
  for (int i = 0; i < 256; i++)
  {
    MONITOR_SetVoltage(MONITOR_U_CH1, (uint8_t)i);
    assert_int_equal(MONITOR_GetVoltage(MONITOR_U_CH1), i);
    assert_int_equal(MONITOR_GetVoltage(MONITOR_U_CH2), 0);
  }

  MONITOR_SetVoltage(MONITOR_U_CH1, 0);
  for (int i = 0; i < 256; i++)
  {
    MONITOR_SetVoltage(MONITOR_U_CH2, (uint8_t)i);
    assert_int_equal(MONITOR_GetVoltage(MONITOR_U_CH2), i);
    assert_int_equal(MONITOR_GetVoltage(MONITOR_U_CH1), 0);
  }

  /**< Check voltage status */
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_OK - 1);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_OK - 1);
  assert_false(MONITOR_GetVoltageStatus());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_OK);
  assert_false(MONITOR_GetVoltageStatus());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_OK - 1);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_OK);
  assert_false(MONITOR_GetVoltageStatus());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_OK);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_OK);
  assert_true(MONITOR_GetVoltageStatus());

  /**< Get supply status */
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_ZERO - 1);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_ZERO - 1);
  assert_false(MONITOR_GetSupply());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_ZERO);
  assert_true(MONITOR_GetSupply());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_ZERO - 1);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_ZERO);
  assert_true(MONITOR_GetSupply());
  MONITOR_SetVoltage(MONITOR_U_CH1, VOLTAGE_ZERO);
  MONITOR_SetVoltage(MONITOR_U_CH2, VOLTAGE_ZERO);
  assert_true(MONITOR_GetSupply());
}

/**< Monitor: temperature check */
static void test_monitor_temperature(void **state)
{
  (void) state;

  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 0);
  for (uint16_t i = 0; i < 256; i++)
  {
    MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, (uint8_t)i);
    if (i < MONITOR_TEMP_MIN)
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_MOTOR), MONITOR_TEMP_LOW);
    } else
    if (i > MONITOR_TEMP_MAX)
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_MOTOR), MONITOR_TEMP_HIGH);
    } else
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_MOTOR), i);
    }
    assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_PCB), MONITOR_TEMP_LOW);
  }

  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0);
  for (uint16_t i = 0; i < 256; i++)
  {
    MONITOR_SetTemperature(MONITOR_TEMP_PCB, (uint8_t)i);
    if (i < MONITOR_TEMP_MIN)
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_PCB), MONITOR_TEMP_LOW);
    } else
    if (i > MONITOR_TEMP_MAX)
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_PCB), MONITOR_TEMP_HIGH);
    } else
    {
      assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_PCB), i);
    }
    assert_int_equal(MONITOR_GetTemperature(MONITOR_TEMP_MOTOR), MONITOR_TEMP_LOW);
  }

  /**< Check temperature status */
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, MONITOR_TEMP_LOW);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, MONITOR_TEMP_LOW);
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 50);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, MONITOR_TEMP_LOW);
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, MONITOR_TEMP_LOW);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 50);
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 50);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 50);
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, MONITOR_TEMP_HIGH);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 50);
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 50);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, MONITOR_TEMP_HIGH);
  assert_true(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, MONITOR_TEMP_HIGH);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, MONITOR_TEMP_HIGH);
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_MOTOR));
  assert_false(MONITOR_GetTemperatureStatus(MONITOR_TEMP_PCB));
}

/**< Common parser check */
static void test_rs485_parser_common(void **state)
{
  (void) state;
  TCommData result;

  /**< Command is not defined */
  TCommData parser_wrong_cmd = {0xDF, 0, 0, 0};
  PARSER_ProcessRS485(&parser_wrong_cmd, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Set_Pos170 cmd */
static void test_rs485_parser_setpos170(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_setpos170 = {C_Set_Pos170, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_setpos170, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command */
  parser_cmd_setpos170.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_setpos170, &result);
  assert_int_equal(result.cmd, R_Set_Pos170);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0, 0), GLOBAL_TargetPos);
  /**< Another correct command */
  parser_cmd_setpos170.arg_1 = 0x01;
  parser_cmd_setpos170.arg_2 = 0x11;
  PARSER_ProcessRS485(&parser_cmd_setpos170, &result);
  assert_int_equal(result.cmd, R_Set_Pos170);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x01, 0x11), GLOBAL_TargetPos);
  /**< Another correct command, value too high */
  parser_cmd_setpos170.arg_1 = 0x07;
  parser_cmd_setpos170.arg_2 = 0xf0;
  PARSER_ProcessRS485(&parser_cmd_setpos170, &result);
  assert_int_equal(result.cmd, R_Set_Pos170);
  assert_int_equal(MAX_TARGET_POS, GLOBAL_TargetPos);
}

/**< Check C_Set_Pos170_Silent cmd */
static void test_rs485_parser_setpos170_silent(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_setpos170_silent = {C_Set_Pos170_Silent, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_setpos170_silent, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, no reply */
  parser_cmd_setpos170_silent.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_setpos170_silent, &result);
  assert_int_equal(result.cmd, 0);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0, 0), GLOBAL_TargetPos);
  /**< Another correct command, no reply */
  parser_cmd_setpos170_silent.arg_1 = 0x01;
  parser_cmd_setpos170_silent.arg_2 = 0x11;
  PARSER_ProcessRS485(&parser_cmd_setpos170_silent, &result);
  assert_int_equal(result.cmd, 0);
  assert_int_equal(CONVERSION_CalcPosFromSetPos170(0x01, 0x11), GLOBAL_TargetPos);
  /**< Another correct command, value too high, no reply */
  parser_cmd_setpos170_silent.arg_1 = 0x07;
  parser_cmd_setpos170_silent.arg_2 = 0xf0;
  PARSER_ProcessRS485(&parser_cmd_setpos170_silent, &result);
  assert_int_equal(result.cmd, 0);
  assert_int_equal(MAX_TARGET_POS, GLOBAL_TargetPos);
}

/**< Check C_Act_Pos170_report cmd */
static void test_rs485_parser_getpos170(void **state)
{
  (void) state;
  TCommData result;

  GLOBAL_RealPos = 0;
  /**< Wrong parameter */
  TCommData parser_cmd_getpos170 = {C_Act_Pos170_report, RS485_DEFAULT_ID, 1, 0};
  PARSER_ProcessRS485(&parser_cmd_getpos170, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command */
  parser_cmd_getpos170.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_getpos170, &result);
  assert_int_equal(result.cmd, R_Act_Pos170_report);
  assert_int_equal(GLOBAL_RealPos, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Another correct command */
  GLOBAL_RealPos = 0x111;
  PARSER_ProcessRS485(&parser_cmd_getpos170, &result);
  assert_int_equal(result.cmd, R_Act_Pos170_report);
  assert_int_equal(GLOBAL_RealPos, ((uint16_t)result.arg_1 << 8) + result.arg_2);
}

/**< Check C_Set_ID cmd */
static void test_rs485_parser_set_id(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_set_id = {C_SET_ID, RS485_BROADCAST_ID, 2, 2};
  PARSER_ProcessRS485(&parser_cmd_set_id, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but different parameters (1 and 2) */
  parser_cmd_set_id.id = RS485_DEFAULT_ID;
  parser_cmd_set_id.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_set_id, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but no access bit ??? */
//  parser_cmd_set_id.arg_1 = 2;
//  parser_cmd_set_id.arg_2 = 2;
//  PARSER_ProcessRS485(&parser_cmd_set_id, &result);
//  assert_int_equal(result.cmd, 0);
//  assert_int_equal(EE_StationId, RS485_DEFAULT_ID);
  /**< Set access bit */
  TCommData parser_cmd_set_access_bit = {C_Set_AccessBit, RS485_DEFAULT_ID, SIGN_ACCESS_1, SIGN_ACCESS_2};
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command */
  parser_cmd_set_id.arg_1 = 2;
  parser_cmd_set_id.arg_2 = 2;
  PARSER_ProcessRS485(&parser_cmd_set_id, &result);
  assert_int_equal(result.cmd, R_SET_ID);
  assert_int_equal(EE_StationId, 2);
  /**< Set default ID (for other tests) */
  EE_StationId = RS485_DEFAULT_ID;
}

/**< Check C_ID_report cmd */
static void test_rs485_parser_id_report(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast */
  TCommData parser_cmd_id_report = {C_ID_REPORT, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_id_report, &result);
  assert_int_equal(result.cmd, R_ID_REPORT);
  assert_int_equal(result.arg_1, EE_StationId);
  assert_int_equal(result.arg_2, EE_StationId);
  /**< Correct command, but wrong parameters */
  parser_cmd_id_report.id = RS485_DEFAULT_ID;
  parser_cmd_id_report.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_id_report, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong parameters */
  parser_cmd_id_report.id = RS485_DEFAULT_ID;
  parser_cmd_id_report.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_id_report, &result);
  assert_int_equal(result.cmd, R_ID_REPORT);
  assert_int_equal(result.arg_1, EE_StationId);
  assert_int_equal(result.arg_2, EE_StationId);
}

/**< Check C_Misc cmd */
static void test_rs485_parser_misc(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast */
  TCommData parser_cmd_misc = {C_Misc, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_misc, &result);
  assert_int_equal(result.cmd, 0);
  /**< Set correct ID */
  parser_cmd_misc.id = RS485_DEFAULT_ID;
  /**< Correct command, Get CRC of the bootloader */
  //parser_cmd_misc.id = RS485_DEFAULT_ID;
  //PARSER_ProcessRS485(&parser_cmd_misc, &result);
  //assert_int_equal(result.cmd, R_Misc);
  /**< Correct command, Get CRC of the firmware */
  //parser_cmd_misc.arg_1 = 1;
  //PARSER_ProcessRS485(&parser_cmd_misc, &result);
  //assert_int_equal(result.cmd, R_Misc);
  /**< Correct command, Get CRC of EEPROM */
  parser_cmd_misc.arg_1 = 2;
  PARSER_ProcessRS485(&parser_cmd_misc, &result);
  assert_int_equal(result.cmd, R_Misc);
  assert_int_equal(EEPROM_GetCRC(), ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, Recalculate CRC of EEPROM */
  parser_cmd_misc.arg_1 = 3;
  PARSER_ProcessRS485(&parser_cmd_misc, &result);
  assert_int_equal(result.cmd, R_Misc);
  assert_int_equal(EEPROM_GetCRC(), ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, Read CRC of EEPROM */
  parser_cmd_misc.arg_1 = 0x12;
  PARSER_ProcessRS485(&parser_cmd_misc, &result);
  assert_int_equal(result.cmd, R_Misc);
  assert_int_equal(EEPROM_GetCRC(), ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, Get default role (ACE1 or ACE2) */
  parser_cmd_misc.arg_1 = 0x20;
  PARSER_ProcessRS485(&parser_cmd_misc, &result);
  assert_int_equal(result.cmd, R_Misc);
  assert_int_equal(GLOBAL_InternalID, result.arg_1);
  assert_int_equal(GLOBAL_InternalID, result.arg_2);
}

/**< Check C_Test cmd */
static void test_rs485_parser_test(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast */
  TCommData parser_cmd_test = {C_Test, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, no signature byte */
  parser_cmd_test.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct commands */
  parser_cmd_test.arg_1 = SIGN_COMM_1;
  #ifdef DEF_DUPLEX
  /**< Correct command, opcode = 2 */
  GLOBAL_PartnerExtStatus = 0xAA55;
  parser_cmd_test.arg_2 = 2;
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, R_Test);
  assert_int_equal(CONVERSION_BuildOldStatus(GLOBAL_PartnerExtStatus), result.arg_1);
  assert_int_equal(CONVERSION_BuildOldStatus(GLOBAL_ExtStatus), result.arg_2);
  /**< Correct command, opcode = 3 */
  GLOBAL_PartnerPower = 0xAA;
  parser_cmd_test.arg_2 = 3;
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, R_Test);
  assert_int_equal(GLOBAL_Power, result.arg_2);
  #endif
  /**< Correct command, opcode = 4 */
  GLOBAL_ExternalStatus = 0x55AA;
  parser_cmd_test.arg_2 = 4;
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, R_Test);
  assert_int_equal(GLOBAL_ExternalStatus, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, opcode = 5 */
  GLOBAL_TargetPos = 0x1122;
  parser_cmd_test.arg_2 = 5;
  PARSER_ProcessRS485(&parser_cmd_test, &result);
  assert_int_equal(result.cmd, R_Test);
  assert_int_equal(GLOBAL_TargetPos, ((uint16_t)result.arg_1 << 8) + result.arg_2);
}

/**< Check C_Read_MP cmd */
static void test_rs485_parser_read_mp(void **state)
{
  (void) state;
  TCommData result;

  EE_Kd = 10;
  EEPROM_SaveVariable(&EE_Kd);
  /**< Correct command, broadcast is working too, wrong EEPROM MP addres (0x20..0x2F) */
  TCommData parser_cmd_read_mp = {C_Read_MP, RS485_BROADCAST_ID, 0x30, 0};
  PARSER_ProcessRS485(&parser_cmd_read_mp, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command */
  parser_cmd_read_mp.arg_1 = 0x20;
  PARSER_ProcessRS485(&parser_cmd_read_mp, &result);
  assert_int_equal(result.cmd, R_Read_MP);
  /**< Default Kd value = 10 */
  assert_int_equal(result.arg_1, 10);
  assert_int_equal(result.arg_2, 10);
}

/**< Check C_Write_MP cmd */
static void test_rs485_parser_write_mp(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast is working too, wrong EEPROM MP addres (0x20..0x2F) */
  TCommData parser_cmd_write_mp = {C_Write_MP, RS485_BROADCAST_ID, 0x30, 12};
  PARSER_ProcessRS485(&parser_cmd_write_mp, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, no broadcast, wrong EEPROM MP addres (0x20..0x2F) */
  parser_cmd_write_mp.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_write_mp, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, no broadcast, correct EEPROM MP addres, but no access bit */
  parser_cmd_write_mp.arg_1 = 0x20;
  PARSER_ProcessRS485(&parser_cmd_write_mp, &result);
  assert_int_equal(result.cmd, 0);
  /**< Set access bit */
  TCommData parser_cmd_set_access_bit = {C_Set_AccessBit, RS485_DEFAULT_ID, SIGN_ACCESS_1, SIGN_ACCESS_2};
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command */
  PARSER_ProcessRS485(&parser_cmd_write_mp, &result);
  assert_int_equal(result.cmd, R_Write_MP);
  assert_int_equal(result.arg_1, 0x20);
  assert_int_equal(result.arg_2, 12);
  assert_int_equal(EE_Kd, 12);
}

/**< Check C_RD_EEPROM_Lo cmd */
static void test_rs485_parser_read_eeprom(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast is working too, read Kd */
  TCommData parser_cmd_read_ee = {C_RD_EEPROM_Lo, RS485_BROADCAST_ID, 0, 0x20};
  PARSER_ProcessRS485(&parser_cmd_read_ee, &result);
  assert_int_equal(result.cmd, R_RD_EEPROM_Lo);
  assert_int_equal(result.arg_1, EE_Kd);
  assert_int_equal(result.arg_2, EE_Kd);
  /**< Arg_1 is always ignored */
  parser_cmd_read_ee.id = RS485_DEFAULT_ID;
  parser_cmd_read_ee.arg_1 = 0x11;
  PARSER_ProcessRS485(&parser_cmd_read_ee, &result);
  assert_int_equal(result.cmd, R_RD_EEPROM_Lo);
  assert_int_equal(result.arg_1, EE_Kd);
  assert_int_equal(result.arg_2, EE_Kd);
}

/**< Check C_WR_EEPROM_Lo cmd */
static void test_rs485_parser_write_eeprom(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, broadcast is working too, no access bit, write Kd */
  TCommData parser_cmd_write_ee = {C_WR_EEPROM_Lo, RS485_BROADCAST_ID, 0x20, 11};
  PARSER_ProcessRS485(&parser_cmd_write_ee, &result);
  assert_int_equal(result.cmd, 0);
  /**< Set access bit */
  TCommData parser_cmd_set_access_bit = {C_Set_AccessBit, RS485_DEFAULT_ID, SIGN_ACCESS_1, SIGN_ACCESS_2};
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, access bit is set */
  parser_cmd_write_ee.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_write_ee, &result);
  assert_int_equal(result.cmd, R_WR_EEPROM_Lo);
  assert_int_equal(result.arg_1, 0x20);
  assert_int_equal(result.arg_2, EE_Kd);
  assert_int_equal(EE_Kd, 11);
}

/**< Check C_Set_AccessBit cmd */
static void test_rs485_parser_set_access_bit(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast, allowed now! */
  TCommData parser_cmd_set_access_bit = {C_Set_AccessBit, RS485_BROADCAST_ID, SIGN_ACCESS_1, SIGN_ACCESS_2};
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  assert_int_equal(result.cmd, C_Set_AccessBit);
  assert_int_equal(result.arg_1, 'A');
  assert_int_equal(result.arg_2, 'E');
  /**< Correct command with default ID */
  parser_cmd_set_access_bit.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  assert_int_equal(result.cmd, C_Set_AccessBit);
  assert_int_equal(result.arg_1, 'A');
  assert_int_equal(result.arg_2, 'E');
  /**< Wrong signature byte */
  parser_cmd_set_access_bit.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Motor_Power cmd */
static void test_rs485_parser_motor_power(void **state)
{
  (void) state;
  TCommData result;

  GLOBAL_IsMaster = true;
  TCommData parser_cmd_set_access_bit = {C_Set_AccessBit, RS485_DEFAULT_ID, SIGN_ACCESS_1, SIGN_ACCESS_2};
  /**< Correct command, without access bit, allowed for not Growings! */
  TCommData parser_cmd_motor_power = {C_Motor_Power, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, motor free */
  parser_cmd_motor_power.arg_1 = 0;
  parser_cmd_motor_power.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_FREE);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, run with calculated power */
  parser_cmd_motor_power.arg_1 = 0x80;
  parser_cmd_motor_power.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, motor brake */
  parser_cmd_motor_power.arg_1 = 0x80;
  parser_cmd_motor_power.arg_2 = 0x01;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_BRAKE);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, rotate forward */
  parser_cmd_motor_power.arg_1 = 0xFD;
  parser_cmd_motor_power.arg_2 = 0x40;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_GOCW);
  assert_int_equal(MOTOR_GetExtPower(), 0x40);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, rotate forward */
  parser_cmd_motor_power.arg_1 = 0xBD;
  parser_cmd_motor_power.arg_2 = 0x60;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_GOCCW);
  assert_int_equal(MOTOR_GetExtPower(), 0x60);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, get motor status */
  MOTOR_SetMode(MOTOR_MODE_RUN1);
  parser_cmd_motor_power.arg_1 = 0x80;
  parser_cmd_motor_power.arg_2 = 0x02;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(((uint16_t)result.arg_1 << 8) + result.arg_2, 0x8000);
  /**< Set access bit */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  /**< Correct command, get motor status */
  MOTOR_SetMode(MOTOR_MODE_GOCW);
  MOTOR_SetExtPower(0x22);
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(result.arg_1, 0xFD);
  assert_int_equal(result.arg_2, MOTOR_GetExtPower());
  /**< Correct command, set external power (0x33) */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  parser_cmd_motor_power.arg_1 = 0x00;
  parser_cmd_motor_power.arg_2 = 0x33;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  assert_int_equal(MOTOR_GetExtPower(), 0x33);
  assert_true(GLOBAL_MotorExtMode);
  /**< Correct command, return to normal mode (PWM value from EEPROM) */
  PARSER_ProcessRS485(&parser_cmd_set_access_bit, &result);
  parser_cmd_motor_power.arg_1 = 0x80;
  parser_cmd_motor_power.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_motor_power, &result);
  assert_int_equal(result.cmd, R_Motor_Power);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  assert_false(GLOBAL_MotorExtMode);
}

/**< Check C_Read_Magnet cmd */
static void test_rs485_parser_read_magnet(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, Read absolute sensor position */
  GLOBAL_MagnetValue = 0xF030;
  TCommData parser_cmd_read_magnet = {C_Read_Magnet, RS485_DEFAULT_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(GLOBAL_MagnetValue, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, Read production compensated sensor position */
  GLOBAL_RealPos = 0x7050;
  parser_cmd_read_magnet.arg_1 = 1;
  parser_cmd_read_magnet.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(GLOBAL_RealPos, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  #ifdef DEF_DUPLEX
  /**< Correct command, Read absolute sensor position of other ACE */
  GLOBAL_PartnerMagnet = 0x3099;
  parser_cmd_read_magnet.arg_1 = 4;
  parser_cmd_read_magnet.arg_2 = 4;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(GLOBAL_PartnerMagnet, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, absolute sensor position of external (voting) position sensor */
  GLOBAL_MagnetExtHall = 0x6011;
  parser_cmd_read_magnet.arg_1 = 5;
  parser_cmd_read_magnet.arg_2 = 5;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(GLOBAL_MagnetExtHall, ((uint16_t)result.arg_1 << 8) + result.arg_2);
  /**< Correct command, perform sensor calibration */
  GLOBAL_DoSensorCalibration = 0;
  parser_cmd_read_magnet.arg_1 = 0x11;
  parser_cmd_read_magnet.arg_2 = 0x11;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(GLOBAL_DoSensorCalibration, SENS_CALIBR_VALUE);
  assert_int_equal(result.arg_1, SIGN_COMM_1);
  assert_int_equal(result.arg_2, SIGN_COMM_2);
  /**< Correct command, Read magnet sensor offset after calibration */
  EE_MagSensorOffs = 0xA5;
  parser_cmd_read_magnet.arg_1 = 0x12;
  parser_cmd_read_magnet.arg_2 = 0x12;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, R_Read_Magnet);
  assert_int_equal(result.arg_1, (uint8_t)EE_MagSensorOffs);
  assert_int_equal(result.arg_2, (uint8_t)EE_MagSensorOffs);
  #endif
  /**< Correct command, wrong parameter */
  parser_cmd_read_magnet.arg_1 = 0xFF;
  parser_cmd_read_magnet.arg_2 = 0xFF;
  PARSER_ProcessRS485(&parser_cmd_read_magnet, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Read_Serial_Num cmd */
static void test_rs485_parser_read_serial_num(void **state)
{
  (void) state;
  TCommData result;
  const char def_str[] = "SNUM-001";
  char str[STRINGS_MAX_LEN];
  uint8_t i;

  strcpy((char*)&EEPROM_Strings[EEPROM_ESN_ADDRESS], def_str);
  /**< Check old command */
  /**< Correct command, but broadcast, allowed now! */
  TCommData parser_cmd_read_sn = {C_Read_Serial_Num, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_sn, &result);
  assert_int_equal(result.cmd, R_Read_Serial_Num);
  /**< Correct command, but string too long */
  parser_cmd_read_sn.id = RS485_DEFAULT_ID;
  parser_cmd_read_sn.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_sn, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_sn.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_sn, &result);
    assert_int_equal(result.cmd, R_Read_Serial_Num);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);

  /**< Check Volz command */
  memset(str, 0, sizeof(str));
  /**< Correct command, but broadcast, allowed now */
  TCommData parser_cmd_read_sn_v = {C_Read_Serial_Num_V, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_sn_v, &result);
  assert_int_equal(result.cmd, R_Read_Serial_Num_V);
  /**< Correct command, but string too long */
  parser_cmd_read_sn_v.id = RS485_DEFAULT_ID;
  parser_cmd_read_sn_v.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_sn_v, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_sn_v.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_sn_v, &result);
    assert_int_equal(result.cmd, R_Read_Serial_Num_V);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);

}

/**< Check C_Read_Product_Descr cmd */
static void test_rs485_parser_read_descr(void **state)
{
  (void) state;
  TCommData result;
  const char def_str[] = "DA-58-Duplex";
  char str[STRINGS_MAX_LEN];
  uint8_t i;

  strcpy((char*)&EEPROM_Strings[EEPROM_PROD_ADDRESS], def_str);
  /**< Check old command */
  /**< Correct command, but broadcast, allowed now */
  TCommData parser_cmd_read_descr = {C_Read_Product_Descr, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_descr, &result);
  assert_int_equal(result.cmd, R_Read_Product_Descr);
  /**< Correct command, but string too long */
  parser_cmd_read_descr.id = RS485_DEFAULT_ID;
  parser_cmd_read_descr.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_descr, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_descr.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_descr, &result);
    assert_int_equal(result.cmd, R_Read_Product_Descr);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);

  /**< Check Volz command */
  memset(str, 0, sizeof(str));
  /**< Correct command, but broadcast, allowed now! */
  TCommData parser_cmd_read_descr_v = {C_Read_Product_Descr_V, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_descr_v, &result);
  assert_int_equal(result.cmd, R_Read_Product_Descr_V);
  /**< Correct command, but string too long */
  parser_cmd_read_descr_v.id = RS485_DEFAULT_ID;
  parser_cmd_read_descr_v.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_descr_v, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_descr_v.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_descr_v, &result);
    assert_int_equal(result.cmd, R_Read_Product_Descr_V);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);
}

/**< Check C_Read_Firmware_Num cmd */
static void test_rs485_parser_read_fw_num(void **state)
{
  (void) state;
  TCommData result;
  const char def_str[] = "FW01.01";
  char str[STRINGS_MAX_LEN];
  uint8_t i;

  strcpy((char*)&EEPROM_Strings[EEPROM_FWREV_ADDRESS], def_str);
  /**< Check old command */
  /**< Correct command, but broadcast, allowed now! */
  TCommData parser_cmd_read_fw = {C_Read_Firmware_Num, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_fw, &result);
  assert_int_equal(result.cmd, R_Read_Firmware_Num);
  /**< Correct command, but string too long */
  parser_cmd_read_fw.id = RS485_DEFAULT_ID;
  parser_cmd_read_fw.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_fw, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_fw.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_fw, &result);
    assert_int_equal(result.cmd, R_Read_Firmware_Num);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);

  /**< Check Volz command */
  memset(str, 0, sizeof(str));
  /**< Correct command, but broadcast, allowed now! */
  TCommData parser_cmd_read_fw_v = {C_Read_Firmware_Num_V, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_fw_v, &result);
  assert_int_equal(result.cmd, R_Read_Firmware_Num_V);
  /**< Correct command, but string too long */
  parser_cmd_read_fw_v.id = RS485_DEFAULT_ID;
  parser_cmd_read_fw_v.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_read_fw_v, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_read_fw_v.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_read_fw_v, &result);
    assert_int_equal(result.cmd, R_Read_Firmware_Num_V);
    assert_int_equal(result.arg_2, strlen(def_str));
    str[i] = result.arg_1;
  }
  str[i] = 0;
  assert_string_equal(str, def_str);
}

/**< Check C_PowerUp_Counter cmd, PowerUps counter is also tested here */
static void test_rs485_parser_read_powerups(void **state)
{
  (void) state;
  TCommData result;
  uint32_t u32;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_powerups = {C_PowerUp_Counter, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong arguments */
  parser_cmd_read_powerups.id = RS485_DEFAULT_ID;
  parser_cmd_read_powerups.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, 0);
  u32 = 10;
  memcpy(&EEPROM_Counters[EEPROM_POWERUPS_ADDR], &u32, sizeof(uint32_t));
  /**< Correct command */
  parser_cmd_read_powerups.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, R_PowerUp_Counter);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 10);
  u32 = 0x20000;
  memcpy(&EEPROM_Counters[EEPROM_POWERUPS_ADDR], &u32, sizeof(uint32_t));
  /**< Correct command, overflow */
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, R_PowerUp_Counter);
  assert_int_equal(result.arg_1, 0xff);
  assert_int_equal(result.arg_2, 0xff);
  /**< Correct command, reset counter */
  EEPROM_ResetPowerUps();
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, R_PowerUp_Counter);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 0);
  /**< Correct command, increment counter */
  EEPROM_IncPowerUps();
  EEPROM_IncPowerUps();
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, R_PowerUp_Counter);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 2);
  /**< Correct command, Volz */
  u32 = 10;
  memcpy(&EEPROM_Counters[EEPROM_POWERUPS_ADDR], &u32, sizeof(uint32_t));
  parser_cmd_read_powerups.cmd = C_PowerUp_Counter_V;
  parser_cmd_read_powerups.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_powerups, &result);
  assert_int_equal(result.cmd, R_PowerUp_Counter_V);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 10);

}

/**< Check C_Runtimer and C_No_Load_Runtimer/C_Load_Runtimer_xx cmds, Runtimer counters are also tested here */
static void test_rs485_parser_read_runtimers(void **state)
{
  (void) state;
  TCommData result;
  uint32_t u32;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_runtimer = {C_Runtimer, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong arguments */
  parser_cmd_read_runtimer.id = RS485_DEFAULT_ID;
  parser_cmd_read_runtimer.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command */
  u32 = 3611;
  memcpy(&EEPROM_Counters[COUNTER_LOAD_ALL], &u32, sizeof(uint32_t));
  /**< Read hours */
  parser_cmd_read_runtimer.arg_1 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 1);
  /**< Read minutes/seconds */
  parser_cmd_read_runtimer.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 11);
  /**< Increment counter */
  EEPROM_IncCounter(COUNTER_LOAD_ALL);
  /**< Read hours */
  parser_cmd_read_runtimer.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 1);
  /**< Read minutes/seconds */
  parser_cmd_read_runtimer.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 12);
  /**< Reset counter */
  EEPROM_ResetCounter(COUNTER_LOAD_ALL);
  /**< Read hours */
  parser_cmd_read_runtimer.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 0);
  /**< Read minutes/seconds */
  parser_cmd_read_runtimer.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 0);
  /**< Correct command */
  u32 = 3611;
  memcpy(&EEPROM_Counters[COUNTER_LOAD_ALL], &u32, sizeof(uint32_t));
  /**< Read hours, Volz */
  parser_cmd_read_runtimer.cmd = C_Runtimer_V;
  parser_cmd_read_runtimer.arg_1 = 0;
  parser_cmd_read_runtimer.arg_2 = 0;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer_V);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 1);
  /**< Read minutes/seconds, Volz */
  parser_cmd_read_runtimer.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_runtimer, &result);
  assert_int_equal(result.cmd, R_Runtimer_V);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 11);
}

/**< Check C_Write_Serial_Num: cmd */
static void test_rs485_parser_write_serial_num(void **state)
{
  (void) state;
  TCommData result;
  const char def_str[] = "SNUM-999";
  uint8_t i;

  memset((char*)&EEPROM_Strings[EEPROM_ESN_ADDRESS], 0, STRINGS_MAX_LEN);
  /**< Correct command, but broadcast */
  TCommData parser_cmd_write_sn = {C_Write_Serial_Num, RS485_BROADCAST_ID, 0, 0};
  PARSER_ProcessRS485(&parser_cmd_write_sn, &result);
  assert_int_equal(result.cmd, R_Write_Serial_Num);
  /**< Correct command, but string too long */
  parser_cmd_write_sn.id = RS485_DEFAULT_ID;
  parser_cmd_write_sn.arg_2 = STRINGS_MAX_LEN;
  PARSER_ProcessRS485(&parser_cmd_write_sn, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, read the whole string */
  for (i = 0; i < strlen(def_str); i++)
  {
    parser_cmd_write_sn.arg_1 = def_str[i];
    parser_cmd_write_sn.arg_2 = i;
    PARSER_ProcessRS485(&parser_cmd_write_sn, &result);
    assert_int_equal(result.cmd, R_Write_Serial_Num);
    assert_int_equal(result.arg_1, def_str[i]);
    assert_int_equal(result.arg_2, i);
  }
  assert_string_equal(&EEPROM_Strings[EEPROM_ESN_ADDRESS], def_str);
}

/**< Check C_Read_Current cmd */
static void test_rs485_parser_read_current(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_current = {C_Read_Current, RS485_BROADCAST_ID , 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_current, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command with default ID */
  //will_return(MONITOR_GetCurrent, 0x80);
  //will_return(MONITOR_GetCurrent, 0x90);
  for (int i = 0; i < 50; i++)
    MONITOR_SetCurrent(0x80);
  parser_cmd_read_current.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_read_current, &result);
  assert_int_equal(result.cmd, R_Read_Current);
  assert_int_equal(result.arg_1, 0x80);
  assert_int_equal(result.arg_2, 0x80);
  #ifdef DEF_DUPLEX
  /**< Correct command with default ID, partner */
  GLOBAL_PartnerI = 0x10;
  parser_cmd_read_current.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_current, &result);
  assert_int_equal(result.cmd, R_Read_Current);
  assert_int_equal(result.arg_1, 0x10);
  assert_int_equal(result.arg_2, 0x10);
  #endif
  /**< Correct command, but wrong first argument */
  parser_cmd_read_current.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_current, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong second argument */
  parser_cmd_read_current.arg_1 = 0;
  parser_cmd_read_current.arg_2 = 2;
  PARSER_ProcessRS485(&parser_cmd_read_current, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Read_Voltage cmd */
static void test_rs485_parser_read_voltage(void **state)
{
  (void) state;
  TCommData result;

  MONITOR_SetVoltage(MONITOR_U_CH1, 0x81);
  MONITOR_SetVoltage(MONITOR_U_CH2, 0x91);
  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_voltage = {C_Read_Voltage, RS485_BROADCAST_ID , 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_voltage, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command with default ID */
  parser_cmd_read_voltage.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_read_voltage, &result);
  assert_int_equal(result.cmd, R_Read_Voltage);
  assert_int_equal(result.arg_1, 0x81);
  assert_int_equal(result.arg_2, 0x91);
  #ifdef DEF_DUPLEX
  /**< Correct command with default ID, partner */
  GLOBAL_PartnerU1 = 0x10;
  GLOBAL_PartnerU2 = 0x11;
  parser_cmd_read_voltage.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_voltage, &result);
  assert_int_equal(result.cmd, R_Read_Voltage);
  assert_int_equal(result.arg_1, 0x10);
  assert_int_equal(result.arg_2, 0x11);
  #endif
  /**< Correct command, but wrong first argument */
  parser_cmd_read_voltage.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_voltage, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong second argument */
  parser_cmd_read_voltage.arg_1 = 0;
  parser_cmd_read_voltage.arg_2 = 2;
  PARSER_ProcessRS485(&parser_cmd_read_voltage, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Read_Temperature cmd */
static void test_rs485_parser_read_temperature(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_temperature = {C_Read_Temperature, RS485_BROADCAST_ID , 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command with default ID */
  parser_cmd_read_temperature.id = RS485_DEFAULT_ID;
  //will_return(MONITOR_GetTemperature, 0x80);
  //will_return(MONITOR_GetTemperature, 0x90);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 0x90);
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, R_Read_Temperature);
  assert_int_equal(result.arg_1, 0x80);
  assert_int_equal(result.arg_2, 0x90);
  /**< Correct command with default ID and Volz command set */
  parser_cmd_read_temperature.cmd = C_Read_Temperature_V;
  //will_return(MONITOR_GetTemperature, 0x80);
  //will_return(MONITOR_GetTemperature, 0x90);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_PCB, 0x90);
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, R_Read_Temperature_V);
  assert_int_equal(result.arg_1, 0x80);
  assert_int_equal(result.arg_2, 0x90);
  #ifdef DEF_DUPLEX
  /**< Correct command with default ID, partner */
  parser_cmd_read_temperature.cmd = C_Read_Temperature;
  GLOBAL_PartnerTempM = 0x10;
  GLOBAL_PartnerTempP = 0x11;
  parser_cmd_read_temperature.arg_2 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, R_Read_Temperature);
  assert_int_equal(result.arg_1, 0x10);
  assert_int_equal(result.arg_2, 0x11);
  #endif
  /**< Correct command, but wrong first argument */
  parser_cmd_read_temperature.arg_1 = 1;
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command, but wrong second argument */
  parser_cmd_read_temperature.arg_1 = 0;
  parser_cmd_read_temperature.arg_2 = 2;
  PARSER_ProcessRS485(&parser_cmd_read_temperature, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Read_Humidity cmd */
static void test_rs485_parser_read_humidity(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command, but broadcast */
  TCommData parser_cmd_read_humidity = {C_Read_Humidity, RS485_BROADCAST_ID , 0, 0};
  PARSER_ProcessRS485(&parser_cmd_read_humidity, &result);
  assert_int_equal(result.cmd, 0);
  /**< Correct command with default ID */
  parser_cmd_read_humidity.id = RS485_DEFAULT_ID;
  PARSER_ProcessRS485(&parser_cmd_read_humidity, &result);
  assert_int_equal(result.cmd, R_Read_Humidity);
  assert_int_equal(result.arg_1, 0);
  assert_int_equal(result.arg_2, 0);
}

/**< Check C_SetDefault cmd */
static void test_rs485_parser_set_default(void **state)
{
  (void) state;
  TCommData result;

  /**< Correct command */
  TCommData parser_cmd_set_default = {C_SetDefault, RS485_DEFAULT_ID, SIGN_DFLT_1, SIGN_DFLT_2};
  PARSER_ProcessRS485(&parser_cmd_set_default, &result);
  assert_int_equal(result.cmd, R_SetDefault);
  assert_int_equal(result.arg_1, SIGN_DFLT_1);
  assert_int_equal(result.arg_2, SIGN_DFLT_2);
  /**< Wrong signature bytes */
  parser_cmd_set_default.arg_1 = SIGN_ACCESS_1;
  parser_cmd_set_default.arg_2 = SIGN_ACCESS_2;
  PARSER_ProcessRS485(&parser_cmd_set_default, &result);
  assert_int_equal(result.cmd, 0);
}

/**< Check C_Read_Revision_Str cmd */
static void test_rs485_parser_read_revision(void **state)
{
  (void) state;
  TCommData result;
  uint8_t len;
  uint8_t i;
  char str[256] = {0};

  TCommData parser_cmd_read_revision = {C_Read_Revision_Str, RS485_DEFAULT_ID, 0xFF, 0};
  PARSER_ProcessRS485(&parser_cmd_read_revision, &result);
  assert_int_equal(result.cmd, R_Read_Revision_Str);
  assert_int_equal(result.arg_1, 0xFF);
  len = result.arg_2;
  for (i = 0; i < len; i++)
  {
    parser_cmd_read_revision.arg_1 = i;
    PARSER_ProcessRS485(&parser_cmd_read_revision, &result);
    assert_int_equal(result.cmd, R_Read_Revision_Str);
    assert_int_equal(result.arg_1, i);
    str[i] = (char)result.arg_2;
  }
  str[i] = 0;
  char str_compare[] = "{\"CI\":{\"Version\":\"";
  str[strlen(str_compare)] = 0;
  assert_string_equal(str, str_compare);
}

/**< Check MVS (Mid-Value Select algorithm) */
static void test_mvs(void **state)
{
  (void) state;

  assert_int_equal((int)MVS_Voting(1, 2, 3), 2);
  assert_int_equal((int)MVS_Voting(1, 2, 2), 2);
  assert_int_equal((int)MVS_Voting(1, 2, 1), 1);
  assert_int_equal((int)MVS_Voting(-1, 2, 2), 2);
  assert_int_equal((int)MVS_Voting(-1, -1, 2), -1);
  assert_int_equal((int)MVS_Voting(1, 100, 2), 2);
  assert_int_equal((int)MVS_Voting(1, 100, 101), 100);
}

/**< Check Volz Echo command */
static void test_volz_echo(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint8_t buff[4] = {0x11, 0x22, 0x33, 0x44};

  if (EE_DevId == 0xFF)
  {
    EE_DevId = 0;
  }

  rx_msg.data = rx_data;
  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 1 + sizeof(buff);
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_ECHO;
  memcpy(&tx_msg.data[CANMSG_OFFSET_DATA], buff, sizeof(buff));
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(tx_msg.len, rx_msg.len);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_ECHO);
  assert_memory_equal(&rx_data[CANMSG_OFFSET_DATA], buff, sizeof(buff));
  #endif
}

/**< Check Volz SetPos command */
static void test_volz_set_pos(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint16_t u16;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 1 + sizeof(uint16_t);
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_SET_POS;
  /**< Set position 0� */
  //will_return(MONITOR_GetCurrent, 0x40);
  MONITOR_SetCurrent(0x40);
  //will_return(MONITOR_GetTemperature, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  u16 = 0;
  memcpy(&tx_msg.data[CANMSG_OFFSET_DATA], &u16, sizeof(uint16_t));
  GLOBAL_RealPos = 0;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_SET_POS);
  assert_memory_equal(&rx_data[CANMSG_OFFSET_DATA], &tx_msg.data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(GLOBAL_TargetPos, 0);
  /**< Set position 45� */
  //will_return(MONITOR_GetCurrent, 0x40);
  //will_return(MONITOR_GetVoltage, 0x80);
  //will_return(MONITOR_GetTemperature, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  u16 = 450;
  memcpy(&tx_msg.data[CANMSG_OFFSET_DATA], &u16, sizeof(uint16_t));
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_SET_POS);
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(u16, CONVERSION_CalcDegreesFromPos(GLOBAL_RealPos));
  assert_int_equal(GLOBAL_TargetPos, CONVERSION_CalcPosFromDegrees(450));
  /**< Set position 90� */
  //will_return(MONITOR_GetCurrent, 0x40);
  MONITOR_SetCurrent(0x40);
  //will_return(MONITOR_GetTemperature, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  u16 = 900;
  memcpy(&tx_msg.data[CANMSG_OFFSET_DATA], &u16, sizeof(uint16_t));
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_SET_POS);
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(u16, CONVERSION_CalcDegreesFromPos(GLOBAL_RealPos));
  assert_int_equal(GLOBAL_TargetPos, CONVERSION_CalcPosFromDegrees(900));
  /**< Set position 175�, must be 170� after execution */
  //will_return(MONITOR_GetCurrent, 0x40);
  MONITOR_SetCurrent(0x40);
  //will_return(MONITOR_GetTemperature, 0x80);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  u16 = 1750;
  memcpy(&tx_msg.data[CANMSG_OFFSET_DATA], &u16, sizeof(uint16_t));
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_SET_POS);
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(u16, CONVERSION_CalcDegreesFromPos(GLOBAL_RealPos));
  assert_int_equal(GLOBAL_TargetPos, CONVERSION_CalcPosFromDegrees(1700));
  #endif
}

/**< Check Volz GetPos command */
static void test_volz_get_pos(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint16_t u16;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 1;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_POS;
  /**< Get position */
  for (int i = 0; i < 50; i++)
    MONITOR_SetCurrent(0x40);
  MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, 0x80);
  MONITOR_SetVoltage(MONITOR_U_CH1, 0x81);
  //GLOBAL_Power = 0x10;
  MOTOR_SetMode(MOTOR_MODE_FREE);
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_POS);
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(u16, CONVERSION_CalcDegreesFromPos(GLOBAL_RealPos));
  /**< Current value */
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 3], 0x40);
  /**< Voltage#1 value */
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 4], 0x81);
  /**< Motor PWM value, free-running mode = 0 */
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 5], 0);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 6], 0x80);
  #endif
}

/**< Check Volz GetTime command */
static void test_volz_get_time(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint32_t u32;

  rx_msg.data = rx_data;

  /**< Get working time */
  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 1;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_TIME;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_TIME);
  memcpy(&u32, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint32_t));
  assert_int_equal(u32 & 0xFFFFFF, GLOBAL_WorkingTime);
  memcpy(&u32, &rx_data[CANMSG_OFFSET_DATA + 3], sizeof(uint32_t));
  assert_int_equal(u32, EEPROM_GetCounter(COUNTER_LOAD_ALL));
  #endif
}

/**< Check Volz GetParam command */
static void test_volz_get_param(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN + 1];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  eeVal_t param;
  uint8_t i;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_PARAM;
  for (i = 0; i < EEPROM_GetParamCount(); i++)
  {
    /**< Read name of parameter i */
    tx_msg.len = 2;
    memset(&tx_msg.data[CANMSG_OFFSET_DATA], 0, CANBUS_BUF_LEN - CANMSG_OFFSET_DATA);
    tx_msg.data[CANMSG_OFFSET_DATA] = i | 0x80;
    memset(rx_data, 0, CANBUS_BUF_LEN + 1);
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 8);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i | 0x80);
    assert_true(EEPROM_GetParam(i, &param));
    assert_string_equal(param.name, &rx_data[CANMSG_OFFSET_DATA + 1]);
    /**< Read value of parameter i */
    tx_msg.len = 3;
    tx_msg.data[CANMSG_OFFSET_DATA] = i;
    tx_msg.data[CANMSG_OFFSET_DATA + 1] = 0;
    memset(rx_data, 0, CANBUS_BUF_LEN + 1);
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3 + param.size);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], 0 | param.type);
    /**< Read minimal value of parameter i */
    tx_msg.data[CANMSG_OFFSET_DATA + 1] = (CANMSG_PARAM_MINVAL << CANMSG_PARAM_OPCODE_OFFS);
    memset(rx_data, 0, CANBUS_BUF_LEN + 1);
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3 + param.size);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], (CANMSG_PARAM_MINVAL << CANMSG_PARAM_OPCODE_OFFS) | param.type);
    /**< Read maximal value of parameter i */
    tx_msg.data[CANMSG_OFFSET_DATA + 1] = (CANMSG_PARAM_MAXVAL << CANMSG_PARAM_OPCODE_OFFS);
    memset(rx_data, 0, CANBUS_BUF_LEN + 1);
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3 + param.size);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], (CANMSG_PARAM_MAXVAL << CANMSG_PARAM_OPCODE_OFFS) | param.type);
    /**< Read default value of parameter i */
    tx_msg.data[CANMSG_OFFSET_DATA + 1] = (CANMSG_PARAM_DFLTVAL << CANMSG_PARAM_OPCODE_OFFS);
    memset(rx_data, 0, CANBUS_BUF_LEN + 1);
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3 + param.size);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], (CANMSG_PARAM_DFLTVAL << CANMSG_PARAM_OPCODE_OFFS) | param.type);
  }
  /**< Read name of not-existing parameter */
  tx_msg.len = 2;
  memset(&tx_msg.data[CANMSG_OFFSET_DATA], 0, CANBUS_BUF_LEN - CANMSG_OFFSET_DATA);
  tx_msg.data[CANMSG_OFFSET_DATA] = 0x7f | 0x80;
  memset(rx_data, 0, CANBUS_BUF_LEN + 1);
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 8);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_PARAM);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0x7f | 0x80);
  uint32_t u32;
  memcpy(&u32, &rx_data[CANMSG_OFFSET_DATA + 1], sizeof(uint32_t));
  assert_int_equal(u32, 0);
  #endif
}

/**< Check Volz SetParam command */
static void test_volz_set_param(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN + 1];
  struct can_message rx_msg;
  TCanMsg tx_msg;

  rx_msg.data = rx_data;

  /**< Write value of parameter 3 (Kd) */
  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_SET_PARAM;
  tx_msg.data[CANMSG_OFFSET_DATA] = 3;
  tx_msg.data[CANMSG_OFFSET_DATA + 1] = (PARAM_TYPE_UINT8 << CANMSG_PARAM_TYPE_OFFS);
  tx_msg.data[CANMSG_OFFSET_DATA + 2] = 0x12;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], (PARAM_TYPE_UINT8 << CANMSG_PARAM_TYPE_OFFS));
  assert_int_equal(EE_Kd, 0x12);
  #endif
}

/**< Check Volz GetHWFW command */
static void test_volz_get_hwfw(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint16_t u16;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 1;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_HWFW;
  PARSER_CanVolz(&tx_msg, &rx_msg);
  assert_int_equal(rx_msg.len, 7);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_HWFW);
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA], sizeof(uint16_t));
  assert_int_equal(u16, *VERSION_GetFW());
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA + 2], sizeof(uint16_t));
  assert_int_equal(u16, *VERSION_GetHW());
  memcpy(&u16, &rx_data[CANMSG_OFFSET_DATA + 4], sizeof(uint16_t));
  assert_int_equal(u16, EEPROM_GetPowerUps());
  #endif
}

/**< Check Volz GetSetString command */
static void test_volz_getset_string(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint8_t i;
  char str[] = "VOLZ-SERVOS";
  char str2[16];
  char ch;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  /**< Write to EEPROM strings */
  tx_msg.len = 3;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_SET_STRING;
  for (i = 0; i < strlen(str); i++)
  {
    tx_msg.data[CANMSG_OFFSET_DATA] = i;
    tx_msg.data[CANMSG_OFFSET_DATA + 1] = str[i];
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_SET_STRING);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], str[i]);
  }
  /**< Read from EEPROM strings */
  memset(str2, 0, sizeof(str2));
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_STRING;
  for (i = 0; i < sizeof(str2); i++)
  {
    tx_msg.data[CANMSG_OFFSET_DATA] = i;
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_STRING);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    if (rx_data[CANMSG_OFFSET_DATA + 1] == 0xff)
      ch = 0;
    else
      ch = rx_data[CANMSG_OFFSET_DATA + 1];
    str2[i] = ch;
    if (ch == 0)
      break;
  }
  /**< Compare strings */
  assert_string_equal(str, str2);
  #endif
}

/**< Check Volz ExecuteOp command */
static void test_volz_execute_op(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_EXECUTE;
  MOTOR_SetMode(MOTOR_MODE_FREE);
  /**< Execute operation: normal mode */
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_EXECUTE_NORMMODE;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_EXECUTE_NORMMODE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_RUN1);
  /**< Execute operation: free-running mode */
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_EXECUTE_FREE;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_EXECUTE_FREE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_FREE);
  /**< Execute operation: continuous CW rotation mode */
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_EXECUTE_ROTCW;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_EXECUTE_ROTCW);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_GOCW);
  /**< Execute operation: continuous CCW rotation mode */
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_EXECUTE_ROTCCW;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_EXECUTE_ROTCCW);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(MOTOR_GetMode(), MOTOR_MODE_GOCCW);
  /**< Execute operation: set neutral position */

  /**< Execute operation: reset status */
  GLOBAL_MyStatus = 0x1111;
  GLOBAL_ExternalStatus = 0x1111;
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_EXECUTE_RESETSTS;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_EXECUTE_RESETSTS);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(GLOBAL_MyStatus, 0);
  assert_int_equal(GLOBAL_ExternalStatus, 0);
  /**< Execute operation: error */
  tx_msg.data[CANMSG_OFFSET_DATA] = 0xff;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_EXECUTE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0xff);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_ERR);

  #endif
}

/**< Check Volz Duplex command */
static void test_volz_duplex(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  #ifdef DEF_DUPLEX
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_DUPLEX;

  /**< Do role forcing: master */
  GLOBAL_RoleForce = ROLE_FORCE_NONE;
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_DUPLEX_FORCEMASTER;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_DUPLEX);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_DUPLEX_FORCEMASTER);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(GLOBAL_RoleForce, ROLE_FORCE_MASTER);
  /**< Do role forcing: slave */
  GLOBAL_RoleForce = ROLE_FORCE_NONE;
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_DUPLEX_FORCESLAVE;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_DUPLEX);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_DUPLEX_FORCESLAVE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_OK);
  assert_int_equal(GLOBAL_RoleForce, ROLE_FORCE_SLAVE);

  /**< Wrong parameter */
  tx_msg.data[CANMSG_OFFSET_DATA] = CANMSG_DUPLEX_FORCESLAVE + 1;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_DUPLEX);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], CANMSG_DUPLEX_FORCESLAVE + 1);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], CANMSG_REPLY_ERR);
  #endif
  #endif
}

/**< Check Volz GetCounter command */
static void test_volz_get_counter(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  #ifdef DEF_DUPLEX
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint32_t uval32;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_COUNTER;

  /**< Counter Ids: 0 to 100 */
  for (uint8_t id = COUNTER_LOAD_0; id < COUNTER_LOAD_100; id++)
  {
    tx_msg.data[CANMSG_OFFSET_DATA] = id;
    for (uint8_t i = 0; i < 100; i++)
    {
      EEPROM_IncCounter(id);
      assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
      assert_int_equal(rx_msg.len, 6);
      assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_COUNTER);
      assert_int_equal(rx_data[CANMSG_OFFSET_DATA], id);
      memcpy(&uval32, &rx_data[CANMSG_OFFSET_DATA + 1U], sizeof(uint32_t));
      assert_int_equal(uval32, EEPROM_GetCounter(id));
    }
  }
  /**< Wrong counter Id */
  tx_msg.data[CANMSG_OFFSET_DATA] = COUNTER_LOAD_LAST;
  assert_false(PARSER_CanVolz(&tx_msg, &rx_msg));
  #endif
  #endif
}

/**< Check Volz GetRevision command */
static void test_volz_get_revision(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;
  uint8_t len;
  uint8_t i;
  char str[256];

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_GET_REVISION;
  tx_msg.data[CANMSG_OFFSET_DATA] = 0xFF;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_REVISION);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0xFF);
  len = rx_data[CANMSG_OFFSET_DATA + 1];

  for (i = 0; i < len; i++)
  {
    tx_msg.data[CANMSG_OFFSET_CMD + 1] = i;
    assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
    assert_int_equal(rx_msg.len, 3);
    assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_GET_REVISION);
    assert_int_equal(rx_data[CANMSG_OFFSET_DATA], i);
    str[i] = (char)rx_data[CANMSG_OFFSET_DATA + 1];
  }
  str[i] = 0;
  char str_compare[] = "{\"CI\":{\"Version\":\"";
  str[strlen(str_compare)] = 0;
  assert_string_equal(str, str_compare);
  #endif
}

/**< Check Volz ReadEE command */
static void test_volz_read_ee(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;

  rx_msg.data = rx_data;

  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 2;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_READ_EE;
  tx_msg.data[CANMSG_OFFSET_DATA] = 0x20;
  /**< Read EEPROM value */
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_READ_EE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0x20);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], EE_Kd);
  tx_msg.data[CANMSG_OFFSET_DATA] = 0x21;
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_READ_EE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0x21);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], EE_Kp);
  #endif
}

/**< Check Volz WriteEE command */
static void test_volz_write_ee(void **state)
{
  (void) state;
  #if !defined DEF_PIPISTREL && !defined DEF_VERTICAL
  uint8_t rx_data[CANBUS_BUF_LEN];
  struct can_message rx_msg;
  TCanMsg tx_msg;

  rx_msg.data = rx_data;
  tx_msg.id = EE_CanId | EE_DevId;
  tx_msg.fmt = CAN_FMT_STDID;
  tx_msg.len = 3;
  memset(tx_msg.data, 0, CANBUS_BUF_LEN);
  memset(rx_data, 0, CANBUS_BUF_LEN);
  tx_msg.data[CANMSG_OFFSET_CMD] = CANMSG_CMD_WRITE_EE;
  tx_msg.data[CANMSG_OFFSET_DATA] = 0x20;
  tx_msg.data[CANMSG_OFFSET_DATA + 1] = 0x10;
  /**< Write EEPROM value */
  assert_true(PARSER_CanVolz(&tx_msg, &rx_msg));
  assert_int_equal(rx_msg.len, 3);
  assert_int_equal(rx_data[CANMSG_OFFSET_CMD], CANMSG_CMD_WRITE_EE);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA], 0x20);
  assert_int_equal(rx_data[CANMSG_OFFSET_DATA + 1], 0x10);
  assert_int_equal(EEPROM_Page[0x20], 0x10);
  assert_int_equal(EE_Kd, 0x10);
  #endif
}

/**< List of tests */
const struct CMUnitTest tests[] =
{
  /**< Conversion module */
  cmocka_unit_test(test_conversion_setpos170),
  cmocka_unit_test(test_conversion_setpos100),
  cmocka_unit_test(test_conversion_setposext),
  cmocka_unit_test(test_conversion_temperature),
  cmocka_unit_test(test_conversion_current),
  cmocka_unit_test(test_conversion_voltage),
  cmocka_unit_test(test_conversion_volz),
  /**< Utils module */
  cmocka_unit_test(test_utils_getlength),
  cmocka_unit_test(test_utils_sign),
  cmocka_unit_test(test_utils_limit),
  cmocka_unit_test(test_utils_inttostr),
  cmocka_unit_test(test_utils_floattostr),
  /**< Redundancy module */
  cmocka_unit_test(test_redundancy),
  /**< Notch filter module */
  cmocka_unit_test(test_notch_init),
  cmocka_unit_test(test_notch_calculation),
  /**< CRC calculation module */
  cmocka_unit_test(test_crc32),
  cmocka_unit_test(test_crc16),
  /**< EEPROM module */
  cmocka_unit_test(test_eeprom_init),
  cmocka_unit_test(test_eeprom_write),
  cmocka_unit_test(test_eeprom_writebyte),
  cmocka_unit_test(test_eeprom_readbyte),
  cmocka_unit_test(test_eeprom_getparam),
  /**< Magnet sensor module */
  cmocka_unit_test(test_magnet_read),
  /**< 2oo3 voting module */
  cmocka_unit_test(test_voting_get_dist),
  cmocka_unit_test(test_voting_process),
  /**< Hall sensors module */
  cmocka_unit_test(test_hall_sensors),
  cmocka_unit_test(test_hall_windings),
  /**< Inetwork module */
  cmocka_unit_test(test_inetwork_send),
  cmocka_unit_test(test_inetwork_parse),
  /**< Logic functions */
  cmocka_unit_test(test_logic_default_values),
  cmocka_unit_test(test_logic_timeouts),
  cmocka_unit_test(test_logic_load_counters),
  cmocka_unit_test(test_logic_reset_pwm),
  /**< Motor functions module */
  cmocka_unit_test(test_motor_process_magnet),
  cmocka_unit_test(test_motor_calc_power),
  cmocka_unit_test(test_motor_damping),
  cmocka_unit_test(test_motor_saver),
  cmocka_unit_test(test_motor_heater),
  cmocka_unit_test(test_motor_current),
  cmocka_unit_test(test_motor_mode),
  /**< Monitor functions module */
  cmocka_unit_test(test_monitor_current),
  cmocka_unit_test(test_monitor_voltage),
  cmocka_unit_test(test_monitor_temperature),
  /**< Volz RS485 protocol */
  cmocka_unit_test(test_rs485_parser_common),
  cmocka_unit_test(test_rs485_parser_setpos170),
  cmocka_unit_test(test_rs485_parser_setpos170_silent),
  cmocka_unit_test(test_rs485_parser_getpos170),
  cmocka_unit_test(test_rs485_parser_set_id),
  cmocka_unit_test(test_rs485_parser_id_report),
  cmocka_unit_test(test_rs485_parser_misc),
  cmocka_unit_test(test_rs485_parser_test),
  cmocka_unit_test(test_rs485_parser_read_mp),
  cmocka_unit_test(test_rs485_parser_write_mp),
  cmocka_unit_test(test_rs485_parser_read_eeprom),
  cmocka_unit_test(test_rs485_parser_write_eeprom),
  cmocka_unit_test(test_rs485_parser_set_access_bit),
  cmocka_unit_test(test_rs485_parser_motor_power),
  cmocka_unit_test(test_rs485_parser_read_magnet),
  cmocka_unit_test(test_rs485_parser_read_serial_num),
  cmocka_unit_test(test_rs485_parser_read_descr),
  cmocka_unit_test(test_rs485_parser_read_fw_num),
  cmocka_unit_test(test_rs485_parser_read_powerups),
  cmocka_unit_test(test_rs485_parser_read_runtimers),
  cmocka_unit_test(test_rs485_parser_write_serial_num),
  cmocka_unit_test(test_rs485_parser_read_current),
  cmocka_unit_test(test_rs485_parser_read_voltage),
  cmocka_unit_test(test_rs485_parser_read_temperature),
  cmocka_unit_test(test_rs485_parser_read_humidity),
  cmocka_unit_test(test_rs485_parser_set_default),
  cmocka_unit_test(test_rs485_parser_read_revision),
  /**< MVS module */
  cmocka_unit_test(test_mvs),
  /**< Volz CAN protocol */
  cmocka_unit_test(test_volz_echo),
  cmocka_unit_test(test_volz_set_pos),
  cmocka_unit_test(test_volz_get_pos),
  cmocka_unit_test(test_volz_get_time),
  cmocka_unit_test(test_volz_get_param),
  cmocka_unit_test(test_volz_set_param),
  cmocka_unit_test(test_volz_get_hwfw),
  cmocka_unit_test(test_volz_getset_string),
  cmocka_unit_test(test_volz_execute_op),
  cmocka_unit_test(test_volz_duplex),
  cmocka_unit_test(test_volz_get_counter),
  cmocka_unit_test(test_volz_get_revision),
  cmocka_unit_test(test_volz_read_ee),
  cmocka_unit_test(test_volz_write_ee),
};

int main(void)
{
  printf("     ----- Start tests for DA-58 -----\n\n");
  return cmocka_run_group_tests(tests, NULL, NULL);
}
