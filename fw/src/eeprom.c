#include "drivers.h"
#include "can_volz.h"
#include "crc16.h"
#include "eeprom.h"
#include "global.h"
#include "rs485.h"
#include "version.h"

uint8_t EEPROM_Page[EEPROM_PAGE_SIZE] __attribute__ ((aligned (4)));
uint8_t EEPROM_Strings[EEPROM_PAGE_SIZE] __attribute__ ((aligned (4)));
uint8_t EEPROM_Counters[EEPROM_PAGE_SIZE] __attribute__ ((aligned (4)));

#ifndef DEF_UNITTEST
  /**< Mutex to ensure atomic operations with writing EEPROM */
  static SemaphoreHandle_t xMutexEEPROM;
#endif

/**< This array is used to match global variables from RAM with their EEPROM representation */
static const eeVal_t EEPROM_Values[] =
{
  {0,  0x10, 1, PARAM_TYPE_UINT8,  (void*)&EE_CanBitrate,    {.byte = CAN_BITRATE_125K}, {.byte = CAN_BITRATE_1M}, {.byte = CAN_BITRATE_500K}, "Bitrat"},
  {1,  0x11, 1, PARAM_TYPE_UINT8,  (void*)&EE_DevId,         {.byte = 0}, {.byte = 15}, {.byte = 0}, "ID"},
  {2,  0x12, 2, PARAM_TYPE_UINT16, (void*)&EE_CanId,         {.word = 0}, {.word = 0x7E0}, {.word = CANBUS_DFLT_ID}, "CanID"},
  {3,  0x20, 1, PARAM_TYPE_UINT8,  (void*)&EE_Kd,            {.byte = 0}, {.byte = 255}, {.byte = 50}, "Kd"},
  {4,  0x21, 1, PARAM_TYPE_UINT8,  (void*)&EE_Kp,            {.byte = 0}, {.byte = 255}, {.byte = 40}, "Kp"},
  {5,  0x31, 1, PARAM_TYPE_UINT8,  (void*)&EE_Ki,            {.byte = 0}, {.byte = 255}, {.byte = 0},  "Ki"},
  {6,  0x22, 1, PARAM_TYPE_UINT8,  (void*)&EE_PWM_Min,       {.byte = 0}, {.byte = 255}, {.byte = 30}, "MinPWM"},
  {7,  0x24, 1, PARAM_TYPE_UINT8,  (void*)&EE_PWM_Max,       {.byte = 0}, {.byte = 255}, {.byte = 255}, "MaxPWM"},
  {8,  0x3E, 1, PARAM_TYPE_UINT8,  (void*)&EE_ResponseDelay, {.byte = 0}, {.byte = 255}, {.byte = 0}, "RspDel"},
  {9,  0x28, 1, PARAM_TYPE_UINT8,  (void*)&EE_Sensor_DB,     {.byte = 0}, {.byte = 255}, {.byte = 2}, "SensDB"},
  {10, 0x29, 1, PARAM_TYPE_UINT8,  (void*)&EE_Options,       {.byte = 0}, {.byte = 255}, {.byte = 0}, "Option"},
  {11, 0x25, 1, PARAM_TYPE_UINT8,  (void*)&EE_PWM_Safe,      {.byte = 0}, {.byte = 255}, {.byte = 128}, "SafPWM"},
  {12, 0x33, 2, PARAM_TYPE_INT16,  (void*)&EE_ZeroPos,       {.sword = -2048}, {.sword = 2048}, {.sword = 0}, "ZeroP"},
  {13, 0x36, 2, PARAM_TYPE_INT16,  (void*)&EE_LossPos,       {.sword = SERVO_MIN_TP}, {.sword = SERVO_MAX_TP}, {.word = 0}, "FlsPos"},
  {14, 0x35, 1, PARAM_TYPE_UINT8,  (void*)&EE_LossTime,      {.byte = 0}, {.byte = 255}, {.byte = 0}, "FlsTim"},
  {15, 0x26, 1, PARAM_TYPE_UINT8,  (void*)&EE_SaverTime,     {.byte = 0}, {.byte = 255}, {.byte = 2}, "SafTim"},
  {16, 0x27, 1, PARAM_TYPE_UINT8,  (void*)&EE_SaverFrame,    {.byte = 0}, {.byte = 255}, {.byte = 60}, "SafFrm"},
  {17, 0x3F, 1, PARAM_TYPE_UINT8,  (void*)&EE_PowerDamper,   {.byte = 0}, {.byte = 255}, {.byte = 0}, "PwrDmp"},
  {18, 0x2A, 1, PARAM_TYPE_UINT8,  (void*)&EE_PWM_Test,      {.byte = 0}, {.byte = 255}, {.byte = 240}, "TstPWM"},
  {19, 0x2C, 1, PARAM_TYPE_UINT8,  (void*)&EE_LossBehavior,  {.byte = 0}, {.byte = 255}, {.byte = 0}, "LosBhv"},
  {20, 0x30, 1, PARAM_TYPE_UINT8,  (void*)&EE_StationId,     {.byte = 0}, {.byte = 31}, {.byte = 1}, "StatID"},
  {21, 0x38, 1, PARAM_TYPE_UINT8,  (void*)&EE_MaxMotorTemp,  {.byte = 0}, {.byte = 255}, {.byte = 120}, "MaxTmp"},
  {22, 0x39, 1, PARAM_TYPE_UINT8,  (void*)&EE_MotorTempHyst, {.byte = 0}, {.byte = 10},  {.byte = 5}, "TmpHys"},
  {23, 0x5F, 1, PARAM_TYPE_INT8,   (void*)&EE_MagSensorOffs, {.sbyte = -100}, {.sbyte = 100}, {.sbyte = 0}, "MagOfs"},
  {24, 0x23, 1, PARAM_TYPE_UINT8,  (void*)&EE_FreshCntLevel, {.byte = 0}, {.byte = 255}, {.byte = 16}, "Freshn"},
  {25, 0x40, 1, PARAM_TYPE_UINT8,  (void*)&EE_NotchWidth,    {.byte = 0}, {.byte = 255}, {.byte = 10}, "NWidth"},
  {26, 0x41, 1, PARAM_TYPE_UINT8,  (void*)&EE_NotchDepth,    {.byte = 0}, {.byte = 255}, {.byte = 10}, "NDepth"},
  {27, 0x45, 1, PARAM_TYPE_UINT8,  (void*)&EE_Options2,      {.byte = 0}, {.byte = 255}, {.byte = 0}, "Opt2"},
  {28, 0x46, 2, PARAM_TYPE_UINT16, (void*)&EE_NotchFreq1,    {.word = 0}, {.word = 8000}, {.word = 0}, "NFreq1"},
  {29, 0x48, 2, PARAM_TYPE_UINT16, (void*)&EE_NotchFreq2,    {.word = 0}, {.word = 8000}, {.word = 0}, "NFreq2"},
  {30, 0x3C, 1, PARAM_TYPE_UINT8,  (void*)&EE_HeaterTemp,    {.byte = 0}, {.byte = 100}, {.byte = 0}, "HeatT"},
  {31, 0x3A, 1, PARAM_TYPE_UINT8,  (void*)&EE_PosErrTime,    {.byte = 0}, {.byte = 100}, {.byte = 0}, "PosErT"},
  {32, 0x3B, 1, PARAM_TYPE_UINT8,  (void*)&EE_PosErrAngle,   {.byte = 0}, {.byte = 100}, {.byte = 0}, "PosErA"},
  {33, 0x3D, 1, PARAM_TYPE_UINT8,  (void*)&EE_MaxCurrent,    {.byte = 0}, {.byte = 0}, {.byte = 0}, "MaxCur"},
  {34, 0xF0, 1, PARAM_TYPE_UINT8,  (void*)&EE_ServoId,       {.byte = 0}, {.byte = 255}, {.byte = 0x58}, "SrvID"},
  {35, 0x14, 1, PARAM_TYPE_UINT8,  (void*)&EE_Baudrate,      {.byte = RS485_BAUDRATE_9600}, {.byte = RS485_BAUDRATE_250000}, {.byte = RS485_BAUDRATE_115200}, "Baudrt"},
  {36, 0x4A, 1, PARAM_TYPE_UINT8,  (void*)&EE_MaxCurrentCW,  {.byte = 0}, {.byte = 0}, {.byte = 0}, "MaxCW"}
};

/** \brief Get value of device counter
 *
 * \param [in] num Number of the counter to read
 * \return Counter values in seconds as uint32_t
 *
 */
uint32_t EEPROM_GetCounter(uint8_t num)
{
  if (num >= COUNTER_LOAD_LAST)
  {
    return 0;
  }

  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[num * sizeof(uint32_t)], sizeof(uint32_t));
  return uval32;
  //return *(uint32_t*)&EEPROM_Counters[num * sizeof(uint32_t)];
}

/** \brief Increment device counters
 *
 * \param [in] num Number of the counter to increment
 * \return void
 *
 */
void EEPROM_IncCounter(uint8_t num)
{
  if (num >= COUNTER_LOAD_LAST)
  {
    return;
  }

  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[num * sizeof(uint32_t)], sizeof(uint32_t));
  uval32++;
  (void) memcpy(&EEPROM_Counters[num * sizeof(uint32_t)], (void*)&uval32, sizeof(uint32_t));
  //(*(uint32_t*)&EEPROM_Counters[num * sizeof(uint32_t)])++;
}

/** \brief Reset device counter
 *
 * \param [in] num Number of the counter to reset
 * \return void
 *
 */
void EEPROM_ResetCounter(uint8_t num)
{
  if (num >= COUNTER_LOAD_LAST)
  {
    return;
  }

  uint32_t uval32 = 0;
  (void) memcpy(&EEPROM_Counters[num * sizeof(uint32_t)], (void*)&uval32, sizeof(uint32_t));
  //*(uint32_t*)&EEPROM_Counters[num * sizeof(uint32_t)] = 0;
}

/** \brief Get value of stalls counter
 *
 * \return Stalls counter as uint32_t
 *
 */
uint32_t EEPROM_GetStallEvents(void)
{
  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[EEPROM_STALLS_ADDR], sizeof(uint32_t));
  return uval32;
  //return *(uint32_t*)&EEPROM_Counters[EEPROM_STALLS_ADDR];
}

/** \brief Increment stall counter
 *
 * \return void
 *
 */
void EEPROM_IncStallEvents(void)
{
  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[EEPROM_STALLS_ADDR], sizeof(uint32_t));
  uval32++;
  (void) memcpy(&EEPROM_Counters[EEPROM_STALLS_ADDR], (void*)&uval32, sizeof(uint32_t));
  //(*(uint32_t*)&EEPROM_Counters[EEPROM_STALLS_ADDR])++;
}

/** \brief Reset stall counter
 *
 * \return void
 *
 */
void EEPROM_ResetStallEvents(void)
{
  uint32_t uval32 = 0;
  (void) memcpy(&EEPROM_Counters[EEPROM_STALLS_ADDR], (void*)&uval32, sizeof(uint32_t));
  //*(uint32_t*)&EEPROM_Counters[EEPROM_STALLS_ADDR] = 0;
}

/** \brief Get value of power-ups counter
 *
 * \return Power-Ups counter as uint32_t
 *
 */
uint32_t EEPROM_GetPowerUps(void)
{
  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[EEPROM_POWERUPS_ADDR], sizeof(uint32_t));
  return uval32;
  //return *(uint32_t*)&EEPROM_Counters[EEPROM_POWERUPS_ADDR];
}

/** \brief Increment power-ups counter
 *
 * \return void
 *
 */
void EEPROM_IncPowerUps(void)
{
  uint32_t uval32;
  (void) memcpy((void*)&uval32, &EEPROM_Counters[EEPROM_POWERUPS_ADDR], sizeof(uint32_t));
  uval32++;
  (void) memcpy(&EEPROM_Counters[EEPROM_POWERUPS_ADDR], (void*)&uval32, sizeof(uint32_t));
  //(*(uint32_t*)&EEPROM_Counters[EEPROM_POWERUPS_ADDR])++;
}

/** \brief Reset stall counter
 *
 * \return void
 *
 */
void EEPROM_ResetPowerUps(void)
{
  uint32_t uval32 = 0;
  (void) memcpy(&EEPROM_Counters[EEPROM_POWERUPS_ADDR], (void*)&uval32, sizeof(uint32_t));
  //*(uint32_t*)&EEPROM_Counters[EEPROM_POWERUPS_ADDR] = 0;
}

/** \brief Recalculate EEPROM CRC
 *
 * \return New CRC16 value as uint16_t
 *
 */
uint16_t EEPROM_RecalculateCRC(void)
{
  uint16_t crc = CRC16_INIT_VAL;

  crc = CRC16_Calc(crc, &EEPROM_Page[sizeof(uint32_t)], EEPROM_PAGE_SIZE - sizeof(uint32_t));
  /**< Write new CRC value to the start of EEPROM page */
  (void) memcpy(EEPROM_Page, (uint8_t*)&crc, sizeof(uint16_t));

  return crc;
}

/** \brief Calculate EEPROM CRC
 *
 * \return CRC16 value as uint16_t
 *
 */
uint16_t EEPROM_CalculateCRC(void)
{
  uint16_t crc = CRC16_INIT_VAL;

  crc = CRC16_Calc(crc, &EEPROM_Page[sizeof(uint32_t)], EEPROM_PAGE_SIZE - sizeof(uint32_t));

  return crc;
}

/** \brief Get EEPROM CRC value
 *
 * \return CRC16 value as uint16_t
 *
 */
uint16_t EEPROM_GetCRC(void)
{
  uint16_t crc;
  (void) memcpy((void*)&crc, EEPROM_Page, sizeof(uint16_t));
  return crc;
  //return *((uint16_t*)EEPROM_Page);
}

/** \brief Check if CRC is correct
 *
 * \return True if CRC is correct, false otherwise
 *
 */
bool EEPROM_CheckCRC(void)
{
  bool result = true;

  if (xSemaphoreTake(xMutexEEPROM, portMAX_DELAY) == pdTRUE)
  {
    result = (EEPROM_CalculateCRC() == EEPROM_GetCRC());
    xSemaphoreGive(xMutexEEPROM);
  }

  return result;
}

/** \brief Write one byte to EEPROM (to avoid EEPROM CRC error)
 *
 * \param [in] addr Addres to write in
 * \param [in] data Data byte to write
 * \return void
 *
 */
void EEPROM_WriteByte(uint8_t addr, uint8_t data)
{
  /**< Don't allow to write to CRC area */
  if (addr < sizeof(uint16_t))
  {
    return;
  }

  if (xSemaphoreTake(xMutexEEPROM, portMAX_DELAY) == pdTRUE)
  {
    EEPROM_Page[addr] = data;
    /**< Change parameter associated with EEPROM address */
    for (uint8_t i = 0; i < sizeof(EEPROM_Values) / sizeof(eeVal_t); i++)
    {
      if ((addr >= EEPROM_Values[i].address) && (addr < (EEPROM_Values[i].address + EEPROM_Values[i].size)))
      {
        (void) memcpy(EEPROM_Values[i].pVal, &EEPROM_Page[EEPROM_Values[i].address], EEPROM_Values[i].size);
        break;
      }
    }
    (void) EEPROM_RecalculateCRC();
    xSemaphoreGive(xMutexEEPROM);
  }
}

/** \brief Read one byte from EEPROM array
 *
 * \param addr Address of the readout
 * \return EEPROM byte as uint8_t
 *
 */
uint8_t EEPROM_ReadByte(uint16_t addr)
{
  return EEPROM_Page[(uint8_t)addr];
}

/** \brief Load all variables from EEPROM
 *
 * \return void
 *
 */
void EEPROM_LoadVariables(void)
{
  uint8_t i;

  for (i = 0; i < sizeof(EEPROM_Values) / sizeof(eeVal_t); i++)
  {
    (void) memcpy(EEPROM_Values[i].pVal, &EEPROM_Page[EEPROM_Values[i].address], EEPROM_Values[i].size);
  }
}

/** \brief Rewrite variable with address
 *
 * \param [in] Pointer to variable
 * \return void
 *
 */
void EEPROM_SaveVariable(void *var)
{
  uint8_t i;

  for (i = 0; i < sizeof(EEPROM_Values) / sizeof(eeVal_t); i++)
  {
    if (var == EEPROM_Values[i].pVal)
    {
      (void) memcpy(&EEPROM_Page[EEPROM_Values[i].address], var, EEPROM_Values[i].size);
      (void) EEPROM_RecalculateCRC();
      break;
    }
  }
}

/** \brief Get number of EEPROM parameters
 *
 * \return Number of EEPROM parameters as uint8_t
 *
 */
uint8_t EEPROM_GetParamCount(void)
{
  return (sizeof(EEPROM_Values) / sizeof(eeVal_t));
}

/** \brief Find parameter by Id
 *
 * \param [in] id Id of the parameter
 * \return Index of parameter or -1 if not found
 *
 */
static int8_t EEPROM_FindParamById(uint8_t id)
{
  uint8_t i;

  for (i = 0; i < EEPROM_GetParamCount(); i++)
  {
    if (id == EEPROM_Values[i].id)
    {
      return i;
    }
  }

  return -1;
}

/** \brief Find parameter by its value
 *
 * \param [in] var Pointer to pVal of eeVal_t
 * \return Index of parameter or -1 if not found
 *
 */
//static int8_t EEPROM_FindParam(void *var)
//{
//  uint8_t i;
//
//  for (i = 0; i < sizeof(EEPROM_Values) / sizeof(eeVal_t); i++)
//  {
//    if (var == EEPROM_Values[i].pVal)
//    {
//      return i;
//    }
//  }
//
//  return -1;
//}

/** \brief Get EEPROM parameter
 *
 * \param [in] num Number of EEPROM parameter
 * \param [in] data Pointer to parameter structure
 * \return bool True if parameter number is valid
 *
 */
bool EEPROM_GetParam(uint8_t num, eeVal_t *param)
{
  //if (num >= EEPROM_GetParamCount())
  //  return false;
  int8_t id = EEPROM_FindParamById(num);
  if (id < 0)
  {
    return false;
  }

  *param = EEPROM_Values[id];

  return true;
}

/** \brief Set EEPROM parameter value
 *
 * \param [in] num Number of EEPROM parameter
 * \param [in] data Pointer to new value
 * \return void
 *
 */
void EEPROM_SetParam(uint8_t num, void *data)
{
  union {
    uint8_t  byte;
    int8_t   sbyte;
    uint16_t word;
    int16_t  sword;
    uint32_t dword;
    float    fl;
  } value;

  if (num >= EEPROM_GetParamCount())
  {
    return;
  }

  (void) memcpy(&value, data, EEPROM_Values[num].size);
  switch (EEPROM_Values[num].type)
  {
    case PARAM_TYPE_UINT8:
      if (value.byte > EEPROM_Values[num].maxVal.byte)
      {
        value.byte = EEPROM_Values[num].maxVal.byte;
      }
      if (value.byte < EEPROM_Values[num].minVal.byte)
      {
        value.byte = EEPROM_Values[num].minVal.byte;
      }
      break;
    case PARAM_TYPE_INT8:
      if (value.sbyte > EEPROM_Values[num].maxVal.sbyte)
      {
        value.sbyte = EEPROM_Values[num].maxVal.sbyte;
      }
      if (value.sbyte < EEPROM_Values[num].minVal.sbyte)
      {
        value.sbyte = EEPROM_Values[num].minVal.sbyte;
      }
      break;
    case PARAM_TYPE_UINT16:
      if (value.word > EEPROM_Values[num].maxVal.word)
      {
        value.word = EEPROM_Values[num].maxVal.word;
      }
      if (value.word < EEPROM_Values[num].minVal.word)
      {
        value.word = EEPROM_Values[num].minVal.word;
      }
      break;
    case PARAM_TYPE_INT16:
      if (value.sword > EEPROM_Values[num].maxVal.sword)
      {
        value.sword = EEPROM_Values[num].maxVal.sword;
      }
      if (value.sword < EEPROM_Values[num].minVal.sword)
      {
        value.sword = EEPROM_Values[num].minVal.sword;
      }
      break;
    case PARAM_TYPE_UINT32:
      if (value.dword > EEPROM_Values[num].maxVal.dword)
      {
        value.dword = EEPROM_Values[num].maxVal.dword;
      }
      if (value.dword < EEPROM_Values[num].minVal.dword)
      {
        value.dword = EEPROM_Values[num].minVal.dword;
      }
      break;
    default:
      /**< Unsupported type, ignore */
      return;
  }

  (void) memcpy(EEPROM_Values[num].pVal, &value, EEPROM_Values[num].size);
  void *p = EEPROM_Values[num].pVal;
  EEPROM_SaveVariable(p);
}

/** \brief Load EEPROM copy to global variables
 *
 * \param [in] ee_copy Number of the copy to load
 * \return void
 *
 */
static void EEPROM_LoadCopy(uint8_t ee_copy)
{
  uint32_t addr;

  switch (ee_copy)
  {
    case EE_COPY_1:
      addr = EEPROM_ADDR_COPY_1;
      break;
    case EE_COPY_2:
      addr = EEPROM_ADDR_COPY_2;
      break;
    default:
      return;
  }

  #ifndef DEF_UNITTEST
    (void) memcpy(EEPROM_Page, (uint8_t*)addr, EEPROM_PAGE_SIZE);
  #else
    (void) addr;
  #endif
}

/** \brief Load strings from EEPROM to global variables
 *
 * \return void
 *
 */
static void EEPROM_LoadStrings(void)
{
  #ifndef DEF_UNITTEST
    (void) memcpy(EEPROM_Strings, (uint8_t*)EEPROM_ADDR_STRINGS, EEPROM_PAGE_SIZE);
  #endif
}

/** \brief Load counters from EEPROM to global variables
 *
 * \return void
 *
 */
static void EEPROM_LoadCounters(void)
{
  #ifndef DEF_UNITTEST
    (void) memcpy(EEPROM_Counters, (uint8_t*)EEPROM_ADDR_COUNTERS, EEPROM_PAGE_SIZE);
  #endif
}

/** \brief Save global variables to EEPROM copy
 *
 * \param [in] ee_copy Number of the copy to save to
 * \return void
 *
 */
static void EEPROM_SaveCopy(uint8_t ee_copy)
{
  uint32_t addr;
  uint16_t i;
  uint16_t crc = 0U;

  switch (ee_copy)
  {
    case EE_COPY_1:
      addr = EEPROM_ADDR_COPY_1;
      break;
    case EE_COPY_2:
      addr = EEPROM_ADDR_COPY_2;
      break;
    default:
      return;
  }
  //crc = EEPROM_RecalculateCRC();
  //if (crc != *(uint16_t*)((uintptr_t)addr))
  #ifndef DEF_UNITTEST
  (void) memcpy((void*)&crc, (uint32_t*)((uintptr_t)addr), sizeof(uint16_t));
  #endif
  if (crc != EEPROM_RecalculateCRC())
  {
    /**< Save only if CRC is different */
    /**< Erase EEPROM page */
    FLASH_EraseRowEE((uint32_t*)((uintptr_t)addr));
    /**< Write page content to EEPROM */
    for (i = 0U; i < EEPROM_PAGE_SIZE; i += FLASH_PAGE_SIZE)
    {
      uint32_t *p = (uint32_t*)(addr + i);
      void *p2 = &EEPROM_Page[i];
      FLASH_WriteWordsEE(p, (uint32_t*)p2, FLASH_PAGE_SIZE / sizeof(uint32_t));
      //FLASH_WriteWordsEE((uint32_t*)((uintptr_t)(addr + i)), (uint32_t*)&EEPROM_Page[i], FLASH_PAGE_SIZE / sizeof(uint32_t));
    }
  }
}

/** \brief Save both EEPROM copies
 *
 * \return void
 *
 */
void EEPROM_SaveBoth(void)
{
  EEPROM_SaveCopy(EE_COPY_1);
  EEPROM_SaveCopy(EE_COPY_2);
}

/** \brief Save strings to EEPROM page
 *
 * \return void
 *
 */
void EEPROM_SaveStrings(void)
{
  uint16_t i;

  /**< Erase EEPROM page */
  FLASH_EraseRowEE((uint32_t*)EEPROM_ADDR_STRINGS);
  /**< Write page content to EEPROM */
  for (i = 0U; i < EEPROM_PAGE_SIZE; i += FLASH_PAGE_SIZE)
  {
    uint32_t *p = (uint32_t*)(EEPROM_ADDR_STRINGS + i);
    void *p2 = &EEPROM_Strings[i];
    FLASH_WriteWordsEE(p, (uint32_t*)p2, FLASH_PAGE_SIZE / sizeof(uint32_t));
    //FLASH_WriteWordsEE((uint32_t*)((uintptr_t)(EEPROM_ADDR_STRINGS + i)), (uint32_t*)&EEPROM_Strings[i], FLASH_PAGE_SIZE / sizeof(uint32_t));
  }
}

/** \brief Save counters to EEPROM page
 *
 * \return void
 *
 */
void EEPROM_SaveCounters(void)
{
  uint16_t i;

  /**< Erase EEPROM page */
  FLASH_EraseRowEE((uint32_t*)EEPROM_ADDR_COUNTERS);
  /**< Write page content to EEPROM */
  for (i = 0U; i < EEPROM_PAGE_SIZE; i += FLASH_PAGE_SIZE)
  {
    uint32_t *p = (uint32_t*)(EEPROM_ADDR_COUNTERS + i);
    void *p2 = &EEPROM_Counters[i];
    FLASH_WriteWordsEE(p, (uint32_t*)p2, FLASH_PAGE_SIZE / sizeof(uint32_t));
    //FLASH_WriteWordsEE((uint32_t*)((uintptr_t)(EEPROM_ADDR_COUNTERS + i)), (uint32_t*)&EEPROM_Counters[i], FLASH_PAGE_SIZE / sizeof(uint32_t));
  }
}

/** \brief Initialize EEPROM memory, perform self-test
 *
 * \return void
 *
 */
void EEPROM_Configuration(void)
{
  uint16_t crc;
  uint16_t crc_read;
  bool firstCopy;
  bool secondCopy;
  uint8_t i;

  //GPIO_SetPin(GPIO_PORTB, 3);

  #ifndef DEF_UNITTEST
    xMutexEEPROM = xSemaphoreCreateMutex();
  #endif

  firstCopy = true;
  secondCopy = true;
  EEPROM_LoadCopy(EE_COPY_1);
  (void) memcpy((void*)&crc_read, EEPROM_Page, sizeof(uint16_t));
  //crc_read = *(uint16_t*)EEPROM_Page;
  crc = CRC16_INIT_VAL;
  crc = CRC16_Calc(crc, &EEPROM_Page[sizeof(uint32_t)], EEPROM_PAGE_SIZE - sizeof(uint32_t));
  if (crc != crc_read)
  {
    /**< first EEPROM page is corrupted */
    firstCopy = false;
  }
  EEPROM_LoadCopy(EE_COPY_2);
  (void) memcpy((void*)&crc_read, EEPROM_Page, sizeof(uint16_t));
  //crc_read = *(uint16_t*)EEPROM_Page;
  crc = CRC16_INIT_VAL;
  crc = CRC16_Calc(crc, &EEPROM_Page[sizeof(uint32_t)], EEPROM_PAGE_SIZE - sizeof(uint32_t));
  if (crc != crc_read)
  {
    /**< second EEPROM page is corrupted */
    secondCopy = false;
  }

  if ((firstCopy == false) && (secondCopy == false))
  {
    /**< Both copies are corrupted Reload default values */
    (void) memset(EEPROM_Page, 0xff, EEPROM_PAGE_SIZE);
    for (i = 0; i < sizeof(EEPROM_Values) / sizeof(eeVal_t); i++)
    {
      (void) memcpy(EEPROM_Values[i].pVal, &EEPROM_Values[i].defVal, EEPROM_Values[i].size);
      (void) memcpy((void*)&EEPROM_Page[EEPROM_Values[i].address], &EEPROM_Values[i].defVal, EEPROM_Values[i].size);
    }
    /**< Save both copies */
    EEPROM_SaveBoth();

  } else
  if (secondCopy == false)
  {
    /**< Second copy is corrupted, reload with first copy */
    EEPROM_LoadCopy(EE_COPY_1);
    EEPROM_SaveCopy(EE_COPY_2);
  } else
  if (firstCopy == false)
  {
    /**< First copy is corrupted, reload with second copy */
    EEPROM_LoadCopy(EE_COPY_2);
    EEPROM_SaveCopy(EE_COPY_1);
  } else
  {
    /**< Both copies are Ok, just load first copy */
    EEPROM_LoadCopy(EE_COPY_1);
  }
  /**< Load variables from current EEPROM content */
  EEPROM_LoadVariables();
  /**< Load strings */
  EEPROM_LoadStrings();
  /**< Load counters */
  EEPROM_LoadCounters();
  /**< Increment Power-Ups counter with every start */
  EEPROM_IncPowerUps();

  /**< Check for wrong EEPROM values (not implemented yet) */
  /**< ... */

  /**< Check FW version */
  if (strncmp((char*)&EEPROM_Strings[EEPROM_FWREV_ADDRESS], VERSION_GetStringFW(), EEPROM_LEN_STRING) != 0)
  {
    /**< Probably new FW version, rewrite EEPROM */
    (void) strcpy((char*)&EEPROM_Strings[EEPROM_FWREV_ADDRESS], VERSION_GetStringFW());
    EEPROM_SaveStrings();
  }


  //GPIO_ClearPin(GPIO_PORTB, 3);
}
