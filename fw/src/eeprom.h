#ifndef EEPROM_H
#define EEPROM_H

#include "defines.h"

#define EEPROM_ADDR_COPY_1      0x00400000U
#define EEPROM_ADDR_COPY_2      0x00400100U

#define EEPROM_ADDR_STRINGS     0x00400200U

#define EEPROM_ADDR_COUNTERS    0x00400300U

#define EEPROM_ESN_ADDRESS      0x00U
#define EEPROM_PROD_ADDRESS     0x30U
#define EEPROM_FWREV_ADDRESS    0x60U
#define EEPROM_HWREV_ADDRESS    0x90U

#define EEPROM_POWERUPS_ADDR    0x20U
#define EEPROM_STALLS_ADDR      0x24U

#define EEPROM_PAGE_SIZE        0x100U

#define EEPROM_WRITE_TIME_MS    3

#define EEPROM_LEN_STRING       32

enum
{
  EE_COPY_1,
  EE_COPY_2
};

typedef struct
{
  uint8_t id;       /**< Id of the parameter */
  uint8_t address;  /**< Address in FRAM */
  uint8_t size;     /**< Size in bytes */
  uint8_t type;     /**< Type of parameter */
  void*   pVal;     /**< Pointer to variable in RAM */
  union {
    uint8_t  byte;
    int8_t   sbyte;
    uint16_t word;
    int16_t  sword;
    uint32_t dword;
    float    fl;
  } minVal;
  union {
    uint8_t  byte;
    int8_t   sbyte;
    uint16_t word;
    int16_t  sword;
    uint32_t dword;
    float    fl;
  } maxVal;
  union {
    uint8_t  byte;
    int8_t   sbyte;
    uint16_t word;
    int16_t  sword;
    uint32_t dword;
    float    fl;
  } defVal;
  char name[PARAM_NAME_LEN];
} eeVal_t;

extern uint8_t EEPROM_Page[EEPROM_PAGE_SIZE];
extern uint8_t EEPROM_Strings[EEPROM_PAGE_SIZE];
extern uint8_t EEPROM_Counters[EEPROM_PAGE_SIZE];

uint32_t EEPROM_GetCounter(uint8_t num);
void EEPROM_IncCounter(uint8_t num);
void EEPROM_ResetCounter(uint8_t num);
uint32_t EEPROM_GetStallEvents(void);
void EEPROM_IncStallEvents(void);
void EEPROM_ResetStallEvents(void);
uint32_t EEPROM_GetPowerUps(void);
void EEPROM_IncPowerUps(void);
void EEPROM_ResetPowerUps(void);
uint16_t EEPROM_RecalculateCRC(void);
uint16_t EEPROM_CalculateCRC(void);
uint16_t EEPROM_GetCRC(void);
bool EEPROM_CheckCRC(void);
void EEPROM_WriteByte(uint8_t addr, uint8_t data);
void EEPROM_SaveVariable(void *var);
uint8_t EEPROM_GetParamCount(void);
bool EEPROM_GetParam(uint8_t num, eeVal_t *param);
void EEPROM_SetParam(uint8_t num, void *data);
uint8_t EEPROM_ReadByte(uint16_t addr);
void EEPROM_LoadVariables(void);
void EEPROM_SaveBoth(void);
void EEPROM_SaveStrings(void);
void EEPROM_SaveCounters(void);
void EEPROM_Configuration(void);

#endif

