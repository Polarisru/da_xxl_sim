#include "conversion.h"
#include "motor.h"

/**
 * \brief Calculate temperature from MCP9700 sensor
 * t = (ADC Value - COEFFB) / COEFFK, voltage value for 0°C = 0.5V
 *
 * \param [in] value Raw value from ADC
 * \return Temperature value shifted to -50°C as uint8_t
 */
uint8_t CONVERSION_CalcTemperature(uint16_t value)
{
  #define MCP9700_COEFF_K       0.01F     // 10mV/°C
  #define MCP9700_COEFF_B       0.500F    // 500mV for 0°C
  #define MCP9700_TEMP_OFFSET   50
  #define MCP9700_TEMP_MIN      (-MCP9700_TEMP_OFFSET)
  #define MCP9700_TEMP_MAX      205

  int16_t temp;
  float fVal;

  fVal = (float)(((float)value * ADC_REF_VOLTAGE / ADC_MAX_VALUE) - MCP9700_COEFF_B) / MCP9700_COEFF_K;
  temp = (int16_t)fVal;

  if (temp < MCP9700_TEMP_MIN)
  {
    temp = MCP9700_TEMP_MIN;
  }
  if (temp > MCP9700_TEMP_MAX)
  {
    temp = MCP9700_TEMP_MAX;
  }
  temp += MCP9700_TEMP_OFFSET;

  return (uint8_t)temp;
}

/**
 * \brief Calculate voltage from 5.1 to 75 resistor divider
 * V = ADC_Value * ADC_REF_VOLTAGE / ADC_MAX_VALUE / divider / 0.2V
 *
 * \param [in] value Raw value from ADC
 * \return Voltage value *0.2V as uint8_t
 */
uint8_t CONVERSION_CalcVoltage(uint16_t value)
{
  #define ADC_VOLT_DIVIDER  (float)(5.1f / (5.1f + 75.0f))
  #define ADC_VOLT_COEFF    (ADC_REF_VOLTAGE / ADC_VOLT_DIVIDER / VOLTAGE_STEP)
  uint32_t voltage;
  float volt_coeff = ADC_VOLT_COEFF;

  voltage = (uint32_t)value * (uint32_t)volt_coeff;
  voltage /= ADC_MAX_VALUE;

  return (uint8_t)voltage;
}

/**
 * \brief Calculate current from INA195
 * We have amplification factor 100, voltage divider 1:2 and 0.01R shunt, so we should get 0.5V for 1A
 * I = ADC_Value / ADC_MAX_VALUE * ADC_REF_VOLTAGE  / 0.5V / 0.05A
 *
 * \param [in] value Raw value from ADC
 * \return Voltage value *0.05A as uint8_t
 */
uint8_t CONVERSION_CalcCurrent(uint16_t value)
{
  #if defined DEF_DA58_SD
    #define ADC_CURR_COEFF  (uint32_t)(ADC_REF_VOLTAGE * 2 / (0.010F * 100U) / CURRENT_STEP * 100U)
  #elif defined DEF_DA58
    #define ADC_CURR_COEFF  (uint32_t)(ADC_REF_VOLTAGE * 2 / (0.005F * 50U) / CURRENT_STEP * 100U)
  #else
    #define ADC_CURR_COEFF  (uint32_t)(ADC_REF_VOLTAGE * 2 / (0.010F * 50U) / CURRENT_STEP * 100U)
  #endif
  uint32_t current;

  current = (uint32_t)value * ADC_CURR_COEFF / 100U;
  current /= ADC_MAX_VALUE;

  /**< Overcurrent protection */
  if (current > UINT8_MAX)
  {
    current = UINT8_MAX;
  }

  return (uint8_t)current;
}

/** \brief Calculate position in 0.1°
 *
 * \param [in] value Value of magnet sensor
 * \return Position in 0.1° as uint16_t
 *
 */
uint16_t CONVERSION_CalcDegreesFromPos(int16_t value)
{
  int32_t pos;

  pos = (int32_t)value * 1800 / 0x800;
  /**< We only need 12 bits */
  return (uint16_t)((uint32_t)pos & 0xFFFFU);
}

/** \brief Calculate sensor value from position in 0.1°
 *
 * \param [in] degrees Degrees value
 * \return Sensor value as int16_t
 *
 */
int16_t CONVERSION_CalcPosFromDegrees(uint16_t degrees)
{
  int16_t value;
  int32_t temp;

  degrees &= 0xFFFU;
  if (degrees >= 0x800U)
  {
    degrees |= 0xF000U;
  }
  temp = (int32_t)((int16_t)degrees);
  value = (int16_t)(temp * 0x800 / 1800);
  return value;
}

/** \brief Calculate position from SetPos170 command (RS485)
 *
 * \param [in] byte1 First byte
 * \param [in] byte2 Second byte
 * \return Position as int16_t
 *
 */
int16_t CONVERSION_CalcPosFromSetPos170(uint8_t byte1, uint8_t byte2)
{
  int16_t result;
  uint16_t uval16;

  /**< Remove freshness part - higher nibble */
  byte1 &= 0x0FU;
  /**< Negative values */
  if (byte1 > 7U)
  {
    byte1 |= 0xF0U;
  }
  /**< Build new target position */
  uval16 = ((uint16_t)byte1 << 8U) + byte2;
  result = (int16_t)uval16;
  /**< Limits for DA26D position command */
  if (result > MAX_TARGET_POS)
  {
    result = MAX_TARGET_POS;
  }
  if (result < -MAX_TARGET_POS)
  {
    result = -MAX_TARGET_POS;
  }

  return result;
}

/** \brief Calculate position from SetPos100 command (RS485)
 *
 * \param [in] byte1 First byte
 * \param [in] byte2 Second byte
 * \return Position as int16_t
 *
 */
int16_t CONVERSION_CalcPosFromSetPos100(uint8_t byte1, uint8_t byte2)
{
  int16_t  result;
  uint16_t uval16;
  int16_t  sval16;

  uval16 = ((uint16_t)byte1 << 8U) + byte2;
  if (uval16 > MAX_TP_KF_TR)
  {
    /**< +100° for 0xDD com transformed according to Kearfott/Schiebel */
    sval16 = MAX_TP_KF;
  } else if (uval16 < MIN_TP_KF_TR)
  {
    /**< -100° for 0xDD com transformed according to Kearfott/Schiebel */
    sval16 = MIN_TP_KF;
  } else
  {
    uval16 = ((uint16_t)byte1 << 7U) + byte2;
    sval16 = (int16_t)uval16;
  }
  /**< Kearfott/Schiebel spec. here sval16: -100...+45 0x4A1...0xB60 or 1185..2912 */
  result = (sval16 - 0x800) * 0.5933F;

  return result;
}

/** \brief Calculate response for SetPos100 command (RS485)
 *
 * \param [in] position Position in internal format
 * \return Response as uint16_t
 *
 */
uint16_t CONVERSION_CalcResponseForSetPos100(int16_t position)
{
  uint8_t byte1;
  uint8_t byte2;
  /**< formula: 2048 + ((2048 - RealPos(magnet)) * angular_ratio) */
  int16_t tmp = (position * 1.687F) + 0x800;
  byte1 = (uint8_t)((uint16_t)tmp >> 8);
  byte2 = (uint8_t)tmp;
  /**< high byte: 0 0 0 bit11 bit10 bit9 bit8 0 */
  byte1 = ((uint8_t)(byte1 << 1U) & 0x1EU);
  /**< add bit7 as bit 0 to high byte: 0 0 0 bit11 bit10 bit9 bit8 bit7 */
  if ((byte2 & 0x80U) > 0U)
  {
    byte1++;
  }
  /**< low byte: 0 bit6 bit5 bit4 bit3 bit2 bit1 bit0 */
  byte2 = byte2 & 0x7FU;

  return (((uint16_t)byte1 << 8) + byte2);
}

/** \brief Calculate position from SetPosExt command (RS485)
 *
 * \param [in] byte1 First byte
 * \param [in] byte2 Second byte
 * \return Position as int16_t
 *
 */
int16_t CONVERSION_CalcPosFromSetPosExt(uint8_t byte1, uint8_t byte2)
{
  uint16_t uval16 = ((uint16_t)byte1 << 8) + byte2;
  if (uval16 < MIN_TP_EXT)
  {
    uval16 = MIN_TP_EXT;
  }
  if (uval16 > MAX_TP_EXT)
  {
    uval16 = MAX_TP_EXT;
  }
  return (int16_t)(((int16_t)uval16 - 0x800) * 0.5933F);
}

/** \brief Calculate response for SetPosExt command (RS485)
 *
 * \param [in] position Position in internal format
 * \return Response as uint16_t
 *
 */
uint16_t CONVERSION_CalcResponseForSetPosExt(int16_t position)
{
  /**< Formula: 2048 + ((2048 - RealPos(magnet)) * angular_ratio) */
  return (uint16_t)((position * 1.687F) + 0x800);
}

/** \brief Build old status (8bit) from current status (16bit)
 *
 * \param status [in] Current status (16bit)
 * \return Status in old format as uint8_t
 *
 */
uint8_t CONVERSION_BuildOldStatus(uint16_t status)
{
  uint8_t new_status = 0;

  /**< Bit 7: Health error */
  if ((status & STAT_ERROR_ACE) > 0U)
  {
    new_status |= OLDSTS_ERROR_TOTAL;
  }
  /**< Bit 6: Master bit */
  if ((status & STAT_MASTER_BIT) > 0U)
  {
    new_status |= OLDSTS_MASTER_BIT;
  }
  /**< Bit 5: Freshness error */
  if ((status & STAT_ERROR_FRESHNESS) > 0U)
  {
    new_status |= OLDSTS_ERROR_FRESHNESS;
  }
  /**< Bit 4: LoC error */
  if ((status & STAT_ERROR_LOSSCOMM) > 0U)
  {
    new_status |= OLDSTS_ERROR_LOSSCOMM;
  }
  /**< Bit 3: Power error */
  if ((status & STAT_ERROR_POWER) > 0U)
  {
    new_status |= OLDSTS_ERROR_POWER;
  }
  /**< Bit 2: Motor temperature error */
  if ((status & STAT_ERROR_MOTOR_TEMP) > 0U)
  {
    new_status |= OLDSTS_ERROR_MOTORTEMP;
  }
  /**< Bit 1: Internal network error */
  if ((status & STAT_ERROR_NOINTCOMM) > 0U)
  {
    new_status |= OLDSTS_ERROR_NOINTCOMM;
  }
  /**< Bit 0: Position error */
  if ((status & (STAT_ERROR_MAGNET | STAT_ERROR_2OO3)) > 0U)
  {
    new_status |= OLDSTS_ERROR_MAGNET;
  }

  return new_status;
}

/** \brief Build Volz status (11-bit CAN) from current status (16bit)
 *
 * \param status uint16_t
 * \return /uint8_t
 *
 */
uint8_t CONVERSION_BuildVolzStatus(uint16_t status)
{
  uint8_t new_status = 0;

  if ((status & STAT_ERROR_BLDC) > 0U)
  {
    new_status |= VOLZSTS_BIT_MOTOR_ERR;
  }
  if ((status & STAT_ERROR_MAGNET) > 0U)
  {
    new_status |= VOLZSTS_BIT_MAGNET_ERR;
  }
  /**< No overcurrent available */
  //if ()
  //  new_status |= VOLZSTS_BIT_OVERCURR;
  if ((status & STAT_ERROR_POWER) > 0U)
  {
    new_status |= VOLZSTS_BIT_UNDERVOLT;
  }
  if ((status & STAT_ERROR_LOSSCOMM) > 0U)
  {
    new_status |= VOLZSTS_BIT_COMM_TIMEOUT;
  }
  /**< No motor saver available */
  //if ()
  //  new_status |= VOLZSTS_BIT_MOTORSAVER;
  if ((status & STAT_ERROR_ACE) > 0U)
  {
    new_status |= VOLZSTS_BIT_SYSTEM_ERR;
  }
  /**< No motor dir available */
  if (MOTOR_GetDir() == MOTOR_DIR_CCW)
  {
    new_status |= VOLZSTS_BIT_MOTOR_DIR;
  }

  return new_status;
}
