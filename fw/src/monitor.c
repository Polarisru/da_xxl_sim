#include "drivers.h"
#include "analog.h"
#include "conversion.h"
#include "global.h"
#include "hall.h"
#include "monitor.h"
#include "power.h"

static uint8_t curr;
static uint8_t voltage1;
static uint8_t voltage2;
static uint8_t temperature1;
static uint8_t temperature2;
static uint8_t max_curr;
static uint8_t min_voltage;

/** \brief Get converted current value
 *
 * \return Current value as uint8_t
 *
 */
uint8_t MONITOR_GetCurrent(void)
{
  return curr;
}

/** \brief Set converted current value
 *
 * \param [in] value Current value to set
 * \return void
 *
 */
void MONITOR_SetCurrent(uint8_t value)
{
  #define KALMAN_COEFF        200U
  #define KALMAN_MULT         100U
  #define KALMAN_MAX_COEFF    255U

  /**< Simple Kalman filter for current values */
  static uint32_t curr_filter = 0;
  int32_t temp_value = (KALMAN_MAX_COEFF - KALMAN_COEFF);
  curr_filter = (curr_filter * KALMAN_COEFF + (uint32_t)value * (uint32_t)temp_value * KALMAN_MULT) / KALMAN_MAX_COEFF;
  curr = (uint8_t)((curr_filter + (KALMAN_MULT / (uint32_t)2)) / KALMAN_MULT);
  //curr = value;
  /**< Get maximal current */
  if (value > max_curr)
  {
    max_curr = value;
  }
}

/** \brief Get maximal current value
 *
 * \return Maximal current value
 *
 */
uint8_t MONITOR_GetMaxCurrent(void)
{
  uint8_t res = max_curr;
  /**< Reset maximal current value after readout */
  max_curr = curr;
  return res;
}

/** \brief Get converted voltage value
 *
 * \param [in] channel Number of the channel
 * \return Voltage value as uint8_t
 *
 */
uint8_t MONITOR_GetVoltage(uint8_t channel)
{
  switch (channel)
  {
    case MONITOR_U_CH1:
      return voltage1;
    case MONITOR_U_CH2:
      return voltage2;
    default:
      return 0;
  }
}

/** \brief Set converted voltage value
 *
 * \param [in] channel Number of the channel
 * \param value Voltage value to set
 * \return void
 *
 */
void MONITOR_SetVoltage(uint8_t channel, uint8_t value)
{
  switch (channel)
  {
    case MONITOR_U_CH1:
      voltage1 = value;
      if ((voltage1 < min_voltage) || (min_voltage == 0U))
      {
        min_voltage = voltage1;
      }
      break;
    case MONITOR_U_CH2:
      voltage2 = value;
      break;
    default:
      /**< Must not occur */
      break;
  }
}

/** \brief Get supply voltage status
 *
 * \return True if voltage is Ok
 *
 */
bool MONITOR_GetVoltageStatus(void)
{
  return ((voltage1 >= VOLTAGE_OK) && (voltage2 >= VOLTAGE_OK));
}

/** \brief
 *
 * \param void
 * \return bool
 *
 */
bool MONITOR_GetSupply(void)
{
  return ((voltage1 >= VOLTAGE_ZERO) || (voltage2 >= VOLTAGE_ZERO));
}

/** \brief Get converted temperature value
 *
 * \param [in] channel Number of the channel
 * \return Temperature value as uint8_t
 *
 */
uint8_t MONITOR_GetTemperature(uint8_t channel)
{
  switch (channel)
  {
    case MONITOR_TEMP_MOTOR:
      /**< No sensor available/sensor defective (temp < -40°C or temp > 150°C) */
      if (temperature1 < MONITOR_TEMP_MIN)
      {
        return MONITOR_TEMP_LOW;
      }
      if (temperature1 > MONITOR_TEMP_MAX)
      {
        return MONITOR_TEMP_HIGH;
      }
      return temperature1;
    case MONITOR_TEMP_PCB:
      /**< No sensor available/sensor defective (temp < -40°C or temp > 150°C) */
      if (temperature2 < MONITOR_TEMP_MIN)
      {
        return MONITOR_TEMP_LOW;
      }
      if (temperature2 > MONITOR_TEMP_MAX)
      {
        return MONITOR_TEMP_HIGH;
      }
      return temperature2;
    default:
      return 0;
  }
}

/** \brief
 *
 * \param [in] channel Number of the channel
 * \param [in] value Temperature value to set
 * \return void
 *
 */
void MONITOR_SetTemperature(uint8_t channel, uint8_t value)
{
  switch (channel)
  {
    case MONITOR_TEMP_MOTOR:
      temperature1 = value;
      break;
    case MONITOR_TEMP_PCB:
      temperature2 = value;
      break;
    default:
      /**< Must not occur */
      break;
  }
}

/** \brief Get temperature sensors status
 *
 * \param [in] channel Number of the channel
 * \return True if sensor is Ok
 *
 */
bool MONITOR_GetTemperatureStatus(uint8_t channel)
{
  switch (channel)
  {
    case MONITOR_TEMP_MOTOR:
      if ((temperature1 == MONITOR_TEMP_LOW) || (temperature1 == MONITOR_TEMP_HIGH))
      {
        return false;
      }
      break;
    case MONITOR_TEMP_PCB:
      if ((temperature2 == MONITOR_TEMP_LOW) || (temperature2 == MONITOR_TEMP_HIGH))
      {
        return false;
      }
      break;
    default:
      /**< Must not occur */
      break;
  }

  return true;
}

/** \brief RTOS task for monitoring non-critical parameters
 * (voltage, current, temperature)
 */
#ifndef DEF_UNITTEST
void MONITOR_Task(void *pParameters)
{
  (void) pParameters;
  static uint8_t param_id = MONITOR_PARAM_CURRENT;
  uint16_t value;
  bool do_supply_check = false;
  //uint8_t bldc_counter = 0;

  max_curr = 0;
  min_voltage = 0;

  /**< Set initial ADC channel */
  ANALOG_SetChannel(ADC_CHANNEL_CURRENT);
  /**< Initialize internal temperature sensor */
 	TEMP_Init();

  while (1)
  {
    /**< Pause for switching channels */
    vTaskDelay(pdMS_TO_TICKS(MONITOR_TASK_DELAY_MS));

    /**< Try to detect problem with power supply */
    if ((do_supply_check == true) && (MONITOR_GetSupply() == false))
    {
      POWER_RestartApp();
    }

    switch (param_id)
    {
      case MONITOR_PARAM_CURRENT:
        /**< Read current value */
        value = ANALOG_GetValue();
        MONITOR_SetCurrent(CONVERSION_CalcCurrent(value));
        ANALOG_SetChannel((uint8_t)ADC_CHANNEL_VOLTAGE1);
        break;
      case MONITOR_PARAM_VOLTAGE1:
        /**< Read voltage ch.1 value */
        value = ANALOG_GetValue();
        MONITOR_SetVoltage(MONITOR_U_CH1, CONVERSION_CalcVoltage(value));
        ANALOG_SetChannel(ADC_CHANNEL_VOLTAGE2);
        break;
      case MONITOR_PARAM_VOLTAGE2:
        /**< Read voltage ch.2 value */
        value = ANALOG_GetValue();
        MONITOR_SetVoltage(MONITOR_U_CH2, CONVERSION_CalcVoltage(value));
        ANALOG_SetChannel(ADC_CHANNEL_TEMP_MOTOR);
        break;
      case MONITOR_PARAM_TEMP_MOTOR:
        /**< Read motor temperature value */
        value = ANALOG_GetValue();
        MONITOR_SetTemperature(MONITOR_TEMP_MOTOR, CONVERSION_CalcTemperature(value));
        ANALOG_SetChannel(ADC_CHANNEL_CURRENT);
        break;
      case MONITOR_PARAM_TEMP_PCB:
        /**< Read MCU temperature value */
        MONITOR_SetTemperature(MONITOR_TEMP_PCB, (uint8_t)(TEMP_GetValue() + 50));
        break;
      #ifdef DEF_DUPLEX
      case MONITOR_PARAM_BLDC:
        /**< Commented out because of Hardware changed not implemented yet */
//        // Test BLDC if running
//        bldc_counter++;
//        if (bldc_counter > 40)
//        {
//          bldc_counter = 0;
//          if (HALL_IsReady() == false)
//          {
//            // Start new test cycle
//            HALL_Stop();
//            HALL_Start();
//          } else
//          {
//            // Test is ready, analyze results
//            HALL_Stop();
//            GLOBAL_Windings = HALL_DoAnalyze();
//            HALL_Start();
//          }
//        }
//        break;
      #endif
      default:
        break;
    }
    param_id++;
    if (param_id >= MONITOR_PARAM_LAST)
    {
      do_supply_check = true;
      param_id = MONITOR_PARAM_CURRENT;
    }
  }
}
#endif
