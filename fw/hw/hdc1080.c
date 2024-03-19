#include "drivers.h"
#include "hdc1080.h"

#ifndef DEF_UNITTEST
  #define I2C_PORT          GPIO_PORTA

  #define I2C_SDA_PIN       16
  #define I2C_SCL_PIN       17

  #define I2C_SDA_MUX       MUX_PA16C_SERCOM1_PAD0
  #define I2C_SCL_MUX       MUX_PA17C_SERCOM1_PAD1
#endif

// Example of usage:
//HDC_Init(SERCOM0);
//for (uint8_t i = 0; i < 10; i++)
//{
//  vTaskDelay(pdMS_TO_TICKS(500));
//  temp = HDC_ReadTemperature();
//}

#ifndef DEF_UNITTEST
  Sercom *HDC1080_Channel;
#else
  uint8_t HDC1080_Channel;
#endif

/** \brief Initialize HDC1980 module
 *
 * \param
 * \return void
 *
 */
#ifndef DEF_UNITTEST
void HDC1080_Configuration(Sercom *channel)
{
  HDC1080_Channel = channel;
  /**< Initialize pins */
	GPIO_SetFunction(I2C_PORT, I2C_SDA_PIN, I2C_SDA_MUX);
	GPIO_SetFunction(I2C_PORT, I2C_SCL_PIN, I2C_SCL_MUX);

  I2C_Init(channel, HDC1080_BAUDRATE);
}
#endif

/** \brief Read temperature value
 *
 * \return Temperature in C° as float
 *
 */
float HDC1080_ReadTemperature(void)
{
  uint8_t data[2];
  uint16_t value;
  float temp;

  if (I2C_ReadReg(HDC1080_Channel, HDC1080_ADDRESS, HDC1080_REG_TEMPERATURE, data, 2) == false)
    return 0;

  value = ((uint16_t)data[0] << 8) + data[1];
  /**< See datasheet, page 14 */
  temp = (float)value * 165.0f / 0x10000 - 40;

  return temp;
}

/** \brief Read relative humidity
 *
 * \return Relative humidity in % as float
 *
 */
float HDC1080_ReadHumidity(void)
{
  uint8_t data[2];
  uint16_t value;

  if (I2C_ReadReg(HDC1080_Channel, HDC1080_ADDRESS, HDC1080_REG_HUMIDITY, data, 2) == false)
    return 0;

  value = ((uint16_t)data[0] << 8) + data[1];
  /**< See datasheet, page 14 */
  return ((float)value / 0x10000 * 100);
}
