#include "driver_spi.h"
#include "driver_gpio.h"

#ifdef DEF_OLD_DA58
  #define SPI_PIN_MISO    23
#else
  #define SPI_PIN_MISO    28
#endif
#define SPI_PIN_CLK     24
#define SPI_PIN_MOSI    25
#define SPI_PIN_CS_D    23
#define SPI_PIN_CS_M    27

#define SPI_PORT        GPIO_PORTA
#ifdef DEF_OLD_DA58
  #define SPI_MISO_PORT   GPIO_PORTB
#else
  #define SPI_MISO_PORT   GPIO_PORTA
#endif

/**< Define short delay for SPI clock */
#define SPI_DELAY       {__NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP(); __NOP();}

/** \brief Initialize SPI bus (only for EEPROM)
 *
 * \return void
 *
 */
void SPI_Init(void)
{
  /**< Initialize pins (software SPI) */
  /**< MISO */
	GPIO_SetDir(SPI_MISO_PORT, SPI_PIN_MISO, false);
	GPIO_SetPullMode(SPI_MISO_PORT, SPI_PIN_MISO, GPIO_PULL_UP);
	GPIO_SetFunction(SPI_MISO_PORT, SPI_PIN_MISO, GPIO_PIN_FUNC_OFF);
  /**< MOSI */
	GPIO_ClearPin(SPI_PORT, SPI_PIN_MOSI);
	GPIO_SetDir(SPI_PORT, SPI_PIN_MOSI, true);
	GPIO_SetFunction(SPI_PORT, SPI_PIN_MOSI, GPIO_PIN_FUNC_OFF);
  /**< CLK */
	GPIO_ClearPin(SPI_PORT, SPI_PIN_CLK);
	GPIO_SetDir(SPI_PORT, SPI_PIN_CLK, true);
	GPIO_SetFunction(SPI_PORT, SPI_PIN_CLK, GPIO_PIN_FUNC_OFF);
  /**< CS pins */
  GPIO_SetPin(SPI_PORT, SPI_PIN_CS_M);
  GPIO_SetPin(SPI_PORT, SPI_PIN_CS_D);
  GPIO_SetDir(SPI_PORT, SPI_PIN_CS_M, true);
  GPIO_SetDir(SPI_PORT, SPI_PIN_CS_D, true);
  GPIO_SetFunction(SPI_PORT, SPI_PIN_CS_M, GPIO_PIN_FUNC_OFF);
  GPIO_SetFunction(SPI_PORT, SPI_PIN_CS_D, GPIO_PIN_FUNC_OFF);
}

/** \brief Select device on SPI bus
 *
 * \param [in] device Device id to select for communication
 * \return void
 *
 */
void SPI_Select(uint8_t device)
{
  /**< Set CLK low for the start bit */
  PORT_IOBUS->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_CLK);
  switch (device)
  {
    case SPI_SELECT_MAGNET:
      PORT_IOBUS->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_CS_M);
      break;
    case SPI_SELECT_DRIVER:
      PORT_IOBUS->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_CS_D);
      break;
    case SPI_SELECT_NONE:
    default:
      /**< Deselect all devices */
      PORT_IOBUS->Group[SPI_PORT].OUTSET.reg = (1UL << SPI_PIN_CS_D);
      PORT_IOBUS->Group[SPI_PORT].OUTSET.reg = (1UL << SPI_PIN_CS_M);
      break;
  }
}

/** \brief Transmit data to SPI bus
 *
 * \param [in] data Data word to transmit
 * \param [in] bits Number of bits to transmit
 * \return void
 *
 */
void SPI_Transmit(uint16_t data, uint8_t bits)
{
  uint8_t i;
  uint16_t comp;

  while (bits-- > 0U)
  {
    i = bits;
    comp = (uint16_t)(1UL << i);
    if ((comp & data) > 0U)
    {
      PORT->Group[SPI_PORT].OUTSET.reg = (1UL << SPI_PIN_MOSI);
    } else
    {
      PORT->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_MOSI);
    }
    PORT->Group[SPI_PORT].OUTSET.reg = (1UL << SPI_PIN_CLK);
    SPI_DELAY;
    PORT->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_CLK);
  }
  PORT->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_MOSI);
}

/** \brief Receive data from SPI bus
 *
 * \param [in] bits Number of bits to receive
 * \return Data value as uint16_t
 *
 */
uint16_t SPI_Receive(uint8_t bits)
{
  uint8_t i;
  uint16_t dout = 0;

  while (bits-- > 0U)
  {
    i = bits;
    PORT->Group[SPI_PORT].OUTSET.reg = (1UL << SPI_PIN_CLK);
    SPI_DELAY;
    PORT->Group[SPI_PORT].OUTCLR.reg = (1UL << SPI_PIN_CLK);
    SPI_DELAY;
    if ((PORT->Group[SPI_MISO_PORT].IN.reg & (1UL << SPI_PIN_MISO)) > 0U)
    {
      dout |= (uint16_t)(1UL << i);
    }
  }

  return dout;
}
