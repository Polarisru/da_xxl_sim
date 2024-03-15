#include "drivers.h"
#include "drv8320.h"

//#ifdef DEF_OLD_DA58S
//  #define DRV8320_BRAKE_PORT  GPIO_PORTA
//  #define DRV8320_DIR_PORT    GPIO_PORTB
//  #define DRV8320_BRAKE_PIN   8
//  #define DRV8320_DIR_PIN     3
//#endif

#define DRV8320_EN_PORT       GPIO_PORTA
#define DRV8320_BRAKE_PORT    GPIO_PORTA
#define DRV8320_DIR_PORT      GPIO_PORTA

#define DRV8320_EN_PIN        22
#define DRV8320_BRAKE_PIN     16
#define DRV8320_DIR_PIN       17

/** \brief Write value to register
 *
 * \param [in] reg Register number
 * \param [in] value Value to write
 * \return void
 *
 */
static void DRV8320_WriteReg(uint8_t reg, uint16_t value)
{
  uint16_t data;

  data = (uint16_t)(DRV8320_WRITE_BIT | DRV8320_REG_VALUE(reg) | (value & DRV8320_VALUE_MASK));
  SPI_Select(SPI_SELECT_DRIVER);
  SPI_Transmit(data, 16);
  SPI_Select(SPI_SELECT_NONE);
}

/** \brief Read value from register
 *
 * \param [in] reg Register number
 * \return Value as uint16_t
 *
 */
static uint16_t DRV8320_ReadReg(uint8_t reg)
{
  uint16_t data;

  data = (uint16_t)(DRV8320_READ_BIT | DRV8320_REG_VALUE(reg));
  SPI_Select(SPI_SELECT_DRIVER);
  SPI_Transmit(data, DRV8320_OUT_LENGTH);
  data = SPI_Receive(DRV8320_DATA_LENGTH);
  SPI_Select(SPI_SELECT_NONE);

  return (data & DRV8320_VALUE_MASK);
}

/** \brief Enable/disable DRV8320
 *
 * \param [in] on Enable if true
 * \return void
 *
 */
void DRV8320_Enable(bool on)
{
  if (on == true)
  {
    GPIO_SetPin(DRV8320_EN_PORT, DRV8320_EN_PIN);
  } else
  {
    GPIO_ClearPin(DRV8320_EN_PORT, DRV8320_EN_PIN);
  }
}

/** \brief Toggle Enable pin
 *
 * \return void
 *
 */
void DRV8320_EnableToggle(void)
{
  GPIO_TogglePin(DRV8320_EN_PORT, DRV8320_EN_PIN);
}

/** \brief Do motor brake, pin logic is inverted, low means do break
 *
 * \param [in] on Brake if true
 * \return void
 *
 */
void DRV8320_DoBrake(bool on)
{
  if (on == true)
  {
    GPIO_ClearPin(DRV8320_BRAKE_PORT, DRV8320_BRAKE_PIN);
  } else
  {
    GPIO_SetPin(DRV8320_BRAKE_PORT, DRV8320_BRAKE_PIN);
  }
}

/** \brief Set/reset free running mode (Hi-Z)
 *
 * \param [in] on Set free running mode if true
 * \return void
 *
 */
void DRV8320_DoFree(bool on)
{
  if (on == true)
  {
    DRV8320_WriteReg(DRV8320_REG_CTRL, DRV8320_CTRL_INIT_VALUE | DRV8320_CTRL_BIT_COAST);
    //GPIO_SetPin(LED_PORT, LED_PIN);
  } else
  {
    DRV8320_WriteReg(DRV8320_REG_CTRL, DRV8320_CTRL_INIT_VALUE);
    //GPIO_ClearPin(LED_PORT, LED_PIN);
  }
}

/** \brief Switch to normal rotating mode (1xPWM)
 *
 * \return void
 *
 */
void DRV8320_SwitchToNormal(void)
{
  DRV8320_WriteReg(DRV8320_REG_CTRL, DRV8320_CTRL_INIT_VALUE);
}

/** \brief Set motor direction
 *
 * \param [in] dir Direction to set
 * \return void
 *
 */
void DRV8320_SetDir(uint8_t dir)
{
  if (dir == MOTOR_DIR_CW)
  {
    GPIO_SetPin(DRV8320_DIR_PORT, DRV8320_DIR_PIN);
  } else
  {
    GPIO_ClearPin(DRV8320_DIR_PORT, DRV8320_DIR_PIN);
  }
}

/** \brief Read errors from DRV8320
 *
 * \return Error word as uint16_t
 *
 */
uint16_t DRV8320_ReadError(void)
{
  uint16_t err = 0;
  uint16_t data;

  /**< Read first fault register */
  data = DRV8320_ReadReg(DRV8320_REG_FAULT1);
  if ((data & DRV8320_FAULT1_BIT_UVLO) > 0U)
  {
    err = err | DRV8320_ERR_UNDERVOLT;
  }
  if ((data & DRV8320_FAULT1_BIT_OTSD) > 0U)
  {
    err = err | DRV8320_ERR_OVERTEMP;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_HA) > 0U)
  {
    err = err | DRV8320_ERR_OVC_HIGHA;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_LA) > 0U)
  {
    err = err | DRV8320_ERR_OVC_LOWA;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_HB) > 0U)
  {
    err = err | DRV8320_ERR_OVC_HIGHB;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_LB) > 0U)
  {
    err = err | DRV8320_ERR_OVC_LOWB;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_HC) > 0U)
  {
    err = err | DRV8320_ERR_OVC_HIGHC;
  }
  if ((data & DRV8320_FAULT1_BIT_VDS_LC) > 0U)
  {
    err = err | DRV8320_ERR_OVC_LOWC;
  }
  /**< Read second fault register with additional info */
  data = DRV8320_ReadReg(DRV8320_REG_FAULT2);
  if ((data & DRV8320_FAULT2_BIT_VGS_HA) > 0U)
  {
    err = err | DRV8320_ERR_GATE_HIGHA;
  }
  if ((data & DRV8320_FAULT2_BIT_VGS_LA) > 0U)
  {
    err = err | DRV8320_ERR_GATE_LOWA;
  }
  if ((data & DRV8320_FAULT2_BIT_VGS_HB) > 0U)
  {
    err = err | DRV8320_ERR_GATE_HIGHB;
  }
  if ((data & DRV8320_FAULT2_BIT_VGS_LB) > 0U)
  {
    err = err | DRV8320_ERR_GATE_LOWB;
  }
  if ((data & DRV8320_FAULT2_BIT_VGS_HC) > 0U)
  {
    err = err | DRV8320_ERR_GATE_HIGHC;
  }
  if ((data & DRV8320_FAULT2_BIT_VGS_LC) > 0U)
  {
    err = err | DRV8320_ERR_GATE_LOWC;
  }

  return err;
}

/** \brief Activate 1xPWM mode
 *
 * \return void
 *
 */
void DRV8320_Activate(void)
{
  /**< Activate 1x PWM mode and monitoring all faults */
  DRV8320_WriteReg(DRV8320_REG_CTRL, DRV8320_CTRL_INIT_VALUE);
  /**< Setup gate currents */
  DRV8320_WriteReg(DRV8320_REG_DRV_HS, DRV8320_HS_INIT_VALUE);
  DRV8320_WriteReg(DRV8320_REG_DRV_LS, DRV8320_LS_INIT_VALUE);
}

/** \brief Initialize DRV8320 module
 *
 * \return void
 *
 */
void DRV8320_Configuration(void)
{
  /**< Initialize controlling pins */
  GPIO_ClearPin(DRV8320_EN_PORT, DRV8320_EN_PIN);
  GPIO_SetPin(DRV8320_BRAKE_PORT, DRV8320_BRAKE_PIN);
  GPIO_ClearPin(DRV8320_DIR_PORT, DRV8320_DIR_PIN);
  GPIO_SetDir(DRV8320_EN_PORT, DRV8320_EN_PIN, true);
  GPIO_SetDir(DRV8320_BRAKE_PORT, DRV8320_BRAKE_PIN, true);
  GPIO_SetDir(DRV8320_DIR_PORT, DRV8320_DIR_PIN, true);
  GPIO_SetFunction(DRV8320_EN_PORT, DRV8320_EN_PIN, GPIO_PIN_FUNC_OFF);
  GPIO_SetFunction(DRV8320_BRAKE_PORT, DRV8320_BRAKE_PIN, GPIO_PIN_FUNC_OFF);
  GPIO_SetFunction(DRV8320_DIR_PORT, DRV8320_DIR_PIN, GPIO_PIN_FUNC_OFF);

  /**< configure pin for blinking LED */
//  GPIO_ClearPin(LED_PORT, LED_PIN);
//  GPIO_SetDir(LED_PORT, LED_PIN, true);
//  GPIO_SetFunction(LED_PORT, LED_PIN, GPIO_PIN_FUNC_OFF);
}
