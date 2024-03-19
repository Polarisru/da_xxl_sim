#include "drivers.h"
#include "hall.h"
#include "motor.h"
#include "rtos.h"

#ifndef DEF_UNITTEST
  #define HALL_PORT           GPIO_PORTA
  #define HALL_A_PIN          20
  #define HALL_B_PIN          19
  #define HALL_C_PIN          18
  #define HALL_A_MUX          MUX_PA20A_EIC_EXTINT4
  #define HALL_B_MUX          MUX_PA19A_EIC_EXTINT3
  #define HALL_C_MUX          MUX_PA18A_EIC_EXTINT2
  #define HALL_A_EIC          4U
  #define HALL_B_EIC          3U
  #define HALL_C_EIC          2U
  #define HALL_CONF0_VALUE    /*EIC_CONFIG_FILTEN4 |*/ EIC_CONFIG_SENSE4_BOTH |\
                              /*EIC_CONFIG_FILTEN3 |*/ EIC_CONFIG_SENSE3_BOTH |\
                              /*EIC_CONFIG_FILTEN2 |*/ EIC_CONFIG_SENSE2_BOTH
  #define HALL_CONF1_VALUE    0U

  #define HALL_CURR_PORT      GPIO_PORTA
  #define HALL_CURR_PIN       1
#endif

static uint8_t HALL_CommArray[HALL_COMM_LEN];
static uint8_t HALL_CurrentArray[HALL_COMM_LEN + 1U];
static volatile uint8_t HALL_CommCounter;
static volatile bool HALL_CommReady;
static int8_t windings[3] = {0, 0, 0};
static uint8_t HALL_MotorDir;

/** \brief Read values from Hall sensors
 *
 * \return Hall sensors value (00000CBA)
 *
 */
#ifndef DEF_UNITTEST
uint8_t HALL_ReadValues(void)
{
  uint32_t inp;
  uint8_t value = 0;

  inp = PORT->Group[HALL_PORT].IN.reg;
  if ((inp & (1UL << HALL_A_PIN)) > 0U)
  {
    value += HALL_CHECK_BIT_A;
  }
  if ((inp & (1UL << HALL_B_PIN)) > 0U)
  {
    value += HALL_CHECK_BIT_B;
  }
  if ((inp & (1UL << HALL_C_PIN)) > 0U)
  {
    value += HALL_CHECK_BIT_C;
  }

  return value;
}

/**< Interrupt handler for external interrupts */
void EIC_Handler(void)
{
  EIC->INTFLAG.reg = (1UL << HALL_A_EIC) | (1UL << HALL_B_EIC) | (1UL << HALL_C_EIC);
  /**< Was commented out for the old Hardware configuration without current measurement */
  //if (GLOBAL_Power == EE_PWM_Max)
  if (1)
  {
    HALL_CommArray[HALL_CommCounter] = HALL_ReadValues();
    if ((PORT->Group[HALL_CURR_PORT].IN.reg & (1UL << HALL_CURR_PIN)) > 0U)
    {
      HALL_CurrentArray[HALL_CommArray[HALL_CommCounter] - 1U]++;
    }
    HALL_CommCounter++;
    if (HALL_CommCounter >= HALL_COMM_LEN)
    {
      HALL_CommCounter = 0;
      HALL_CommReady = true;
      HALL_MotorDir = MOTOR_GetDir();
    }
  }
}

/** \brief Initialize Hall sensors (input only)
 *
 * \return void
 *
 */
void HALL_Configuration(void)
{
  /**< Setup power and clock for pin interrupt as event */
  GCLK->PCHCTRL[EIC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_EIC;

  /**< Configure Hall sensors as inputs with external interrupts */
  GPIO_SetPullMode(HALL_PORT, HALL_A_PIN, GPIO_PULL_UP);
  GPIO_SetPullMode(HALL_PORT, HALL_B_PIN, GPIO_PULL_UP);
  GPIO_SetPullMode(HALL_PORT, HALL_C_PIN, GPIO_PULL_UP);
  GPIO_SetFunction(HALL_PORT, HALL_A_PIN, HALL_A_MUX);
  GPIO_SetFunction(HALL_PORT, HALL_B_PIN, HALL_B_MUX);
  GPIO_SetFunction(HALL_PORT, HALL_C_PIN, HALL_C_MUX);

  /**< Configure current-comparator pin */
  GPIO_ClearPin(HALL_CURR_PORT, HALL_CURR_PIN);
  GPIO_SetDir(HALL_CURR_PORT, HALL_CURR_PIN, false);
  GPIO_SetFunction(HALL_CURR_PORT, HALL_CURR_PIN, GPIO_PIN_FUNC_OFF);
  GPIO_SetPullMode(HALL_CURR_PORT, HALL_CURR_PIN, GPIO_PULL_UP);

  /**< Configure external interrupts */
  /**< Disable EIC module to apply new settings, some registers are enabled-protected */
  if (EIC->CTRLA.bit.ENABLE == 0)
  {
    EIC->CTRLA.reg = EIC_CTRLA_SWRST;
    while ((EIC->SYNCBUSY.reg & EIC_SYNCBUSY_SWRST) > 0U) {}
  } else
  {
    EIC->CTRLA.bit.ENABLE = 0;
    while ((EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) > 0U) {}
  }
  EIC->INTENSET.reg |= (1UL << HALL_A_EIC) | (1UL << HALL_B_EIC) | (1UL << HALL_C_EIC);
  EIC->ASYNCH.reg |= (1UL << HALL_A_EIC) | (1UL << HALL_B_EIC) | (1UL << HALL_C_EIC);
  EIC->DEBOUNCEN.reg |= (1UL << HALL_A_EIC) | (1UL << HALL_B_EIC) | (1UL << HALL_C_EIC);
  EIC->DPRESCALER.reg |= 0U;
  /**< Setup external interrupts sensing */
  EIC->CONFIG[0].reg |= HALL_CONF0_VALUE;
  EIC->CONFIG[1].reg |= HALL_CONF1_VALUE;
  EIC->CTRLA.reg |= EIC_CTRLA_ENABLE;
  while ((EIC->SYNCBUSY.reg & EIC_SYNCBUSY_ENABLE) > 0U) {}

  /**< Disable EIC interrupts */
  NVIC_DisableIRQ(EIC_IRQn);
  NVIC_SetPriority(DMAC_IRQn, 7);
}

/** \brief Start readout of Hall sensors
 *
 * \return void
 *
 */
void HALL_Start(void)
{
  uint8_t i;

  HALL_CommCounter = 0;
  HALL_CommReady = false;
  HALL_MotorDir = MOTOR_DIR_CW;
  for (i = 0; i < HALL_COMM_LEN; i++)
  {
    HALL_CurrentArray[i] = 0;
  }
  /**< Start interrupts */
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_EnableIRQ(EIC_IRQn);
}

/** \brief Stop readout of Hall sensors
 *
 * \return void
 *
 */
void HALL_Stop(void)
{
  /**< Disable EIC interrupts */
  NVIC_DisableIRQ(EIC_IRQn);
}

/** \brief Check if Hall's test is ready
 *
 * \return True if check is ready
 *
 */
bool HALL_IsReady(void)
{
  return HALL_CommReady;
}

/** \brief Perform checking of Hall sensors
 *
 * \return Error code as uint8_t, 0 - everything is Ok
 *
 */
uint8_t HALL_DoCheck(void)
{
  uint8_t i = 0;

  for (i = 0; i < HALL_COMM_LEN; i++)
  {
    HALL_CommArray[i] = 0;
    HALL_CurrentArray[i] = 0;
  }

  HALL_CommCounter = 0;
  HALL_CommReady = false;
  /**< Start interrupts */
  NVIC_ClearPendingIRQ(EIC_IRQn);
  NVIC_EnableIRQ(EIC_IRQn);
  i = 0;
  while ((HALL_CommReady == false) && (i < 25U))
  {
    i++;
    vTaskDelay(pdMS_TO_TICKS(HALL_CHECK_DELAY_MS));
  }
  /**< Disable EIC interrupts */
  NVIC_DisableIRQ(EIC_IRQn);

  return 0;
}
#endif

/** \brief Check Hall sensors
 *
 * \param [in] data Data array with Hall sensors values
 * \return uint8_t
 *
 */
uint8_t HALL_CheckSensors(const uint8_t *data)
{
  uint8_t i;
  uint8_t res = 0;
  uint8_t m_count = 0xFFU;
  uint8_t p_count = 0x00U;

  /**< Analyze Halls */
  for (i = 0; i < HALL_COMM_LEN; i++)
  {
    m_count &= data[i];
    p_count |= data[i];
  }

  /**< Process Hall sensors */
  i = m_count | (uint8_t)(~(m_count | p_count));
  if ((i & HALL_CHECK_BIT_A) > 0U)
  {
    res |= HALL_CHECK_HALL_A;
  }
  if ((i & HALL_CHECK_BIT_B) > 0U)
  {
    res |= HALL_CHECK_HALL_B;
  }
  if ((i & HALL_CHECK_BIT_C) > 0U)
  {
    res |= HALL_CHECK_HALL_C;
  }

  return res;
}

/** \brief Check BLDC windings
 *
 * \param [in] data Data array with measured currents
 * \return uint8_t
 *
 */
uint8_t HALL_CheckWindings(const uint8_t *data, uint8_t dir)
{
  uint8_t i;
  uint8_t res = 0;
  int8_t step;

  for (i = 0; i < 3U; i++)
  {
    windings[i] = 0;
  }

  /**< Analyze windings */
  for (i = 0; i < HALL_COMM_LEN; i++)
  {
    if (data[i] == 0U)
    {
      step = -1;
    } else
    {
      step = 1;
    }
    switch (i + 1U)
    {
      case HALL_CHECK_BIT_A:
      case (HALL_CHECK_BIT_B | HALL_CHECK_BIT_C):
        if (dir == MOTOR_DIR_CCW)
        {
          windings[0] += step;
        } else
        {
          windings[2] += step;
        }
        windings[1] += step;
        break;
      case HALL_CHECK_BIT_B:
      case (HALL_CHECK_BIT_A | HALL_CHECK_BIT_C):
        if (dir == MOTOR_DIR_CCW)
        {
          windings[1] += step;
        } else
        {
          windings[0] += step;
        }
        windings[2] += step;
        break;
      case HALL_CHECK_BIT_C:
      case (HALL_CHECK_BIT_A | HALL_CHECK_BIT_B):
        if (dir == MOTOR_DIR_CCW)
        {
          windings[2] += step;
        } else
        {
          windings[1] += step;
        }
        windings[0] += step;
        break;
      default:
        /**< Must not occur */
        break;
    }
  }

  /**< Process windings */
  if (windings[0] < 0)
  {
    res |= HALL_CHECK_WIND_A;
  }
  if (windings[1] < 0)
  {
    res |= HALL_CHECK_WIND_B;
  }
  if (windings[2] < 0)
  {
    res |= HALL_CHECK_WIND_C;
  }

  return res;
}

/** \brief Analyze BLDC test results
 *
 * \return Test result as uint8_t, (0.0.WA.WB.WC.HA.HB.HC)
 *
 */
uint8_t HALL_DoAnalyze(void)
{
  if (HALL_CommReady == false)
  {
    /**< Rotation was not complete */
    return (HALL_CHECK_HALL_A | HALL_CHECK_HALL_B | HALL_CHECK_HALL_C);
  }

  return (HALL_CheckSensors(HALL_CommArray) | HALL_CheckWindings(HALL_CurrentArray, HALL_MotorDir));
}
