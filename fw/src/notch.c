#include <math.h>
#include "notch.h"

static notch_t x_1[NOTCH_NUM_LAST];
static notch_t x_2[NOTCH_NUM_LAST];
static notch_t y_1[NOTCH_NUM_LAST];
static notch_t y_2[NOTCH_NUM_LAST];
static notch_t fc1[NOTCH_NUM_LAST];
static notch_t fc2[NOTCH_NUM_LAST];
static notch_t fc3[NOTCH_NUM_LAST];
static notch_t fc4[NOTCH_NUM_LAST];
static notch_t fc5[NOTCH_NUM_LAST];
static bool NOTCH_On[NOTCH_NUM_LAST] = {false, false};

/** \brief Initialize notch filter coefficients
 *
 * \return void
 *
 */
static void NOTCH_Init(uint8_t num, float freq, float width, float attenuation)
{
  float zeta_n;
  float zeta_d;
  float wn;
  float t1;
  float t2;
  float t3;
  float t4;
  float t7;

  zeta_n = width;
  zeta_d = width * attenuation;
  wn = freq;
  t1 = wn * NOTCH_DT;
  t2 = t1 * t1;
  t3 = 4.0F * zeta_d * t1;
  t4 = 1.0F / (4.0F + t3 + t2);
  t7 = 4.0F * zeta_n * t1;

  /**< filter_coeff(1) = (4.0 + temp7 + temp2) * temp4 */
  fc1[num] = (notch_t)roundf((4.0F + t7 + t2) * t4 * (float)NOTCH_FACTOR);
  /**< filter_coeff(2) = 2.0 * (temp2 - 4.0) * temp4 */
  fc2[num] = (notch_t)roundf(2.0F * (t2 - 4.0F) * t4 * (float)NOTCH_FACTOR);
  /**< filter_coeff(3) = (4.0 - temp7 + temp2) * temp4 */
  fc3[num] = (notch_t)roundf((4.0F - t7 + t2) * t4 * (float)NOTCH_FACTOR);
  /**< filter_coeff(4) = -filter_coeff(2) */
  fc4[num] = -fc2[num];
  /**< filter_coeff(5) = -(4.0 - temp3 + temp2) * temp4 */
  fc5[num] = (notch_t)roundf(-(4.0F - t3 + t2) * t4 * (float)NOTCH_FACTOR);

  /**< Reset previous values */
  x_1[num] = 0;
  x_2[num] = 0;
  y_1[num] = 0;
  y_2[num] = 0;
}

void NOTCH_GetCoefficients(uint8_t num, notch_t *c1, notch_t *c2, notch_t *c3, notch_t *c4, notch_t *c5)
{
  if (num > NOTCH_NUM_2)
  {
    return;
  }

  *c1 = fc1[num];
  *c2 = fc2[num];
  *c3 = fc3[num];
  *c4 = fc4[num];
  *c5 = fc5[num];
}

/** \brief Recalculate filter coefficients
 *
 * \param [in] num Number of the notch filter
 * \param [in] freq RPM frequency in protocol units (/100)
 * \param [in] width Relative width of the filter in protocol units (/1000)
 * \param [in] attenuation Filter attenuation
 * \return void
 *
 */
void NOTCH_RecalcCoeff(uint8_t num, uint16_t freq, uint8_t width, uint8_t depth)
{
  static uint16_t curr_freq[NOTCH_NUM_LAST] = {0, 0};
  static uint8_t curr_width[NOTCH_NUM_LAST] = {0, 0};
  static uint8_t curr_depth[NOTCH_NUM_LAST] = {0, 0};

  if (num > NOTCH_NUM_2)
  {
    return;
  }

  if ((freq == 0U) || (depth == 0U))
  {
    /**< Disable notch filter and exit */
    NOTCH_On[num] = false;
    /**< To prevent wrong behavior after activating previous frequency again */
    curr_freq[num] = 0;
    return;
  }

  /**< Skip if values already applied */
  if ((curr_freq[num] == freq) && (curr_width[num] == width) && (curr_depth[num] == depth))
  {
    return;
  }

  if (depth > NOTCH_FILTER_MAX_DEPTH)
  {
    return;
  }

  NOTCH_On[num] = true;

  curr_freq[num] = freq;
  curr_width[num] = width;
  curr_depth[num] = depth;

  NOTCH_Init(num, (float)freq / 100, (float)width / 1000, depth);
}

/** \brief Calculate notch filter value
 *
 * Y(n) = f_coeff(1) * X(n) + f_coeff(2) * X(n-1) + f_coeff(3) * X(n-2) + f_coeff(4) * Y(n-1) + f_coeff(5) * Y(n-2)
 *
 * \param [in] num Number of the notch filter
 * \param [in] x Input value
 * \return Output value as int32_t
 *
 */
int32_t NOTCH_Calculate(uint8_t num, int32_t x)
{
  notch_t val;
  int32_t xval;

  if (num > NOTCH_NUM_2)
  {
    return x;
  }

  /**< No attenuation if notch filter is deactivated */
  if (NOTCH_On[num] == false)
  {
    return x;
  }

  xval = x * NOTCH_IN_MULT;

  /**< Calculate new value */
  val = ((fc1[num] * xval)  + (fc2[num] * x_1[num]) + (fc3[num] * x_2[num]) +
         (fc4[num] * y_1[num]) + (fc5[num] * y_2[num])) / NOTCH_FACTOR;

  /**< Update filter values */
  x_2[num] = x_1[num];
  x_1[num] = xval;
  y_2[num] = y_1[num];
  y_1[num] = val;

  return (int32_t)(val / NOTCH_IN_MULT);
}
